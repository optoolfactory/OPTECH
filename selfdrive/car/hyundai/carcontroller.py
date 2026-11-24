from cereal import car
from common.conversions import Conversions as CV
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_driver_steer_torque_limits, create_gas_interceptor_command
from selfdrive.car.hyundai import hyundaicanfd, hyundaican
from selfdrive.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CANFD_CAR, CAR, FEATURES, LEGACY_SAFETY_MODE_CAR
import random
from random import randint
from common.params import Params

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2

# KIA SOUL EV (SK3), carrot e5cf102 (measured in that car): its MDPS starts taking our torque (MDPS12 ToiActive 0 -> 1)
# only when the requested torque is near 0 at its check - every start seen was at |torque| <= 9, a steady 30..409 was
# refused for good. So while it does not take our torque (ignored anyway), send 0 with the request bit still on: it
# starts at its next check and our torque then ramps up from 0. Carrot also found: do NOT toggle the request bit to
# coax it (faults the MDPS or turns short slips into full drops); keep the request steady.
SK3_NOACT_ZERO_FRAMES = 10    # MDPS not taking torque this long (100 Hz, small debounce) -> send 0
SK3_NOACT_WARN_FRAMES = 100   # ... and this long while moving -> cluster 'keep hands on wheel'


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  # optech (K7 HEV HUD feedback, like OPKR which always sends both lanes visible): while openpilot is on show both
  # lane lines all the time, so the HUD lines just mean 'on'. The model's lane visibility flips around 50 % on faded
  # lines / intersections and made the lines come and go every second (drivelog still records the real values)
  if enabled:
    sys_state = 3
  elif hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6
  # optech (K7 HEV HUD feedback): while openpilot is off show no lane lines, not lines that come and go with the model's
  # lane visibility (lane departure warnings still show)
  if not enabled and not sys_warning:
    sys_state = 1

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.params = CarControllerParams(CP)
    self.ldws_car = Params().get_bool("OptechLdwsCar")  # 토글 > LDWS 차량 (read at start: reboot to apply)
    self.packer = CANPacker(dbc_name)
    self.angle_limit_counter = 0
    self.frame = 0
    self.sk3 = CP.carFingerprint == CAR.KIA_SOUL_EV_SK3
    self.mdps_noact_frames = 0  # SK3: frames the MDPS has not taken our torque while steering is on
    self.lkas_noact_frames = 0  # SK3: the same while moving (cluster warning)

    self.accel_last = 0
    self.apply_steer_last = 0
    self.non_scc_speed_buttons = False
    # comma pedal (Ray EV / Porter II EV without SCC)
    self.pedal = bool(CP.flags & HyundaiFlags.NON_SCC_PEDAL)
    self.pedal_packer = None
    self.pedal_gas_last = 0.
    self.pedal_max = 0.7
    if self.pedal:
      from selfdrive.car.hyundai.pedal import command_dbc
      self.pedal_packer = CANPacker(command_dbc(CP))  # the Ray EV's strong scaling has its own command DBC
    self.car_fingerprint = CP.carFingerprint
    #self.send_lfa_mfa_lkas = True if self.car_fingerprint in FEATURES["send_lfa_mfa"] and self.car_fingerprint not in [CAR.HYUNDAI_GENESIS] else False
    self.send_lfa_mfa_lkas = CP.flags & HyundaiFlags.SEND_LFA.value
    self.last_button_frame = 0
    self.pcmCruiseButtonDelay = 0
    self.jerkStartLimit = 1.0
    self.speedCameraHapticEndFrame = 0
    self.hapticFeedbackWhenSpeedCamera = 0
    self.maxAngleFrames = MAX_ANGLE_FRAMES
    self.softHoldMode = 1
    self.blinking_signal = False #아이콘 깜박이용 1Hz
    self.blinking_frame = int(1.0 / DT_CTRL)
    self.steerDeltaUp = 3
    self.steerDeltaDown = 7
    self.button_wait = 12
    self.jerk_count = 0

    # for Legacy mode car auto resume (DH etc, not tested)
    self.resume_cnt = 0
    self.resume_wait_timer = 0
    self.button_alive = 0
    self.button_alive_frame = 0
    # optech drive record: cruise buttons openpilot sent to the car (CLU11), counted since start -
    # RES while the car's SCC is off / RES while on / SET- / GAP / CANCEL (drivelog reads the change)
    self.btn_counts = [0, 0, 0, 0, 0]

  def drive_record_text(self, CS, can_sends):
    """carControl.debugTextCC for selfdrive/carrot/tools/drivelog.py (nothing else shows it)."""
    for msg in can_sends:
      if msg[0] == 1265:  # CLU11: CF_Clu_CruiseSwState is the low 3 bits of byte 0
        btn = msg[2][0] & 0x7
        if btn == Buttons.RES_ACCEL:
          self.btn_counts[1 if CS.out.cruiseState.enabled else 0] += 1
        elif btn == Buttons.SET_DECEL:
          self.btn_counts[2] += 1
        elif btn == Buttons.GAP_DIST:
          self.btn_counts[3] += 1
        elif btn == Buttons.CANCEL:
          self.btn_counts[4] += 1
    scc = CS.scc_diag if CS.scc_diag is not None else (-1, -1, -1, -1)
    cam = CS.cam_lkas_diag if CS.cam_lkas_diag is not None else (-1, -1)
    fca = CS.fca_diag if CS.fca_diag is not None else (-1, -1)
    fcw = CS.cam_fcw_diag if CS.cam_fcw_diag is not None else (-1, -1)
    return "OPT i={} a={} f={} m={} cl={} cw={} r0={} r1={} s={} g={} c={} fi={} fs={} cf={} cu={}".format(
      *scc, *cam, *self.btn_counts, *fca, *fcw)

  def update(self, CC, CS):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    self.params.STEER_DELTA_UP = self.steerDeltaUp
    self.params.STEER_DELTA_DOWN = self.steerDeltaDown
    apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    if not CC.latActive:
      apply_steer = 0

    # KIA SOUL EV (SK3): MDPS not taking our torque -> request 0 so it starts, then ramp up from 0 (see SK3_NOACT_*)
    mdps_noact = self.sk3 and CC.latActive and int(round(getattr(CS, "steer_state", 1))) == 0
    self.mdps_noact_frames = self.mdps_noact_frames + 1 if mdps_noact else 0
    self.lkas_noact_frames = self.lkas_noact_frames + 1 if mdps_noact and CS.out.vEgoRaw >= 2.0 else 0
    if self.mdps_noact_frames >= SK3_NOACT_ZERO_FRAMES:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    # accel + longitudinal
    accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    if CC.activeHda == 2 and self.speedCameraHapticEndFrame < 0: # 과속카메라 감속시작
      self.speedCameraHapticEndFrame = self.frame + (8.0 / DT_CTRL)  #6초간 켜줌..
    elif CC.activeHda != 2:
      self.speedCameraHapticEndFrame = -1

    if self.frame < self.speedCameraHapticEndFrame and self.hapticFeedbackWhenSpeedCamera>0:
      haptic_stop = (self.speedCameraHapticEndFrame - (5.0/DT_CTRL)) < self.frame < (self.speedCameraHapticEndFrame - (3.0/DT_CTRL))
      if not haptic_stop:
         left_lane_warning = right_lane_warning = self.hapticFeedbackWhenSpeedCamera 
     
    if self.frame % self.blinking_frame == 0:
      self.blinking_signal = True
    elif self.frame % self.blinking_frame == self.blinking_frame / 2:
      self.blinking_signal = False

    jerk = actuators.jerk
    #jerk = accel - self.accel_last
    can_sends = []

    # *** common hyundai stuff ***
    if self.frame % 100 == 0:
      self.jerkStartLimit = float(int(Params().get("JerkStartLimit", encoding="utf8"))) * 0.1
      self.hapticFeedbackWhenSpeedCamera = int(Params().get("HapticFeedbackWhenSpeedCamera", encoding="utf8"))
      self.maxAngleFrames = int(Params().get("MaxAngleFrames", encoding="utf8"))
      self.softHoldMode = int(Params().get("SoftHoldMode", encoding="utf8"))
      self.steerDeltaUp = int(Params().get("SteerDeltaUp", encoding="utf8"))
      self.steerDeltaDown = int(Params().get("SteerDeltaDown", encoding="utf8"))
      from selfdrive.car.hyundai.pedal import PEDAL_PARAMS
      if self.CP.flags & HyundaiFlags.NON_SCC and self.CP.carFingerprint in PEDAL_PARAMS:  # Ray EV / Porter II EV menu
        _, max_param, buttons_param = PEDAL_PARAMS[self.CP.carFingerprint]
        self.non_scc_speed_buttons = Params().get_bool(buttons_param)
        try:
          self.pedal_max = min(max(int(Params().get(max_param, encoding="utf8") or 70), 10), 70) / 100.
        except ValueError:
          self.pedal_max = 0.5

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and self.CP.openpilotLongitudinalControl:
      addr, bus = 0x7d0, 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, 5
      can_sends.append([addr, 0, b"\x02\x3E\x80\x00\x00\x00\x00\x00", bus])

    # >90 degree steering fault prevention
    # Count up to MAX_ANGLE_FRAMES, at which point we need to cut torque to avoid a steering fault
    if CC.latActive and abs(CS.out.steeringAngleDeg) >= MAX_ANGLE:
      self.angle_limit_counter += 1
    else:
      self.angle_limit_counter = 0

    # Cut steer actuation bit for two frames and hold torque with induced temporary fault
    torque_fault = CC.latActive and self.angle_limit_counter > self.maxAngleFrames
    lat_active = CC.latActive and not torque_fault

    if self.angle_limit_counter >= self.maxAngleFrames + MAX_ANGLE_CONSECUTIVE_FRAMES:
      self.angle_limit_counter = 0

    if self.sk3:
      # KIA SOUL EV (SK3), carrot e5cf102: no 2-frame request cut above 85 degrees. This MDPS drops steering at each
      # cut and has to start again, so it let go over and over through every sharp turn; the >90 degree EPS fault the
      # cut prevents was never seen on this car (it held steering at 89 degrees under high torque).
      torque_fault = False
      lat_active = CC.latActive

    # CAN-FD platforms
    if self.CP.carFingerprint in CANFD_CAR:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2
      hda2_long = hda2 and self.CP.openpilotLongitudinalControl

      # steering control
      can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, CC.enabled, lat_active, apply_steer))

      # disable LFA on HDA2
      if self.frame % 5 == 0 and hda2:
        can_sends.append(hyundaicanfd.create_cam_0x2a4(self.packer, CS.cam_0x2a4))

      # LFA and HDA icons
      if self.frame % 5 == 0 and (not hda2 or hda2_long):
        can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CP, CC.enabled))

      if self.CP.openpilotLongitudinalControl:
        if hda2:
          can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.frame))
        if self.frame % 2 == 0:
          can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CP, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                           set_speed_in_units))
          self.accel_last = accel
      else:
        # button presses
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
          # cruise cancel
          if CC.cruiseControl.cancel:
            if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
              can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, CS.cruise_info))
              self.last_button_frame = self.frame
            else:
              for _ in range(20):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, CS.buttons_counter+1, Buttons.CANCEL))
              self.last_button_frame = self.frame

          # cruise standstill resume
          elif CC.cruiseControl.resume:
            if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
              # TODO: resume for alt button cars
              pass
            else:
              for _ in range(20):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, CS.buttons_counter+1, Buttons.RES_ACCEL))
              self.last_button_frame = self.frame
    else:
      can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.car_fingerprint, self.send_lfa_mfa_lkas, apply_steer, lat_active,
                                                torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                hud_control.leftLaneVisible or CC.enabled, hud_control.rightLaneVisible or CC.enabled,
                                                left_lane_warning, right_lane_warning, ldws_car=self.ldws_car,
                                                noact_warning=self.lkas_noact_frames >= SK3_NOACT_WARN_FRAMES))

      if not self.CP.openpilotLongitudinalControl:
        if CC.cruiseControl.cancel:
          can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP.carFingerprint))
        elif CC.cruiseControl.resume:
          if self.CP.carFingerprint in LEGACY_SAFETY_MODE_CAR:            
            if self.resume_wait_timer > 0:
              self.resume_wait_timer -= 1
            else:
              can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint))
              self.resume_cnt += 1
              if self.resume_cnt >= int(randint(4, 5) * 2):
                self.resume_cnt = 0
                self.resume_wait_timer = int(randint(20, 25) * 2)
              
          else:
            # send resume at a max freq of 10Hz
            if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
              # send 25 messages at a time to increases the likelihood of resume being accepted
              #can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint)] * 25)
              can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint))
              self.last_button_frame = self.frame
        else:
          self.resume_wait_timer = 0
          self.resume_cnt = 0
          target = int(set_speed_in_units+0.5)
          current = int(CS.out.cruiseState.speed*CV.MS_TO_KPH + 0.5)

          #CC.debugTextCC = "BTN:00,T:{:.1f},C:{:.1f},{},{}".format(target, current, self.wait_timer, self.alive_timer)
          if self.CP.flags & HyundaiFlags.NON_SCC and not self.non_scc_speed_buttons:
            pass  # cruise-control-only car: the driver sets the speed unless '크루즈 속도 자동 조절' is on
          elif (self.frame - self.last_button_frame) > self.button_wait:
            if (self.frame - self.button_alive_frame) > self.button_alive:
              self.button_wait = randint(8,15)
              self.last_button_frame = self.frame
            elif CC.enabled and CS.cruise_buttons[-1] == Buttons.NONE:
              if not CS.out.cruiseState.enabled:
                if CC.longActive and (hud_control.leadVisible or current > 10.0):
                  can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint))
                  CC.debugTextCC = "BTN:++,T:{:.1f},C:{:.1f}".format(target, current)
                #elif CC.longActive:
                #  can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL, self.CP.carFingerprint))
                #  CC.debugTextCC = "BTN:--,T:{:.1f},C:{:.1f}".format(target, current)
                #elif CS.out.cruiseGap != hud_control.cruiseGap:
                #  can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP.carFingerprint))
                #  #print("currentGap = {}, target = {}".format(CS.out.cruiseGap, hud_control.cruiseGap))
              elif CS.out.cruiseGap != hud_control.cruiseGap:
                can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.GAP_DIST, self.CP.carFingerprint))
                CC.debugTextCC = "currentGap = {}, target = {}".format(CS.out.cruiseGap, hud_control.cruiseGap)
              elif target < current and current>= 31:
                can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.SET_DECEL, self.CP.carFingerprint))
                CC.debugTextCC = "BTN:--,T:{:.1f},C:{:.1f}".format(target, current)
              elif target > current and current < 160:
                can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP.carFingerprint))
                CC.debugTextCC = "BTN:++,T:{:.1f},C:{:.1f}".format(target, current)
          else:
            self.button_alive = randint(4, 8) #randint(12, 18)
            self.button_alive_frame = self.frame

      #CC.debugTextCC = "230206"

      # K7 2020 also needs MDPS12 feedback toward the camera, or LKAS/FCA faults appear while openpilot steers (carrot-wip fix)
      if self.CP.carFingerprint in (CAR.GENESIS_G90_2019, CAR.GENESIS_G90, CAR.K7, CAR.K7_PE):
        can_sends.append(hyundaican.create_mdps12(self.packer, self.frame, CS.mdps12))

      # comma pedal: openpilot long through the accelerator; the car's own cruise control is cancelled while engaged
      if self.pedal:
        from selfdrive.car.hyundai.pedal import pedal_target, next_command
        if CC.enabled and CS.out.cruiseState.enabled and (self.frame - self.last_button_frame) * DT_CTRL > 0.04:
          can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP.carFingerprint))
          self.last_button_frame = self.frame
        if self.frame % 4 == 0:
          ready = getattr(CS, "pedal_valid", False) and getattr(CS, "pedal_state", 5) == 0
          active = CC.longActive and ready and not CC.cruiseControl.override and not CS.out.gasPressed and not CS.out.brakePressed
          if active:
            target = pedal_target(accel, CS.out.vEgo, hud_control.setSpeed, self.pedal_max)
            self.pedal_gas_last = next_command(target, self.pedal_gas_last)
          else:
            self.pedal_gas_last = 0.
          can_sends.append(create_gas_interceptor_command(self.pedal_packer, self.pedal_gas_last, (self.frame // 4) & 0xF))

      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl and not self.CP.flags & HyundaiFlags.NON_SCC:
        # TODO: unclear if this is needed
        startingJerk = self.jerkStartLimit
        jerkLimit = 5.0
        self.jerk_count += DT_CTRL
        jerk_max = interp(self.jerk_count, [0, 1.5, 2.5], [startingJerk, startingJerk, jerkLimit])
        a_error = accel - CS.out.aEgo
        v_error = actuators.speed - CS.out.vEgo
        cb_upper = cb_lower = 0
        if actuators.longControlState == LongCtrlState.off:
          jerk_u = jerkLimit
          jerk_l = jerkLimit          
          self.jerk_count = 0
        elif actuators.longControlState == LongCtrlState.stopping or hud_control.softHold:
          jerk_u = 0.5
          jerk_l = jerkLimit
          self.jerk_count = 0
        else:
          jerk_u = min(max(0.5, jerk * 2.0), jerk_max)
          jerk_l = min(max(1.0, -jerk * 2.0), jerk_max)
          cb_upper = clip(0.9 + accel * 0.2, 0, 1.2)
          cb_lower = clip(0.8 + accel * 0.2, 0, 1.2)

        can_sends.extend(hyundaican.create_acc_commands_mix_scc(self.CP, self.packer, CC.enabled, accel, jerk_u, jerk_l, int(self.frame / 2),
                                                      hud_control, set_speed_in_units, stopping, CC, CS, self.softHoldMode, cb_upper, cb_lower))
        self.accel_last = accel

      # 20 Hz LFA MFA message
      if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC, self.blinking_signal))

      # 5 Hz ACC options
      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl and not self.CP.flags & HyundaiFlags.NON_SCC:
        if self.CP.sccBus == 0:
          can_sends.extend(hyundaican.create_acc_opt(self.CP, CS, self.packer))
        elif CS.scc13 is not None:
          can_sends.append(hyundaican.create_acc_opt_copy(self.CP, CS, self.packer))

      # 2 Hz front radar options
      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl  and self.CP.sccBus == 0 and not self.CP.flags & HyundaiFlags.NON_SCC:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.accel = accel

    try:
      CC.debugTextCC = self.drive_record_text(CS, can_sends)
    except Exception:  # noqa: BLE001  (the drive record must never stop the car's messages)
      pass

    self.frame += 1
    return new_actuators, can_sends
