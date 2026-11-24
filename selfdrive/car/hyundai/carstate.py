from collections import deque
import copy
import math

from cereal import car
from common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.hyundai.values import HyundaiFlags, DBC, FEATURES, CAMERA_SCC_CAR, CANFD_CAR, EV_CAR, HYBRID_CAR, Buttons, CarControllerParams, CAR
from selfdrive.car.interfaces import CarStateBase
from common.realtime import DT_CTRL

PREV_BUTTON_SAMPLES = 8
CLUSTER_SAMPLE_RATE = 20  # frames
GearShifter = car.CarState.GearShifter

# KIA SOUL EV (SK3), carrot e5cf102: TCS13 ACCEnable (0 = ACC allowed, 1-3 = not) blips non-zero for 1-2 frames
# with no real fault, and openpilot turned off at once (accFaulted = immediate disable; with openpilot long and the
# SCC radar off cruiseState.available comes from the same signal = wrongCarMode). On this car it counts only after
# this many frames in a row (100 Hz, ~70 ms); a real lasting fault still turns openpilot off within ~70 ms.
ACC_FAULT_DEBOUNCE_FRAMES = 7

def driver_pressed(gas, gas2):
  from selfdrive.car.hyundai.pedal import driver_pressed as _pressed
  return _pressed(gas, gas2)


def non_scc_cruise_signals(CP):
  """(available msg, sig, engaged msg, sig, set speed msg, sig) for cars without SCC (StarPilot get_non_scc_cruise_signals)"""
  if CP.carFingerprint == CAR.KIA_RAY_EV:
    return "LABEL11", "CC_React", "LABEL11", "CC_Engaged", "E_EMS11", "Cruise_Limit_Target"
  if CP.carFingerprint in EV_CAR:
    return "LABEL11", "CC_React", "EMS12", "ACC_ACT", "E_EMS11", "Cruise_Limit_Target"
  if CP.carFingerprint in HYBRID_CAR:
    return "E_CRUISE_CONTROL", "CRUISE_LAMP_M", "E_CRUISE_CONTROL", "CRUISE_LAMP_S", "ELECT_GEAR", "SLC_SET_SPEED"
  return "EMS16", "CRUISE_LAMP_M", "EMS16", "CRUISE_LAMP_S", "LVR12", "CF_Lvr_CruiseSet"


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    self.cruise_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.main_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)

    self.gear_msg_canfd = "GEAR_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS else "GEAR_SHIFTER"
    if CP.carFingerprint in CANFD_CAR:
      self.shifter_values = can_define.dv[self.gear_msg_canfd]["GEAR"]
    elif self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    else:  # preferred and elect gear methods use same definition
      self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    self.is_metric = False
     #ajouatom
    self.prev_pcmMode = False
    self.mdps_error_cnt = 0

    self.brake_error = False
    self.buttons_counter = 0
    # optech drive record: SCC radar -> cluster codes and the stock camera's own lane state (see update)
    self.scc_diag = None
    self.cam_lkas_diag = None
    self.fca_diag = None
    self.cam_fcw_diag = None

    self.cruise_info = {}

    # On some cars, CLU15->CF_Clu_VehicleSpeed can oscillate faster than the dash updates. Sample at 5 Hz
    self.cluster_speed = 0
    self.cluster_speed_counter = CLUSTER_SAMPLE_RATE

    self.params = CarControllerParams(CP)
    self.gear_shifter = GearShifter.drive # Gear_init for Nexo ?? unknown 21.02.23.LSW

    self.totalDistance = 0.0
    self.speedLimitDistance = 0
    self.acc_fault_frames = 0  # KIA SOUL EV (SK3): TCS13 ACCEnable != 0 frames in a row (ACC_FAULT_DEBOUNCE_FRAMES)

  def acc_disabled(self, cp):
    """TCS13 ACCEnable says ACC is not allowed (1-3). Soul EV (SK3): only after ACC_FAULT_DEBOUNCE_FRAMES in a row."""
    raw = cp.vl["TCS13"]["ACCEnable"] != 0
    if self.CP.carFingerprint != CAR.KIA_SOUL_EV_SK3:
      return raw
    self.acc_fault_frames = min(self.acc_fault_frames + 1, ACC_FAULT_DEBOUNCE_FRAMES) if raw else 0
    return self.acc_fault_frames >= ACC_FAULT_DEBOUNCE_FRAMES

  def update(self, cp, cp_cam):
    if self.CP.carFingerprint in CANFD_CAR:
      return self.update_canfd(cp, cp_cam)

    ret = car.CarState.new_message()
    cp_cruise = cp_cam if self.CP.sccBus == 2 or self.CP.carFingerprint in CAMERA_SCC_CAR else cp
    self.is_metric = cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"] == 0
    speed_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    ret.doorOpen = any([cp.vl["CGW1"]["CF_Gway_DrvDrSw"], cp.vl["CGW1"]["CF_Gway_AstDrSw"],
                        cp.vl["CGW2"]["CF_Gway_RLDrSw"], cp.vl["CGW2"]["CF_Gway_RRDrSw"]])

    ret.seatbeltUnlatched = cp.vl["CGW1"]["CF_Gway_DrvSeatBeltSw"] == 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHL_SPD11"]["WHL_SPD_FL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_FR"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RR"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    self.cluster_speed_counter += 1
    if self.cluster_speed_counter > CLUSTER_SAMPLE_RATE:
      self.cluster_speed = cp.vl["CLU15"]["CF_Clu_VehicleSpeed"]
      self.cluster_speed_counter = 0

      # mimic how dash converts to imperial
      if not self.is_metric:
        self.cluster_speed = math.floor(self.cluster_speed * CV.KPH_TO_MPH + CV.KPH_TO_MPH)

    ret.vEgoCluster = self.cluster_speed * speed_conv

    ret.steeringAngleDeg = cp.vl["SAS11"]["SAS_Angle"]
    ret.steeringRateDeg = cp.vl["SAS11"]["SAS_Speed"]
    ret.yawRate = cp.vl["ESP12"]["YAW_RATE"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cp.vl["CGW1"]["CF_Gway_TurnSigLh"], cp.vl["CGW1"]["CF_Gway_TurnSigRh"])
    ret.steeringTorque = cp.vl["MDPS12"]["CR_Mdps_StrColTq"]
    ret.steeringTorqueEps = cp.vl["MDPS12"]["CR_Mdps_OutTq"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    #ret.steerFaultTemporary = cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0 or cp.vl["MDPS12"]["CF_Mdps_ToiFlt"] != 0
    if not ret.standstill and cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0:
      self.mdps_error_cnt += 1
    else:
      self.mdps_error_cnt = 0

    ret.steerFaultTemporary = self.mdps_error_cnt > 50
    ret.steerFaultTemporary = False

    # cruise state
    acc_disabled = self.acc_disabled(cp)  # once per frame: it counts frames on the Soul EV (SK3)
    ret.cruiseState.pcmMode = self.prev_pcmMode
    if self.CP.flags & HyundaiFlags.NON_SCC:
      avail_msg, avail_sig, on_msg, on_sig, speed_msg, speed_sig = non_scc_cruise_signals(self.CP)
      ret.cruiseState.available = cp.vl[avail_msg][avail_sig] != 0
      ret.cruiseState.enabled = cp.vl[on_msg][on_sig] != 0
      ret.cruiseState.standstill = False
      ret.cruiseState.speed = cp.vl[speed_msg][speed_sig] * speed_conv
    elif self.CP.openpilotLongitudinalControl and self.CP.sccBus == 0: # SCC0: Disable SCC Module
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.available = not acc_disabled
      ret.cruiseState.enabled = cp.vl["TCS13"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
    elif self.CP.openpilotLongitudinalControl: # SCC2 배선개조된경우
      ret.cruiseState.available = cp_cruise.vl["SCC11"]["MainMode_ACC"] == 1
      if ret.cruiseState.available: # 크루즈버튼이 눌림.
        ret.cruiseState.enabled = cp_cruise.vl["SCC12"]["ACCMode"] != 0
        ret.cruiseState.standstill = False
        ret.cruiseState.pcmMode = True
      else:
        ret.cruiseState.available = not acc_disabled
        ret.cruiseState.enabled = cp.vl["TCS13"]["ACC_REQ"] == 1
        ret.cruiseState.standstill = False
    else:
      ret.cruiseState.available = cp_cruise.vl["SCC11"]["MainMode_ACC"] == 1
      ret.cruiseState.enabled = cp_cruise.vl["SCC12"]["ACCMode"] != 0
      ret.cruiseState.standstill = cp_cruise.vl["SCC11"]["SCCInfoDisplay"] == 4.
      ret.cruiseState.speed = cp_cruise.vl["SCC11"]["VSetDis"] * speed_conv

      ret.cruiseGap = cp_cruise.vl["SCC11"]["TauGapSet"]

    self.prev_pcmMode = ret.cruiseState.pcmMode

    # optech drive record (drivelog, through carcontroller -> carControl.debugTextCC): what the SCC radar tells the
    # cluster - SCCInfoDisplay (its messages, e.g. 'SCC 작동조건이 아님'), DriverAlertDisplay, ACCFailInfo - and ACCMode
    self.scc_diag = None
    if not (self.CP.flags & HyundaiFlags.NON_SCC):
      try:
        self.scc_diag = (int(cp_cruise.vl["SCC11"]["SCCInfoDisplay"]), int(cp_cruise.vl["SCC11"]["DriverAlertDisplay"]),
                         int(cp_cruise.vl["SCC12"]["ACCFailInfo"]), int(cp_cruise.vl["SCC12"]["ACCMode"]))
      except KeyError:  # SCC module not read on this setup (openpilot long with the radar off)
        pass
    # ... and the forward collision-avoidance (FCA / AEB) fault and state the radar reports to the car: a fault there
    # is the cluster's '전방 충돌방지 보조 시스템을 점검하세요' (tester + user 2026-11-23)
    self.fca_diag = None
    if not (self.CP.flags & HyundaiFlags.NON_SCC):
      try:
        if self.CP.flags & HyundaiFlags.USE_FCA.value:
          self.fca_diag = (int(cp_cruise.vl["FCA11"]["FCA_Failinfo"]), int(cp_cruise.vl["FCA11"]["FCA_Status"]))
        else:
          self.fca_diag = (int(cp_cruise.vl["SCC12"]["AEB_Failinfo"]), int(cp_cruise.vl["SCC12"]["AEB_Status"]))
      except KeyError:
        pass

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["TCS13"]["DriverBraking"] != 0
    if self.CP.flags & HyundaiFlags.NON_SCC_NO_TCS:
      ret.brakePressed = cp.vl["E_EMS11"]["Brake_Pedal_Pos"] > 0  # no TCS13 on this car (Porter II EV, carrot)
    ret.brakeHoldActive = cp.vl["TCS15"]["AVH_LAMP"] == 2 # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
    ret.parkingBrake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
    ret.accFaulted = acc_disabled  # TCS13 ACCEnable: 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED
    if self.CP.flags & HyundaiFlags.NON_SCC:
      ret.accFaulted = False  # TCS13 ACCEnable means nothing without SCC
    ret.brakeLights = bool(cp.vl["TCS13"]["BrakeLight"] or ret.brakePressed)
    ret.driverOverride = cp.vl["TCS13"]["DriverOverride"]

    if self.CP.carFingerprint in (HYBRID_CAR | EV_CAR):
      if self.CP.carFingerprint in HYBRID_CAR:
        ret.gas = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] / 254.
        ret.engineRpm = cp.vl["E_EMS11"]["N"] # opkr
        ret.motorRpm = cp.vl["ELECT_GEAR"]["Elect_Motor_Speed"] * 30
      else:
        ret.gas = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] / 254.
        ret.engineRpm = 0.0
        ret.motorRpm = cp.vl["ELECT_GEAR"]["Elect_Motor_Speed"] * 30 # opkr, may multiply deceleration ratio in line with engine rpm
      ret.chargeMeter = cp.vl["EV_Info"]["OPKR_EV_Charge_Level"] # opkr
      ret.gasPressed = ret.gas > 0
      if self.CP.flags & HyundaiFlags.NON_SCC_PEDAL:
        # the pedal box sits between the pedal and the car: the car's pedal signal is our command, the driver's is in GAS_SENSOR
        cp_pedal = getattr(self, "cp_pedal", None)
        self.pedal_valid = cp_pedal is not None and cp_pedal.can_valid
        self.pedal_state = int(cp_pedal.vl["GAS_SENSOR"]["STATE"]) if cp_pedal is not None else 5
        ret.accFaulted = not self.pedal_valid or self.pedal_state != 0
        if cp_pedal is not None:
          ret.gasPressed = driver_pressed(cp_pedal.vl["GAS_SENSOR"]["INTERCEPTOR_GAS"], cp_pedal.vl["GAS_SENSOR"]["INTERCEPTOR_GAS2"])
    else:
      ret.gas = cp.vl["EMS12"]["PV_AV_CAN"] / 100.
      ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])
      ret.engineRpm = cp.vl["EMS_366"]["N"]
      ret.motorRpm = 0
      ret.chargeMeter = 0

    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    if self.CP.carFingerprint in FEATURES["use_cluster_gears"]:
      gear = cp.vl["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in FEATURES["use_tcu_gears"]:
      gear = cp.vl["TCU12"]["CUR_GR"]
    elif self.CP.carFingerprint in FEATURES["use_elect_gears"]:
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
      if self.CP.carFingerprint in (CAR.NEXO):
        gear_disp = cp.vl["ELECT_GEAR"]
  
        gear_shifter = GearShifter.unknown
  
        if gear == 1546:  # Thank you for Neokii  # fix PolorBear 22.06.05
          gear_shifter = GearShifter.drive
        elif gear == 2314:
          gear_shifter = GearShifter.neutral
        elif gear == 2569:
          gear_shifter = GearShifter.park
        elif gear == 2566:
          gear_shifter = GearShifter.reverse
  
        if gear_shifter != GearShifter.unknown and self.gear_shifter != gear_shifter:
          self.gear_shifter = gear_shifter
  
        ret.gearShifter = self.gear_shifter
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]

    if not self.CP.carFingerprint in (CAR.NEXO):
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    if self.CP.flags & HyundaiFlags.NON_SCC:
      pass  # no SCC12 / FCA11 to read stock AEB from
    elif not self.CP.openpilotLongitudinalControl:
      aeb_src = "FCA11" if self.CP.flags & HyundaiFlags.USE_FCA.value else "SCC12"
      aeb_sig = "FCA_CmdAct" if self.CP.flags & HyundaiFlags.USE_FCA.value else "AEB_CmdAct"
      aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
      aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src][aeb_sig] != 0
      ret.stockFcw = aeb_warning and not aeb_braking
      ret.stockAeb = aeb_warning and aeb_braking
    elif self.CP.sccBus == 2:
      aeb_src = "FCA11" if self.CP.flags & HyundaiFlags.USE_FCA.value else "SCC12"
      aeb_sig = "FCA_CmdAct" if self.CP.flags & HyundaiFlags.USE_FCA.value else "AEB_CmdAct"
      aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
      aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src][aeb_sig] != 0
      ret.stockFcw = aeb_warning and not aeb_braking
      ret.stockAeb = aeb_warning and aeb_braking

    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["LCA11"]["CF_Lca_IndLeft"] != 0
      ret.rightBlindspot = cp.vl["LCA11"]["CF_Lca_IndRight"] != 0

    # save the entire LKAS11 and CLU11
    self.lkas11 = copy.copy(cp_cam.vl["LKAS11"])
    # the stock camera's own lane state and warning (before openpilot rewrites LKAS11), for the drive record
    self.cam_lkas_diag = (int(self.lkas11.get("CF_Lkas_LdwsSysState", -1)), int(self.lkas11.get("CF_Lkas_SysWarning", -1)))
    # the stock camera's forward collision warning state and radar fusion state (LKAS11), for the drive record
    self.cam_fcw_diag = (int(self.lkas11.get("CF_Lkas_FcwSysState", -1)), int(self.lkas11.get("CF_Lkas_FusionState", -1)))
    self.clu11 = copy.copy(cp.vl["CLU11"])
    self.steer_state = cp.vl["MDPS12"]["CF_Mdps_ToiActive"]  # 0 NOT ACTIVE, 1 ACTIVE
    self.prev_cruise_buttons = self.cruise_buttons[-1]
    self.cruise_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"])
    self.main_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwMain"])
    self.mdps12 = copy.copy(cp.vl["MDPS12"])
    tpms_unit = cp.vl["TPMS11"]["UNIT"] * 0.725 if int(cp.vl["TPMS11"]["UNIT"]) > 0 else 1.
    ret.tpms.fl = tpms_unit * cp.vl["TPMS11"]["PRESSURE_FL"]
    ret.tpms.fr = tpms_unit * cp.vl["TPMS11"]["PRESSURE_FR"]
    ret.tpms.rl = tpms_unit * cp.vl["TPMS11"]["PRESSURE_RL"]
    ret.tpms.rr = tpms_unit * cp.vl["TPMS11"]["PRESSURE_RR"]

    #self.scc11 = copy.copy(cp_cruise.vl["SCC11"])
    #self.scc12 = copy.copy(cp_cruise.vl["SCC12"])
    #self.scc13 = copy.copy(cp_cruise.vl["SCC13"]) if "SCC13" in cp_cruise.vl else None
    #self.scc14 = copy.copy(cp_cruise.vl["SCC14"])
    self.scc11 = cp_cruise.vl["SCC11"] if "SCC11" in cp_cruise.vl else None
    self.scc12 = cp_cruise.vl["SCC12"] if "SCC12" in cp_cruise.vl else None
    self.scc13 = cp_cruise.vl["SCC13"] if "SCC13" in cp_cruise.vl else None
    self.scc14 = cp_cruise.vl["SCC14"] if "SCC14" in cp_cruise.vl else None
    cluSpeed = cp.vl["CLU11"]["CF_Clu_Vanz"]
    decimal = cp.vl["CLU11"]["CF_Clu_VanzDecimal"]
    if 0. < decimal < 0.5:
      cluSpeed += decimal

    ret.vEgoCluster = cluSpeed * speed_conv
    vEgoClu, aEgoClu = self.update_clu_speed_kf(ret.vEgoCluster)
    ret.vCluRatio = (ret.vEgo / vEgoClu) if (vEgoClu > 3. and ret.vEgo > 3.) else 1.0

    self.totalDistance += ret.vEgo * DT_CTRL # 후진할때는?
    ret.totalDistance = self.totalDistance

    if self.CP.naviCluster == 1:
      speedLimit = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"]
      speedLimitCam = cp.vl["Navi_HU"]["SpeedLim_Nav_Cam"]
      ret.speedLimit = speedLimit if speedLimit < 255 and speedLimitCam == 1 else 0
      if ret.speedLimit>0 and not ret.gasPressed:
        if self.speedLimitDistance <= self.totalDistance:
          self.speedLimitDistance = self.totalDistance + ret.speedLimit * 6  #일반적으로 속도*6M 시점에 안내하는것으로 보임.
        self.speedLimitDistance = max(self.totalDistance+1, self.speedLimitDistance) #구간또는 거리가 벗어난경우에는 1M를 유지함.
      else:
        self.speedLimitDistance = self.totalDistance
      ret.speedLimitDistance = self.speedLimitDistance - self.totalDistance
    else:
      ret.speedLimit = 0
      ret.speedLimitDistance = 0

    #scc12_2 = cp_cam.vl["SCC12"]
    #scc12 = cp.vl["SCC12"]
    #print("scc12_2=", scc12_2)
    #print("scc12=", scc12)
    return ret

  def update_canfd(self, cp, cp_cam):
    ret = car.CarState.new_message()

    if self.CP.carFingerprint in (EV_CAR | HYBRID_CAR):
      if self.CP.carFingerprint in EV_CAR:
        ret.gas = cp.vl["ACCELERATOR"]["ACCELERATOR_PEDAL"] / 255.
      else:
        ret.gas = cp.vl["ACCELERATOR_ALT"]["ACCELERATOR_PEDAL"] / 1023.
      ret.gasPressed = ret.gas > 1e-5
    else:
      ret.gasPressed = bool(cp.vl["ACCELERATOR_BRAKE_ALT"]["ACCELERATOR_PEDAL_PRESSED"])

    ret.brakePressed = cp.vl["TCS"]["DriverBraking"] == 1

    ret.doorOpen = cp.vl["DOORS_SEATBELTS"]["DRIVER_DOOR_OPEN"] == 1
    ret.seatbeltUnlatched = cp.vl["DOORS_SEATBELTS"]["DRIVER_SEATBELT_LATCHED"] == 0

    gear = cp.vl[self.gear_msg_canfd]["GEAR"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    # TODO: figure out positions
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_1"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_2"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_3"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_4"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1

    ret.steeringRateDeg = cp.vl["STEERING_SENSORS"]["STEERING_RATE"]
    ret.steeringAngleDeg = cp.vl["STEERING_SENSORS"]["STEERING_ANGLE"] * -1
    ret.steeringTorque = cp.vl["MDPS"]["STEERING_COL_TORQUE"]
    ret.steeringTorqueEps = cp.vl["MDPS"]["STEERING_OUT_TORQUE"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    ret.steerFaultTemporary = cp.vl["MDPS"]["LKA_FAULT"] != 0

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["BLINKERS"]["LEFT_LAMP"],
                                                                      cp.vl["BLINKERS"]["RIGHT_LAMP"])
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"]["FL_INDICATOR"] != 0
      ret.rightBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"]["FR_INDICATOR"] != 0

    ret.cruiseState.available = True
    self.is_metric = cp.vl["CLUSTER_INFO"]["DISTANCE_UNIT"] != 1
    if not self.CP.openpilotLongitudinalControl:
      speed_factor = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
      cp_cruise_info = cp_cam if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else cp
      ret.cruiseState.speed = cp_cruise_info.vl["SCC_CONTROL"]["VSetDis"] * speed_factor
      ret.cruiseState.standstill = cp_cruise_info.vl["SCC_CONTROL"]["CRUISE_STANDSTILL"] == 1
      ret.cruiseState.enabled = cp_cruise_info.vl["SCC_CONTROL"]["ACCMode"] in (1, 2)
      self.cruise_info = copy.copy(cp_cruise_info.vl["SCC_CONTROL"])

    cruise_btn_msg = "CRUISE_BUTTONS_ALT" if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS else "CRUISE_BUTTONS"
    self.prev_cruise_buttons = self.cruise_buttons[-1]
    self.cruise_buttons.extend(cp.vl_all[cruise_btn_msg]["CRUISE_BUTTONS"])
    self.main_buttons.extend(cp.vl_all[cruise_btn_msg]["ADAPTIVE_CRUISE_MAIN_BTN"])
    self.buttons_counter = cp.vl[cruise_btn_msg]["COUNTER"]

    if self.CP.flags & HyundaiFlags.CANFD_HDA2:
      self.cam_0x2a4 = copy.copy(cp_cam.vl["CAM_0x2a4"])

    return ret

  @staticmethod
  def get_can_parser(CP):
    if CP.carFingerprint in CANFD_CAR:
      return CarState.get_can_parser_canfd(CP)

    signals = [
      # signal_name, signal_address
      ("WHL_SPD_FL", "WHL_SPD11"),
      ("WHL_SPD_FR", "WHL_SPD11"),
      ("WHL_SPD_RL", "WHL_SPD11"),
      ("WHL_SPD_RR", "WHL_SPD11"),

      ("YAW_RATE", "ESP12"),

      ("CF_Gway_DrvSeatBeltInd", "CGW4"),

      ("CF_Gway_DrvSeatBeltSw", "CGW1"),
      ("CF_Gway_DrvDrSw", "CGW1"),       # Driver Door
      ("CF_Gway_AstDrSw", "CGW1"),       # Passenger Door
      ("CF_Gway_RLDrSw", "CGW2"),        # Rear left Door
      ("CF_Gway_RRDrSw", "CGW2"),        # Rear right Door
      ("CF_Gway_TurnSigLh", "CGW1"),
      ("CF_Gway_TurnSigRh", "CGW1"),
      ("CF_Gway_ParkBrakeSw", "CGW1"),

      ("CYL_PRES", "ESP12"),

      ("CF_Clu_CruiseSwState", "CLU11"),
      ("CF_Clu_CruiseSwMain", "CLU11"),
      ("CF_Clu_SldMainSW", "CLU11"),
      ("CF_Clu_ParityBit1", "CLU11"),
      ("CF_Clu_VanzDecimal" , "CLU11"),
      ("CF_Clu_Vanz", "CLU11"),
      ("CF_Clu_SPEED_UNIT", "CLU11"),
      ("CF_Clu_DetentOut", "CLU11"),
      ("CF_Clu_RheostatLevel", "CLU11"),
      ("CF_Clu_CluInfo", "CLU11"),
      ("CF_Clu_AmpInfo", "CLU11"),
      ("CF_Clu_AliveCnt1", "CLU11"),

      ("CF_Clu_VehicleSpeed", "CLU15"),

      ("ACCEnable", "TCS13"),
      ("BrakeLight", "TCS13"),
      ("DCEnable", "TCS13"),
      ("ACC_REQ", "TCS13"),
      ("DriverBraking", "TCS13"),
      ("DriverOverride", "TCS13"), # scc smoother
      ("StandStill", "TCS13"),
      ("PBRAKE_ACT", "TCS13"),

      ("ESC_Off_Step", "TCS15"),
      ("AVH_LAMP", "TCS15"),

      ("CR_Mdps_StrColTq", "MDPS12"),
      ("CF_Mdps_Def", "MDPS12"),
      ("CF_Mdps_ToiActive", "MDPS12"),
      ("CF_Mdps_ToiUnavail", "MDPS12"),
      ("CF_Mdps_ToiFlt", "MDPS12"),
      ("CF_Mdps_MsgCount2", "MDPS12"),
      ("CF_Mdps_Chksum2", "MDPS12"),
      ("CF_Mdps_SErr", "MDPS12"),
      ("CR_Mdps_StrTq", "MDPS12"),
      ("CF_Mdps_FailStat", "MDPS12"),
      ("CR_Mdps_OutTq", "MDPS12"),

      ("SAS_Angle", "SAS11"),
      ("SAS_Speed", "SAS11"),
      ("UNIT", "TPMS11"),
      ("PRESSURE_FL", "TPMS11"),
      ("PRESSURE_FR", "TPMS11"),
      ("PRESSURE_RL", "TPMS11"),
      ("PRESSURE_RR", "TPMS11"),

      ("N", "EMS_366"),

      ("OPKR_EV_Charge_Level", "EV_Info"),
    ]
    checks = [
      # address, frequency
      ("MDPS12", 50),
      ("TCS13", 50),
      ("TCS15", 10),
      ("CLU11", 50),
      ("CLU15", 5),
      ("ESP12", 100),
      ("CGW1", 10),
      ("CGW2", 5),
      ("CGW4", 5),
      ("WHL_SPD11", 50),
      ("SAS11", 100),
      ("TPMS11", 5),      
    ]

    if CP.flags & HyundaiFlags.NON_SCC:
      avail_msg, avail_sig, on_msg, on_sig, speed_msg, speed_sig = non_scc_cruise_signals(CP)
      signals += [(avail_sig, avail_msg), (on_sig, on_msg), (speed_sig, speed_msg), ("Brake_Pedal_Pos", "E_EMS11")]
      # these cars are not driven on this code yet: only the messages steering needs are timed; TCS/TPMS/cluster-only
      # messages a cruise-control-only car may not have are read when present (enforce_checks=False)
      checks = [c for c in checks if c[0] not in ("TCS13", "TCS15", "TPMS11", "CGW4", "CLU15", "CGW2")]
      for m in (avail_msg, on_msg, speed_msg):
        if m not in [c[0] for c in checks]:
          checks.append((m, 10))
    elif not CP.openpilotLongitudinalControl and CP.carFingerprint not in CAMERA_SCC_CAR:
      signals += [
        ("MainMode_ACC", "SCC11"),
        ("VSetDis", "SCC11"),
        ("SCCInfoDisplay", "SCC11"),
        ("ACC_ObjDist", "SCC11"),
        ("TauGapSet", "SCC11"),
        ("ACCMode", "SCC12"),
        ("DriverAlertDisplay", "SCC11"),  # optech drive record (cluster messages from the SCC radar)
        ("ACCFailInfo", "SCC12"),
        ("AEB_Failinfo", "SCC12"),  # optech drive record: forward collision-avoidance fault / state
        ("AEB_Status", "SCC12"),
      ]
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]

      if CP.flags & HyundaiFlags.USE_FCA.value:
        signals += [
          ("FCA_CmdAct", "FCA11"),
          ("CF_VSM_Warn", "FCA11"),
          ("CF_VSM_DecCmdAct", "FCA11"),
          ("FCA_Failinfo", "FCA11"),  # optech drive record
          ("FCA_Status", "FCA11"),
        ]
        checks.append(("FCA11", 50))
      else:
        signals += [
          ("AEB_CmdAct", "SCC12"),
          ("CF_VSM_Warn", "SCC12"),
          ("CF_VSM_DecCmdAct", "SCC12"),
        ]

    if CP.enableBsm:
      signals += [
        ("CF_Lca_IndLeft", "LCA11"),
        ("CF_Lca_IndRight", "LCA11"),
      ]
      checks.append(("LCA11", 50))

    if CP.carFingerprint in (HYBRID_CAR | EV_CAR):
      if CP.carFingerprint in HYBRID_CAR:
        signals.append(("CR_Vcu_AccPedDep_Pos", "E_EMS11"))
        signals.append(("N", "E_EMS11"))
      else:
        signals.append(("Accel_Pedal_Pos", "E_EMS11"))
      checks.append(("E_EMS11", 50))
    else:
      signals += [
        ("PV_AV_CAN", "EMS12"),
        ("CF_Ems_AclAct", "EMS16"),
      ]
      checks += [
        ("EMS12", 100),
        ("EMS16", 100),
      ]

    if CP.carFingerprint in FEATURES["use_cluster_gears"]:
      signals.append(("CF_Clu_Gear", "CLU15"))
    elif CP.carFingerprint in FEATURES["use_tcu_gears"]:
      signals.append(("CUR_GR", "TCU12"))
      checks.append(("TCU12", 100))
    elif CP.carFingerprint in FEATURES["use_elect_gears"]:
      signals.append(("Elect_Gear_Shifter", "ELECT_GEAR"))
      signals.append(("Elect_Motor_Speed", "ELECT_GEAR"))
      checks.append(("ELECT_GEAR", 20))
    else:
      signals.append(("CF_Lvr_Gear", "LVR12"))
      checks.append(("LVR12", 100))

    if CP.naviCluster == 1:
      signals += [
        ("SpeedLim_Nav_Clu", "Navi_HU"),
        ("SpeedLim_Nav_Cam","Navi_HU")
      ]
      checks.append(("Navi_HU", 5))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0, enforce_checks=False)

  @staticmethod
  def get_body_can_parser(CP):
    # comma pedal on the cars without SCC (only when the menu switch is on and the pedal was seen at start)
    if CP.flags & HyundaiFlags.NON_SCC_PEDAL:
      from selfdrive.car.hyundai.pedal import PEDAL_DBC
      signals = [("INTERCEPTOR_GAS", "GAS_SENSOR"), ("INTERCEPTOR_GAS2", "GAS_SENSOR"), ("STATE", "GAS_SENSOR")]
      return CANParser(PEDAL_DBC[CP.carFingerprint], signals, [("GAS_SENSOR", 50)], 0)
    return None

  @staticmethod
  def get_cam_can_parser(CP):
    if CP.carFingerprint in CANFD_CAR:
      return CarState.get_cam_can_parser_canfd(CP)

    signals = [
      # signal_name, signal_address
      ("CF_Lkas_LdwsActivemode", "LKAS11"),
      ("CF_Lkas_LdwsSysState", "LKAS11"),
      ("CF_Lkas_SysWarning", "LKAS11"),
      ("CF_Lkas_LdwsLHWarning", "LKAS11"),
      ("CF_Lkas_LdwsRHWarning", "LKAS11"),
      ("CF_Lkas_HbaLamp", "LKAS11"),
      ("CF_Lkas_FcwBasReq", "LKAS11"),
      ("CF_Lkas_ToiFlt", "LKAS11"),
      ("CF_Lkas_HbaSysState", "LKAS11"),
      ("CF_Lkas_FcwOpt", "LKAS11"),
      ("CF_Lkas_HbaOpt", "LKAS11"),
      ("CF_Lkas_FcwSysState", "LKAS11"),
      ("CF_Lkas_FcwCollisionWarning", "LKAS11"),
      ("CF_Lkas_MsgCount", "LKAS11"),
      ("CF_Lkas_FusionState", "LKAS11"),
      ("CF_Lkas_FcwOpt_USM", "LKAS11"),
      ("CF_Lkas_LdwsOpt_USM", "LKAS11"),
    ]
    checks = [
      ("LKAS11", 100)
    ]
    if CP.sccBus == 2:
      signals += [
        ("MainMode_ACC", "SCC11"),
        ("SCCInfoDisplay", "SCC11"),
        ("AliveCounterACC", "SCC11"),
        ("VSetDis", "SCC11"),
        ("ObjValid", "SCC11"),
        ("DriverAlertDisplay", "SCC11"),
        ("TauGapSet", "SCC11"),
        ("ACC_ObjStatus", "SCC11"),
        ("ACC_ObjLatPos", "SCC11"),
        ("ACC_ObjDist", "SCC11"),
        ("ACC_ObjRelSpd", "SCC11"),
        ("Navi_SCC_Curve_Status", "SCC11"),
        ("Navi_SCC_Curve_Act", "SCC11"),
        ("Navi_SCC_Camera_Act", "SCC11"),
        ("Navi_SCC_Camera_Status", "SCC11"),

        ("ACCMode", "SCC12"),
        ("CF_VSM_Prefill", "SCC12"),
        ("CF_VSM_DecCmdAct", "SCC12"),
        ("CF_VSM_HBACmd", "SCC12"),
        ("CF_VSM_Warn", "SCC12"),
        ("CF_VSM_Stat", "SCC12"),
        ("CF_VSM_BeltCmd", "SCC12"),
        ("ACCFailInfo", "SCC12"),
        ("StopReq", "SCC12"),
        ("ACCFailInfo", "SCC12"),
        ("CR_VSM_DecCmd", "SCC12"),
        ("aReqRaw", "SCC12"), #aReqMax
        ("TakeOverReq", "SCC12"),
        ("PreFill", "SCC12"),
        ("aReqValue", "SCC12"), #aReqMin
        ("CF_VSM_ConfMode", "SCC12"),
        ("AEB_Failinfo", "SCC12"),
        ("AEB_Status", "SCC12"),
        ("AEB_CmdAct", "SCC12"),
        ("AEB_StopReq", "SCC12"),
        ("CR_VSM_Alive", "SCC12"),
        ("CR_VSM_ChkSum", "SCC12"),

        ("SCCDrvModeRValue", "SCC13"),
        ("SCC_Equip", "SCC13"),
        ("AebDrvSetStatus", "SCC13"),

        ("JerkUpperLimit", "SCC14"),
        ("JerkLowerLimit", "SCC14"),
        ("ComfortBandUpper", "SCC14"),
        ("ComfortBandLower", "SCC14"),

        ("SCC14_Signal_29", "SCC14"),
        ("SCC14_Signal_48", "SCC14"),
      ]
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
      if CP.flags & HyundaiFlags.USE_FCA.value:
        signals += [
          ("CF_VSM_Prefill", "FCA11"),
          ("CF_VSM_HBACmd", "FCA11"),
          ("CF_VSM_BeltCmd", "FCA11"),
          ("CR_VSM_DecCmd", "FCA11"),
          ("FCA_Status", "FCA11"),
          ("FCA_StopReq", "FCA11"),
          ("FCA_DrvSetStatus", "FCA11"),
          ("FCA_Failinfo", "FCA11"),
          ("CR_FCA_Alive", "FCA11"),
          ("FCA_RelativeVelocity", "FCA11"),
          ("FCA_TimetoCollision", "FCA11"),
          ("CR_FCA_ChkSum", "FCA11"),
          ("PAINT1_Status", "FCA11"),
          ("FCA_CmdAct", "FCA11"),
          ("CF_VSM_Warn", "FCA11"),
          ("CF_VSM_DecCmdAct", "FCA11"),

          #("FCA_USM", "FCA12"),
          #("FCA_DrvSetState", "FCA12"),
        ]
        checks += [
          ("FCA11", 50),
          #("FCA12", 50),
        ]
      else:
        signals += [
          ("AEB_CmdAct", "SCC12"),
          ("CF_VSM_Warn", "SCC12"),
          ("CF_VSM_DecCmdAct", "SCC12"),
        ]

      if CP.hasLfaHda:
        signals += [
          ("HDA_USM", "LFAHDA_MFC"),
          ("HDA_Active", "LFAHDA_MFC"),
          ("HDA_Icon_State", "LFAHDA_MFC"),
          ("HDA_LdwSysState", "LFAHDA_MFC"),
          ("HDA_Icon_Wheel", "LFAHDA_MFC"),
        ]
        checks += [("LFAHDA_MFC", 20)]

      return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2, enforce_checks=False)

    if not CP.openpilotLongitudinalControl and CP.carFingerprint in CAMERA_SCC_CAR:
      signals += [
        ("MainMode_ACC", "SCC11"),
        ("VSetDis", "SCC11"),
        ("SCCInfoDisplay", "SCC11"),
        ("ACC_ObjDist", "SCC11"),
        ("ACCMode", "SCC12"),
      ]
      checks += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]

      if CP.flags & HyundaiFlags.USE_FCA.value:
        signals += [
          ("FCA_CmdAct", "FCA11"),
          ("CF_VSM_Warn", "FCA11"),
          ("CF_VSM_DecCmdAct", "FCA11"),
          ("FCA_Failinfo", "FCA11"),  # optech drive record
          ("FCA_Status", "FCA11"),
        ]
        checks.append(("FCA11", 50))
      else:
        signals += [
          ("AEB_CmdAct", "SCC12"),
          ("CF_VSM_Warn", "SCC12"),
          ("CF_VSM_DecCmdAct", "SCC12"),
        ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)

  @staticmethod
  def get_can_parser_canfd(CP):

    cruise_btn_msg = "CRUISE_BUTTONS_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS else "CRUISE_BUTTONS"
    gear_msg = "GEAR_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS else "GEAR_SHIFTER"
    signals = [
      ("WHEEL_SPEED_1", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_2", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_3", "WHEEL_SPEEDS"),
      ("WHEEL_SPEED_4", "WHEEL_SPEEDS"),

      ("GEAR", gear_msg),

      ("STEERING_RATE", "STEERING_SENSORS"),
      ("STEERING_ANGLE", "STEERING_SENSORS"),
      ("STEERING_COL_TORQUE", "MDPS"),
      ("STEERING_OUT_TORQUE", "MDPS"),
      ("LKA_FAULT", "MDPS"),

      ("DriverBraking", "TCS"),

      ("COUNTER", cruise_btn_msg),
      ("CRUISE_BUTTONS", cruise_btn_msg),
      ("ADAPTIVE_CRUISE_MAIN_BTN", cruise_btn_msg),

      ("DISTANCE_UNIT", "CLUSTER_INFO"),

      ("LEFT_LAMP", "BLINKERS"),
      ("RIGHT_LAMP", "BLINKERS"),

      ("DRIVER_DOOR_OPEN", "DOORS_SEATBELTS"),
      ("DRIVER_SEATBELT_LATCHED", "DOORS_SEATBELTS"),
    ]

    checks = [
      ("WHEEL_SPEEDS", 100),
      (gear_msg, 100),
      ("STEERING_SENSORS", 100),
      ("MDPS", 100),
      ("TCS", 50),
      (cruise_btn_msg, 50),
      ("CLUSTER_INFO", 4),
      ("BLINKERS", 4),
      ("DOORS_SEATBELTS", 4),
    ]

    if CP.enableBsm:
      signals += [
        ("FL_INDICATOR", "BLINDSPOTS_REAR_CORNERS"),
        ("FR_INDICATOR", "BLINDSPOTS_REAR_CORNERS"),
      ]
      checks += [
        ("BLINDSPOTS_REAR_CORNERS", 20),
      ]

    if not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and not CP.openpilotLongitudinalControl:
      signals += [
        ("ACCMode", "SCC_CONTROL"),
        ("VSetDis", "SCC_CONTROL"),
        ("CRUISE_STANDSTILL", "SCC_CONTROL"),
      ]
      checks += [
        ("SCC_CONTROL", 50),
      ]

    if CP.carFingerprint in EV_CAR:
      signals += [
        ("ACCELERATOR_PEDAL", "ACCELERATOR"),
      ]
      checks += [
        ("ACCELERATOR", 100),
      ]
    elif CP.carFingerprint in HYBRID_CAR:
      signals += [
        ("ACCELERATOR_PEDAL", "ACCELERATOR_ALT"),
      ]
      checks += [
        ("ACCELERATOR_ALT", 100),
      ]
    else:
      signals += [
        ("ACCELERATOR_PEDAL_PRESSED", "ACCELERATOR_BRAKE_ALT"),
      ]
      checks += [
        ("ACCELERATOR_BRAKE_ALT", 100),
      ]

    bus = 5 if CP.flags & HyundaiFlags.CANFD_HDA2 else 4
    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, bus)

  @staticmethod
  def get_cam_can_parser_canfd(CP):
    signals = []
    checks = []
    if CP.flags & HyundaiFlags.CANFD_HDA2:
      signals += [(f"BYTE{i}", "CAM_0x2a4") for i in range(3, 24)]
      checks += [("CAM_0x2a4", 20)]
    elif CP.flags & HyundaiFlags.CANFD_CAMERA_SCC:
      signals += [
        ("COUNTER", "SCC_CONTROL"),
        ("NEW_SIGNAL_1", "SCC_CONTROL"),
        ("MainMode_ACC", "SCC_CONTROL"),
        ("ACCMode", "SCC_CONTROL"),
        ("CRUISE_INACTIVE", "SCC_CONTROL"),
        ("ZEROS_9", "SCC_CONTROL"),
        ("CRUISE_STANDSTILL", "SCC_CONTROL"),
        ("ZEROS_5", "SCC_CONTROL"),
        ("DISTANCE_SETTING", "SCC_CONTROL"),
        ("VSetDis", "SCC_CONTROL"),
      ]

      checks += [
        ("SCC_CONTROL", 50),
      ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 6)
