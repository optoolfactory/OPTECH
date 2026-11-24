#!/usr/bin/env python3
"""optech '주행 기록 내려받기' (설정 > 고급 > 주행 기록): a small web page on the device for the driver's phone.

The menu button sets the param OptechLogServer to 1 and shows a QR code of http://<device ip>:8088. Then, for
SERVE_S, this serves the drive records in /data/drivelog (selfdrive/carrot/tools/drivelog.py) to a phone on the same
Wi-Fi / hotspot: a list of drives, each one as a .txt download, and all of them as one .zip. Read only, only that
folder, no other files. Afterwards the server stops and the param goes back to 0.
"""
import datetime
import html
import io
import os
import threading
import time
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

LOG_DIR = "/data/drivelog"
PORT = 8088
SERVE_S = 15 * 60
PARAM = "OptechLogServer"


def log_files():
  """Drive records (newest first), then crash tracebacks and diag.txt."""
  try:
    names = [f for f in os.listdir(LOG_DIR) if f.endswith(".txt") and os.path.isfile(os.path.join(LOG_DIR, f))]
  except OSError:
    return [], []
  # newest first by when each was last written: a name from 1970 (clock not set yet) says nothing about its age
  drives = sorted([f for f in names if f[:1].isdigit()], key=lambda f: os.path.getmtime(os.path.join(LOG_DIR, f)),
                  reverse=True)
  others = sorted([f for f in names if not f[:1].isdigit()], reverse=True)
  return drives, others


def drive_label(name, mtime):
  """20261121-143005.txt written until 14:52 -> '11/21 14:30 ~ 14:52 주행'. A drive that started with the clock at 1970
  (no network time, no GPS fix yet; drivelog renames it once the clock is set) shows what is known."""
  end = datetime.datetime.fromtimestamp(mtime)
  if len(name) >= 15 and name[8] == "-" and not name.startswith("19"):
    return f"{name[4:6]}/{name[6:8]} {name[9:11]}:{name[11:13]} ~ {end:%H:%M} 주행"
  if end.year >= 2024:
    return f"{end:%m/%d} ? ~ {end:%H:%M} 주행 (시작 시각 모름)"
  return f"날짜 모르는 주행 ({name})"


def size_text(path):
  n = os.path.getsize(path)
  return f"{n / 1e6:.1f}MB" if n >= 1e5 else f"{max(1, n // 1000)}KB"


def page():
  drives, others = log_files()
  rows = []
  for f in drives:
    p = os.path.join(LOG_DIR, f)
    rows.append(f'<a class="b" href="/f/{html.escape(f)}">{html.escape(drive_label(f, os.path.getmtime(p)))}'
                f'<small>{size_text(p)}</small></a>')
  extra = "".join(f'<a class="s" href="/f/{html.escape(f)}">{html.escape(f)} ({size_text(os.path.join(LOG_DIR, f))})</a>'
                  for f in others)
  body = "".join(rows) if rows else "<p>아직 주행 기록이 없어요.</p>"
  return f"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>OPTECH 주행 기록</title>
<style>
body{{font-family:sans-serif;margin:16px;background:#111;color:#eee}}
h1{{font-size:22px}} p{{color:#aaa;line-height:1.5}}
a.all{{display:block;background:#2e7d32;color:#fff;text-align:center;padding:18px;border-radius:10px;font-size:20px;
text-decoration:none;margin:16px 0}}
a.b{{display:flex;justify-content:space-between;background:#333;color:#fff;padding:16px;border-radius:8px;margin:8px 0;
text-decoration:none;font-size:18px}}
a.b small{{color:#aaa}} a.s{{display:block;color:#9ad;margin:6px 0;font-size:14px}}
</style></head><body>
<h1>OPTECH 주행 기록</h1>
<p>파일을 받은 뒤 카카오톡 등으로 보내 주세요. 기록에는 위치(좌표)가 들어 있지 않아요.</p>
<a class="all" href="/all.zip">전체 기록 한 번에 받기 (zip)</a>
{body}
<p style="margin-top:24px">기타 기록</p>{extra or '<p>없음</p>'}
</body></html>"""


def zip_all():
  drives, others = log_files()
  buf = io.BytesIO()
  with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
    for f in drives + others:
      p = os.path.join(LOG_DIR, f)
      try:
        # a zip can't hold a time before 1980: z.write() of a file last written while the clock was at 1970 raised
        # ValueError and stopped the whole download (2026-11-24, Soul EV tester: '다운로드 받을 수 없다')
        t = time.localtime(os.path.getmtime(p))
        info = zipfile.ZipInfo(f, date_time=tuple(t[:6]) if t.tm_year >= 1980 else (1980, 1, 1, 0, 0, 0))
        info.compress_type = zipfile.ZIP_DEFLATED
        with open(p, "rb") as fh:
          z.writestr(info, fh.read())
      except (OSError, ValueError):
        pass
  return buf.getvalue()


class Handler(BaseHTTPRequestHandler):
  def send(self, code, ctype, data, filename=None):
    self.send_response(code)
    self.send_header("Content-Type", ctype)
    self.send_header("Content-Length", str(len(data)))
    if filename:
      self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
    self.end_headers()
    self.wfile.write(data)

  def do_GET(self):  # noqa: N802
    try:
      self.get()
    except Exception as e:  # noqa: BLE001  (an error otherwise just drops the connection: the phone says nothing useful)
      try:
        self.send(500, "text/plain; charset=utf-8", f"기록을 보내지 못했어요: {type(e).__name__}: {e}".encode())
      except Exception:  # noqa: BLE001
        pass

  def get(self):
    path = unquote(self.path.split("?")[0])
    if path in ("/", "/index.html"):
      self.send(200, "text/html; charset=utf-8", page().encode())
    elif path == "/all.zip":
      name = datetime.datetime.now().strftime("optech_drivelog_%Y%m%d-%H%M.zip")
      self.send(200, "application/zip", zip_all(), name)
    elif path.startswith("/f/"):
      name = path[3:]
      drives, others = log_files()
      if name in drives or name in others:  # only names listed in the folder: no other paths
        with open(os.path.join(LOG_DIR, name), "rb") as f:
          self.send(200, "text/plain; charset=utf-8", f.read(), "optech_" + name)
      else:
        self.send(404, "text/plain; charset=utf-8", "없는 파일이에요.".encode())
    else:
      self.send(404, "text/plain; charset=utf-8", "없는 페이지예요.".encode())

  def log_message(self, *args):
    pass


def serve_for(seconds, should_stop):
  server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
  server.daemon_threads = True
  t = threading.Thread(target=server.serve_forever, daemon=True)
  t.start()
  end = time.monotonic() + seconds
  try:
    while time.monotonic() < end and not should_stop():
      time.sleep(1)
  finally:
    server.shutdown()
    server.server_close()


def main():
  from common.params import Params
  params = Params()
  params.put(PARAM, "0")
  while True:
    if params.get(PARAM, encoding="utf8") == "1":
      try:
        # pressing the button again while serving keeps it at 1: the 15 minutes count from the first press
        serve_for(SERVE_S, lambda: params.get(PARAM, encoding="utf8") != "1")
      except OSError:  # port in use: try again on the next press
        pass
      params.put(PARAM, "0")
    time.sleep(2)


if __name__ == "__main__":
  main()
