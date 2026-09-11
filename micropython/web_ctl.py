# 轻量纯遥控 HTTP 服务：只发 drive.html 一次，摇杆/按键均 204，无大段 HTML

import gc
import socket

import padog
import web_common as wc

_HDR_204 = b'HTTP/1.1 204 No Content\r\nConnection: keep-alive\r\nContent-Length: 0\r\n\r\n'
_HDR_200 = b'HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nConnection: close\r\n\r\n'


def _load_drive_page():
  try:
    with open('drive.html', 'r') as f:
      return f.read()
  except Exception as e:
    print('drive.html missing:', e)
    return '<html><body><h1>drive.html not found</h1></body></html>'


def _wants_page(req_line):
  if req_line.find('f=') >= 0 or req_line.find('key=') >= 0:
    return False
  if req_line.find('jy=') >= 0:
    return False
  if req_line.find('grip=') >= 0:
    return False
  return True


def _reply(cl, body):
  try:
    if body is None:
      cl.sendall(_HDR_204)
    else:
      cl.sendall(_HDR_200)
      cl.sendall(body)
  except Exception:
    pass
  try:
    cl.close()
  except Exception:
    pass


def _handle(cl, raw):
  if not raw:
    _reply(cl, None)
    return
  if isinstance(raw, bytes):
    req = raw.decode('utf-8', 'ignore')
  else:
    req = str(raw)
  req_line = req.split('\n')[0].lower().replace(' ', '')
  for tag in ('get/?', 'http/1.1', 'http/1.0'):
    req_line = req_line.replace(tag, '')
  req_data = req_line
  if req_data.find('favicon.ico') >= 0:
    _reply(cl, None)
    return

  key = wc.parse_key(req_data)
  if key:
    wc.handle_control_key(key)

  wc.process_arm_from_req(req_data)
  wc.process_grip_from_req(req_data)
  if req_data.find('f=') >= 0:
    wc.process_dog_from_req(req_data, 0, 0, dog_when_arm=True)

  if _wants_page(req_data):
    _reply(cl, _PAGE)
  else:
    _reply(cl, None)


_PAGE = _load_drive_page()
gc.collect()

addr = (padog.selfadd, 80)
s = socket.socket()
try:
  s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
except Exception:
  pass
s.bind(addr)
s.listen(8)
print('web_ctl listening on:', addr, '(drive.html)')


while True:
  try:
    cl, _ = s.accept()
  except Exception:
    continue
  try:
    raw = cl.recv(768)
  except Exception:
    raw = b''
  try:
    _handle(cl, raw)
  except Exception:
    _reply(cl, None)
