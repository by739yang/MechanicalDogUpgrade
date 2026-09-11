# 网页遥控共用：摇杆解析、四足 move、动作按键（web_ctl / web_c 均可 import）

import padog
import mech_arm

JOY_THR_MAX = 6.0
JOY_THR_MIN = -3.0
JOY_DEAD = 10
JOY_TURN_DEAD = 20

# 纯遥控页按键（不含标定 hi/ip/sc 等）
CTL_KEYS = frozenset((
    'g0', 'g1', 'is', 'go', 'gc',
    'btn_stand', 'btn_sit', 'btn_wave', 'btn_crawl', 'btn_stop',
    'am1', 'am0', 'btn_grip_open', 'btn_grip_close',
))


def _leading_int(s):
  s = (s or '').strip()
  if not s:
    return None
  i = 0
  if s[0] in '+-':
    i = 1
  j = i
  while j < len(s) and '0' <= s[j] <= '9':
    j += 1
  if j == i:
    return None
  return int(s[:j])


def _parse_pair(req, k1, k2):
  i = req.find(k1 + '=')
  if i < 0:
    return None, None
  j = req.find(k2 + '=', i + 2)
  if j < 0:
    return None, None
  v1 = _leading_int(req[i + len(k1) + 1 : j])
  v2 = _leading_int(req[j + len(k2) + 1 :])
  if v1 is None or v2 is None:
    return None, None
  return v1, v2


def parse_dog_stick(req):
  return _parse_pair(req, 'f', 't')


def parse_arm_stick(req):
  jy, jx = _parse_pair(req, 'jy', 'jx')
  if jy is None and jx is None:
    return None, None
  if jy is None:
    jy = 0
  if jx is None:
    jx = 0
  return jy, jx


def parse_grip(req):
  i = req.find('grip=')
  if i < 0:
    return None
  return _leading_int(req[i + 5 :])


def _joy_f_to_thr(value_f):
  vf = float(value_f)
  if abs(vf) < JOY_DEAD:
    return 0.0
  thr = vf * JOY_THR_MAX / 100.0
  try:
    thr = thr * float(getattr(padog, 'joy_fwd_sign', 1))
  except Exception:
    pass
  if thr > JOY_THR_MAX:
    return JOY_THR_MAX
  if thr < JOY_THR_MIN:
    return JOY_THR_MIN
  return thr


def _thr_is_backward(thr):
  try:
    if int(getattr(padog, 'joy_fwd_sign', 1)) < 0:
      return float(thr) > 0.35
  except Exception:
    pass
  return float(thr) < -0.35


def _thr_is_forward(thr):
  try:
    if int(getattr(padog, 'joy_fwd_sign', 1)) < 0:
      return float(thr) < -0.35
  except Exception:
    pass
  return float(thr) > 0.35


def _set_joy_turn(t):
  try:
    padog.set_joy_turn(t)
  except AttributeError:
    padog.joy_turn = float(t)


def apply_dog_stick(thr, turn, force=False):
  """force=True：纯遥控页双摇杆，机械臂开启时仍控狗。"""
  if getattr(padog, 'crawl_phase', 0):
    return
  if mech_arm.is_enabled() and not force:
    return
  t = -int(turn)
  if abs(t) < JOY_TURN_DEAD:
    _set_joy_turn(0)
    L, R = 1, 1
  else:
    _set_joy_turn(t)
    L, R = padog._turn_phase_lr(float(t))
  bf = _thr_is_backward(thr)
  ff = _thr_is_forward(thr)
  bt = abs(t) >= JOY_TURN_DEAD
  _ts = float(getattr(padog, 'TURN_DRV_SPD', 2.5))
  _walk = int(getattr(padog, 'gait_mode', 0)) == 1
  _go = padog.drive if _walk else padog.move
  if not bt and not bf and not ff:
    _go(0, L, R)
    return
  if bt:
    _go(_ts, L, R)
    return
  if bf and not bt:
    _go(float(getattr(padog, 'BACK_DRV_SPD', 2.0)), 1, 1)
    return
  _go(thr, L, R)


def process_dog_from_req(req, thr_cache, turn_cache, dog_when_arm=False):
  vf, vt = parse_dog_stick(req)
  if vf is None:
    return thr_cache, turn_cache
  thr = _joy_f_to_thr(vf)
  turn = int(vt)
  _ie = getattr(padog, 'inplace_step_end_ms', 0) or 0
  try:
    import utime
    if _ie and utime.ticks_diff(_ie, utime.ticks_ms()) > 0:
      padog.set_leg_sit_offsets(0, 0)
      padog.move(4, 1, 1)
      return thr, turn
  except Exception:
    pass
  if thr == 0 and abs(turn) < JOY_TURN_DEAD:
    padog.set_leg_sit_offsets(0, 0)
    if not getattr(padog, 'crawl_phase', 0):
      _set_joy_turn(0)
      padog.move(0, 0, 0)
  else:
    padog.set_leg_sit_offsets(0, 0)
    apply_dog_stick(thr, turn, force=dog_when_arm)
  return thr, turn


def process_arm_from_req(req):
  jy, jx = parse_arm_stick(req)
  if jy is None:
    return
  mech_arm.apply_stick(jy, jx)


def process_grip_from_req(req):
  gp = parse_grip(req)
  if gp is None:
    return
  mech_arm.set_grip_pct(gp)


def parse_key(req):
  i = req.find('key=')
  if i < 0:
    return ''
  j = req.find('&', i + 4)
  if j < 0:
    return req[i + 4 :].strip().lower()
  return req[i + 4 : j].strip().lower()


def handle_control_key(value):
  if not value:
    return False
  if value == 'g0':
    padog.stable(False)
    padog.gait(0)
  elif value == 'g1':
    padog.stable(False)
    padog.gait(1)
  elif value == 'is':
    padog.stable(False)
    padog.gait(0)
    try:
      import utime
      padog.inplace_step_end_ms = utime.ticks_add(utime.ticks_ms(), 5000)
    except Exception:
      pass
  elif value == 'btn_stand':
    padog.stable(False)
    padog.action_stand()
  elif value == 'btn_sit':
    padog.stable(False)
    padog.action_sit_direct()
  elif value == 'btn_wave':
    padog.stable(False)
    padog.action_wave_direct()
  elif value == 'btn_crawl':
    padog.stable(False)
    padog.action_crawl()
  elif value == 'btn_stop':
    padog.set_joy_turn(0)
    padog.move(0, 0, 0)
  elif value == 'am1':
    mech_arm.set_enabled(True)
  elif value == 'am0':
    mech_arm.set_enabled(False)
  elif value == 'btn_grip_open':
    mech_arm.grip_open()
  elif value == 'btn_grip_close':
    mech_arm.grip_close()
  elif value == 'go':
    padog.stable(True)
  elif value == 'gc':
    padog.stable(False)
  else:
    return False
  return True
