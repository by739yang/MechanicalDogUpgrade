# 顶装三轴机械臂
# 大臂/小臂：PCA9685 舵机；夹爪：GPIO PWM 舵机（默认 GPIO12，50Hz）

import PA_SERVO
from machine import Pin, PWM

ARM_UPPER_BOARD = 0x40
ARM_FORE_BOARD = 0x40
ARM_GRIP_BOARD = 0x41
ARM_STICK_DEAD = 6

arm_enabled = False
_upper_angle = 145.0
_fore_angle = 125.0
_upper_target = 145.0
_fore_target = 125.0
_grip_pct = 0.0
_grip_pwm = None
_grip_digital_pin = None
_grip_pwm_failed = False
_grip_fail_logged = False


def _clamp(a, lo, hi):
  if a < lo:
    return lo
  if a > hi:
    return hi
  return a


def _cfg(name, default):
  try:
    import padog
    return getattr(padog, name, default)
  except Exception:
    return default


def _cfg_fallback(primary, fallback, default):
  v = _cfg(primary, None)
  if v is not None:
    return v
  return _cfg(fallback, default)


def _board_addr(name, default):
  return int(_cfg(name, default))


def _upper_min():
  return float(_cfg_fallback('arm_upper_min', 'arm_base_min', 0))


def _upper_max():
  return float(_cfg_fallback('arm_upper_max', 'arm_base_max', 180))


def _fore_min():
  return float(_cfg('arm_fore_min', 30))


def _fore_max():
  return float(_cfg('arm_fore_max', 140))


def _upper_center():
  return float(_cfg_fallback('arm_upper_init', 'arm_base_init', 145))


def _fore_center():
  return float(_cfg('arm_fore_init', 125))


def _upper_walk_goal():
  return _clamp(float(_cfg_fallback('arm_upper_walk', 'arm_upper_init', 145)), _upper_min(), _upper_max())


def _fore_walk_goal():
  return _clamp(float(_cfg_fallback('arm_fore_walk', 'arm_fore_init', 125)), _fore_min(), _fore_max())


def _walk_rate():
  return float(_cfg('arm_walk_rate', 0.15))


def _upper_rate():
  return float(_cfg_fallback('arm_upper_rate', 'arm_base_rate', 2.5))


def _fore_rate():
  return float(_cfg('arm_fore_rate', 2.5))


def _approach(current, target, rate):
  c = float(current)
  t = float(target)
  r = abs(float(rate))
  if r <= 0.0:
    return t
  d = t - c
  if abs(d) <= r:
    return t
  if d > 0.0:
    return c + r
  return c - r


def _dog_is_moving():
  try:
    import padog
    if getattr(padog, 'crawl_phase', 0):
      return True
    return abs(float(getattr(padog, 'spd', 0))) >= 0.05
  except Exception:
    return False


def _stick_to_angle(stick, center, vmin, vmax, direction):
  """摇杆 -100~0~+100 → vmin~center~vmax。"""
  c = float(center)
  lo = float(vmin)
  hi = float(vmax)
  half_up = hi - c
  half_dn = c - lo
  if half_up <= 0.0:
    half_up = 90.0
  if half_dn <= 0.0:
    half_dn = 90.0
  s = float(stick)
  d = float(direction)
  if s >= 0.0:
    a = c + d * s * half_up / 100.0
  else:
    a = c + d * s * half_dn / 100.0
  return _clamp(a, lo, hi)


def _grip_open():
  return float(_cfg('arm_grip_open', 90))


def _grip_close():
  return float(_cfg('arm_grip_close', 180))


def _grip_gpio():
  return int(_cfg('arm_grip_gpio', -1))


def _grip_open_level():
  return 1 if int(_cfg('arm_grip_open_level', 0)) else 0


def _grip_close_level():
  return 1 if int(_cfg('arm_grip_close_level', 1)) else 0


def _grip_min_us():
  return int(_cfg('arm_grip_min_us', 500))


def _grip_max_us():
  return int(_cfg('arm_grip_max_us', 2500))


def _grip_pwm_hz():
  return int(_cfg('arm_grip_pwm_hz', 50))


def _grip_use_digital():
  return int(_cfg('arm_grip_digital', 0)) != 0


def _grip_angle_from_pct():
  go = _grip_open()
  gc = _grip_close()
  return go + (float(_grip_pct) / 100.0) * (gc - go)


def _grip_via_pca():
  """arm_grip_gpio=-1 时走 PCA9685（0x41 ch6，与板载文档一致）。"""
  return _grip_gpio() < 0


def _grip_pulse_us(angle_deg):
  lo = _grip_min_us()
  hi = _grip_max_us()
  if hi <= lo:
    hi = lo + 2000
  a = _clamp(float(angle_deg), 0.0, 180.0)
  return int(lo + (hi - lo) * a / 180.0)


def _grip_pwm_write(pwm, angle_deg):
  pulse = _grip_pulse_us(angle_deg)
  try:
    pwm.duty_ns(pulse * 1000)
    return True
  except Exception:
    pass
  try:
    pwm.duty_u16(int(pulse * 65535 / 20000))
    return True
  except Exception:
    pass
  try:
    pwm.duty(int(pulse * 1023 / 20000))
    return True
  except Exception as e:
    print('arm grip pwm write:', e)
  return False


def _write_grip_pca(angle):
  gb = _board_addr('arm_grip_board', ARM_GRIP_BOARD)
  ch = _grip_ch()
  pulse = _grip_pulse_us(angle)
  try:
    if gb == 0x40:
      if PA_SERVO.servos40 is not None:
        PA_SERVO.servos40.position(ch, us=pulse)
        return True
    elif gb == 0x41:
      if PA_SERVO.servos41 is not None:
        PA_SERVO.servos41.position(ch, us=pulse)
        return True
      print('arm grip: PCA9685 0x41 missing, ch', ch)
  except Exception as e:
    print('arm grip pca 0x%x ch%d:' % (gb, ch), e)
  return False


def _write_grip_gpio_pwm(angle):
  pwm = _grip_pwm_obj()
  if pwm is None:
    return False
  return _grip_pwm_write(pwm, angle)


def _write_grip(force=False):
  global _grip_fail_logged
  ang = _grip_angle_from_pct()
  ok = False
  if _grip_via_pca():
    ok = _write_grip_pca(ang)
  if not ok and _use_grip_gpio():
    if _grip_use_digital():
      lv = _grip_close_level() if float(_grip_pct) >= 50.0 else _grip_open_level()
      pin = _grip_digital_pin_obj()
      if pin is not None:
        try:
          pin.value(lv)
          ok = True
        except Exception as e:
          if not _grip_fail_logged:
            print('arm grip digital:', e)
            _grip_fail_logged = True
    else:
      ok = _write_grip_gpio_pwm(ang)
  if ok:
    _grip_fail_logged = False
  elif (force or arm_enabled) and not _grip_fail_logged:
    print('arm grip fail pct=%.0f ang=%.1f gpio=%d pca=%s' % (
        _grip_pct, ang, _grip_gpio(), _grip_via_pca()))
    _grip_fail_logged = True


def _grip_pwm_obj():
  global _grip_pwm, _grip_pwm_failed
  if _grip_pwm_failed:
    return None
  if _grip_pwm is not None:
    return _grip_pwm
  g = _grip_gpio()
  try:
    _grip_pwm = PWM(Pin(g))
    _grip_pwm.freq(_grip_pwm_hz())
    if _grip_pwm_write(_grip_pwm, _grip_angle_from_pct()):
      print('mech_arm grip PWM gpio%d pulse_us=%d ok' % (
          g, _grip_pulse_us(_grip_angle_from_pct())))
    else:
      print('mech_arm grip PWM gpio%d write fail' % g)
      _grip_pwm = None
      _grip_pwm_failed = True
  except Exception as e:
    print('mech_arm grip PWM gpio%d fail:' % g, e)
    _grip_pwm = None
    _grip_pwm_failed = True
  return _grip_pwm


def _grip_digital_pin_obj():
  global _grip_digital_pin
  if _grip_digital_pin is None:
    g = _grip_gpio()
    try:
      _grip_digital_pin = Pin(g, Pin.OUT)
      _grip_digital_pin.value(_grip_open_level())
      print('mech_arm grip digital gpio%d ok' % g)
    except Exception as e:
      print('mech_arm grip digital gpio%d fail:' % g, e)
      _grip_digital_pin = None
  return _grip_digital_pin


def _use_grip_gpio():
  return _grip_gpio() >= 0


def _upper_ch():
  return int(_cfg_fallback('arm_upper_ch', 'arm_base_ch', 6))


def _fore_ch():
  return int(_cfg('arm_fore_ch', 7))


def _grip_ch():
  return int(_cfg('arm_grip_ch', 6))


def _servo_on_board(board_addr, ch, angle):
  a = float(angle)
  ba = int(board_addr)
  ch = int(ch)
  try:
    if ba == 0x40:
      PA_SERVO.servos40.position(ch, a)
    elif ba == 0x41:
      if PA_SERVO.servos41 is None:
        print('arm: PCA9685 0x41 missing, ch', ch)
        return
      PA_SERVO.servos41.position(ch, a)
  except Exception as e:
    print('arm servo 0x%x ch%d:' % (ba, ch), e)


def is_enabled():
  return bool(arm_enabled)


def ensure_enabled():
  """遥控页未点「机械臂开」时，首次控臂/夹爪自动使能。"""
  global arm_enabled, _upper_angle, _fore_angle, _grip_pct
  if arm_enabled:
    return
  arm_enabled = True
  _home_arm_angles()
  _grip_pct = float(_cfg('arm_grip_init', 0))
  if _grip_pct > 100.0:
    _grip_pct = 100.0
  if _grip_pct < 0.0:
    _grip_pct = 0.0
  _write_servos(force=True)
  _write_grip(force=True)
  try:
    import padog
    padog.move(0, 0, 0)
    padog.set_joy_turn(0)
  except Exception:
    pass
  print('mech_arm auto on upper=%.1f fore=%.1f grip=%.0f%%' % (
      _upper_angle, _fore_angle, _grip_pct))


def _home_arm_angles():
  global _upper_angle, _fore_angle, _upper_target, _fore_target
  _upper_angle = _upper_center()
  _fore_angle = _fore_center()
  _upper_target = _upper_angle
  _fore_target = _fore_angle


def set_enabled(on):
  global arm_enabled, _grip_pct
  arm_enabled = bool(on)
  _home_arm_angles()
  _grip_pct = float(_cfg('arm_grip_init', 0))
  if _grip_pct > 100.0:
    _grip_pct = 100.0
  if _grip_pct < 0.0:
    _grip_pct = 0.0
  _write_arm_pose()
  _write_grip(force=True)
  if arm_enabled:
    try:
      import padog
      padog.move(0, 0, 0)
      padog.set_joy_turn(0)
    except Exception:
      pass
  print('mech_arm on=%s upper=%.1f fore=%.1f grip=%.0f%%' % (
      arm_enabled, _upper_angle, _fore_angle, _grip_pct))


def get_angles():
  return _upper_angle, _fore_angle, _grip_pct


def _write_servos(force=False):
  if not arm_enabled and not force and not _dog_is_moving():
    return
  _write_arm_pose()
  _write_grip(force=force)


def _write_arm_pose():
  u = _clamp(_upper_angle, _upper_min(), _upper_max())
  f = _clamp(_fore_angle, _fore_min(), _fore_max())
  ub = _board_addr('arm_upper_board', ARM_UPPER_BOARD)
  fb = _board_addr('arm_fore_board', ARM_FORE_BOARD)
  _servo_on_board(ub, _upper_ch(), u)
  _servo_on_board(fb, _fore_ch(), f)


def _write_arm_only():
  if not arm_enabled and not _dog_is_moving():
    return
  _write_arm_pose()


def apply_stick(stick_y, stick_x):
  """jy→大臂，jx→小臂；摇杆位置=目标角，由 tick 平滑逼近。"""
  global _upper_target, _fore_target
  if not arm_enabled:
    return
  sy = float(stick_y)
  sx = float(stick_x)
  ud = float(_cfg_fallback('arm_upper_dir', 'arm_base_dir', 1))
  fd = float(_cfg('arm_fore_dir', 1))
  uc = _upper_center()
  fc = _fore_center()
  _upper_target = _stick_to_angle(sy, uc, _upper_min(), _upper_max(), ud)
  _fore_target = _stick_to_angle(sx, fc, _fore_min(), _fore_max(), fd)


def _arm_servo_step():
  global _upper_angle, _fore_angle, _upper_target, _fore_target
  if _dog_is_moving():
    ug = _upper_walk_goal()
    fg = _fore_walk_goal()
    rate = _walk_rate()
    _upper_angle = _approach(_upper_angle, ug, rate)
    _fore_angle = _approach(_fore_angle, fg, rate)
    _upper_target = ug
    _fore_target = fg
    _write_arm_pose()
    return
  if not arm_enabled:
    return
  _upper_angle = _approach(_upper_angle, _upper_target, _upper_rate())
  _fore_angle = _approach(_fore_angle, _fore_target, _fore_rate())
  _write_arm_pose()


def grip_open():
  global _grip_pct
  _grip_pct = 0.0
  _write_grip(force=True)


def grip_close():
  global _grip_pct
  _grip_pct = 100.0
  _write_grip(force=True)


def set_grip_pct(pct):
  global _grip_pct, _grip_pwm_failed, _grip_fail_logged, _grip_pwm
  p = float(pct)
  if p < 0.0:
    p = 0.0
  if p > 100.0:
    p = 100.0
  _grip_pct = p
  if _grip_pwm_failed:
    _grip_pwm_failed = False
    _grip_fail_logged = False
    _grip_pwm = None
  _write_grip(force=True)
  go = _grip_open()
  gc = _grip_close()
  print('mech_arm grip pct=%.0f angle=%.1f open=%.0f close=%.0f span=%.0f gpio=%d' % (
      _grip_pct, _grip_angle_from_pct(), go, gc, abs(gc - go), _grip_gpio()))


def grip_pct():
  return float(_grip_pct)


def init_arm_pose():
  """上电写入大/小臂初始化角。"""
  _home_arm_angles()
  _write_arm_pose()


def init_grip():
  """上电初始化夹爪（PCA9685 或 GPIO PWM）。"""
  global _grip_pwm, _grip_pwm_failed, _grip_fail_logged
  _grip_pwm = None
  _grip_pwm_failed = False
  _grip_fail_logged = False
  _write_grip(force=True)
  if _grip_via_pca():
    print('mech_arm grip PCA 0x%x ch%d ok' % (
        _board_addr('arm_grip_board', ARM_GRIP_BOARD), _grip_ch()))


def tick():
  _arm_servo_step()
  _write_grip(force=True)
