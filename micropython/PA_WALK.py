# Copyright Deng（灯哥） Py-apple dog project
# WALK 四足顺序步态：LF(1)→RF(2)→RR(3)→LR(4)，每腿 25% 摆动 + 75% 支撑
# 相位 t 由 padog.mainloop 递增；本模块只算足端 (x,y)，不改 padog.t

from math import sin, cos, pi, tan

faai = 0.5
Ts = 1.0
# 摆动相起点相对步幅终点（与 TROT  xs 语义一致：后撤为正）
_XS_RATIO = 0.35

acc = None
gyro_p = 0
_f_gyro_p = None


def _init_imu():
  global acc, _f_gyro_p
  if acc is not None:
    return
  try:
    import PA_SERVO
    import PA_IMU
    import PA_AVGFILT
    import array
    from machine import I2C, Pin
    try:
      acc = PA_IMU.accel(PA_SERVO._i2c_servo)
    except Exception:
      i2cc = I2C(1, scl=Pin(32), sda=Pin(33), freq=100000)
      acc = PA_IMU.accel(i2cc)
    acc.error_gy()
    gyro_data_p = array.array('i', [0] * 10)
    _f_gyro_p = PA_AVGFILT.avg_filiter(gyro_data_p)
  except Exception as e:
    print("WALK IMU off:", e)
    acc = None
    _f_gyro_p = None


def _body_h():
  try:
    import padog
    return float(padog.R_H)
  except Exception:
    return 110.0


def cal_adjust(CG_Y, l, xk, sita, period):
  bh = _body_h()
  return CG_Y + period * (l + xk) / 4.0 + bh * tan(sita * 1.5)


def _read_gyro_p():
  global gyro_p
  _init_imu()
  if acc is None or _f_gyro_p is None:
    gyro_p = 0.0
    return 0.0
  try:
    import PA_IMU
    ay = acc.get_values()
    q = PA_IMU.IMUupdate(
        ay["GyX"] / 65.5 * 0.0174533,
        ay["GyY"] / 65.5 * 0.0174533,
        ay["GyZ"] / 65.5 * 0.0174533,
        ay["AcX"] / 8192,
        ay["AcY"] / 8192,
        ay["AcZ"] / 8192,
    )
    gyro_p = _f_gyro_p.avg(round(q[1]))
  except Exception:
    gyro_p = 0.0
  return gyro_p


def _cycle_len():
  return 4.0 * float(faai) * float(Ts)


def _swing_len():
  return float(faai) * float(Ts)


def _xs_xf(xf):
  xf = float(xf)
  if xf == 0.0:
    return 0.0, 0.0
  return -_XS_RATIO * xf, xf


def _leg_xy(local_t, xs, xf, h):
  """单腿轨迹：local_t 为减去该腿相移后的时间。"""
  T = _cycle_len()
  swing = _swing_len()
  stance = T - swing
  if stance <= 0.0:
    stance = swing
  phi = local_t % T
  if phi < swing:
    sigma = 2.0 * pi * phi / swing
    y = h * (1.0 - cos(sigma)) / 2.0
    x = (xf - xs) * ((sigma - sin(sigma)) / (2.0 * pi)) + xs
  else:
    u = (phi - swing) / stance
    y = 0.0
    x = xf + (xs - xf) * u
  return x, y


def _swing_leg_index(t):
  """当前处于摆动相的腿 0..3（1→2→3→4）。"""
  swing = _swing_len()
  if swing <= 0.0:
    return 0
  return int(float(t) / swing) % 4


def _apply_cg(CG_X, CG_Y, l, xf, swing_idx):
  """迈腿前重心落在支撑三角内（不阻塞相位推进）。"""
  try:
    import padog
    sita = _read_gyro_p() * pi / 180.0
    if swing_idx == 0:
      yst = cal_adjust(CG_Y, 0, l, sita, -1)
    elif swing_idx == 1:
      yst = cal_adjust(CG_Y, l, 0, sita, -1)
    else:
      yst = cal_adjust(CG_Y, l, xf, sita, 1)
    padog.gesture(0, int(CG_X), int(yst))
  except Exception:
    pass


def cal_w(CG_X, CG_Y, l, xf, h, t, r1, r4, r2, r3):
  """
  WALK 主函数。
  相序 1(LF)→2(RF)→3(RR)→4(LR)，四腿相位差 faai*Ts。
  返回 x1..x4, y1..y4；x 符号与 PA_TROT 一致（取负后乘 r）。
  """
  xs, xf = _xs_xf(xf)
  h = float(h)
  t = float(t)
  swing = _swing_len()
  off1 = 0.0
  off2 = swing
  off3 = 2.0 * swing
  off4 = 3.0 * swing

  _apply_cg(CG_X, CG_Y, l, xf, _swing_leg_index(t))

  x1b, y1 = _leg_xy(t - off1, xs, xf, h)
  x2b, y2 = _leg_xy(t - off2, xs, xf, h)
  x3b, y3 = _leg_xy(t - off3, xs, xf, h)
  x4b, y4 = _leg_xy(t - off4, xs, xf, h)

  x1 = -x1b * float(r1)
  x2 = -x2b * float(r2)
  x3 = -x3b * float(r3)
  x4 = -x4b * float(r4)

  return x1, x2, x3, x4, y1, y2, y3, y4
