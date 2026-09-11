#Copyright Deng（灯哥） (ream_d@yeah.net)  Py-apple dog project
#Github:https://github.com/ToanTech/py-apple-quadruped-robot
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at:http://www.apache.org/licenses/LICENSE-2.0

#引入模块
import PA_SERVO
import PA_TROT
import PA_WALK
import PA_IK
import PA_ATTITUDE
#import PA_STABLIZE
import time
from machine import Pin,I2C

import utime

# GPIO21/22 为舵机 PCA9685 与 IMU 共用 I2C，不可占用 22 作 LED（否则 PCA9685 报 ENODEV）
led = Pin(2, Pin.OUT)
led.value(0)        # 上电亮（依板载 LED 极性可改为 value(1)）
#连接网络，启用WEBREPL
selfadd=0
  
def do_connect_STA(essid, password):
    global selfadd
    import network 
    wifi = network.WLAN(network.STA_IF)  
    if not wifi.isconnected(): 
        print('connecting to network...')
        wifi.active(True) 
        wifi.connect(essid, password) 
        while not wifi.isconnected():
            pass 
    print('network config:', wifi.ifconfig())
    selfadd=wifi.ifconfig()[0]

def do_connect_AP():
    global selfadd
    import network 
    wifi = network.WLAN(network.AP_IF)  
    if not wifi.isconnected(): 
        print('connecting to network...')
        wifi.active(True) 
        while not wifi.isconnected():
            pass 
    print('network config:', wifi.ifconfig())
    selfadd=wifi.ifconfig()[0]

exec(open('config.py').read())
exec(open('config_s.py').read())   #加载舵机中位
_g = globals()
for _i in range(1, 5):
    _k = "init_%dp" % _i
    if _k not in _g:
        _g[_k] = 90
for _hk, _hd in (("hip_k_roll", 0.0), ("hip_k_pitch", 0.0), ("hip_k_turn", 0.0), ("hip_delta_max", 18.0),
                 ("in_pit", 0), ("in_rol", 0), ("cal_leg_sel", 1),
                 ("leg_len_ref", 149.0), ("H_goal", 100), ("joy_fwd_sign", 1),
                 ("trot_cg_f", 0.38), ("trot_cg_b", 1.6), ("trot_cg_t", 0),
                 ("shank_ik_bias_per_mm", 0.25), ("shank_ik_bias_deg", 0.0),
                 ("front_leg_y_offset", 0.0), ("rear_leg_y_offset", 0.0),
                 ("leg1_s_trim", 0.0), ("leg2_s_trim", 0.0), ("leg3_s_trim", 0.0), ("leg4_s_trim", 0.0),
                 ("leg2_z_mul", 1.0), ("leg3_z_mul", 1.0), ("leg4_z_mul", 1.0),
                 ("arm_base_init", 145), ("arm_grip_init", 0),
                 ("arm_upper_init", 145), ("arm_fore_init", 125),
                 ("arm_upper_min", 0), ("arm_upper_max", 180),
                 ("arm_fore_min", 30), ("arm_fore_max", 140),
                 ("arm_upper_rate", 5.0), ("arm_fore_rate", 5.0),
                 ("arm_upper_dir", 1), ("arm_fore_dir", 1),
                 ("arm_upper_ch", 6), ("arm_fore_ch", 7), ("arm_grip_ch", 6),
                 ("arm_upper_board", 0x40), ("arm_fore_board", 0x40), ("arm_grip_board", 0x41),
                 ("arm_grip_gpio", -1), ("arm_grip_open_level", 0), ("arm_grip_close_level", 1),
                 ("arm_grip_digital", 0), ("arm_grip_pwm_hz", 50),
                 ("arm_grip_min_us", 500), ("arm_grip_max_us", 2500),
                 ("arm_upper_walk", 145), ("arm_fore_walk", 125), ("arm_walk_rate", 0.15),
                 ("arm_base_min", 0), ("arm_base_max", 180),
                 ("arm_grip_open", 90), ("arm_grip_close", 180),
                 ("arm_base_rate", 1.8), ("arm_base_dir", 1), ("arm_grip_stick_sign", 1),
                 ("walk_faai", 0.30), ("walk_speed_scale", 1.4), ("walk_roll_trim", 3),
                 ("trot_roll_trim", 0), ("trot_right_h_mul", 0.80)):
    if _hk not in _g:
        _g[_hk] = _hd


def _walk_faai():
  try:
    wf = float(walk_faai)
    if wf > 0.05:
      return wf
  except NameError:
    pass
  try:
    return float(faai) * 0.55
  except NameError:
    return 0.30


def _walk_speed_scale():
  try:
    sc = float(walk_speed_scale)
    if sc > 0.05:
      return sc
  except NameError:
    pass
  return 1.0


def _walk_roll_trim():
  try:
    return float(walk_roll_trim)
  except NameError:
    return 0.0


def _sync_walk_pa_timing():
  import PA_WALK
  PA_WALK.faai = _walk_faai()
  try:
    PA_WALK.Ts = float(Ts)
  except NameError:
    PA_WALK.Ts = 1.0


def _sync_pa_step_timing():
  """PA_TROT / PA_WALK 内建 Ts、faai 与 config 一致，否则 cal_t 相位与 padog 的 t 复位错位，步态「空甩不走」。"""
  g = globals()
  import PA_TROT
  if "Ts" in g:
    Ts_ = float(g["Ts"])
    PA_TROT.Ts = Ts_
  if "faai" in g:
    PA_TROT.faai = float(g["faai"])
  _sync_walk_pa_timing()


_sync_pa_step_timing()
led.value(1)        # 网络就绪灭灯（与上电 value(0) 对应）
#=============一些中间或初始变量=============
t=0
init_x=0;init_y=-100
ges_x_1=0;ges_x_2=0;ges_x_3=0;ges_x_4=0
ges_y_1=init_y;ges_y_2=init_y;ges_y_3=init_y;ges_y_4=init_y
PIT_S=0;ROL_S=0;X_S=0
# 上电默认姿态目标：与 config_s 中 in_pit / in_rol / in_y 一致（网页保存后重启仍生效）
PIT_goal=int(in_pit);ROL_goal=int(in_rol);X_goal=int(in_y)
spd=0;L=0;R=0
joy_turn=0
# 原 ESP32 左转/右转：move(2, ±1, ∓1) 的 spd=2 + trot_cg_b → 后退沿用此 spd
BACK_DRV_SPD = 2.0
TURN_DRV_SPD = 2.5
# 横杆转向(%) → 仅髋角差动
HIP_TURN_DEAD = 10
HIP_TURN_STICK_SCALE = 8.0
TURN_HIP_GAIN = 1.0
H_goal=int(H_goal)
R_H=H_goal
init_case=0
key_stab=False;gait_mode=0
try:
  print("padog geom=%.2f back_spd=%s turn=hip trot_cg_b=%s" % (
      _geom_scale(), BACK_DRV_SPD, trot_cg_b))
except Exception:
  pass
stop_run_node=0
crawl_phase=0
crawl_until_ms=0
crawl_settle_until_ms=0
crawl_saved_h=int(H_goal)
CRAWL_DURATION_MS=5000
CRAWL_SETTLE_MS=400
CRAWL_SHANK_FRONT=20
CRAWL_SHANK_REAR=30
CRAWL_FWD_SPD=-3.5
# 网页「步态测试」结束时刻(ticks_ms)；未到期时每帧 move(4,1,1) 与前进同 TROT
inplace_step_end_ms = 0
direct_pose_freeze = False
pose_anim_active = False
pose_anim_from = None
pose_anim_to = None
pose_anim_start_ms = 0
pose_anim_end_ms = 0
pose_anim_hold_freeze = False
# 前后腿偏置由 config_s 提供（勿在此写死 0，否则会覆盖 rear_leg_y_offset 等）

def _geom_scale():
  """腿长相对灯哥小机(80+69)的比例；站立/行走共用，使 IK 足端目标与小机同一套参数语义。"""
  ref = float(leg_len_ref)
  if ref < 1.0:
    ref = 149.0
  return (float(l1) + float(l2)) / ref


def _ik_hc(r_h):
  """cal_ges 站高：R_H 与网页滑条一致；腿长超出小机参考时加 mm 偏移（勿乘 geom_scale，否则调高反而更矮）。"""
  ref = float(leg_len_ref)
  if ref < 1.0:
    ref = 149.0
  extra_len = (float(l1) + float(l2)) - ref
  if extra_len < 0.0:
    extra_len = 0.0
  return float(r_h) + extra_len


def _scale_mm(d):
  return float(d) * _geom_scale()


def _partial_geom_scale(frac):
  """步幅/重心勿全乘腿长比(1.43)，否则大狗水平命令过大→缓慢滑步。"""
  gs = _geom_scale()
  f = float(frac)
  if f < 0.0:
    f = 0.0
  if f > 1.0:
    f = 1.0
  return 1.0 + (gs - 1.0) * f


def _joy_forward_motion():
  """摇杆前推为正向行走时 True（与 joy_fwd_sign 一致，勿用 spd 正负代替）。"""
  try:
    jfs = int(joy_fwd_sign)
  except NameError:
    jfs = 1
  if jfs < 0:
    return float(spd) < 0
  return float(spd) > 0


def _joy_backward_motion():
  """摇杆后拉为后退时 True（与 joy_fwd_sign 一致）。"""
  try:
    jfs = int(joy_fwd_sign)
  except NameError:
    jfs = 1
  if jfs < 0:
    return float(spd) > 0
  return float(spd) < 0


# 抗滑步：大狗步幅仍略保守；TROT 步频/摆动相可配 config speed/faai
_LARGE_STRIDE_XF_MUL = 0.90
_LARGE_STRIDE_GEOM_FRAC = 0.52
_LARGE_STRIDE_XS_RATIO = 0.0
_LARGE_CG_GEOM_FRAC = 0.35
_LARGE_BWD_CG_MUL = 0.50
_LARGE_H_TROT_MUL = 0.96


def _trot_right_h_mul():
  try:
    return float(trot_right_h_mul)
  except NameError:
    return 0.88


def _apply_trot_swing_y(p_):
  """右侧 RF/RR 抬腿高度略降，减轻左高右低。"""
  rm = _trot_right_h_mul()
  if rm >= 0.999:
    return p_
  y1 = float(p_[4])
  y2 = float(p_[5])
  y3 = float(p_[6])
  y4 = float(p_[7])
  if y2 > 0.05:
    y2 *= rm
  if y3 > 0.05:
    y3 *= rm
  return (p_[0], p_[1], p_[2], p_[3], y1, y2, y3, y4)


def _walk_phase_step():
  """WALK 相位增量。"""
  try:
    sp = float(speed)
    ts = float(Ts)
    fa = _walk_faai()
  except Exception:
    sp, ts, fa = 0.045, 1.0, 0.30
  denom = ts - sp
  if denom < 0.02:
    denom = ts if ts > 0.02 else 1.0
  auto = sp * (4.0 * fa * ts) / denom * _walk_speed_scale()
  try:
    ws = float(walk_speed)
    if ws > 0.001:
      return ws
  except NameError:
    pass
  return auto


def _walk_rol_s():
  """直行 WALK 时滚转微调，抑制左/右偏。"""
  rs = float(ROL_S)
  try:
    jt = float(joy_turn)
  except NameError:
    jt = 0.0
  if abs(jt) >= HIP_TURN_DEAD:
    return rs
  if abs(float(spd)) < 0.05:
    return rs
  return rs + _walk_roll_trim()


def _trot_roll_trim():
  try:
    return float(trot_roll_trim)
  except NameError:
    return 0.0


def _trot_rol_s():
  """直行 TROT 时滚转微调，抑制左/右偏。"""
  rs = float(ROL_S)
  try:
    jt = float(joy_turn)
  except NameError:
    jt = 0.0
  if abs(jt) >= HIP_TURN_DEAD:
    return rs
  if abs(float(spd)) < 0.05:
    return rs
  return rs + _trot_roll_trim()


def _shank_ik_bias():
  """大狗同 Hc 下 IK 小腿角比小机低约 20°+；进 cal_test_shank 前加偏置，使站立/摆动与小机舵机语义一致。"""
  try:
    fixed = float(shank_ik_bias_deg)
    if fixed > 0.0:
      return fixed
  except NameError:
    pass
  per_mm = 0.375
  try:
    per_mm = float(shank_ik_bias_per_mm)
  except NameError:
    pass
  ref = float(leg_len_ref)
  if ref < 1.0:
    ref = 149.0
  extra = (float(l1) + float(l2)) - ref
  if extra < 0.0:
    extra = 0.0
  return extra * per_mm


def _leg_cfg(name, leg_n, default=0.0):
  try:
    return float(globals()["leg%d_%s" % (leg_n, name)])
  except (KeyError, NameError, ValueError):
    return float(default)


def cal_test_shank(x, leg_trim=0.0):
  # 灯哥小机二次拟合；大狗始终走同一曲线（勿混回 raw IK，否则小腿舵机偏低、整体偏矮）
  x = float(x) + _shank_ik_bias() + float(leg_trim)
  p1, p2, p3 = 0.006649, 0.4414, 5.53
  return p1 * x * x + p2 * x + p3


def _foot_y_targets(p_y1, p_y2, p_y3, p_y4):
  """足端竖直目标：仅前后偏置，四腿一致（勿对单腿 z_mul 硬阈值，易抖动）。"""
  try:
    fy = float(front_leg_y_offset)
  except NameError:
    fy = 0.0
  try:
    ry = float(rear_leg_y_offset)
  except NameError:
    ry = 0.0
  y1 = float(p_y1) + fy
  y2 = float(p_y2) + fy
  y3 = float(p_y3) + ry
  y4 = float(p_y4) + ry
  return y1, y2, y3, y4


def _clamp_deg(a):
  if a > 180:
    return 180
  if a < 0:
    return 0
  return a


def _trot_turn_lr():
  """转向只用 L/R 相位（与 example 一致）；纯后退无转向时才关足端差动。"""
  try:
    jt = float(joy_turn)
  except NameError:
    jt = 0.0
  if abs(jt) >= HIP_TURN_DEAD:
    return 1.0, 1.0, 1.0, 1.0
  try:
    if _joy_backward_motion() and abs(float(spd)) > 0.35:
      return 1.0, 1.0, 1.0, 1.0
  except NameError:
    pass
  return 1.0, 1.0, 1.0, 1.0


def _turn_phase_lr(jt):
  """与 example/web 旧版一致：jt>0 左转 L=-1,R=1；jt<0 右转 L=1,R=-1（勿混足端系数）。"""
  jf = float(jt)
  if jf > float(HIP_TURN_DEAD):
    return -1, 1
  if jf < -float(HIP_TURN_DEAD):
    return 1, -1
  return 1, 1


def _hip_leg_deltas():
  # 髋辅助偏航：jt>0 左转取反，jt<0 右转同向；步态相位由 _turn_phase_lr 承担
  global ROL_S, PIT_S, joy_turn, hip_k_roll, hip_k_pitch, hip_k_turn, hip_delta_max
  r = float(ROL_S)
  p = float(PIT_S)
  turn_mag = 0.0
  jt = 0.0
  try:
    jt = float(joy_turn)
  except NameError:
    jt = 0.0
  if abs(jt) >= HIP_TURN_DEAD:
    turn_mag = (abs(jt) / float(HIP_TURN_STICK_SCALE)) * float(TURN_HIP_GAIN)
    if jt > 0:
      turn_mag = -turn_mag
  kr = float(hip_k_roll)
  kp = float(hip_k_pitch)
  kt = float(hip_k_turn)
  dmx = float(hip_delta_max)
  d1 = kr * r + kp * p + kt * turn_mag
  d2 = -kr * r + kp * p - kt * turn_mag
  d3 = -kr * r - kp * p - kt * turn_mag
  d4 = kr * r - kp * p + kt * turn_mag

  def _clip(x):
    if x > dmx:
      return dmx
    if x < -dmx:
      return -dmx
    return x

  return _clip(d1), _clip(d2), _clip(d3), _clip(d4)


def _crawl_active():
  try:
    return int(crawl_phase) != 0
  except NameError:
    return False


def _crawl_shank_servodelta(leg_n):
  """爬行压低：按 IK 公式分两类（与 servo_output 中 ±cal_test_shank 一致）。"""
  if not _crawl_active():
    return 0.0
  n = int(leg_n)
  if n in (1, 4):
    return -float(CRAWL_SHANK_FRONT if n == 1 else CRAWL_SHANK_REAR)
  return float(CRAWL_SHANK_FRONT if n == 2 else CRAWL_SHANK_REAR)


def _crawl_finish():
  global crawl_phase, crawl_until_ms, crawl_settle_until_ms, R_H
  crawl_phase = 0
  crawl_until_ms = 0
  crawl_settle_until_ms = 0
  move(0, 0, 0)
  gait(0)
  height(int(crawl_saved_h))
  R_H = int(crawl_saved_h)
  gesture(int(in_pit), int(in_rol), int(in_y))
  set_leg_sit_offsets(0, 0)


def _crawl_mainloop_service():
  """主循环驱动爬行：蹲低→前进5s→恢复；每帧重发前进，避免网页摇杆 move(0) 打断。"""
  global crawl_phase
  if not _crawl_active():
    return
  now = utime.ticks_ms()
  if int(crawl_phase) == 1:
    move(0, 0, 0)
    if utime.ticks_diff(crawl_settle_until_ms, now) <= 0:
      crawl_phase = 2
      gait(0)
      move(float(CRAWL_FWD_SPD), 1, 1)
  elif int(crawl_phase) == 2:
    gait(0)
    move(float(CRAWL_FWD_SPD), 1, 1)
    if utime.ticks_diff(crawl_until_ms, now) <= 0:
      _crawl_finish()


def servo_output(case,init,ham1,ham2,ham3,ham4,shank1,shank2,shank3,shank4):
  # 逻辑腿：1左前 2右前 3右后 4左后；硬件见 PA_SERVO 逻辑通道 0-11（双 PCA9685）
  h1, h2, h3, h4 = _hip_leg_deltas()
  cs1 = _crawl_shank_servodelta(1)
  cs2 = _crawl_shank_servodelta(2)
  cs3 = _crawl_shank_servodelta(3)
  cs4 = _crawl_shank_servodelta(4)
  if case==0 and init==0:
    PA_SERVO.angle(0, _clamp_deg(init_1p + h1))
    PA_SERVO.angle(1, init_1h+90-ham1)
    PA_SERVO.angle(2, _clamp_deg((init_1s-90)+cal_test_shank(shank1, _leg_cfg("s_trim", 1)) + cs1))
    PA_SERVO.angle(6, _clamp_deg(init_2p + h2))
    PA_SERVO.angle(7, init_2h-90+ham2)
    PA_SERVO.angle(8, _clamp_deg((init_2s+90)-cal_test_shank(shank2, _leg_cfg("s_trim", 2)) + cs2))
    PA_SERVO.angle(9, _clamp_deg(init_3p + h3))
    PA_SERVO.angle(10, init_3h-90+ham3)
    PA_SERVO.angle(11, _clamp_deg((init_3s+90)-cal_test_shank(shank3, _leg_cfg("s_trim", 3)) + cs3))
    PA_SERVO.angle(3, _clamp_deg(init_4p + h4))
    PA_SERVO.angle(4, init_4h+90-ham4)
    PA_SERVO.angle(5, _clamp_deg((init_4s-90)+cal_test_shank(shank4, _leg_cfg("s_trim", 4)) + cs4))
  else:
    PA_SERVO.angle(0, _clamp_deg(init_1p + h1))
    PA_SERVO.angle(1, init_1h)
    PA_SERVO.angle(2, init_1s)
    PA_SERVO.angle(6, _clamp_deg(init_2p + h2))
    PA_SERVO.angle(7, init_2h)
    PA_SERVO.angle(8, init_2s)
    PA_SERVO.angle(9, _clamp_deg(init_3p + h3))
    PA_SERVO.angle(10, init_3h)
    PA_SERVO.angle(11, init_3s)
    PA_SERVO.angle(3, _clamp_deg(init_4p + h4))
    PA_SERVO.angle(4, init_4h)
    PA_SERVO.angle(5, init_4s)



sitaa=0
def show_circle(times):
  global sitaa,stop_run_node
  for i in range(times):
    stop_run_node=1
    while True:
      h=50
      R=15
      PIT_C=atan(R*sin(sitaa)/h)*180/pi
      ROL_C=atan(R*cos(sitaa)/h)*180/pi
      if sitaa>=3.14*2:
        sitaa=0
        break
      else:
        sitaa=sitaa+0.1
      P_G=PA_ATTITUDE.cal_ges(PIT_C,ROL_C,l,b,w,X_S,_ik_hc(R_H))
      ges_x_1=P_G[0];ges_x_2=P_G[1]; ges_x_3=P_G[2]; ges_x_4=P_G[3];ges_y_1=P_G[4];ges_y_2=P_G[5]; ges_y_3=P_G[6]; ges_y_4=P_G[7]
      A_=PA_IK.ik(ma_case,l1,l2,ges_x_1,ges_x_2,ges_x_3,ges_x_4,ges_y_1,ges_y_2,ges_y_3,ges_y_4)
      servo_output(ma_case,init_case,A_[0],A_[1],A_[2],A_[3],A_[4],A_[5],A_[6],A_[7])
  stop_run_node=0


def height(goal):    #高度调节函数
    global H_goal, R_H
    H_goal = goal
    # cal_ges 用的是 R_H；原先用 Kp_H 每帧逼近 H_goal，网页滑条会明显“拖很久”。目标高度直接同步到 R_H。
    R_H = goal

def gesture(PIT,ROL,X):
    global PIT_goal,ROL_goal,X_goal
    PIT_goal=PIT
    ROL_goal=ROL
    X_goal=X

#快速调节函数（适用于串口）
def g(PIT):
    global PIT_goal
    PIT_goal=PIT
    
def m(spd_,L_,R_):
    global spd,L,R
    spd=spd_;L=L_;R=R_
    if (L_ + R_) != 0 and abs(spd_) > 0:
      gait(0)
#快速调节函数（适用于串口）


def set_joy_turn(turn_pct):
    global joy_turn
    joy_turn = float(turn_pct)


def move_back(spd_=None):
    """后退 = 原 L/R 指令的 spd=2、L=R=1、trot_cg_b，无左右差动。"""
    if spd_ is None:
        spd_ = BACK_DRV_SPD
    set_joy_turn(0)
    gait(0)
    move(float(spd_), 1, 1)


def move_forward(spd_=-5):
    set_joy_turn(0)
    gait(0)
    move(float(spd_), 1, 1)


def turn_hip(turn_pct, spd_=None):
    """转向：L/R 与 example 相同；jt>0 左 / jt<0 右（CAM L=+100 / R=-100）。"""
    if spd_ is None:
        spd_ = TURN_DRV_SPD
    set_joy_turn(turn_pct)
    L, R = _turn_phase_lr(float(turn_pct))
    gait(0)
    move(float(spd_), L, R)


def drive(spd_, L_, R_):
    """仅更新 spd/L/R（WALK 摇杆用，不切 gait_mode）。"""
    global spd, L, R, direct_pose_freeze, init_case, inplace_step_end_ms
    spd = float(spd_)
    L = L_
    R = R_
    if (L_ + R_) != 0 and abs(spd_) > 0:
      servo_init(0)
      direct_pose_freeze = False
      inplace_step_end_ms = 0


def move(spd_,L_,R_):
    global spd,L,R,direct_pose_freeze,init_case,inplace_step_end_ms
    spd=float(spd_);L=L_;R=R_
    if (L_ + R_) != 0 and abs(spd_) > 0:
      gait(0)
      servo_init(0)
      direct_pose_freeze = False
      inplace_step_end_ms = 0
  
def stable(key):
    global key_stab
    key_stab=key
  
def servo_init(key):
    global init_case
    init_case=key
    
def gait(mode):   #设置步态
    global gait_mode, t, X_goal, PIT_goal, ROL_goal
    nm = int(mode)
    if nm != int(gait_mode):
        t = 0
    if nm == 0:
        PIT_goal = int(in_pit)
        ROL_goal = int(in_rol)
        X_goal = int(in_y)
    gait_mode = nm
  


def _wait_with_control(ms):
    end_time = utime.ticks_add(utime.ticks_ms(), ms)
    while utime.ticks_diff(end_time, utime.ticks_ms()) > 0:
        mainloop()
        time.sleep_ms(20)

def set_leg_sit_offsets(front_y, rear_y):
    global front_leg_y_offset, rear_leg_y_offset
    front_leg_y_offset = front_y
    rear_leg_y_offset = rear_y


def _apply_stand_angles_direct():
    """标定站立：12 路直接等于 init_*，不叠加髋姿态耦合（避免「站立变矮」）。"""
    PA_SERVO.angle(0, _clamp_deg(init_1p))
    PA_SERVO.angle(1, init_1h)
    PA_SERVO.angle(2, init_1s)
    PA_SERVO.angle(6, _clamp_deg(init_2p))
    PA_SERVO.angle(7, init_2h)
    PA_SERVO.angle(8, init_2s)
    PA_SERVO.angle(9, _clamp_deg(init_3p))
    PA_SERVO.angle(10, init_3h)
    PA_SERVO.angle(11, init_3s)
    PA_SERVO.angle(3, _clamp_deg(init_4p))
    PA_SERVO.angle(4, init_4h)
    PA_SERVO.angle(5, init_4s)


def _apply_sit_angles_direct():
    # 坐下：前腿抬高、后腿下沉（纯舵机）；数值可按实机微调
    _apply_pose_blend(_stand_pose(), _sit_pose(), 1.0)


def _pose_blend_ms():
  try:
    return int(pose_blend_ms)
  except Exception:
    return 900


def _stand_pose():
  return (
    (0, float(init_1p)), (1, float(init_1h)), (2, float(init_1s)),
    (3, float(init_4p)), (4, float(init_4h)), (5, float(init_4s)),
    (6, float(init_2p)), (7, float(init_2h)), (8, float(init_2s)),
    (9, float(init_3p)), (10, float(init_3h)), (11, float(init_3s)),
  )


def _sit_pose():
  return (
    (0, float(init_1p - 4)), (1, float(init_1h - 16)), (2, float(init_1s - 12)),
    (3, float(init_4p)), (4, float(init_4h - 28)), (5, float(init_4s + 24)),
    (6, float(init_2p + 4)), (7, float(init_2h + 16)), (8, float(init_2s + 12)),
    (9, float(init_3p)), (10, float(init_3h + 28)), (11, float(init_3s - 24)),
  )


def _apply_pose_blend(p0, p1, t):
  if t < 0.0:
    t = 0.0
  if t > 1.0:
    t = 1.0
  for i in range(12):
    pin = p0[i][0]
    a = p0[i][1] + (p1[i][1] - p0[i][1]) * t
    PA_SERVO.angle(pin, _clamp_deg(a))


def _pose_anim_begin(p_from, p_to, hold_freeze):
  global pose_anim_active, pose_anim_from, pose_anim_to
  global pose_anim_start_ms, pose_anim_end_ms, pose_anim_hold_freeze, direct_pose_freeze
  global crawl_phase, crawl_until_ms, crawl_settle_until_ms, inplace_step_end_ms
  crawl_phase = 0
  crawl_until_ms = 0
  crawl_settle_until_ms = 0
  inplace_step_end_ms = 0
  move(0, 0, 0)
  gait(0)
  pose_anim_from = p_from
  pose_anim_to = p_to
  pose_anim_start_ms = utime.ticks_ms()
  pose_anim_end_ms = utime.ticks_add(pose_anim_start_ms, _pose_blend_ms())
  pose_anim_hold_freeze = bool(hold_freeze)
  pose_anim_active = True
  direct_pose_freeze = True


def _pose_anim_step():
  global pose_anim_active, direct_pose_freeze
  if not pose_anim_active:
    return False
  now = utime.ticks_ms()
  total = utime.ticks_diff(pose_anim_end_ms, pose_anim_start_ms)
  if total <= 0:
    total = 1
  elapsed = utime.ticks_diff(now, pose_anim_start_ms)
  t = float(elapsed) / float(total)
  if t >= 1.0:
    t = 1.0
    _apply_pose_blend(pose_anim_from, pose_anim_to, t)
    pose_anim_active = False
    direct_pose_freeze = pose_anim_hold_freeze
  else:
    _apply_pose_blend(pose_anim_from, pose_anim_to, t)
  return True


def _wait_pose_anim_done():
    while pose_anim_active:
        mainloop()
        time.sleep_ms(20)


def action_stand():
    global direct_pose_freeze, crawl_phase, crawl_until_ms, crawl_settle_until_ms
    if pose_anim_active:
        return
    crawl_phase = 0
    crawl_until_ms = 0
    crawl_settle_until_ms = 0
    move(0, 0, 0)
    gait(0)
    height(int(H_goal))
    gesture(0, 0, in_y)
    set_leg_sit_offsets(0, 0)
    if direct_pose_freeze:
        _pose_anim_begin(_sit_pose(), _stand_pose(), False)
    else:
        direct_pose_freeze = False
        _apply_stand_angles_direct()


def action_sit_direct():
    global direct_pose_freeze, crawl_phase, crawl_until_ms, crawl_settle_until_ms, inplace_step_end_ms
    if pose_anim_active:
        return
    if direct_pose_freeze:
        return
    crawl_phase = 0
    crawl_until_ms = 0
    crawl_settle_until_ms = 0
    inplace_step_end_ms = 0
    move(0, 0, 0)
    gait(0)
    set_leg_sit_offsets(0, 0)
    height(86)
    gesture(0, 0, in_y)
    _pose_anim_begin(_stand_pose(), _sit_pose(), True)


def action_sit():
    action_sit_direct()


def action_crawl():
    global crawl_phase, crawl_until_ms, crawl_settle_until_ms
    global direct_pose_freeze, inplace_step_end_ms, crawl_saved_h
    global pose_anim_active
    global R_H, PIT_goal, ROL_goal, X_goal
    pose_anim_active = False
    direct_pose_freeze = False
    inplace_step_end_ms = 0
    set_joy_turn(0)
    move(0, 0, 0)
    gait(0)
    servo_init(0)
    set_leg_sit_offsets(0, 0)
    crawl_saved_h = int(H_goal)
    R_H = crawl_saved_h
    PIT_goal = int(in_pit)
    ROL_goal = int(in_rol)
    X_goal = int(in_y)
    now = utime.ticks_ms()
    crawl_settle_until_ms = utime.ticks_add(now, CRAWL_SETTLE_MS)
    crawl_until_ms = utime.ticks_add(now, CRAWL_SETTLE_MS + CRAWL_DURATION_MS)
    crawl_phase = 1


def action_wave_direct():
    # 先坐下(纯舵机) → 双前腿再抬高 → 左前挥手 → 标定站立
    action_sit_direct()
    _wait_pose_anim_done()
    PA_SERVO.angle(1, _clamp_deg(init_1h - 38))
    PA_SERVO.angle(2, _clamp_deg(init_1s - 38))
    PA_SERVO.angle(7, _clamp_deg(init_2h + 38))
    PA_SERVO.angle(8, _clamp_deg(init_2s + 38))
    time.sleep_ms(420)
    lift_h = _clamp_deg(init_1h - 34)
    lift_s = _clamp_deg(init_1s - 42)
    PA_SERVO.angle(0, _clamp_deg(init_1p))
    PA_SERVO.angle(1, lift_h)
    PA_SERVO.angle(2, lift_s)
    time.sleep_ms(300)
    for _ in range(3):
        PA_SERVO.angle(1, min(180, lift_h + 20))
        time.sleep_ms(260)
        PA_SERVO.angle(1, max(0, lift_h - 12))
        time.sleep_ms(260)
    time.sleep_ms(200)
    action_stand()


def action_wave():
    action_wave_direct()


def mainloop():
    global t
    global R_H
    global PIT_S,ROL_S,X_S
    global ges_x_1,ges_x_2,ges_x_3,ges_x_4
    global ges_y_1,ges_y_2,ges_y_3,ges_y_4
    global next_blink_time
    global crawl_until_ms
    global crawl_phase
    global inplace_step_end_ms
    global direct_pose_freeze
    global pose_anim_active
    #锁定
    if stop_run_node==1:
      return 0
    _crawl_mainloop_service()
    if not _crawl_active():
      crawl_until_ms = 0
    # 步态测试：原地 TROT；四腿一致，不再单独压低后腿
    if inplace_step_end_ms:
      if utime.ticks_diff(inplace_step_end_ms, utime.ticks_ms()) > 0:
        set_leg_sit_offsets(0, 0)
        gait(0)
        move(3, 1, 1)
      else:
        inplace_step_end_ms = 0
        set_leg_sit_offsets(0, 0)
    if _pose_anim_step():
      try:
        import mech_arm
        mech_arm.tick()
      except Exception:
        pass
      return 0
    if direct_pose_freeze:
      try:
        import mech_arm
        mech_arm.tick()
      except Exception:
        pass
      return 0
    _gs = _geom_scale()
    #判断步态模式
    if gait_mode==0:
        if L == 0 and R == 0:
            t = 0
        elif spd != 0:
            t = t + speed
            if t >= Ts:
                t = t - Ts
        _h_trot = float(h) * _LARGE_H_TROT_MUL
        if (L + R) != 0 and spd != 0:
          s = abs(float(spd))
          _h_trot *= max(0.62, min(0.92, s / 5.5))
          if not _joy_forward_motion():
            _h_trot *= 0.82
        _xgs = _partial_geom_scale(_LARGE_STRIDE_GEOM_FRAC)
        _xf = float(spd) * 10.0 * _LARGE_STRIDE_XF_MUL * _xgs
        _xs = 0.0
        if spd != 0:
          _xs = -_LARGE_STRIDE_XS_RATIO * _xf
        _lr1, _lr4, _lr2, _lr3 = _trot_turn_lr()
        P_ = PA_TROT.cal_t(t, _xs, _xf, _h_trot, L * _lr1, L * _lr4, R * _lr2, R * _lr3)
        P_ = _apply_trot_swing_y(P_)
    elif gait_mode==1:
        import PA_WALK
        _wf = _walk_faai()
        PA_WALK.faai = _wf
        PA_WALK.Ts = float(Ts)
        _walk_cycle = 4.0 * _wf * float(Ts)
        if t >= _walk_cycle:
            t = 0
        elif L == 0 and R == 0:
            t = 0
        elif spd != 0:
            t = t + _walk_phase_step()
        _xgs = _partial_geom_scale(_LARGE_STRIDE_GEOM_FRAC)
        _wxf = float(spd) * 10.0 * _LARGE_STRIDE_XF_MUL * _xgs
        _h_walk = float(h) * _LARGE_H_TROT_MUL
        if (L + R) != 0 and spd != 0:
          s = abs(float(spd))
          _h_walk *= max(0.62, min(0.92, s / 5.5))
          if not _joy_forward_motion():
            _h_walk *= 0.82
        _lr1, _lr4, _lr2, _lr3 = _trot_turn_lr()
        _wr1 = float(L) * _lr1
        _wr2 = float(R) * _lr2
        _wr3 = float(R) * _lr3
        _wr4 = float(L) * _lr4
        P_ = PA_WALK.cal_w(CG_X, CG_Y, l, _wxf, _h_walk, t, _wr1, _wr4, _wr2, _wr3)

    #高度调节器1
    if R_H>H_goal:
        R_H=R_H-abs(R_H-H_goal)*Kp_H
    elif R_H<H_goal:
        R_H=R_H+abs(R_H-H_goal)*Kp_H
    #姿态调节器
    if PIT_S>PIT_goal:   #俯仰
        PIT_S=PIT_S-abs(PIT_S-PIT_goal)*Kp_G
    elif PIT_S<PIT_goal:
        PIT_S=PIT_S+abs(PIT_S-PIT_goal)*Kp_G

    if ROL_S>ROL_goal:   #滚转
        ROL_S=ROL_S-abs(ROL_S-ROL_goal)*Kp_G
    elif ROL_S<ROL_goal:
        ROL_S=ROL_S+abs(ROL_S-ROL_goal)*Kp_G
          
    if X_S>X_goal:   #X位置
        X_S=X_S-abs(X_S-X_goal)*Kp_G
    elif X_S<X_goal:
        X_S=X_S+abs(X_S-X_goal)*Kp_G
    #姿态角度限位  
    if PIT_S>=pit_max_ang:PIT_S=pit_max_ang
    if PIT_S<=-pit_max_ang:PIT_S=-pit_max_ang
    if ROL_S>=rol_max_ang:ROL_S=rol_max_ang
    if ROL_S<=-rol_max_ang:ROL_S=-rol_max_ang
    #TROT模态根据迈腿长度自动调节重心
    _hc = _ik_hc(R_H)
    _cgk = _partial_geom_scale(_LARGE_CG_GEOM_FRAC)
    _tr = _trot_rol_s()
    if gait_mode==0:
        if (L+R)!=0 and _joy_forward_motion():
            P_G=PA_ATTITUDE.cal_ges(PIT_S,_tr,l,b,w,X_S-abs(float(spd))*float(trot_cg_f)*_cgk,_hc)
        elif _joy_backward_motion() and abs(float(spd))>0:
            # 后拉 L=R=1 与原地转 thr>0 均走后退重心（勿用 trot_cg_t=0 那套）
            P_G=PA_ATTITUDE.cal_ges(PIT_S,_tr,l,b,w,X_S+abs(float(spd))*float(trot_cg_b)*_cgk*_LARGE_BWD_CG_MUL,_hc)
        elif (L+R)==0 and abs(float(spd))>0:
            P_G=PA_ATTITUDE.cal_ges(PIT_S,_tr,l,b,w,X_S+abs(float(spd))*float(trot_cg_t)*_cgk,_hc)
        elif (L+R)!=0:
            P_G=PA_ATTITUDE.cal_ges(PIT_S,_tr,l,b,w,X_S+abs(float(spd))*float(trot_cg_b)*_cgk*_LARGE_BWD_CG_MUL,_hc)
        else:
            P_G=PA_ATTITUDE.cal_ges(PIT_S,_tr,l,b,w,X_S,_hc)
    elif gait_mode==1:
        _wr = _walk_rol_s()
        if (L+R)!=0 and abs(float(spd))>0 and _joy_forward_motion():
            P_G=PA_ATTITUDE.cal_ges(PIT_S,_wr,l,b,w,X_S-abs(float(spd))*float(trot_cg_f)*_cgk*0.65,_hc)
        elif _joy_backward_motion() and abs(float(spd))>0:
            P_G=PA_ATTITUDE.cal_ges(PIT_S,_wr,l,b,w,X_S+abs(float(spd))*float(trot_cg_b)*_cgk*_LARGE_BWD_CG_MUL*0.65,_hc)
        else:
            P_G=PA_ATTITUDE.cal_ges(PIT_S,_wr,l,b,w,X_S,_hc)
    else:
        P_G=PA_ATTITUDE.cal_ges(PIT_S,ROL_S,l,b,w,X_S,_hc)
    ges_x_1=P_G[0];ges_x_2=P_G[1]; ges_x_3=P_G[2]; ges_x_4=P_G[3]
    ges_y_1=P_G[4];ges_y_2=P_G[5]; ges_y_3=P_G[6]; ges_y_4=P_G[7]
    #自稳调节器（只静态稳定）:
    if spd==0 and L==0 and R==0 and key_stab==True:
        #PA_STABLIZE.stab()
        pass
    #作动
    fy1, fy2, fy3, fy4 = _foot_y_targets(P_[4], P_[5], P_[6], P_[7])
    A_=PA_IK.ik(ma_case,l1,l2,P_[0]+ges_x_1,P_[1]+ges_x_2,P_[2]+ges_x_3,P_[3]+ges_x_4,fy1+ges_y_1,fy2+ges_y_2,fy3+ges_y_3,fy4+ges_y_4)
    servo_output(ma_case,init_case,A_[0],A_[1],A_[2],A_[3],A_[4],A_[5],A_[6],A_[7])
    try:
      import mech_arm
      mech_arm.tick()
    except Exception:
      pass
    
    
    










