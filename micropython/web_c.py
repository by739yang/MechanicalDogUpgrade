import padog
import socket
import utime
import gc

# 标定页按键：此类请求不执行底部摇杆 move，避免 thr 解析错误或未归零时干扰舵机微调
_CAL_NO_MOVE_KEYS = frozenset(('ip', 'id', 'hi', 'hd', 'si', 'sd', 'l1', 'l2', 'l3', 'l4', 't9', 'sc', 'ss',
                               'btn_stand', 'btn_sit', 'btn_wave', 'btn_crawl',
                               'btn_grip_open', 'btn_grip_close', 'am1', 'am0'))


def _ma():
  import mech_arm
  return mech_arm


def _parse_key_value(req):
  i = req.find('key=')
  if i < 0:
    return ''
  j = i + 4
  k = req.find('&', j)
  if k < 0:
    return req[j:]
  return req[j:k]


def _leading_int(s):
  """只取前导符号+数字，避免 t 值与 http/1.0 等粘连时 int() 整段失败导致 thr/turn 不更新。"""
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


# 与 example/workSpace/web_c.py 一致：thr = value_f * JOY_THR_MAX / 100（GetY 前推为正 → thr 为正）
# 大狗若与灯哥小机物理方向相反，在 config_s 设 joy_fwd_sign=-1，勿在函数内写死 vf=-vf
JOY_THR_MAX = 6.0
JOY_THR_MIN = -3.0
JOY_DEAD = 10
JOY_TURN_DEAD = 15


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


def _set_joy_turn_val(t):
  try:
    padog.set_joy_turn(t)
  except AttributeError:
    padog.joy_turn = float(t)


def _apply_stick_move(thr, turn):
  """摇杆 X 取反后 jt>0=左 jt<0=右；转向 L/R 与 example 一致，横杆优先于微弱前后。"""
  if getattr(padog, 'crawl_phase', 0):
    return
  t = -int(turn)
  if abs(t) < JOY_TURN_DEAD:
    _set_joy_turn_val(0)
    L, R = 1, 1
  else:
    _set_joy_turn_val(t)
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


def _parse_joy_f_t(req):
  # 解析摇杆 f/t：支持 f=80t=0 或 f=80&t=0；f 与 t 之间只取数字，避免误解析
  i = req.find('f=')
  if i < 0:
    return None, None
  k = req.find('t=', i + 2)
  if k < 0:
    return None, None
  vf = _leading_int(req[i + 2 : k])
  vt = _leading_int(req[k + 2 :])
  if vf is None or vt is None:
    return None, None
  return vf, vt


#-----------------------HTTP Server-----------------------#
user_leg_num=str(int(getattr(padog, 'cal_leg_sel', 1)))
url_cal="cal.html"
url_c="control.html"
thr=0;turn=0;L=0;R=0;Pitch=int(getattr(padog,'in_pit',0));Roll=int(getattr(padog,'in_rol',0));Yst=padog.in_y
value=''
# 与网页高度滑条一致：显示值 = 下发 hgt = padog.R_H
Hgt=int(getattr(padog, 'H_goal', 90))
color_leg1='#7DFF7D';color_leg2='#FF9E9E';color_leg3='#FF9E9E';color_leg4='#FF9E9E';
test_add=0
url_n=url_c
g_s_num=100

addr = (padog.selfadd,80) #定义socket绑定的地址，ip地址为本地，端口为80
s = socket.socket()     #创建一个socket对象
try:
  s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
except:
  pass
s.bind(addr)            #绑定地址
s.listen(8)             #略增积压，减轻并发 GET 时排队
gc.collect()
print('listening on:', addr, 'free:', gc.mem_free())
padog.gesture(Pitch,Roll,Yst)

def _http_204(cl):
  try:
    cl.sendall('HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n')
  except:
    pass


def _http_send_file(cl, path, tail=''):
  try:
    cl.sendall('HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/html; charset=utf-8\r\n\r\n')
  except:
    return
  try:
    with open(path, 'r') as f:
      while True:
        chunk = f.read(512)
        if not chunk:
          break
        try:
          cl.sendall(chunk)
        except:
          return
  except Exception as e:
    print(path, 'read err:', e)
    try:
      cl.sendall('<html><body><h1>' + path + ' not found</h1></body></html>')
    except:
      pass
    return
  if tail:
    try:
      cl.sendall(tail)
    except:
      pass


def _http_send_body(cl, body):
  try:
    cl.sendall('HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: text/html; charset=utf-8\r\n\r\n')
  except:
    return
  i = 0
  n = len(body)
  while i < n:
    try:
      cl.sendall(body[i:i + 1024])
    except:
      break
    i += 1024


def _control_slider_script():
  return """
        <script type="text/javascript">
        (function(){
          var e1=document.getElementById("dimSlide1");if(e1)e1.value=""" + str(int(getattr(padog, 'in_pit', 0))) + """;
          var e2=document.getElementById("dimSlide2");if(e2)e2.value=""" + str(int(getattr(padog, 'in_rol', 0))) + """;
          var e3=document.getElementById("dimSlide3");if(e3)e3.value=""" + str(Yst) + """;
        })();
        </script>
        """


def _send_cal_tail(cl):
  try:
    cl.sendall("""
      <center><table border="3">
      <tr>
      <th bgcolor='""" + color_leg1 + """'>1左前 髋：""" + str(padog.init_1p) + """<Br/>大：""" + str(padog.init_1h) + """ 小：""" + str(padog.init_1s) + """</th>
      <th bgcolor='""" + color_leg2 + """'>2右前 髋：""" + str(padog.init_2p) + """<Br/>大：""" + str(padog.init_2h) + """ 小：""" + str(padog.init_2s) + """</th>
      </tr>
      <tr>
      <th bgcolor='""" + color_leg4 + """'>4左后 髋：""" + str(padog.init_4p) + """<Br/>大：""" + str(padog.init_4h) + """ 小：""" + str(padog.init_4s) + """</th>
      <th bgcolor='""" + color_leg3 + """'>3右后 髋：""" + str(padog.init_3p) + """<Br/>大：""" + str(padog.init_3h) + """ 小：""" + str(padog.init_3s) + """</th>
      </tr>
      </table></center><Br/><Br/>
      <center><h1>控制器参数设定</h1><form action="/" method="get" accept-charset="utf-8">
      <p>大腿(杆1)长 : <input name="l1" value='""" + str(padog.l1) + """'/></p>
      <p>小腿(杆2)长 : <input name="l2" value='""" + str(padog.l2) + """'/></p>
      <p>机器人长度 : <input name="l" value='""" + str(padog.l) + """'/></p>
      <p>机器人宽度 : <input name="b" value='""" + str(padog.b) + """'/></p>
      <p>腿间距 : <input name="w" value='""" + str(padog.w) + """'/></p>
      <p>TROT步频 : <input name="speed" value='""" + str(padog.speed) + """'/></p>
      <p>TROT抬腿高度 : <input name="h" value='""" + str(padog.h) + """'/></p>
      <p>腿长参考 : <input name="leg_len_ref" value='""" + str(getattr(padog, "leg_len_ref", 149.0)) + """'/></p>
      <p>TROT前进重心P : <input name="trot_cg_f" value='""" + str(padog.trot_cg_f) + """'/></p>
      <p>TROT后退重心P : <input name="trot_cg_b" value='""" + str(padog.trot_cg_b) + """'/></p>
      <p>TROT转向重心P : <input name="trot_cg_t" value='""" + str(padog.trot_cg_t) + """'/></p>
      <p>WALK抬腿高度 : <input name="walk_h" value='""" + str(padog.walk_h) + """'/></p>
      <p>WALK步频 : <input name="walk_speed" value='""" + str(padog.walk_speed) + """'/></p>
      <p>高度P : <input name="Kp_H" value='""" + str(padog.Kp_H) + """'/></p>
      <p>姿态P : <input name="Kp_G" value='""" + str(padog.Kp_G) + """'/></p>
      <p>CG_X : <input name="CG_X" value='""" + str(padog.CG_X) + """'/></p>
      <p>CG_Y : <input name="CG_Y" value='""" + str(padog.CG_Y) + """'/></p>
      <p>hip_k_roll : <input name="hip_k_roll" value='""" + str(padog.hip_k_roll) + """'/></p>
      <p>hip_k_pitch : <input name="hip_k_pitch" value='""" + str(padog.hip_k_pitch) + """'/></p>
      <p>hip_k_turn : <input name="hip_k_turn" value='""" + str(padog.hip_k_turn) + """'/></p>
      <p>hip_delta_max : <input name="hip_delta_max" value='""" + str(padog.hip_delta_max) + """'/></p>
      <input type="Submit" value="更改控制器参数" /></form></center></body></html>
      """)
  except Exception as e:
    print('cal tail err:', e)


while True:
  cl, addr = s.accept() #接受客户端的连接请求，cl为此链接创建的一个新的scoket对象，addr客户端地址
  #print('client connected from:', addr)
  try:
    raw = cl.recv(1024)
  except:
    try:
      cl.close()
    except:
      pass
    continue
  if isinstance(raw, bytes):
    req = raw.decode('utf-8', 'ignore')
  else:
    req = str(raw)
  req = req.replace('\r\n', '\n').replace('\r', '\n')
  req = req.split('\n')
  #http header 解析
  req_data=req[0].lstrip().rstrip().replace(' ','').lower()
  if req_data.find('favicon.ico')>-1:
    try:
      cl.sendall('HTTP/1.1 204 No Content\r\nConnection: close\r\n\r\n')
    except:
      pass
    cl.close()
    continue
  else:
    req_data = (
      req_data.replace('get/?', '')
      .replace('http/1.1', '')
      .replace('http/1.0', '')
      .replace('http/2', '')
      .replace("b'", '')
    )
    #print('req_data',req_data)
    if req_data.find('speed')>-1:
      print(req_data.replace('&',';'))
      exec(req_data.replace('&',';'))
      padog.l1=l1
      padog.l2=l2
      padog.l=l
      padog.b=b
      padog.w=w
      padog.speed=speed
      padog.h=h
      padog.Kp_H=kp_h
      padog.Kp_G=kp_g
      padog.CG_X=cg_x
      padog.CG_Y=cg_y
      padog.walk_h=walk_h
      padog.walk_speed=walk_speed
      padog.trot_cg_f=trot_cg_f
      padog.trot_cg_b=trot_cg_b
      padog.trot_cg_t=trot_cg_t
      try:
        padog.leg_len_ref=leg_len_ref
        padog.joy_fwd_sign=joy_fwd_sign
      except NameError:
        pass
      try:
        padog.hip_k_roll=hip_k_roll
        padog.hip_k_pitch=hip_k_pitch
        padog.hip_k_turn=hip_k_turn
        padog.hip_delta_max=hip_delta_max
      except NameError:
        pass
      try:
        padog._sync_pa_step_timing()
      except AttributeError:
        pass
    #判断摇杆（勿用 find('t=') 全串搜索，否则会命中 pit= 中的 t=）
    value_f, value_t = _parse_joy_f_t(req_data)
    if value_f is not None and value_t is not None:
      try:
        thr = _joy_f_to_thr(value_f)
        turn = int(value_t)
      except:
        pass
    #判断按钮
    value = _parse_key_value(req_data).strip().lower()
    #Pitch
    if req_data.find('pit=')>-1:
      index_p = req_data.find('pit=')
      value_p = req_data[index_p+4:index_p+7].lstrip().rstrip()
      if value_p!='/':
        print('pit:',str(value_p))
        Pitch=int(value_p)
    #Roll
    if req_data.find('rol=')>-1:
      index_r = req_data.find('rol=')
      value_r = req_data[index_r+4:index_r+7].lstrip().rstrip()
      if value_r!='/':
        #print('rol:',str(value_r))
        Roll=int(value_r)
    #Height
    if req_data.find('hgt=')>-1:
      index_h = req_data.find('hgt=')
      value_h = req_data[index_h+4:index_h+7].lstrip().rstrip()
      if value_h!='/':
        print('hgt:',str(value_h))
        Hgt=int(value_h)
    #Y_controller
    if req_data.find('yst=')>-1:
      index_y = req_data.find('yst=')
      value_y = req_data[index_y+4:index_y+7].lstrip().rstrip()
      if value_y!='/':
        print('yst:',str(value_y))
        Yst=int(value_y)
    #运动控制用（不可接在 yst 的 if/elif 上，否则无 yst= 时 key=is 等永远不执行）
    if value == 'ss':
      Pitch=0;Roll=0          #清除姿态
      padog.stable(False)     #清除陀螺仪
      padog.gait(0)           #重置步态模式
      user_leg_num=str(int(padog.cal_leg_sel))
      url_n=url_cal
      print('enter cal page')
    elif value == 'go':
      print('True')
      padog.stable(True)
    elif value == 'gc':
      print('False')
      padog.stable(False)
    elif value == 'g0':
      padog.stable(False)   #切换walk步态时自动关闭陀螺仪，防止冲突
      padog.gait(0)
    elif value == 'g1':
      padog.stable(False)
      padog.gait(1)
    elif value == 'is':
      # 步态测试：mainloop 内略弱 TROT + 压低后腿摆腿目标；见 padog.inplace_step_end_ms
      padog.stable(False)
      padog.gait(0)
      padog.inplace_step_end_ms = utime.ticks_add(utime.ticks_ms(), 5000)
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
    elif value == 'am1':
      _ma().set_enabled(True)
    elif value == 'am0':
      _ma().set_enabled(False)
    elif value == 'btn_grip_open':
      _ma().grip_open()
    elif value == 'btn_grip_close':
      _ma().grip_close()
    #标定判断用
    if value == 'l2':
      user_leg_num='2'
      padog.cal_leg_sel = 2
      color_leg1='#FF9E9E';color_leg2='#7DFF7D';color_leg3='#FF9E9E';color_leg4='#FF9E9E'
    elif value == 'l4':
      user_leg_num='4'
      padog.cal_leg_sel = 4
      color_leg1='#FF9E9E';color_leg2='#FF9E9E';color_leg3='#FF9E9E';color_leg4='#7DFF7D'
    elif value == 'l1':
      user_leg_num='1'
      padog.cal_leg_sel = 1
      color_leg1='#7DFF7D';color_leg2='#FF9E9E';color_leg3='#FF9E9E';color_leg4='#FF9E9E'
    elif value == 'l3':
      user_leg_num='3'
      padog.cal_leg_sel = 3
      color_leg1='#FF9E9E';color_leg2='#FF9E9E';color_leg3='#7DFF7D';color_leg4='#FF9E9E'
    elif value == 'sc':   #保存并退出
      s_f = open("config_s.py", "w+")
      #保存中位
      s_f.write("init_1p="+str(padog.init_1p)+"\n")
      s_f.write("init_1h="+str(padog.init_1h)+"\n")
      s_f.write("init_1s="+str(padog.init_1s)+"\n")
      s_f.write("init_2p="+str(padog.init_2p)+"\n")
      s_f.write("init_2h="+str(padog.init_2h)+"\n")
      s_f.write("init_2s="+str(padog.init_2s)+"\n")
      s_f.write("init_3p="+str(padog.init_3p)+"\n")
      s_f.write("init_3h="+str(padog.init_3h)+"\n")
      s_f.write("init_3s="+str(padog.init_3s)+"\n")
      s_f.write("init_4p="+str(padog.init_4p)+"\n")
      s_f.write("init_4h="+str(padog.init_4h)+"\n")
      s_f.write("init_4s="+str(padog.init_4s)+"\n")
      #保存机械、步态参数
      s_f.write("l1="+str(padog.l1)+"\n")
      s_f.write("l2="+str(padog.l2)+"\n")
      s_f.write("l="+str(padog.l)+"\n")
      s_f.write("b="+str(padog.b)+"\n")
      s_f.write("w="+str(padog.w)+"\n")
      s_f.write("speed="+str(padog.speed)+"\n")
      s_f.write("h="+str(padog.h)+"\n")
      s_f.write("Kp_H="+str(padog.Kp_H)+"\n")
      s_f.write("Kp_G="+str(padog.Kp_G)+"\n")
      s_f.write("CG_X="+str(padog.CG_X)+"\n")
      s_f.write("CG_Y="+str(padog.CG_Y)+"\n")
      s_f.write("walk_h="+str(padog.walk_h)+"\n")
      s_f.write("walk_speed="+str(padog.walk_speed)+"\n")
      s_f.write("ma_case="+str(padog.ma_case)+"\n")
      s_f.write("leg_len_ref="+str(getattr(padog, "leg_len_ref", 149.0))+"\n")
      s_f.write("joy_fwd_sign="+str(getattr(padog, "joy_fwd_sign", 1))+"\n")
      s_f.write("trot_cg_f="+str(padog.trot_cg_f)+"\n")
      s_f.write("trot_cg_b="+str(padog.trot_cg_b)+"\n")
      s_f.write("trot_cg_t="+str(padog.trot_cg_t)+"\n")
      try:
        import PA_TROT
        s_f.write("faai="+str(PA_TROT.faai)+"\n")
      except Exception:
        s_f.write("faai=0.40\n")
      s_f.write("hip_k_roll="+str(padog.hip_k_roll)+"\n")
      s_f.write("hip_k_pitch="+str(padog.hip_k_pitch)+"\n")
      s_f.write("hip_k_turn="+str(padog.hip_k_turn)+"\n")
      s_f.write("hip_delta_max="+str(padog.hip_delta_max)+"\n")
      s_f.write("arm_upper_init="+str(getattr(padog, 'arm_upper_init', getattr(padog, 'arm_base_init', 90)))+"\n")
      s_f.write("arm_fore_init="+str(getattr(padog, 'arm_fore_init', 90))+"\n")
      s_f.write("arm_upper_min="+str(getattr(padog, 'arm_upper_min', getattr(padog, 'arm_base_min', 45)))+"\n")
      s_f.write("arm_upper_max="+str(getattr(padog, 'arm_upper_max', getattr(padog, 'arm_base_max', 135)))+"\n")
      s_f.write("arm_fore_min="+str(getattr(padog, 'arm_fore_min', 30))+"\n")
      s_f.write("arm_fore_max="+str(getattr(padog, 'arm_fore_max', 140))+"\n")
      s_f.write("arm_upper_rate="+str(getattr(padog, 'arm_upper_rate', getattr(padog, 'arm_base_rate', 1.8)))+"\n")
      s_f.write("arm_fore_rate="+str(getattr(padog, 'arm_fore_rate', 1.8))+"\n")
      s_f.write("arm_upper_dir="+str(getattr(padog, 'arm_upper_dir', getattr(padog, 'arm_base_dir', 1)))+"\n")
      s_f.write("arm_fore_dir="+str(getattr(padog, 'arm_fore_dir', 1))+"\n")
      s_f.write("arm_upper_ch="+str(getattr(padog, 'arm_upper_ch', 6))+"\n")
      s_f.write("arm_fore_ch="+str(getattr(padog, 'arm_fore_ch', 7))+"\n")
      s_f.write("arm_grip_gpio="+str(getattr(padog, 'arm_grip_gpio', -1))+"\n")
      s_f.write("arm_grip_open_level="+str(getattr(padog, 'arm_grip_open_level', 0))+"\n")
      s_f.write("arm_grip_close_level="+str(getattr(padog, 'arm_grip_close_level', 1))+"\n")
      s_f.write("arm_fore_board="+str(getattr(padog, 'arm_fore_board', 0x40))+"\n")
      s_f.write("arm_base_init="+str(getattr(padog, 'arm_base_init', 90))+"\n")
      s_f.write("arm_grip_init="+str(getattr(padog, 'arm_grip_init', 90))+"\n")
      s_f.write("arm_base_min="+str(getattr(padog, 'arm_base_min', 45))+"\n")
      s_f.write("arm_base_max="+str(getattr(padog, 'arm_base_max', 135))+"\n")
      s_f.write("arm_grip_open="+str(getattr(padog, 'arm_grip_open', 110))+"\n")
      s_f.write("arm_grip_close="+str(getattr(padog, 'arm_grip_close', 70))+"\n")
      s_f.write("arm_base_rate="+str(getattr(padog, 'arm_base_rate', 1.8))+"\n")
      s_f.write("arm_base_dir="+str(getattr(padog, 'arm_base_dir', 1))+"\n")
      s_f.write("arm_grip_stick_sign="+str(getattr(padog, 'arm_grip_stick_sign', 1))+"\n")
      #保存重心平移量
      s_f.write("H_goal="+str(int(getattr(padog, 'H_goal', Hgt)))+"\n")
      s_f.write("in_y="+str(Yst)+"\n")
      s_f.write("in_pit="+str(Pitch)+"\n")
      s_f.write("in_rol="+str(Roll)+"\n")
      s_f.write("cal_leg_sel="+str(int(padog.cal_leg_sel))+"\n")
      try:
        s_f.flush()
      except:
        pass
      s_f.close()
      padog.in_pit=int(Pitch);padog.in_rol=int(Roll);padog.in_y=int(Yst)
      padog.H_goal=int(Hgt)
      padog.R_H=int(Hgt)
      padog.PIT_goal=int(Pitch);padog.ROL_goal=int(Roll);padog.X_goal=int(Yst)
      user_leg_num=str(int(padog.cal_leg_sel))
      url_n=url_c
      padog.servo_init(0)
            
    elif value == 'hi':
      exec("padog.init_"+user_leg_num+"h="+"padog.init_"+user_leg_num+"h+1")
    elif value == 'hd':
      exec("padog.init_"+user_leg_num+"h="+"padog.init_"+user_leg_num+"h-1")
    elif value == 'si':
      exec("padog.init_"+user_leg_num+"s="+"padog.init_"+user_leg_num+"s+1")
    elif value == 'sd':
      exec("padog.init_"+user_leg_num+"s="+"padog.init_"+user_leg_num+"s-1")
    elif value == 'ip':
      exec("padog.init_"+user_leg_num+"p="+"padog.init_"+user_leg_num+"p+1")
    elif value == 'id':
      exec("padog.init_"+user_leg_num+"p="+"padog.init_"+user_leg_num+"p-1")
    elif value == 't9':
      padog.servo_init(1)
      
  _send_full_page = (req_data.find('speed')>-1 or (req_data.find('f=')==-1 and req_data.find('g0')==-1 and req_data.find('g1')==-1 and req_data.find('go')==-1 and req_data.find('gc')==-1 and req_data.find('pit=')==-1 and req_data.find('rol=')==-1 and req_data.find('yst=')==-1 and req_data.find('hgt=')==-1))
  if _send_full_page:
    try:
      if url_n == url_cal:
        _http_send_file(cl, 'cal.html')
        _send_cal_tail(cl)
      else:
        _http_send_file(cl, 'control.html', _control_slider_script())
    except Exception as e:
      print('web_c page err:', e)
  else:
    _http_204(cl)
  try:
    cl.close()
  except:
    pass

  #命令（标定 ip/hi/… 及姿态按钮时不跑摇杆 move）
  if value not in _CAL_NO_MOVE_KEYS:
    _arm_on = _ma().is_enabled()
    _joy_f = None
    _joy_t = None
    if req_data.find('f=') >= 0:
      _jf, _jt = _parse_joy_f_t(req_data)
      if _jf is not None and _jt is not None:
        _joy_f, _joy_t = int(_jf), int(_jt)
    if _arm_on and _joy_f is not None and _joy_t is not None:
      _ma().apply_stick(_joy_f, _joy_t)
    elif not _arm_on:
      if thr > JOY_THR_MAX:
        thr = JOY_THR_MAX
      elif thr < JOY_THR_MIN:
        thr = JOY_THR_MIN
      _ie = getattr(padog, 'inplace_step_end_ms', 0) or 0
      if _ie and utime.ticks_diff(_ie, utime.ticks_ms()) > 0:
        padog.set_leg_sit_offsets(0, 0)
        padog.move(4, 1, 1)
      elif thr==0 and abs(int(turn)) < JOY_TURN_DEAD:
        padog.set_leg_sit_offsets(0, 0)
        if not getattr(padog, 'crawl_phase', 0):
          try:
            padog.set_joy_turn(0)
          except AttributeError:
            padog.joy_turn = 0
          padog.move(0,0,0)
      else:
        padog.set_leg_sit_offsets(0, 0)
        _apply_stick_move(thr, turn)

  if value not in ('btn_stand', 'btn_sit', 'btn_wave', 'btn_crawl'):
    padog.height(Hgt)
    padog.gesture(Pitch,Roll,Yst)






