# tools/golden —— 宿主侧 golden 对照测试

**目的**：把 MicroPython 参考实现和 C 移植版**逐数值对照**，替代"靠肉眼看数字"。

**为什么能在电脑上跑**：`PA_IK` / `PA_ATTITUDE` / `PA_TROT` / `PA_WALK` 都是**纯数学**模块，
只 `import math`，所以既能在 CPython 里直接 `exec`（生成参考值），
也能用宿主 gcc 编译（运行 C 版）——**不用烧板子**。

验收标准来自 `../../ESP-IDF_C迁移表.md`：

| 模块 | 容差 |
|---|---|
| IK 关节角 | < 0.5° |
| 步态轨迹 | < 1 mm |
| 姿态 8 输出 | < 0.5 mm |

---

## 怎么用

```powershell
cd tools\golden
.\run_golden.bat
```

它会做四步：生成参考值 → 找 gcc → 编译宿主测试 → 逐行比对并打印最大误差。

也可以分步：

```powershell
python gen_golden.py                                   # 只重新生成参考值
gcc -O2 -Wall -Wextra -std=c11 -I..\..\firmware\src `
    -o build\test_kinematics.exe test_kinematics.c `
    ..\..\firmware\src\control\kinematics.c -lm
build\test_kinematics.exe golden\ik.csv
```

---

## 设计要点

### 参考值来自原实现，不是我重写的

`gen_golden.py` 直接 `exec` `micropython/PA_IK.py`，把 `ik()` 拿出来当参考。
所以参考值**100% 来自学长的原始代码**，不存在"我理解错了原逻辑"的风险。

原始模块只 `import math`，能直接在 CPython 里跑。
`PA_TROT` / `PA_WALK` 里有 `from machine import I2C, Pin` 和 `import padog`，
到时候需要几十行 stub —— 但 `cal_t` / `cal_w` 本身是纯数学。

### 覆盖了两个分支

| case | 含义 | 实机状态 |
|---|---|---|
| 0 | 串联腿 | **实机在跑这个**（`config_s.py` 里 `ma_case=0`） |
| 1 | 并联腿 | ⚠️ 实机未验证过，但照样测（将来可能用） |

### 输入采样刻意覆盖边界

`ik_samples()` 生成 151 个点，包含：
- `x == 0`（走原实现的 `else` 分支，用到那个 `pi - 1.5707` 怪写法）
- `x > 0` / `x < 0`（两个 `if` 分支）
- 接近最小/最大可达半径（9.0 ~ 267.0 mm，留 1 mm 余量避免 `acos` 浮点越界）

### 测试程序输出用 ASCII

不是偷懒 —— 这是 `../../问题与解决记录.md` 里 **P-06** 的教训：
程序往 Windows 控制台打非 ASCII 会踩代码页坑。数值工具用英文最稳。

---

## 当前结果（2026-09-19）

### kinematics ← PA_IK.py

```
rows     : 74   (case=0 series: 37, case=1 parallel: 37)
samples  : 592 joint angles compared
tolerance: 0.500 deg

per-field max error (deg):
  ham1   0.0000211    shank1  0.0000250
  ham2   0.0000302    shank2  0.0000357
  ham3   0.0000421    shank3  0.0000207
  ham4   0.0000555    shank4  0.0000603

RESULT: PASS -- max error 0.0000603 deg  (8290x inside the tolerance)
```

### body_pose ← PA_ATTITUDE.py

```
rows     : 87
samples  : 696 foot-target values compared
tolerance: 0.500 mm

per-field max error (mm):
  x1  0.000011874   y1  0.000022257
  x2  0.000011874   y2  0.000021105
  x3  0.000008295   y3  0.000022257     (x3/y3 对应腿4)
  x4  0.000008295   y4  0.000014321     (x4/y4 对应腿3)

RESULT: PASS -- max error 0.000022257 mm  (22465x inside the tolerance)
```

### gait_trot ← PA_TROT.py

```
rows     : 1040
samples  : 8320 trajectory values compared
tolerance: 1.000 mm

per-field max error (mm):
  x1..x4   0.000007750
  y1,y3    0.000019430
  y2,y4    0.000007629

RESULT: PASS -- max error 0.000019430 mm  (51467x inside the tolerance)
```

覆盖了 4 组时序参数（含实机值 `Ts=1.0 / faai=0.42`）、5 组运动参数、4 组腿系数，
时间采样刻意取到三个边界（`t=0`、`t=faai*Ts`、`t=Ts` 附近）以覆盖两个分支。

### gait_walk ← PA_WALK.py（含一个**副作用**）

```
rows        : 984
foot samples: 7872  (tolerance 1.000 mm)
gesture     : 2952  (exact integer match required)

per-field max error (mm):  1.2e-5 ~ 6.7e-5
worst foot case: x4  expected=20.0850232  got=20.0850906  err=0.0000674
gesture mismatches: 0

RESULT: PASS -- foot max error 0.000067425 mm (14831x inside 1.000 mm), gestures exact
```

**这个模块最有意思的地方:原实现有副作用。**

`cal_w()` 内部会调 `_apply_cg()`，而后者执行 `padog.gesture(0, int(CG_X), int(yst))`
—— **直接改 padog 的重心目标**（`PIT_goal`/`ROL_goal`/`X_goal`）。
C 版把它变成了**显式输出** `gait_walk_gesture_t`，由调用方决定怎么用。

而且原实现用 `int()` **向零截断**（不是四舍五入），所以那 3 个输出是整数、要求精确相等。
参考值里有 457 行是负数 `grol`，专门覆盖这个截断行为。

**又一个 Python 语义陷阱（和滑动平均那个同类）：**

`_leg_xy()` 里的 `phi = local_t % T`，而 `local_t` 可能为负（`t - off2/off3/off4`）。
Python 的 `%` 对负数返回**非负**结果（地板取模），C 的 `fmodf` 保留被除数符号 ——
两者**差整整一个周期**，轨迹会完全错位。`gait_walk.c` 用 `py_fmodf()` 复刻。

**还有一个"测试自身的漏洞"是我自己发现并补上的：**

第一版参考值里我把腿系数写死成 `(1,1,1,1)` —— 这样**根本测不出**原实现形参
`(r1, r4, r2, r3)` 那个错位映射对不对（映射错了也照样"通过"）。
改成 4 组含非对称值的组合（含 `1,-2,-1,0.5` 这种）后，映射才真正被钉住。

### moving_avg ← PA_AVGFILT.py（**有状态**，测的是一串调用序列）

```
rows        : 264   (stateful: rows are fed in order)
windows     : 2, 7
mismatches  : 0

RESULT: PASS -- all 264 outputs match exactly
```

这个模块的测法和其他三个不同：CSV 是**一串按顺序的调用**，同一 `window` 的行
必须依次喂给同一个滤波器实例（遇到新 `window` 就重新初始化）。
输出是**整数**，所以要求精确相等，没有容差。

原实现有个**移植陷阱**：`return self.cache[1] // (self.len - 3)` 用的是 Python 的 `//`，
对负数**向 -∞ 取整**；而 C 的 `/` 是**向 0 截断**。陀螺仪原始值有负数，所以这是真实差异。
`filter_moving_avg.c` 显式实现了向下取整。

### control_chain ← `padog.py` 的 `mainloop()`（**P3：全链路**）

```
rows        : 90   (gait0/trot: 58, gait1/walk: 32)
crawl rows  : 26   (crawl_phase != 0 in the input)
pairs       : 1080 (12 channels x 90 rows; each pair = ON + OFF)
exact pairs : 1080 / 1080
max delta   : 0 duty counts  (allowed 0)
mismatches  : 0

RESULT: PASS -- all 12 channels match the original mainloop() exactly
```

**这一套和前面七套的性质不同。** 前面都是"一个模块对一个模块"；这一套对照的是
**整条控制链**，而且参考值不是我拼出来的 ——

> **把原版 `padog.py` 整个 exec 进来，直接调用它的 `mainloop()`，记录它写出的
> 12 组占空比。** 每一行都重新 exec 一次 padog.py，所以模块级状态每行都是干净的。

于是那些**从来没被单独测过、也一行没迁过**的东西全进了对照范围：

- **大狗缩放层**：`_geom_scale()`（=1.7987）、`_partial_geom_scale()`（故意不全乘）、
  `_ik_hc()`（= `R_H + 119`）、以及 6 个 `_LARGE_*` 系数
- **编排层**：抬腿高度按速度自适应、姿态 `slew` 环（每帧只逼近一步）、
  按步态模式与摇杆方向**选重心的 5 个分支**
- **`padog.py` 第 57~81 行那张 59 项的默认值注入表**
- `_hip_leg_deltas()` / `_apply_trot_swing_y()` / `_foot_y_targets()` /
  `cal_test_shank()` / `servo_output()`

CSV：15 个输入列（`spd, L, R, gait_mode, t, R_H, H_goal, PIT_S, PIT_goal, ROL_S,
ROL_goal, X_S, X_goal, joy_turn, crawl_phase`）+ 12 组 (ON, OFF)。

> ⚠️ **`t` 只在 `[0, Ts]` 内取值**。原版 `PA_TROT.cal_t()` 没有 `else` 分支，
> `t > Ts` 会直接 `UnboundLocalError` —— 那不是原版的可达域（`mainloop` 里
> `if t >= Ts: t = t - Ts` 先回绕了）。C 版**故意**做了回绕（迁移表 §8.10），
> 所以喂越界输入等于在测"我自己的改进"，而不是在测等价性。

### ⚠️ 这一套也踩了同一个坑（P-25）

第一版**1080/1080 全绿**，但参考环境是错的：
`PA_WALK._apply_cg()` 会调 `padog.gesture()` **直接改重心目标**，而参考环境里
`sys.modules['padog']` 是 `mpy_stubs` 的空壳、`gesture` 是 no-op
⇒ **副作用被静默吞掉**，C 版于是"精确匹配了一个原版并不产生的行为"，
而且还把这个错误**写进了注释**当作有意差异。

修法：`load_padog_ns()` 把 `padog.py` **exec 进一个真模块对象并注册为
`sys.modules['padog']`**。改完重跑，**90 行里 24 行的参考值变了**，
C 版失败 125 处 —— 修 C 后才重新精确通过。

⇒ 教训：**stub 只对纯函数安全；凡是 stub 掉一个"会被调用"的东西，
先问它原本会改什么。** 以及：**测试通过之后，要反过来审一遍参考值自己是怎么来的。**

### ✅ 验证验证器：这个测试真的有牙齿吗

对 14 处做过故意破坏（都在临时副本上），**全部 FAIL**：

| 破坏 | 不匹配数 |
|---|---:|
| 去掉爬行服务 | 188 |
| `joy_forward_motion` 取反 | 351 |
| 去掉 `_apply_trot_swing_y` | 88 |
| 去掉俯仰限位 | 10 |
| 去掉站高 slew | 188 |
| `trot_cg_f` 符号取反 | 264 |
| `_hip_leg_deltas` 清零 | 234 |
| `_walk_phase_step` 清零 | 141 |
| WALK 重心系数 `0.65 → 1.65` | 27 |
| **不施加 WALK 的 gesture 副作用** | **125** ← 就是 P-25 那个 |

唯一"活下来"的是把 WALK 重心系数从 0.65 微调到 0.66 —— 变化幅度低于占空比的
量化步长，属于**必然测不出**的（而真正的改动 0.65→1.65 会 FAIL）。

### servo_map ← PA_SERVO.py + padog.py 的 `servo_output()`（**P2 的映射层**）

```
[A] angle / duty path   (Servos.position + PCA9685.duty)
rows        : 284   (angle: 240, duty: 44)
exact rows  : 284 / 284
min/max duty: 102 / 511
mismatches  : 0

[B] joint angles -> 12 channels   (padog.servo_output)
rows        : 60   (ik path: 44, direct-stand path: 16)
values      : 720  (12 channels x 2 registers)
exact values: 720 / 720
mismatches  : 0

RESULT: PASS -- all angle/duty/PWM values match the MicroPython reference
```

这是**精确相等**的测试（0 个计数单位的容差），而且参考值的取法和前面都不同 ——
它是"原代码真正会写进 PCA9685 的字节"：

| 段 | 参考值怎么来的 |
|---|---|
| 角度 → 占空比 → (ON, OFF) | **直接 import 真的 `PA_SERVO.py`**，给它一个**记录型 I2C**。这个假 I2C 不模拟任何硬件行为，只把原代码要写的寄存器原样记下来 |
| 关节角 → 12 路舵机角 | **exec 真版 `padog.py`**（`load_padog_ns()`），调它自己的 `servo_output()` |

> ⚠️ **第一版不是这样，而且错了。** 最初用 `ast` 把函数"抠"出来、手工拼一个命名空间 ——
> 结果把 `padog.py` 第 57~81 行那张**59 项的默认值注入表**一起丢掉了，
> 于是参考值和 C 版**都把小腿曲线偏置算成 0.375，真值是 0.25**，
> 测试全绿却都是错的。详见成长手册 **P-23**。
> 现在 `hip`（来自真版 `_hip_leg_deltas()`）与 `cs`（来自真版
> `_crawl_shank_servodelta()`）都由参考实现算出来再写进 CSV，不由我手填。

> `_hip_leg_deltas()`（髋辅助偏航）由**真正的输入** `ROL_S` / `PIT_S` / `joy_turn`
> 驱动 —— 顺带把这几个函数也一起测了。

**采样刻意覆盖的边界**：角度 0/180/越界（±1000）、分数角、真机中位角；
占空比的 0 与 4095 两个特殊分支；四条腿**互不相等的**腿系数（否则映射错位测不出来，
见 P-18）；爬行压低增量、小腿微调、三组不同几何（改变 `shank_ik_bias`）、
以及"中位角被改过"的输入（证明 C 版真的用了配置里的中位值）。

### ⚠️ 这个测试立刻抓出了一个真 bug（P-21）

参考值里 `duty=0` 那一行是 `pwm(index, 0, 4096)` —— **4096**。
而 C 驱动把高字节写成 `(off >> 8) & 0x0F`，把 **bit4（FULL OFF 标志）抹掉了**，
于是"12 路无脉冲"实际上写的是 `ON=0/OFF=0`。正确掩码是 `& 0x1F`。

这个 bug 从 P0 就在，只插 USB 不接舵机永远看不出来（没有负载可观察），
**是逐位对照逼出来的**。修完真机 `readback` 能看到 `OFF=4096`。

### ⚠️ 而且它自己也错过一次（P-23）

上面那张表里"关节角 → 12 路"那一栏，**第一版不是 exec 真版 padog**，
而是手工拼的命名空间 —— 于是参考值漏掉了 `padog.py` 的默认值注入表，
`shank_ik_bias` 取到死代码兜底值 0.375（真值 0.25），
**参考值和 C 版错在同一个地方，测试全绿**。

两个教训叠在一起就是这一节的结论：
**测试抓到了驱动的错，却没有抓到自己的错。**
所以"参考值的运行环境必须是真环境"和"编解码要有独立第二来源"是同一件事的两面。

### ✅ 验证验证器：这个测试真的有牙齿吗

把 `servo_map.c` 里腿2大腿那一行的 `+ ham2` 改成 `- ham2`，重跑：

```
FIRST MISMATCH: servo_output row=0 expected=(0,308) got=(0,102)
mismatches  : 44        (44 = IK 行数，全都中招)
RESULT: FAIL
```

一个正负号就全线 FAIL。对照那条"删掉代码结果一点没变"的 P-19，
这就是**测试有牙齿**和**测试没牙齿**的区别。

### app_config ← config.py / config_s.py（**不是数值对照，是行为对照**）

```
implementation : app_config.c   (纯 C，不依赖 ESP-IDF)
sizeof(app_config_t) = 360 bytes
checks         : 68   failures: 0

RESULT: PASS -- all 68 checks passed
```

配置模块和前五个不同：它**不是一个数学函数**，而是一个结构体 + 校验规则 + 持久化。
所以这里没有 golden CSV，改成一组**断言式检查**：

| 检查组 | 内容 |
|---|---|
| defaults | 每一项默认值都与 `config.py` / `config_s.py` 的实测值一致（30 项） |
| validate | 越界值被限幅到合法区间，且能报出改了哪一项（`changed=11`） |
| round trip | `save` → `load` 后逐字段相等，CRC 一致 |
| no saved config | 空存储 → 用默认值 + 返回 `NOENT`（不是报错崩溃） |
| corrupted CRC | 人为改坏一个字节 → 拒绝载入 + 报 CRC 错，用默认值 |
| version mismatch | 版本号对不上 → 拒绝载入 + 报版本错，用默认值 |
| reset | 擦除 + 回默认值 |

**怎么做到不用板子**：持久化后端是通过**函数指针表** `app_cfg_store_t` 注入的，
所以宿主机上塞一个 RAM 假后端就行，配置逻辑本身零 IDF 依赖。
真机上换成 NVS 后端（`app_config_nvs.c`），上层代码一行不改。

**这个测试替不了什么**：`app_cfg_store_t` 的假后端在内存里，"配置能熬过一次断电"
是 **Flash 行为**，宿主机测不了 —— 这一条是在真机上验的（见下）。

真机验证记录（2026-09-19，COM4）：

```
改 h_goal=70 / faai=0.33 / ap_ssid="TestDog" → cfg save → 硬复位
  ✅ 配置已从 NVS 载入（version=1, crc=0xE09D2647）
  ✅ h_goal=70.0  faai=0.330  AP 热点: ssid="TestDog"      ← 熬过断电
cfg reset
  ✅ 已恢复出厂默认并擦除 NVS，NVS 条目 168 → 154
  ✅ 再启动报 NOENT + 用默认值                            ← 擦除确实生效
限幅（在真机上同样成立，不是只在 PC 上成立）
  ✅ cfg set h_goal 9999        → 250.0000
  ✅ cfg set ma_case 5          → 1
  ✅ cfg set arm_upper_board 0x7F → 64 (0x40)   ← 顺带证明十六进制解析正确
```

> ⚠️ **这里有个安全细节**：`_board_backup_20260919/config.py` 里有**真实的手机热点 SSID 和密码**。
> 那份备份是 gitignore 的，绝不会提交。固件里的默认值是**占位符**（`RobotDog` / `robotdog123`），
> 而且 `test_app_config.c` 里有一条检查专门断言**那个已退役的真实密码不出现在默认值里** ——
> 防止以后有人图省事把真密码填回源码。

### ✅ 验证验证器：测试真的有牙齿吗

一次"故意搞破坏"的实验 —— 把 `filter_moving_avg.c` 里的向下取整修正抽掉
（等价于让它退化成 C 的截断除法），重跑同一个测试：

```
FIRST MISMATCH: window=2 step=9 in=0 expected=-4 got=-3
mismatches  : 101
RESULT: FAIL -- 101 of 264 outputs differ
```

**264 项里 101 项不符，第一个错就在负数取整上。** 这说明测试不是"假装通过"。

> 你也可以自己做这个实验：随便改一下某个系数，或者删掉一行修正，重跑 `run_golden.bat`。
> **应该立刻 FAIL。** 如果改坏了它还 PASS，那才是真问题。

**结论**：五个数学模块都与 MicroPython 参考**数值等价**（整数滤波器为精确相等），
配置模块的 68 条行为检查全部通过，舵机映射层 1004 项**精确相等**。
残余误差量级 8e-6 ~ 6e-5，纯粹是 C `float` 与 Python `double` 的舍入差
（固件刻意用 `float`：ESP32 只有单精度硬件 FPU，`double` 是软件模拟，慢一两个数量级）。
舵机映射层之所以能做到**零误差**，是因为它的输入在两侧都是"已经算好的数"，
不经过浮点积分链。

---

## 怎么让 `import machine` / `import padog` 的模块也能在电脑上跑

`PA_TROT.py` 顶层有 `from machine import I2C, Pin` 和 `import padog`；
`PA_WALK.py` 的函数内部会 `import PA_SERVO`。这些在 CPython 里都会失败。

`mpy_stubs.py` 往 `sys.modules` 里塞两个**最小假模块**：
- `machine`：只让 `I2C` / `Pin` / `PWM` 这些名字存在（构造即抛异常，防止被真用）
- `padog`：提供 `R_H` / `gesture()` 等属性，让 `PA_WALK` 走正常路径而不是被 except 兜住

**设计原则**：stub 只提供名字与最小属性，**不实现任何硬件行为**。
如果某个"纯数学"模块真的调用了硬件路径，生成参考值时就会报错 ——
那说明"纯数学"的判断错了，该换策略，而不是给 stub 加实现去掩盖。

`check_mpy_loadable.py` 用来快速验证：装上 stub 后哪些模块能加载、关键函数能否调用。
加新模块前先跑它。

```powershell
python check_mpy_loadable.py
```

---

## 迁移中发现的三处原实现问题

| 位置 | 问题 | 处置 |
|---|---|---|
| `PA_IK.py` | `acos`/`asin` 参数不检查定义域，越界抛 `ValueError` | C 版 clamp 到 [-1,1]；迁移表要求"异常输入不产生 NaN" |
| `PA_ATTITUDE.cal_ges()` | 算了 `AB1_y..AB4_y` 四个横向坐标，但**返回值里没有它们**（纯无效计算）；而形参 `w`（左右腿间距）**只被这四个式子使用** ⇒ **`w` 对输出毫无影响** | C 版删掉无效计算（删后 golden 误差一位不差，实证其无效）；保留 `w` 形参以维持签名一致并注明 |
| `PA_TROT.cal_t()` | **只有两个分支，没有 else** —— `t > Ts` 会 `UnboundLocalError`；且 `Ts`/`faai` 是模块级全局（隐藏状态） | C 版：时序参数改成显式入参；对 `t<0 \|\| t>Ts` 做相位回绕（`t==Ts` 不回绕，保持与原实现一致） |

## 已知可改进项

现在有 7 个 `test_*.c`，比对/报告逻辑高度重复（CSV 组各约 120 行）。
计划抽一个 `golden_util.h` 共享头（CSV 读取 + 最大误差统计 + PASS/FAIL 打印）。
`test_app_config.c` 是断言式的，只会共用报告部分。
暂时保持每个测试文件自包含、便于单读。

`test_servo_map.c` 里的 `servo_output.csv` 有 60 列 —— 宽表读起来不直观。
如果再加输入维度，应该改成 self-describing 的表头（`ik,hip1,...`）而不是继续加列。

---

## 加新模块时怎么做

1. 在 `gen_golden.py` 里加一个 `gen_<name>()`，从对应 `micropython/PA_*.py` 生成 CSV
2. 写一个 `test_<name>.c`，读 CSV、调用 C 版、逐项比对
3. 在 `run_golden.bat` 里加一行编译+运行
4. **把生成的 CSV 提交进仓库** —— C 测试只读它，不依赖 Python

## 已完成 / 待办

| 模块 | 参考来源 | 状态 |
|---|---|---|
| `kinematics.c` | `PA_IK.py` | ✅ 通过（592 项，最大 6.0e-5°） |
| `body_pose.c` | `PA_ATTITUDE.py` | ✅ 通过（696 项，最大 2.2e-5 mm） |
| `gait_trot.c` | `PA_TROT.py` | ✅ 通过（8320 项，最大 1.9e-5 mm） |
| `filter_moving_avg.c` | `PA_AVGFILT.py` | ✅ 通过（264 项，整数精确相等） |
| `gait_walk.c` | `PA_WALK.py` | ✅ 通过（7872 项足端 + 2952 项重心整数，最大 6.7e-5 mm） |
| `app_config.c` | `config.py` / `config_s.py` | ✅ 通过（68 条行为检查；NVS 持久化在真机验证） |
| `servo_map.c` | `PA_SERVO.py` + `padog.servo_output()` | ✅ 通过（284 + 720 = **1004 项精确相等**） |
| `control_chain.c` | `padog.py` 的 `mainloop()` | ✅ 通过（**1080 组精确相等**，参考值 = 原版 mainloop 真跑） |

**P3 的控制链已迁移并全链路验证。** 舵机的**物理**通道→关节映射与转向
（"ch7 到底是不是右前大腿、正转是抬起还是压下"）只能上机实测，
见 `../../硬件实物核对清单.md` 阶段 E 与固件的 `lgtest` 命令。

下一步：把 `app_config` 补齐（注入表里还有 18 个字段没进配置结构体）→
把 `control_chain` 接进固定周期任务 → 加步态控制台命令。
