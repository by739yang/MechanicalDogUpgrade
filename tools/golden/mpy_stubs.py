#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mpy_stubs.py —— 给纯数学的 MicroPython 模块提供最小 stub

为什么需要：`PA_TROT.py` 顶层有
    from machine import I2C, Pin
    import padog
`machine` 在 CPython 里不存在，`padog` 则会去碰真实硬件（I2C、WiFi、舵机）。
但我们要的 `cal_t()` 本身是纯数学 —— 所以塞两个假的模块进去，让 import 通过即可。

**注意**：stub 只提供名字与最小属性，不实现任何硬件行为。
如果某个模块真的调用了硬件路径，我们会在生成参考值时看到异常 —— 那说明
"纯数学"的判断错了，应该换策略（而不是给 stub 加实现来掩盖）。
"""
import sys
import types
from pathlib import Path

#: 仓库根下的 micropython/ —— 原始参考代码所在
MPY_DIR = Path(__file__).resolve().parents[2] / "micropython"


def _make_machine():
    m = types.ModuleType("machine")

    class _Dummy:
        """占位：只保证名字存在，构造即抛异常（提醒不要真的用它）"""

        def __init__(self, *a, **k):
            raise RuntimeError("mpy_stubs: machine stub 不能真正使用")

    m.I2C = _Dummy
    m.Pin = _Dummy
    m.PWM = _Dummy
    m.Timer = _Dummy
    m.UART = _Dummy
    m.reset = lambda: None
    m.freq = lambda *a: 240000000
    return m


def _make_padog():
    """
    假的 padog。

    `PA_TROT.py` 顶层 `import padog`（不知道为什么，反正它没用到）。
    `PA_WALK.py` 在函数内部才用到 `padog.R_H` / `padog.gesture()`，
    且都包在 try/except 里 —— 这里给上合理默认值，让它们走"正常路径"
    而不是被 except 兜住，能测到更多真实代码。
    """
    p = types.ModuleType("padog")
    # PA_WALK._body_h() 会读它；真机上由 padog.mainloop 从 H_goal 来
    p.R_H = 110.0
    # 传给 PA_ATTITUDE.cal_ges 的姿态量，这里只求不炸
    p.PIT_goal = 0.0
    p.ROL_goal = 0.0
    p.X_goal = 0.0
    p.spd = 0.0
    p.gesture = lambda *a, **k: None
    p.move = lambda *a, **k: None
    p.set_leg_sit_offsets = lambda *a, **k: None
    return p


def install():
    """把 stub 注册进 sys.modules（幂等）"""
    if "machine" not in sys.modules:
        sys.modules["machine"] = _make_machine()
    if "padog" not in sys.modules:
        sys.modules["padog"] = _make_padog()


# ============================================================================
#  PA_SERVO.py 用的「记录型」I2C
# ============================================================================
#
# PA_SERVO.py 在**顶层**就 new I2C 并 new 两个 Servos（会真的写寄存器），
# 所以想拿它当参考值，就必须让 `machine.I2C` / `machine.Pin` 可用。
#
# 但这里**仍然不模拟任何硬件行为** —— RecordingI2C 只做一件事：
# 把「原代码要求硬件做什么」原样记下来。参考值 = 原代码发出的写操作序列，
# 而不是我对硬件的猜测。这正是我们要对照的东西。
#
# `readfrom_mem` 的返回值取自**真机实测**（MODE1=0x21、PRESCALE=122），
# 好让 `freq()` 里的读-改-写走正常路径而不是被 except 兜住。

#: 真机实测值：0x40/0x41 的 MODE1 与 PRESCALE
REAL_MODE1 = 0x21
REAL_PRESCALE = 122

#: LED0_ON_L 起，16 路 × 4 字节
_LED0 = 0x06
_LED_END = _LED0 + 16 * 4


class RecordingI2C:
    """只记录、不模拟。"""

    instances = []

    def __init__(self, id=0, scl=None, sda=None, freq=100000, **kw):
        self.id = id
        self.scl = scl
        self.sda = sda
        self.freq = freq
        self.writes = []   # (addr, reg, bytes)
        RecordingI2C.instances.append(self)

    # ---- MicroPython machine.I2C 的接口 ----
    def scan(self):
        return [0x40, 0x41, 0x70]

    def writeto_mem(self, addr, reg, data):
        self.writes.append((int(addr), int(reg), bytes(data)))

    def readfrom_mem(self, addr, reg, n):
        reg = int(reg)
        if reg == 0x00:
            return bytes([REAL_MODE1]) + bytes(max(0, n - 1))
        if reg == 0xFE:
            return bytes([REAL_PRESCALE]) + bytes(max(0, n - 1))
        return bytes(n)

    # ---- 给参考值生成器用的查询接口 ----
    def led_writes(self):
        """只保留写 LED 占空比寄存器的操作：[(addr, pca_ch, on, off), ...]

        ⚠️ 高字节**不能**只取低 4 位：bit4 是「全开/全关」标志。
        `PCA9685.duty(index, 0)` 写的是 `pwm(index, 0, 4096)`，
        4096 的高字节是 `0x10` —— 只 & 0x0F 会把它错读成 0。
        （同样的错误曾存在于 C 驱动里，见成长手册 P-21。）
        """
        out = []
        for addr, reg, data in self.writes:
            if _LED0 <= reg < _LED_END and (reg - _LED0) % 4 == 0 and len(data) == 4:
                ch = (reg - _LED0) // 4
                on = data[0] | ((data[1] & 0x1F) << 8)
                off = data[2] | ((data[3] & 0x1F) << 8)
                out.append((addr, ch, on, off))
        return out

    def clear(self):
        self.writes.clear()


class FakePin:
    """
    只记引脚号，不驱动任何东西。

    `PA_SERVO` 顶层要 `Pin(22)`，`padog.py` 顶层要 `Pin(2, Pin.OUT)` 然后
    `led.value(0/1)` —— 所以还得提供 `OUT` / `IN` 这两个类属性与 `value()`。
    """

    OUT = 1
    IN = 0
    PULL_UP = 2
    PULL_DOWN = 3

    def __init__(self, num, mode=None, *a, **kw):
        self.num = num
        self.mode = mode
        self._val = 0
        self.writes = []

    def value(self, v=None):
        if v is None:
            return self._val
        self._val = v
        self.writes.append(v)

    def on(self):
        self.value(1)

    def off(self):
        self.value(0)


def _make_utime():
    """MicroPython 的 utime —— 用真实时钟实现就够了，参考值与时间无关。"""
    import time as _t
    m = types.ModuleType("utime")
    m.ticks_ms = lambda: int(_t.monotonic() * 1000) & 0x3FFFFFFF
    m.ticks_us = lambda: int(_t.monotonic() * 1000000) & 0x3FFFFFFF
    m.ticks_diff = lambda a, b: a - b
    m.ticks_add = lambda a, b: a + b
    m.sleep_ms = lambda ms: _t.sleep(ms / 1000.0)
    m.sleep_us = lambda us: None
    m.sleep = lambda s: _t.sleep(s)
    return m


# ============================================================================
#  可控时钟 —— 给"会读 utime.ticks_ms()"的原代码用（姿态动画层）
# ============================================================================
#
# `padog.py` 的纯数学部分（PA_IK / PA_TROT / PA_WALK / servo_output）不读时钟，
# 所以 `_make_utime()` 用真实时钟就够了。但**姿态动画层**不一样：
#   `_pose_anim_begin()` / `_pose_anim_step()` 读 `utime.ticks_ms()`，
#   `_wait_pose_anim_done()` 还会 `while pose_anim_active: mainloop(); time.sleep_ms(20)`，
#   `mainloop()` 开头的 `inplace_step_end_ms` 服务也要比时刻。
#
# 用真实时钟会同时踩两个坑：
#   1. 参考值**不可复现** —— 插值系数 `t = (now-start)/total` 每次都不一样；
#   2. `_wait_pose_anim_done()` 会真的睡 900 ms 甚至更久，golden 生成变得很慢。
#
# 为什么钉住时钟**仍然忠实**：原代码只用 `ticks_ms()` / `ticks_add()` / `ticks_diff()`
# 三个操作，语义是"单调递增的整数毫秒 + 环绕安全的差值"。下面这个假时钟给出的
# 就是同样的语义，而且把真实休眠的抖动去掉了 —— 板子上真正的 tick 序列是
# `t0, t0+20, t0+40, ...`，假时钟给出的正是这个序列的**标称值**。
# 被测代码里没有任何"依赖真实流逝时间"的逻辑（除 sleep 本身），所以这是
# **更强的可复现性**，不是更弱的忠实度。
#
# 与板子唯一的差别：MicroPython 的 `ticks_ms()` 会在 2^30 附近回绕，这里不回绕。
# 测试模拟的总时长只有几秒、起点固定在 10 万毫秒级，**回绕路径不可达**
# （原代码也不会跨过它）。


class ControllableClock:
    """假时钟：`ticks_ms()` 返回一个由生成器**显式推进**的整数毫秒计数。"""

    def __init__(self, now_ms=0):
        self.now_ms = int(now_ms)

    def set(self, now_ms):
        self.now_ms = int(now_ms)

    def advance(self, ms):
        self.now_ms += int(ms)

    def ticks_ms(self):
        return self.now_ms

    def ticks_us(self):
        return self.now_ms * 1000

    def sleep_ms(self, ms):
        self.advance(ms)

    def sleep_us(self, us):
        self.advance(int(us) // 1000)

    def sleep(self, s):
        self.advance(int(round(float(s) * 1000.0)))


def install_controllable_clock(now_ms=0):
    """
    把 `sys.modules['utime']` 换成可控时钟，并给 CPython 的 `time` 补上 `sleep_ms()`。

    两处 monkeypatch 的理由：

    * `sys.modules['utime']` —— `padog.py` 顶层 `import utime`，拿到的是
      `sys.modules` 里那**一个**对象；换掉它，`ns['utime']` 就是假时钟。
      （`install_for_mainloop()` 只在名字不存在时才建 `utime`，所以不会被覆盖。）
    * `time.sleep_ms` —— 原代码写的是 `time.sleep_ms(20)`，MicroPython 的 `time`
      有这个方法（CPython 没有；`install_for_servo()` 只补了 `sleep_us`）。
      必须让它**推进同一个时钟**，否则 `_wait_pose_anim_done()` 的循环会
      要么立刻跑完（不推进时间 → 死循环或次数不对）、要么真的睡 900 ms。

    ⚠️ 必须**在 `load_padog_ns()` 之前**调用。
    """
    clock = ControllableClock(now_ms)

    m = types.ModuleType("utime")
    m.ticks_ms = clock.ticks_ms
    m.ticks_us = clock.ticks_us
    m.ticks_add = lambda a, b: a + b
    m.ticks_diff = lambda a, b: a - b
    m.sleep_ms = clock.sleep_ms
    m.sleep_us = clock.sleep_us
    m.sleep = clock.sleep
    sys.modules["utime"] = m

    import time as _t
    _t.sleep_ms = clock.sleep_ms
    _t.sleep = clock.sleep
    return clock


def _make_mech_arm():
    """机械臂：mainloop 里调它的 tick()，与运动数学无关。"""
    m = types.ModuleType("mech_arm")
    m.tick = lambda *a, **k: None
    m.init_arm_pose = lambda *a, **k: None
    m.init_grip = lambda *a, **k: None
    return m


def _make_network():
    m = types.ModuleType("network")
    m.STA_IF = 0
    m.AP_IF = 1
    return m


def install_for_mainloop():
    """
    为 exec **整个 padog.py** 准备环境。

    与 `install_for_servo()` 的区别：这次要跑的是主循环，所以还要
    `utime` / `mech_arm` / `network` 这几个 MicroPython 侧的名字，
    以及能做 `Pin(2, Pin.OUT).value(0)` 的引脚 stub。

    注意：**仍然不模拟任何硬件行为**。参考值 = 原代码要求硬件做的事。
    """
    install_for_servo()
    for name, factory in (("utime", _make_utime),
                          ("mech_arm", _make_mech_arm),
                          ("network", _make_network)):
        if name not in sys.modules:
            sys.modules[name] = factory()


def load_padog_source(config_py_text):
    """
    读 `micropython/padog.py`，并处理它模块级的
        exec(open('config.py').read())
        exec(open('config_s.py').read())
    这两行：

    - `config_s.py` 用**仓库里的真文件**（它与板子备份逐字节相同）。
    - `config.py` 公开仓库里只有脱敏的 `config.example.py`（真文件含 WiFi 凭据，
      在 gitignore 的备份目录里）。数学路径需要的键（`Ts`/`faai`/`pit_max_ang`/
      `rol_max_ang`/`xs_max`）example 里全都有。
    - `do_connect_STA()` / `do_connect_AP()` 这两行**必须注释掉**：
      前者是 `while not wifi.isconnected(): pass` 的死循环（成长手册 P-16），
      在电脑上会直接挂住。这一段与运动数学毫无关系。
    """
    src = (MPY_DIR / "padog.py").read_text(encoding="utf-8")
    # ⚠️ 必须显式带上 encoding='utf-8'：
    #    padog.py 里是 `open('config.py').read()`，用的是**系统默认编码**。
    #    在板子上是 UTF-8 所以没事；在中文 Windows 上是 GBK，会直接
    #    UnicodeDecodeError（这正是成长手册 P-01 那个坑的另一副面孔）。
    #    我们的替身文件是 UTF-8 写的，所以读取端也必须是 UTF-8。
    src = src.replace("open('config.py')",
                      "open(%r, encoding='utf-8')" % str(config_py_text))
    src = src.replace("open('config_s.py')",
                      "open(%r, encoding='utf-8')" % str(MPY_DIR / "config_s.py"))
    return src


def _neutralize_wifi_calls(text):
    """把 config.py 里的 do_connect_STA(...) / do_connect_AP() 调用注释掉。"""
    out = []
    for line in text.splitlines():
        s = line.lstrip()
        if s.startswith("do_connect_STA(") or s.startswith("do_connect_AP("):
            out.append("# [golden] 已注释：" + line)
        else:
            out.append(line)
    return "\n".join(out) + "\n"



def install_for_servo():
    """把 machine.I2C / machine.Pin 换成可记录的版本，并清空记录。"""
    install()

    # `ustruct` 就是 MicroPython 的 `struct`，直接别名过去
    import struct as _struct
    sys.modules.setdefault("ustruct", _struct)

    # CPython 的 time 没有 sleep_us()（MicroPython 有）；补一个空实现。
    # 它只是 PCA9685 手册要求的"退出 sleep 后等 5 µs"，与参考值无关。
    import time as _time
    if not hasattr(_time, "sleep_us"):
        _time.sleep_us = lambda us: None  # type: ignore[attr-defined]

    m = sys.modules["machine"]
    m.I2C = RecordingI2C
    m.Pin = FakePin
    RecordingI2C.instances = []

