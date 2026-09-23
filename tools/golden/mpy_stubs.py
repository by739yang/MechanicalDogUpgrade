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
    """只记引脚号。PA_SERVO 顶层 I2C(0, scl=Pin(22), sda=Pin(21)) 需要它能被构造。"""

    def __init__(self, num, *a, **kw):
        self.num = num


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

