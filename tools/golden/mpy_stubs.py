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
