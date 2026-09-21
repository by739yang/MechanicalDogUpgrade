#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一次性探测：装上 stub 后，哪些 MicroPython 模块能在 CPython 里加载并调用"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mpy_stubs

ROOT = HERE.parents[1]
MPY = ROOT / "micropython"
mpy_stubs.install()


def load(name):
    ns = {"__name__": name[:-3], "__file__": str(MPY / name)}
    exec(compile((MPY / name).read_text(encoding="utf-8"), name, "exec"), ns)
    return ns


for name in ("PA_IK.py", "PA_ATTITUDE.py", "PA_TROT.py", "PA_WALK.py", "PA_AVGFILT.py"):
    try:
        ns = load(name)
        keys = [k for k in ns if not k.startswith("__")]
        print("OK    %-18s 顶层名字: %s" % (name, ", ".join(sorted(keys)[:10])))
    except Exception as e:
        print("FAIL  %-18s %r" % (name, e))

print()
print("--- 探测 cal_t / cal_w / avg_filiter ---")
try:
    ns = load("PA_TROT.py")
    ns["Ts"] = 1.0
    ns["faai"] = 0.42
    r = ns["cal_t"](0.3, 0.0, 38.0, 38.7, 1.0, 1.0, 1.0, 1.0)
    print("cal_t  ->", tuple(round(v, 6) for v in r))
except Exception as e:
    print("cal_t FAIL %r" % (e,))

try:
    ns = load("PA_WALK.py")
    r = ns["cal_w"](0.0, 28.0, 230.0, 38.0, 38.7, 0.3, 1.0, 1.0, 1.0, 1.0)
    print("cal_w  ->", tuple(round(v, 6) for v in r))
except Exception as e:
    print("cal_w FAIL %r" % (e,))

try:
    ns = load("PA_AVGFILT.py")
    import array
    f = ns["avg_filiter"](array.array("i", [0] * 5))
    print("avg_filiter 对象方法:", [m for m in dir(f) if not m.startswith("_")])
    print("avg(10) ->", f.avg(10))
except Exception as e:
    print("PA_AVGFILT FAIL %r" % (e,))
