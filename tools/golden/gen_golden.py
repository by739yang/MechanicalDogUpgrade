#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_golden.py —— 从原始 MicroPython 模块生成 golden 参考向量表

思路：**直接 exec 原始的 micropython/*.py**，让参考值 100% 来自原实现，
而不是我手抄或重新推导。原始模块只 import math（少数需要 machine/padog stub），
所以能在 CPython 里直接跑。

输出：tools/golden/golden/<suite>.csv
      这些 CSV 会提交进仓库 —— C 版测试只读 CSV，不需要装 Python。

用法：
    python tools/golden/gen_golden.py
"""
import os
import sys
import math
import random
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MPY = ROOT / "micropython"
OUT = Path(__file__).resolve().parent / "golden"

# 真实几何参数（config_s.py 实测值）
L1 = 130.0
L2 = 138.0

# 可达域留 1 mm 余量：正好落在边界上会让 acos 因浮点误差越界抛 ValueError
RMIN = abs(L1 - L2) + 1.0
RMAX = L1 + L2 - 1.0


def load_module(filename):
    """在独立命名空间里 exec 一个 MicroPython 模块，返回其全局命名空间。"""
    path = MPY / filename
    if not path.is_file():
        raise SystemExit("找不到 %s" % path)
    ns = {"__name__": filename[:-3], "__file__": str(path)}
    src = path.read_text(encoding="utf-8")
    exec(compile(src, filename, "exec"), ns)
    return ns


def reachable(x, y):
    r = math.hypot(x, y)
    return RMIN <= r <= RMAX


def ik_samples(n_random=140, seed=20260919):
    """生成一批 (x, y) 采样点，覆盖 x>0 / x<0 / x==0 三个分支与边界附近。"""
    pts = []

    # 1) 显式边界与特殊情况
    pts += [
        (0.0, -140.0),        # x==0 -> 走 else 分支
        (0.0, -RMIN - 0.5),   # 接近最小可达
        (0.0, -RMAX + 0.5),   # 接近最大可达
        (RMAX - 6.0, 0.0),    # 水平伸出（x>0）
        (-(RMAX - 6.0), 0.0), # 水平伸出（x<0）
        (5.0, -140.0),
        (-5.0, -140.0),
        (60.0, -100.0),
        (-60.0, -100.0),
        (100.0, -180.0),
        (-100.0, -180.0),
    ]

    # 2) 固定种子的随机采样（可复现）
    rnd = random.Random(seed)
    tries = 0
    while len(pts) < n_random + 11 and tries < 20000:
        tries += 1
        x = rnd.uniform(-RMAX * 0.95, RMAX * 0.95)
        y = rnd.uniform(-RMAX * 0.99, -20.0)
        if reachable(x, y):
            pts.append((x, y))

    # 只保留可达点
    ok = [p for p in pts if reachable(*p)]
    return ok


def gen_ik(ns, fh):
    """PA_IK.ik —— 串联腿(case=0) 与 并联腿(case=1)"""
    ik = ns["ik"]
    pts = ik_samples()
    print("  IK 采样点: %d 个（可达域 %.1f ~ %.1f mm）" % (len(pts), RMIN, RMAX))

    fh.write("# PA_IK.ik golden vectors\n")
    fh.write("# 由 tools/golden/gen_golden.py 从 micropython/PA_IK.py 直接 exec 生成\n")
    fh.write("# 列: case,l1,l2,x1,x2,x3,x4,y1,y2,y3,y4,ham1,ham2,ham3,ham4,shank1,shank2,shank3,shank4\n")
    fh.write("case,l1,l2,x1,x2,x3,x4,y1,y2,y3,y4,"
             "ham1,ham2,ham3,ham4,shank1,shank2,shank3,shank4\n")

    rows = 0
    skipped = 0
    for case in (0, 1):
        # 每次取 4 个点组成一组（同一组内四腿各自独立，正好覆盖不同分支）
        for i in range(0, len(pts) - 3, 4):
            grp = pts[i:i + 4]
            if len(grp) < 4:
                break
            xs = [round(p[0], 6) for p in grp]
            ys = [round(p[1], 6) for p in grp]
            try:
                out = ik(case, L1, L2, xs[0], xs[1], xs[2], xs[3],
                         ys[0], ys[1], ys[2], ys[3])
            except (ValueError, ZeroDivisionError) as e:
                skipped += 1
                continue
            if any(v is None or (isinstance(v, float) and math.isnan(v)) for v in out):
                skipped += 1
                continue
            fh.write("%d,%.6f,%.6f,%s,%s,%s\n" % (
                case, L1, L2,
                ",".join("%.6f" % v for v in xs),
                ",".join("%.6f" % v for v in ys),
                ",".join("%.9f" % v for v in out),
            ))
            rows += 1

    print("  IK golden 行数: %d（跳过 %d 行超出定义域）" % (rows, skipped))
    return rows


def main():
    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    OUT.mkdir(parents=True, exist_ok=True)
    print("仓库根: %s" % ROOT)
    print("输出目录: %s" % OUT)

    print("\n[1/1] IK ...")
    ns_ik = load_module("PA_IK.py")
    with open(OUT / "ik.csv", "w", encoding="utf-8", newline="\n") as fh:
        n = gen_ik(ns_ik, fh)

    print("\n完成。共 1 个 suite, %d 行。" % n)
    print("提示：这些 CSV 要提交进仓库，C 版测试只读它们。")


if __name__ == "__main__":
    main()
