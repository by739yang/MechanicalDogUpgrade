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

# 让 PA_TROT / PA_WALK 这类 import machine / padog 的纯数学模块也能加载
sys.path.insert(0, str(Path(__file__).resolve().parent))
import mpy_stubs  # noqa: E402

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


def gen_body_pose(ns, fh):
    """PA_ATTITUDE.cal_ges —— 俯仰/滚转/X偏移 -> 四足足端目标

    ⚠️ 注意原实现返回顺序是 (AB1_x, AB2_x, AB4_x, AB3_x, AB1_z, AB2_z, AB4_z, AB3_z)
       —— 第 3、4 项对应**腿4 和 腿3**（交换过）。CSV 按位置记录，C 版照样复刻。
    """
    cal_ges = ns["cal_ges"]

    # 真实几何与姿态限幅（config_s.py: l=230 b=120 w=220；config.py: pit/rol_max_ang=15）
    L, B, W = 230.0, 120.0, 220.0
    PIT_MAX, ROL_MAX = 15.0, 15.0

    fh.write("# PA_ATTITUDE.cal_ges golden vectors\n")
    fh.write("# 由 tools/golden/gen_golden.py 从 micropython/PA_ATTITUDE.py 直接 exec 生成\n")
    fh.write("# 列: pit,rol,l,b,w,x,hc,x1,x2,x3,x4,y1,y2,y3,y4\n")
    fh.write("# 注意: 输出的 x3 对应腿4, x4 对应腿3（原实现交换过）\n")
    fh.write("pit,rol,l,b,w,x,hc,x1,x2,x3,x4,y1,y2,y3,y4\n")

    cases = []
    # 1) 显式覆盖：零姿态、单轴极值、双轴组合、X 偏移极值
    for pit in (0.0, PIT_MAX, -PIT_MAX):
        for rol in (0.0, ROL_MAX, -ROL_MAX):
            for xoff in (0.0, 40.0, -40.0):
                cases.append((pit, rol, xoff))
    # 2) 固定种子随机（在限幅内）
    rnd = random.Random(20260920)
    for _ in range(60):
        cases.append((
            round(rnd.uniform(-PIT_MAX, PIT_MAX), 4),
            round(rnd.uniform(-ROL_MAX, ROL_MAX), 4),
            round(rnd.uniform(-60.0, 60.0), 4),
        ))
    # 3) 几组不同的站高（Hc 变化）
    hs = [169.0, 200.0, 219.0, 259.0]

    rows = 0
    for i, (pit, rol, xoff) in enumerate(cases):
        hc = hs[i % len(hs)]
        out = cal_ges(pit, rol, L, B, W, xoff, hc)
        if any(v is None or math.isnan(v) for v in out):
            continue
        fh.write("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n" % (
            pit, rol, L, B, W, xoff, hc,
            ",".join("%.9f" % v for v in out),
        ))
        rows += 1

    print("  body_pose golden 行数: %d" % rows)
    return rows


def gen_gait_trot(ns, fh):
    """PA_TROT.cal_t —— TROT 小跑步态轨迹

    ⚠️ 原实现的形参顺序是 `(t, xs, xf, h, r1, r4, r2, r3)` —— 注意是 r1,r4,r2,r3，
       不是 r1,r2,r3,r4。CSV 按这个原顺序记录，C 版也保持同样签名。

    ⚠️ `cal_t` 读的是模块级全局 `Ts` / `faai`，而 padog.py 运行时会用
       `_sync_pa_step_timing()` 从 config 同步进来。实测运行值：
         Ts   = 1.0   （config.py）
         faai = 0.42  （config_s.py 覆盖了 config.py 的 0.5）
       所以 CSV 里把 ts / faai 作为显式输入记录，C 版也改成显式参数（消除隐藏状态）。
    """
    cal_t = ns["cal_t"]

    # (ts, faai)：实机运行值 + 模块默认值 + 两个变化值，检验对时序参数的敏感性
    timing = [(1.0, 0.42), (1.0, 0.5), (1.0, 0.30), (0.8, 0.42)]

    # (xs, xf, h)：xf 来自 padog 的 `spd*10*0.90*_xgs`，h 来自 `h*0.96*...`
    # 实机 spd=3 时约 xf=38.2 / h=38.7
    motion = [
        (0.0, 38.2, 38.7),     # 接近实机巡航
        (0.0, 60.0, 65.0),     # 大步幅
        (-20.0, 40.0, 50.0),   # 起止点不同
        (10.0, -40.0, 30.0),   # 反向
        (0.0, 0.0, 40.0),      # 零步幅（原地抬腿）
    ]

    # 腿系数（自然顺序 leg1..leg4）。Python 形参是 (r1,r4,r2,r3)，下面按此重排。
    rsets = [
        (1.0, 1.0, 1.0, 1.0),
        (1.0, 1.0, -1.0, -1.0),
        (0.0, 1.0, 0.0, 1.0),
        (-1.0, 1.0, -1.0, 1.0),
    ]

    fh.write("# PA_TROT.cal_t golden vectors\n")
    fh.write("# 由 tools/golden/gen_golden.py 从 micropython/PA_TROT.py 直接 exec 生成\n")
    fh.write("# 列: ts,faai,t,xs,xf,h,r1,r4,r2,r3,x1,x2,x3,x4,y1,y2,y3,y4\n")
    fh.write("# 注意: r 的顺序与原实现一致，是 r1,r4,r2,r3\n")
    fh.write("ts,faai,t,xs,xf,h,r1,r4,r2,r3,x1,x2,x3,x4,y1,y2,y3,y4\n")

    rows = 0
    skipped = 0
    for (ts, faai) in timing:
        ns["Ts"] = ts
        ns["faai"] = faai
        stance = faai * ts
        swing = ts - stance

        # 时间采样：显式覆盖两个分支与三个边界（t=0、t=faai*Ts、t=Ts）
        tset = [0.0, stance]
        for f in (0.1, 0.3, 0.5, 0.7, 0.9, 0.99):
            tset.append(stance * f)
        for f in (0.01, 0.25, 0.5, 0.75, 0.99):
            tset.append(stance + swing * f)
        tset = sorted(set(round(v, 9) for v in tset))

        for t in tset:
            if t > ts:
                continue
            for (xs, xf, h) in motion:
                for (l1r, l2r, l3r, l4r) in rsets:
                    try:
                        out = cal_t(t, xs, xf, h, l1r, l4r, l2r, l3r)
                    except Exception:
                        skipped += 1
                        continue
                    fh.write("%.6f,%.6f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n" % (
                        ts, faai, t, xs, xf, h, l1r, l4r, l2r, l3r,
                        ",".join("%.9f" % v for v in out),
                    ))
                    rows += 1

    print("  gait_trot golden 行数: %d（跳过 %d）" % (rows, skipped))
    return rows


def gen_moving_avg(ns, fh):
    """PA_AVGFILT.avg_filiter —— 滑动平均（**有状态**，所以测的是一串调用序列）

    原实现的一个关键细节：窗口大小 = `len(cache_data) - 3`。
    cache 的前 3 个槽被挪用为「长度标记 / 累加和 / 未用」，真正的数据窗口在后面：
      - `array('i',[0]*5)`  → 窗口 **2**（PA_STABLIZE 用）
      - `array('i',[0]*10)` → 窗口 **7**（PA_WALK 用）

    另一个细节：返回的是 `self.cache[1] // (len-3)` —— **整数向下取整**。
    Python 的 `//` 对负数向 -∞ 取整，C 的 `/` 向 0 截断，两者不同，必须复刻。
    """
    from array import array

    cls = ns["avg_filiter"]

    # (cache 长度, 说明)：分别对应 PA_STABLIZE 和 PA_WALK 的实际用法
    setups = [
        (5, "PA_STABLIZE / PA_AVGFILT 默认用法 -> 窗口 2"),
        (10, "PA_WALK 用法 -> 窗口 7"),
    ]

    fh.write("# PA_AVGFILT.avg_filiter golden vectors\n")
    fh.write("# 由 tools/golden/gen_golden.py 从 micropython/PA_AVGFILT.py 直接 exec 生成\n")
    fh.write("# ⚠️ 这是一串**调用序列**：同一 window 的行必须按 step 递增顺序喂给同一个滤波器实例\n")
    fh.write("# 列: window,step,in,out\n")
    fh.write("# window = 窗口大小 = len(cache_data) - 3\n")
    fh.write("# out 是整数（Python 的 // 向下取整）\n")
    fh.write("window,step,in,out\n")

    rows = 0
    for (clen, note) in setups:
        fh.write("# %s (cache len=%d)\n" % (note, clen))
        filt = cls(array("i", [0] * clen))
        w = clen - 3

        # 值序列：0、正、负、交替、大值 —— 覆盖累加与负数的向下取整
        vals = [0, 10, 20, 30, -5, -25, 100, 7, -7, 0, 32767, -32768]
        rnd = random.Random(20260921)
        vals += [rnd.randint(-3000, 3000) for _ in range(120)]

        for step, v in enumerate(vals):
            out = filt.avg(v)
            fh.write("%d,%d,%d,%d\n" % (w, step, v, out))
            rows += 1

    print("  moving_avg golden 行数: %d（窗口 2 与 7）" % rows)
    return rows


def gen_gait_walk(ns, fh):
    """PA_WALK.cal_w —— WALK 四足顺序步态

    ⚠️ 这个模块有个**副作用**：`cal_w` 内部会调 `_apply_cg()`，
       而 `_apply_cg()` 会执行 `padog.gesture(0, int(CG_X), int(yst))`
       —— **改的是 padog 的姿态目标**。C 版必须把它变成显式输出。

    ⚠️ `_body_h()` 读 `padog.R_H`，`_read_gyro_p()` 在无 IMU 时返回 0。
       两者都通过 stub 控制，以便生成确定性的参考值。

    ⚠️ 形参顺序同样是 `(..., t, r1, r4, r2, r3)`。
    """
    cal_w = ns["cal_w"]
    padog = sys.modules["padog"]

    captured = {}

    def recorder(pit, rol, x):
        captured["pit"] = pit
        captured["rol"] = rol
        captured["x"] = x

    padog.gesture = recorder

    # 参数空间较大，用固定种子采样而不是全笛卡尔积，控制 CSV 规模
    timing = [(1.0, 0.30), (1.0, 0.42), (1.0, 0.5)]   # 实机 walk_faai=0.30
    rnd = random.Random(20260922)

    fh.write("# PA_WALK.cal_w golden vectors\n")
    fh.write("# 由 tools/golden/gen_golden.py 从 micropython/PA_WALK.py 直接 exec 生成\n")
    fh.write("# 列: ts,faai,cg_x,cg_y,l,xf,h,t,r1,r4,r2,r3,body_h,gyro,"
             "x1,x2,x3,x4,y1,y2,y3,y4,gpit,grol,gx\n")
    fh.write("# gpit/grol/gx 是原实现 padog.gesture(0, int(CG_X), int(yst)) 的三个实参\n")
    fh.write("# grol = int(cg_x) 向零截断；gx = int(yst)，注意不是四舍五入\n")
    fh.write("ts,faai,cg_x,cg_y,l,xf,h,t,r1,r4,r2,r3,body_h,gyro,"
             "x1,x2,x3,x4,y1,y2,y3,y4,gpit,grol,gx\n")

    cases = []
    # 腿系数（自然顺序 leg1..leg4）。**必须含非对称值**，否则测不出原实现
    # 形参 (r1,r4,r2,r3) 的错位映射 —— 全是 1 的话映射错了也照样"通过"。
    rsets = [
        (1.0, 1.0, 1.0, 1.0),
        (1.0, -1.0, 0.5, -2.0),
        (0.0, 1.0, -1.0, 0.0),
        (-1.5, 1.0, 1.0, -1.5),
    ]
    # 1) 显式边界：xf=0（走 _xs_xf 特例）、cg_x 负数（测 int() 向零截断 vs 地板）
    for (ts, faai) in timing:
        cycle = 4.0 * faai * ts
        for tf in (0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 0.999):
            for ri in (0, 1):
                cases.append((ts, faai, cycle * tf, 0.0, 0.0, 38.7, 110.0, 0.0, ri))
                cases.append((ts, faai, cycle * tf, 38.2, -7.5, 60.0, 140.0, -3.0, ri))
    # 2) 随机采样：覆盖三个 _apply_cg 分支、不同步幅/高度/重心/陀螺/腿系数
    for _ in range(900):
        ts, faai = timing[rnd.randrange(len(timing))]
        cycle = 4.0 * faai * ts
        cases.append((
            ts, faai,
            round(rnd.uniform(0.0, cycle * 0.999), 6),
            rnd.choice([0.0, 20.0, 38.2, -30.0, 60.0]),
            rnd.choice([0.0, 12.0, -7.5, 30.0]),
            rnd.choice([38.7, 50.0, 65.0]),
            rnd.choice([110.0, 140.0, 180.0]),
            rnd.choice([0.0, 0.0, 2.0, -3.0, 5.0]),
            rnd.randrange(len(rsets)),
        ))

    rows = 0
    skipped = 0
    for (ts, faai, t, xf, cg_x, h, bh, gy, ri) in cases:
        l1r, l2r, l3r, l4r = rsets[ri]
        ns["Ts"] = ts
        ns["faai"] = faai
        padog.R_H = bh
        ns["_read_gyro_p"] = (lambda g: (lambda: g))(gy)
        captured.clear()
        try:
            # 按原实现形参顺序传：(..., t, r1, r4, r2, r3)
            out = cal_w(cg_x, 28.0, 230.0, xf, h, t, l1r, l4r, l2r, l3r)
        except Exception:
            skipped += 1
            continue
        if not captured:
            skipped += 1
            continue
        fh.write(
            "%.6f,%.6f,%.4f,%.1f,%.1f,%.6f,%.6f,%.6f,"
            "%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%s,%d,%d,%d\n" % (
                ts, faai, cg_x, 28.0, 230.0, xf, h, t,
                l1r, l4r, l2r, l3r, bh, gy,
                ",".join("%.9f" % v for v in out),
                captured["pit"], captured["rol"], captured["x"],
            ))
        rows += 1

    padog.gesture = lambda *a, **k: None
    print("  gait_walk golden 行数: %d（跳过 %d）" % (rows, skipped))
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

    # PA_TROT / PA_WALK 顶层会 import machine / padog，先装上最小 stub
    mpy_stubs.install()

    total = 0

    print("\n[1/5] PA_IK -> kinematics.c ...")
    ns_ik = load_module("PA_IK.py")
    with open(OUT / "ik.csv", "w", encoding="utf-8", newline="\n") as fh:
        total += gen_ik(ns_ik, fh)

    print("\n[2/5] PA_ATTITUDE -> body_pose.c ...")
    ns_att = load_module("PA_ATTITUDE.py")
    with open(OUT / "body_pose.csv", "w", encoding="utf-8", newline="\n") as fh:
        total += gen_body_pose(ns_att, fh)

    print("\n[3/5] PA_TROT -> gait_trot.c ...")
    ns_trot = load_module("PA_TROT.py")
    with open(OUT / "gait_trot.csv", "w", encoding="utf-8", newline="\n") as fh:
        total += gen_gait_trot(ns_trot, fh)

    print("\n[4/5] PA_AVGFILT -> filter_moving_avg.c ...")
    ns_flt = load_module("PA_AVGFILT.py")
    with open(OUT / "moving_avg.csv", "w", encoding="utf-8", newline="\n") as fh:
        total += gen_moving_avg(ns_flt, fh)

    print("\n[5/5] PA_WALK -> gait_walk.c ...")
    ns_walk = load_module("PA_WALK.py")
    with open(OUT / "gait_walk.csv", "w", encoding="utf-8", newline="\n") as fh:
        total += gen_gait_walk(ns_walk, fh)

    print("\n完成。共 5 个 suite, %d 行。" % total)
    print("提示：这些 CSV 要提交进仓库，C 版测试只读它们。")


if __name__ == "__main__":
    main()
