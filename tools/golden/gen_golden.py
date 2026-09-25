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
import ast
import os
import sys
import math
import random
import struct
import types
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


# ============================================================================
#  P2: 舵机映射层 (servo_map.c)
# ============================================================================
#
# 这一层有两段原代码要对照，取法各不相同：
#
# 1. 角度 -> 占空比 -> (ON, OFF)：**直接 import 真的 PA_SERVO.py**。
#    它的顶层会 new I2C 并 new 两片 Servos，所以用 mpy_stubs 里那个
#    "记录型" I2C —— 它不模拟硬件，只是把原代码要写的寄存器记下来。
#    于是参考值 = 原代码真正会写进 PCA9685 的字节。
#
# 2. 关节角 -> 12 路舵机角：把 `padog.py` 里的 `servo_output()` 及其依赖的
#    几个函数**用 ast 原样抠出来** exec，而不是手抄公式。
#    `_hip_leg_deltas()` 依赖摇杆/相位（属于 P3），这里替换成一个常量提供者；
#    `_crawl_shank_servodelta()` / `cal_test_shank()` / `_clamp_deg()` /
#    `_leg_cfg()` / `_shank_ik_bias()` 都用**原件**。

PADOG_SRC = (MPY / "padog.py")

#: 要抠出来的函数（全部来自 padog.py，不重写）
_PADOG_FUNCS = (
    "_clamp_deg",
    "_leg_cfg",
    "_shank_ik_bias",
    "cal_test_shank",
    "_crawl_active",
    "_crawl_shank_servodelta",
    "servo_output",
)

#: 真机的中位角（config_s.py 实测值），列序 = 腿1..腿4 × 髋/大腿/小腿
REAL_INIT = [
    102.0, 84.0, 92.0,   # 腿1 左前
    96.0, 91.0, 85.0,    # 腿2 右前
    108.0, 78.0, 68.0,   # 腿3 右后
    92.0, 98.0, 102.0,   # 腿4 左后
]


def extract_padog_funcs(names):
    """用 ast 从 padog.py 抠出指定顶层函数的**原始源码**并 exec。"""
    src = PADOG_SRC.read_text(encoding="utf-8")
    tree = ast.parse(src)
    found = {}
    segs = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            segs.append(ast.get_source_segment(src, node))
            found[node.name] = True
    missing = [n for n in names if n not in found]
    if missing:
        raise SystemExit("padog.py 里找不到这些函数（改名了？）: %s" % missing)

    ns = {"__name__": "padog_extracted"}
    exec(compile("\n\n".join(segs), "padog.py[extracted]", "exec"), ns)
    return ns


def load_servo_module():
    """加载真的 PA_SERVO.py（用记录型 I2C），并屏蔽它顶层的 scan 打印。"""
    mpy_stubs.install_for_servo()
    import contextlib
    import io
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ns = load_module("PA_SERVO.py")
    return ns, ns["_i2c_servo"]


class _AttrNS:
    """让 dict 支持属性访问，好把 exec 出来的命名空间当成 PA_SERVO 模块用。"""

    def __init__(self, d):
        self.__dict__.update(d)


def gen_servo_angle(fh, ch_samples, duty_samples):
    """
    角度 -> (addr, pca_ch, on, off)

    CSV 列：kind,ch,val,addr,pca_ch,on,off
      kind=0 表示 angle(ch, val)      —— val 是角度
      kind=1 表示 该通道所在板的 pca9685.duty(pca_ch, val) —— val 是占空比
    """
    ns, rec = load_servo_module()

    rows = 0

    def one(kind, ch, val, fn):
        nonlocal rows
        rec.clear()
        fn()
        writes = rec.led_writes()
        if len(writes) != 1:
            raise SystemExit("期望 1 次 LED 写，实际 %d 次：ch=%s val=%s" % (len(writes), ch, val))
        addr, pca_ch, on, off = writes[0]
        fh.write("%d,%d,%.4f,%d,%d,%d,%d\n" % (kind, ch, val, addr, pca_ch, on, off))
        rows += 1

    for ch, deg in ch_samples:
        one(0, ch, deg, lambda ch=ch, deg=deg: ns["angle"](ch, deg))

    # 占空比的三个特殊分支：0（无脉冲）、4095（全开）、以及中间值
    for ch, duty in duty_samples:
        addr = 0x40 if ch < 6 else 0x41
        pca_ch = ch if ch < 6 else ch - 6
        dev = ns["servos40"] if ch < 6 else ns["servos41"]
        one(1, ch, float(duty),
            lambda dev=dev, pca_ch=pca_ch, duty=duty: dev.pca9685.duty(pca_ch, duty))

    print("  servo_angle golden 行数: %d" % rows)
    return rows


def gen_servo_output(fh, cases, tmpdir):
    """
    关节角 -> 12 路舵机输出

    ⚠️ 参考值用的是**真版 padog.py 的命名空间**（不是手搭的）。
    第一版是 `extract_padog_funcs` 手工拼命名空间，结果漏掉了 padog.py
    第 57~81 行的**默认值注入表** —— 于是 `shank_ik_bias_per_mm` 取不到，
    `_shank_ik_bias()` 退到死代码兜底值 0.375，而 C 版也硬编码 0.375，
    **两边错在同一个地方，测试全绿**。同类教训见成长手册 P-21 / P-22。

    现在 `hip` 与 `cs` 也由真版函数算出来再写进 CSV
    （`_hip_leg_deltas()` / `_crawl_shank_servodelta()`）——
    手填的输入列 = 将来某天的假 FAIL 或假 PASS。

    CSV 列（逗号分隔）：
      ik, ROL_S, PIT_S, joy_turn, crawl, h1..h4, ham×4, sh×4, cs×4,
      init×12, trim×4, l1, l2, ref, 然后 12 组 (on,off) —— 顺序为**逻辑通道 0..11**
    """
    LOGICAL = [(0x40, 0), (0x40, 1), (0x40, 2), (0x40, 3), (0x40, 4), (0x40, 5),
               (0x41, 0), (0x41, 1), (0x41, 2), (0x41, 3), (0x41, 4), (0x41, 5)]

    rows = 0
    for c in cases:
        ns = load_padog_ns(tmpdir)
        rec = sys.modules["PA_SERVO"]._i2c_servo

        ns["l1"] = c["l1"]
        ns["l2"] = c["l2"]
        ns["leg_len_ref"] = c["ref"]
        ns["ROL_S"] = c["rol_s"]
        ns["PIT_S"] = c["pit_s"]
        ns["joy_turn"] = c["joy_turn"]
        ns["crawl_phase"] = c["crawl"]

        # 中位角在 padog.py 里是模块级全局（来自 config_s.py），不是形参
        _joints = ("p", "h", "s")   # 髋 / 大腿 / 小腿
        for i, val in enumerate(c["init"]):
            ns["init_%d%s" % (i // 3 + 1, _joints[i % 3])] = val
        for i in range(4):
            ns["leg%d_s_trim" % (i + 1)] = c["trim"][i]

        # hip 与 cs 由**真版函数**算，保证与参考值实际用到的完全一致
        hip = list(ns["_hip_leg_deltas"]())
        cs = [ns["_crawl_shank_servodelta"](n) for n in (1, 2, 3, 4)]

        rec.clear()
        ns["servo_output"](int(c["mode_case"]), int(c["mode_init"]),
                           c["ham"][0], c["ham"][1], c["ham"][2], c["ham"][3],
                           c["shank"][0], c["shank"][1], c["shank"][2], c["shank"][3])

        got = {(a, ch): (on, off) for (a, ch, on, off) in rec.led_writes()}
        if len(got) != 12:
            raise SystemExit("期望 12 个通道，实际 %d 个：%s" % (len(got), sorted(got)))

        fields = [1 if (int(c["mode_case"]) == 0 and int(c["mode_init"]) == 0) else 0]
        fields += [c["rol_s"], c["pit_s"], c["joy_turn"], c["crawl"]]
        fields += hip
        fields += c["ham"] + c["shank"] + cs
        fields += c["init"] + c["trim"] + [c["l1"], c["l2"], c["ref"]]
        for key in LOGICAL:
            on, off = got[key]
            fields += [on, off]

        fh.write(",".join(
            ("%d" % f) if isinstance(f, int) else ("%.6f" % f) for f in fields
        ) + "\n")
        rows += 1

    print("  servo_output golden 行数: %d  (参考值 = 真版 padog 命名空间)" % rows)
    return rows


def servo_angle_samples():
    """(ch, deg) 与 (ch, duty) 采样：覆盖限幅、分数、负值、边界。"""
    degs = [
        0.0, 1.0, 45.5, 90.0, 90.25, 120.0, 179.0, 180.0,
        181.0, 200.0, 1000.0,          # 越上界 -> 应被限幅
        -1.0, -90.0, -1000.0,          # 越下界 -> 应被限幅
        84.0, 92.0, 102.0, 68.0,       # 真机中位角
        37.123456, 143.987654,
    ]
    chs = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
    ang = [(ch, d) for d in degs for ch in chs]
    duties = [(ch, d) for d in (0, 1, 101, 102, 103, 255, 511, 512, 2048, 4094, 4095)
              for ch in (0, 5, 6, 11)]
    return ang, duties


def servo_output_cases(n_random=60, seed=20260919):
    """生成 servo_output 的输入组合：站姿附近 + 大范围 + 非 IK 路径。"""
    cases = []
    rnd = random.Random(seed)

    def mk(ik, ham, shank, rol_s=0.0, pit_s=0.0, joy_turn=0.0, trim=None, crawl=0,
           l1=130.0, l2=138.0, ref=149.0, init=None):
        # 注意："mode_*" 是 servo_output 的形参 case/init；"init" 是 12 个中位角。
        # 两者同名会互相覆盖，所以显式分开命名。
        # `hip` 与 `cs` **不由这里提供**：它们必须由参考实现自己算出来
        # （`_hip_leg_deltas()` / `_crawl_shank_servodelta()`），
        # 否则 CSV 里写的和参考值实际用到的会是两个数（P-22 踩过一次）。
        # 这里只提供**真正的输入**：ROL_S / PIT_S / joy_turn / crawl_phase。
        return {
            "mode_case": 0 if ik else 1,
            "mode_init": 0 if ik else 1,
            "rol_s": rol_s, "pit_s": pit_s, "joy_turn": joy_turn,
            "ham": list(ham),
            "shank": list(shank),
            "init": list(init) if init else list(REAL_INIT),
            "trim": list(trim) if trim else [0.0, 0.0, 0.0, 0.0],
            "l1": l1, "l2": l2, "ref": ref, "crawl": crawl,
        }

    # 1) 站姿：ham≈90、shank≈40（由中位角反推，见 servo_map.h 注释）
    cases.append(mk(True, [90.0] * 4, [40.0] * 4))
    cases.append(mk(False, [0.0] * 4, [0.0] * 4))
    # 2) 髋辅助偏航：通过**真正的输入** ROL_S / PIT_S / joy_turn 驱动
    #    （`_hip_leg_deltas()` 会自己把它们换算成 h1..h4；顺带也测了这个函数）
    for (rs, ps, jt) in ((5.0, 0.0, 0.0), (-5.0, 3.0, 0.0), (0.0, 0.0, 100.0),
                         (0.0, 0.0, -100.0), (12.0, -8.0, 60.0), (0.0, 0.0, 8.0)):
        cases.append(mk(True, [90.0] * 4, [40.0] * 4, rol_s=rs, pit_s=ps, joy_turn=jt))
        cases.append(mk(False, [0.0] * 4, [0.0] * 4, rol_s=rs, pit_s=ps, joy_turn=jt))
    # 3) 爬行压低增量（由原 _crawl_shank_servodelta 算出：腿1=-20 腿2=+20 腿3=+30 腿4=-30）
    cases.append(mk(True, [90.0] * 4, [40.0] * 4, crawl=1))
    cases.append(mk(True, [70.0] * 4, [55.0] * 4, crawl=1))
    # 4) 小腿微调非零（原 _leg_cfg("s_trim", n)）
    cases.append(mk(True, [90.0] * 4, [40.0] * 4, trim=[2.0, -3.0, 5.0, 0.5]))
    cases.append(mk(True, [110.0] * 4, [20.0] * 4, trim=[-5.0, 5.0, 0.0, 10.0]))
    # 5) 几何不同 -> shank_ik_bias 不同（含 extra<0 的 0 偏置分支）
    for (l1, l2, ref) in ((130.0, 138.0, 149.0), (120.0, 120.0, 149.0), (100.0, 100.0, 149.0)):
        cases.append(mk(True, [90.0] * 4, [40.0] * 4, l1=l1, l2=l2, ref=ref))
    # 6) 四腿各不相同（否则映射错位也测不出来 —— 见成长手册 P-18）
    cases.append(mk(True, [60.0, 90.0, 120.0, 45.0], [10.0, 40.0, 70.0, 100.0]))
    cases.append(mk(True, [130.0, 30.0, 95.0, 150.0], [5.0, 95.0, 35.0, 60.0]))
    # 7) 大范围 / 越界（走占空比限幅）
    cases.append(mk(True, [-40.0, 200.0, 0.0, 400.0], [-50.0, 0.0, 150.0, 300.0]))
    # 8) 中位角被改（证明 C 版确实用了配置里的中位角）
    alt_init = list(REAL_INIT)
    for i in range(12):
        alt_init[i] += (i % 5) * 3.0 - 6.0
    cases.append(mk(True, [90.0] * 4, [40.0] * 4, init=alt_init))
    cases.append(mk(False, [0.0] * 4, [0.0] * 4, init=alt_init))
    # 9) 随机（固定种子，可复现）
    while len(cases) < n_random:
        cases.append(mk(
            rnd.random() < 0.75,
            [rnd.uniform(20.0, 150.0) for _ in range(4)],
            [rnd.uniform(-20.0, 120.0) for _ in range(4)],
            rol_s=rnd.uniform(-12.0, 12.0),
            pit_s=rnd.uniform(-12.0, 12.0),
            joy_turn=rnd.choice([0.0, 8.0, 35.0, -35.0, 100.0, -100.0]),
            trim=[rnd.uniform(-8.0, 8.0) for _ in range(4)],
            crawl=rnd.randint(0, 1),
            l1=rnd.choice([130.0, 125.0, 140.0]),
            l2=rnd.choice([138.0, 130.0, 145.0]),
            ref=rnd.choice([149.0, 145.0, 160.0]),
        ))
    return cases


# ============================================================================
#  P3: 全链路对照 (control_chain.c)
# ============================================================================
#
# 前面每个 suite 都是"一个模块对一个模块"。这一套不一样：它对照的是
# **整条控制链**，而且参考值不是我把模块拼起来，而是 ——
#
#     把 micropython/padog.py **整个 exec 进来，直接调用它的 mainloop()**
#
# 也就是"原版固件真正在做的那一步"。这样连那些**从来没被单独测过**的东西
# 也一起对照了：
#   - 大狗缩放层（_geom_scale / _partial_geom_scale / _ik_hc / 6 个 _LARGE_* 系数）
#   - 姿态 slew 环（R_H / PIT_S / ROL_S / X_S 按 Kp 逼近目标）与限位
#   - 按步态模式与摇杆方向**选择重心分支**的那一大串 if/elif
#   - _hip_leg_deltas / _trot_rol_s / _walk_rol_s / _apply_trot_swing_y …
#
# 输入（CSV 每行）：spd, L, R, gait, t, R_H, H_goal, PIT_S, PIT_goal,
#                   ROL_S, ROL_goal, X_S, X_goal, joy_turn, crawl_phase
# 输出：12 组 (on, off)

#: padog.mainloop() 依赖的、来自 config 的模块级名字（数学路径用到的全部）
_CHAIN_CFG_KEYS = (
    "Ts", "faai", "pit_max_ang", "rol_max_ang", "xs_max",
)


def _chain_config_text():
    """从脱敏模板生成一份可用于对照的 config.py（见 mpy_stubs.load_padog_source）。"""
    text = (MPY / "config.example.py").read_text(encoding="utf-8")
    return mpy_stubs._neutralize_wifi_calls(text)


def load_padog_ns(tmpdir):
    """exec 一次 padog.py，返回它的命名空间（含 mainloop）。"""
    cfg_path = Path(tmpdir) / "config_for_golden.py"
    cfg_path.write_text(_chain_config_text(), encoding="utf-8")

    mpy_stubs.install_for_mainloop()

    # padog.py 里有 `import PA_SERVO` / `import PA_TROT` 等，需要 micropython/ 在 sys.path 上
    if str(MPY) not in sys.path:
        sys.path.insert(0, str(MPY))

    # ---- 把 padog.py **exec 进一个真正的模块对象**，并注册成 sys.modules['padog'] ----
    #
    # 为什么不能像以前那样 exec 进一个普通 dict：`PA_WALK._apply_cg()` 里有
    #     import padog
    #     ...
    #     padog.gesture(0, int(CG_X), int(yst))
    # 而 `padog.gesture()` 会**直接改 `PIT_goal` / `ROL_goal` / `X_goal`**，
    # 且 `cal_w()` 在 mainloop 里的位置**在姿态 slew 环之前** ——
    # 所以原版是**同一帧**就用了被改过的目标。
    #
    # 如果 `sys.modules['padog']` 还是 mpy_stubs 那个空壳（gesture 是 no-op），
    # 参考值就会**漏掉这个副作用**，WALK 那 32 行全是假的，
    # 而 C 版会"精确匹配一个原版并不产生的行为"。这和 P-23 是同一类错误：
    # **参考环境必须是真的。**
    #
    # exec 进模块自己的 `__dict__`（而不是替换 `__dict__`，那是只读属性）之后，
    # `padog.gesture(...)` 改的就是 mainloop 读的那份全局量。
    src = mpy_stubs.load_padog_source(cfg_path)
    mod = types.ModuleType("padog")
    mod.__file__ = str(MPY / "padog.py")
    sys.modules["padog"] = mod
    ns = mod.__dict__
    exec(compile(src, "padog.py", "exec"), ns)

    # ---- 把 WALK 的 IMU 路径**按硬件事实**固定为"关闭" ----
    #
    # `PA_WALK._init_imu()` 在没有 IMU 时会走 except 分支把 acc 置 None，
    # 于是 `_read_gyro_p()` 恒返回 0。本机**确实没有 IMU**（4 次独立实测，
    # 见 HANDOFF.md），所以"gyro_p 恒为 0"就是板子上的真实行为。
    #
    # 为什么不让它自己去失败：那样失败原因会是"stub 缺 sleep_ms"或
    # "记录型 I2C 不会报错所以 accel() 竟然成功了"，两者都与板子无关，
    # 反而可能把 IMU 路径**打开**（板子上它是关的）→ 参考值就错了。
    # 显式关掉 = 与板子一致，且理由写在这里。
    import PA_WALK
    PA_WALK.acc = None
    PA_WALK._f_gyro_p = None
    PA_WALK.gyro_p = 0.0
    PA_WALK._read_gyro_p = lambda: 0.0
    return ns


def gen_control_chain(fh, cases, tmpdir):
    """
    每行输入跑一次原版 mainloop()，记录它写出的 12 组 (ON, OFF)。

    ⚠️ mainloop 会修改 t / R_H / PIT_S / ROL_S / X_S 等全局量，
    所以**每个 case 都重新 exec 一遍 padog.py**（模块级状态归零）。
    这样就不存在"上一条 case 污染下一条"的风险 —— 代价只是多花一两秒。
    """
    LOGICAL = [(0x40, 0), (0x40, 1), (0x40, 2), (0x40, 3), (0x40, 4), (0x40, 5),
               (0x41, 0), (0x41, 1), (0x41, 2), (0x41, 3), (0x41, 4), (0x41, 5)]

    rows = 0
    for c in cases:
        ns = load_padog_ns(tmpdir)
        rec = ns["_i2c_servo"] if "_i2c_servo" in ns else None
        if rec is None:
            # padog 不直接持有 I2C，是 PA_SERVO 顶层建的
            import PA_SERVO  # noqa: F401
            rec = sys.modules["PA_SERVO"]._i2c_servo

        # 注入本 case 的输入
        for k, v in c.items():
            ns[k] = v
        # 这些是 mainloop 里会被读写、且必须处于"干净初值"的量
        ns["stop_run_node"] = 0
        ns["direct_pose_freeze"] = False
        ns["pose_anim_active"] = False
        ns["inplace_step_end_ms"] = 0
        ns["key_stab"] = False

        rec.clear()
        ns["mainloop"]()

        got = {(a, ch): (on, off) for (a, ch, on, off) in rec.led_writes()}
        if len(got) != 12:
            raise SystemExit("case %d 期望 12 路，实际 %d 路：%s" % (
                rows, len(got), sorted(got)))

        fields = [c[k] for k in CHAIN_INPUT_KEYS]
        for key in LOGICAL:
            on, off = got[key]
            fields += [on, off]
        fh.write(",".join(
            ("%d" % f) if isinstance(f, int) else ("%.6f" % f) for f in fields
        ) + "\n")
        rows += 1

    print("  control_chain golden 行数: %d  (参考值 = 原版 mainloop() 真跑一次)" % rows)
    return rows


#: CSV 里的输入列顺序（C 侧按同一顺序解析）
CHAIN_INPUT_KEYS = (
    "spd", "L", "R", "gait_mode", "t", "R_H", "H_goal",
    "PIT_S", "PIT_goal", "ROL_S", "ROL_goal", "X_S", "X_goal",
    "joy_turn", "crawl_phase",
)


def control_chain_cases(n_random=90, seed=20260919):
    """
    生成 mainloop 的输入组合：站立 / 原地踏步 / 前进 / 后退 / 转弯 / WALK / 爬行。

    ⚠️ **`t` 只在 [0, Ts] 内取值**，这不是保守，而是必须：
    `PA_TROT.cal_t()` 只有 `t<=Ts*faai` 与 `t>Ts*faai and t<=Ts` 两个分支、
    **没有 else**（成长手册 P-19），所以 `t > Ts` 会让原版直接
    `UnboundLocalError`。原版永远不会踩到，因为 `mainloop()` 里
    `if t >= Ts: t = t - Ts` 先把相位回绕了。
    C 版**故意**对 `t<0 || t>Ts` 做了回绕（迁移表 §8.10），
    所以喂越界输入等于在测"我自己的改进"，而不是在测等价性。
    ⇒ 全链路对照只在**原版的可达域**内做；越界行为差异单独记录。
    """
    rnd = random.Random(seed)

    def mk(spd, L, R, gait, t=0.0, R_H=81.0, H_goal=81.0,
           PIT_S=0.0, PIT_goal=0.0, ROL_S=0.0, ROL_goal=0.0,
           X_S=18.0, X_goal=18.0, joy_turn=0.0, crawl_phase=0):
        return {
            "spd": float(spd), "L": L, "R": R, "gait_mode": gait, "t": float(t),
            "R_H": float(R_H), "H_goal": float(H_goal),
            "PIT_S": float(PIT_S), "PIT_goal": float(PIT_goal),
            "ROL_S": float(ROL_S), "ROL_goal": float(ROL_goal),
            "X_S": float(X_S), "X_goal": float(X_goal),
            "joy_turn": float(joy_turn), "crawl_phase": int(crawl_phase),
        }

    cases = []
    # 1) 站立（spd=L=R=0）：走"无重心偏置"分支
    cases.append(mk(0, 0, 0, 0))
    cases.append(mk(0, 0, 0, 0, PIT_S=5.0, PIT_goal=5.0, ROL_S=-4.0, ROL_goal=-4.0))
    cases.append(mk(0, 0, 0, 0, R_H=60.0, H_goal=60.0, X_S=0.0, X_goal=0.0))
    # 2) TROT 前进（L=R=1）—— 走 trot_cg_f 分支
    for t in (0.0, 0.1, 0.21, 0.42, 0.5, 0.79, 0.99):
        cases.append(mk(-3.0, 1, 1, 0, t=t))
    # 3) TROT 后退（joy_fwd_sign=-1 时 spd 正即后退）
    for t in (0.0, 0.3, 0.7):
        cases.append(mk(3.0, 1, 1, 0, t=t))
    # 4) 原地踏步（L=R=0 但 spd!=0）—— spd 不为 0，走 trot_cg_t 分支
    for t in (0.0, 0.25, 0.6):
        cases.append(mk(3.0, 0, 0, 0, t=t))
    # 5) 转向（joy_turn 超过 HIP_TURN_DEAD=10）
    for jt in (100.0, -100.0):
        for t in (0.0, 0.35, 0.8):
            cases.append(mk(-2.5, -1, 1, 0, t=t, joy_turn=jt))
    # 6) WALK（gait_mode=1）
    for t in (0.0, 0.12, 0.3, 0.55, 0.88):
        cases.append(mk(-2.0, 1, 1, 1, t=t))
    cases.append(mk(2.0, 1, 1, 1, t=0.4))
    cases.append(mk(2.0, 0, 0, 1, t=0.4))
    # 7) 爬行压低（crawl_phase=1 -> cs 非零）
    for t in (0.0, 0.4):
        cases.append(mk(-3.0, 1, 1, 0, t=t, crawl_phase=1))
    # 8) 姿态 slew 环未到位（目标 != 当前，验证 Kp 逼近这一步）
    cases.append(mk(0, 0, 0, 0, PIT_S=0.0, PIT_goal=12.0, ROL_S=0.0, ROL_goal=-8.0))
    cases.append(mk(0, 0, 0, 0, R_H=150.0, H_goal=70.0, X_S=0.0, X_goal=40.0))
    cases.append(mk(-3.0, 1, 1, 0, t=0.2, PIT_S=-3.0, PIT_goal=3.0, X_S=0.0, X_goal=30.0))
    # 9) 超限（验证 pit_max_ang / rol_max_ang 限位）
    cases.append(mk(0, 0, 0, 0, PIT_S=40.0, PIT_goal=40.0))
    cases.append(mk(0, 0, 0, 0, ROL_S=-40.0, ROL_goal=-40.0))
    # 10) 随机（固定种子，可复现）
    while len(cases) < n_random:
        gait = rnd.randint(0, 1)
        spd = rnd.choice([-4.0, -3.0, -1.5, 0.0, 1.5, 3.0, 4.0])
        lr = rnd.choice([(1, 1), (-1, 1), (1, -1), (-1, -1), (0, 0), (1, 0)])
        cases.append(mk(
            spd, lr[0], lr[1], gait,
            t=rnd.uniform(0.0, 0.95),
            R_H=rnd.uniform(50.0, 110.0), H_goal=rnd.uniform(50.0, 110.0),
            PIT_S=rnd.uniform(-10.0, 10.0), PIT_goal=rnd.uniform(-10.0, 10.0),
            ROL_S=rnd.uniform(-10.0, 10.0), ROL_goal=rnd.uniform(-10.0, 10.0),
            X_S=rnd.uniform(-20.0, 40.0), X_goal=rnd.uniform(-20.0, 40.0),
            joy_turn=rnd.choice([0.0, 35.0, -35.0, 100.0, -100.0]),
            crawl_phase=rnd.randint(0, 1),
        ))
    return cases


# ============================================================================
#  P3: 多帧序列对照 (control_chain_cmd.c)
# ============================================================================
#
# 单帧对照（control_chain.csv）每行都是"干净初值 + 跑一帧"，所以**结构上**测不出：
#   1. 跨帧延续（相位 t、姿态 slew、四个目标是否被保留）；
#   2. 命令语义（move()/gait() 到底重置了哪些量）；
#   3. 长时间漂移。
#
# 这一套把原版 `mainloop()` **连续跑 N 次**，中间按脚本调用原版的
# `move()` / `gait()` / `height()` / `gesture()` / `set_joy_turn()`
# （就是网页摇杆会调的那几个），逐帧记录 12 组占空比。

#: 序列脚本：每条 = (帧号, 函数名, 参数元组)
CHAIN_SEQ_SCRIPTS = {
    # 1) 站立 20 帧 → TROT 前进 120 帧 → 加转向 40 帧 → 回直行 20 帧 → 停 20 帧
    1: [
        (0, "move", (0.0, 0, 0)),
        (20, "move", (-3.0, 1, 1)),
        (140, "set_joy_turn", (100.0,)),
        (180, "set_joy_turn", (0.0,)),
        (200, "move", (0.0, 0, 0)),
    ],
    # 2) WALK：**必须用 drive()，不能用 move()**
    #    ⚠️ 这一条原来写的是 `gait(1)` + `move(...)`，而 `move()` 内部会 `gait(0)`
    #    ⇒ 实际跑的是 TROT（四条腿对角同步）。是**可视化把腿画出来**才发现的：
    #    图上不是四拍顺序。这就是 P-26：**用 move() 无法进入 WALK**，
    #    原版为此专门有 `drive()`（注释："WALK 摇杆用，不切 gait_mode"）。
    2: [
        (0, "drive", (-2.0, 1, 1)),
        (0, "gait", (1,)),
        (80, "drive", (2.0, 1, 1)),
        (120, "drive", (0.0, 0, 0)),
    ],
    # 3) 行进中反复 move()：验"模式没变就不重置相位"（原版 move 里 gait(0) 的行为）
    3: [
        (0, "move", (-3.0, 1, 1)),
        (15, "move", (-3.5, 1, 1)),
        (30, "move", (-4.0, 1, 1)),
        (45, "move", (-3.0, 1, 1)),
        (60, "move", (-2.0, 1, 1)),
    ],
    # 4) 高度与姿态命令：验 height() / gesture() 对目标的作用
    4: [
        (0, "move", (0.0, 0, 0)),
        (10, "height", (60.0,)),
        (40, "gesture", (5.0, -4.0, 30.0)),
        (80, "height", (81.0,)),
        (110, "gesture", (0.0, 0.0, 18.0)),
    ],
    # 5) TROT ↔ WALK 切换：验"模式变了才 t=0"；switching 后必须用 drive() 才留在 WALK
    5: [
        (0, "move", (-3.0, 1, 1)),
        (40, "gait", (1,)),
        (60, "drive", (-3.0, 1, 1)),
        (100, "gait", (0,)),
        (160, "move", (0.0, 0, 0)),
    ],
}

#: 每个序列跑多少帧
CHAIN_SEQ_FRAMES = {1: 240, 2: 140, 3: 100, 4: 140, 5: 200}


#: 动作编号（写进命令 CSV，C 侧按同一张表解释）
CHAIN_SEQ_ACTION_CODES = {
    "move": 0,
    "gait": 1,
    "height": 2,
    "gesture": 3,
    "set_joy_turn": 4,
    "drive": 5,
}


def gen_control_chain_seq(fh, tmpdir):
    """
    多帧序列：每行 = 一帧。

    CSV 列：`seq, frame,` 然后 12 组 (ON, OFF)。

    命令**不写进这个 CSV**，而是另写一份 `control_chain_seq_cmds.csv`
    （`seq, frame, action, a0, a1, a2`），C 侧测试**从文件读**。
    ⇒ 之所以不把脚本硬编码在 C 测试里：那就成了"同一份脚本两个副本"，
    两边一漂移就会产生假 FAIL 或假 PASS（成长手册 P-22 那一类）。
    """
    LOGICAL = [(0x40, 0), (0x40, 1), (0x40, 2), (0x40, 3), (0x40, 4), (0x40, 5),
               (0x41, 0), (0x41, 1), (0x41, 2), (0x41, 3), (0x41, 4), (0x41, 5)]

    rows = 0
    cmd_rows = 0
    with open(OUT / "control_chain_seq_cmds.csv", "w", encoding="utf-8", newline="\n") as cfh:
        for seq_id in sorted(CHAIN_SEQ_SCRIPTS):
            script = CHAIN_SEQ_SCRIPTS[seq_id]
            n_frames = CHAIN_SEQ_FRAMES[seq_id]

            ns = load_padog_ns(tmpdir)
            rec = sys.modules["PA_SERVO"]._i2c_servo

            by_frame = {}
            for (fr, fn, args) in script:
                by_frame.setdefault(fr, []).append((fn, args))
                if fn not in CHAIN_SEQ_ACTION_CODES:
                    raise SystemExit("脚本里出现未知动作: %s" % fn)
                a = list(args) + [0.0, 0.0, 0.0]
                cfh.write("%d,%d,%d,%g,%g,%g\n" % (
                    seq_id, fr, CHAIN_SEQ_ACTION_CODES[fn], a[0], a[1], a[2]))
                cmd_rows += 1

            for fr in range(n_frames):
                for (fn, args) in by_frame.get(fr, []):
                    ns[fn](*args)

                rec.clear()
                ns["mainloop"]()
                writes = {(a, ch): (on, off) for (a, ch, on, off) in rec.led_writes()}
                if len(writes) != 12:
                    raise SystemExit("seq %d 帧 %d：期望 12 路，实际 %d 路"
                                     % (seq_id, fr, len(writes)))
                fields = [seq_id, fr]
                for key in LOGICAL:
                    on, off = writes[key]
                    fields += [on, off]
                fh.write(",".join(str(v) for v in fields) + "\n")
                rows += 1

            print("  seq %d: %d 帧（%d 条命令）" % (seq_id, n_frames, len(script)))

    print("  control_chain_seq golden 行数: %d 帧，命令 %d 条"
          "（参考值 = 原版 mainloop 连跑）" % (rows, cmd_rows))
    return rows


# ============================================================================
#  P4: 姿态动画 / 动作层 (action.c)
# ============================================================================
#
# 这一层和前面每一套都不同：它**跨模块改状态**。`action_stand()` 会
# `move()/gait()/height()/gesture()/set_leg_sit_offsets()`，
# `_pose_anim_begin()` 还会清 chain 的爬行状态，`mainloop` 开头那段会
# `move(3,1,1)`。所以参考值不能只记 12 路占空比 —— 还要记"调用完之后那些
# **模块级全局**变成了什么"。
#
# 取法（和 `control_chain` 同一套）：
#   把真版 `padog.py` exec 进来（`load_padog_ns()`），把模块级全局**按用例注入**，
#   调**原函数**，然后一起写进 CSV：
#     * 12 路**角度**（把真版 `PA_SERVO.angle` 包一层记下它收到的实参）
#     * 12 路**占空比**（记录型 I2C 记下原代码真正写进 PCA9685 的字节）
#     * 22 个模块级全局的**事后值**（副作用有没有如实发生，全在这里）
#
# 时钟：`mpy_stubs.install_controllable_clock()` 把 `utime.ticks_ms()` 钉死。
# 为什么必须钉、为什么钉了仍然忠实，见 `mpy_stubs.py` 里那段长注释。
#
# ⚠️ 三套 CSV 的分工：
#   `action_pose.csv` —— 纯函数（姿态表 / 插值 / 两种直写），比**角度**（0 容差）
#   `action_cmd.csv`  —— 三个动作入口 + mainloop 的姿态动画段，比**占空比 + 22 个全局量**
#   `action_wave.csv` —— `action_wave_direct()` 整条阻塞脚本，比**整条写寄存器日志**（含时刻）

#: 参考值/被测代码共用的模拟时钟起点。只要求非 0 且离"环绕"很远，
#: 好让"截止时刻已过期 / 未过期 / 正好相等"三个分支都落在有意义的位置。
ACTION_CLOCK_BASE = 100000

#: 12 个中位角在 CSV 里的列序 —— 与 `config_s.py` / `control_chain_cfg_t.init` 一致：
#: 腿1..腿4 × (髋, 大腿, 小腿)。⚠️ **不是**逻辑通道序（见 `servo_map.h` 那张表）。
ACTION_INIT_COLS = ("1p", "1h", "1s", "2p", "2h", "2s",
                    "3p", "3h", "3s", "4p", "4h", "4s")

#: 真机中位角（config_s.py 实测值）= 第一组采样
ACTION_INIT_REAL = [102, 84, 92, 96, 91, 85, 108, 78, 68, 92, 98, 102]


def _action_init_sets():
    """中位角采样组。

    * `real`：真机值。
    * `distinct`：12 个**互不相等**、量级各异的值 —— 否则"通道映射错位"测不出来
      （成长手册 P-18：全相等 / 对称的输入等于没测）。
    * `out_of_range`：两侧都越界（>180 与 <0），用来钉 `_clamp_deg` 加在哪几路上：
      `_apply_pose_blend` **每路都夹**、`_apply_stand_angles_direct` **只夹髋**。
      这两件事只有在**角度**域才看得见（`PA_SERVO.angle` 会把占空比夹回
      [102,511]，所以占空比里"越界"和"夹过"长得一模一样）。
    """
    return (
        ("real config_s.py", list(ACTION_INIT_REAL)),
        ("in-range, 12 distinct (P-18)",
         [11, 22, 33, 44, 55, 66, 77, 88, 99, 111, 122, 133]),
        ("out of range on both sides",
         [200, -10, 190, 96, 91, 185, 108, -40, 68, -25, 98, 102]),
    )


#: 插值系数采样。**全部是二进制精确值**（0/±0.5/±0.25/±0.75/1），且 `_sit_pose()`
#: 的 12 个偏移全是 4 的倍数 ⇒ `p0 + (p1-p0)*t` 在 float 与 double 里**逐位相同**。
#: 这就是"角度可以要求 0 容差"的**前提**，不是碰运气。
#: 端点外的两个值（-0.5 / 1.5）专门覆盖 `t` 的夹限分支。
ACTION_BLEND_T = (-0.5, 0.0, 0.25, 0.5, 0.75, 1.0, 1.5)


class _AngleRecorder:
    """把真版 `PA_SERVO.angle` 包一层：记下它**收到的角度**，然后原样调用。

    为什么要包：`PA_SERVO.angle()` 内部会把角度夹到占空比区间 `[102, 511]`
    （`Servos.position()`），于是**在占空比里看不出角度越界** —— 而 `_clamp_deg`
    究竟加在哪几路上正是这一层最容易抄错的地方。包一层才能拿到"原代码真正往下传的
    角度"，于是可以要求**角度 0 容差**。

    包装**不改变任何行为**：记完立刻调用原件；`_orig` 保存的也是原件。
    """

    def __init__(self, orig, clock):
        self._orig = orig
        self._clock = clock
        self.log = []          # [(clock_ms, logical_ch, deg), ...]

    def hook(self, ch, deg):
        self.log.append((self._clock.now_ms, int(ch), float(deg)))
        return self._orig(ch, deg)

    def clear(self):
        self.log.clear()


_ANGLE_REC = None


def _angle_recorder(clock):
    """装一次、全局复用（重复包装会记两遍）"""
    global _ANGLE_REC
    if _ANGLE_REC is None:
        servo = sys.modules["PA_SERVO"]
        _ANGLE_REC = _AngleRecorder(servo.angle, clock)
        servo.angle = _ANGLE_REC.hook
    return _ANGLE_REC


def _action_env(tmpdir, clock):
    """一次"真版 padog 环境"：返 (命名空间, 记录型I2C, 角度记录**列表**)

    第三个是 `_AngleRecorder.log` 本身（一个 list），`clear()` / `len()` 都是 list 的。
    """
    ns = load_padog_ns(tmpdir)
    rec = sys.modules["PA_SERVO"]._i2c_servo
    return ns, rec, _angle_recorder(clock).log


def _action_set_init(ns, init12):
    for i, name in enumerate(ACTION_INIT_COLS):
        ns["init_%s" % name] = init12[i]


def _capture(rec, log):
    """把一次调用记录的 (角度, 占空比) 取出来，并**校验一一对应**。

    角度来自 `PA_SERVO.angle` 的包装（原代码传下去的值），占空比来自记录型 I2C
    （原代码真正写进 PCA9685 的字节）。两者必须**同长度、同通道集合**；
    否则说明"一次 angle() 正好写一次寄存器"这个假设不成立 —— 直接报错，不猜
    （成长手册 P-22：参考值用到的输入必须由参考实现自己产生）。
    """
    writes = rec.led_writes()
    if len(writes) != len(log):
        raise SystemExit("action: %d 次 angle() 却记录到 %d 次寄存器写"
                         % (len(log), len(writes)))
    degs, duties = {}, {}
    for (_t, ch, deg) in log:
        if ch in degs:
            raise SystemExit("action: 同一次调用里逻辑通道 %d 被写了两次" % ch)
        degs[ch] = deg
    for (addr, ch, on, off) in writes:
        lch = ch if addr == 0x40 else ch + 6
        if on != 0 or not (0 <= off <= 511):
            raise SystemExit("action: 意外的 (ON,OFF)=(%d,%d)；本层的占空比只落在 "
                             "[102,511]（0 会走 4096 那个特殊分支）" % (on, off))
        duties[lch] = off
    if set(degs) != set(duties):
        raise SystemExit("action: 角度与占空比的通道集合不一致：%s vs %s"
                         % (sorted(degs), sorted(duties)))
    return degs, duties


def _gv(v):
    """CSV 取值：bool -> 0/1（**必须先判**，bool 是 int 的子类），int -> %d，其余 %.6f"""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, int):
        return "%d" % v
    return "%.6f" % v


def _require_float_exact(v, what):
    """断言一个浮点参考值**能被 float32 精确表示**。

    为什么要有这一条：`action.c` 用 `float`（ESP32 只有单精度硬件 FPU，见 README），
    参考值来自 Python 的 `double`。C 侧与参考值做**精确相等**比较时，只有当这个值
    在 float 里可精确表示才成立。与其事后偷偷放一个容差，不如在**生成参考值时**
    就把不合规的值挡下来 —— 这是"不放松容差"的机器化版本（成长手册 P-17/P-18）。
    """
    back = struct.unpack("<f", struct.pack("<f", float(v)))[0]
    if back != float(v):
        raise SystemExit("action: %s=%r 不能被 float32 精确表示，"
                         "这个用例无法做 0 容差比较（换采样点，别放宽容差）"
                         % (what, v))


# ---------------------------------------------------------------------------
#  [A] action_pose.csv —— 纯函数，比角度
# ---------------------------------------------------------------------------

def gen_action_pose(fh, tmpdir, clock):
    fh.write("# padog.py 姿态表/插值 golden vectors\n")
    fh.write("# 参考值 = 真版 padog.py 命名空间（load_padog_ns），时钟由 "
             "mpy_stubs.install_controllable_clock() 钉死\n")
    fh.write("# 列: fn,param,init_1p..init_4s(12),deg0..deg11(12),duty0..duty11(12)\n")
    fh.write("# fn=0: _apply_pose_blend(_stand_pose(), _sit_pose(), param)\n")
    fh.write("# fn=1: _apply_stand_angles_direct()      fn=2: _apply_sit_angles_direct()\n")
    fh.write("# deg  = 原代码传给 PA_SERVO.angle() 的角度（未过占空比夹限），C 侧按 "
             "**0 容差**比\n")
    fh.write("# duty = 它最终写进寄存器的占空比计数\n")
    fh.write("# param 只取二进制精确值、sit 偏移全是 4 的倍数 => 插值 float/double "
             "逐位相同\n")
    fh.write("# init 列序 = 腿1..腿4 x (髋,大腿,小腿)，与 config_s.py 一致\n")

    ns, rec, log = _action_env(tmpdir, clock)
    clock.set(ACTION_CLOCK_BASE)

    rows = 0
    for note, init12 in _action_init_sets():
        fh.write("# init set: %s\n" % note)
        _action_set_init(ns, init12)
        calls = [(0, t) for t in ACTION_BLEND_T] + [(1, 0.0), (2, 0.0)]
        for (fn, param) in calls:
            rec.clear()
            log.clear()
            if fn == 0:
                # 用**原函数自己的两张表**，不在生成器里另造输入（P-22）
                ns["_apply_pose_blend"](ns["_stand_pose"](), ns["_sit_pose"](), param)
            elif fn == 1:
                ns["_apply_stand_angles_direct"]()
            elif fn == 2:
                ns["_apply_sit_angles_direct"]()
            else:
                raise SystemExit("action_pose: 未知 fn=%d" % fn)

            degs, duties = _capture(rec, log)
            if len(degs) != 12:
                raise SystemExit("action_pose: 期望 12 路，实际 %d 路" % len(degs))

            fields = [fn, param] + list(init12)
            fields += [degs[i] for i in range(12)]
            fields += [duties[i] for i in range(12)]
            fh.write(",".join(_gv(v) for v in fields) + "\n")
            rows += 1

    print("  action_pose golden 行数: %d（3 组中位角 x 9 个采样）" % rows)
    return rows


# ---------------------------------------------------------------------------
#  [B] action_cmd.csv —— 动作入口 + mainloop 姿态动画段，比占空比 + 22 个全局量
# ---------------------------------------------------------------------------

#: 前置状态列（33 = 21 + 12 个中位角）。C 侧按同一顺序解析。
ACTION_CMD_PRE = (
    "action", "now_ms", "pair", "anim_start_ms", "anim_end_ms", "anim_hold_freeze",
    "cur_h_goal", "gait_mode", "t",
    "crawl_phase", "crawl_until_ms", "crawl_settle_until_ms", "inplace_step_end_ms",
    "direct_pose_freeze", "pose_anim_active",
    "front_y", "rear_y", "in_y", "in_pit", "in_rol", "init_case",
) + tuple("init_%s" % s for s in ACTION_INIT_COLS)

#: 事后状态列（35 = 1 + 12 占空比 + 22 个全局量）。
#: ⚠️ **这 22 个的顺序必须与 test_action.c 的 `dump_state()` 一一对应** ——
#: 它就是"副作用有没有如实发生"的全部证据（P-25）。
ACTION_CMD_POST = ("n_writes",) + tuple("duty%d" % i for i in range(12)) + (
    "direct_pose_freeze", "pose_anim_active", "pose_anim_hold_freeze",
    "pose_anim_start_ms", "pose_anim_end_ms",
    "crawl_phase", "crawl_until_ms", "crawl_settle_until_ms", "inplace_step_end_ms",
    "spd", "L", "R", "gait_mode", "t",
    "H_goal", "R_H", "PIT_goal", "ROL_goal", "X_goal",
    "front_y", "rear_y", "init_case",
)

#: 动作编号（写进 CSV，C 侧按同一张表解释）
ACTION_CMD_CODES = {
    0: "action_stand",
    1: "action_sit",           # 别名，与 2 等价（生成器会自己验一遍）
    2: "action_sit_direct",
    3: "mainloop",             # 只用到它的"inplace 服务 + 姿态动画"那两段
}


def _cmd_cases():
    """`action_cmd.csv` 的用例表。

    每行 = **一次调用**（`action_stand` / `action_sit` / `action_sit_direct` /
    `mainloop`），前置状态全部由列显式给出，事后状态全部逐项对照。
    """
    A0 = ACTION_CLOCK_BASE + 300
    base = dict(action=0, now=A0, pair=0, anim_start=0, anim_end=0, hold=0,
                h_goal=81.0, gait=0, t=0.0, crawl=0, until=0, settle=0,
                inplace=0, freeze=0, active=0, front=0.0, rear=0.0,
                in_y=18, in_pit=0, in_rol=0, init_case=0,
                init=list(ACTION_INIT_REAL))

    def mk(**kw):
        c = dict(base)
        c.update(kw)
        return c

    cases = []

    # ---- action_stand（0）----
    # 直写支：12 路 = 中位角（髋夹、腿不夹），H_goal 被 height() 覆写
    cases.append(mk(action=0))
    # 动画支（direct_pose_freeze=True）：坐 -> 站，**一个舵机都不写**
    cases.append(mk(action=0, freeze=1))
    # 提前 return：pose_anim_active 为真 => 连爬行都不清。
    # ⚠️ h_goal 用 70.5（而不是 70.6）：这一支**不改** H_goal，它会原样出现在事后
    #    状态里，而事后状态要求 float32 精确可表示（0 容差）。截断本身的用例在下面。
    cases.append(mk(action=0, active=1, freeze=1, crawl=1, until=12345,
                    inplace=ACTION_CLOCK_BASE + 999, h_goal=70.5))
    # height(int(H_goal)) 的**向零截断**（80.9 -> 80，-3.7 -> -3）
    cases.append(mk(action=0, h_goal=80.9))
    cases.append(mk(action=0, h_goal=-3.7))
    # 直写支 + 一大堆副作用同时非默认：
    #   gait_mode=1 => 第一次 gait(0) 会把 t 归零；in_pit/in_rol 非 0 时
    #   "gesture 在 gait 之后"这件事才看得出来（否则两者写成同一组值）
    cases.append(mk(action=0, h_goal=80.9, gait=1, t=0.5, crawl=1, until=12345,
                    settle=23456, inplace=ACTION_CLOCK_BASE + 999, init_case=1,
                    front=7.0, rear=-3.0, in_y=25, in_pit=3, in_rol=-2))
    # 动画支 + 同样的非默认：这一支里 gesture 之后**还有一次 gait(0)**
    #   => 三个目标最终应等于 int(in_pit)/int(in_rol)/int(in_y)（不是 0/0/25）
    cases.append(mk(action=0, freeze=1, h_goal=70.6, gait=1, t=0.5, crawl=1,
                    until=12345, settle=23456, inplace=ACTION_CLOCK_BASE + 999,
                    init_case=1, front=7.0, rear=-3.0, in_y=25,
                    in_pit=3, in_rol=-2))
    # 越界中位角（角度由 action_pose 那套钉，这里看占空比与状态）
    cases.append(mk(action=0, init=[200, -10, 190, 96, 91, 185,
                                    108, -40, 68, -25, 98, 102]))

    # ---- action_sit / action_sit_direct（1 / 2）----
    cases.append(mk(action=1))
    cases.append(mk(action=2))
    # 两个提前 return。⚠️ 它们在**外部状态上无法区分**（都在清爬行之前 return）——
    # 这不是输入没区分度，而是这两条分支本来就没有可观察差异；留着是为了
    # 万一将来原实现把顺序改了，对照关系还在。
    cases.append(mk(action=1, freeze=1, crawl=1, inplace=ACTION_CLOCK_BASE + 999))
    cases.append(mk(action=1, active=1, crawl=1, inplace=ACTION_CLOCK_BASE + 999))
    cases.append(mk(action=1, active=1, freeze=1))
    # 正常路径 + 非默认副作用（注意：这一支的最后一次 gait(0) 同样覆盖 gesture）
    cases.append(mk(action=2, crawl=1, until=12345, settle=23456,
                    inplace=ACTION_CLOCK_BASE + 999, init_case=1,
                    front=5.0, rear=-4.0, in_y=-7, in_pit=3, in_rol=-2,
                    gait=1, t=0.5))
    # 另一组中位角（证明 C 版真的用了配置里的中位角）
    cases.append(mk(action=2, init=[11, 22, 33, 44, 55, 66, 77, 88, 99, 111, 122, 133],
                    in_y=25))

    # ---- mainloop 的 inplace 服务 + 姿态动画段（3）----
    # ⚠️ 这些行**必须** pose_anim_active=True：否则 mainloop 会继续往下跑整条运动链，
    #    那不是本模块的范围。crawl 的三个量固定为 0，于是 mainloop 第 877~879 行
    #    都是 no-op（那几行属于 chain 层，control_chain.c 已经实现）。
    common = dict(action=3, active=1, anim_start=A0, anim_end=A0 + 900, pair=0)
    cases.append(mk(**common))                                   # t = 0
    cases.append(mk(**dict(common, now=A0 + 225)))               # t = 0.25
    cases.append(mk(**dict(common, now=A0 + 450)))
    cases.append(mk(**dict(common, now=A0 + 899)))               # 差 1 ms 没到
    cases.append(mk(**dict(common, now=A0 + 900)))               # t = 1 -> 完成
    cases.append(mk(**dict(common, now=A0 + 1500)))              # t > 1 -> 完成
    cases.append(mk(**dict(common, now=A0 + 900, hold=1)))       # 完成后冻结
    cases.append(mk(**dict(common, now=A0 + 400, pair=1)))       # 坐 -> 站的反向
    cases.append(mk(**dict(common, now=A0, anim_start=A0, anim_end=A0)))       # total<=0
    cases.append(mk(**dict(common, now=A0 + 1, anim_start=A0, anim_end=A0)))   # total<=0 且已到
    cases.append(mk(**dict(common, now=A0 - 100)))               # 负 elapsed -> t<0
    cases.append(mk(**dict(common, init=[11, 22, 33, 44, 55, 66,
                                         77, 88, 99, 111, 122, 133],
                             now=A0 + 300)))
    # inplace 服务三个分支
    cases.append(mk(**dict(common, inplace=A0 + 500, init_case=1)))   # 未过期 -> move(3,1,1)
    cases.append(mk(**dict(common, inplace=A0 - 500)))                # 已过期 -> 只清
    cases.append(mk(**dict(common, inplace=A0)))                      # 正好相等 -> 走过期支
    # 未过期 + 动画同帧完成：move() 先置 direct_pose_freeze=False，
    # 之后 _pose_anim_step() 又置成 hold_freeze -> 顺序错了就会 FAIL
    cases.append(mk(**dict(common, inplace=A0 + 500, init_case=1,
                           now=A0 + 900, hold=1)))
    cases.append(mk(**dict(common, inplace=A0 + 500, front=4.0, rear=-6.0,
                           now=A0 + 100)))
    return cases


def _action_cmd_reference(c, tmpdir, clock):
    """跑一次真版 `padog.py`，返回 (写了几路, {通道: 占空比}, 22 个事后全局量)"""
    ns, rec, log = _action_env(tmpdir, clock)
    _action_set_init(ns, c["init"])

    # 注入前置状态。**只注入本层真的会读的那些量**；其余保持模块初值
    # （`load_padog_ns()` 每次重新 exec padog.py，所以模块级状态天然干净）。
    ns["in_y"] = c["in_y"]
    ns["in_pit"] = c["in_pit"]
    ns["in_rol"] = c["in_rol"]
    ns["H_goal"] = c["h_goal"]          # R_H 不动：padog.py 第 156~157 行已经 = int(H_goal)
    ns["gait_mode"] = c["gait"]
    ns["t"] = c["t"]
    ns["crawl_phase"] = c["crawl"]
    ns["crawl_until_ms"] = c["until"]
    ns["crawl_settle_until_ms"] = c["settle"]
    ns["inplace_step_end_ms"] = c["inplace"]
    ns["direct_pose_freeze"] = bool(c["freeze"])
    ns["pose_anim_active"] = bool(c["active"])
    ns["pose_anim_hold_freeze"] = bool(c["hold"])
    ns["pose_anim_start_ms"] = c["anim_start"]
    ns["pose_anim_end_ms"] = c["anim_end"]
    ns["front_leg_y_offset"] = c["front"]
    ns["rear_leg_y_offset"] = c["rear"]
    ns["init_case"] = c["init_case"]
    # 三个重心目标的"上电初值"（padog.py:146 是 `int(in_pit)/int(in_rol)/int(in_y)`）。
    # ⚠️ 用**原函数** `gesture()` 施加，不在这里手抄那个表达式（P-22 / P-24 的教训：
    #    要断言/设置什么，就让权威实现去做）。
    ns["gesture"](int(c["in_pit"]), int(c["in_rol"]), int(c["in_y"]))
    # 动画的两个端点：用**原函数自己的两张表**，与 `action_sit_direct()` 内部一致
    ns["pose_anim_from"] = ns["_stand_pose"]() if c["pair"] == 0 else ns["_sit_pose"]()
    ns["pose_anim_to"] = ns["_sit_pose"]() if c["pair"] == 0 else ns["_stand_pose"]()

    rec.clear()
    log.clear()
    clock.set(c["now"])

    fn = c["action"]
    if fn == 0:
        ns["action_stand"]()
    elif fn == 1:
        ns["action_sit"]()
    elif fn == 2:
        ns["action_sit_direct"]()
    elif fn == 3:
        ns["mainloop"]()
    else:
        raise SystemExit("action_cmd: 未知 action=%d" % fn)

    _degs, duties = _capture(rec, log)

    post = [
        ns["direct_pose_freeze"], ns["pose_anim_active"], ns["pose_anim_hold_freeze"],
        ns["pose_anim_start_ms"], ns["pose_anim_end_ms"],
        ns["crawl_phase"], ns["crawl_until_ms"], ns["crawl_settle_until_ms"],
        ns["inplace_step_end_ms"],
        ns["spd"], ns["L"], ns["R"], ns["gait_mode"], ns["t"],
        ns["H_goal"], ns["R_H"], ns["PIT_goal"], ns["ROL_goal"], ns["X_goal"],
        ns["front_leg_y_offset"], ns["rear_leg_y_offset"], ns["init_case"],
    ]
    if len(post) != len(ACTION_CMD_POST) - 13:
        raise SystemExit("action_cmd: 事后状态列数对不上")
    return len(log), duties, post


def gen_action_cmd(fh, tmpdir, clock):
    fh.write("# padog.py 动作层入口 golden vectors（参考值 = 真版 padog 命名空间）\n")
    fh.write("# action: 0=action_stand 1=action_sit 2=action_sit_direct 3=mainloop\n")
    fh.write("#   （3 只用到 mainloop 的 inplace 服务 + _pose_anim_step 那两段；\n")
    fh.write("#     所以 action=3 的行必须 pose_anim_active=1、crawl 三个量=0）\n")
    fh.write("# 前置列: %s\n" % ",".join(ACTION_CMD_PRE))
    fh.write("# 事后列: %s\n" % ",".join(ACTION_CMD_POST))
    fh.write("# 事后那几个全局量就是「副作用有没有如实发生」的全部证据（成长手册 P-25）\n")
    fh.write("# 所有事后浮点值都保证能被 float32 精确表示 => C 侧可以要求 0 容差\n")

    rows = 0
    for c in _cmd_cases():
        pre = [c["action"], c["now"], c["pair"], c["anim_start"], c["anim_end"],
               c["hold"], c["h_goal"], c["gait"], c["t"],
               c["crawl"], c["until"], c["settle"], c["inplace"],
               c["freeze"], c["active"], c["front"], c["rear"],
               c["in_y"], c["in_pit"], c["in_rol"], c["init_case"]] + list(c["init"])
        if len(pre) != len(ACTION_CMD_PRE):
            raise SystemExit("action_cmd: 前置列数对不上（%d vs %d）"
                             % (len(pre), len(ACTION_CMD_PRE)))

        n, duties, post = _action_cmd_reference(c, tmpdir, clock)

        # "不放松容差"的机器化保证：事后每个浮点值都必须 float32 精确可表示
        for (name, v) in zip(ACTION_CMD_POST[13:], post):
            if isinstance(v, float):
                _require_float_exact(v, "action=%d %s" % (c["action"], name))

        fields = pre + [n] + [duties.get(i, 0) for i in range(12)] + post
        fh.write(",".join(_gv(v) for v in fields) + "\n")
        rows += 1

    _verify_aliases(tmpdir, clock)
    print("  action_cmd golden 行数: %d（含 mainloop 姿态动画与 inplace 服务）" % rows)
    return rows


def _verify_aliases(tmpdir, clock):
    """参考侧验证那一对别名函数。

    `action_sit()` / `action_wave()` 在原实现里就是一行调用。这里**在真版参考实现
    自己身上跑两遍、比结果**，而不是靠读代码下结论（成长手册 P-24：
    "要断言某个值，就把它算出来"）。不一致就直接让生成失败。

    ⚠️ 顺带把"别名等价"钉在了**参考实现**上：C 版只有一个函数，
    所以 CSV 里只留一份参考值。
    """
    pairs = (("action_sit", "action_sit_direct"),
             ("action_wave", "action_wave_direct"))
    for (a, b) in pairs:
        got = []
        for fn in (a, b):
            ns, rec, log = _action_env(tmpdir, clock)
            rec.clear()
            log.clear()
            clock.set(ACTION_CLOCK_BASE)
            ns[fn]()
            got.append((
                [(addr, ch, on, off) for (addr, ch, on, off) in rec.led_writes()],
                ns["direct_pose_freeze"], ns["pose_anim_active"],
                ns["pose_anim_hold_freeze"], ns["inplace_step_end_ms"],
                ns["pose_anim_start_ms"], ns["pose_anim_end_ms"],
                clock.now_ms - ACTION_CLOCK_BASE,
            ))
        if got[0] != got[1]:
            raise SystemExit("golden 参考侧：%s() 与 %s() 的结果不一致" % (a, b))
    print("  参考侧别名验证: action_sit == action_sit_direct, "
          "action_wave == action_wave_direct")


# ---------------------------------------------------------------------------
#  [C] action_wave.csv + action_wave_final.csv —— 整条阻塞脚本，比"写寄存器日志"
# ---------------------------------------------------------------------------

def gen_action_wave(fh, wfh, tmpdir, clock):
    """
    参考值 = 把真版 `action_wave_direct()` **一次跑完**，记下每一次 `angle()`：
    (时刻, 通道, 角度) 与 (addr, pca_ch, on, off)。时钟是假的，所以整条脚本
    瞬间跑完而且完全可复现。

    ⚠️ `action_wave_direct()` 里 `_wait_pose_anim_done()` 会调 `mainloop()`；
    在这个状态下 mainloop 只会走到 `_pose_anim_step()` 就 return
    （爬行与 inplace 都被 `action_sit_direct()` 清了），所以它写出的就是姿态动画的
    12 路插值 —— 这正是 C 版 `action_wave_step()` 的 WAIT 阶段要做的事。

    **分组**：原实现"写一组舵机 -> sleep 一段"，所以**同一毫秒内的那些写正好就是一步**
    （12 路插值 / 4 路 / 3 路 / 1 路）。按时间戳分组，`t_off` 列同时把每一步的
    **时序**钉住 —— 光比"写了什么"是钉不住 `sleep` 时长的。
    """
    ns, rec, log = _action_env(tmpdir, clock)
    clock.set(ACTION_CLOCK_BASE)
    rec.clear()
    log.clear()
    ns["action_wave_direct"]()

    writes = rec.led_writes()
    if len(writes) != len(log):
        raise SystemExit("action_wave: %d 次 angle() 却记录到 %d 次寄存器写"
                         % (len(log), len(writes)))

    groups = []
    for i, (t, _ch, _deg) in enumerate(log):
        if not groups or groups[-1][0] != t:
            groups.append((t, []))
        groups[-1][1].append(i)

    fh.write("# padog.action_wave_direct() golden vectors（参考值 = 原版真跑一次）\n")
    fh.write("# 列: group,t_off,ch,duty\n")
    fh.write("# t_off = 相对本次动作开始时刻的毫秒偏移 —— 它把每一步之后的\n")
    fh.write("#         time.sleep_ms() 也钉住了（只比写了什么是钉不住时序的）\n")
    fh.write("# group = 同一毫秒内的那些写；组内按逻辑通道升序\n")
    fh.write("# ch = 逻辑通道 0..11；duty = 写进 PCA9685 的占空比计数\n")

    rows = 0
    for gi, (t, idxs) in enumerate(groups):
        for i in sorted(idxs, key=lambda k: log[k][1]):
            _t, ch, _deg = log[i]
            addr, pch, on, off = writes[i]
            lch = pch if addr == 0x40 else pch + 6
            if lch != ch or on != 0 or not (0 <= off <= 511):
                raise SystemExit("action_wave: 两个记录器对不上："
                                 "angle(ch=%d) vs i2c(addr=%#x,pca=%d,on=%d,off=%d)"
                                 % (ch, addr, pch, on, off))
            fh.write("%d,%d,%d,%d\n" % (gi, t - ACTION_CLOCK_BASE, ch, off))
            rows += 1

    # ---- 结束时的状态（它是"动作末尾留下了什么"的完整证据）----
    final = [
        clock.now_ms - ACTION_CLOCK_BASE, len(groups), len(writes),
        ns["pose_anim_active"], ns["pose_anim_hold_freeze"], ns["direct_pose_freeze"],
        ns["pose_anim_start_ms"] - ACTION_CLOCK_BASE,
        ns["pose_anim_end_ms"] - ACTION_CLOCK_BASE,
        ns["inplace_step_end_ms"], ns["crawl_phase"], ns["gait_mode"], ns["t"],
        ns["spd"], ns["L"], ns["R"], ns["H_goal"], ns["R_H"],
        ns["PIT_goal"], ns["ROL_goal"], ns["X_goal"],
        ns["front_leg_y_offset"], ns["rear_leg_y_offset"], ns["init_case"],
    ]
    wfh.write("# padog.action_wave_direct() 结束状态 golden\n")
    wfh.write("# 列: t_off,n_groups,n_writes,pose_anim_active,pose_anim_hold_freeze,"
              "direct_pose_freeze,anim_start_off,anim_end_off,inplace_step_end_ms,"
              "crawl_phase,gait_mode,t,spd,L,R,H_goal,R_H,PIT_goal,ROL_goal,X_goal,"
              "front_y,rear_y,init_case\n")
    wfh.write(",".join(_gv(v) for v in final) + "\n")

    print("  action_wave golden 行数: %d（%d 组，%d 个全局量）"
          % (rows, len(groups), len(final)))
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

    print("\n[1/7] PA_IK -> kinematics.c ...")
    ns_ik = load_module("PA_IK.py")
    with open(OUT / "ik.csv", "w", encoding="utf-8", newline="\n") as fh:
        total += gen_ik(ns_ik, fh)

    print("\n[2/7] PA_ATTITUDE -> body_pose.c ...")
    ns_att = load_module("PA_ATTITUDE.py")
    with open(OUT / "body_pose.csv", "w", encoding="utf-8", newline="\n") as fh:
        total += gen_body_pose(ns_att, fh)

    print("\n[3/7] PA_TROT -> gait_trot.c ...")
    ns_trot = load_module("PA_TROT.py")
    with open(OUT / "gait_trot.csv", "w", encoding="utf-8", newline="\n") as fh:
        total += gen_gait_trot(ns_trot, fh)

    print("\n[4/7] PA_AVGFILT -> filter_moving_avg.c ...")
    ns_flt = load_module("PA_AVGFILT.py")
    with open(OUT / "moving_avg.csv", "w", encoding="utf-8", newline="\n") as fh:
        total += gen_moving_avg(ns_flt, fh)

    print("\n[5/7] PA_WALK -> gait_walk.c ...")
    ns_walk = load_module("PA_WALK.py")
    with open(OUT / "gait_walk.csv", "w", encoding="utf-8", newline="\n") as fh:
        total += gen_gait_walk(ns_walk, fh)

    # 最后两个 suite 会把 machine.I2C / machine.Pin 换成"记录型"实现，
    # 所以放在最后，避免影响前面依赖 stub 会抛异常的模块。
    print("\n[6/7] PA_SERVO -> servo_map.c（角度/占空比路径）...")
    ang, duties = servo_angle_samples()
    with open(OUT / "servo_angle.csv", "w", encoding="utf-8", newline="\n") as fh:
        total += gen_servo_angle(fh, ang, duties)

    print("\n[7/8] padog.servo_output -> servo_map.c（关节角 -> 12 路）...")
    import tempfile
    with tempfile.TemporaryDirectory() as tmpdir:
        with open(OUT / "servo_output.csv", "w", encoding="utf-8", newline="\n") as fh:
            total += gen_servo_output(fh, servo_output_cases(), tmpdir)

        print("\n[8/9] padog.mainloop() -> control_chain.c（全链路，单帧，P3）...")
        with open(OUT / "control_chain.csv", "w", encoding="utf-8", newline="\n") as fh:
            total += gen_control_chain(fh, control_chain_cases(), tmpdir)

        print("\n[9/9] padog.mainloop() 连跑 -> control_chain_cmd.c（多帧序列，P3）...")
        with open(OUT / "control_chain_seq.csv", "w", encoding="utf-8", newline="\n") as fh:
            total += gen_control_chain_seq(fh, tmpdir)

        # ---- P4：姿态动画 / 动作层 ----------------------------------------
        #
        # ⚠️ 这一段**放在最后**，而且 clock 只在这里装：它会把
        # `sys.modules['utime']` 换成可控时钟、给 CPython 的 `time` 补 `sleep_ms`。
        # 前面几套（尤其 control_chain 的爬行分支要读 ticks_ms）必须继续看到
        # **安装之前**的那个 utime，否则参考值会变 —— 那是"改了别人的参考值"，
        # 不是我要做的事。
        print("\n[10/12] padog 姿态表/插值 -> action.c（纯函数，P4）...")
        clock = mpy_stubs.install_controllable_clock(ACTION_CLOCK_BASE)
        with open(OUT / "action_pose.csv", "w", encoding="utf-8", newline="\n") as fh:
            total += gen_action_pose(fh, tmpdir, clock)

        print("\n[11/12] padog 动作入口 + mainloop 姿态动画 -> action.c（P4）...")
        with open(OUT / "action_cmd.csv", "w", encoding="utf-8", newline="\n") as fh:
            total += gen_action_cmd(fh, tmpdir, clock)

        print("\n[12/12] padog.action_wave_direct() -> action.c（阻塞脚本，P4）...")
        with open(OUT / "action_wave.csv", "w", encoding="utf-8", newline="\n") as fh:
            with open(OUT / "action_wave_final.csv", "w",
                      encoding="utf-8", newline="\n") as wfh:
                total += gen_action_wave(fh, wfh, tmpdir, clock)

    print("\n完成。共 12 个 suite, %d 行。" % total)
    print("提示：这些 CSV 要提交进仓库，C 版测试只读它们。")


if __name__ == "__main__":
    main()
