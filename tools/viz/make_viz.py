#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_viz.py —— 把控制链的逐帧轨迹做成**自包含 HTML**(双击用浏览器打开)

数据来源：`chain_trace.csv`（由 `dump_chain_trace.c` 导出，见那个文件顶部说明）。
它本身的可信度来自 `control_chain` 已被 golden 逐帧钉住（单帧 1080 + 多帧 9840，
零容差）—— 所以看到的轨迹就是**原版的轨迹**。

⚠️ **画面不是证据，数字才是。** 图里：
  - 足端位置（髋到足那条线的终点）= 精确数据；
  - 膝的位置 = 按 l1/l2 两连杆几何**解算出来的示意**（原版只给足端目标）；
  - 髋角（横摆）没有画进来，所以是**侧视示意**，不是三维姿态。
它能让你一眼看出"四条腿是不是按该有的顺序抬""摆动方向对不对"这类**粗大问题**，
但看不出零点几度的差别。

用法：
    python make_viz.py [chain_trace.csv] [输出.html]
"""

import csv
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

#: 序列名字（与 gen_golden.py 的 CHAIN_SEQ_SCRIPTS 对应）
SEQ_NAMES = {
    1: "序列1：站立 → TROT 前进 → 加转向 → 回直行 → 停",
    2: "序列2：WALK 前进 → 后退 → 停",
    3: "序列3：TROT 行进中反复改速度（验证相位不重置）",
    4: "序列4：高度 60 → 姿态(俯仰5/滚转-4/重心30) → 回高度 81 → 回姿态",
    5: "序列5：TROT ↔ WALK 切换",
}

#: 腿的物理位置（与 servo_map 的逻辑通道表一致：腿1 左前 / 腿2 右前 / 腿3 右后 / 腿4 左后）
LEG_NAMES = {0: "腿1 左前", 1: "腿2 右前", 2: "腿3 右后", 3: "腿4 左后"}
LEG_COLORS = {0: "#2b6cb0", 1: "#c05621", 2: "#2f855a", 3: "#805ad5"}

#: 腿长（config_s.py：l1 大腿 130、l2 小腿 138）
L1, L2 = 130.0, 138.0
#: 前后腿间距（config_s.py：l = 230）
BODY_L = 230.0
#: 抬腿判定阈值（mm）：足端比站立高度高这么多就算"摆动相"
SWING_LIFT_MM = 12.0


def load(path):
    seqs = {}
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.reader(fh):
            if not row or row[0].startswith("#"):
                continue
            seq = int(row[0])
            frame = int(row[1])
            ang = [float(v) for v in row[2:14]]
            ikx = [float(v) for v in row[14:18]]
            iky = [float(v) for v in row[18:22]]
            seqs.setdefault(seq, []).append(
                {"f": frame, "ang": ang, "x": ikx, "y": iky})
    return seqs


HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>机器狗步态可视化（数据来自原版 mainloop 的逐帧对照）</title>
<style>
  body { font-family: "Segoe UI", "Microsoft YaHei", sans-serif; margin: 16px;
         background: #f7fafc; color: #1a202c; }
  h1 { font-size: 18px; margin: 0 0 4px 0; }
  .sub { color: #4a5568; font-size: 13px; margin-bottom: 12px; }
  .bar { background:#fff; border:1px solid #e2e8f0; border-radius:6px;
         padding:10px 12px; margin-bottom:12px; }
  select, button { font-size: 14px; padding: 4px 10px; }
  input[type=range] { width: 380px; vertical-align: middle; }
  .row { display:flex; gap:16px; flex-wrap:wrap; align-items:center; }
  canvas { background:#fff; border:1px solid #e2e8f0; border-radius:6px; }
  .cap { font-size:12px; color:#4a5568; margin-top:4px; }
  code { background:#edf2f7; padding:1px 4px; border-radius:3px; }
  .warn { background:#fffaf0; border:1px solid #f6e05e; border-radius:6px;
          padding:8px 12px; font-size:12.5px; color:#744210; margin-bottom:12px; }
</style>
</head>
<body>

<h1>机器狗步态可视化</h1>
<div class="sub" id="subtitle"></div>

<div class="warn">
  <b>画面不是证据，数字才是。</b>
  足端位置来自被 golden 逐帧钉住的控制链输出（精确）；
  <b>膝的位置是按大小腿长度 l1/l2 两连杆几何解算的示意</b>（原版只给足端目标）；
  髋的横摆角没有画进来，所以这是<b>侧视示意</b>，不是三维姿态。
</div>

<div class="bar">
  <div class="row">
    <label>序列：<select id="seq"></select></label>
    <button id="play">▶ 播放</button>
    <input type="range" id="slider" min="0" max="0" value="0">
    <span id="frameLabel"></span>
    <label>速度：<select id="speed">
      <option value="200">慢</option>
      <option value="80" selected>中</option>
      <option value="30">快</option>
    </select></label>
  </div>
</div>

<div class="row">
  <div>
    <canvas id="gait" width="980" height="150"></canvas>
    <div class="cap">步态图：每条横道 = 一条腿，<b>有颜色的段 = 抬起（摆动相）</b>，灰段 = 落地（支撑相）。
      竖线 = 当前帧。TROT 应看到对角两腿同步；WALK 应看到 1→2→3→4 依次。</div>
  </div>
</div>

<div class="row">
  <div>
    <canvas id="left" width="480" height="330"></canvas>
    <div class="cap">左侧视：腿1 左前 / 腿4 左后</div>
  </div>
  <div>
    <canvas id="right" width="480" height="330"></canvas>
    <div class="cap">右侧视：腿2 右前 / 腿3 右后</div>
  </div>
</div>

<div class="row">
  <div>
    <canvas id="traj" width="980" height="300"></canvas>
    <div class="cap">足端竖直位置随时间（上 = 抬得高）。四条腿的相位差在这里最直观。</div>
  </div>
</div>

<script>
const DATA = __DATA__;
const SEQ_NAMES = __SEQ_NAMES__;
const LEG_NAMES = __LEG_NAMES__;
const LEG_COLORS = __LEG_COLORS__;
const L1 = __L1__, L2 = __L2__, BODY_L = __BODY_L__;
const SWING_LIFT_MM = __SWING_LIFT_MM__;

const seqKeys = Object.keys(DATA).map(Number).sort((a,b)=>a-b);
let curSeq = seqKeys[0], frame = 0, playing = false, timer = null;

const $ = id => document.getElementById(id);
const seqSel = $("seq");
seqKeys.forEach(s => {
  const o = document.createElement("option");
  o.value = s; o.textContent = SEQ_NAMES[s] || ("序列 " + s);
  seqSel.appendChild(o);
});
seqSel.value = curSeq;

function frames() { return DATA[curSeq]; }

function setupSeq() {
  const n = frames().length;
  $("slider").max = n - 1;
  $("slider").value = 0;
  frame = 0;
  $("subtitle").textContent =
      SEQ_NAMES[curSeq] + "   共 " + n + " 帧。"
    + " 链节拍 65 ms/帧 ⇒ 实际时长约 " + (n * 0.065).toFixed(1) + " 秒。";
  drawAll();
}

/* ---------- 工具 ---------- */

/** 站立高度：取第一帧四条腿 y 的最大绝对值，作为"落地"参考线 */
/* 地面线：所有帧里最低的足端位置（y 向下为负 ⇒ 取 min） */
function groundY() {
  let m = Infinity;
  for (const fr of frames()) for (const y of fr.y) m = Math.min(m, y);
  return m;
}
/* 抬腿判定基准：**滑动窗口**（±half 帧）内的局部最低点。
   与 Python 端 swing_windows() 用同一判据。
   为什么不用固定基准：机身滚转会被 WALK 的 gesture() 改，右腿整体有持续偏移，
   用"全局最低点"会把右腿误判成整段都在抬。滑动窗口能分开"缓慢漂移"与"几帧的抬腿"。 */
function localBaseline(leg, half) {
  const y = frames().map(fr => fr.y[leg]);
  const n = y.length, out = new Array(n);
  for (let i = 0; i < n; ++i) {
    let lo = Infinity;
    for (let j = Math.max(0, i - half); j < Math.min(n, i + half + 1); ++j) lo = Math.min(lo, y[j]);
    out[i] = lo;
  }
  return out;
}

/** 两连杆解算膝的位置；返回 [kx, ky]。
 *  两个解里取"膝更高"的那个 —— 看起来像一条弯着的腿。
 *  （这是**示意**，见页面顶部说明。） */
function knee(hx, hy, fx, fy) {
  let dx = fx - hx, dy = fy - hy;
  let d = Math.hypot(dx, dy);
  const dmax = L1 + L2 - 0.001;
  if (d > dmax) { const k = dmax / d; dx *= k; dy *= k; d = dmax; }
  if (d < 1e-6) return [hx, hy];
  const a = (L1*L1 - L2*L2 + d*d) / (2*d);
  const h = Math.sqrt(Math.max(0, L1*L1 - a*a));
  const ux = dx/d, uy = dy/d;
  const nx = -uy, ny = ux;
  const p1 = [hx + a*ux + h*nx, hy + a*uy + h*ny];
  const p2 = [hx + a*ux - h*nx, hy + a*uy - h*ny];
  return (p1[1] >= p2[1]) ? p1 : p2;   /* 取更高的膝 */
}

/* ---------- 步态图 ---------- */

function drawGait() {
  const c = $("gait"), g = c.getContext("2d");
  const W = c.width, H = c.height;
  g.clearRect(0, 0, W, H);
  const fs = frames(), n = fs.length;
  const padL = 78, padR = 12, padT = 10, padB = 18;
  const w = W - padL - padR;
  const trackH = (H - padT - padB) / 4;
  const gnd = groundY();
  const base = localBaseline(leg, 10);

  /* 参考线：每 1 秒一条（65 ms/帧 ⇒ 15.38 帧）*/
  g.strokeStyle = "#edf2f7"; g.lineWidth = 1;
  const fps = 1 / 0.065;
  for (let t = 1; t * fps < n; ++t) {
    const x = padL + (t * fps / n) * w;
    g.beginPath(); g.moveTo(x, padT); g.lineTo(x, H - padB); g.stroke();
  }
  g.fillStyle = "#a0aec0"; g.font = "10px sans-serif";
  for (let t = 1; t * fps < n; ++t) {
    const x = padL + (t * fps / n) * w;
    g.fillText(t + "s", x + 2, H - 6);
  }

  for (let leg = 0; leg < 4; ++leg) {
    const y0 = padT + leg * trackH + 3, hh = trackH - 6;
    g.fillStyle = "#f7fafc"; g.fillRect(padL, y0, w, hh);
    g.strokeStyle = "#e2e8f0"; g.strokeRect(padL + 0.5, y0 + 0.5, w - 1, hh - 1);

    /* 落地 = 浅灰底，抬起 = 彩色 */
    let runStart = -1;
    for (let i = 0; i <= n; ++i) {
      const lift = (i < n) ? (fs[i].y[leg] - base[i]) : 0;  /* 相对局部最低点的抬升 */
      const swing = lift > 4.0;                              /* >4 mm 即算摆动相 */
      if (swing && runStart < 0) runStart = i;
      if (!swing && runStart >= 0) {
        const x1 = padL + (runStart / n) * w, x2 = padL + (i / n) * w;
        g.fillStyle = LEG_COLORS[leg];
        g.fillRect(x1, y0 + 1, Math.max(1, x2 - x1), hh - 2);
        runStart = -1;
      }
    }
    g.fillStyle = "#2d3748"; g.font = "11px sans-serif";
    g.fillText(LEG_NAMES[leg], 6, y0 + hh / 2 + 4);
  }

  /* 当前帧游标 */
  const x = padL + (frame / Math.max(1, n - 1)) * w;
  g.strokeStyle = "#e53e3e"; g.lineWidth = 2;
  g.beginPath(); g.moveTo(x, padT); g.lineTo(x, H - padB); g.stroke();
}

/* ---------- 侧视 ---------- */

function drawSide(canvasId, legs, title) {
  const c = $(canvasId), g = c.getContext("2d");
  const W = c.width, H = c.height;
  g.clearRect(0, 0, W, H);
  const fr = frames()[frame];
  const gnd = groundY();

  /* 世界坐标（mm）-> 画布。髋在 y=0，脚在 y≈-200 */
  const margin = 40;
  const sx = (W - 2 * margin) / (BODY_L + 2 * 220);
  const sy = (H - 2 * margin) / 320;
  const s = Math.min(sx, sy);
  const cx = W / 2, cy = margin + 20;     /* 髋所在高度 */

  const tx = wx => cx + wx * s;
  const ty = wy => cy - wy * s;

  /* 地面线 */
  g.strokeStyle = "#cbd5e0"; g.setLineDash([4, 4]); g.lineWidth = 1;
  g.beginPath(); g.moveTo(0, ty(gnd)); g.lineTo(W, ty(gnd)); g.stroke();
  g.setLineDash([]);

  /* 机身 */
  g.fillStyle = "#e2e8f0"; g.strokeStyle = "#a0aec0";
  const bx1 = tx(-BODY_L / 2), bx2 = tx(BODY_L / 2);
  g.fillRect(bx1, ty(0) - 14, bx2 - bx1, 28);
  g.strokeRect(bx1 + 0.5, ty(0) - 13.5, bx2 - bx1 - 1, 27);
  g.fillStyle = "#4a5568"; g.font = "11px sans-serif";
  g.fillText("机身", bx1 + 6, ty(0) + 4);

  legs.forEach(leg => {
    /* 腿的髋位置：前腿(1,2) 在机身前端，后腿(3,4) 在后端 */
    const front = (leg === 0 || leg === 1);
    const hx = front ? (BODY_L / 2) : (-BODY_L / 2);
    const fx = hx + fr.x[leg];
    const fy = 0 + fr.y[leg];
    const [kx, ky] = knee(hx, 0, fx, fy);

    g.strokeStyle = LEG_COLORS[leg];
    g.lineWidth = 4; g.lineCap = "round";
    g.beginPath(); g.moveTo(tx(hx), ty(0)); g.lineTo(tx(kx), ty(ky)); g.stroke();
    g.lineWidth = 3;
    g.beginPath(); g.moveTo(tx(kx), ty(ky)); g.lineTo(tx(fx), ty(fy)); g.stroke();

    /* 髋 / 膝 / 足 */
    g.fillStyle = LEG_COLORS[leg];
    g.beginPath(); g.arc(tx(hx), ty(0), 4, 0, 6.284); g.fill();
    g.beginPath(); g.arc(tx(kx), ty(ky), 3, 0, 6.284); g.fill();
    g.beginPath(); g.arc(tx(fx), ty(fy), 5, 0, 6.284); g.fill();

    g.fillStyle = "#2d3748"; g.font = "11px sans-serif";
    g.fillText(LEG_NAMES[leg], tx(fx) - 24, ty(fy) + 18);
  });

  g.fillStyle = "#4a5568"; g.font = "12px sans-serif";
  g.fillText(title, 10, 16);
  g.fillText("第 " + fr.f + " 帧  t=" + (frame * 0.065).toFixed(2) + " s",
             W - 150, 16);
}

/* ---------- 足端高度曲线 ---------- */

function drawTraj() {
  const c = $("traj"), g = c.getContext("2d");
  const W = c.width, H = c.height;
  g.clearRect(0, 0, W, H);
  const fs = frames(), n = fs.length;
  const padL = 50, padR = 14, padT = 14, padB = 26;
  const w = W - padL - padR, h = H - padT - padB;

  let lo = 1e9, hi = -1e9;
  for (const fr of fs) for (const y of fr.y) { lo = Math.min(lo, y); hi = Math.max(hi, y); }
  const gnd = groundY();
  const ymin = Math.min(lo, gnd) - 10, ymax = Math.max(hi, gnd) + 10;

  const X = i => padL + (i / Math.max(1, n - 1)) * w;
  const Y = v => padT + (1 - (v - ymin) / (ymax - ymin)) * h;

  /* 地面线 */
  g.strokeStyle = "#cbd5e0"; g.setLineDash([4, 4]);
  g.beginPath(); g.moveTo(padL, Y(gnd)); g.lineTo(W - padR, Y(gnd)); g.stroke();
  g.setLineDash([]);
  g.fillStyle = "#a0aec0"; g.font = "10px sans-serif";
  g.fillText("地面（最低足端）", padL + 4, Y(gnd) - 4);

  /* 每 1 秒的竖线 */
  const fps = 1 / 0.065;
  g.strokeStyle = "#edf2f7";
  for (let t = 1; t * fps < n; ++t) {
    const x = X(t * fps);
    g.beginPath(); g.moveTo(x, padT); g.lineTo(x, H - padB); g.stroke();
  }

  for (let leg = 0; leg < 4; ++leg) {
    g.strokeStyle = LEG_COLORS[leg]; g.lineWidth = 2;
    g.beginPath();
    for (let i = 0; i < n; ++i) {
      const px = X(i), py = Y(fs[i].y[leg]);
      if (i === 0) g.moveTo(px, py); else g.lineTo(px, py);
    }
    g.stroke();
  }

  /* 游标 */
  g.strokeStyle = "#e53e3e"; g.lineWidth = 2;
  g.beginPath(); g.moveTo(X(frame), padT); g.lineTo(X(frame), H - padB); g.stroke();

  /* 图例 */
  let lx = padL + 8;
  g.font = "11px sans-serif";
  for (let leg = 0; leg < 4; ++leg) {
    g.fillStyle = LEG_COLORS[leg];
    g.fillRect(lx, padT + 2, 12, 3);
    g.fillStyle = "#2d3748";
    g.fillText(LEG_NAMES[leg], lx + 16, padT + 7);
    lx += 132;
  }
  g.fillStyle = "#a0aec0";
  g.fillText("↑ 抬得高", 8, padT + 12);
}

/* ---------- 主循环 ---------- */

function drawAll() {
  drawGait();
  drawSide("left",  [0, 3], "左侧视");
  drawSide("right", [1, 2], "右侧视");
  drawTraj();
  const n = frames().length;
  $("frameLabel").textContent = (frame + 1) + " / " + n;
  $("slider").value = frame;
}

function tick() {
  const dt = parseInt($("speed").value, 10);
  if (playing) {
    frame = (frame + 1) % frames().length;
    drawAll();
  }
  timer = setTimeout(tick, dt);
}

$("play").onclick = () => {
  playing = !playing;
  $("play").textContent = playing ? "⏸ 暂停" : "▶ 播放";
};
$("slider").oninput = e => { frame = parseInt(e.target.value, 10); drawAll(); };
seqSel.onchange = e => { curSeq = parseInt(e.target.value, 10); setupSeq(); };

setupSeq();
tick();
</script>
</body>
</html>
"""


def swing_windows(fr_list, leg, half=10, thresh=4.0):
    """
    返回这条腿的摆动相窗口 [(起帧, 止帧), ...]。

    判据：该帧的足端高度比**前后 ±half 帧内的局部最低点**高 `thresh` 毫米以上。

    ⚠️ 为什么不用固定基准（这一条踩了两次）：
      1. 第一版用 `max(-y)` 当"地面" —— 那是**抬得最高**的一刻，符号反了，
         永远判不出抬腿；
      2. 第二版用"每条腿自己的全局最低点" —— 但机身姿态（滚转）会被
         WALK 的 `gesture()` 改，右腿整体有持续偏移，于是右腿被误判成整段都在抬。
    ⇒ 滑动窗口把"缓慢的机身漂移"和"几帧的抬腿"分开了。
    """
    y = [fr["y"][leg] for fr in fr_list]
    n = len(y)
    win, start = [], None
    for i in range(n):
        lo = min(y[max(0, i - half): min(n, i + half + 1)])
        swing = (y[i] - lo) > thresh
        if swing and start is None:
            start = i
        elif not swing and start is not None:
            win.append((start, i - 1))
            start = None
    if start is not None:
        win.append((start, n - 1))
    return win


def report_phase(seqs):
    """
    把"步态相位"用**数字**打出来。

    为什么值得做：可视化是为了让眼睛能检查，但"图对不对"不该只靠眼睛。
    这里把每条腿的摆动相窗口算出来，于是 TROT（对角同步）与
    WALK（1→2→3→4 依次）都能**用文字确认**。
    """
    print("\n---- 摆动相（抬腿）窗口，按帧 ----")
    for s in sorted(seqs):
        fr = seqs[s]
        print("  seq %d  (%d 帧):" % (s, len(fr)))
        for leg in range(4):
            w = swing_windows(fr, leg)
            body = ", ".join("%d-%d" % (a, b) for a, b in w[:8]) or "（整段都没抬）"
            if len(w) > 8:
                body += " …(共 %d 段)" % len(w)
            print("    %-10s %s" % (LEG_NAMES[leg], body))

    # TROT 的对角同步性：腿1 与 腿3 应基本同步，腿2 与 腿4 应基本同步
    fr = seqs.get(1)
    if fr and len(fr) > 60:
        w1, w3 = swing_windows(fr, 0), swing_windows(fr, 2)
        w2, w4 = swing_windows(fr, 1), swing_windows(fr, 3)

        def overlap(a, b):
            n = 0
            for (a0, a1) in a:
                for (b0, b1) in b:
                    n += max(0, min(a1, b1) - max(a0, b0) + 1)
            return n

        o13, o23 = overlap(w1, w3), overlap(w2, w3)
        print("\n  TROT 对角同步检查（seq 1）：")
        print("    腿1 ∩ 腿3 重叠帧 = %d   腿2 ∩ 腿3 重叠帧 = %d" % (o13, o23))
        if o13 > 0 and o23 == 0:
            print("    ✔ 腿1 与 腿3（对角）同步、腿2 与它们不同步 —— 符合 TROT")
        else:
            print("    ⚠ 与 TROT 的预期不符，需要人看一眼")

    # WALK 的依次性：腿1 应先抬，然后 2、3、4
    fr = seqs.get(2)
    if fr:
        firsts = []
        for leg in range(4):
            w = swing_windows(fr, leg)
            firsts.append(w[0][0] if w else -1)
        print("\n  WALK 顺序检查（seq 2）：各腿**首次**抬起帧 = %s" % firsts)
        order = sorted(range(4), key=lambda i: (firsts[i] if firsts[i] >= 0 else 10**9))
        print("    → 顺序 = %s" % " → ".join(LEG_NAMES[i] for i in order))
        if all(f >= 0 for f in firsts) and len(set(firsts)) == 4:
            print("    ✔ 四条腿在不同帧首次抬起（顺序步态，不是对角同步）")
        else:
            print("    ⚠ 四条腿的首抬帧有重复或缺失，需要人看一眼")


def main():
    # 控制台是 GBK，输出里有中文与箭头符号（成长手册 P-05）
    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    src = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "chain_trace.csv"
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE / "gait_viewer.html"

    if not src.is_file():
        raise SystemExit("找不到 %s（先跑 dump_chain_trace.exe）" % src)

    seqs = load(src)
    if not seqs:
        raise SystemExit("%s 里没有数据" % src)

    total = sum(len(v) for v in seqs.values())
    html = (HTML
            .replace("__DATA__", json.dumps(seqs, separators=(",", ":")))
            .replace("__SEQ_NAMES__", json.dumps(SEQ_NAMES, ensure_ascii=False))
            .replace("__LEG_NAMES__", json.dumps(LEG_NAMES, ensure_ascii=False))
            .replace("__LEG_COLORS__", json.dumps(LEG_COLORS))
            .replace("__L1__", str(L1))
            .replace("__L2__", str(L2))
            .replace("__BODY_L__", str(BODY_L))
            .replace("__SWING_LIFT_MM__", str(SWING_LIFT_MM)))

    out.write_text(html, encoding="utf-8")
    print("已写出 %s" % out)
    print("  序列 %d 个，共 %d 帧" % (len(seqs), total))
    for s in sorted(seqs):
        print("    seq %d: %d 帧  %s" % (s, len(seqs[s]), SEQ_NAMES.get(s, "")))

    report_phase(seqs)

    print("\n双击打开这个 HTML 即可（浏览器，无需联网、无需装任何东西）。")


if __name__ == "__main__":
    main()
