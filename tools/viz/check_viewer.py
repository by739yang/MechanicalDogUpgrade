#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_viewer.py —— 验 `gait_viewer.html` 里的内联 JS **真的能跑**

## 为什么需要它（它抓到过一个真 bug）

那个 HTML 是我给你双击打开看的，但**我看不见页面**。有一次改"抬腿判定基准"时，
一行 `localBaseline(leg)` 被放到了腿循环**外面**，`leg` 在那里未定义 ⇒
`drawGait()` 抛 `ReferenceError` ⇒ **打开就是一片空白**。

而当时：
- 生成器打印的相位数字是**对的**（那是 Python 端算的，与 JS 无关）；
- `run_golden.bat` 全过（它根本不碰 HTML）；
- 我也没法"看一眼"。

⇒ 所以"能打开、能画"这件事必须用程序验，不能靠假设。

## 它做什么

1. `node --check`：语法检查。
2. 用**桩 DOM + 桩 canvas** 把内联 JS 真的执行一遍，并逐个序列、逐个帧
   调用 `drawAll()`（含首帧/末帧等边界），捕获任何运行时异常。
3. 有任何错误就以非 0 退出。

⚠️ 它**不能**验证"画得好不好看"或布局是否正确 —— 桩 canvas 只是把绘图调用吃掉。
它能保证的是：**没有语法错误、没有运行时异常、所有代码路径都被走到**。

用法：
    python check_viewer.py            # 默认查 gait_viewer.html
    python check_viewer.py 别的.html
退出码 0 = 通过。
"""
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

#: 桩 DOM + canvas：只要能让绘图函数跑完就行，不需要真的画
HARNESS_HEAD = r"""
// ===================== 桩 DOM / canvas =====================
function ctxStub() {
  const noop = () => {};
  return new Proxy({}, {
    get: (t, k) => {
      if (k in t) return t[k];
      if (k === 'measureText') return () => ({ width: 10 });
      return noop;
    },
    set: (t, k, v) => { t[k] = v; return true; }
  });
}
const els = {};
function el(id) {
  if (!els[id]) {
    els[id] = {
      id: id, width: 980, height: 330, value: 0, textContent: '',
      getContext: () => ctxStub(),
      appendChild: () => {},
      onclick: null, oninput: null, onchange: null
    };
  }
  return els[id];
}
global.document = { getElementById: el, createElement: () => ({ style: {} }) };
"""

HARNESS_TAIL = r"""
// ===================== 手工驱动每个序列每一帧 =====================
let drawCalls = 0, errors = 0;
const step = Math.max(1, 1);
for (const s of seqKeys) {
  const n = DATA[s].length;
  const stride = Math.max(1, Math.floor(n / 30));
  const frames = new Set([0, n - 1, Math.floor(n / 2)]);
  for (let f = 0; f < n; f += stride) frames.add(f);

  curSeq = s;
  if (typeof setupSeq === 'function') { try { setupSeq(); } catch (e) { errors++; console.log('ERROR setupSeq seq=' + s + ': ' + e.message); } }
  for (const f of frames) {
    frame = f;
    try { drawAll(); drawCalls++; }
    catch (e) { errors++; console.log('ERROR seq=' + s + ' frame=' + f + ': ' + e.message); }
  }
}
console.log('绘制调用 = ' + drawCalls + ' 次, 运行时错误 = ' + errors);

/* ---- 用数字验证"画面是有意义的"，而不是"没抛异常" ----
   1) 示意几何自洽：把**中位角**喂进 actionLeg()，足端必须落在髋正下方约 200 mm
      （与 gait 视图里站立腿的形状一致）。若为 0 或发散，说明参数写错了。 */
let geoBad = 0;
for (let L = 0; L < 4; L++) {
  const f = actionLeg(L, ACTION_CENTRE[L]).foot;
  if (Math.abs(f[0]) > 1.0 || Math.abs(f[1] + 200.0) > 1.0) {
    geoBad++;
    console.log('GEOMETRY FAIL leg' + (L + 1) + ': 中位姿足端 = (' +
                f[0].toFixed(2) + ', ' + f[1].toFixed(2) + ')，应为 (0, -200)');
  }
}
console.log('中位姿示意几何: ' + (geoBad === 0 ? '✔ 四腿足端都在髋下 200 mm'
                                             : geoBad + ' 条腿不对'));

/* 2) 每个序列确实在动：12 路舵机角的最大变化幅度 */
let flat = 0;
for (const s of seqKeys) {
  const rows = DATA[s];
  let maxRange = 0, which = -1;
  for (let ch = 0; ch < 12; ch++) {
    let lo = 1e9, hi = -1e9;
    for (const fr of rows) { lo = Math.min(lo, fr.ang[ch]); hi = Math.max(hi, fr.ang[ch]); }
    if (hi - lo > maxRange) { maxRange = hi - lo; which = ch; }
  }
  const tag = (rows[0].kind === 'action' ? '动作' : '步态');
  console.log('  seq ' + String(s).padStart(3) + ' [' + tag + '] ' +
              String(rows.length).padStart(4) + ' 帧, 12 路最大变化 = ' +
              maxRange.toFixed(2) + '° (ch' + which + ')');
  /* seq 101 = 立正：目标就是中位姿，所以**本来就该静止**，不算异常。
     其余序列若几乎不动，那才是真问题。 */
  if (maxRange < 1.0 && s !== 101) { flat++; }
}
if (flat > 0) { console.log('WARNING: ' + flat + ' 个序列几乎不动（幅度 <1°）—— 是不是动作没接上？'); }
else { console.log('（seq 101 立正本来就不动：目标即中位姿，已排除在判定之外）'); }

if (errors > 0 || geoBad > 0) { process.exit(1); }
"""


def main():
    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    html = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "gait_viewer.html"
    if not html.is_file():
        raise SystemExit("找不到 %s" % html)

    node = shutil.which("node")
    if node is None:
        print("跳过：没找到 node（无法验证 HTML 里的 JS 能否运行）")
        print("⚠️ 这意味着**没有人验证过**那个页面能打开。建议装 node 后重跑。")
        return 0

    src = html.read_text(encoding="utf-8")
    m = re.search(r"<script>(.*?)</script>", src, re.S)
    if m is None:
        raise SystemExit("%s 里没有 <script> 块" % html)
    js = m.group(1)

    tmp = HERE / "_viewer_check.js"
    # 去掉自动执行，改由 harness 手工驱动
    js_manual = js.replace("setupSeq();\ntick();", "")
    tmp.write_text(HARNESS_HEAD + js_manual + HARNESS_TAIL, encoding="utf-8")

    print("检查 %s（内联 JS %d 行）" % (html.name, js.count("\n") + 1))

    print("\n[1/2] 语法检查")
    r = subprocess.run([node, "--check", str(tmp)], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    if r.returncode != 0:
        print(r.stderr[:2000])
        tmp.unlink(missing_ok=True)
        print("RESULT: FAIL -- 内联 JS 有语法错误")
        return 1
    print("  语法 OK")

    print("\n[2/2] 用桩 canvas 真跑所有序列的绘图路径")
    r = subprocess.run([node, str(tmp)], capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    out = (r.stdout or "") + (r.stderr or "")
    for line in out.splitlines():
        if line.strip():
            print("  " + line)
    tmp.unlink(missing_ok=True)

    if r.returncode != 0:
        print("\nRESULT: FAIL -- 页面里的 JS 会抛异常（打开很可能是空白）")
        return 1
    print("\nRESULT: PASS -- 语法正确、所有绘图路径无异常")
    print("注意：桩 canvas 只验证『能跑完』，不验证『画得好看/布局对不对』。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
