# tools/viz —— 步态/动作可视化（不需要板子）

**目的**：把逐帧的关节角与足端轨迹画出来，**让眼睛也能检查**。
数值对照对"四条腿该依次抬却对角同步"这类**结构性错误完全沉默**，画出来一眼就看见。

## 怎么用

```powershell
cd tools\viz

# 1) 从控制链导出逐帧轨迹（中间量，golden CSV 里没有）
gcc -O2 -Wall -Wextra -Werror -std=c11 -I..\..\firmware\src -I..\golden ^
    -o dump_chain_trace.exe dump_chain_trace.c ^
    ..\..\firmware\src\control\control_chain.c ^
    ..\..\firmware\src\control\control_chain_cmd.c ^
    ..\..\firmware\src\control\kinematics.c ^
    ..\..\firmware\src\control\body_pose.c ^
    ..\..\firmware\src\control\gait_trot.c ^
    ..\..\firmware\src\control\gait_walk.c ^
    ..\..\firmware\src\control\servo_map.c -lm
dump_chain_trace.exe ..\golden\golden\control_chain_seq.csv ^
    ..\golden\golden\control_chain_seq_cmds.csv chain_trace.csv

# 2) 生成自包含 HTML
python make_viz.py

#    （chain_trace.csv 与 *.exe 都是可重建的中间产物，**不进仓库**）

# 3) ★ 验页面真的能跑（见下）
python check_viewer.py
```

然后**双击 `gait_viewer.html`**：浏览器打开，无需联网、无需装任何东西。
可切序列、播放/暂停、拖进度、调速。

## 看到的是什么

| 面板 | 看什么 |
|---|---|
| 步态图 | 4 条横道（每条腿一行），**彩色段 = 抬腿**、灰段 = 落地；竖线 = 当前帧。TROT 应是对角同步，WALK 应是 1→2→3→4 依次 |
| 左右侧视 | 机身 + 髋/膝/足三点 + 大小腿连杆，逐帧动 |
| 足端高度曲线 | 四条腿的离地高度随时间，相位差与抬腿高度最直观 |

## ⚠️ 画面**不是**证据

- **足端位置 = 精确数据**（来自被 golden 逐帧钉住的控制链输出）；
- **膝的位置 = 按大小腿长度解算的示意图**（原版只给足端目标）；
- **髋的横摆角没有画进来** ⇒ 这是**侧视示意**，不是三维姿态。

它能看出"顺序对不对、方向对不对"这类粗大错误，**看不出零点几度**。

## ⚠️ 为什么必须有 `check_viewer.py`

这个 HTML 是**给人双击打开的**，而写它的人（我）**看不见页面**。已经踩过一次：

> 改"抬腿判定基准"时，一行 `localBaseline(leg)` 被放到了腿循环**外面**，
> `leg` 在那里未定义 ⇒ `drawGait()` 抛 `ReferenceError` ⇒ **打开是一片空白**。
>
> 而当时生成器打印的相位数字是**对的**（那是 Python 端算的，与 JS 无关），
> `run_golden.bat` 也全过（它根本不碰 HTML）—— **没有任何检查会失败**。

`check_viewer.py` 做两件事：
1. `node --check` 语法检查；
2. **用桩 DOM + 桩 canvas 把内联 JS 真执行一遍**，遍历全部序列的绘图路径
   （含首帧/末帧等边界），捕获运行时异常。

**改完 HTML 或 `make_viz.py` 之后一定要跑它。** 它不能验证"好不好看/布局对不对"，
但能保证"能打开、不抛异常、所有绘图路径都走得到"。

## 排查顺序（图不对时）

**先量原始数字，再在图里找原因。** `make_viz.py` 末尾会把每条腿的摆动相窗口
**用文字打出来**（以及 TROT 对角同步、WALK 顺序两项判定）—— 那是与画面独立的第二条证据链。
这一条也踩过两次：抬腿判据先是符号反了，后又把"机身滚转造成的持续偏移"误判成整段抬腿。
