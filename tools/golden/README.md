# tools/golden —— 宿主侧 golden 对照测试

**目的**：把 MicroPython 参考实现和 C 移植版**逐数值对照**，替代"靠肉眼看数字"。

**为什么能在电脑上跑**：`PA_IK` / `PA_ATTITUDE` / `PA_TROT` / `PA_WALK` 都是**纯数学**模块，
只 `import math`，所以既能在 CPython 里直接 `exec`（生成参考值），
也能用宿主 gcc 编译（运行 C 版）——**不用烧板子**。

验收标准来自 `../../ESP-IDF_C迁移表.md`：

| 模块 | 容差 |
|---|---|
| IK 关节角 | < 0.5° |
| 步态轨迹 | < 1 mm |
| 姿态 8 输出 | < 0.5 mm |

---

## 怎么用

```powershell
cd tools\golden
.\run_golden.bat
```

它会做四步：生成参考值 → 找 gcc → 编译宿主测试 → 逐行比对并打印最大误差。

也可以分步：

```powershell
python gen_golden.py                                   # 只重新生成参考值
gcc -O2 -Wall -Wextra -std=c11 -I..\..\firmware\src `
    -o build\test_kinematics.exe test_kinematics.c `
    ..\..\firmware\src\control\kinematics.c -lm
build\test_kinematics.exe golden\ik.csv
```

---

## 设计要点

### 参考值来自原实现，不是我重写的

`gen_golden.py` 直接 `exec` `micropython/PA_IK.py`，把 `ik()` 拿出来当参考。
所以参考值**100% 来自学长的原始代码**，不存在"我理解错了原逻辑"的风险。

原始模块只 `import math`，能直接在 CPython 里跑。
`PA_TROT` / `PA_WALK` 里有 `from machine import I2C, Pin` 和 `import padog`，
到时候需要几十行 stub —— 但 `cal_t` / `cal_w` 本身是纯数学。

### 覆盖了两个分支

| case | 含义 | 实机状态 |
|---|---|---|
| 0 | 串联腿 | **实机在跑这个**（`config_s.py` 里 `ma_case=0`） |
| 1 | 并联腿 | ⚠️ 实机未验证过，但照样测（将来可能用） |

### 输入采样刻意覆盖边界

`ik_samples()` 生成 151 个点，包含：
- `x == 0`（走原实现的 `else` 分支，用到那个 `pi - 1.5707` 怪写法）
- `x > 0` / `x < 0`（两个 `if` 分支）
- 接近最小/最大可达半径（9.0 ~ 267.0 mm，留 1 mm 余量避免 `acos` 浮点越界）

### 测试程序输出用 ASCII

不是偷懒 —— 这是 `../../问题与解决记录.md` 里 **P-06** 的教训：
程序往 Windows 控制台打非 ASCII 会踩代码页坑。数值工具用英文最稳。

---

## 当前结果（2026-09-19）

```
kinematics golden test  (C float vs MicroPython double)
rows     : 74   (case=0 series: 37, case=1 parallel: 37)
samples  : 592 joint angles compared
tolerance: 0.500 deg

per-field max error (deg):
  ham1   0.0000211    shank1  0.0000250
  ham2   0.0000302    shank2  0.0000357
  ham3   0.0000421    shank3  0.0000207
  ham4   0.0000555    shank4  0.0000603

RESULT: PASS -- max error 0.0000603 deg  (8290x inside the tolerance)
```

**结论**：`firmware/src/control/kinematics.c` 与 `PA_IK.py` 数值等价。
残余的 6e-5 度纯粹是 C `float` 与 Python `double` 的舍入差
（固件刻意用 `float`：ESP32 只有单精度硬件 FPU，`double` 是软件模拟，慢一两个数量级）。

---

## 加新模块时怎么做

1. 在 `gen_golden.py` 里加一个 `gen_<name>()`，从对应 `micropython/PA_*.py` 生成 CSV
2. 写一个 `test_<name>.c`，读 CSV、调用 C 版、逐项比对
3. 在 `run_golden.bat` 里加一行编译+运行
4. **把生成的 CSV 提交进仓库** —— C 测试只读它，不依赖 Python

## 已完成 / 待办

| 模块 | 参考来源 | 状态 |
|---|---|---|
| `kinematics.c` | `PA_IK.py` | ✅ 通过（592 项，6e-5°） |
| `body_pose.c` | `PA_ATTITUDE.py` | ⬜ 待做 |
| `gait_trot.c` | `PA_TROT.py` | ⬜ 待做（需 `machine`/`padog` stub） |
| `gait_walk.c` | `PA_WALK.py` | ⬜ 待做（同上） |
| `filter_moving_avg.c` | `PA_AVGFILT.py` | ⬜ 待做 |
