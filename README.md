# MechanicalDogUpgrade

本仓库用于备份“基于灯哥开源的四足机器狗”原始 MicroPython 程序，并持续整理 ESP-IDF C 版本升级资料。

## 目录说明

```text
.
├── README.md                   # 本文件
├── HANDOFF.md                  # 当前状态、决策和下一阶段交接说明（新对话先读这个）
├── ESP-IDF_C迁移表.md          # MicroPython 到 ESP-IDF C 的迁移计划与验收标准
├── 硬件实物核对清单.md         # 上电前的实物核对项（含 2026-09-19 实测记录）
├── 问题与解决记录.md           # 踩过的坑与解决方法（成长手册）
├── firmware/                   # ESP-IDF C 工程 —— P0 已完成
│   ├── platformio.ini
│   ├── sdkconfig.defaults
│   ├── README.md               # 构建 / 烧录 / 串口控制台命令
│   └── src/
│       ├── main.c              # app_main：打印构建与芯片信息
│       ├── bsp/                # bsp_i2c：I2C 总线、扫描、位操作扫描
│       ├── drivers/            # drv_pca9685：双板 PCA9685 驱动
│       ├── control/            # 纯数学模块（从 PA_*.py 迁移）
│       │   ├── kinematics       #   逆运动学        ← PA_IK.py
│       │   ├── body_pose        #   机身姿态→足端   ← PA_ATTITUDE.py
│       │   ├── gait_trot        #   TROT 步态       ← PA_TROT.py
│       │   ├── gait_walk        #   WALK 步态(含重心副作用) ← PA_WALK.py
│       │   └── filter_moving_avg#   滑动平均(有状态) ← PA_AVGFILT.py
│       └── app/                # app_p0：P0 自检 + 串口控制台
├── tools/
│   └── golden/                 # 宿主侧 golden 对照测试（不用烧板子）
│       ├── README.md           # 用法与设计说明
│       ├── gen_golden.py       # 从原始 MicroPython 模块生成参考值
│       ├── mpy_stubs.py        # 给 machine / padog 提供最小 stub
│       ├── check_mpy_loadable.py  # 验证哪些模块能在 CPython 里加载
│       ├── golden/*.csv        # 参考向量表（已提交，C 测试只读它）
│       ├── test_*.c            # 宿主测试程序（每个模块一个）
│       └── run_golden.bat      # 一键：生成 + 编译 + 比对（全部套件）
├── diagnostics/                # 上机诊断脚本（Python，跑在电脑上）
│   ├── board_scan_i2c.py       # 扫描三条候选总线
│   ├── board_scan_all_pins.py  # 全引脚双极性权威扫描
│   ├── board_find_i2c.py       # 早期版本，保留为 readfrom_mem 假阳性的反面教材
│   ├── capture_boot_log.py     # 硬复位并抓完整启动日志 + 自动判定关键项
│   ├── send_cmd.py             # 向串口控制台发命令并抓回显
│   └── hard_reset.py           # 用 DTR/RTS 硬复位（mpremote 进不去时用）
├── micropython/                # 原始机器狗 MicroPython 运行代码（行为参考）
│   ├── main.py / padog.py / PA_*.py / mech_arm.py
│   ├── web_*.py / control.html / drive.html / cal.html
│   ├── config.example.py       # Wi-Fi 脱敏模板
│   └── tools/                  # 原始烧录辅助脚本
└── esp32cam/                   # ESP32-CAM 程序，Wi-Fi 已脱敏
    └── esp32cam_v5.ino.example
```

## 硬件基线（2026-09-19 实测）

| 项目 | 值 |
|---|---|
| 主控 | ESP32（**非** S3），rev 3.1，4 MB Flash，CH340 → COM4 |
| 唯一 I2C 总线 | **SDA = GPIO21 / SCL = GPIO22 / 100 kHz** |
| 舵机驱动 | PCA9685 **0x40**（左半身）、**0x41**（右半身）；`0x70` 是 all-call 广播地址 |
| IMU | **本机没有** —— 4 次独立验证（三条候选总线 + 9 引脚 × 72 有序组合双极性全扫描；位操作扫描 112 个地址仅命中 3 个） |
| 逻辑电源 | PCA9685 逻辑电走 **USB 5V**，不走电池那路（已实测） |
| 原固件 | MicroPython 1.13.0 / ESP-IDF 3.3.2 |

## 进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| **P0** | 工程、日志、I2C 扫描、PCA9685 单通道控制 | ✅ **完成**（2026-09-19 上机验收） |
| P1 | 配置迁移 NVS + 逆运动学 + 姿态/步态纯数学 | 🟡 **进行中** — **纯数学模块全部迁移完毕（5/5）**：golden 测试台跑通 5 套、合计 20 696 项，最大误差 ≤ 6.7e-5（整数项精确相等）；已迁移 `kinematics` / `body_pose` / `gait_trot` / `gait_walk` / `filter_moving_avg`；**待做 NVS 配置** |
| P2 | 固定周期运动循环 + 12 路舵机输出 | ⬜ 未开始 |
| P3 | TROT / WALK 步态与姿态数学 | ⬜ 未开始 |
| P4 | IMU 与稳定控制 | ⛔ 本机无 IMU（决策 F1 = A：暂不加装，P4 再议） |
| P5 | Web 通信重构 | ⬜ 未开始 |
| P6 | 机械臂、遥测、电源保护 | ⬜ 未开始 |

> 注意：C 版目前**只完成了底层**。烧录 C 固件后，狗**不会走路、没有网页、机械臂不可用** ——
> 这些都在 P1~P6。MicroPython 版仍保留为行为对照基准。

## 安全说明

- 公开仓库中没有真实 Wi-Fi 账号和密码。
- 使用 MicroPython 代码前，将 `micropython/config.example.py` 复制为 `config.py`，再填入自己的 Wi-Fi 信息。
- 使用 ESP32-CAM 示例时，把 `YOUR_WIFI_SSID` 和 `YOUR_WIFI_PASSWORD` 替换为本机 2.4 GHz 网络信息。
- 不要提交真实 `config.py`、`secrets.h`、固件 `.bin`、测试图片和本地缓存。
- 板上 MicroPython 文件的完整备份（含真实 `config.py`）保存在本机 `_board_backup_20260919/`，
  该目录被 `.gitignore` 忽略，**不会进入公开仓库**。

## C 版本目标

- VS Code + PlatformIO
- ESP-IDF 框架
- C 语言
- FreeRTOS
- 固定周期运动控制、命令队列、安全超时、IMU 闭环
