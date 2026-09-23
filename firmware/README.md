# firmware —— 机械狗 ESP-IDF C 迁移工程

> 阶段：**P1 完成**（P0 工程骨架/日志/I2C/PCA9685 + P1 配置 NVS 与纯数学模块）
> 依据：`../ESP-IDF_C迁移表.md` 的阶段 P0/P1；硬件事实来自 `../硬件实物核对清单.md`；
> 踩过的坑与解决方法见 `../问题与解决记录.md`

---

## 1. 硬件基线（2026-09-19 实测）

| 项目 | 值 |
|---|---|
| 主控 | ESP32（**非** S3），rev 3.1，4 MB Flash，CH340 → COM4 |
| 唯一 I2C 总线 | **SDA = GPIO21，SCL = GPIO22，100 kHz** |
| 舵机驱动 | PCA9685 **0x40**（左半身）、**0x41**（右半身）；`0x70` 是 all-call 广播地址 |
| IMU | **本机没有** —— 4 次独立验证（三条候选总线 + 9 引脚 × 72 有序组合双极性全扫描；位操作扫描 112 个地址仅命中 3 个） |
| 逻辑电源 | PCA9685 逻辑电走 **USB 5V**，不走电池那路（**实测：不插电池也能应答 `0x40`**） |
| 舵机电源 | 舵机 V+ 走电池（11.1 V 3500 mAh），经两片 HW-674(XL4016E1) 可调降压 |
| 原固件 | MicroPython 1.13.0 / ESP-IDF 3.3.2 |

## 2. 构建环境

| 项目 | 值 |
|---|---|
| PlatformIO Core | 6.1.19 |
| 平台 | **`espressif32@6.5.0`**（钉住版本） |
| ESP-IDF | **v5.1.2** |
| 框架 | `espidf` |

### 为什么钉住 `espressif32@6.5.0`

本机 `~/.platformio/packages/framework-espidf` 是 **ESP-IDF v5.1.2**，正对应 `espressif32@6.5.0`。
钉住版本可以复现，并避免重复下载框架包。

### 为什么用 legacy `driver/i2c.h`

IDF 5.1.2 **没有**新版 `driver/i2c_master.h`（IDF 5.2 才引入）。
因此 `bsp_i2c.c` 使用 legacy master API（`i2c_param_config` / `i2c_master_write_to_device` /
`i2c_master_write_read_device` / `i2c_master_read_from_device`）。
将来升级到 IDF ≥ 5.2 时再迁移到新 API。

## 3. 构建 / 烧录 / 监视

```powershell
cd C:\Users\boyi\Desktop\MechanicalDogUpgrade\firmware

# 编译
pio run

# 烧录（会覆盖板上的 MicroPython 固件，见 §6）
pio run -t upload

# 串口监视（115200，带异常解码）
pio device monitor
```

指定端口（默认会自动选 COM4）：

```powershell
pio run -t upload --upload-port COM4
pio device monitor --port COM4 --baud 115200
```

## 4. 上电行为

**上电不会让任何舵机动作**（全部通道被置为「无脉冲」= 舵机松力）。自检依次做六件事：

1. 从 NVS 载入配置（没有 / 损坏 / 版本不符 → 用默认值并打印原因）
2. I2C 总线恢复（发 9 个时钟脉冲，解开从机把 SDA 拉死的情况）
3. **位操作扫描**全总线 112 个地址，核对 `0x40` / `0x41` 是否都在
4. 安装 I2C 驱动，定点探测 `0x40` / `0x41` / `0x70`
5. 初始化两片 PCA9685（50 Hz），并把**全部通道置为「无脉冲」**
6. 回读 `MODE1` / `PRESCALE` 并打印，与 MicroPython 版实测值（`0x21` / `122`）对照

然后启动串口控制台等待命令。

> 为什么扫描要用**位操作**而不是 I2C 驱动：IDF 5.1.2 的 I2C 驱动把事件等待时间
> **硬性下限钉在 1000 ms**（`I2C_CMD_ALIVE_INTERVAL_TICK`），所以探测一个**不存在的地址**
> 每次都要等满 1 秒 —— 112 个地址就是 112 秒。位操作扫描没有这个下限：
> **112 个地址 21 ms 扫完**。详见 `../问题与解决记录.md` 的 P-13/P-14。

## 5. 串口控制台命令

| 命令 | 说明 |
|---|---|
| `help` | 显示帮助 |
| `scan` | 重新扫描 I2C 总线并核对 |
| `status` | 回读两片板的 `MODE1` / `PRESCALE` |
| `freq <hz>` | 设置频率（24..1526，默认 50） |
| `set <board> <ch> <us>` | 单通道输出指定脉宽（限制 500..2500 µs） |
| `deg <board> <ch> <0..180>` | 按 MicroPython 的换算输出对应脉宽 |
| `all <board> <us>` | 该板全部 16 路输出同一脉宽（⚠️ 会同时驱动所有舵机） |
| `off <board>` | 该板全部通道无脉冲（松力 / 安全态） |
| `sweep <board> <ch> <from> <to> <step> <delay_ms>` | 慢速往返扫动，总时长上限 30 s |
| `raw <board> <reg_hex>` | 读一个寄存器（调试） |
| `cfg` | 列出全部配置项与当前值 |
| `cfg get <key>` | 读一项 |
| `cfg set <key> <value>` | 改一项（越界会**自动限幅**并告诉你改了哪几项） |
| `cfg save` | 保存到 NVS（写前算 CRC） |
| `cfg load` | 从 NVS 重新载入 |
| `cfg reset` | 恢复出厂默认**并擦除** NVS |
| `cfg defaults` | 只把当前内存里的值恢复为默认（不写 NVS） |

`board`：`0` = 0x40（左半身），`1` = 0x41（右半身）。

### 配置项（`cfg` 支持的 key）

- **中位角**：`c1_hip` / `c1_knee` / `c1_shank` …… `c4_*`（腿1..腿4 各 3 个，共 12 项）
- **几何**：`r_h` / `r_l1` / `r_l2` 等（机身与腿长）
- **姿态**：`h_goal` / `pit_goal` / `rol_goal` 等
- **步态**：`faai` / `walk_faai` / `gyro_p` / `ma_case` / `joy_fwd_sign` / `cal_leg_sel`
- **机械臂**：`arm_*`
- **网络**：`ap_ssid` / `ap_password`

> `cfg set` 的越界值不会被拒绝，而是被**限幅**到合法区间并提示 —— 这样在串口调试时
> 不会因为打错一个字就丢掉整条命令。限幅规则在 `app_config.c` 的 `app_config_validate()`。

### 第一次单通道测试建议

```
status
set 0 0 1500          # 左半身 ch0 回中位
sweep 0 0 1300 1700 10 20
off 0
```

> ⚠️ **测试前必须架空狗腿或使用支架。** 先测 `off` 确认能松力，
> 再小范围慢速试动，确认方向与限位后再扩大范围。

## 6. 重要：烧录会覆盖 MicroPython

板上原固件是 MicroPython 1.13.0 + 学长的机器狗程序。`pio run -t upload` 会把它覆盖掉。

回滚方式：

- 板上的 MicroPython 脚本原件已备份在 `../micropython/`
  （另有 `_board_backup_20260910`、`_local_backup_before_board_sync_20260910`）
- MicroPython 固件 `micropython.bin` 在原始材料目录
  `机械狗毕业设计所有材料\程序\cx\主要程序\micropython.bin`
- 恢复：`esptool --chip esp32 --port COM4 write_flash -z 0x1000 micropython.bin`，
  再用 `mpremote` 把 `../micropython/*.py`、`*.html` 上传回板子

## 7. 验收标准

### P0（来自迁移表，2026-09-19 上机验收）

- [x] 工程可编译、可烧录
- [x] 串口日志正常（能看到构建信息与自检输出）
- [x] I2C 扫描看到 **0x40、0x41**
- [x] 回读 `MODE1` / `PRESCALE`，与 MicroPython 版实测一致（`0x21` / `122`）
- [ ] 12 个逻辑通道可**逐个**安全动作（每个单独 `set`，先架空）← 需要电池 + 舵机带电，归入 P2
- [x] `off` 能把整片板置为无脉冲

### P1（2026-09-19 验收）

- [x] 5 个纯数学模块与 MicroPython 参考**数值等价**（20 696 项对照，最大误差 ≤ 6.7e-5）
- [x] 上述对照在**宿主机**完成，不依赖板子（见 `../tools/golden/`）
- [x] 配置模块 68 条行为检查通过（默认值 / 限幅 / CRC / 版本 / 环回 / 擦除）
- [x] 配置**真机持久化**：`cfg set` → `cfg save` → 硬复位 → 值仍在
- [x] 越界值在真机上确实被限幅（`h_goal 9999` → 250）
- [x] 首次上电用默认值且不崩（NVS 为空 → `NOENT` + 默认值）

## 8. 目录结构

```
firmware/
├── platformio.ini
├── sdkconfig.defaults
├── README.md
└── src/
    ├── main.c                  入口：打印构建/芯片信息 -> app_p0_start()
    ├── app/
    │   ├── app_p0.{h,c}        P0 自检 + 串口控制台（含 cfg 命令挂载）
    │   ├── app_config.{h,c}    配置结构体 + 默认值 + 校验限幅 + CRC（纯 C）
    │   ├── app_config_nvs.{h,c} NVS 持久化后端（app_cfg_store_t 实现）
    │   └── app_cfg_cmd.{h,c}   cfg 控制台命令（offsetof 字段表，66 项）
    ├── bsp/
    │   └── bsp_i2c.{h,c}       I2C 主机总线（GPIO21/22, 100 kHz）+ 位操作扫描
    ├── drivers/
    │   └── drv_pca9685.{h,c}   PCA9685 驱动（双板）
    └── control/                从 PA_*.py 迁移的纯数学模块
        ├── kinematics.{h,c}          ← PA_IK.py        逆运动学
        ├── body_pose.{h,c}           ← PA_ATTITUDE.py  机身姿态 -> 足端
        ├── gait_trot.{h,c}           ← PA_TROT.py      TROT 步态
        ├── gait_walk.{h,c}           ← PA_WALK.py      WALK 步态（重心副作用改显式输出）
        └── filter_moving_avg.{h,c}   ← PA_AVGFILT.py   滑动平均（有状态）
```

### 分层与可测试性

`control/` 里全部是**纯函数、零 IDF 依赖**，所以能用宿主 gcc 直接编译并与
MicroPython 参考逐数值对照（`../tools/golden/`）。

`app_config.c` 同样零 IDF 依赖：持久化后端是**注入**进去的函数指针表
`app_cfg_store_t`，宿主机塞 RAM 假后端、真机塞 NVS 后端，上层逻辑一行不改。
这是 P1 能在**不插板子**的情况下验完的主要原因。

后续阶段按迁移表增加 `arm/`（arm_control）、`comm/`（wifi / http_server / websocket / telemetry）。

## 9. 已知与 MicroPython 版的差异（有意保留，便于对照）

1. **PRESCALE 不减 1**：MicroPython 的 `PA_SERVO.py` 用
   `prescale = int(25000000/4096/freq + 0.5)`（50 Hz → 122），
   而数据手册是 `round(25000000/(4096*freq)) - 1`（50 Hz → 121）。
   本驱动默认**复刻 MicroPython**（写 122），以保证脉冲一致。
   改 `DRV_PCA9685_PRESCALE_LEGACY` 为 `0` 即切换为手册公式。
2. **占空比满量程用 4095 而非 4096**：与 MicroPython 的 `_us2duty()` 一致。
3. **角度→脉宽换算** `drv_pca9685_us_from_degrees_ref()` 也是复刻 MicroPython
   （含整数截断），专供 P1 做 Python/C 对照。
