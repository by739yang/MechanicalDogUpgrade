# HANDOFF：MechanicalDogUpgrade 代码移植交接文档

> 更新时间：2026-09-11（正文）／2026-09-19（最新状态见下方）
> 用途：下一轮新对话开始 ESP-IDF C 版本移植时，先完整阅读本文件，再阅读 `ESP-IDF_C迁移表.md` 和 `micropython/` 下的原始代码。
> 本文件是当前项目状态的权威说明。不要根据论文中的理想架构直接假设实物已经实现那些功能。

---

## ⚠️ 2026-09-19 重大更新（读下文之前先看这段）

本轮上机实测**推翻了下面若干条旧结论**，以本段为准：

1. **本机没有 IMU。** 用 `i2c.scan()`（权威 ACK 判定）＋ 全引脚**双极性**扫描
   （9 个引脚 × 72 个有序组合）确认：整块板子上只有一个 I2C 总线
   （**SCL=GPIO22 / SDA=GPIO21**），上面是两片 PCA9685（`0x40`、`0x41`）。
   **18/19、32/33 以及任何其它引脚组合上都扫不到 `0x68/0x69`。**
   ⇒ 下文"MPU6050 是最重要的硬件阻塞项"应改为 **"待决策：是否加装 IMU"**。
2. **已决定（F1 = A）：暂不加装 IMU。** 先完成 P0–P3（工程 / 舵机 / 开环步态 / Web），P4 再补。
3. **参考电路图不能用于推断本机引脚分配。** 那张图是灯哥菠萝狗官方课件，
   把 21/22 分给 IMU；但实物上 21/22 挂的是 PCA9685。它只能用于理解设计意图与电源拓扑。
4. **实物接线与代码一致**：PCA9685 确实在 21/22，`PA_SERVO.py` **不用改引脚**。
   实测 `MODE1=0x21`、`PRESCALE=122`（≈50.2 Hz），与 `PA_SERVO.py` 的 `freq=50` 吻合，
   说明舵机驱动链路是通的。
5. **新发现的软件问题**：`padog.do_connect_STA()` 是 `while not wifi.isconnected(): pass`
   —— **无超时的死循环**，热点不在就卡死在启动阶段（本轮就被卡住过，需硬复位才恢复）。
   C 版必须加超时与失败降级。
6. **P0 已开始**：`firmware/` 已建立（PlatformIO + ESP-IDF v5.1.2 + C + FreeRTOS），
   含串口日志、I2C 扫描、PCA9685 单通道驱动与串口控制台。详见 `firmware/README.md`。
7. 板上 MicroPython 全部 20 个文件（含真实 `config.py`）已备份到
   `_board_backup_20260919/`（该目录被 `.gitignore` 忽略，不会进公开仓库）。
8. **IDF 5.1.2 legacy I2C 驱动的一个硬陷阱（查源码确认）**：
   `components/driver/i2c/i2c.c` 里有
   ```c
   #define I2C_CMD_ALIVE_INTERVAL_TICK (1000 / portTICK_PERIOD_MS)
   ```
   在 `i2c_master_cmd_begin()` 的事件等待循环里，等待时间被**强制抬到不低于
   1000 ms**。结果是：器件 **ACK** 时 DONE 事件立刻到达（实测 273~491 µs），
   器件 **NACK** 时**根本不产生事件**，只能干等 1000 ms 后报 `ESP_ERR_TIMEOUT`。
   ⇒ 探测一个不存在的地址固定要 **1 秒**，112 个地址的全总线扫描要 **112 秒**。
   `ticks_to_wait` 和 `i2c_set_timeout()` **都改不了**（默认 SCL 超时实测是 8000）。
   MicroPython 1.13 用的是 IDF 3.3.2，那版没这个下限，所以同样扫描只要 28 ms。
   **对策**：全总线扫描用 **GPIO 位操作**在驱动安装前做（`bsp_i2c_scan_bitbang()`），
   约 20 ms；驱动路径只探"确定存在"的地址。这一条对所有后续阶段都适用 ——
   任何 I2C 错误路径都会有 1 秒延迟，设计超时/重试时必须考虑。

详细的实测数据、脚本与核对清单见《硬件实物核对清单.md》（放在桌面，§0.2.1 与附录 G）
以及 `diagnostics/` 目录。

---

## 0. 新对话开始时必须知道的事

1. 目标是：**VS Code + PlatformIO + ESP-IDF 框架 + C + FreeRTOS**，把现有 MicroPython 机器狗迁移成正式的嵌入式 C 工程。
2. 现有 MicroPython 程序是行为参考，不是要逐行翻译的对象。
3. 当前运行版本主要是**开环固定轨迹 + 逆运动学 + PCA9685 舵机输出**，不是论文中描述的完整 CPG/Kalman 闭环。
4. MPU6050 驱动代码存在，但实测确认**本机没有 IMU**（2026-09-19，全引脚双极性扫描）。
   姿态闭环的阻塞项已从"硬件找不到"改为 **"是否决定加装"**。
5. Web 控制是阻塞式 HTTP GET 轮询，交互差和走路不稳是已知问题，不要在 MicroPython 中继续大修，除非只是临时小修。
6. 不要把真实 Wi-Fi 密码、固件 `.bin`、测试图片和本地缓存上传到公开 GitHub。
7. 新对话的第一个任务不是写完整步态，而是先建立 PlatformIO + ESP-IDF 工程，跑通日志、I2C 扫描和 PCA9685 单通道驱动。
   **（此步已于 2026-09-19 开始，工程在 `firmware/`，见 `firmware/README.md`。）**

---

## 1. 用户目标与开发方向

用户接手的是学长毕设：基于灯哥开源四足机器狗的四足机器人项目，想把它作为嵌入式练手项目。

最终方向：

- 主线：ESP-IDF C 版本。
- 开发环境：VS Code + PlatformIO。
- 语言：C。
- 实时系统：FreeRTOS。
- 初期硬件：继续使用现有 ESP32 + 两片 PCA9685 + 现有舵机。
- 后期可选：把实时控制下放到 STM32 或 RP2040，ESP32 负责 Wi-Fi、网页和后期视觉。
- 暂时不要优先做：SLAM、激光雷达、复杂视觉、完整自主导航。

用户偏好：

- 希望解释直接、工程化。
- 希望能在 VS Code 里构建、上传、监控。
- 不想再依赖 uPyCraft。
- 接受先验证硬件和底层驱动，再写高层算法。

---

## 2. 重要路径

### 本机原始材料

```text
C:\Users\boyi\Desktop\机械狗毕业设计所有材料
```

### 当前实际运行的 MicroPython 程序

```text
C:\Users\boyi\Desktop\机械狗毕业设计所有材料\程序\cx\主要程序
```

重要文件和备份：

```text
main.py
padog.py
PA_SERVO.py
PA_IMU.py
PA_ATTITUDE.py
PA_IK.py
PA_TROT.py
PA_WALK.py
PA_STABLIZE.py
mech_arm.py
web_c.py
web_common.py
web_ctl.py
config.py
config_s.py
control.html
drive.html
cal.html
_board_backup_20260910\
_local_backup_before_board_sync_20260910\
```

### GitHub 本地仓库和升级目录

```text
C:\Users\boyi\Desktop\MechanicalDogUpgrade
```

仓库关系：

- 正确的 GitHub 仓库：`https://github.com/by739yang/MechanicalDogUpgrade`
- 本地远端应为：`https://github.com/by739yang/MechanicalDogUpgrade.git`
- 不要再把新内容推送到 `by739yang.github.io`。
- `by739yang.github.io` 之前误上传的内容已清空，但仓库本身还存在。
- 当前 GitHub token 没有 `delete_repo` 权限，因此没有通过 API 删除该仓库。
- 如果要删除 `by739yang.github.io` 仓库本身，需要在 GitHub 网页 Settings 的 Danger Zone 手动删除。

---

## 3. 硬件现状

### 主控制器

- ESP32 DevKit 类开发板。
- 电脑识别为 `USB-SERIAL CH340 (COM4)`，但这可能随 USB 口和机器变化。
- 串口日志波特率：115200。
- 当前 Type-C 主要连接 ESP32，涉及下载、调试和逻辑供电。
- 没有确认 Type-C 是锂电池充电口，不能把它当充电口使用。

### 舵机总线

- I2C0 主总线：
  - SDA：GPIO21
  - SCL：GPIO22
  - 频率：100 kHz
- 代码中的备用 I2C 引脚：
  - SDA GPIO18 / SCL GPIO19
  - SDA GPIO33 / SCL GPIO32
- 物理接线必须再确认，不要只按代码推断。

### PCA9685

- 左侧板：I2C 地址 `0x40`
- 右侧板：I2C 地址 `0x41`
- 启动扫描最后看到的设备：`0x40`、`0x41`、`0x70`
- 其中 `0x70` 可能是 PCA9685 的广播/相关地址，不是 MPU6050。

### PCA9685 逻辑通道

| 逻辑通道 | 板卡 | 板载通道 | 含义 |
|---:|---|---:|---|
| 0 | 0x40 | ch0 | 左前髋 |
| 1 | 0x40 | ch1 | 左前大腿 |
| 2 | 0x40 | ch2 | 左前小腿 |
| 3 | 0x40 | ch3 | 左后髋 |
| 4 | 0x40 | ch4 | 左后大腿 |
| 5 | 0x40 | ch5 | 左后小腿 |
| 6 | 0x41 | ch0 | 右前髋 |
| 7 | 0x41 | ch1 | 右前大腿 |
| 8 | 0x41 | ch2 | 右前小腿 |
| 9 | 0x41 | ch3 | 右后髋 |
| 10 | 0x41 | ch4 | 右后大腿 |
| 11 | 0x41 | ch5 | 右后小腿 |

### 机械臂当前配置

| 机构 | 板卡 | 通道 | 代码配置 |
|---|---|---:|---|
| 大臂 | 0x40 | ch6 | `arm_upper_ch=6` |
| 小臂 | 0x40 | ch7 | `arm_fore_ch=7` |
| 夹爪 | 0x41 | ch6 | `arm_grip_gpio=-1`，走 PCA9685 |
| 夹爪备用 GPIO | 可选 GPIO12 | 50 Hz PWM | 当前未启用 |

源码里曾把 `0x40 ch6` 注释成“底座”，但当前运行配置是把它作为大臂通道；迁移前必须逐通道确认机械臂真实映射。

### IMU（2026-09-19 已结案）

- **结论：本机没有 IMU。** 不是接线错、不是地址错、不是缺上拉 —— 是实物上没有这个器件。
- 已排除的路径：
  - `SCL22/SDA21`（代码主用）→ 只有 `0x40`、`0x41`、`0x70`
  - `SCL19/SDA18`（参考图标的 IMU 总线）→ 空
  - `SCL32/SDA33`（代码的 IMU 回退）→ 空
  - 9 个引脚 × 72 个**有序**组合（双极性）全引脚扫描 → 只有 `SCL22/SDA21` 出现器件
  - 开内部上拉后重扫 → 仍为空
- 决策：**F1 = A，暂不加装 IMU**，先完成 P0–P3，P4 再补。
- 若将来加装：直接挂到唯一总线 `GPIO21/22`（地址 `0x68`）即可，**不需要改到别的引脚**。

### 电源与视觉

- 论文声称采用 11.1 V 锂电池组，但当前没有验证实物电源树。
- 当前代码没有电压、电流、舵机母线监测。
- 用户看到过蓝色电路板和圆柱形元件，但尚不能确认 BMS、充电模块、降压模块或电容接线。
- 迁移时如果加入电压/电流检测，应作为新增硬件单独设计，不能凭空指定引脚。
- 用户之前反馈 ESP32-CAM 实际已经损坏；视觉不是当前 P0/P1 迁移任务。
---

## 4. 当前 MicroPython 软件状态

### 启动结构

`main.py` 当前做两件事：

1. 启动 Web 线程。
2. 在主循环中调用 `padog.mainloop()`。

当前 `web_ui_mode=1`，即使用：

```text
web_c.py + control.html
```

### 控制链

```text
main.py
  ├── web_thread()
  │     └── web_c.py / web_ctl.py
  └── control_loop()
        └── padog.mainloop()
              ├── PA_TROT.py / PA_WALK.py
              ├── PA_ATTITUDE.py
              ├── PA_IK.py
              ├── servo_output()
              │     └── PA_SERVO.py
              │           └── PCA9685 0x40 / 0x41
              └── mech_arm.tick()
```

### 当前行为定性

- 运行方式：开环位置控制。
- 步态：硬编码 TROT 和 WALK 轨迹。
- 姿态：`PA_ATTITUDE.py` 产生目标姿态偏移，但不是来自 IMU 的实时闭环。
- IMU 稳定：`PA_STABLIZE.stab()` 在主循环中被注释掉。
- Web：同步阻塞 socket，HTTP GET 轮询。
- Web 轮询周期：`control.html` 约 120 ms；`drive.html` 狗约 80 ms、机械臂约 35 ms。
- 没有 WebSocket、命令序号、时间戳、心跳和过期命令丢弃。
- 没有足端接触检测、舵机电流检测和真正的运动平衡控制器。

### 已知上次运行日志

曾在 2.4 GHz 手机热点环境中成功连接：

```text
I2C0 scan: ['0x40', '0x41', '0x70']
network config: ('10.23.178.215', ...)
listening on: ('10.23.178.215', 80)
```

注意：

- `10.23.178.215` 是当时的 DHCP 地址，不是固定 IP。
- 当前热点配置可能已经改变。
- 手机热点必须是 2.4 GHz。
- 真实 Wi-Fi 配置只保存在本机 `config.py`，不要提交公开仓库。

### 已知历史故障

- uPyCraft 曾把板上的 `main.py` 截断到 256 字节，导致 `SyntaxError`。
- 已从 `_board_backup_20260910` 和 `_local_backup_before_board_sync_20260910` 恢复。
- 后续不要再使用 uPyCraft 做主要开发。
- 烧 C 固件前应再次用 `mpremote` 或其它安全方式备份板上的 MicroPython 文件。

---

## 5. GitHub 和文件安全状态

公开仓库：

`https://github.com/by739yang/MechanicalDogUpgrade`

仓库性质：

- Public。
- 描述：本仓库用于备份“基于灯哥开源的四足机器狗”原始 MicroPython 程序，并持续整理 ESP-IDF C 版本升级资料。
- 默认分支：`main`。
- 初始备份提交：`18785a1`。
- 当前远端和本地分支已同步。

已在仓库中脱敏：

- `micropython/config.example.py`
- `esp32cam/esp32cam_v5.ino.example`

没有提交：

- 真实 Wi-Fi 密码。
- `micropython.bin`。
- `test/` 大体积测试文件。
- `__pycache__`、`.pyc`。
- 本地旧备份目录。
- ESP32-CAM 第三方库完整目录。

`.gitignore` 已忽略：

```text
config.py
config.local.py
secrets.h
wifi_config.h
.env
__pycache__/
*.pyc
.pio/
build/
*.bin
_board_backup_*/
_local_backup_*/
test/
*.log
```

安全原则：

- 任何含真实密码的配置只保留本机。
- 需要示例时使用 `YOUR_WIFI_SSID` 和 `YOUR_WIFI_PASSWORD`。
- 不要把 GitHub token 打印到终端或写进文件。

---

## 6. 已经确定的技术决策

### 最终主路线

```text
VS Code
  + PlatformIO
  + ESP-IDF framework
  + C
  + FreeRTOS
```

暂不建议：

- 继续用 MicroPython 做完整重构。
- 先做摄像头和 SLAM。
- 一开始就直接把全部控制迁移到 STM32。

### 建议的第一版 PlatformIO 配置

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = espidf
monitor_speed = 115200
upload_speed = 921600
```

实际板卡若不是标准 `esp32dev`，要按 ESP32 型号和 Flash 参数调整。

### 建议的 C 工程位置

```text
C:\Users\boyi\Desktop\MechanicalDogUpgrade\firmware
```

建议结构：

```text
firmware/
├── platformio.ini
├── src/
│   ├── main.c
│   ├── app/
│   ├── bsp/
│   ├── drivers/
│   ├── control/
│   ├── arm/
│   └── comm/
└── web/
```

### 任务划分

| 任务 | 核心 | 目标频率 | 职责 |
|---|---:|---:|---|
| `motion_task` | Core 1 | 100~200 Hz | 步态、姿态目标、IK、舵机输出 |
| `imu_task` | Core 1 | 200~500 Hz，初期 200 Hz | IMU 读取、滤波、姿态估计 |
| `comm_task` | Core 0 | 事件驱动，命令 20~50 Hz | Wi-Fi、HTTP、WebSocket、命令队列 |
| `arm_task` | Core 1 或 0 | 50~100 Hz | 机械臂和夹爪平滑控制 |
| `safety_task` | Core 1 | 50~100 Hz | 心跳、超时、急停、看门狗 |
| `telemetry_task` | Core 0 | 10~20 Hz | 姿态、电压、错误码上报 |

初期为了减少 I2C 竞争，也可以先让 `motion_task` 同时读 IMU；后续再拆分。
---

## 7. MicroPython 到 C 的迁移顺序

### 阶段 P0：工程和硬件确认

- 创建 PlatformIO + ESP-IDF 工程。
- 配置串口日志和 `app_main`。
- 实现 I2C 扫描。
- 确认能看到 `0x40`、`0x41`。
- 确认 MPU6050 是否存在。
- 实现 PCA9685 单通道控制。
- 只测试一个舵机，先架空或不装机械负载。

### 阶段 P1：配置和数学模块

- `config.py` / `config_s.py` 迁移到 NVS。
- `PA_IK.py` 迁移为 `kinematics.c`。
- `PA_ATTITUDE.py` 迁移为 `body_pose.c`。
- `PA_TROT.py` / `PA_WALK.py` 迁移为 `gait_trot.c` / `gait_walk.c`。
- 用 Python 结果做离线 golden test。

### 阶段 P2：固定周期舵机输出

- 实现 `motion_task`。
- 映射 12 路逻辑舵机。
- 加角度限幅和速率限制。
- 实现急停和超时停车。
- 目标：站立 10 分钟不重启。

### 阶段 P3：步态

- 原地踏步。
- 低速直行。
- 原地转向。
- 低速连续转向。
- 最后再做 WALK 和不平地面。

### 阶段 P4：IMU 闭环

- 先解决 MPU6050 硬件。
- 实现 `drv_mpu6050.c`。
- 实现互补滤波或 Mahony/Madgwick。
- 再接 `balance_controller.c`。
- `PA_STABLIZE.py` 不能直接照搬，因为原版未形成有效闭环。

### 阶段 P5：Web 和通信

- 用 `esp_http_server` 提供页面。
- 用 WebSocket 或低频结构化命令替代 HTTP GET 轮询。
- 命令包含 `seq`、时间戳、速度、转向、姿态、身高、机械臂、急停和心跳。
- 只执行最新有效命令。
- 200~300 ms 心跳超时后自动停车。

### 阶段 P6：机械臂和电源遥测

- 机械臂平滑控制。
- 夹爪限位。
- 加入舵机母线电压/电流检测。
- 记录跌落并触发保护。

---

## 8. 验收标准

| 项目 | 验收标准 |
|---|---|
| I2C 扫描 | 0x40、0x41 必现；MPU 如果存在应看到 0x68 或 0x69 |
| PCA9685 | 12 个逻辑通道都能安全单独动作 |
| 控制循环 | 100~200 Hz，平均抖动尽量小于 2 ms |
| 急停 | 小于 100 ms 生效 |
| 通信 | 命令 20~50 Hz，无旧命令覆盖新命令 |
| 心跳 | 200~300 ms 超时后停车或进入安全姿态 |
| 站立 | 10 分钟不重启 |
| 原地踏步 | 60 秒不摔，初期用支架或低高度 |
| 低速直行 | 平地 3 米不失控 |
| IK 对照 | 与 Python 参考角度误差小于 0.5° |
| 步态轨迹对照 | 与 Python 参考误差小于 1 mm |
| 配置 | 标定参数写入 NVS，重启保持 |

---

## 9. 安全注意事项

1. 测试舵机时先架空狗腿或使用支架。
2. 不要让舵机长时间堵转。
3. 不要让 ESP32 的 3.3 V 给舵机供电。
4. 舵机电源和逻辑电源必须共地，但供电能力要分开评估。
5. 上电前用万用表确认电池、降压和舵机母线电压。
6. 未确认充电电路前，不把 Type-C 当锂电池充电口。
7. 大动作测试前先在低速度、低高度、无机械臂负载下进行。
8. 不在运动循环中执行大块 HTTP 页面发送。
9. 不在中断中做浮点数学、打印和阻塞 I2C。
10. 任何真实 Wi-Fi 密码、token、密钥都不要提交到公开仓库。
---

## 10. 当前仍有待确认的问题

- 用户看到的蓝色 `HW-674`/`RAD` 板具体是什么功能。
- 两块 PCA9685、蓝色板和电池之间的完整电源拓扑。
- Type-C 是否只负责 ESP32 下载/逻辑供电。
- ~~MPU6050 是否实际存在，为什么 I2C 扫描没有发现。~~
  ✅ **已结案（2026-09-19）：实物上没有 IMU**，全引脚双极性扫描确认。
  剩余决策：是否加装（当前选 F1 = A，暂不加装）。
- 机械臂实际是几自由度，源码中的“底座/大臂/小臂/夹爪”映射是否准确。
- 当前舵机型号和额定电压，3D 文件中出现 `SPT5435LV`，但实物需确认。
- 是否存在电压/电流采样电路。
- 当前开发板是否还能稳定进入下载模式。
- 用户手上的 ESP32-CAM 是否确认损坏，是否需要以后替换。

---

## 11. 新对话的第一个具体动作

下一次代码移植对话建议严格按以下顺序开始：

```text
1. 读取 HANDOFF.md
2. 读取 ESP-IDF_C迁移表.md
3. 读取 micropython/ 中的 main.py、padog.py、PA_SERVO.py、config_s.py
4. 检查 GitHub 工作区是否干净
5. 在 MechanicalDogUpgrade 下创建 firmware/
6. 建立 PlatformIO + ESP-IDF 空工程
7. 写出 app_main 和串口日志
8. 写出 I2C 扫描命令
9. 只测试 0x40、0x41，先不驱动狗腿
10. 记录实测输出后，再进入 PCA9685 单通道驱动
```

第一版 C 工程不要做：

- 不要先写复杂姿态融合。
- 不要先写 WebSocket。
- 不要先迁移机械臂。
- 不要先改机械结构。
- 不要在没有确认实物接线前改引脚。

---

## 12. 结论

当前项目已经完成：

- 原始 MicroPython 代码恢复和备份。
- 当前控制架构、问题点和硬件接口梳理。
- ESP-IDF C 迁移表。
- GitHub 公开备份仓库。
- 交接文档。

**2026-09-19 新增完成：**

- 硬件疑点实测结案：I2C 只有一条总线（22/21）挂两片 PCA9685；**本机无 IMU**。
- 板上 MicroPython 20 个文件全量备份（`_board_backup_20260919/`，不入公开仓库）。
- 参考答案：判读了灯哥菠萝狗官方电路图（结论：不能用于推断本机引脚）。
- **P0 工程落地**：`firmware/`（PlatformIO + ESP-IDF 5.1.2 + C + FreeRTOS），
  含串口日志、I2C 扫描、PCA9685 驱动、串口控制台。
- 诊断脚本：`diagnostics/`（三条总线扫描、全引脚扫描、硬复位）。

下一阶段要从“备份和规划”切换到“C 工程实现”。正确顺序是先底层、再控制、再交互、最后高级功能，避免把现有 MicroPython 的开环和不稳定问题直接复制到 C 工程中。

**紧接着要做的事（P0 收尾）：**

1. 烧录 `firmware/` 到板子，验证串口日志、I2C 扫描、`0x40/0x41` 回读
   （预期 `MODE1=0x21`、`PRESCALE=122`）。
2. 架空狗腿，用串口控制台逐个验证 12 个腿部通道的**物理位置与转动方向**
   （对应的实物核对项见《硬件实物核对清单.md》阶段 E，该文件放在桌面）。
3. 然后进入 P1：把 `config_s.py` / `PA_IK.py` / `PA_ATTITUDE.py` 迁到 C，
   用 Python 结果做离线 golden test。