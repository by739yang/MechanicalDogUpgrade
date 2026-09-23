# ESP-IDF C 迁移表

> 版本：2026-09-11（正文）／2026-09-19（实测校订）
> 依据：`程序\cx\主要程序` 中当前实际运行的 MicroPython 代码，不按论文中的理想架构假设。
> 目标：VS Code + PlatformIO + ESP-IDF 框架 + C，先把现有行为完整迁移，再逐步替换成实时闭环架构。
>
> **2026-09-19 上机实测校订**：第 0 节第 2、4 条已结案；P0 工程已落地在 `firmware/`
> （PlatformIO + **ESP-IDF v5.1.2** + C）。⚠️ IDF 5.1.2 **没有** 新版 `driver/i2c_master.h`，
> 因此 `firmware/src/bsp/bsp_i2c.c` 使用 legacy `driver/i2c.h`；升级到 IDF ≥5.2 后再迁移新 API。

## 0. 先确认的事实

1. 当前主控制路径是开环固定轨迹 + 逆运动学 + PCA9685 舵机输出，不是论文里完整的 CPG/Kalman 闭环。
2. `PA_STABLIZE.py` 存在，但主循环中的调用被注释掉。
   **✅ 2026-09-19 实测结案：本机没有 IMU** —— `SCL22/SDA21` 上只有 `0x40/0x41/0x70`；
   `SCL19/SDA18`、`SCL32/SDA33` 全空；9 引脚 × 72 个有序组合（双极性）全引脚扫描也无 `0x68/0x69`。
   ⇒ 不是"扫不到"，是实物上没有。F1 决策：**暂不加装**，P0–P3 先行。
3. 当前 Web 控制是阻塞式 HTTP GET 轮询，控制命令和网页服务都在 MicroPython 运行时中竞争资源。
4. **✅ 2026-09-19 实测确认：代码引脚是对的。** 唯一 I2C 总线就是 **GPIO21/GPIO22**，
   两片 PCA9685（`0x40`/`0x41`）都挂在这条总线上；实测 `MODE1=0x21`、`PRESCALE=122`（≈50.2 Hz），
   与 `PA_SERVO.py` 的 `freq=50` 完全一致。
   `GPIO18/19` 与 `GPIO32/33` 上**没有任何器件** —— 代码里的 fallback 分支从未生效、也无需生效。
   注意：代码用 `I2C(0, scl=Pin(22), sda=Pin(21))` 作主总线且构造不抛异常，
   所以 `except` 回退分支实际上永远不会执行。
5. `config_s.py` 运行时会被 Web 标定流程重写。C 版本不能继续改写源码，应改为 NVS 或 LittleFS 配置。
6. Type-C 在现有证据下仍应视为 ESP32 下载/调试/逻辑供电相关接口，不能在没有确认充电电路前当作锂电池充电口。
7. **2026-09-19 新发现**：`padog.do_connect_STA()` 是 `while not wifi.isconnected(): pass`，
   **无超时的死循环** —— 热点不在就卡死在启动阶段，连 REPL 都进不去。
   C 版（`comm/wifi.c`）必须有连接超时 + 失败降级（AP 模式或停车待命）。
8. **2026-09-19 新发现**：MicroPython 版的 I2C 探测不可靠 ——
   `i2c.readfrom_mem()` 在总线无器件时可能不抛异常而返回残留数据（出现"SCL 根本没接线却读成功"的假阳性）。
   C 版一律用 `esp_err_t` 判断，不受此影响。

## 0.1 P0 落地状态（2026-09-19）

| 项 | 状态 |
|---|---|
| PlatformIO 工程 | ✅ `firmware/` |
| 平台 / 框架 | ✅ `espressif32@6.5.0` + `espidf`（ESP-IDF v5.1.2） |
| 串口日志 | ✅ `main.c` 打印构建时间 / IDF 版本 / 芯片 / Flash / 空闲堆 / 硬件基线 |
| I2C 扫描 | ✅ `bsp/bsp_i2c.c` + `app_p0.c` 自检，核对 `0x40`/`0x41` |
| PCA9685 驱动 | ✅ `drivers/drv_pca9685.c`（含 `MODE1`/`PRESCALE` 回读校验） |
| 单通道控制 | ✅ 串口控制台：`set` / `deg` / `all` / `off` / `sweep` / `raw` / `status` / `scan` |
| 上电安全 | ✅ 上电只初始化并置"无脉冲/松力"，**不自动驱动舵机** |
| 脉宽限幅 | ✅ 强制 `[500, 2500] µs` |

## 0.2 P1 落地状态（2026-09-19）

| 项 | 状态 |
|---|---|
| golden 对照测试台 | ✅ `tools/golden/`：从原 MicroPython 直接生成参考值 → mingw gcc 编译 C 版 → 逐数值比对（**不用板子**） |
| `PA_IK.py` → `kinematics.c` | ✅ 592 项，最大误差 6.0e-5°（容差 0.5°，余量 8290×） |
| `PA_ATTITUDE.py` → `body_pose.c` | ✅ 696 项，最大误差 2.2e-5 mm（容差 0.5 mm） |
| `PA_TROT.py` → `gait_trot.c` | ✅ 8320 项，最大误差 1.9e-5 mm（容差 1 mm） |
| `PA_WALK.py` → `gait_walk.c` | ✅ 7872 项足端（最大 6.7e-5 mm）+ 2952 项重心整数（**精确相等**） |
| `PA_AVGFILT.py` → `filter_moving_avg.c` | ✅ 264 项整数**精确相等** |
| `config.py`/`config_s.py` → `app_config.c` | ✅ 68 条行为检查（默认值/限幅/CRC/版本/环回/空存储/擦除） |
| NVS 持久化 | ✅ **真机验证**：`cfg set` → `cfg save` → 硬复位 → 值仍在；`cfg reset` 确实擦除 |
| 配置越界限幅 | ✅ 真机验证（`h_goal 9999` → 250；`ma_case 5` → 1；`arm_upper_board 0x7F` → 0x40） |
| 合计对照项 | **20 696** 项，全部通过 |

> **P1 的验收为什么可信**：参考值不是我重写一遍，而是**直接 `exec` 原 MicroPython 模块**
> 产生的；C 版与它在同一批输入上逐项比数值。发现的三处原实现问题（无效计算、
> 定义域不检查、无 else 分支）都记录在 `问题与解决记录.md` 的 P-17~P-19。

## 1. 当前 MicroPython 控制链

```text
main.py
  ├── web_thread()
  │     └── web_c.py 或 web_ctl.py
  │           └── HTTP GET 命令
  │                 ├── control.html / drive.html
  │                 └── web_common.py 解析
  └── control_loop()
        └── padog.mainloop()
              ├── 读取 spd / L / R / gait_mode
              ├── PA_TROT.py 或 PA_WALK.py
              ├── PA_ATTITUDE.py
              ├── PA_IK.py
              ├── servo_output()
              │     └── PA_SERVO.py
              │           └── PCA9685 0x40 / 0x41
              └── mech_arm.tick()
```

## 2. 模块迁移总表

| 现有文件 | 当前职责 | C 模块/文件 | ESP-IDF 任务 | 驱动/接口 | 引脚、总线、地址 | 迁移验收标准 |
|---|---|---|---|---|---|---|
| `main.py` | 创建 Web 线程并循环调用 `padog.mainloop()` | `main.c`、`app_init.c`、`task_motion.c`、`task_comm.c` | `app_init`、`motion_task`、`comm_task` | FreeRTOS、`esp_timer`、NVS | 无直接 GPIO | 上电完成自检；I2C 扫描成功；10 分钟不重启 |
| `boot.py` | 空文件 | 不需要独立模块 | Boot/初始化阶段 | NVS、分区挂载 | 无 | 不迁移，作为历史文件保留 |
| `config.py` | Wi-Fi 和基础步态参数 | `app_config.c`、`defaults.h` | `app_init` 读取 | NVS、Wi-Fi | Wi-Fi STA/AP；无 GPIO | 配置可保存、重启保持、非法值被限幅 |
| `config_s.py` | 舵机中位、几何尺寸、步态和机械臂参数 | `nvs_config.c`、`app_config.h` | `app_init` / `config_task` | NVS 或 LittleFS | 无直接 GPIO | 标定参数写入 NVS，重新上电后保持 |
| `padog.py` | 机器人状态、步态调度、姿态目标、舵机映射 | `robot_state.c`、`robot.c`、`task_motion.c`、`servo_map.c` | `motion_task`，建议 100~200 Hz | FreeRTOS、I2C、`esp_timer` | 使用 I2C0；SDA 21、SCL 22 | 站立、停止、前进、后退行为与 Python 版一致；控制循环固定周期 |
| `PA_SERVO.py` | PCA9685 驱动和逻辑通道映射 | `drv_pca9685.c`、`servo_map.c` | `motion_task` 初始化、输出阶段 | ESP-IDF I2C master | I2C0：SDA 21、SCL 22、100 kHz；0x40、0x41；备用 18/19 | 扫描能看到 0x40、0x41；12 个逻辑通道均可单独控制 |
| `PA_IMU.py` | MPU6050 原始数据读取和姿态计算 | `drv_mpu6050.c`、`attitude.c` | `imu_task`，建议 200 Hz | ESP-IDF I2C master | 同一 I2C 总线；MPU6050 假设地址 0x68/0x69；备用 32/33 | 能稳定读取加速度、陀螺仪、温度；无超时；姿态角连续 |
| `PA_AVGFILT.py` | 滑动平均滤波 | `filter_moving_avg.c` | `imu_task` 或 `motion_task` | 无外设 | 无 | 与 Python 参考输出误差不超过 1 个计数单位 |
| `PA_ATTITUDE.py` | 把俯仰、滚转、X 偏移转换到四腿足端目标 | `body_pose.c` | `motion_task` | 纯数学 | 无 | 相同输入下 8 个输出与 Python 版误差不超过 0.5 mm |
| `PA_IK.py` | 串联/并联腿逆运动学 | `kinematics.c` | `motion_task` | 纯数学 | 无 | 相同输入下关节角误差不超过 0.5°；异常输入不产生 NaN |
| `PA_TROT.py` | TROT 相位和小跑步态轨迹 | `gait_trot.c` | `motion_task` | 纯数学 | 无 | 相同 `t/xs/xf/h` 下轨迹一致；相位不倒退 |
| `PA_WALK.py` | WALK 顺序步态和可选 IMU 重心调整 | `gait_walk.c`、`gait_scheduler.c` | `motion_task` | 纯数学；IMU 可选 | 使用 IMU 时走同一 I2C 总线 | 四腿相序、摆动/支撑相位正确；低速不摔 |
| `PA_STABLIZE.py` | 静态姿态稳定控制，当前未接入主循环 | `balance_controller.c` | `motion_task` | MPU6050 | I2C0 21/22；备用 32/33 | 仅在 IMU 验证通过后启用；倾斜恢复不发散、不振荡 |
| `mech_arm.py` | 机械臂、大小臂、夹爪控制和缓动 | `arm_control.c`、`drv_pca9685.c` | `arm_task`，建议 50~100 Hz | PCA9685；可选 GPIO PWM | 0x40 ch6/ch7；0x41 ch6；可选 GPIO12、50 Hz | 角度限幅、平滑逼近、夹爪开合和急停正确 |
| `web_c.py` | 完整 HTTP 页面、按钮、标定、参数保存 | `http_server.c`、`calibration.c`、`shared_config.c` | `comm_task` | `esp_http_server` 或 lwIP、NVS | Wi-Fi；TCP 80 | 页面可访问；标定保存；不阻塞 `motion_task` |
| `web_common.py` | 摇杆、按钮、普通 HTTP 参数解析 | `command_parser.c`、`command_queue.c` | `comm_task` | 无外设 | 无 | 命令有序列号和时间戳；旧命令不会覆盖新命令 |
| `web_ctl.py` | 轻量 HTTP 服务 | `http_server.c` 的轻量路由 | `comm_task` | `esp_http_server` 或 lwIP | Wi-Fi；TCP 80 | 控制命令单次响应、不加载大页面；断连后停车 |
| `control.html` | 完整控制页，120 ms HTTP 轮询 | LittleFS 资源或压缩内嵌资源 | 由 `comm_task` 提供 | HTTP/WebSocket | 无 | 手机访问正常；控制流与页面加载分离 |
| `drive.html` | 轻量遥控页，狗 80 ms、臂 35 ms 轮询 | LittleFS 资源或压缩内嵌资源 | 由 `comm_task` 提供 | HTTP/WebSocket | 无 | 20~50 Hz 有效控制；无过期命令执行 |
| `cal.html` | 标定页；`web_c.py` 动态生成后段参数表 | `calibration.c` + Web 资源 | `comm_task` | HTTP、NVS | 无 | 标定值保存并可恢复；越界自动拒绝 |
| `_served_control.html` | 已生成/重复页面 | 不迁移 | 无 | 无 | 无 | 视为构建产物，不进入 C 工程 |
| `flash_upload.py` | 主机侧 MicroPython 刷写和上传 | `tools/` 下的 esptool 脚本 | 主机进程 | esptool、mpremote、串口 | COM 口、460800 | 仅用于 MicroPython 回滚，不进入 ESP32 固件 |
| `一键上传.bat` | MicroPython 上传入口 | `tools/` 或手动命令 | 主机 | esptool、mpremote | COM 口 | 由 `pio run -t upload` 或 `idf.py flash` 替代 |
| `flash_config.json` | MicroPython 烧录参数 | 不迁移为运行时模块 | 主机 | esptool | COM 口、0x1000 | 仅作为历史记录 |
| `micropython.bin` | MicroPython 固件 | 不迁移 | 无 | esptool | Flash | 保留用于回滚 |

## 3. ESP-IDF 任务建议

> 优先级数字只是起点，最终要结合实测调整。第一阶段可以让 `motion_task` 同时读取 IMU，避免一开始就引入多任务抢 I2C。

| 任务 | 建议核心 | 建议频率 | 主要职责 | 必须避免 |
|---|---:|---:|---|---|
| `app_init` | Core 0 | 启动一次 | NVS、I2C、Wi-Fi、PCA9685、IMU、任务创建 | 不在初始化中阻塞太久 |
| `motion_task` | Core 1 | 100~200 Hz | 步态、姿态目标、IK、舵机输出 | 不直接处理 HTTP，不做长时间等待 |
| `imu_task` | Core 1 | 200~500 Hz；初期可 200 Hz | 读取 MPU6050、滤波、姿态估计 | 不和 `motion_task` 并发霸占 I2C |
| `comm_task` | Core 0 | 事件驱动，命令 20~50 Hz | Wi-Fi、HTTP、WebSocket、命令入队 | 不直接写舵机 |
| `arm_task` | Core 1 或 Core 0 | 50~100 Hz | 机械臂缓动、夹爪控制 | 不遮挡安全控制 |
| `safety_task` | Core 1 | 50~100 Hz | 心跳、超时、电压/电流、急停、看门狗 | 不能被网络任务低优先级拖住 |
| `telemetry_task` | Core 0 | 10~20 Hz | 姿态、状态、电压、错误码上报 | 不发送大块 HTML |

### 通信队列原则

- `comm_task` 只解析命令并写入队列。
- `motion_task` 只取最新有效命令，不执行已经过期的命令。
- 命令中必须包含 `seq`、时间戳、模式、速度、转向、姿态、身高、机械臂和急停字段。
- 断连或心跳超时后，`motion_task` 应自动进入停止或安全姿态。

## 4. 引脚、总线和地址表

| 功能 | 当前代码位置 | 当前引脚/地址 | ESP-IDF 对应 | 备注 |
|---|---|---|---|---|
| 状态 LED | `padog.py` | GPIO2 | `GPIO_NUM_2` | 代码中 `0` 表示上电亮，`1` 表示网络就绪灭；实物极性要确认 |
| 主 I2C 总线 | `PA_SERVO.py` | SDA GPIO21，SCL GPIO22，100 kHz | `I2C_NUM_0` | 当前舵机和 IMU 共用这条总线 |
| 主 I2C 备用引脚 | `PA_SERVO.py` | SDA GPIO18，SCL GPIO19 | `I2C_NUM_0` 重新映射 | 仅初始化失败时使用，需实物确认 |
| IMU 备用 I2C | `PA_WALK.py`、`PA_STABLIZE.py` | SDA GPIO33，SCL GPIO32 | `I2C_NUM_1` | 当前代码的备用路径，未证明已接线 |
| PCA9685 左侧板 | `PA_SERVO.py` | 0x40 | I2C 从设备 | 逻辑通道 0~5；机械臂大臂/小臂当前也在这块板 |
| PCA9685 右侧板 | `PA_SERVO.py` | 0x41 | I2C 从设备 | 逻辑通道 6~11；夹爪当前在 ch6 |
| MPU6050 | `PA_IMU.py` | 假设 0x68 | I2C 从设备 | **实测本机无此器件**（全引脚双极性扫描确认）；将来加装时挂到 21/22 即可 |
| 夹爪备用 GPIO PWM | `mech_arm.py` | GPIO12，50 Hz | LEDC 或 MCPWM | 当前 `arm_grip_gpio=-1`，默认走 PCA9685 0x41 ch6 |
| USB 串口调试 | ESP32 DevKit 默认 | UART0，115200 | `uart_config_t` | 用于日志和烧录；当前 MicroPython 主程序未主动配置 UART 应用协议 |
| 电压/电流检测 | 当前代码没有 | 未定义 | ADC 或 INA226 | 迁移时新增，不能凭空指定引脚，必须按实物电路确定 |

### PCA9685 逻辑通道映射

当前 `PA_SERVO.angle(pin_num, degrees)` 的逻辑通道如下：

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

机械臂当前占用：

| 机构 | 板卡 | 通道 | 当前配置 |
|---|---|---:|---|
| 大臂 | 0x40 | ch6 | `arm_upper_ch=6` |
| 小臂 | 0x40 | ch7 | `arm_fore_ch=7` |
| 夹爪 | 0x41 | ch6 | `arm_grip_gpio=-1`，走 PCA9685 |

> 注意：源码注释曾把 0x40 ch6 称为“底座”，但当前运行配置把它作为大臂通道，而且没有独立底座通道。迁移前必须逐通道确认实物机械臂映射。

## 5. 建议的 C 工程目录

```text
firmware/
├── platformio.ini
├── src/
│   ├── main.c
│   ├── app/
│   │   ├── app_init.c
│   │   ├── app_config.c
│   │   ├── robot_state.c
│   │   ├── command_queue.c
│   │   └── safety.c
│   ├── bsp/
│   │   ├── bsp_i2c.c
│   │   ├── bsp_gpio.c
│   │   └── bsp_time.c
│   ├── drivers/
│   │   ├── drv_pca9685.c
│   │   ├── drv_mpu6050.c
│   │   └── drv_ina226.c
│   ├── control/
│   │   ├── kinematics.c
│   │   ├── gait_trot.c
│   │   ├── gait_walk.c
│   │   ├── body_pose.c
│   │   ├── attitude.c
│   │   └── balance_controller.c
│   ├── arm/
│   │   └── arm_control.c
│   └── comm/
│       ├── wifi.c
│       ├── http_server.c
│       ├── websocket.c
│       ├── command_parser.c
│       └── telemetry.c
└── web/
    ├── control.html
    ├── drive.html
    └── cal.html
```

## 6. 迁移顺序与阶段验收

| 阶段 | 迁移内容 | 必须先完成 | 阶段验收标准 |
|---|---|---|---|
| P0 | 工程、日志、I2C、PCA9685 | PlatformIO ESP-IDF 工程可编译烧录 | I2C 扫描看到 0x40、0x41；每个舵机可单独安全动作 —— ✅ 已验收（除"每个舵机单独动作"移入 P2，需电池） |
| P1 | 配置和逆运动学 | NVS、`kinematics.c` | Python/C 离线对照误差在容差内；配置重启保持 —— ✅ **已验收**（见 §0.2） |
| P2 | 运动循环和 12 路舵机输出 | `motion_task`、`servo_map.c` | 固定周期运行；站立 10 分钟不重启；急停有效 |
| P3 | TROT/WALK 和姿态数学 | `gait_trot.c`、`gait_walk.c`、`body_pose.c` | 原地踏步 60 秒不摔；低速直行 3 米可控 |
| P4 | IMU 和稳定控制 | MPU6050 实物和 `attitude.c` | 静态角度稳定；倾斜时姿态估计正确；闭环不发散 |
| P5 | Web 通信重构 | HTTP 服务和命令队列 | 20~50 Hz 控制；旧命令丢弃；断连自动停车 |
| P6 | 机械臂、遥测、电源保护 | `arm_control.c`、电流/电压检测 | 机械臂限幅平滑；电压跌落可记录并触发保护 |

## 7. 关键验收指标

| 项目 | 目标值 | 说明 |
|---|---:|---|
| 运动循环频率 | 100~200 Hz | 先以 100 Hz 稳定，再提高到 200 Hz |
| 循环抖动 | 平均值小于 2 ms | 用 `esp_timer_get_time()` 记录周期 |
| 通信命令频率 | 20~50 Hz | 不再使用 35~120 ms 的无序号 GET 轮询 |
| 心跳超时 | 200~300 ms | 超时后停车或进入安全姿态 |
| 急停响应 | 小于 100 ms | 安全任务独立于 Web 任务 |
| 站立测试 | 10 分钟不重启 | 记录电流、电压、错误码 |
| 原地踏步 | 60 秒不摔 | 初期用支架或低高度 |
| 低速直行 | 3 米不失控 | 先低速、平地、无机械臂负载 |
| IK 对照 | 角度误差小于 0.5° | 与 MicroPython 参考实现做离线对比 |
| 步态轨迹对照 | 误差小于 1 mm | 对同一时间点的四足目标进行对比 |
| I2C 扫描 | 0x40、0x41 必现 | MPU6050 需额外看到 0x68 或 0x69 |

## 8. 迁移时必须修正的旧设计

1. 把 `web_c.py` 中的网页服务从运动主循环中移出。
2. 把字符串 HTTP 参数解析改成有长度、类型、范围检查的二进制或结构化命令。
3. 不执行 `exec()` 解析用户参数；C 版本必须使用白名单字段和范围校验。
4. 不把标定结果写回 `config_s.py`；改为 NVS 或 LittleFS。
5. 不让多个任务无锁访问同一个 I2C 总线。
6. 不在控制任务里加载/发送大块 HTML。
7. `stable()` 不能只是一个无效的布尔开关；先完成 IMU 和姿态闭环，再启用。
8. 没有足端接触和电流反馈前，不要宣称已经实现真正的自适应稳定控制。
9. **`PA_IK.py` 不做定义域检查**：`acos`/`asin` 的参数越界会抛 `ValueError`
   （CPython）或产生 `NaN`（MicroPython）。C 版必须 clamp 到 `[-1, 1]`，
   并对除零做保护 —— 验收标准要求"异常输入不产生 NaN"。
   已在 `firmware/src/control/kinematics.c` 落实。
10. **`PA_ATTITUDE.cal_ges()` 内部有一段无效计算**：它算了 `AB1_y..AB4_y` 四个横向坐标，
    但返回值只有 `AB*_x` 与 `AB*_z`，那四个值从未被使用。
    连带后果：**形参 `w`（左右腿间距，config_s.py 里 220）只被这四个式子使用，
    因此 `w` 对 `cal_ges()` 的输出毫无影响** —— 改它不会产生任何效果。
    C 版（`body_pose.c`）删除了这段无效计算；删除后 golden 误差一位不差，实证其无效。
    `w` 形参保留以维持签名一致，并注明是"死参数"。

## 9. 迁移前必须实物确认的未知项

- ✅ ESP32 板子的准确型号和 GPIO 丝印：**HW-394（≈ESP-WROOM-32），rev 3.1，4 MB Flash**；
  实测唯一 I2C 在 **SDA=GPIO21 / SCL=GPIO22**，与代码一致。
- ✅ 两个 PCA9685 的真实板号：**0x40 / 0x41**（0x70 是 all-call 广播）；
  **逻辑电走 USB 5V**（不插电池也能应答，已实测）；舵机 V+ 走电池经两片 HW-674 降压。
- ~~MPU6050 是否实际安装、供电和维护地址，当前扫描未发现。~~
  ✅ **已结案（2026-09-19）：实物上没有 IMU。** 剩余为决策项：是否加装（当前暂不加装）。
- ⬜ 机械臂三个舵机的真实通道，特别是源码注释和当前配置不一致的部分（P6）。
- ⬜ **12 路腿部舵机的真实通道 → 关节映射与转动方向** —— 需要舵机带电，
  **归入 P2 第一步**（P0 的 `set`/`sweep` 控制台命令已可用于逐个试动）。
- ✅ Type-C 只连接 ESP32 与 PCA9685 逻辑侧，不参与动力供电。
- ⬜ 电池、BMS、降压、舵机母线的实际拓扑：已目视（11.1 V 3500 mAh + 两片 HW-674 8A），
  **待测**：两片 HW-674 的实际输出电压。
- ⬜ 是否已有电压/电流采样电路；如果没有，迁移计划中应作为新增硬件处理。

---

## 结论

第一版 C 迁移不要一次重写所有东西。推荐顺序是：

```text
PlatformIO + ESP-IDF 工程
-> I2C / PCA9685 驱动
-> NVS 配置
-> IK / TROT / WALK 纯数学模块
-> motion_task 和 12 路舵机
-> IMU / 姿态闭环
-> HTTP/WebSocket 通信
-> 机械臂 / 遥测 / 电源保护
```

这样可以在每一步都拿现有 MicroPython 行为做对照，避免把原问题带到 C 版本里。