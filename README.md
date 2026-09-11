# MechanicalDogUpgrade

本仓库用于备份“基于灯哥开源的四足机器狗”原始 MicroPython 程序，并持续整理 ESP-IDF C 版本升级资料。

## 目录说明

```text
.
├── HANDOFF.md                  # 当前状态、决策和下一阶段交接说明
├── ESP-IDF_C迁移表.md          # MicroPython 到 ESP-IDF C 的迁移计划
├── micropython/                # 原始机器狗 MicroPython 运行代码
│   ├── main.py
│   ├── padog.py
│   ├── PA_*.py
│   ├── mech_arm.py
│   ├── web_*.py
│   ├── *.html
│   ├── config.example.py       # Wi-Fi 脱敏模板
│   └── tools/                  # 原始烧录辅助脚本
└── esp32cam/                   # ESP32-CAM 程序，Wi-Fi 已脱敏
    └── esp32cam_v5.ino.example
```

## 安全说明

- 公开仓库中没有真实 Wi-Fi 账号和密码。
- 使用 MicroPython 代码前，将 `micropython/config.example.py` 复制为 `config.py`，再填入自己的 Wi-Fi 信息。
- 使用 ESP32-CAM 示例时，把 `YOUR_WIFI_SSID` 和 `YOUR_WIFI_PASSWORD` 替换为本机 2.4 GHz 网络信息。
- 不要提交真实 `config.py`、`secrets.h`、固件 `.bin`、测试图片和本地缓存。
- 原始完整程序仍保存在本机项目中，GitHub 上是经过脱敏和裁剪的备份。

## 当前已知状态

- 主控：ESP32。
- 舵机驱动：两片 PCA9685，地址 `0x40`、`0x41`。
- 主 I2C：`SDA GPIO21`、`SCL GPIO22`，100 kHz。
- MPU6050：代码中存在驱动，但当前启动扫描未发现 `0x68/0x69`。
- 当前控制路径以固定轨迹和逆运动学为主，实时闭环和 IMU 稳定控制尚未完成。

## C 版本目标

- VS Code + PlatformIO
- ESP-IDF 框架
- C 语言
- FreeRTOS
- 固定周期运动控制、命令队列、安全超时、IMU 闭环