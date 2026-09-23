# -*- coding: utf-8 -*-
"""
monitor_serial.py —— 只监听串口、**不复位、不发命令**，把日志写到文件

为什么需要它：验收项"站立 10 分钟不重启"要求板子**连续运行**，
不能像 capture_boot_log.py 那样先硬复位。本脚本只被动读，
所以可以在板子正在跑运动任务时观察它。

用法：
    python monitor_serial.py COM4 620 [输出文件]
    python monitor_serial.py COM4 620 out.txt --check-reboot

默认输出到 serial_monitor_<时间戳>.log。
加 --check-reboot 会在结束时报告是否出现过启动横幅（= 中途重启过）。

注意：Windows 控制台是 GBK，所以输出统一按 utf-8 replace 解码，
并额外写一份 UTF-8 文件（见成长手册 P-05）。
"""
import sys
import time
from datetime import datetime
from pathlib import Path

import serial

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

#: 判定"板子重启过"的关键字（main.c 的启动横幅）
BOOT_MARKERS = (
    "MechanicalDogUpgrade",
    "app_main",
    "ESP-IDF",
    "CPU 频率",
)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

    check_reboot = "--check-reboot" in sys.argv
    files = [a for a in sys.argv[3:] if not a.startswith("--")]
    out_path = Path(files[0]) if files else Path(
        "serial_monitor_%s.log" % datetime.now().strftime("%Y%m%d_%H%M%S"))

    print("=" * 60)
    print(" 被动监听 %s，%g 秒（不复位、不发命令）" % (port, seconds))
    print(" 输出: %s" % out_path.resolve())
    print("=" * 60)

    s = serial.Serial(port, 115200, timeout=0.2)
    # 刻意**不**动 DTR/RTS：那会复位板子
    t0 = time.time()
    total = 0
    boot_hits = 0
    last_report = t0

    with open(out_path, "w", encoding="utf-8", newline="\n") as fh:
        try:
            while time.time() - t0 < seconds:
                n = s.in_waiting
                if n:
                    raw = s.read(n)
                    total += len(raw)
                    text = raw.decode("utf-8", "replace")
                    fh.write(text)
                    fh.flush()
                    if check_reboot:
                        for m in BOOT_MARKERS:
                            if m in text:
                                boot_hits += 1
                                print("\n!!! 检测到启动标记 '%s' —— 板子中途重启过 !!!" % m)
                                break
                    sys.stdout.write(text)
                else:
                    time.sleep(0.05)

                now = time.time()
                if now - last_report >= 30.0:
                    last_report = now
                    print("\n--- 已监听 %.0f 秒，收到 %d 字节 ---" % (now - t0, total))
        except KeyboardInterrupt:
            print("\n用户中断。")
        finally:
            s.close()

    elapsed = time.time() - t0
    print("\n" + "=" * 60)
    print("监听结束: %.1f 秒, 共 %d 字节" % (elapsed, total))
    print("日志: %s" % out_path.resolve())
    if check_reboot:
        print("重启检查: %s" % ("发生过重启 ✘" if boot_hits else "无重启迹象 ✔"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
