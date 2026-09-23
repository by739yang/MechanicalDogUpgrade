# -*- coding: utf-8 -*-
"""
capture_boot_log.py —— 硬复位板子并抓取完整串口启动日志

用途：烧录后不想手动开监视器时，一条命令抓到自检输出。
      比 mpremote 可靠：直接操作 DTR/RTS，不依赖 raw REPL。

用法：
    python capture_boot_log.py [COM口] [秒数]
    python capture_boot_log.py COM4 12
"""
import codecs
import sys
import time
from pathlib import Path

import serial

# Windows 控制台默认按 ANSI(GBK) 编码输出，遇到无法编码的字符会直接抛
# UnicodeEncodeError 把脚本打断。强制 UTF-8 + replace。
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
BAUD = 115200


def main():
    print("打开 %s @ %d ..." % (PORT, BAUD))
    s = serial.Serial(PORT, BAUD, timeout=0.2)

    # 清掉缓冲区里可能残留的旧数据
    s.reset_input_buffer()

    # CH340 + ESP32 DevKit: DTR->IO0, RTS->EN
    # 正常启动模式：IO0 高，然后拉低 EN 复位，再放开
    s.dtr = False          # IO0 = HIGH
    s.rts = True           # EN  = LOW  -> 复位
    time.sleep(0.25)
    s.rts = False          # EN  = HIGH -> 放开，开始启动
    print("已触发硬复位，开始抓日志 %.0f 秒 ...\n" % SECONDS)

    buf = bytearray()
    dec = codecs.getincrementaldecoder("utf-8")("replace")
    t0 = time.time()
    while time.time() - t0 < SECONDS:
        n = s.in_waiting
        if n:
            chunk = s.read(n)
            buf += chunk
            # 实时回显：用增量解码器，避免多字节字符被切断
            sys.stdout.write(dec.decode(chunk))
            sys.stdout.flush()
        else:
            time.sleep(0.02)
    s.close()

    text = buf.decode("utf-8", "replace")
    print("\n\n================ 抓取结束 ================")
    print("共 %d 字节" % len(buf))

    # 用**绝对路径**（脚本自己所在目录），不要依赖"当前工作目录"。
    # 教训见成长手册 P-20：相对路径 + "我以为的当前目录"曾在仓库根目录
    # 留下一个多余文件，还被提交了进去。
    out = Path(__file__).resolve().parent / "boot_log.txt"
    with open(out, "w", encoding="utf-8") as f:
        f.write(text)
    print("已保存到 %s" % out)

    # 顺手做关键项判定，省得肉眼找
    print("\n---- 关键项自动判定 ----")
    present = [
        ("ESP-IDF 版本", "ESP-IDF  :"),
        ("芯片信息", "目标芯片 :"),
        ("Flash 大小", "Flash    :"),
        ("总线恢复完成", "总线已释放"),
        ("位操作扫描完成", "位操作扫描完成"),
        ("全扫描命中 3 个", "命中 3 个"),
        ("扫描结果 0x40/41/70", "扫描结果（3 个）：0x40, 0x41, 0x70"),
        ("0x40 驱动应答", "0x40  PCA9685 左半身"),
        ("0x41 驱动应答", "0x41  PCA9685 右半身"),
        ("0x70 all-call", "0x70  PCA9685 all-call"),
        ("MODE1=0x21 回读", "MODE1=0x21"),
        ("PRESCALE=122 与 Python 一致", "与 MicroPython 版实测一致"),
        ("控制台就绪", "控制台就绪"),
    ]
    absent = [
        ("不该出现'无应答'", "无应答"),
        ("不该出现崩溃", "Guru Meditation"),
        ("不该出现看门狗", "Task watchdog got triggered"),
    ]

    for name, needle in present:
        print("  %-28s %s" % (name, "✔ 找到" if needle in text else "✘ 没找到"))
    for name, needle in absent:
        print("  %-28s %s" % (name, "✔ 干净" if needle not in text else "✘ 出现了！"))


main()
