# -*- coding: utf-8 -*-
"""
send_cmd.py —— 向 P0 串口控制台发送命令并抓回显

用法：
    python send_cmd.py COM4 "help" "status" "raw 0 0" "raw 0 FE"
    python send_cmd.py COM4 --delay 1.0 "scan"
"""
import sys
import time

import serial

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"
args = sys.argv[2:]
delay = 1.2
if args and args[0] == "--delay":
    delay = float(args[1])
    args = args[2:]

BAUD = 115200


def drain(s, seconds):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < seconds:
        n = s.in_waiting
        if n:
            buf += s.read(n)
        else:
            time.sleep(0.02)
    return buf.decode("utf-8", "replace")


def main():
    s = serial.Serial(PORT, BAUD, timeout=0.2)
    s.reset_input_buffer()
    print("已连接 %s @ %d" % (PORT, BAUD))
    print("先把开机遗留输出排空 1.5 秒 ...")
    drain(s, 1.5)

    for cmd in args:
        print("\n" + "=" * 60)
        print(">>> %s" % cmd)
        print("=" * 60)
        s.write((cmd + "\r\n").encode("utf-8"))
        s.flush()
        out = drain(s, delay)
        # 去掉 ANSI 颜色码，方便阅读
        import re
        out = re.sub(r"\x1b\[[0-9;]*m", "", out)
        print(out.rstrip())

    s.close()


main()
