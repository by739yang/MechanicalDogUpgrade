# -*- coding: utf-8 -*-
"""esptool 风格的硬复位：直接拉 DTR/RTS（CH340 上通常 RTS->EN, DTR->IO0）。"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM4"

s = serial.Serial(PORT, 115200, timeout=0.2)
try:
    # 确保 IO0 为高（正常启动模式），然后拉低 EN 复位，再放开
    s.dtr = True          # 先随便设一下，确保句柄已生效
    s.rts = False
    time.sleep(0.05)

    s.dtr = False         # IO0 = HIGH -> 正常启动
    s.rts = True          # EN = LOW  -> 复位
    time.sleep(0.2)
    s.rts = False         # EN = HIGH -> 释放复位，开始启动
    time.sleep(0.2)

    # 读一段启动日志，确认板子活了
    t0 = time.time()
    buf = b""
    while time.time() - t0 < 4.0:
        n = s.in_waiting
        if n:
            buf += s.read(n)
        else:
            time.sleep(0.02)
    txt = buf.decode("utf-8", "ignore")
    print("---- 复位后串口输出 (%d 字节) ----" % len(buf))
    print(txt[:3000] if txt else "(无输出)")
finally:
    s.close()
print("---- reset done ----")
