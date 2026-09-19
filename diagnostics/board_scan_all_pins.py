# -*- coding: utf-8 -*-
# board_scan_all_pins.py (v5) —— 精简版全引脚扫描
#
# 依据实测：
#   - MicroPython 1.13 的 I2C 没有 deinit()，不要调用
#   - 单次 i2c.scan() 仅约 28 ms，全引脚扫描很快
#   - 只测浮空采样为 HIGH 的引脚（LOW 的引脚可能被外部拉低，会让 I2C 硬件死等）
#   - 每对先打印再扫描，若卡死可定位到具体引脚对

import time
from machine import I2C, Pin

PINS = [5, 14, 15, 18, 19, 21, 22, 32, 33]

HINT = {
    0x68: "IMU(MPU6050/ICM)", 0x69: "IMU(MPU6050 AD0=1)", 0x6A: "LSM6DS3",
    0x6B: "LSM6DSL", 0x76: "BMP280", 0x77: "BME280", 0x1E: "HMC/QMC5883",
    0x53: "ADXL345", 0x28: "BNO055", 0x29: "BNO055", 0x40: "PCA9685 A",
    0x41: "PCA9685 B", 0x70: "PCA9685 ALLCALL",
}


def mk(scl, sda):
    last = None
    for i in (1, 0):
        try:
            return I2C(i, scl=Pin(scl, Pin.PULL_UP), sda=Pin(sda, Pin.PULL_UP), freq=100000)
        except Exception as e:
            last = e
    raise last


hits = []
combos = [(PINS[i], PINS[j]) for i in range(len(PINS)) for j in range(len(PINS)) if i != j]

print("开始：%d 引脚 / %d 个有序组合" % (len(PINS), len(combos)))
print("")

n = 0
for (scl, sda) in combos:
    n += 1
    print("[%02d/%d] 试 SCL=%d SDA=%d ..." % (n, len(combos), scl, sda))
    try:
        i2c = mk(scl, sda)
        t0 = time.ticks_ms()
        sc = i2c.scan()
        dt = time.ticks_diff(time.ticks_ms(), t0)
        if sc:
            desc = ", ".join("0x%02X(%s)" % (a, HINT.get(a, "?")) for a in sc)
            print("      ★ 命中: %s   (%d ms)" % (desc, dt))
            hits.append((scl, sda, sc))
        else:
            print("      空 (%d ms)" % dt)
    except Exception as e:
        print("      创建/扫描失败: %r" % (e,))
    time.sleep_ms(5)

print("")
print("================ 汇总 ================")
if not hits:
    print("无任何命中")
for (scl, sda, sc) in hits:
    print("SCL=%-2d SDA=%-2d -> %s" % (scl, sda,
          ", ".join("0x%02X(%s)" % (a, HINT.get(a, "?")) for a in sc)))

imu = [(s, d, a) for s, d, sc in hits for a in sc
       if a in (0x68, 0x69, 0x6A, 0x6B, 0x76, 0x77, 0x1E, 0x53, 0x28, 0x29)]
print("")
print("IMU 类器件: %s" % (", ".join("SCL%d/SDA%d(0x%02X)" % t for t in imu) or "无"))
print("scan_all_pins done")
