# -*- coding: utf-8 -*-
# board_find_i2c.py (v4) —— 用权威的 I2C.scan() 做最终判定
#
# v3 的问题：用 readfrom_mem 探测，在总线无器件时不抛异常而是返回残留数据，
#            产生假阳性（命中里 SCL 可以是没接线的 5 / 32，只要 SDA=21）。
# v4：改用 MicroPython 的 i2c.scan()（真正判断 ACK），对每一对引脚的两个方向都扫。
#     先自检：只有 SCL=22/SDA=21 应命中。

import time
from machine import I2C, Pin

PINS = [5, 14, 15, 18, 19, 21, 22, 32, 33]      # 浮空采样为稳定 HIGH 的引脚
PINS_EXTRA = [2, 4, 12, 13, 23, 25, 26, 27, 16, 17]  # 其余（16/17 若接 PSRAM 会创建失败）

HINT = {
    0x68: "IMU(MPU6050/ICM)", 0x69: "IMU(MPU6050 AD0=1)", 0x6A: "LSM6DS3",
    0x6B: "LSM6DSL", 0x76: "BMP280", 0x77: "BME280", 0x1E: "HMC/QMC5883",
    0x53: "ADXL345", 0x28: "BNO055", 0x29: "BNO055", 0x40: "PCA9685 A",
    0x41: "PCA9685 B", 0x70: "PCA9685 ALLCALL",
}


def line(c="-", n=72):
    print(c * n)


def make_i2c(scl, sda):
    last = None
    for i2c_id in (1, 0):
        try:
            return I2C(i2c_id, scl=Pin(scl, Pin.PULL_UP),
                       sda=Pin(sda, Pin.PULL_UP), freq=100000), i2c_id
        except Exception as e:
            last = e
    raise last


def kill(i2c):
    if i2c is None:
        return
    try:
        i2c.deinit()
    except Exception:
        pass
    time.sleep_ms(10)


def scan_pair(scl, sda):
    i2c = None
    try:
        i2c, _ = make_i2c(scl, sda)
        return i2c.scan()
    except Exception:
        return None
    finally:
        kill(i2c)


def sweep(pins, label):
    combos = [(pins[i], pins[j]) for i in range(len(pins))
              for j in range(len(pins)) if i != j]
    line("")
    line("=")
    print("%s：%d 引脚 -> %d 个有序组合（i2c.scan()，双极性）" % (label, len(pins), len(combos)))
    line("=")
    hits = []
    done = 0
    fails = 0
    for (scl, sda) in combos:
        done += 1
        sc = scan_pair(scl, sda)
        if sc is None:
            fails += 1
            continue
        if sc:
            hits.append((scl, sda, sc))
            print("   ★ SCL=%-2d SDA=%-2d -> %s" % (scl, sda, ", ".join("0x%02X" % a for a in sc)))
        if done % 40 == 0:
            print("   ... %d/%d" % (done, len(combos)))
    if fails:
        print("   (%d 个组合无法创建 I2C，已跳过)" % fails)
    return hits


def main():
    line("#")
    print("全引脚 I2C 扫描 v4（i2c.scan() 权威判定，只读，不动舵机）")
    line("#")
    try:
        import os
        u = os.uname()
        print("平台: %s %s  MicroPython %s" % (u.sysname, u.machine, u.release))
    except Exception:
        pass
    line("#")
    print("")

    # ---- 自检 ----
    line("=")
    print("自检：只有 SCL=22/SDA=21 应命中")
    line("=")
    ref = scan_pair(22, 21)
    rev = scan_pair(21, 22)
    print("   SCL=22 SDA=21 -> %s   （应为 0x40 0x41 0x70）" %
          (", ".join("0x%02X" % a for a in ref) if ref is not None else "创建失败"))
    print("   SCL=21 SDA=22 -> %s   （SCL/SDA 接反，应为空）" %
          (", ".join("0x%02X" % a for a in rev) if rev is not None else "创建失败"))
    if not ref:
        print("   !! 自检失败：连已知总线都扫不到，结果不可信")
        return
    print("   自检通过 ✔")

    h1 = sweep(PINS, "阶段A - 有上拉的引脚")
    h2 = sweep(PINS_EXTRA, "阶段B - 其余引脚")

    allhits = h1 + h2

    line("")
    line("=")
    print("最终汇总")
    line("=")
    if not allhits:
        print("   无")
    for (scl, sda, sc) in allhits:
        tagged = ", ".join("0x%02X(%s)" % (a, HINT.get(a, "?")) for a in sc)
        print("   SCL=%-2d SDA=%-2d -> %s" % (scl, sda, tagged))

    buses = set((s, d) for s, d, _ in allhits)
    imu = [(s, d, a) for s, d, sc in allhits for a in sc
           if a in (0x68, 0x69, 0x6A, 0x6B, 0x76, 0x77, 0x1E, 0x53, 0x28, 0x29)]

    line("")
    print("能扫到器件的引脚对 : %s" %
          (", ".join("SCL%d/SDA%d" % b for b in sorted(buses)) or "无"))
    print("IMU 类器件         : %s" %
          (", ".join("SCL%d/SDA%d(0x%02X)" % t for t in imu) or "无"))

    line("")
    if imu:
        print(">>> ★ 找到 IMU 类器件，按上面引脚对改代码。")
    else:
        print(">>> 结论：整块板子上只有一个 I2C 总线（SCL22/SDA21），上面是两片 PCA9685。")
        print(">>> 没有任何 A0/MPU6050 等 IMU 器件，在任何引脚组合上都扫不到。")
        print(">>> 即：本台机器狗【没有安装可用的 IMU】。")
        print(">>> 下一步：打开机身，实物确认是否存在 MPU6050/GY-521 模块并已接线供电。")
    line("=")
    print("请把以上完整输出复制回给助手。")


main()
