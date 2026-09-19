# -*- coding: utf-8 -*-
# board_scan_i2c.py —— 机器狗 I2C 总线扫描与设备识别
#
# 目的：判定 GPIO21/22 与 GPIO18/19 各自通向哪个模块，并确认 MPU6050 是否存在。
#       起因见 硬件实物核对清单.md 的 §0.2.1：
#         参考电路图说 21/22 = MPU6050、18/19 = PCA9685，
#         而代码把 21/22 当舵机主总线，且从未扫过 18/19 上的 IMU。
#
# 安全性：
#   - 只做 I2C **读**操作。
#   - 唯一的写操作：当 WHO_AM_I(0x75) 返回 0x68（确认是 MPU6050）后，才写 0x6B 唤醒它。
#   - **不写 PCA9685 任何寄存器**，因此不会改变任何舵机位置。
#
# 运行方式（在电脑上）：
#   python -m mpremote connect COM4 run board_scan_i2c.py
#   或双击同目录的 扫描I2C.bat

import time
from machine import I2C, Pin

FREQ = 100000

BUSES = (
    ("A", 22, 21, "代码里 PA_SERVO 的主总线"),
    ("B", 19, 18, "参考电路图的 IMU 总线（本项目从未扫过）"),
    ("C", 32, 33, "代码里 PA_STABLIZE / PA_WALK 的 IMU 回退总线"),
)

# WHO_AM_I(0x75) 返回值对照
WHOAMI = {
    0x68: "MPU6050 / MPU6000",
    0x70: "MPU6500 / ICM series",
    0x71: "MPU9250",
    0x73: "MPU9255",
    0x19: "BMI160 (alt)",
    0xD1: "BMI160",
    0xEA: "QMI8658",
    0x6A: "LSM6DS3 / LSM6DSO",
    0x6C: "LSM6DSL",
}


def _line(ch="-", n=68):
    print(ch * n)


def release_used_buses():
    """main.py 里的 PA_SERVO 等可能已占用 I2C 外设，先尽量释放，避免 'I2C already in use'。"""
    print(">> 尝试释放 main.py 已占用的 I2C 外设 ...")
    found = False
    for name in ("PA_SERVO", "PA_STABLIZE", "PA_STABILIZE", "PA_WALK", "PA_IMU", "pair"):
        try:
            m = __import__(name)
        except Exception:
            continue
        for attr in dir(m):
            try:
                v = getattr(m, attr)
            except Exception:
                continue
            if isinstance(v, I2C):
                try:
                    v.deinit()
                    print("   [释放] %s.%s" % (name, attr))
                    found = True
                except Exception:
                    pass
    if not found:
        print("   （没有发现已占用的实例，正常）")


def open_bus(scl_pin_no, sda_pin_no, internal_pullup):
    """优先用 I2C(1)，避开 main.py 占用的 I2C(0)；失败再退到 I2C(0)。"""
    if internal_pullup:
        scl = Pin(scl_pin_no, Pin.PULL_UP)
        sda = Pin(sda_pin_no, Pin.PULL_UP)
    else:
        scl = Pin(scl_pin_no)
        sda = Pin(sda_pin_no)
    last = None
    for i2c_id in (1, 0):
        try:
            return I2C(i2c_id, scl=scl, sda=sda, freq=FREQ), i2c_id
        except Exception as e:
            last = e
    raise last


def rd(i2c, addr, reg, n=1):
    try:
        return i2c.readfrom_mem(addr, reg, n)
    except Exception:
        return None


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


def mpu6050_live_test(i2c, addr):
    """仅在 WHO_AM_I 确认为 0x68 后调用：唤醒并读一帧原始数据。"""
    try:
        i2c.writeto_mem(addr, 0x6B, b"\x00")   # PWR_MGMT_1: 解除 SLEEP
        time.sleep_ms(100)
        i2c.writeto_mem(addr, 0x1C, b"\x08")   # ACCEL_CONFIG: AFS_SEL=2 -> ±4g, 8192 LSB/g
        time.sleep_ms(20)
        i2c.writeto_mem(addr, 0x1B, b"\x08")   # GYRO_CONFIG: FS_SEL=1 -> ±500dps, 65.5 LSB/(deg/s)
        time.sleep_ms(20)
        raw = i2c.readfrom_mem(addr, 0x3B, 14)
    except Exception as e:
        print("       !! 读取失败: %r" % (e,))
        return False

    ax = s16(raw[0], raw[1])
    ay = s16(raw[2], raw[3])
    az = s16(raw[4], raw[5])
    tmp = s16(raw[6], raw[7]) / 340.0 + 36.53
    gx = s16(raw[8], raw[9])
    gy = s16(raw[10], raw[11])
    gz = s16(raw[12], raw[13])

    print("       原始14字节: %s" % " ".join("%02X" % b for b in raw))
    print("       AcX=%6d  AcY=%6d  AcZ=%6d   (±4g, 8192 LSB/g)" % (ax, ay, az))
    print("       GyX=%6d  GyY=%6d  GyZ=%6d   (±500dps, 65.5 LSB/dps)" % (gx, gy, gz))
    print("       Tmp=%.2f C" % tmp)
    ok = 7000 <= abs(az) <= 9200
    print("       静止判据: |AcZ| 应约 8192 (1g)。实测 %d -> %s" % (az, "正常" if ok else "异常"))
    return ok


def probe(i2c, addr):
    """对扫描到的地址做只读识别。"""
    hit = []

    if addr in (0x68, 0x69):
        b = rd(i2c, addr, 0x75)
        if b is not None:
            tag = WHOAMI.get(b[0], "未知型号")
            hit.append("WHO_AM_I(0x75) = 0x%02X  ->  %s" % (b[0], tag))
            if b[0] == 0x68:
                hit.append("*** 已确认是 MPU6050，做实时数据验证 ***")
                mpu6050_live_test(i2c, addr)
        else:
            hit.append("0x75 读不到（可能不是 IMU 类器件）")

    if addr in (0x40, 0x41, 0x70):
        m1 = rd(i2c, addr, 0x00)     # MODE1
        pre = rd(i2c, addr, 0xFE)    # PRESCALE
        if m1 is not None:
            hit.append("MODE1(0x00) = 0x%02X   (PCA9685 复位默认 0x11；0x01=已退出SLEEP)"
                       % m1[0])
        if pre is not None:
            f = 25000000.0 / 4096.0 / (pre[0] - 0.5) if pre[0] > 0 else 0
            hit.append("PRESCALE(0xFE) = %d  -> 约 %.1f Hz" % (pre[0], f))
        if addr == 0x70:
            hit.append("注意: 0x70 是 PCA9685 的 ALL-CALL 广播地址，不是 IMU")

    if not hit:
        # 通用：把 0x00~0x07 读出来看看
        b = rd(i2c, addr, 0x00, 8)
        if b is not None:
            hit.append("reg[0x00..0x07] = %s" % " ".join("%02X" % x for x in b))
        else:
            hit.append("读寄存器失败（地址能应答但读不出内容）")

    for h in hit:
        print("       %s" % h)
    return hit


def scan_one_bus(label, scl, sda, note, internal_pullup):
    mode = "内部上拉" if internal_pullup else "无上拉"
    tag = "%s  SCL=%d SDA=%d  [%s]" % (label, scl, sda, mode)
    _line()
    print(">> 总线 %s" % tag)
    print("   说明: %s" % note)

    try:
        i2c, i2c_id = open_bus(scl, sda, internal_pullup)
    except Exception as e:
        print("   !! 无法创建 I2C: %r" % (e,))
        return None

    print("   使用外设: I2C(%d)" % i2c_id)
    try:
        addrs = i2c.scan()
    except Exception as e:
        print("   !! scan() 失败: %r" % (e,))
        addrs = None

    if addrs is None:
        try:
            i2c.deinit()
        except Exception:
            pass
        return None

    if not addrs:
        print("   结果: 无任何器件应答")
        if not internal_pullup:
            print("   （下面会开内部上拉再试一次 —— 若无上拉时扫不到、开上拉能扫到，")
            print("     说明这条总线缺外部上拉电阻）")
    else:
        print("   结果: %s" % ", ".join("0x%02X" % a for a in addrs))
        for a in addrs:
            probe(i2c, a)

    try:
        i2c.deinit()
    except Exception:
        pass
    time.sleep_ms(50)
    return addrs


def main():
    _line("=")
    print("机器狗 I2C 总线扫描  (只读，不会让舵机动作)")
    _line("=")
    print("固件:", getattr(__import__("sys"), "implementation", None))
    try:
        import os
        s = os.uname()
        print("平台: %s %s  版本: %s" % (s.sysname, s.machine, s.release))
    except Exception:
        pass
    _line()

    release_used_buses()

    summary = {}
    for label, scl, sda, note in BUSES:
        # 先按原样扫；再开内部上拉扫（诊断上拉缺失）
        a1 = scan_one_bus(label, scl, sda, note, False)
        a2 = scan_one_bus(label, scl, sda, note, True)
        summary[label] = {"no_pullup": a1, "pullup": a2, "pins": (scl, sda)}

    # ---------- 汇总 ----------
    _line("=")
    print("汇总")
    _line("=")
    for label, scl, sda, note in BUSES:
        s = summary.get(label, {})
        def fmt(a):
            if a is None:
                return "创建/扫描失败"
            if not a:
                return "(空)"
            return " ".join("0x%02X" % x for x in a)
        print("总线 %s  SCL=%d SDA=%d" % (label, scl, sda))
        print("    无上拉: %s" % fmt(s.get("no_pullup")))
        print("    内部上拉: %s" % fmt(s.get("pullup")))

    print("")
    _line("=")
    print("判读指引")
    _line("=")

    def has(label, addr):
        s = summary.get(label, {})
        for k in ("no_pullup", "pullup"):
            a = s.get(k)
            if a and addr in a:
                return True
        return False

    pca_bus = None
    for label in ("A", "B", "C"):
        if has(label, 0x40) or has(label, 0x41):
            pca_bus = label
    imu_bus = None
    for label in ("A", "B", "C"):
        if has(label, 0x68) or has(label, 0x69):
            imu_bus = label

    print("PCA9685 所在总线 : %s" % (pca_bus if pca_bus else "未发现"))
    print("MPU6050 所在总线 : %s" % (imu_bus if imu_bus else "未发现"))

    if imu_bus and imu_bus != "A":
        print("")
        print(">>> 关键结论：IMU 不在代码当前使用的 21/22 上，而在总线 %s。" % imu_bus)
        print(">>> 说明 HANDOFF.md 里的『MPU6050 硬件阻塞项』可以解除，改成修引脚即可。")
    elif imu_bus == "A":
        print("")
        print(">>> IMU 与 PCA9685 同在 21/22，与参考电路图不符（两模块共总线）。")
    else:
        print("")
        print(">>> 三条总线都没有 IMU。若也无 0x40/0x41，先查供电与共地。")
        print(">>> 空结果常见原因：模块未供电 / 未共地 / 缺上拉 / SDA与SCL接反。")
    _line("=")
    print("请把以上完整输出复制回给助手。")


main()
