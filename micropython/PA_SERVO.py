#Copyright Deng（灯哥） (ream_d@yeah.net)  Py-apple dog project
#Github:https://github.com/ToanTech/py-apple-quadruped-robot
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at:http://www.apache.org/licenses/LICENSE-2.0
#函数说明：
#PA_SERVO.angle(逻辑通道0-11, 角度) 双板：0x40左半身、0x41右半身，见 angle() 内注释
#PA_SERVO.release()  #释放所有舵机到软状态（若需请自行扩展双板）
#移植说明：
#针对不同主控，重新按照 PA_SERVO.angle,PA_SERVO.release 函数形式封装舵机库，就可以通用运动控制/步态控制代码


from machine import I2C
from machine import Pin
import ustruct
import time

class PCA9685:
    def __init__(self, i2c, address=0x40):
        self.i2c = i2c
        self.address = address
        self.reset()

    def _write(self, address, value):
        self.i2c.writeto_mem(self.address, address, bytearray([value]))

    def _read(self, address):
        return self.i2c.readfrom_mem(self.address, address, 1)[0]

    def reset(self):
        self._write(0x00, 0x00) # Mode1

    def freq(self, freq=None):

        if freq is None:
            return int(25000000.0 / 4096 / (self._read(0xfe) - 0.5))
        prescale = int(25000000.0 / 4096.0 / freq + 0.5)
        old_mode = self._read(0x00) # Mode 1
        self._write(0x00, (old_mode & 0x7F) | 0x10) # Mode 1, sleep
        self._write(0xfe, prescale) # Prescale
        self._write(0x00, old_mode) # Mode 1
        time.sleep_us(5)
        self._write(0x00, old_mode | 0xa1) # Mode 1, autoincrement on

    def pwm(self, index, on=None, off=None):
        if on is None or off is None:
            data = self.i2c.readfrom_mem(self.address, 0x06 + 4 * index, 4)
            return ustruct.unpack('<HH', data)
        data = ustruct.pack('<HH', on, off)
        self.i2c.writeto_mem(self.address, 0x06 + 4 * index,  data)

    def duty(self, index, value=None, invert=False):
        if value is None:
            pwm = self.pwm(index)
            if pwm == (0, 4096):
                value = 0
            elif pwm == (4096, 0):
                value = 4095
            value = pwm[1]
            if invert:
                value = 4095 - value
            return value
        if not 0 <= value <= 4095:
            raise ValueError("Out of range")
        if invert:
            value = 4095 - value
        if value == 0:
            self.pwm(index, 0, 4096)
        elif value == 4095:
            self.pwm(index, 4096, 0)
        else:
            self.pwm(index, 0, value)




class Servos:
    def __init__(self, i2c, address=0x40, freq=50, min_us=500, max_us=2500,  #根据舵机参数自行设置
                 degrees=180):
        self.period = 1000000 / freq
        self.min_duty = self._us2duty(min_us)
        self.max_duty = self._us2duty(max_us)
        self.degrees = degrees
        self.freq = freq
        self.pca9685 = PCA9685(i2c, address)
        self.pca9685.freq(freq)

    def _us2duty(self, value):
        return int(4095 * value / self.period)

    def position(self, index, degrees=None, radians=None, us=None, duty=None):
        span = self.max_duty - self.min_duty
        if degrees is not None:
            duty = self.min_duty + span * degrees / self.degrees
        elif radians is not None:
            duty = self.min_duty + span * radians / math.radians(self.degrees)
        elif us is not None:
            duty = self._us2duty(us)
        elif duty is not None:
            pass
        else:
            return self.pca9685.duty(index)
        duty = min(self.max_duty, max(self.min_duty, int(duty)))
        self.pca9685.duty(index, duty)

    def release(self, index):
        self.pca9685.duty(index, 0)

    def position_duty(self, index, degrees=None, radians=None, us=None, duty=None):
        int_dutu=int(duty)
        self.pca9685.duty(index, int_dutu)

# 双路 PCA9685：0x40 左半身(左前0-2、左后3-5)，0x41 右半身(右前0-2、右后3-5)
# angle(pin_num,deg) 中 pin_num 0-11 为逻辑通道（四足）：
#   0-2 左前髋/大/小，3-5 左后髋/大/小，6-8 右前髋/大/小，9-11 右后髋/大/小
# 机械臂见 mech_arm.py：0x40 ch6 底座，0x41 ch6 夹爪（勿与四足 ch0-5 混接）
# 固定 I2C0，避免其它模块再 new I2C(22,21) 时重复绑定同一外设导致运行中报 ENODEV
try:
  _i2c_servo = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
except Exception:
  _i2c_servo = I2C(0, scl=Pin(19), sda=Pin(18), freq=100000)

try:
  _devs = _i2c_servo.scan()
  print("I2C0 scan:", [hex(x) for x in _devs])
except Exception as _e:
  print("I2C0 scan failed:", _e)

servos40 = Servos(_i2c_servo, address=0x40)
servos41 = None
try:
  servos41 = Servos(_i2c_servo, address=0x41)
except Exception as e:
  print("PCA9685 0x41 init skip:", e)

def angle(pin_num, degrees):
    pin_num = int(pin_num)
    if pin_num < 0 or pin_num > 11:
        raise ValueError("servo pin 0-11")
    if pin_num < 6:
        servos40.position(pin_num, degrees)
    elif servos41 is not None:
        servos41.position(pin_num - 6, degrees)




