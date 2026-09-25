/**
 * @file    driver/i2c.h
 * @brief   宿主测试用的 `driver/i2c.h` 桩
 *
 * 只是为了让 `bsp/bsp_i2c.h` 能被编译（它 include 了这个头）。
 * `bsp_i2c.c` **不参与宿主测试** —— 由 `host_sim.c` 提供一份假总线实现。
 */
#pragma once

#include <stdint.h>

typedef int i2c_port_t;
#define I2C_NUM_0 0
#define I2C_NUM_1 1
