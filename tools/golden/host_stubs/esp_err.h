/**
 * @file    esp_err.h
 * @brief   宿主测试用的 ESP-IDF `esp_err.h` 桩
 *
 * ⚠️ 这不是"模拟硬件"，只是让固件里那些**与硬件无关的层**（`app_chain` /
 * `motion` / `servo_out` / `drv_pca9685` 的逻辑）能在电脑上编译并运行。
 * 真正碰硬件的那一层由 `host_sim.c` 里的**假 I2C 总线 + PCA9685 影子寄存器**接住。
 *
 * 见 `tools/golden/README.md` 的"宿主桩层"一节：它能验**逻辑**，
 * **不能**验真实时序、并发、优先级、栈深度。
 */
#pragma once

#include <stdio.h>
#include <stdint.h>

typedef int esp_err_t;

#define ESP_OK                  0
#define ESP_FAIL                (-1)
#define ESP_ERR_NO_MEM          0x101
#define ESP_ERR_INVALID_ARG     0x102
#define ESP_ERR_INVALID_STATE   0x103
#define ESP_ERR_INVALID_SIZE    0x104
#define ESP_ERR_NOT_FOUND       0x105
#define ESP_ERR_NOT_SUPPORTED   0x106
#define ESP_ERR_TIMEOUT         0x107

static inline const char *esp_err_to_name(esp_err_t err)
{
    switch (err) {
    case ESP_OK:                return "ESP_OK";
    case ESP_FAIL:              return "ESP_FAIL";
    case ESP_ERR_NO_MEM:        return "ESP_ERR_NO_MEM";
    case ESP_ERR_INVALID_ARG:   return "ESP_ERR_INVALID_ARG";
    case ESP_ERR_INVALID_STATE: return "ESP_ERR_INVALID_STATE";
    case ESP_ERR_INVALID_SIZE:  return "ESP_ERR_INVALID_SIZE";
    case ESP_ERR_NOT_FOUND:     return "ESP_ERR_NOT_FOUND";
    case ESP_ERR_NOT_SUPPORTED: return "ESP_ERR_NOT_SUPPORTED";
    case ESP_ERR_TIMEOUT:       return "ESP_ERR_TIMEOUT";
    default:                    return "ESP_ERR_?";
    }
}
