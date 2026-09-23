/**
 * @file    servo_out.c
 * @brief   servo_map（纯 C）-> drv_pca9685（硬件）的粘合层，含占空比缓存
 */

#include "app/servo_out.h"

#include <string.h>

#include "bsp/bsp_i2c.h"
#include "drivers/drv_pca9685.h"
#include "esp_log.h"

static const char *TAG = "servo_out";

/*
 * 编译期保证两套地址常量没有漂移。
 * servo_map.h 为了能被宿主 gcc 编译，故意不 include 驱动头文件（那会拉进 esp_err.h），
 * 代价就是这两个地址各写了一份 —— 这个断言把代价变成了"编不过"，而不是"悄悄写错板子"。
 */
_Static_assert(SERVO_MAP_ADDR_LEFT == DRV_PCA9685_ADDR_LEFT,
               "servo_map.h 与 drv_pca9685.h 的 0x40 地址不一致");
_Static_assert(SERVO_MAP_ADDR_RIGHT == DRV_PCA9685_ADDR_RIGHT,
               "servo_map.h 与 drv_pca9685.h 的 0x41 地址不一致");

/** 每路缓存的上次写入值 + 是否已经有有效值 */
typedef struct {
    uint16_t on;
    uint16_t off;
    bool     valid;
} servo_cache_t;

static servo_cache_t s_cache[SERVO_MAP_CHANNELS];
static uint32_t      s_writes = 0;

/** 无脉冲状态：ON=0、OFF=4096（LEDn_OFF_H 的 FULL OFF 标志） */
static void no_pulse(uint16_t *on, uint16_t *off)
{
    *on  = 0;
    *off = 4096;
}

esp_err_t servo_out_init(void)
{
    memset(s_cache, 0, sizeof(s_cache));
    s_writes = 0;

    /* drv_pca9685_init() 已经把两片板所有通道置为无脉冲，所以缓存这样初始化是与硬件一致的 */
    for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
        no_pulse(&s_cache[ch].on, &s_cache[ch].off);
        s_cache[ch].valid = true;
    }

    ESP_LOGI(TAG, "舵机输出层就绪：%d 路，缓存已初始化为「无脉冲」（松力）",
             (int)SERVO_MAP_CHANNELS);
    return ESP_OK;
}

void servo_out_invalidate(void)
{
    for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
        s_cache[ch].valid = false;
    }
}

uint32_t servo_out_write_count(void)
{
    return s_writes;
}

void servo_out_reset_write_count(void)
{
    s_writes = 0;
}

/** 写一路并更新缓存。失败时不更新缓存，以便下一帧重试。 */
static esp_err_t write_one(uint8_t ch, uint16_t on, uint16_t off)
{
    const servo_map_hw_t *hw = servo_map_hw(ch);
    if (hw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_err_t err = drv_pca9685_set_pwm(hw->board_addr, hw->pca_ch, on, off);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X ch%u 写入失败(%s)，缓存保持不变，下一帧重试",
                 hw->board_addr, hw->pca_ch, esp_err_to_name(err));
        return err;
    }

    s_cache[ch].on    = on;
    s_cache[ch].off   = off;
    s_cache[ch].valid = true;
    ++s_writes;
    return ESP_OK;
}

esp_err_t servo_out_apply_deg(const float deg[SERVO_MAP_CHANNELS])
{
    if (deg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t first_err = ESP_OK;

    for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
        const uint16_t duty = servo_map_deg_to_duty(deg[ch]);
        uint16_t on = 0, off = 0;
        servo_map_duty_to_pwm(duty, &on, &off);

        /* 只写变化的通道 —— 见 servo_out.h 里关于 0.5 ms/通道 的说明 */
        if (s_cache[ch].valid && s_cache[ch].on == on && s_cache[ch].off == off) {
            continue;
        }

        const esp_err_t err = write_one(ch, on, off);
        if (err != ESP_OK && first_err == ESP_OK) {
            first_err = err;
        }
    }

    return first_err;
}

esp_err_t servo_out_all_off(void)
{
    esp_err_t first_err = ESP_OK;

    for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
        uint16_t on = 0, off = 0;
        no_pulse(&on, &off);
        if (s_cache[ch].valid && s_cache[ch].on == on && s_cache[ch].off == off) {
            continue;
        }
        const esp_err_t err = write_one(ch, on, off);
        if (err != ESP_OK && first_err == ESP_OK) {
            first_err = err;
        }
    }

    return first_err;
}

esp_err_t servo_out_readback(uint8_t logical_ch, uint16_t *on, uint16_t *off)
{
    if (on == NULL || off == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const servo_map_hw_t *hw = servo_map_hw(logical_ch);
    if (hw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 读 LEDn_ON_L 开始的 4 个字节 */
    uint8_t raw[4] = { 0 };
    const esp_err_t err = bsp_i2c_read_reg(hw->board_addr,
                                           (uint8_t)(DRV_PCA9685_REG_LED0_ON_L + 4 * hw->pca_ch),
                                           raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    /*
     * ⚠️ 高字节要 & 0x1F，不能 & 0x0F：bit4 是 FULL ON / FULL OFF 标志，
     *    OFF=4096 就是靠它表示的。见成长手册 P-21。
     */
    *on  = (uint16_t)(raw[0] | ((raw[1] & 0x1Fu) << 8));
    *off = (uint16_t)(raw[2] | ((raw[3] & 0x1Fu) << 8));
    return ESP_OK;
}
