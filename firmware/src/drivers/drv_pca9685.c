/**
 * @file    drv_pca9685.c
 * @brief   PCA9685 驱动实现
 */

#include "drivers/drv_pca9685.h"

#include "bsp/bsp_i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pca9685";

/* 最近一次设置的标称频率与周期。两块板共用同一频率（P0 阶段足够）。 */
static float    s_freq_hz   = DRV_PCA9685_DEFAULT_HZ;
static uint32_t s_period_us = 20000;

static uint8_t prescale_calc(float freq_hz)
{
    if (freq_hz < 24.0f) {
        freq_hz = 24.0f;
    }
    if (freq_hz > 1526.0f) {
        freq_hz = 1526.0f;
    }

    const int r = (int)(DRV_PCA9685_OSC_HZ / 4096.0f / freq_hz + 0.5f);
#if DRV_PCA9685_PRESCALE_LEGACY
    int p = r;          /* 复刻 MicroPython：50Hz -> 122 */
#else
    int p = r - 1;      /* 数据手册：50Hz -> 121 */
#endif
    if (p < 3) {
        p = 3;
    }
    if (p > 255) {
        p = 255;
    }
    return (uint8_t)p;
}

float drv_pca9685_actual_freq(uint8_t prescale)
{
    return DRV_PCA9685_OSC_HZ / 4096.0f / (float)(prescale + 1);
}

float drv_pca9685_get_freq(void)
{
    return s_freq_hz;
}

esp_err_t drv_pca9685_read_reg(uint8_t addr, uint8_t reg, uint8_t *val)
{
    return bsp_i2c_read_reg(addr, reg, val, 1);
}

esp_err_t drv_pca9685_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    return bsp_i2c_write_reg(addr, reg, &val, 1);
}

esp_err_t drv_pca9685_set_freq(uint8_t addr, float freq_hz)
{
    const uint8_t prescale = prescale_calc(freq_hz);

    /* MODE1: 进入 SLEEP 才能改 PRESCALE */
    uint8_t mode1 = 0;
    esp_err_t err = drv_pca9685_read_reg(addr, DRV_PCA9685_REG_MODE1, &mode1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X 读 MODE1 失败: %s", addr, esp_err_to_name(err));
        return err;
    }

    err = drv_pca9685_write_reg(addr, DRV_PCA9685_REG_MODE1,
                               (uint8_t)((mode1 & 0x7F) | DRV_PCA9685_MODE1_SLEEP));
    if (err != ESP_OK) {
        return err;
    }

    err = drv_pca9685_write_reg(addr, DRV_PCA9685_REG_PRESCALE, prescale);
    if (err != ESP_OK) {
        return err;
    }

    /* 退出 SLEEP，打开自动递增与 ALLCALL；RESTART 位自清 */
    err = drv_pca9685_write_reg(addr, DRV_PCA9685_REG_MODE1,
                               (uint8_t)(DRV_PCA9685_MODE1_ALLCALL |
                                         DRV_PCA9685_MODE1_AI |
                                         DRV_PCA9685_MODE1_RESTART));
    if (err != ESP_OK) {
        return err;
    }

    /* 数据手册要求退出 SLEEP 后等 500 µs 才可下发 PWM */
    vTaskDelay(pdMS_TO_TICKS(1));

    s_freq_hz   = freq_hz;
    s_period_us = (uint32_t)(1000000.0f / freq_hz);

    ESP_LOGI(TAG, "0x%02X 频率 %.2f Hz -> PRESCALE=%u (实约 %.2f Hz) 周期 %u us",
             addr, freq_hz, prescale, drv_pca9685_actual_freq(prescale),
             (unsigned)s_period_us);
    return ESP_OK;
}

esp_err_t drv_pca9685_init(uint8_t addr, float freq_hz)
{
    /* 先确认器件在总线上 */
    esp_err_t err = bsp_i2c_probe(addr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X 无应答，PCA9685 不存在或接线/供电异常", addr);
        return ESP_ERR_NOT_FOUND;
    }

    /* 复位到已知状态：写 MODE1 = 0x00 */
    err = drv_pca9685_write_reg(addr, DRV_PCA9685_REG_MODE1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02X 复位 MODE1 失败: %s", addr, esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    err = drv_pca9685_set_freq(addr, freq_hz);
    if (err != ESP_OK) {
        return err;
    }

    /* 上电即进入安全态：所有通道无脉冲，舵机松力，避免意外动作 */
    return drv_pca9685_all_off(addr);
}

esp_err_t drv_pca9685_set_pwm(uint8_t addr, uint8_t ch, uint16_t on, uint16_t off)
{
    if (ch >= DRV_PCA9685_CHANNELS) {
        return ESP_ERR_INVALID_ARG;
    }
    if (on > 4096 || off > 4096) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * 4096 表示「整周期全关」：**LEDn_OFF_H 的 bit4**（FULL OFF 标志）置位。
     *
     * ⚠️ 高字节必须 & 0x1F，**不能 & 0x0F** —— bit4 正是那个全开/全关标志。
     *    4096 >> 8 == 0x10，& 0x0F 会把它抹成 0，于是「无脉冲」变成了
     *    ON=0/OFF=0。这是 P2 做 golden 对照时发现的真 bug（成长手册 P-21）。
     *    0..4095 的正常占空比高字节最大是 0x0F，所以 & 0x1F 对它们没有影响。
     */
    const uint8_t data[4] = {
        (uint8_t)(on & 0xFF),
        (uint8_t)((on >> 8) & 0x1F),
        (uint8_t)(off & 0xFF),
        (uint8_t)((off >> 8) & 0x1F),
    };

    return bsp_i2c_write_reg(addr, (uint8_t)(DRV_PCA9685_REG_LED0_ON_L + 4 * ch),
                             data, sizeof(data));
}

esp_err_t drv_pca9685_set_channel_us(uint8_t addr, uint8_t ch, uint16_t us)
{
    if (us < DRV_PCA9685_US_MIN || us > DRV_PCA9685_US_MAX) {
        ESP_LOGW(TAG, "0x%02X ch%u 脉宽 %u us 超出安全范围 [%d, %d]，已拒绝",
                 addr, ch, us, DRV_PCA9685_US_MIN, DRV_PCA9685_US_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    /* 与 MicroPython 一致：用 4095 作为满量程（不是 4096） */
    const uint32_t off = (uint32_t)(4095.0f * (float)us / (float)s_period_us);
    return drv_pca9685_set_pwm(addr, ch, 0, (uint16_t)off);
}

esp_err_t drv_pca9685_set_all_us(uint8_t addr, uint16_t us)
{
    esp_err_t first_err = ESP_OK;
    for (uint8_t ch = 0; ch < DRV_PCA9685_CHANNELS; ++ch) {
        esp_err_t err = drv_pca9685_set_channel_us(addr, ch, us);
        if (err != ESP_OK && first_err == ESP_OK) {
            first_err = err;
        }
    }
    return first_err;
}

esp_err_t drv_pca9685_all_off(uint8_t addr)
{
    esp_err_t first_err = ESP_OK;
    for (uint8_t ch = 0; ch < DRV_PCA9685_CHANNELS; ++ch) {
        esp_err_t err = drv_pca9685_set_pwm(addr, ch, 0, 4096);
        if (err != ESP_OK && first_err == ESP_OK) {
            first_err = err;
        }
    }
    return first_err;
}

uint16_t drv_pca9685_us_from_degrees_ref(uint16_t degrees)
{
    /* 复刻 PA_SERVO.Servos.position()：
     *   period   = 1e6 / 50 = 20000
     *   min_duty = int(4095 * 500  / period) = 102
     *   max_duty = int(4095 * 2500 / period) = 511
     *   duty     = min_duty + span * degrees / 180   (整数截断)
     *   us       = duty * period / 4095
     */
    if (degrees > 180) {
        degrees = 180;
    }
    const uint32_t period   = 20000u;
    const uint32_t min_duty = (uint32_t)(4095u * 500u / period);
    const uint32_t max_duty = (uint32_t)(4095u * 2500u / period);
    const uint32_t span     = max_duty - min_duty;
    const uint32_t duty     = min_duty + (span * (uint32_t)degrees) / 180u;

    return (uint16_t)((duty * period) / 4095u);
}
