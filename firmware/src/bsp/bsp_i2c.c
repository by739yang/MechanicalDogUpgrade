/**
 * @file    bsp_i2c.c
 * @brief   I2C 主机总线实现（legacy driver/i2c.h，ESP-IDF 5.1.2）
 */

#include "bsp/bsp_i2c.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "bsp_i2c";

static bool s_installed = false;

/**
 * @brief 总线恢复：从设备若把 SDA 拉低不放，用 9 个 SCL 脉冲把它顶出去。
 *
 * 必须在 I2C 驱动安装之前调用（要直接控制 GPIO）。
 */
esp_err_t bsp_i2c_bus_recover(void)
{
    if (s_installed) {
        ESP_LOGW(TAG, "总线恢复需在 I2C 安装前调用，已跳过");
        return ESP_ERR_INVALID_STATE;
    }

    /* 开漏 + 上拉：既能读又能拉低 */
    gpio_set_direction(BSP_I2C_SCL_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(BSP_I2C_SDA_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(BSP_I2C_SCL_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BSP_I2C_SDA_GPIO, GPIO_PULLUP_ONLY);

    /* 先确保两根线都释放（高） */
    gpio_set_level(BSP_I2C_SCL_GPIO, 1);
    gpio_set_level(BSP_I2C_SDA_GPIO, 1);
    esp_rom_delay_us(10);

    const int sda_before = gpio_get_level(BSP_I2C_SDA_GPIO);

    for (int i = 0; i < 9; ++i) {
        gpio_set_level(BSP_I2C_SCL_GPIO, 0);
        esp_rom_delay_us(5);
        gpio_set_level(BSP_I2C_SCL_GPIO, 1);
        esp_rom_delay_us(5);
    }

    /* 补一个 STOP：SCL 高时 SDA 由低变高 */
    gpio_set_level(BSP_I2C_SDA_GPIO, 0);
    esp_rom_delay_us(5);
    gpio_set_level(BSP_I2C_SCL_GPIO, 1);
    esp_rom_delay_us(5);
    gpio_set_level(BSP_I2C_SDA_GPIO, 1);
    esp_rom_delay_us(10);

    const int sda_after = gpio_get_level(BSP_I2C_SDA_GPIO);

    ESP_LOGI(TAG, "总线恢复: 9 个 SCL 脉冲已发出; SDA 恢复前=%d 恢复后=%d %s",
             sda_before, sda_after, (sda_after == 1) ? "(总线已释放 ✔)" : "(SDA 仍为低 ✘)");
    return ESP_OK;
}

esp_err_t bsp_i2c_init(void)
{
    if (s_installed) {
        ESP_LOGW(TAG, "I2C 已初始化，跳过");
        return ESP_OK;
    }

    const i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = BSP_I2C_SDA_GPIO,
        .scl_io_num       = BSP_I2C_SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BSP_I2C_CLK_HZ,
        .clk_flags        = 0,
    };

    esp_err_t err = i2c_param_config(BSP_I2C_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config 失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 收紧硬件 SCL 超时。默认值下探测"不存在的地址"要约 1000 ms，
       112 个地址的全总线扫描会拖到 ~112 秒（实测）。 */
    int old_to = 0;
    (void)i2c_get_timeout(BSP_I2C_PORT, &old_to);
    err = i2c_set_timeout(BSP_I2C_PORT, BSP_I2C_SCL_TIMEOUT_CYCLES);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c_set_timeout 失败: %s（继续用默认值）", esp_err_to_name(err));
    } else {
        int new_to = 0;
        (void)i2c_get_timeout(BSP_I2C_PORT, &new_to);
        ESP_LOGI(TAG, "SCL 超时: %d -> %d (APB 80MHz 周期, 约 %d us)",
                 old_to, new_to, new_to / 80);
    }

    /* 主机模式不需要收发缓冲，故 rx/tx buf 为 0 */
    err = i2c_driver_install(BSP_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install 失败: %s", esp_err_to_name(err));
        return err;
    }

    s_installed = true;
    ESP_LOGI(TAG, "I2C 就绪: port=%d SDA=GPIO%d SCL=GPIO%d %d Hz",
             (int)BSP_I2C_PORT, BSP_I2C_SDA_GPIO, BSP_I2C_SCL_GPIO, BSP_I2C_CLK_HZ);
    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    if (!s_installed) {
        return ESP_OK;
    }
    esp_err_t err = i2c_driver_delete(BSP_I2C_PORT);
    if (err == ESP_OK) {
        s_installed = false;
        ESP_LOGI(TAG, "I2C 已卸载");
    }
    return err;
}

esp_err_t bsp_i2c_write_reg(uint8_t dev, uint8_t reg, const uint8_t *data, size_t len)
{
    if (!s_installed) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > 0 && data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 组装 [reg][data...] 一次发出 */
    uint8_t buf[1 + 32];
    if (len > sizeof(buf) - 1) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = reg;
    if (len > 0) {
        memcpy(&buf[1], data, len);
    }

    return i2c_master_write_to_device(BSP_I2C_PORT, dev, buf, len + 1,
                                      pdMS_TO_TICKS(BSP_I2C_TIMEOUT_MS));
}

esp_err_t bsp_i2c_read_reg(uint8_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    if (!s_installed) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_write_read_device(BSP_I2C_PORT, dev, &reg, 1, data, len,
                                        pdMS_TO_TICKS(BSP_I2C_TIMEOUT_MS));
}

esp_err_t bsp_i2c_probe_timed(uint8_t dev, int64_t *elapsed_us)
{
    if (!s_installed) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t dummy = 0;
    const int64_t t0 = esp_timer_get_time();
    /* 只要能读到 1 个字节，就说明地址被 ACK 了 */
    esp_err_t err = i2c_master_read_from_device(BSP_I2C_PORT, dev, &dummy, 1,
                                                pdMS_TO_TICKS(BSP_I2C_SCAN_TIMEOUT_MS));
    if (elapsed_us != NULL) {
        *elapsed_us = esp_timer_get_time() - t0;
    }
    if (err != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

esp_err_t bsp_i2c_probe(uint8_t dev)
{
    return bsp_i2c_probe_timed(dev, NULL);
}

size_t bsp_i2c_scan(uint8_t *found, size_t max_found)
{
    if (!s_installed) {
        ESP_LOGE(TAG, "扫描前必须先 bsp_i2c_init()");
        return 0;
    }
    if (found == NULL || max_found == 0) {
        return 0;
    }

    const int64_t t_start = esp_timer_get_time();
    size_t count = 0;
    size_t probed = 0;
    bool aborted = false;

    for (uint16_t addr = BSP_I2C_ADDR_MIN; addr <= BSP_I2C_ADDR_MAX; ++addr) {
        const int64_t elapsed_ms = (esp_timer_get_time() - t_start) / 1000;

        /* 最后一道保险：无论单个探测多慢，扫描总时长都不会失控 */
        if (elapsed_ms > BSP_I2C_SCAN_BUDGET_MS) {
            aborted = true;
            ESP_LOGW(TAG, "  已达墙钟预算 %d ms，扫描在 0x%02X 处提前结束",
                     BSP_I2C_SCAN_BUDGET_MS, addr);
            break;
        }

        /* 每 16 个地址打一次进度：万一某次探测异常慢，日志能指出卡在哪一段 */
        if ((probed % 16) == 0) {
            ESP_LOGI(TAG, "  扫描进度 0x%02X.. (已探测 %u 个, 命中 %u, 已用 %lld ms)",
                     addr, (unsigned)probed, (unsigned)count, (long long)elapsed_ms);
        }
        ++probed;

        if (bsp_i2c_probe((uint8_t)addr) == ESP_OK) {
            if (count < max_found) {
                found[count] = (uint8_t)addr;
            }
            ++count;
        }
    }

    ESP_LOGI(TAG, "  扫描%s: 探测 %u 个地址, 命中 %u 个, 总耗时 %lld ms",
             aborted ? "(提前结束)" : "完成", (unsigned)probed, (unsigned)count,
             (long long)((esp_timer_get_time() - t_start) / 1000));
    return count;
}
