/**
 * @file    bsp_i2c.c
 * @brief   I2C 主机总线实现（legacy driver/i2c.h，ESP-IDF 5.1.2）
 */

#include "bsp/bsp_i2c.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "bsp_i2c";

static bool s_installed = false;

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

esp_err_t bsp_i2c_probe(uint8_t dev)
{
    if (!s_installed) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t dummy = 0;
    /* 只要能读到 1 个字节，就说明地址被 ACK 了 */
    esp_err_t err = i2c_master_read_from_device(BSP_I2C_PORT, dev, &dummy, 1,
                                                pdMS_TO_TICKS(BSP_I2C_SCAN_TIMEOUT_MS));
    if (err != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
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

    size_t count = 0;
    for (uint16_t addr = BSP_I2C_ADDR_MIN; addr <= BSP_I2C_ADDR_MAX; ++addr) {
        if (bsp_i2c_probe((uint8_t)addr) == ESP_OK) {
            if (count < max_found) {
                found[count] = (uint8_t)addr;
            }
            ++count;
        }
    }
    return count;
}
