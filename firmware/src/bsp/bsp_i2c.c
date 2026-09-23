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
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "bsp_i2c";

static bool s_installed = false;

/** 总线互斥锁（递归）。见 bsp_i2c.h 的说明。 */
static SemaphoreHandle_t s_lock = NULL;

esp_err_t bsp_i2c_lock(uint32_t timeout_ms)
{
    if (s_lock == NULL) {
        /* 驱动还没装：此时不会有并发，直接放行 */
        return ESP_OK;
    }
    if (xSemaphoreTakeRecursive(s_lock, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGE(TAG, "取 I2C 互斥锁超时（%u ms）—— 可能有任务在长时间占用总线",
                 (unsigned)timeout_ms);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void bsp_i2c_unlock(void)
{
    if (s_lock != NULL) {
        xSemaphoreGiveRecursive(s_lock);
    }
}

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

    /* 总线互斥锁：P2 起运动任务与控制台任务会并发访问同一条 I2C */
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateRecursiveMutex();
        if (s_lock == NULL) {
            ESP_LOGE(TAG, "创建 I2C 互斥锁失败");
            (void)i2c_driver_delete(BSP_I2C_PORT);
            return ESP_ERR_NO_MEM;
        }
    }

    s_installed = true;
    ESP_LOGI(TAG, "I2C 就绪: port=%d SDA=GPIO%d SCL=GPIO%d %d Hz（含递归互斥锁）",
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

    esp_err_t lock_err = bsp_i2c_lock(BSP_I2C_TIMEOUT_MS);
    if (lock_err != ESP_OK) {
        return lock_err;
    }
    const esp_err_t err = i2c_master_write_to_device(BSP_I2C_PORT, dev, buf, len + 1,
                                                     pdMS_TO_TICKS(BSP_I2C_TIMEOUT_MS));
    bsp_i2c_unlock();
    return err;
}

esp_err_t bsp_i2c_read_reg(uint8_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    if (!s_installed) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lock_err = bsp_i2c_lock(BSP_I2C_TIMEOUT_MS);
    if (lock_err != ESP_OK) {
        return lock_err;
    }
    const esp_err_t err = i2c_master_write_read_device(BSP_I2C_PORT, dev, &reg, 1, data, len,
                                                       pdMS_TO_TICKS(BSP_I2C_TIMEOUT_MS));
    bsp_i2c_unlock();
    return err;
}

esp_err_t bsp_i2c_probe_timed(uint8_t dev, int64_t *elapsed_us)
{
    if (!s_installed) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t dummy = 0;
    const int64_t t0 = esp_timer_get_time();
    esp_err_t lock_err = bsp_i2c_lock(BSP_I2C_TIMEOUT_MS);
    if (lock_err != ESP_OK) {
        return lock_err;
    }
    /* 只要能读到 1 个字节，就说明地址被 ACK 了 */
    esp_err_t err = i2c_master_read_from_device(BSP_I2C_PORT, dev, &dummy, 1,
                                                pdMS_TO_TICKS(BSP_I2C_SCAN_TIMEOUT_MS));
    bsp_i2c_unlock();
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

/* ==========================================================================
 * 位操作 I2C 扫描
 *
 * 见 bsp_i2c.h 里 bsp_i2c_scan_bitbang() 的说明：IDF 5.1.2 的 legacy 驱动把
 * 事件等待下限硬编码成 1000 ms，导致"探测失败地址"每次要 1 秒。
 * 位操作绕开驱动，整条总线约 20 ms。
 * ========================================================================== */

static void bb_delay(void)
{
    esp_rom_delay_us(BSP_I2C_BB_DELAY_US);
}

static void bb_sda(int level)
{
    /* 开漏输出：写 1 = 释放（由上拉拉高），写 0 = 主动拉低 */
    gpio_set_level(BSP_I2C_SDA_GPIO, level ? 1 : 0);
}

static void bb_scl(int level)
{
    gpio_set_level(BSP_I2C_SCL_GPIO, level ? 1 : 0);
}

static void bb_start(void)
{
    bb_sda(1);
    bb_scl(1);
    bb_delay();
    bb_sda(0);
    bb_delay();
    bb_scl(0);
    bb_delay();
}

static void bb_stop(void)
{
    bb_sda(0);
    bb_delay();
    bb_scl(1);
    bb_delay();
    bb_sda(1);
    bb_delay();
}

/** @return 1 = 收到 ACK，0 = NACK */
static int bb_write_byte(uint8_t b)
{
    for (int i = 7; i >= 0; --i) {
        bb_sda((b >> i) & 1);
        bb_delay();
        bb_scl(1);
        bb_delay();
        bb_scl(0);
        bb_delay();
    }
    /* 第 9 个时钟：释放 SDA，看从机是否拉低 */
    bb_sda(1);
    bb_delay();
    bb_scl(1);
    bb_delay();
    const int ack = (gpio_get_level(BSP_I2C_SDA_GPIO) == 0);
    bb_scl(0);
    bb_delay();
    return ack;
}

esp_err_t bsp_i2c_bitbang_begin(void)
{
    if (s_installed) {
        ESP_LOGE(TAG, "位操作扫描需在 I2C 驱动安装前（或 deinit 后）调用");
        return ESP_ERR_INVALID_STATE;
    }

    gpio_set_direction(BSP_I2C_SCL_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(BSP_I2C_SDA_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(BSP_I2C_SCL_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BSP_I2C_SDA_GPIO, GPIO_PULLUP_ONLY);

    bb_sda(1);
    bb_scl(1);
    esp_rom_delay_us(10);
    return ESP_OK;
}

size_t bsp_i2c_scan_bitbang(uint8_t *found, size_t max_found)
{
    if (s_installed) {
        ESP_LOGE(TAG, "位操作扫描需在 I2C 驱动安装前（或 deinit 后）调用");
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
        /* 位操作本来就快，这里只是防御性的最后一道保险 */
        if (((esp_timer_get_time() - t_start) / 1000) > BSP_I2C_SCAN_BUDGET_MS) {
            aborted = true;
            ESP_LOGW(TAG, "  已达墙钟预算 %d ms，位操作扫描在 0x%02X 处提前结束",
                     BSP_I2C_SCAN_BUDGET_MS, addr);
            break;
        }
        ++probed;

        bb_start();
        const int ack = bb_write_byte((uint8_t)(addr << 1)); /* 只发地址 + 读 ACK */
        bb_stop();

        if (ack) {
            if (count < max_found) {
                found[count] = (uint8_t)addr;
            }
            ++count;
        }
    }

    ESP_LOGI(TAG, "位操作扫描%s: 探测 %u 个地址, 命中 %u 个, 总耗时 %lld ms",
             aborted ? "(提前结束)" : "完成", (unsigned)probed, (unsigned)count,
             (long long)((esp_timer_get_time() - t_start) / 1000));
    return count;
}
