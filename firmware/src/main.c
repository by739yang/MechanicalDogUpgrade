/**
 * @file    main.c
 * @brief   机械狗 ESP-IDF C 迁移 —— P0 阶段入口
 *
 * 阶段目标（见 ESP-IDF_C迁移表.md）：
 *   - 建立 PlatformIO + ESP-IDF 工程，串口日志可用
 *   - 实现 I2C 扫描，确认能看到 0x40、0x41
 *   - 实现 PCA9685 单通道控制（串口命令驱动，上电不自动动作）
 *
 * 硬件基线（2026-09-19 实测，见仓库根的《硬件实物核对清单.md》）：
 *   - ESP32（非 S3），4 MB Flash
 *   - 唯一 I2C 总线：SDA=GPIO21, SCL=GPIO22, 100 kHz
 *   - PCA9685 0x40 / 0x41；0x70 是 all-call 广播地址
 *   - 板上没有 IMU
 */

#include <stdio.h>

#include "sdkconfig.h"

#include "app/app_p0.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "main";

void app_main(void)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);

    uint32_t flash_size = 0;
    (void)esp_flash_get_size(NULL, &flash_size);

    ESP_LOGI(TAG, "============================================================");
    ESP_LOGI(TAG, " MechanicalDogUpgrade / firmware");
    ESP_LOGI(TAG, " ESP-IDF C 迁移 —— 阶段 P0");
    ESP_LOGI(TAG, " 工程 + 日志 + I2C 扫描 + PCA9685 单通道控制");
    ESP_LOGI(TAG, "============================================================");
    ESP_LOGI(TAG, "构建时间 : %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "ESP-IDF  : %s", esp_get_idf_version());
    ESP_LOGI(TAG, "目标芯片 : %s, %d 核, rev %d.%d",
             CONFIG_IDF_TARGET, chip.cores,
             chip.revision / 100, chip.revision % 100);
    ESP_LOGI(TAG, "Flash    : %u MB", (unsigned)(flash_size / (1024u * 1024u)));
    ESP_LOGI(TAG, "空闲堆   : %u 字节", (unsigned)esp_get_free_heap_size());
    ESP_LOGI(TAG, "硬件基线 : PCA9685 0x40/0x41 @ SDA21/SCL22 100kHz；本机无 IMU");

    app_p0_start();
}
