/**
 * @file    app_config_nvs.h
 * @brief   用 ESP-IDF NVS 实现 app_config 的存储后端
 *
 * 为什么放在 `app/` 而不是 `bsp/`：它同时依赖 `app_config.h`（应用层）和
 * `nvs_flash.h`（ESP-IDF），是两者的**桥接**，放应用层更合适。
 *
 * 底层分区在启动日志里可见，**无需修改分区表**：
 * ```
 * I (69) boot:  0 nvs   WiFi data   01 02  00009000  00006000    # 24 KB @ 0x9000
 * ```
 */
#pragma once

#include "esp_err.h"

#include "app/app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 NVS 分区。
 *
 * 处理两种常见情况：分区已满（`ESP_ERR_NVS_NO_FREE_PAGES`）和
 * 版本更新（`ESP_ERR_NVS_NEW_VERSION_FOUND`）—— 两者都擦除后重试。
 */
esp_err_t app_config_nvs_init(void);

/**
 * @brief 取得 NVS 存储后端（供 `app_config_load/save/reset` 使用）。
 */
const app_cfg_store_t *app_config_nvs_store(void);

/**
 * @brief 读取 NVS 分区使用统计（用于 `cfg info`）。
 */
esp_err_t app_config_nvs_stats(size_t *used_entries, size_t *total_entries,
                               size_t *free_entries, size_t *total_bytes);

#ifdef __cplusplus
}
#endif
