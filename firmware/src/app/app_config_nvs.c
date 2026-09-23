/**
 * @file    app_config_nvs.c
 * @brief   ESP-IDF NVS 存储后端实现
 */

#include "app/app_config_nvs.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "cfg_nvs";

/* ==========================================================================
 * app_cfg_store_t 的三个回调
 *
 * 把 nvs 的错误码翻译成 app_cfg_ret_t —— app_config 层不认识 esp_err_t，
 * 这样它才能被宿主测试编译。
 * ========================================================================== */

static int nvs_get_blob_cb(const char *key, void *out, size_t *len)
{
    if (key == NULL || out == NULL || len == NULL) {
        return APP_CFG_ERR_ARG;
    }

    nvs_handle_t h;
    esp_err_t e = nvs_open(APP_CFG_NVS_NS, NVS_READONLY, &h);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        return APP_CFG_ERR_NOENT;   /* 命名空间还没建过 = 首次开机 */
    }
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open 失败: %s", esp_err_to_name(e));
        return APP_CFG_ERR_IO;
    }

    size_t want = *len;
    e = nvs_get_blob(h, key, out, &want);
    nvs_close(h);

    if (e == ESP_ERR_NVS_NOT_FOUND) {
        *len = 0;
        return APP_CFG_ERR_NOENT;
    }
    if (e == ESP_ERR_NVS_INVALID_LENGTH) {
        /* 存的大小和我们期望的不一样 —— 让上层按"版本不符"处理 */
        *len = want;
        return APP_CFG_OK;
    }
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "nvs_get_blob 失败: %s", esp_err_to_name(e));
        return APP_CFG_ERR_IO;
    }

    *len = want;
    return APP_CFG_OK;
}

static int nvs_set_blob_cb(const char *key, const void *data, size_t len)
{
    if (key == NULL || data == NULL || len == 0) {
        return APP_CFG_ERR_ARG;
    }

    nvs_handle_t h;
    esp_err_t e = nvs_open(APP_CFG_NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open(RW) 失败: %s", esp_err_to_name(e));
        return APP_CFG_ERR_IO;
    }

    e = nvs_set_blob(h, key, data, len);
    if (e == ESP_OK) {
        e = nvs_commit(h);   /* 不 commit 掉电就丢 */
    }
    nvs_close(h);

    if (e != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob/commit 失败: %s", esp_err_to_name(e));
        return APP_CFG_ERR_IO;
    }
    return APP_CFG_OK;
}

static int nvs_erase_key_cb(const char *key)
{
    if (key == NULL) {
        return APP_CFG_ERR_ARG;
    }

    nvs_handle_t h;
    esp_err_t e = nvs_open(APP_CFG_NVS_NS, NVS_READWRITE, &h);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        return APP_CFG_ERR_NOENT;
    }
    if (e != ESP_OK) {
        return APP_CFG_ERR_IO;
    }

    e = nvs_erase_key(h, key);   /* 键不存在也算成功语义 */
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        e = ESP_OK;
    }
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    nvs_close(h);

    return (e == ESP_OK) ? APP_CFG_OK : APP_CFG_ERR_IO;
}

static const app_cfg_store_t s_store = {
    .get_blob  = nvs_get_blob_cb,
    .set_blob  = nvs_set_blob_cb,
    .erase_key = nvs_erase_key_cb,
};

const app_cfg_store_t *app_config_nvs_store(void)
{
    return &s_store;
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

esp_err_t app_config_nvs_init(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS 需要擦除后重建 (%s)", esp_err_to_name(e));
        e = nvs_flash_erase();
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "nvs_flash_erase 失败: %s", esp_err_to_name(e));
            return e;
        }
        e = nvs_flash_init();
    }
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init 失败: %s", esp_err_to_name(e));
        return e;
    }

    /* 打印一下用量，便于观察 */
    nvs_stats_t st;
    if (nvs_get_stats(NULL, &st) == ESP_OK) {
        ESP_LOGI(TAG, "NVS 就绪: 已用 %u / 共 %u 条目, 空闲 %u",
                 (unsigned)st.used_entries, (unsigned)st.total_entries,
                 (unsigned)st.free_entries);
    } else {
        ESP_LOGI(TAG, "NVS 就绪");
    }
    return ESP_OK;
}

esp_err_t app_config_nvs_stats(size_t *used_entries, size_t *total_entries,
                               size_t *free_entries, size_t *total_bytes)
{
    nvs_stats_t st;
    const esp_err_t e = nvs_get_stats(NULL, &st);
    if (e != ESP_OK) {
        return e;
    }
    if (used_entries)  { *used_entries  = st.used_entries; }
    if (total_entries) { *total_entries = st.total_entries; }
    if (free_entries)  { *free_entries  = st.free_entries; }
    if (total_bytes)   { *total_bytes   = 0; }
    return ESP_OK;
}
