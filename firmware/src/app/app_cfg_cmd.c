/**
 * @file    app_cfg_cmd.c
 * @brief   `cfg` 串口命令实现
 *
 * 用一张**字段表**同时驱动"打印"和"设置"，避免写 50 个 switch 分支。
 * 表项用 `offsetof` 定位字段，类型决定打印格式与解析方式。
 */

#include "app/app_cfg_cmd.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/app_config_nvs.h"
#include "esp_log.h"

static const char *TAG = "cfg";

/** 当前生效的配置（本模块持有；只读暴露给外部） */
static app_config_t s_cfg;

/** 上次加载的结果码，供 `cfg info` 显示 */
static int s_last_load_rc = APP_CFG_ERR_NOENT;

/* ==========================================================================
 * 字段表
 * ========================================================================== */

typedef enum {
    CT_F32,   /**< float  */
    CT_I32,   /**< int32_t */
    CT_STR,   /**< char[]，固定长度 */
} cfg_type_t;

typedef struct {
    const char *name;
    cfg_type_t  type;
    size_t      offset;
    size_t      str_len;   /**< 仅 CT_STR 用 */
} cfg_field_t;

#define OFF(m) offsetof(app_config_t, m)

static const cfg_field_t kFields[] = {
    /* ---- 舵机中位角（按腿1..腿4，每腿 髋/大/小） ---- */
    { "c1_hip",   CT_F32, OFF(servo_center[0][0]), 0 },
    { "c1_thigh", CT_F32, OFF(servo_center[0][1]), 0 },
    { "c1_shank", CT_F32, OFF(servo_center[0][2]), 0 },
    { "c2_hip",   CT_F32, OFF(servo_center[1][0]), 0 },
    { "c2_thigh", CT_F32, OFF(servo_center[1][1]), 0 },
    { "c2_shank", CT_F32, OFF(servo_center[1][2]), 0 },
    { "c3_hip",   CT_F32, OFF(servo_center[2][0]), 0 },
    { "c3_thigh", CT_F32, OFF(servo_center[2][1]), 0 },
    { "c3_shank", CT_F32, OFF(servo_center[2][2]), 0 },
    { "c4_hip",   CT_F32, OFF(servo_center[3][0]), 0 },
    { "c4_thigh", CT_F32, OFF(servo_center[3][1]), 0 },
    { "c4_shank", CT_F32, OFF(servo_center[3][2]), 0 },
    /* ---- 几何 ---- */
    { "l1", CT_F32, OFF(l1), 0 },
    { "l2", CT_F32, OFF(l2), 0 },
    { "l",  CT_F32, OFF(l), 0 },
    { "b",  CT_F32, OFF(b), 0 },
    { "w",  CT_F32, OFF(w), 0 },
    { "leg_len_ref", CT_F32, OFF(leg_len_ref), 0 },
    /* ---- 站姿与姿态 ---- */
    { "h_goal",      CT_F32, OFF(h_goal), 0 },
    { "in_y",        CT_F32, OFF(in_y), 0 },
    { "in_pit",      CT_F32, OFF(in_pit), 0 },
    { "in_rol",      CT_F32, OFF(in_rol), 0 },
    { "pit_max_ang", CT_F32, OFF(pit_max_ang), 0 },
    { "rol_max_ang", CT_F32, OFF(rol_max_ang), 0 },
    { "xs_max",      CT_F32, OFF(xs_max), 0 },
    { "cg_x",        CT_F32, OFF(cg_x), 0 },
    { "cg_y",        CT_F32, OFF(cg_y), 0 },
    { "kp_h",        CT_F32, OFF(kp_h), 0 },
    { "kp_g",        CT_F32, OFF(kp_g), 0 },
    /* ---- 步态 ---- */
    { "ts",         CT_F32, OFF(ts), 0 },
    { "faai",       CT_F32, OFF(faai), 0 },
    { "speed",      CT_F32, OFF(speed), 0 },
    { "h",          CT_F32, OFF(h), 0 },
    { "trot_cg_f",  CT_F32, OFF(trot_cg_f), 0 },
    { "trot_cg_b",  CT_F32, OFF(trot_cg_b), 0 },
    { "trot_cg_t",  CT_F32, OFF(trot_cg_t), 0 },
    { "walk_faai",  CT_F32, OFF(walk_faai), 0 },
    { "walk_h",     CT_F32, OFF(walk_h), 0 },
    { "walk_speed", CT_F32, OFF(walk_speed), 0 },
    /* ---- 髋辅助 ---- */
    { "hip_k_roll",    CT_F32, OFF(hip_k_roll), 0 },
    { "hip_k_pitch",   CT_F32, OFF(hip_k_pitch), 0 },
    { "hip_k_turn",    CT_F32, OFF(hip_k_turn), 0 },
    { "hip_delta_max", CT_F32, OFF(hip_delta_max), 0 },
    /* ---- 其他整数 ---- */
    { "ma_case",      CT_I32, OFF(ma_case), 0 },
    { "joy_fwd_sign", CT_I32, OFF(joy_fwd_sign), 0 },
    { "cal_leg_sel",  CT_I32, OFF(cal_leg_sel), 0 },
    /* ---- 机械臂 ---- */
    { "arm_upper_init", CT_F32, OFF(arm_upper_init), 0 },
    { "arm_fore_init",  CT_F32, OFF(arm_fore_init), 0 },
    { "arm_upper_min",  CT_F32, OFF(arm_upper_min), 0 },
    { "arm_upper_max",  CT_F32, OFF(arm_upper_max), 0 },
    { "arm_fore_min",   CT_F32, OFF(arm_fore_min), 0 },
    { "arm_fore_max",   CT_F32, OFF(arm_fore_max), 0 },
    { "arm_upper_rate", CT_F32, OFF(arm_upper_rate), 0 },
    { "arm_fore_rate",  CT_F32, OFF(arm_fore_rate), 0 },
    { "arm_grip_open",  CT_F32, OFF(arm_grip_open), 0 },
    { "arm_grip_close", CT_F32, OFF(arm_grip_close), 0 },
    { "arm_upper_ch",   CT_I32, OFF(arm_upper_ch), 0 },
    { "arm_fore_ch",    CT_I32, OFF(arm_fore_ch), 0 },
    { "arm_grip_ch",    CT_I32, OFF(arm_grip_ch), 0 },
    { "arm_upper_board", CT_I32, OFF(arm_upper_board), 0 },
    { "arm_fore_board",  CT_I32, OFF(arm_fore_board), 0 },
    { "arm_grip_board",  CT_I32, OFF(arm_grip_board), 0 },
    { "arm_grip_gpio",   CT_I32, OFF(arm_grip_gpio), 0 },
    /* ---- WiFi（纯 AP） ---- */
    { "ap_ssid",     CT_STR, OFF(ap_ssid),     sizeof(s_cfg.ap_ssid) },
    { "ap_password", CT_STR, OFF(ap_password), sizeof(s_cfg.ap_password) },
};

#define NFIELDS ((int)(sizeof(kFields) / sizeof(kFields[0])))

static const cfg_field_t *find_field(const char *name)
{
    for (int i = 0; i < NFIELDS; ++i) {
        if (strcmp(kFields[i].name, name) == 0) {
            return &kFields[i];
        }
    }
    return NULL;
}

static void *field_ptr(const cfg_field_t *f)
{
    return (void *)((uint8_t *)&s_cfg + f->offset);
}

/* ==========================================================================
 * 打印
 * ========================================================================== */

static void print_field(const cfg_field_t *f)
{
    const void *p = field_ptr(f);
    switch (f->type) {
    case CT_F32:
        ESP_LOGI(TAG, "  %-16s = %.4f", f->name, (double)(*(const float *)p));
        break;
    case CT_I32:
        ESP_LOGI(TAG, "  %-16s = %ld", f->name, (long)(*(const int32_t *)p));
        break;
    case CT_STR:
        ESP_LOGI(TAG, "  %-16s = \"%s\"", f->name, (const char *)p);
        break;
    }
}

static void print_all(void)
{
    ESP_LOGI(TAG, "---- 配置（version=%u, crc=0x%08lX）----",
             (unsigned)s_cfg.version, (unsigned long)s_cfg.crc32);
    for (int i = 0; i < NFIELDS; ++i) {
        print_field(&kFields[i]);
    }
}

static void print_info(void)
{
    size_t used = 0, total = 0, freee = 0, bytes = 0;
    const esp_err_t e = app_config_nvs_stats(&used, &total, &freee, &bytes);

    ESP_LOGI(TAG, "---- 配置状态 ----");
    ESP_LOGI(TAG, "  结构版本      : %u (APP_CFG_VERSION=%u)",
             (unsigned)s_cfg.version, (unsigned)APP_CFG_VERSION);
    ESP_LOGI(TAG, "  CRC32         : 0x%08lX", (unsigned long)s_cfg.crc32);
    ESP_LOGI(TAG, "  sizeof(结构)  : %u 字节", (unsigned)sizeof(app_config_t));
    ESP_LOGI(TAG, "  上次加载结果  : %d (%s)",
             s_last_load_rc, app_cfg_strerror(s_last_load_rc));
    if (e == ESP_OK) {
        ESP_LOGI(TAG, "  NVS 条目      : 已用 %u / 共 %u, 空闲 %u",
                 (unsigned)used, (unsigned)total, (unsigned)freee);
    } else {
        ESP_LOGW(TAG, "  NVS 统计不可用: %s", esp_err_to_name(e));
    }
    ESP_LOGI(TAG, "  提示：改完记得敲 `cfg save`，否则掉电就丢");
}

static void print_list(void)
{
    char line[128];
    size_t used = 0;
    ESP_LOGI(TAG, "可用字段名（%d 个）:", NFIELDS);
    for (int i = 0; i < NFIELDS; ++i) {
        const int n = snprintf(&line[used], sizeof(line) - used, "%s%s",
                               (used == 0) ? "" : " ", kFields[i].name);
        if (n <= 0 || (size_t)n >= sizeof(line) - used) {
            ESP_LOGI(TAG, "  %s", line);
            used = 0;
            line[0] = '\0';
            continue;
        }
        used += (size_t)n;
    }
    if (used > 0) {
        ESP_LOGI(TAG, "  %s", line);
    }
}

/* ==========================================================================
 * 设置
 * ========================================================================== */

static void cmd_get(const char *name)
{
    const cfg_field_t *f = (name != NULL) ? find_field(name) : NULL;
    if (f == NULL) {
        ESP_LOGW(TAG, "没有这个字段：%s（敲 `cfg list` 看全部）",
                 (name != NULL) ? name : "(空)");
        return;
    }
    print_field(f);
}

static void cmd_set(const char *name, const char *value)
{
    if (name == NULL || value == NULL) {
        ESP_LOGW(TAG, "用法: cfg set <name> <value>");
        return;
    }

    const cfg_field_t *f = find_field(name);
    if (f == NULL) {
        ESP_LOGW(TAG, "没有这个字段：%s（敲 `cfg list` 看全部）", name);
        return;
    }

    void *p = field_ptr(f);
    switch (f->type) {
    case CT_F32: {
        char *end = NULL;
        const float v = strtof(value, &end);
        if (end == value || *end != '\0') {
            ESP_LOGW(TAG, "值不是合法数字：%s", value);
            return;
        }
        *(float *)p = v;
        break;
    }
    case CT_I32: {
        char *end = NULL;
        const long v = strtol(value, &end, 0);   /* base 0：允许 0x40 这种写法 */
        if (end == value || *end != '\0') {
            ESP_LOGW(TAG, "值不是合法整数：%s", value);
            return;
        }
        *(int32_t *)p = (int32_t)v;
        break;
    }
    case CT_STR:
        strncpy((char *)p, value, f->str_len - 1);
        ((char *)p)[f->str_len - 1] = '\0';
        break;
    }

    /* 立刻校验限幅，并如实报告被改了 */
    int changed = 0;
    char msg[128];
    app_config_validate(&s_cfg, &changed, msg, sizeof(msg));
    if (changed > 0) {
        ESP_LOGW(TAG, "%s 已设置，但被限幅（改动了 %d 项，首项：%s）",
                 name, changed, msg);
    }
    print_field(f);
    ESP_LOGI(TAG, "（尚未保存，敲 `cfg save` 写入 NVS）");
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

int app_cfg_cmd_init(void)
{
    const esp_err_t e = app_config_nvs_init();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "NVS 初始化失败，改用默认配置（本次修改无法保存）");
        app_config_defaults(&s_cfg);
        app_config_validate(&s_cfg, NULL, NULL, 0);
        s_last_load_rc = APP_CFG_ERR_IO;
        return APP_CFG_ERR_IO;
    }

    s_last_load_rc = app_config_load(&s_cfg, app_config_nvs_store());

    if (s_last_load_rc == APP_CFG_OK) {
        ESP_LOGI(TAG, "配置已从 NVS 载入（version=%u, crc=0x%08lX）",
                 (unsigned)s_cfg.version, (unsigned long)s_cfg.crc32);
    } else {
        ESP_LOGW(TAG, "未使用已保存的配置：%s", app_cfg_strerror(s_last_load_rc));
        switch (s_last_load_rc) {
        case APP_CFG_ERR_NOENT:
            ESP_LOGI(TAG, "  首次开机属正常，已用内置默认值（= config.py + config_s.py）");
            break;
        case APP_CFG_ERR_CRC:
        case APP_CFG_ERR_VERSION:
            ESP_LOGW(TAG, "  已回落到默认值，避免半损坏的配置把舵机中位角搞乱");
            break;
        default:
            break;
        }
    }

    /* 关键几项打出来，便于和 config_s.py 对照 */
    ESP_LOGI(TAG, "  关键值: h_goal=%.1f faai=%.3f walk_faai=%.3f ma_case=%ld",
             (double)s_cfg.h_goal, (double)s_cfg.faai, (double)s_cfg.walk_faai,
             (long)s_cfg.ma_case);
    return s_last_load_rc;
}

const app_config_t *app_cfg_cmd_get(void)
{
    return &s_cfg;
}

void app_cfg_cmd_handle(const char *sub, const char *args)
{
    if (sub == NULL || sub[0] == '\0') {
        print_all();
        return;
    }

    if (strcmp(sub, "info") == 0) {
        print_info();
    } else if (strcmp(sub, "list") == 0) {
        print_list();
    } else if (strcmp(sub, "get") == 0) {
        cmd_get(args);
    } else if (strcmp(sub, "set") == 0) {
        if (args == NULL) {
            ESP_LOGW(TAG, "用法: cfg set <name> <value>");
            return;
        }
        char name[64];
        size_t i = 0;
        while (args[i] != '\0' && args[i] != ' ' && i < sizeof(name) - 1) {
            name[i] = args[i];
            ++i;
        }
        name[i] = '\0';
        const char *value = args + i;
        while (*value == ' ') {
            ++value;
        }
        if (*value == '\0') {
            ESP_LOGW(TAG, "用法: cfg set <name> <value>");
            return;
        }
        cmd_set(name, value);
    } else if (strcmp(sub, "save") == 0) {
        const int rc = app_config_save(&s_cfg, app_config_nvs_store());
        if (rc == APP_CFG_OK) {
            ESP_LOGI(TAG, "已保存到 NVS（version=%u, crc=0x%08lX）。"
                          "掉电重启后仍然生效。",
                     (unsigned)s_cfg.version, (unsigned long)s_cfg.crc32);
        } else {
            ESP_LOGE(TAG, "保存失败: %s", app_cfg_strerror(rc));
        }
    } else if (strcmp(sub, "load") == 0) {
        s_last_load_rc = app_config_load(&s_cfg, app_config_nvs_store());
        ESP_LOGI(TAG, "重新载入: %s", app_cfg_strerror(s_last_load_rc));
        ESP_LOGI(TAG, "  关键值: h_goal=%.1f faai=%.3f",
                 (double)s_cfg.h_goal, (double)s_cfg.faai);
    } else if (strcmp(sub, "reset") == 0) {
        const int rc = app_config_reset(&s_cfg, app_config_nvs_store());
        s_last_load_rc = APP_CFG_ERR_NOENT;
        ESP_LOGI(TAG, "已恢复出厂默认并擦除 NVS: %s", app_cfg_strerror(rc));
        ESP_LOGI(TAG, "  h_goal=%.1f（config_s.py 原值 81）", (double)s_cfg.h_goal);
    } else {
        ESP_LOGW(TAG, "未知子命令 '%s'。用法：cfg [info|list|get|set|save|load|reset]",
                 sub);
    }
}
