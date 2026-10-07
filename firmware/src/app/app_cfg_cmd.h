/**
 * @file    app_cfg_cmd.h
 * @brief   配置相关的串口控制台命令（`cfg ...`）
 *
 * 与 `app_config.c` 分开的原因：`app_config.c` 是**纯 C、可宿主测试**的，
 * 而本模块要用 ESP-IDF 的 NVS 与日志，只在固件里编译。
 *
 * 命令：
 *   cfg                     打印当前全部配置
 *   cfg info                版本 / CRC / 结构大小 / NVS 用量 / 上次加载结果
 *   cfg get <name>          读单项
 *   cfg set <name> <value>  改单项（改完立即校验限幅）
 *   cfg save                写入 NVS
 *   cfg load                从 NVS 重新读取
 *   cfg reset               恢复出厂默认并擦除 NVS
 *   cfg list                列出所有可设的字段名
 */
#pragma once

#include "app/app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 NVS 并加载配置（在自检阶段调用）。
 * @return 加载结果码（`APP_CFG_OK` / `APP_CFG_ERR_NOENT` 都算正常）
 */
int app_cfg_cmd_init(void);

/** @brief 取当前生效的配置（只读） */
const app_config_t *app_cfg_cmd_get(void);

/**
 * @brief 取**可写**的配置。
 *
 * ⚠️ 改完**必须**调 `app_cfg_cmd_apply_live()`，否则改动不会生效 ——
 * 控制链与动作层各自留了一份配置副本（它们收的是按值传的 `*_cfg_t`）。
 */
app_config_t *app_cfg_cmd_get_mut(void);

/**
 * @brief 把当前配置推到"每帧直接读配置"的消费者（控制链 + 动作层），**不复位状态**。
 *
 * 用于标定键（中位角 ±1°）与 `cfg set`。原版里这些量是模块级全局、每帧直接读，
 * 所以"改一下下一帧就变、而且不会重新站"；`app_chain_reload_cfg()` 会复位状态，
 * 拿它做标定会导致每点一次都重新收敛约 9 秒。
 *
 * @return `APP_CFG_OK`，或底层错误码（早期启动阶段链还没建时返回 OK，属正常）
 */
int app_cfg_cmd_apply_live(void);

/** @brief 把当前配置写入 NVS（= 网页的 `sc` / 控制台的 `cfg save`）。 */
int app_cfg_cmd_save(void);

/** @brief 处理一条 `cfg ...` 命令。`sub` 为子命令，`args` 为其后的参数（可为 NULL）。 */
void app_cfg_cmd_handle(const char *sub, const char *args);

#ifdef __cplusplus
}
#endif
