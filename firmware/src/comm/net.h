/**
 * @file    comm/net.h
 * @brief   P5 传输层：WiFi 纯 AP + HTTP 服务 + 命令任务（**只在固件里编译**）
 *
 * 与 `app_cfg_cmd.c` 是 `app_config.c` 的 IDF 侧同一个套路：本文件是
 * `comm/proto.c` / `comm/cmd_queue.c` / `comm/web_cmd.c` 三个**纯 C** 模块的 IDF 外壳。
 * 那三个模块不 include 任何 ESP-IDF 头文件，所以能在电脑上逐值验证；
 * 本文件负责"把字节从网上搬进来、把命令落到机器人上"。
 *
 * ## 数据流（§3「通信队列原则」）
 *
 * ```text
 *   浏览器 ──HTTP──> httpd 任务
 *                      │ ① proto_decode_legacy() / proto_decode()   解析+校验
 *                      │ ② proto_peek_estop()                       急停旁路
 *                      │ ③ cmd_queue_post()                         放进单槽邮箱
 *                      ▼
 *                  （互斥锁保护 src/net.c 的 s_q）
 *                      ▼
 *   comm 任务 (100 Hz) ──> cmd_queue_tick()
 *                      │ FRESH  → web_cmd_process_request() → app_chain_* / motion_*
 *                      │ HOLD   → 输入归零、保持姿态（= btn_stop 语义）
 *                      │ RELAX  → motion_stop(超时) 放松舵机
 *                      ▼
 *                  控制任务 (100 Hz) → servo_out → I2C → PCA9685
 * ```
 *
 * ⚠️ **httpd 任务里不碰舵机、不做数学**（§8.1/§8.6）：它只解析和入队。
 *
 * ## 超时只能有**一个** owner（这条是踩过坑才定下来的）
 *
 * - **短超时 250 ms**：输入归零、保持姿态（= `btn_stop` 语义）。
 * - **长超时 2 s**：放松舵机（`motion_stop`，安全态）。
 * - 两者都归 `cmd_queue`，阈值只有一处。
 * - **`motion` 自己的命令超时必须关掉**（`motion_set_timeout_ms(0)`）。
 *   ⚠️ 一开始把它设成 3 秒当"兜底"，结果是：网页轮询一停（哪怕只是客户端在等
 *   站姿收敛那十几秒没发命令），它就先触发并**终止控制任务**；而任务退出后
 *   **没有任何东西会再把它拉起来** ⇒ 重新连上后摇杆永久失效。
 *   真机日志：`motion: 命令超时（3003 ms > 3000 ms）：松力停车` +
 *   `控制任务结束，原因: 命令超时`。
 * - ⇒ 重新连上的恢复由 `comm_task` 负责：状态从 RELAX 回到 FRESH 时重新
 *   `motion_start()`。**急停例外** —— 急停在邮箱里是闩锁的，状态会停在 RELAX，
 *   必须显式清除才回到 FRESH，所以不会"按了急停自己站起来"。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app/app_config.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动 AP + HTTP 服务 + 命令任务（读 `app_config` 里的 SSID/密码）。
 *
 * @note 要求 `app_cfg_cmd_init()` 与 `app_motion_cmd_init()` 已经调用过。
 *       重复调用是安全的（已启动就直接返回）。
 */
esp_err_t net_start(void);

/** @brief 停 HTTP 服务（AP 保持，便于排查）。 */
void net_stop(void);

/** @brief 传输层是否已经启动 */
bool net_is_up(void);

/** @brief AP 的 IP 字符串（未启动返回 `"-"`） */
const char *net_ip_str(void);

/** @brief 当前连在 AP 上的客户端数 */
uint32_t net_client_count(void);

/** @brief 处理一条 `net ...` 控制台命令（`sub` 可为 NULL 表示无参数）。 */
void net_cmd_handle(const char *sub, const char *args);

#ifdef __cplusplus
}
#endif
