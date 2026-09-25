/**
 * @file    comm/cmd_queue.h
 * @brief   单槽命令邮箱 + 存活策略（P5 传输层的"策略"那一半）
 *
 * ## 为什么是单槽邮箱，不是 FIFO
 *
 * `ESP-IDF_C迁移表.md` §3「通信队列原则」第 2 条要求"只取最新有效命令，
 * 不执行已经过期的命令"。FIFO 会让运动侧追着积压的旧命令跑 —— 那正是原版
 * GET 轮询抖动的成因。所以这里**只有一个槽**：写者覆盖，读者取走。
 *
 * ## 本模块**不做**什么（刻意，避免重复实现 —— P-22/P-27）
 *
 * | 事情 | 归谁 |
 * |---|---|
 * | 帧的语法/白名单/范围校验 | `proto_decode()` / `proto_decode_legacy()` |
 * | `seq` 严格递增、去重、回绕 | 协议层的解码器（并已计数） |
 * | 20~50 Hz 限速、洪泛丢弃 | 协议层的解码器（`min_interval_ms`）|
 * | **本文档负责**：最新命令的存放、**新鲜度判定、两级超时**、急停闩锁、计数 |
 *
 * ⚠️ `proto_decoder_t` **自己也有** `hb_ms` / `long_ms` / `last_rx_ms` 和
 * `proto_is_stale*()` —— 那是给**协议层自己的逐帧诊断**（`heartbeat_gaps` 等）用的。
 * 两份阈值会不会漂？**不会**，因为数字只有一处：本模块的
 * `cmd_queue_init()` 定阈值，接线时由 `net` 层把同一组值推给解码器
 * （`proto_decoder_set_hb_ms(q->hb_ms)` / `proto_decoder_set_long_ms(q->long_ms)`）。
 * ⇒ 谁要改阈值，改 `CMD_QUEUE_DEFAULT_*` 或 `cmd_queue_init()` 的实参，**别改两处**。
 *
 * ⚠️ `seq` 这里**再过一次**（用协议层那唯一的 `proto_seq_newer()`，不自己写比较），
 * 因为解码器可能被多个来源共用，而"写进邮箱的顺序"必须自己保证。
 *
 * ## 两级超时（§0.5(7)）
 *
 * ```
 * 收到命令 ──hb_ms──> HOLD（输入归零、保持姿态）──long_ms──> RELAX（放松舵机）
 *   FRESH              = btn_stop 语义：狗还站着       = estop 语义：会塌
 * ```
 *
 * 为什么不是一超时就放松：断网时人往往不在旁边。**保持姿态**比"松掉让它趴下"
 * 更可控；但一直保持会让舵机堵转发热，所以再叠一层长超时才真正放松。
 * 两个阈值都必须是显式参数，不许藏在代码里。
 *
 * ⚠️ **HOLD 不等于"硬停动作"**：动作（挥手/坐下）进行中被打断会停在
 * "前腿抬着后腿站着"的中间姿态。所以 HOLD 只表示"不再接受新的运动命令"，
 * 由调用方（`net_task`）按当前模式决定要不要动 —— 见 §0.5(6) 第 2 条。
 *
 * ## 线程安全
 *
 * 本模块**自己不加锁**（纯 C，可宿主测试）。真机上 `net_task` 的 IDF 包装
 * 用一把互斥锁把 `cmd_queue_post*()` 和 `cmd_queue_tick()` 串起来。
 */

#ifndef COMM_CMD_QUEUE_H
#define COMM_CMD_QUEUE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "comm/proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 默认短超时（毫秒）：§7 要求心跳超时 200~300 ms */
#define CMD_QUEUE_DEFAULT_HB_MS     250u
/** 默认长超时（毫秒）：超过它才真正放松舵机 */
#define CMD_QUEUE_DEFAULT_LONG_MS   2000u

/** 邮箱的存活状态 */
typedef enum {
    CMD_QUEUE_IDLE = 0,   /**< 从来没收到过有效命令（上电后一直没人连） */
    CMD_QUEUE_FRESH,      /**< 在 `hb_ms` 之内 —— 正常遥控中 */
    CMD_QUEUE_HOLD,       /**< 超过 `hb_ms`、未超过 `long_ms` —— 输入归零、保持姿态 */
    CMD_QUEUE_RELAX,      /**< 超过 `long_ms` —— 放松舵机（安全态） */
} cmd_queue_state_t;

/** 邮箱计数（**必须可读回** —— 否则"断连自动停车"在真机上没法证明，只能宣称） */
typedef struct {
    uint32_t accepted;      /**< 收下的命令数 */
    uint32_t rejected;      /**< 协议层拒掉的帧数（由 `cmd_queue_count_reject()` 累加）*/
    uint32_t dropped_seq;   /**< 因 `seq` 不更新而被丢弃的（重复/重放/乱序） */
    uint32_t hold_entries;  /**< 进入 HOLD 的次数（= "断连过一次"） */
    uint32_t relax_entries; /**< 进入 RELAX 的次数 */
    uint32_t estop_latched; /**< 收到过的急停请求次数 */
    uint32_t ticks;         /**< `cmd_queue_tick()` 调用次数 */
} cmd_queue_counters_t;

/** 邮箱本体（调用方分配，可放静态区） */
typedef struct {
    proto_cmd_t cmd;          /**< 最近一条被接受的命令 */
    uint32_t    seq;          /**< 它的 `seq` */
    uint32_t    rx_ms;        /**< 它被接受的本机时刻 */
    bool        valid;        /**< 有没有收到过命令 */
    bool        estop;        /**< 急停闩锁：只能由 `cmd_queue_clear_estop()` 清 */

    uint32_t    hb_ms;        /**< 短超时 */
    uint32_t    long_ms;      /**< 长超时 */

    cmd_queue_state_t last_state;   /**< 上一次 `cmd_queue_tick()` 报出的状态（算边沿用） */
    cmd_queue_counters_t cnt;
} cmd_queue_t;

/** 初始化（清零 + 设阈值）。`hb_ms == 0` 会取默认值；`long_ms <= hb_ms` 会取默认值。 */
void cmd_queue_init(cmd_queue_t *q, uint32_t hb_ms, uint32_t long_ms);

/** 改阈值。参数非法时返回 false 且不改。 */
bool cmd_queue_set_timeouts(cmd_queue_t *q, uint32_t hb_ms, uint32_t long_ms);

/**
 * @brief 放进一条**已经通过协议层校验**的命令。
 *
 * @param q        邮箱
 * @param cmd      命令（按值拷贝）
 * @param now_ms   本机时刻
 * @return true    收下了
 * @return false   丢弃（`seq` 不严格更新）—— 已计入 `dropped_seq`
 *
 * @note 即使 `cmd->estop` 为真，本函数**也**会更新"最后收到时刻"：
 *       急停也是客户端活着的证据。
 */
bool cmd_queue_post(cmd_queue_t *q, const proto_cmd_t *cmd, uint32_t now_ms);

/** 协议层拒了一帧 —— 只计数，不动邮箱（邮箱里那条仍然有效）。 */
void cmd_queue_count_reject(cmd_queue_t *q);

/**
 * @brief 从**任意**字节流里读到的急停位（`proto_peek_estop()`）单独置位。
 *
 * §0.5(7)：急停**旁路新鲜度**，即使整帧因其它字段越界被拒也要生效。
 * 只会往安全侧失败（假 1 → 停），所以宁可误停。
 */
void cmd_queue_latch_estop(cmd_queue_t *q, bool est);

/** 清急停闩锁（人工确认后）。 */
void cmd_queue_clear_estop(cmd_queue_t *q);

/**
 * @brief 推进一次存活判定，并取走最新命令。
 *
 * @param q        邮箱
 * @param now_ms   本机时刻
 * @param out      输出：最新命令（可为 NULL）。**只有 `state != CMD_QUEUE_IDLE`
 *                 且 `valid` 时才有意义**；IDLE 时内容是未定义的
 * @param changed  输出：状态**刚刚**变成新值（边沿）。调用方据此**只执行一次**
 *                 进入动作（如"归零输入"），而不是每帧重复执行
 * @return 当前状态
 *
 * @note `now_ms` 用 `uint32_t` 且**按无符号回绕**做差，所以 49.7 天回绕不会误判。
 */
cmd_queue_state_t cmd_queue_tick(cmd_queue_t *q, uint32_t now_ms,
                                 proto_cmd_t *out, bool *changed);

/** 读计数快照（按值拷贝，调用方不需要持锁读后续字段）。 */
void cmd_queue_get_counters(const cmd_queue_t *q, cmd_queue_counters_t *out);

/** 当前状态名（`"IDLE"` / `"FRESH"` / `"HOLD"` / `"RELAX"`）。**只有这一处**（P-27）。 */
const char *cmd_queue_state_name(cmd_queue_state_t st);

/** 距上次收到命令的毫秒数（用无符号回绕）。没收到过返回 `UINT32_MAX`。 */
uint32_t cmd_queue_age_ms(const cmd_queue_t *q, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* COMM_CMD_QUEUE_H */
