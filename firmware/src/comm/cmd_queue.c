/**
 * @file    comm/cmd_queue.c
 * @brief   单槽命令邮箱 + 存活策略的实现（见 `cmd_queue.h` 的设计说明）
 *
 * 纯 C：不 include 任何 ESP-IDF 头文件，不分配内存，不用浮点。
 * 自己不加锁（调用方负责串行化）。
 */

#include "comm/cmd_queue.h"

#include <string.h>

/* ==========================================================================
 * 初始化与阈值
 * ======================================================================== */

void cmd_queue_init(cmd_queue_t *q, uint32_t hb_ms, uint32_t long_ms)
{
    if (q == NULL) {
        return;
    }
    memset(q, 0, sizeof(*q));

    q->hb_ms = (hb_ms == 0u) ? CMD_QUEUE_DEFAULT_HB_MS : hb_ms;

    /*
     * 长超时必须**严格大于**短超时，否则两级策略没有意义（HOLD 永远进不去）。
     * 实参不合理时取 `hb + 默认长超时`，是确定的、可预期的行为 —— 不静默取 0。
     */
    q->long_ms = (long_ms <= q->hb_ms) ? (q->hb_ms + CMD_QUEUE_DEFAULT_LONG_MS)
                                      : long_ms;

    /* memset 已经把它清成 CMD_QUEUE_IDLE(0) 了，这里写出来只为表明意图 */
    q->last_state = CMD_QUEUE_IDLE;
}

bool cmd_queue_set_timeouts(cmd_queue_t *q, uint32_t hb_ms, uint32_t long_ms)
{
    if (q == NULL || hb_ms == 0u || long_ms <= hb_ms) {
        return false;
    }
    q->hb_ms = hb_ms;
    q->long_ms = long_ms;
    return true;
}

/* ==========================================================================
 * 写入
 * ======================================================================== */

bool cmd_queue_post(cmd_queue_t *q, const proto_cmd_t *cmd, uint32_t now_ms)
{
    if (q == NULL || cmd == NULL) {
        return false;
    }

    /*
     * 顺序由**协议层那唯一的** `proto_seq_newer()` 判定（回绕安全），
     * 本模块不自己写 `a > b` —— 那种比较在 49.7 天回绕时会判错。
     */
    if (q->valid && !proto_seq_newer(cmd->seq, q->seq)) {
        q->cnt.dropped_seq++;
        return false;
    }

    q->cmd    = *cmd;
    q->seq    = cmd->seq;
    q->rx_ms  = now_ms;
    q->valid  = true;
    q->cnt.accepted++;

    if (cmd->est != 0) {
        q->estop = true;
        q->cnt.estop_latched++;
    }
    return true;
}

void cmd_queue_count_reject(cmd_queue_t *q)
{
    if (q != NULL) {
        q->cnt.rejected++;
    }
}

void cmd_queue_latch_estop(cmd_queue_t *q, bool est)
{
    if (q == NULL || !est) {
        /* `est == false` **不清**闩锁 —— 清闩锁是 `cmd_queue_clear_estop()` 的事。
           否则一次 peek 读不到 est 就把急停解除了，那是危险的方向。 */
        return;
    }
    q->estop = true;
    q->cnt.estop_latched++;
}

void cmd_queue_clear_estop(cmd_queue_t *q)
{
    if (q != NULL) {
        q->estop = false;
    }
}

/* ==========================================================================
 * 读取
 * ======================================================================== */

cmd_queue_state_t cmd_queue_tick(cmd_queue_t *q, uint32_t now_ms,
                                 proto_cmd_t *out, bool *changed)
{
    if (q == NULL) {
        if (changed != NULL) {
            *changed = false;
        }
        return CMD_QUEUE_IDLE;
    }
    q->cnt.ticks++;

    cmd_queue_state_t st;
    if (!q->valid) {
        st = CMD_QUEUE_IDLE;
    } else {
        /* 无符号相减 ⇒ 49.7 天回绕也是对的 */
        const uint32_t age = now_ms - q->rx_ms;
        if (age <= q->hb_ms) {
            st = CMD_QUEUE_FRESH;
        } else if (age <= q->long_ms) {
            st = CMD_QUEUE_HOLD;
        } else {
            st = CMD_QUEUE_RELAX;
        }
    }

    /*
     * 急停压过新鲜度，而且**直接报成 RELAX**：
     * 让调用方不可能"忘了看 estop 标志"而继续遥控。
     */
    if (q->estop) {
        st = CMD_QUEUE_RELAX;
    }

    if (st != q->last_state) {
        if (st == CMD_QUEUE_HOLD) {
            q->cnt.hold_entries++;
        } else if (st == CMD_QUEUE_RELAX) {
            q->cnt.relax_entries++;
        }
        q->last_state = st;
        if (changed != NULL) {
            *changed = true;
        }
    } else if (changed != NULL) {
        *changed = false;
    }

    if (out != NULL) {
        if (q->estop) {
            /* 急停时给出一个**全零 + est=1** 的命令：调用方拿到什么都不可能乱动 */
            proto_cmd_zero(out);
            out->seq = q->seq;
            out->est = 1;
        } else if (q->valid) {
            *out = q->cmd;
        }
    }
    return st;
}

void cmd_queue_get_counters(const cmd_queue_t *q, cmd_queue_counters_t *out)
{
    if (q == NULL || out == NULL) {
        return;
    }
    *out = q->cnt;
}

uint32_t cmd_queue_age_ms(const cmd_queue_t *q, uint32_t now_ms)
{
    if (q == NULL || !q->valid) {
        return UINT32_MAX;
    }
    return now_ms - q->rx_ms;
}

const char *cmd_queue_state_name(cmd_queue_state_t st)
{
    /* ⚠️ 状态名只有这一处（控制台/日志/网页共用）—— 写两份就会有一份是错的（P-27） */
    switch (st) {
    case CMD_QUEUE_IDLE:  return "IDLE";
    case CMD_QUEUE_FRESH: return "FRESH";
    case CMD_QUEUE_HOLD:  return "HOLD";
    case CMD_QUEUE_RELAX: return "RELAX";
    default:              return "?";
    }
}
