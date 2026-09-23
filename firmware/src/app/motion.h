/**
 * @file    motion.h
 * @brief   P2：固定周期运动任务 —— 12 路舵机的限速输出、急停与超时停车
 *
 * ## 这一阶段要达成什么（来自迁移表 P2）
 *
 * | 要求 | 实现方式 |
 * |---|---|
 * | 固定周期运行 | FreeRTOS 任务 + `vTaskDelayUntil()`，默认 **100 Hz（10 ms）** |
 * | 12 路舵机映射 | `servo_map.c`（已用 golden 对照验证） |
 * | 角度限幅 | 目标与当前角都夹到 `[0, 180]`；占空比再由 `servo_map` 夹一次 |
 * | 速率限制 | 每帧最多移动 `rate_dps × dt`，避免目标突变把舵机"甩"过去 |
 * | 急停 | `motion_estop()`：≤1 个周期内 12 路松力并停任务 |
 * | 超时停车 | 超过 `timeout_ms` 没有命令就松力停车 |
 * | 站立 10 分钟不重启 | 有周期性统计日志（ticks / 周期抖动 / I2C 写次数）可作证据 |
 *
 * ## 单写者不变式（重要）
 *
 * **运动任务在跑的时候，只有它写舵机。** 控制台任务不允许直接写 PWM，
 * 否则两个任务会交错写同一条 I2C（迁移表 §8.5）。
 * 因此：
 *   - 控制台的 `lg` / `set` / `deg` / `all` / `off` 等命令在运动任务运行时**拒绝执行**；
 *   - `motion_estop()` 只**置标志**，由运动任务自己完成"松力 + 退出"；
 *   - 任务没在跑时，控制台才可以自由写。
 *
 * ## 为什么超时后是"松力"而不是"保持站姿"
 *
 * 本机**没有 IMU**，P2 是纯开环。超时意味着"没人指挥了"，此时：
 *   - 松力：狗趴下。安全，电流小。
 *   - 保持：舵机持续对抗重力，**堵转电流大、发热**，且没有任何反馈能保证它站得住。
 * 所以默认松力。这个取舍在 P4 有了 IMU 闭环之后才可能改。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control/servo_map.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 默认控制周期（毫秒）—— 100 Hz */
#define MOTION_DEFAULT_PERIOD_MS   10u
/** 默认速率上限（度/秒）—— 舵机空载约 300~600 °/s，取一个保守值 */
#define MOTION_DEFAULT_RATE_DPS    120.0f
/** 默认超时（毫秒）—— 这么久没有命令就松力停车 */
#define MOTION_DEFAULT_TIMEOUT_MS  10000u
/** 周期性统计日志的间隔（毫秒） */
#define MOTION_STATS_LOG_MS        10000u

/** 周期统计快照 */
typedef struct {
    bool     running;          /**< 任务是否在跑 */
    bool     estopped;         /**< 是否因为急停而停止 */
    bool     settled;          /**< 12 路是否都已到达目标 */
    uint32_t ticks;            /**< 循环次数 */
    uint32_t overruns;         /**< 实测周期超过标称周期的次数 */
    int64_t  period_last_us;   /**< 最近一次实测周期 */
    int64_t  period_min_us;    /**< 实测最短周期 */
    int64_t  period_max_us;    /**< 实测最长周期 */
    int64_t  period_avg_us;    /**< 实测平均周期 */
    int64_t  jitter_max_us;    /**< |实测周期 - 标称周期| 的最大值 */
    int64_t  work_last_us;     /**< 最近一帧的"干活"耗时（限速计算 + I2C 写 + 统计） */
    int64_t  work_max_us;      /**< 干活耗时的最大值 —— 离标称周期还有多少余量看这个 */
    int64_t  work_avg_us;      /**< 干活耗时的平均值 */
    uint32_t i2c_writes;       /**< 累计通道写次数 */
    uint32_t stop_reason;      /**< 停止原因，见 MOTION_STOP_* */
    float    period_ms;        /**< 当前标称周期 */
    float    rate_dps;         /**< 当前速率上限 */
    uint32_t timeout_ms;       /**< 当前超时（0 = 关闭） */
    float    target[SERVO_MAP_CHANNELS];  /**< 目标角度 */
    float    current[SERVO_MAP_CHANNELS]; /**< 当前（限速后）角度 */
} motion_stats_t;

/** 停止原因 */
#define MOTION_STOP_NONE     0u  /**< 从没停过 / 还在跑 */
#define MOTION_STOP_USER     1u  /**< motion_stop() */
#define MOTION_STOP_ESTOP    2u  /**< motion_estop() */
#define MOTION_STOP_TIMEOUT  3u  /**< 命令超时 */
#define MOTION_STOP_ERROR    4u  /**< 控制任务内部错误 */

/** @brief 建锁与初始化（不启动任务）。必须在 I2C 与 PCA9685 初始化之后调用。 */
esp_err_t motion_init(void);

/**
 * @brief 启动固定周期控制任务。
 * @note 重复调用是安全的（已经在跑就直接返回 ESP_OK）。
 */
esp_err_t motion_start(void);

/**
 * @brief 停止控制任务，并把 12 路置为无脉冲（松力）。
 * @param reason  记入统计的停止原因
 */
void motion_stop(uint32_t reason);

/**
 * @brief 急停：置标志，由运动任务在**下一个周期内**完成"松力 + 退出"。
 *
 * @param why  原因字符串，只用于日志（可为 NULL）
 *
 * @note 如果任务没在跑，本函数会**直接**在当前任务里松力。
 * @note 刻意不在本函数里做 I2C 写 —— 见头文件的单写者不变式。
 */
void motion_estop(const char *why);

/** @brief 刷新"命令心跳"，重置超时计时。任何来自用户的动作都该调它。 */
void motion_keepalive(void);

/** @brief 任务是否在跑 */
bool motion_is_running(void);

/**
 * @brief 设置 12 路目标角度（线程安全）。
 * @param deg  逻辑通道 0..11 的角度；会被夹到 [0, 180]
 * @note 会同时刷新心跳。`current` 由任务按速率上限逐步逼向目标。
 */
esp_err_t motion_set_target(const float deg[SERVO_MAP_CHANNELS]);

/**
 * @brief 目标设为"直接站姿"（`servo_output` 的 else 分支：12 路 = 中位角）。
 *
 * 这是 P2 唯一一个**有物理意义**的姿态，也是"站立 10 分钟"测试用的目标。
 */
esp_err_t motion_set_target_stand(void);

/** @brief 读统计快照（线程安全） */
void motion_get_stats(motion_stats_t *out);

/** @brief 设置控制周期（毫秒）。1..1000。**下一个控制周期即生效**，无需重启任务。 */
esp_err_t motion_set_period_ms(uint32_t ms);

/** @brief 设置速率上限（度/秒）。1..2000。 */
esp_err_t motion_set_rate_dps(float dps);

/** @brief 设置命令超时（毫秒）。0 = 关闭超时。 */
esp_err_t motion_set_timeout_ms(uint32_t ms);

/** @brief 停止原因的字符串（日志用） */
const char *motion_stop_reason_str(uint32_t reason);

#ifdef __cplusplus
}
#endif
