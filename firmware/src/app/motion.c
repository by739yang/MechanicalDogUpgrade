/**
 * @file    motion.c
 * @brief   P2：固定周期运动任务实现
 */

#include "app/motion.h"

#include <math.h>
#include <string.h>

#include "app/app_chain.h"
#include "app/app_action.h"
#include "app/app_cfg_cmd.h"
#include "app/servo_out.h"
#include "bsp/bsp_i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "motion";

/** 控制任务栈大小（字节）。里面只有浮点运算与日志，4 KB 足够。 */
#define MOTION_TASK_STACK   4096
/** 控制任务优先级：高于控制台（5），低于 WiFi/系统任务 */
#define MOTION_TASK_PRIO    10
/** 把控制任务钉在 core 1，减少与 WiFi/控制台的相互干扰 */
#define MOTION_TASK_CORE    1

typedef struct {
    float target[SERVO_MAP_CHANNELS];
    float current[SERVO_MAP_CHANNELS];
    bool  target_valid;      /**< 是否已经有人给过目标 */
} motion_state_t;

static SemaphoreHandle_t s_mutex = NULL;      /**< 保护下面的状态 */
static TaskHandle_t      s_task  = NULL;
static volatile bool     s_run   = false;     /**< 任务是否应当继续 */
static volatile bool     s_estop = false;     /**< 急停请求 */
static volatile int64_t  s_last_cmd_us = 0;   /**< 上次命令时间（心跳） */

static motion_state_t s_state;
static uint32_t s_period_ms  = MOTION_DEFAULT_PERIOD_MS;
static float    s_rate_dps   = MOTION_DEFAULT_RATE_DPS;
static uint32_t s_timeout_ms = MOTION_DEFAULT_TIMEOUT_MS;
static uint32_t s_mode       = MOTION_MODE_POSE;

/* 统计（任务写、控制台读，用 s_mutex 保护） */
static motion_stats_t s_stats;

/*
 * ACTION 模式的"上一帧输出"。不能借 POSE 的 `s_state.current`：
 * 任务体开头会把 `current` 清零（`target_valid` 为假时），而宿主桩里
 * `vTaskDelayUntil()` 是 longjmp 让出点、任务体每次被驱动都从头进一次 —— 借用它的话
 * 每个"动作没产生角度"的帧都会被填成 0°，把 0° 打给舵机。见 host_stubs/host_sim.c。
 */
static float s_action_last[SERVO_MAP_CHANNELS];
/** ⚠️ `volatile`：控制台（`motion_stop()` / `motion_estop()`）会把它清掉，
 *  只有运动任务会写它旁边的数组 —— 它只是一个"缓存还有效吗"的提示位 */
static volatile bool s_action_last_valid = false;

static void stats_reset(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.period_min_us = INT64_MAX;
    s_stats.period_ms     = (float)s_period_ms;
    s_stats.rate_dps      = s_rate_dps;
    s_stats.timeout_ms    = s_timeout_ms;
    s_stats.mode          = s_mode;
}

const char *motion_mode_name(uint32_t mode)
{
    switch (mode) {
    case MOTION_MODE_CHAIN:  return "CHAIN";
    case MOTION_MODE_ACTION: return "ACTION";
    case MOTION_MODE_POSE:
    default:                 return "POSE";
    }
}

esp_err_t motion_set_mode(uint32_t mode)
{
    if (mode != MOTION_MODE_POSE && mode != MOTION_MODE_CHAIN &&
        mode != MOTION_MODE_ACTION) {
        return ESP_ERR_INVALID_ARG;
    }
    s_mode = mode;
    if (s_mutex != NULL && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_stats.mode = mode;
        xSemaphoreGive(s_mutex);
    }
    ESP_LOGI(TAG, "控制模式 = %s (%s)", motion_mode_name(mode),
             (mode == MOTION_MODE_CHAIN)  ? "控制链/步态"
             : (mode == MOTION_MODE_ACTION) ? "姿态动画/动作层"
                                            : "直接 12 路角度");
    return ESP_OK;
}

uint32_t motion_get_mode(void)
{
    return s_mode;
}

const char *motion_stop_reason_str(uint32_t reason)
{
    switch (reason) {
    case MOTION_STOP_NONE:    return "无（还在运行）";
    case MOTION_STOP_USER:    return "用户停止";
    case MOTION_STOP_ESTOP:   return "急停";
    case MOTION_STOP_TIMEOUT: return "命令超时";
    case MOTION_STOP_ERROR:   return "内部错误";
    default:                  return "?";
    }
}

/* ==========================================================================
 * 目标 / 当前角度
 * ========================================================================== */

static float clamp_angle(float a)
{
    if (a > 180.0f) {
        return 180.0f;
    }
    if (a < 0.0f) {
        return 0.0f;
    }
    return a;
}

esp_err_t motion_set_target(const float deg[SERVO_MAP_CHANNELS])
{
    if (deg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* "直接给 12 路角度"本身就是 POSE 语义 —— 隐式切回 POSE，避免
     * 在 CHAIN 模式下设了目标却看不到任何效果那种**静默无效**。 */
    if (s_mode != MOTION_MODE_POSE) {
        ESP_LOGW(TAG, "收到直接角度目标，控制模式自动从 %s 切回 POSE", motion_mode_name(s_mode));
        s_mode = MOTION_MODE_POSE;
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_stats.mode = s_mode;
            xSemaphoreGive(s_mutex);
        }
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
        s_state.target[ch] = clamp_angle(deg[ch]);
    }
    s_state.target_valid = true;
    xSemaphoreGive(s_mutex);

    motion_keepalive();
    return ESP_OK;
}

esp_err_t motion_set_target_stand(void)
{
    const app_config_t *cfg = app_cfg_cmd_get();
    if (cfg == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    servo_map_input_t in;
    memset(&in, 0, sizeof(in));
    in.ik_path = false;             /* -> servo_output 的 else 分支：直接站姿 */
    in.shank_bias = servo_map_shank_bias(cfg->l1, cfg->l2, cfg->leg_len_ref);
    for (int leg = 0; leg < SERVO_MAP_LEGS; ++leg) {
        for (int j = 0; j < SERVO_MAP_JOINTS; ++j) {
            in.init[leg][j] = cfg->servo_center[leg][j];
        }
    }

    float deg[SERVO_MAP_CHANNELS];
    servo_map_legs_to_angles(&in, deg);
    return motion_set_target(deg);
}

void motion_get_stats(motion_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    if (s_mutex == NULL) {
        memset(out, 0, sizeof(*out));
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        memset(out, 0, sizeof(*out));
        return;
    }
    *out = s_stats;
    out->running  = s_run;
    out->estopped = s_estop;
    memcpy(out->target, s_state.target, sizeof(out->target));
    memcpy(out->current, s_state.current, sizeof(out->current));
    xSemaphoreGive(s_mutex);
}

/* ==========================================================================
 * 参数
 * ========================================================================== */

esp_err_t motion_set_period_ms(uint32_t ms)
{
    if (ms < 1 || ms > 1000) {
        return ESP_ERR_INVALID_ARG;
    }
    s_period_ms = ms;
    if (s_mutex != NULL && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_stats.period_ms = (float)ms;
        xSemaphoreGive(s_mutex);
    }
    return ESP_OK;
}

esp_err_t motion_set_rate_dps(float dps)
{
    if (dps < 1.0f || dps > 2000.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    s_rate_dps = dps;
    if (s_mutex != NULL && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_stats.rate_dps = dps;
        xSemaphoreGive(s_mutex);
    }
    return ESP_OK;
}

esp_err_t motion_set_timeout_ms(uint32_t ms)
{
    if (ms > 3600000u) {
        return ESP_ERR_INVALID_ARG;
    }
    s_timeout_ms = ms;
    if (s_mutex != NULL && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_stats.timeout_ms = ms;
        xSemaphoreGive(s_mutex);
    }
    motion_keepalive();
    return ESP_OK;
}

bool motion_is_running(void)
{
    return s_run;
}

void motion_keepalive(void)
{
    s_last_cmd_us = esp_timer_get_time();
}

void motion_estop(const char *why)
{
    ESP_LOGE(TAG, "*** 急停 *** %s", (why != NULL) ? why : "");

    /* 急停之后不保留"上一帧动作输出"：重启后不会把上一次动作的姿态重新通电 */
    s_action_last_valid = false;

    if (s_run) {
        /* 任务在跑 -> 只置标志，由任务在下一个周期内松力并退出（单写者不变式） */
        s_estop = true;
        return;
    }

    /* 任务没在跑 -> 当前任务可以直接写 */
    const esp_err_t err = servo_out_all_off();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "急停松力时有通道写失败: %s", esp_err_to_name(err));
    }
    if (s_mutex != NULL && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_stats.estopped   = true;
        s_stats.stop_reason = MOTION_STOP_ESTOP;
        s_stats.running    = false;
        xSemaphoreGive(s_mutex);
    }
}

/* ==========================================================================
 * 控制任务
 * ========================================================================== */

/** 按速率上限把 current 朝 target 推进一步 */
static bool step_towards(float *current, const float *target, float max_step)
{
    bool all_done = true;

    for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
        const float diff = target[ch] - current[ch];
        if (fabsf(diff) <= max_step) {
            current[ch] = target[ch];
            continue;
        }
        current[ch] += (diff > 0.0f) ? max_step : -max_step;
        all_done = false;
    }
    return all_done;
}

static void log_stats_line(void)
{
    motion_stats_t st;
    motion_get_stats(&st);

    ESP_LOGI(TAG,
             "统计: ticks=%u 模式=%s 周期 us(avg/max)=%lld/%lld 抖动max=%lld us "
             "干活 us(avg/max)=%lld/%lld 超期=%u I2C写=%u 链帧=%u 动作帧=%u 已到位=%s",
             (unsigned)st.ticks,
             motion_mode_name(st.mode),
             (long long)st.period_avg_us, (long long)st.period_max_us,
             (long long)st.jitter_max_us,
             (long long)st.work_avg_us, (long long)st.work_max_us,
             (unsigned)st.overruns,
             (unsigned)st.i2c_writes, (unsigned)st.chain_frames,
             (unsigned)st.action_frames,
             st.settled ? "是" : "否");
}

static void motion_task(void *arg)
{
    (void)arg;

    TickType_t last_wake   = xTaskGetTickCount();
    int64_t    last_us     = esp_timer_get_time();
    int64_t    sum_period  = 0;
    int64_t    sum_work    = 0;
    int64_t    next_log_us = last_us + (int64_t)MOTION_STATS_LOG_MS * 1000;

    ESP_LOGI(TAG, "控制任务启动: 周期 %u ms (%.0f Hz), 速率上限 %.0f °/s, 超时 %u ms%s",
             (unsigned)s_period_ms, 1000.0f / (float)s_period_ms,
             (double)s_rate_dps, (unsigned)s_timeout_ms,
             (s_timeout_ms == 0) ? "（已关闭）" : "");

    /* 起始点 = 硬件当前姿态（上电时是无脉冲，但这里只关心角度缓存） */
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        for (uint8_t ch = 0; ch < SERVO_MAP_CHANNELS; ++ch) {
            if (!s_state.target_valid) {
                s_state.current[ch] = 0.0f;
            }
        }
        xSemaphoreGive(s_mutex);
    }

    motion_keepalive();

    while (s_run) {
        /* 每帧重读周期，这样 motion_set_period_ms() 下一个周期就生效 */
        const TickType_t period_ticks = pdMS_TO_TICKS(s_period_ms);
        const int64_t    nominal_us   = (int64_t)s_period_ms * 1000;

        /* ---- 1. 急停优先，最高优先级处理 ---- */
        if (s_estop) {
            ESP_LOGE(TAG, "执行急停：12 路松力");
            (void)servo_out_all_off();
            if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
                s_stats.estopped    = true;
                s_stats.stop_reason = MOTION_STOP_ESTOP;
                xSemaphoreGive(s_mutex);
            }
            s_run = false;
            break;
        }

        /* ---- 2. 命令超时 ---- */
        if (s_timeout_ms > 0) {
            const int64_t idle_us = esp_timer_get_time() - s_last_cmd_us;
            if (idle_us > (int64_t)s_timeout_ms * 1000) {
                ESP_LOGW(TAG, "命令超时（%lld ms > %u ms）：松力停车",
                         (long long)(idle_us / 1000), (unsigned)s_timeout_ms);
                (void)servo_out_all_off();
                if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
                    s_stats.stop_reason = MOTION_STOP_TIMEOUT;
                    xSemaphoreGive(s_mutex);
                }
                s_run = false;
                break;
            }
        }

        /* ---- 3. 一帧限速推进 / 控制链推进 ---- */
        const int64_t now_us = esp_timer_get_time();
        int64_t       dt_us  = now_us - last_us;   /* 距上一帧开始的整周期 */
        last_us = now_us;

        /* dt 异常保护：首次循环、或长时间被抢占（比如日志阻塞）时不要一次走太远 */
        if (dt_us <= 0) {
            dt_us = nominal_us;
        }
        if (dt_us > 200000) {
            dt_us = 200000;
        }
        const float max_step = s_rate_dps * ((float)dt_us / 1000000.0f);

        float snapshot[SERVO_MAP_CHANNELS];
        bool  settled  = true;
        bool  have_out = true;   /**< false = 本帧没有任何角度可下发（ACTION 模式才有） */

        if (s_mode == MOTION_MODE_CHAIN) {
            /*
             * CHAIN：角度由控制链给。`app_chain_step()` 内部按自己的节拍
             * （默认 65 ms）推进；没到节拍时它把上一帧的角度原样还回来，
             * 而 `servo_out` 只写变化的通道 ⇒ 这些重复帧的 I2C 开销是 0。
             *
             * ⚠️ 这里**刻意不做速率限制** —— 步态轨迹是被 golden 逐位验证过的，
             * 限速就改了轨迹。见 motion.h 的 MOTION_MODE_CHAIN 说明。
             */
            const bool advanced = app_chain_step(now_us / 1000, snapshot);
            if (advanced) {
                if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
                    ++s_stats.chain_frames;
                    xSemaphoreGive(s_mutex);
                }
            }
            settled = true;   /* 链模式下没有"是否到位"这个概念 */
        } else if (s_mode == MOTION_MODE_ACTION) {
            /*
             * ACTION：角度由姿态动画 / 动作层给。⚠️ 这里**刻意不做速率限制**，
             * 理由和 CHAIN 一样：姿态表、混合插值、挥手脚本的时序都是被 golden
             * 逐数值钉住的验证过的产物，限速就改了它（见 motion.h）。
             *
             * 动作没产生角度的帧（原版在 `time.sleep_ms()` 里、动画已做完、
             * 直写站姿之后的稳态）**保持上一帧输出**，不松力 —— 松力只归
             * `estop` / `motion stop` / 超时停车。
             */
            float act[SERVO_MAP_CHANNELS];
            const bool produced = app_action_step(now_us / 1000, act);

            if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
                if (produced) {
                    /* 自己留一份"上一帧输出"：POSE 那份 `s_state.current` 在任务体开头会被
                     * 清零（宿主桩里 `vTaskDelayUntil()` 是 longjmp 让出点，任务体每次被驱动
                     * 都从头进一次；真机上那段是死循环、不会重入，但"动作模式沿用上一帧输出"
                     * 本来就不该搭在 POSE 的限速缓存上）。 */
                    memcpy(s_action_last, act, sizeof(s_action_last));
                    s_action_last_valid = true;
                    memcpy(s_state.current, act, sizeof(s_state.current));
                    ++s_stats.action_frames;
                }
                if (s_action_last_valid) {
                    memcpy(snapshot, s_action_last, sizeof(snapshot));
                } else {
                    /* 还没有任何动作输出过：什么都不下发（也就不会把 0° 打给舵机） */
                    have_out = false;
                }
                xSemaphoreGive(s_mutex);
            } else {
                have_out = false;
            }
            settled = true;   /* 动作模式下同样没有"是否到位"这个概念 */
        } else {
            if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
                if (s_state.target_valid) {
                    settled = step_towards(s_state.current, s_state.target, max_step);
                }
                memcpy(snapshot, s_state.current, sizeof(snapshot));
                xSemaphoreGive(s_mutex);
            } else {
                memset(snapshot, 0, sizeof(snapshot));
            }
        }

        /* ---- 4. 输出（servo_out 内部只写变化的通道） ---- */
        if (have_out) {
            const esp_err_t err = servo_out_apply_deg(snapshot);
            if (err != ESP_OK) {
                /* 单通道写失败已在 servo_out 里记日志，这里不中断控制循环 */
                ESP_LOGW(TAG, "本帧有通道写入失败（%s），已跳过并在下一帧重试",
                         esp_err_to_name(err));
            }
        }

        /* ---- 5. 统计 ---- */
        const int64_t work_us = esp_timer_get_time() - now_us;
        sum_period += dt_us;
        sum_work   += work_us;

        if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
            ++s_stats.ticks;
            s_stats.period_last_us = dt_us;
            if (dt_us < s_stats.period_min_us) {
                s_stats.period_min_us = dt_us;
            }
            if (dt_us > s_stats.period_max_us) {
                s_stats.period_max_us = dt_us;
            }
            const int64_t jitter = (dt_us > nominal_us) ? (dt_us - nominal_us)
                                                        : (nominal_us - dt_us);
            if (jitter > s_stats.jitter_max_us) {
                s_stats.jitter_max_us = jitter;
            }
            if (dt_us > nominal_us + nominal_us / 2) {
                ++s_stats.overruns;
            }
            s_stats.work_last_us = work_us;
            if (work_us > s_stats.work_max_us) {
                s_stats.work_max_us = work_us;
            }
            if (s_stats.ticks > 0) {
                s_stats.period_avg_us = sum_period / (int64_t)s_stats.ticks;
                s_stats.work_avg_us   = sum_work / (int64_t)s_stats.ticks;
            }
            s_stats.settled    = settled;
            s_stats.i2c_writes = servo_out_write_count();
            xSemaphoreGive(s_mutex);
        }

        /* ---- 6. 周期统计日志：这是"10 分钟不重启"的证据 ---- */
        if (now_us >= next_log_us) {
            log_stats_line();
            next_log_us = now_us + (int64_t)MOTION_STATS_LOG_MS * 1000;
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }

    /* ---- 退出：确保舵机是松力状态 ---- */
    (void)servo_out_all_off();

    if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        s_stats.running = false;
        s_stats.settled = true;
        xSemaphoreGive(s_mutex);
    }

    ESP_LOGW(TAG, "控制任务结束，原因: %s", motion_stop_reason_str(s_stats.stop_reason));

    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t motion_init(void)
{
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == NULL) {
            ESP_LOGE(TAG, "创建状态互斥锁失败");
            return ESP_ERR_NO_MEM;
        }
    }

    memset(&s_state, 0, sizeof(s_state));
    memset(s_action_last, 0, sizeof(s_action_last));
    s_action_last_valid = false;
    stats_reset();
    s_run   = false;
    s_estop = false;

    ESP_LOGI(TAG, "运动模块就绪（未启动）。默认周期 %u ms、速率 %.0f °/s、超时 %u ms",
             (unsigned)MOTION_DEFAULT_PERIOD_MS, (double)MOTION_DEFAULT_RATE_DPS,
             (unsigned)MOTION_DEFAULT_TIMEOUT_MS);
    return ESP_OK;
}

esp_err_t motion_start(void)
{
    if (s_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_run) {
        ESP_LOGW(TAG, "控制任务已在运行");
        return ESP_OK;
    }

    s_estop = false;

    /* 保留已有目标，但把统计清零重新计 */
    uint32_t keep_reason = s_stats.stop_reason;
    float    target[SERVO_MAP_CHANNELS];
    bool     target_valid;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    memcpy(target, s_state.target, sizeof(target));
    target_valid = s_state.target_valid;
    xSemaphoreGive(s_mutex);

    stats_reset();
    s_stats.stop_reason = keep_reason;
    s_stats.running     = true;
    s_run               = true;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        memcpy(s_state.target, target, sizeof(target));
        s_state.target_valid = target_valid;
        xSemaphoreGive(s_mutex);
    }

    motion_keepalive();

    if (xTaskCreatePinnedToCore(motion_task, "motion", MOTION_TASK_STACK, NULL,
                                MOTION_TASK_PRIO, &s_task, MOTION_TASK_CORE) != pdPASS) {
        s_run = false;
        ESP_LOGE(TAG, "创建控制任务失败（内存不足？）");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void motion_stop(uint32_t reason)
{
    if (!s_run) {
        /* 没在跑也要保证松力 */
        s_action_last_valid = false;
        (void)servo_out_all_off();
        if (s_mutex != NULL && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_stats.stop_reason = reason;
            s_stats.running     = false;
            xSemaphoreGive(s_mutex);
        }
        return;
    }

    if (s_mutex != NULL && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_stats.stop_reason = reason;
        xSemaphoreGive(s_mutex);
    }

    /* 停车就忘记"上一帧动作输出"：重启后不会把上一次动作的姿态重新通电，
     * 必须重新下 `action ...` 命令（松力态保持到那时为止） */
    s_action_last_valid = false;

    s_run = false;

    /* 等任务自己收尾（它会做 all_off 并 vTaskDelete） */
    for (int i = 0; i < 100 && s_task != NULL; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_task != NULL) {
        ESP_LOGW(TAG, "控制任务未在 1 秒内退出");
    }
}
