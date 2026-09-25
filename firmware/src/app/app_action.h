/**
 * @file    app_action.h
 * @brief   把 `control/action`（姿态动画 / 动作层）接到固件上的应用层封装
 *
 * ## 这一层解决什么
 *
 * `control/action.{h,c}` 已经是**逐数值对照真版 `padog.py`（容差 0）验证过**的一层，
 * 但它刻意做成"不读时钟、不写寄存器、改别的模块状态的都做成显式效果表"。
 * 那些约定必须有人来兑现，本模块就是那个调用方：
 *
 * 1. **配置**：`action_cfg_defaults()` 是原版的默认值，本模块再用 `app_config` 的实际值
 *    覆盖它真正拥有的那几个字段（12 个中位角、`in_pit`/`in_rol`/`in_y`）。覆盖来源逐条写在
 *    `app_action_init()` 里；**没被覆盖的字段保持 action 层自己的默认值**。
 * 2. **效果**：`action_effects_t` 里那 7 种跨模块副作用**按顺序**交给
 *    `control_chain_cmd_*`（经 `app_chain` 的窄接口）施加 —— 顺序有意义
 *    （`action_stand()` 是先 `gesture(0,0,in_y)` 再 `gait(0)`，两者都写重心目标）。
 * 3. **时钟**：action 层的所有时间入口都要显式 `now_ms`，本模块从 `app_action_step()`
 *    的入参往下传（真机上是 `esp_timer_get_time()/1000`）。
 * 4. **掩码**：`action_wave_step()` 是**步进器**，只写 `ch_mask` 里的通道。
 *    本模块把没被置位的通道**填成上一次的输出**（原版那几路根本没被碰过，寄存器里
 *    留着的就是上一帧的值），于是调用方拿到的永远是完整的 12 路。
 *
 * ## 与 `motion` 的关系
 *
 * `MOTION_MODE_ACTION` 下运动任务每帧只调 `app_action_step()`，把角度**径直**下发
 * （不加速率限制：姿态/动作轨迹本身就是被 golden 钉住的验证过的产物）。
 * 动作没产生角度时**保持上一帧输出**，不松力 —— 松力只由 `estop` / `motion stop` /
 * 超时停车负责（单写者不变式，见 `motion.h`）。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control/action.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 可请求的动作。
 *
 * | 值 | 对应原实现 | 会不会写舵机 |
 * |---|---|---|
 * | `APP_ACTION_STAND` | `action_stand()` | 冻结时启动"坐→站"动画；否则 12 路直写中位角 |
 * | `APP_ACTION_SIT` / `APP_ACTION_SIT_DIRECT` | `action_sit()` = `action_sit_direct()` | 不写，启动"站→坐"动画（`hold_freeze=True`） |
 * | `APP_ACTION_WAVE` | `action_wave()` = `action_wave_direct()` | 由步进器一段一段写（掩码） |
 * | `APP_ACTION_INPLACE_STEP` | 网页的"步态测试"（`web_c.py:398` 写 `inplace_step_end_ms`）+ `mainloop` 881~888 | 本身不写；它下的是**命令层**的 TROT（见下） |
 * | `APP_ACTION_CRAWL` | `action_crawl()`（`padog.py:810~831`） | **不写** —— 它一个舵机都不碰，只改状态（爬行状态机在 `control_chain.c`） |
 *
 * ⚠️ `SIT` 与 `SIT_DIRECT` 在原实现里是同一个函数（`action_sit()` 只是一行别名），
 * 分成两个命令名只是为了和原函数名一一对应，行为完全一样。
 *
 * ⚠️ `CRAWL` 是**入口序列**，不是爬行本体：`action_crawl()` 在原版里被**故意**没搬进
 * `control/action.c`（爬行状态机整个在 `control_chain.c`，搬过去两边会抢
 * `crawl_phase`，见 `control/action.h`）。所以它在**本层**实现，只把
 * `padog.py:815~831` 那 14 行副作用按原顺序施加（含 `crawl_phase = 1` 与两个截止时刻），
 * 爬行随后由控制链自己推进。
 */
typedef enum {
    APP_ACTION_NONE = 0,
    APP_ACTION_STAND,
    APP_ACTION_SIT,
    APP_ACTION_SIT_DIRECT,
    APP_ACTION_WAVE,
    APP_ACTION_INPLACE_STEP,
    APP_ACTION_CRAWL,
} app_action_kind_t;

/** 状态快照（控制台 / 测试用） */
typedef struct {
    /** 当前（或最近一次）请求的种类 */
    app_action_kind_t kind;
    /** 还有请求没被服务（下一帧 `app_action_step()` 会处理它） */
    bool pending;
    /** 动作还在进行：挥手步进器在跑 / 姿态动画还没做完 / 原地踏步测试还没被取消 */
    bool busy;
    /** 挥手步进器还在跑 */
    bool wave_running;
    /** 姿态动画正在进行（`pose_anim_active`） */
    bool pose_anim_active;
    /** 姿态被冻结（`direct_pose_freeze`：坐姿做完之后为 true） */
    bool direct_pose_freeze;
    /** 挥手进度（`action_wave_t`） */
    int wave_phase;
    int wave_swing;
    /** 累计"产生了角度"的帧数 */
    uint32_t steps;
    /** 累计施加过的效果条数 —— 它是"副作用真的走了命令层"的证据 */
    uint32_t effects;
    /** 效果表装不下时被丢掉的次数。**非 0 就是 P-25 那一类错误** */
    uint32_t overflow;
} app_action_status_t;

/**
 * @brief 初始化：建 cfg / state，并置成"没有动作"。
 *
 * 必须在 `app_cfg_cmd_init()` 之后调用（要读配置）。**不会驱动舵机**。
 *
 * 覆盖来源（其余字段**保持 `action_cfg_defaults()` 的值**，逐条列在实现里）：
 * | 字段 | 来源 |
 * |---|---|
 * | `init[4][3]` | `app_config_t.servo_center`（腿序同是 腿1..腿4，直接对拷） |
 * | `in_pit` / `in_rol` / `in_y` | `app_config_t` 的同名字段 |
 */
esp_err_t app_action_init(void);

/**
 * @brief 请求一个动作。下一帧 `app_action_step()` 服务它。
 *
 * @note `APP_ACTION_INPLACE_STEP` 会按网页那条命令（`web_c.py:398`）
 *       先把 `inplace_step_end_ms` 设成 `now + APP_ACTION_INPLACE_STEP_MS`，
 *       于是下一帧真的走到 `mainloop` 881~888 那条分支（否则那个量是 0，
 *       整个服务是空转 —— 原始来源是网页命令，不是动作层）。
 */
esp_err_t app_action_request(app_action_kind_t kind);

/**
 * @brief 取消正在进行的动作：清请求、清挥手脚本、复位姿态动画状态。
 *
 * @note **本函数不写舵机**（单写者不变式：松力是 `motion stop` / `estop` 的事）。
 * @note 会顺带 `move(0, 0, 0)` 把"原地踏步测试"下过的行走命令收回来 ——
 *       否则取消动作之后命令层还留着 `spd=3`，下次切回 CHAIN 会突然走起来。
 */
void app_action_stop(void);

/**
 * @brief 推进一帧。
 *
 * @param now_ms 当前时刻（毫秒，真机上是 `esp_timer_get_time()/1000`）
 * @param out    输出：**完整的 12 路**逻辑通道角度。只有返回 true 时才被写。
 * @return true  = 本帧产生了角度（调用方直接下发，不限速）
 * @return false = 本帧没有动作要输出，调用方**保持上一帧输出**（不要松力）
 */
bool app_action_step(int64_t now_ms, float out[ACTION_CHANNELS]);

/** @brief 读状态快照（线程安全） */
void app_action_get_status(app_action_status_t *out);

/**
 * @brief 只读地取本层的 cfg（测试用它逐字段对照"覆盖来源"；**不要改**）。
 * @return 从未 `app_action_init()` 过时返回 NULL
 */
const action_cfg_t *app_action_get_cfg(void);

/**
 * @brief 动作种类名（日志用，未知返回 "?"） */
const char *app_action_kind_name(app_action_kind_t kind);

/* ==================================================================== */
/*  `inplace_step_end_ms` 的读写（宿主测试 / 协议层用）                   */
/* ==================================================================== */

/**
 * @brief 读回 `inplace_step_end_ms`（毫秒，0 = 没有"原地步态测试"在排队）。
 *
 * 它是 `action_state_t` 的字段，`app_action_status_t` 里没有 ⇒ 单独开一个 getter：
 * "`action_crawl()` 把它清零"这件事**只能靠读回来证明**（P-25：只断言别的量没变，
 * 是抓不到一个被吞掉的副作用的）。
 */
int32_t app_action_get_inplace_step_end_ms(void);

/**
 * @brief 直接写 `inplace_step_end_ms`（复刻原版**网络层**那一步）。
 *
 * ```python
 * elif value == 'is':                                  # web_c.py:394~398
 *     padog.gait(0)
 *     padog.inplace_step_end_ms = utime.ticks_add(utime.ticks_ms(), 5000)
 * ```
 * 原版这个全局是**网络层直接写**的；C 版平时由 `APP_ACTION_INPLACE_STEP` 代劳，
 * 但那一条路径**下一帧就会被 `move(3,1,1)` 清掉**（原版行为，见 `control/action.h`
 * 第 4 条）⇒ 想摆一个"非 0 的前置值"就只剩这条窄接口。
 *
 * @note 宿主测试用它制造非 0 初值（P-18：默认值是 0 时，"清零"这件事测不出真假）。
 */
esp_err_t app_action_set_inplace_step_end_ms(int32_t end_ms);

#ifdef __cplusplus
}
#endif
