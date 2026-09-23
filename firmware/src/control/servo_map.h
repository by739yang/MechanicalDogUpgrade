/**
 * @file    servo_map.h
 * @brief   关节角 -> 12 路舵机输出（P2 的映射层，纯 C，可在电脑上测试）
 *
 * ## 它复刻的是哪两段原代码
 *
 * 1. `PA_SERVO.py` 的 `Servos.position()` + `PCA9685.duty()`：
 *    角度 -> 占空比计数 -> 寄存器里的 (ON, OFF)。
 *    ⚠️ 注意占空比是 **4095 满量程**（不是 4096），这是刻意的，见驱动头文件注释。
 *
 * 2. `padog.py` 的 `servo_output()`：
 *    逻辑关节角（髋/大腿/小腿 × 4 腿）-> 12 个舵机角度。
 *    这里含原实现那些**不对称**的 ±90 与正负号，是本模块最容易抄错的部分。
 *
 * ## 逻辑通道编号（原实现的约定，**不要"整理"**）
 *
 * | 逻辑通道 | 腿 | 关节 | 硬件 |
 * |---|---|---|---|
 * | 0,1,2 | 腿1 左前 | 髋 / 大腿 / 小腿 | 0x40 ch0,1,2 |
 * | 3,4,5 | 腿4 左后 | 髋 / 大腿 / 小腿 | 0x40 ch3,4,5 |
 * | 6,7,8 | 腿2 右前 | 髋 / 大腿 / 小腿 | 0x41 ch0,1,2 |
 * | 9,10,11 | 腿3 右后 | 髋 / 大腿 / 小腿 | 0x41 ch3,4,5 |
 *
 * **腿部编号不是物理顺序**（腿3 是右后、腿4 是左后），通道编号则按"左前/左后"排在
 * 0x40、"右前/右后"排在 0x41。两者都不直观，所以只用这一张表，不许在别处再写一遍。
 *
 * ## 可测试性
 *
 * 本文件**不依赖 ESP-IDF、不碰 I2C** —— 只做算术。真正的"写寄存器"由调用方
 * （`servo_out.c`）完成。宿主机上可以直接用 gcc 编译，与从原 MicroPython
 * exec 出来的输出逐位对照（见 `tools/golden/test_servo_map.c`）。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 四足逻辑通道数 */
#define SERVO_MAP_CHANNELS 12
/** 每条腿的关节数 */
#define SERVO_MAP_JOINTS   3
/** 腿部数量 */
#define SERVO_MAP_LEGS     4

/**
 * 两片 PCA9685 的 I2C 地址。
 *
 * ⚠️ 这里**故意不 include `drivers/drv_pca9685.h`** —— 那个头文件会拉进
 * `esp_err.h`，本模块就再也不能用宿主 gcc 编译、也就失去了 golden 对照能力。
 * 代价是这两个常量与驱动头文件重复；由 `servo_out.c` 里的 `_Static_assert`
 * 保证两者一致（编译期检查，不会漂移）。
 */
#define SERVO_MAP_ADDR_LEFT   0x40u
#define SERVO_MAP_ADDR_RIGHT  0x41u

/* ---------------- 角度 -> 占空比（复刻 Servos.position） ---------------- */

/** 舵机频率（Hz），与 PA_SERVO.Servos 默认一致 */
#define SERVO_MAP_FREQ_HZ     50.0f
/** 一个周期的微秒数（1e6 / 50 = 20000） */
#define SERVO_MAP_PERIOD_US   20000u
/** 角度量程（PA_SERVO.Servos 的 degrees=180） */
#define SERVO_MAP_DEGREES     180.0f
/** 脉宽安全范围（µs） */
#define SERVO_MAP_US_MIN      500u
#define SERVO_MAP_US_MAX      2500u
/** 占空比满量程：**4095**（复刻原实现，不是 4096） */
#define SERVO_MAP_DUTY_FULL   4095u

/**
 * @brief `int(4095 * us / period)` —— 复刻 `Servos._us2duty()`。
 * @note 两个满量程常量：`min_duty = 102`、`max_duty = 511`（50 Hz）。
 */
uint16_t servo_map_min_duty(void);
uint16_t servo_map_max_duty(void);

/**
 * @brief 角度 -> 占空比计数，复刻 `Servos.position(index, degrees)`。
 *
 * ```python
 * duty = self.min_duty + span * degrees / self.degrees   # span = max_duty - min_duty
 * duty = min(self.max_duty, max(self.min_duty, int(duty)))
 * ```
 *
 * 注意顺序：**先 `int()` 截断，再限幅**。角度越界不会报错，只会被限幅到
 * `[min_duty, max_duty]` —— 这一点原实现如此，C 版保持。
 */
uint16_t servo_map_deg_to_duty(float degrees);

/**
 * @brief 占空比 -> 寄存器里的 (ON, OFF)，复刻 `PCA9685.duty()`。
 *
 * ```python
 * if value == 0:        pwm(index, 0, 4096)   # 整周期全关 = 无脉冲 = 舵机松力
 * elif value == 4095:   pwm(index, 4096, 0)
 * else:                 pwm(index, 0, value)
 * ```
 *
 * @param duty  0..4095
 */
void servo_map_duty_to_pwm(uint16_t duty, uint16_t *on, uint16_t *off);

/* ---------------- 硬件位置 ---------------- */

/** 一个逻辑通道对应的硬件位置 */
typedef struct {
    uint8_t board_addr; /**< `SERVO_MAP_ADDR_LEFT`(0x40) 或 `SERVO_MAP_ADDR_RIGHT`(0x41) */
    uint8_t pca_ch;     /**< 该板上的通道 0..5 */
} servo_map_hw_t;

/**
 * @brief 逻辑通道(0..11) -> 硬件位置。越界返回 NULL。
 */
const servo_map_hw_t *servo_map_hw(uint8_t logical_ch);

/* ---------------- 小腿舵机二次拟合曲线 ---------------- */

/**
 * @brief 复刻 `padog._shank_ik_bias()`。
 *
 * 大狗在同一 Hc 下 IK 小腿角比小机低 20°+，进曲线前先加这个偏置：
 *   `extra = max(0, (l1 + l2) - leg_len_ref)`，`bias = extra * 0.375`
 * 实测值：`(130+138-149) * 0.375 = 44.625`。
 *
 * ⚠️ 原实现还会先看模块级 `shank_ik_bias_deg`，但 `config_s.py` 里没有这个变量，
 *    所以走的是 per_mm 这条路。C 版只实现这条（并把这事实写在这里）。
 */
float servo_map_shank_bias(float l1, float l2, float leg_len_ref);

/**
 * @brief 复刻 `padog.cal_test_shank(x, leg_trim)`：`0.006649x² + 0.4414x + 5.53`
 *        （x 已含 bias 与 trim）。
 */
float servo_map_shank_curve(float x, float leg_trim, float bias);

/* ---------------- 关节角 -> 12 路舵机角 ---------------- */

/** `servo_map_legs_to_angles()` 的输入 */
typedef struct {
    /** 是否走 IK 路径（原 `case==0 and init==0`）。false = 直接站姿（用中位角） */
    bool  ik_path;

    /** 大腿 IK 角，索引 0..3 = 腿1..腿4 */
    float ham[SERVO_MAP_LEGS];
    /** 小腿 IK 角，索引 0..3 = 腿1..腿4 */
    float shank[SERVO_MAP_LEGS];
    /** 髋辅助偏航增量 h1..h4（来自 `padog._hip_leg_deltas()`；P2 站立时全 0） */
    float hip[SERVO_MAP_LEGS];
    /** 爬行压低增量 cs1..cs4（来自 `_crawl_shank_servodelta()`；不爬行时全 0） */
    float crawl_cs[SERVO_MAP_LEGS];

    /** 中位角 `[腿][关节]`，腿 0..3 = 腿1..腿4，关节 0=髋 1=大腿 2=小腿 */
    float init[SERVO_MAP_LEGS][SERVO_MAP_JOINTS];
    /** 每条腿的小腿微调 `s_trim`（原 `_leg_cfg("s_trim", n)`，config_s.py 里没有 => 0） */
    float s_trim[SERVO_MAP_LEGS];

    /** `servo_map_shank_bias()` 的结果，由调用方按当前几何算好传入 */
    float shank_bias;
} servo_map_input_t;

/**
 * @brief 算出 12 个舵机角度。
 *
 * @param in        输入
 * @param out_deg   输出：按**逻辑通道 0..11** 排列的舵机角度。
 *                  未经过 `_clamp_deg` 的项保持原实现那样不夹（见下）。
 *
 * 原实现里 `_clamp_deg`（夹到 [0,180]）**只加在髋和小腿上，大腿没有夹**：
 * ```python
 * angle(1, init_1h+90-ham1)      # 大腿：不夹
 * angle(0, clamp_deg(init_1p+h1))  # 髋：夹
 * ```
 * C 版照样不夹大腿 —— 越界会由 `servo_map_deg_to_duty()` 的占空比限幅兜住，
 * 结果与原实现一致。**不要"顺手"给大腿也加上 clamp**，那会改变行为。
 */
void servo_map_legs_to_angles(const servo_map_input_t *in,
                              float out_deg[SERVO_MAP_CHANNELS]);

/**
 * @brief 便利函数：算角度并直接得到 12 组 (ON, OFF)。
 *
 * 等价于对每个通道依次调用 `servo_map_legs_to_angles()` 与
 * `servo_map_deg_to_duty()` / `servo_map_duty_to_pwm()`。
 */
void servo_map_legs_to_pwm(const servo_map_input_t *in,
                           uint16_t on_out[SERVO_MAP_CHANNELS],
                           uint16_t off_out[SERVO_MAP_CHANNELS]);

/** @brief 逻辑通道名（日志用），例如 0 -> "L-F hip"。越界返回 "?"。 */
const char *servo_map_channel_name(uint8_t logical_ch);

#ifdef __cplusplus
}
#endif
