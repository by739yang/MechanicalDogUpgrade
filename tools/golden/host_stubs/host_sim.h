/**
 * @file    host_sim.h
 * @brief   宿主桩层的模拟内核：可控时钟 + 假 I2C 总线 + PCA9685 影子寄存器 + 任务驱动
 *
 * ## 这一层能做到什么
 *
 * 把固件里**除了真硬件之外**的所有层都跑起来：
 * ```
 * app_config defaults -> app_chain -> control_chain(+cmd) -> servo_map
 *   -> servo_out -> drv_pca9685 -> （假 I2C）-> PCA9685 影子寄存器
 * ```
 * 于是可以在电脑上断言：
 *   - 节拍门控（65 ms 到了才推进）
 *   - 急停 / 超时（用模拟时钟精确量"多少毫秒之后生效"）
 *   - 模式切换（pose ↔ chain）
 *   - "只写变化的通道"（数影子寄存器被写的次数）
 *   - **寄存器里最终是什么字节**（连 P-21 那个 `&0x1F` 编码也一起验）
 *
 * ## 这一层做不到什么（必须说清楚）
 *
 * - 真实时序：抖动、中断延迟、I2C 实际耗时、WiFi 占用
 * - 并发：桩是**单线程**的，互斥锁永远成功 ⇒ 查不出死锁/优先级反转
 * - 栈深度、看门狗、任务优先级
 * - 真板子上任何电气/接线问题
 */
#pragma once

#include <stdint.h>

/* TickType_t / SemaphoreHandle_t / TaskHandle_t 等基础类型来自 FreeRTOS 桩 */
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* ---------------- 模拟时钟 ---------------- */

int64_t host_now_us(void);
void    host_advance_us(int64_t us);
void    host_clock_reset(void);

/* ---------------- 日志 ---------------- */

void host_log_set_quiet(int quiet);

/* ---------------- 假 I2C / PCA9685 影子寄存器 ---------------- */

#define HOST_PCA_BOARDS 2          /* 0x40 与 0x41 */

/** 清空影子寄存器（全部置 0）并把写计数清零 */
void host_pca_reset(void);

/** 从影子寄存器取一个字节（读不到返回 0） */
uint8_t host_pca_read(uint8_t addr, uint8_t reg);

/**
 * @brief 取某个逻辑通道的 (ON, OFF)，按 PCA9685 的真实位域解析。
 *
 * 高字节 **必须 & 0x1F**：bit4 是 FULL ON / FULL OFF 标志，
 * `OFF=4096` 就是靠它表示的（成长手册 P-21）。
 */
void host_pca_get_pwm(uint8_t addr, uint8_t ch, uint16_t *on, uint16_t *off);

/** 影子寄存器被写的**次数**（用来验"只写变化的通道"） */
unsigned long host_pca_write_count(void);

/** 单个通道被写的次数（非 0 即说明这一路被重写过） */
unsigned long host_pca_channel_write_count(uint8_t addr, uint8_t ch);

/** 按逻辑通道（0..11）数：哪几路被写过 */
void host_pca_reset_write_stats(void);

/* ---------------- 任务驱动 ---------------- */

/** 取回被记录的任务入口（见 freertos/task.h） */
int  host_task_get(host_task_fn_t *fn, void **arg, uint32_t *stack,
                   unsigned *prio, int *core);
int  host_task_exited(void);
void host_task_reset(void);

/**
 * @brief 驱动任务循环若干"帧"。
 *
 * 每帧：调用任务体一次（任务体内部会 `vTaskDelayUntil` 把模拟时钟推进一个周期），
 * 直到任务退出或达到 `max_frames`。
 *
 * @return 实际驱动的帧数（= 任务体被调用的次数）
 */
int host_task_run(int max_frames);

/** 取锁次数（证明代码真的走了锁路径） */
unsigned long host_sem_take_count(void);
void          host_sem_reset_count(void);

/* ---------------- 测试小工具 ---------------- */

extern int g_host_fail;
extern int g_host_checks;

#define HOST_CHECK(cond, ...)                                                  \
    do {                                                                       \
        ++g_host_checks;                                                       \
        if (!(cond)) {                                                         \
            ++g_host_fail;                                                      \
            printf("  FAIL: ");                                                \
            printf(__VA_ARGS__);                                               \
            printf("   (%s:%d)\n", __FILE__, __LINE__);                        \
        }                                                                      \
    } while (0)

/** 打印一节小标题 */
void host_section(const char *title);
