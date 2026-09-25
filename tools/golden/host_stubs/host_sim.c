/**
 * @file    host_sim.c
 * @brief   宿主桩层的模拟内核实现
 *
 * 这个文件同时提供：
 *   - `esp_timer` / `esp_log` / FreeRTOS 的桩实现
 *   - **假 I2C 总线**（把 `drv_pca9685.c` 的写落进 PCA9685 影子寄存器）
 *
 * ⚠️ 它是**单线程**的：互斥锁恒成功，任务不真的并发。
 * 所以它能验逻辑，不能验并发与真实时序。见 host_sim.h 的说明。
 */

#include "host_sim.h"

#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bsp/bsp_i2c.h"
#include "drivers/drv_pca9685.h"
#include "esp_err.h"

/* ==========================================================================
 * 模拟时钟
 * ========================================================================== */

static int64_t s_now_us = 0;

int64_t host_now_us(void) { return s_now_us; }
int64_t esp_timer_get_time(void) { return s_now_us; }

void host_advance_us(int64_t us) { s_now_us += us; }

void host_clock_reset(void) { s_now_us = 0; }

TickType_t xTaskGetTickCount(void) { return (TickType_t)(s_now_us / 1000); }

/* ==========================================================================
 * 日志
 * ========================================================================== */

static int s_quiet = 1;

void host_log_set_quiet(int quiet) { s_quiet = quiet; }

void host_log_write(const char *level, const char *tag, const char *fmt, ...)
{
    if (s_quiet) {
        return;
    }
    /* ⚠️ 用 %ld 而不是 %lld：mingw 默认的 printf 不带 C99 的 ll 长度修饰符 */
    printf("[%8ld ms] %s %-12s ", (long)(s_now_us / 1000), level,
           (tag != NULL) ? tag : "?");
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
}

void host_section(const char *title)
{
    printf("\n--------------------------------------------------------\n");
    printf(" %s\n", title);
    printf("--------------------------------------------------------\n");
}

/* ==========================================================================
 * 互斥锁桩（单线程 ⇒ 恒成功）
 * ========================================================================== */

static unsigned long s_take_count = 0;
static int           s_dummy_mutex = 0;

SemaphoreHandle_t xSemaphoreCreateMutex(void) { return &s_dummy_mutex; }
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void) { return &s_dummy_mutex; }

BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout)
{
    (void)sem;
    (void)timeout;
    ++s_take_count;
    return pdTRUE;
}

BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t sem, TickType_t timeout)
{
    return xSemaphoreTake(sem, timeout);
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) { (void)sem; return pdTRUE; }
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t sem) { (void)sem; return pdTRUE; }

unsigned long host_sem_take_count(void) { return s_take_count; }
void          host_sem_reset_count(void) { s_take_count = 0; }

/* ==========================================================================
 * 任务桩：把任务体记下来，由测试驱动
 * ========================================================================== */

typedef struct {
    host_task_fn_t fn;
    void          *arg;
    uint32_t       stack;
    unsigned       prio;
    int            core;
    int            exists;
    int            exited;
} host_task_t;

static host_task_t s_task;

/** 任务让出点（见 vTaskDelayUntil 的说明） */
static jmp_buf s_task_jmp;
static int     s_frames_budget = 0;
static int     s_frames_done   = 0;

BaseType_t xTaskCreatePinnedToCore(host_task_fn_t fn, const char *name,
                                   uint32_t stack, void *arg,
                                   unsigned prio, TaskHandle_t *out, int core)
{
    (void)name;
    s_task.fn     = fn;
    s_task.arg    = arg;
    s_task.stack  = stack;
    s_task.prio   = prio;
    s_task.core   = core;
    s_task.exists = 1;
    s_task.exited = 0;
    if (out != NULL) {
        *out = &s_dummy_mutex;
    }
    return pdPASS;
}

BaseType_t xTaskCreate(host_task_fn_t fn, const char *name, uint32_t stack,
                       void *arg, unsigned prio, TaskHandle_t *out)
{
    return xTaskCreatePinnedToCore(fn, name, stack, arg, prio, out, 0);
}

int host_task_get(host_task_fn_t *fn, void **arg, uint32_t *stack,
                  unsigned *prio, int *core)
{
    if (!s_task.exists) {
        return 0;
    }
    if (fn != NULL)    { *fn = s_task.fn; }
    if (arg != NULL)   { *arg = s_task.arg; }
    if (stack != NULL) { *stack = s_task.stack; }
    if (prio != NULL)  { *prio = s_task.prio; }
    if (core != NULL)  { *core = s_task.core; }
    return 1;
}

int  host_task_exited(void) { return s_task.exited; }
void host_task_reset(void) { memset(&s_task, 0, sizeof(s_task)); }

void vTaskDelay(TickType_t ticks)
{
    s_now_us += (int64_t)ticks * 1000;
}

/*
 * ⚠️ 这里必须用 `setjmp/longjmp` 把 `vTaskDelayUntil()` 做成**协程式让出点**。
 *
 * 原因：真机上任务体是 `while (running) { ...; vTaskDelayUntil(...); }` 这样的
 * **死循环**，由调度器在 delay 处切走。宿主上是单线程，如果只是"调用一次任务体"，
 * 调用永远不会返回（本测试第一版就是这样挂死的）。
 *
 * 所以：`vTaskDelayUntil()` 累加帧计数，达到预算就 `longjmp` 回 `host_task_run()`。
 * 语义上等价于"任务在 delay 处被挂起，控制权回到测试"。
 * 区别：局部变量在下次进入时会重新初始化（统计用的 sum 之类），
 * 而 `static` 状态（运行标志、姿态、app_chain 的命令）都会保留 —— 这对被测逻辑没影响。
 */
void vTaskDelayUntil(TickType_t *prev, TickType_t period)
{
    if (prev == NULL) {
        return;
    }
    *prev += period;
    const int64_t target_us = (int64_t)(*prev) * 1000;
    if (s_now_us < target_us) {
        s_now_us = target_us;   /* ← 关键：把模拟时钟推到唤醒时刻 */
    }

    ++s_frames_done;
    if (s_frames_done >= s_frames_budget) {
        longjmp(s_task_jmp, 1);   /* 让出：回到 host_task_run() */
    }
}

void vTaskDelete(TaskHandle_t task)
{
    (void)task;
    s_task.exited = 1;
}

int host_task_run(int max_frames)
{
    if (!s_task.exists || s_task.fn == NULL || max_frames <= 0) {
        return 0;
    }
    s_frames_budget = max_frames;
    s_frames_done   = 0;
    if (setjmp(s_task_jmp) == 0) {
        s_task.fn(s_task.arg);    /* 正常返回 = 任务自己退出了 */
    }
    return s_frames_done;
}

/* ==========================================================================
 * 假 I2C 总线 + PCA9685 影子寄存器
 * ========================================================================== */

#define PCA_REG_COUNT 256

static uint8_t s_reg[HOST_PCA_BOARDS][PCA_REG_COUNT];
static unsigned long s_write_total = 0;
static unsigned long s_ch_write[HOST_PCA_BOARDS][16];

/** 0x40 -> 0，0x41 -> 1；其它地址返回 -1 */
static int board_index(uint8_t addr)
{
    if (addr == DRV_PCA9685_ADDR_LEFT)  { return 0; }
    if (addr == DRV_PCA9685_ADDR_RIGHT) { return 1; }
    return -1;
}

void host_pca_reset(void)
{
    memset(s_reg, 0, sizeof(s_reg));
    memset(s_ch_write, 0, sizeof(s_ch_write));
    s_write_total = 0;
}

uint8_t host_pca_read(uint8_t addr, uint8_t reg)
{
    const int b = board_index(addr);
    if (b < 0) {
        return 0;
    }
    return s_reg[b][reg];
}

void host_pca_get_pwm(uint8_t addr, uint8_t ch, uint16_t *on, uint16_t *off)
{
    const int b = board_index(addr);
    if (b < 0 || ch >= 16) {
        if (on)  { *on = 0; }
        if (off) { *off = 0; }
        return;
    }
    const uint8_t base = (uint8_t)(DRV_PCA9685_REG_LED0_ON_L + 4 * ch);
    /* 高字节 & 0x1F：bit4 是 FULL ON / FULL OFF 标志（见 P-21） */
    if (on != NULL) {
        *on  = (uint16_t)(s_reg[b][base] | ((s_reg[b][base + 1] & 0x1Fu) << 8));
    }
    if (off != NULL) {
        *off = (uint16_t)(s_reg[b][base + 2] | ((s_reg[b][base + 3] & 0x1Fu) << 8));
    }
}

unsigned long host_pca_write_count(void) { return s_write_total; }

unsigned long host_pca_channel_write_count(uint8_t addr, uint8_t ch)
{
    const int b = board_index(addr);
    if (b < 0 || ch >= 16) {
        return 0;
    }
    return s_ch_write[b][ch];
}

void host_pca_reset_write_stats(void)
{
    memset(s_ch_write, 0, sizeof(s_ch_write));
    s_write_total = 0;
}

/* ---- bsp_i2c 的假实现（真的 bsp_i2c.c 不参与宿主构建） ---- */

esp_err_t bsp_i2c_init(void) { return ESP_OK; }
esp_err_t bsp_i2c_deinit(void) { return ESP_OK; }

esp_err_t bsp_i2c_write_reg(uint8_t dev, uint8_t reg, const uint8_t *data, size_t len)
{
    if (len > 0 && data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const int b = board_index(dev);
    if (b < 0) {
        /* 写到不存在的从机：真总线上会 NACK，这里如实报错 */
        return ESP_ERR_NOT_FOUND;
    }
    /* 模拟 PCA9685 的自动递增：连续写到 reg, reg+1, ... */
    for (size_t i = 0; i < len; ++i) {
        const unsigned r = (unsigned)reg + i;
        if (r < PCA_REG_COUNT) {
            s_reg[b][r] = data[i];
        }
    }
    ++s_write_total;

    /* 只统计 LED 占空比寄存器的写（4096 那种整块写不算通道写） */
    if (len == 4 && reg >= DRV_PCA9685_REG_LED0_ON_L &&
        reg < (uint8_t)(DRV_PCA9685_REG_LED0_ON_L + 64) &&
        ((reg - DRV_PCA9685_REG_LED0_ON_L) % 4) == 0) {
        ++s_ch_write[b][(reg - DRV_PCA9685_REG_LED0_ON_L) / 4];
    }
    return ESP_OK;
}

esp_err_t bsp_i2c_read_reg(uint8_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    const int b = board_index(dev);
    if (b < 0) {
        return ESP_ERR_NOT_FOUND;
    }
    for (size_t i = 0; i < len; ++i) {
        const unsigned r = (unsigned)reg + i;
        data[i] = (r < PCA_REG_COUNT) ? s_reg[b][r] : 0;
    }
    return ESP_OK;
}

esp_err_t bsp_i2c_probe(uint8_t dev) { return (board_index(dev) >= 0) ? ESP_OK : ESP_ERR_NOT_FOUND; }

esp_err_t bsp_i2c_probe_timed(uint8_t dev, int64_t *elapsed_us)
{
    if (elapsed_us != NULL) {
        *elapsed_us = 0;
    }
    return bsp_i2c_probe(dev);
}

esp_err_t bsp_i2c_lock(uint32_t timeout_ms) { (void)timeout_ms; return ESP_OK; }
void      bsp_i2c_unlock(void) { }

esp_err_t bsp_i2c_bus_recover(void) { return ESP_OK; }
esp_err_t bsp_i2c_bitbang_begin(void) { return ESP_OK; }
size_t    bsp_i2c_scan_bitbang(uint8_t *found, size_t max_found)
{
    if (found == NULL || max_found < 2) {
        return 0;
    }
    found[0] = DRV_PCA9685_ADDR_LEFT;
    found[1] = DRV_PCA9685_ADDR_RIGHT;
    return 2;
}
