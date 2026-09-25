/**
 * @file    freertos/task.h
 * @brief   宿主测试用的任务桩 —— **把任务体变成测试可以自己驱动的函数**
 *
 * 关键设计：`xTaskCreatePinnedToCore()` 不真的起线程，而是把入口函数与参数记下来，
 * 测试随后用 `host_task_step()` 一帧一帧地驱动它。
 *
 * `vTaskDelayUntil()` 会把**模拟时钟推进到唤醒目标**，所以
 * 「任务循环 + 固定周期」这件事在电脑上是有确定时序的。
 */
#pragma once

#include "freertos/FreeRTOS.h"

typedef void *TaskHandle_t;

typedef void (*host_task_fn_t)(void *);

/**
 * 记录任务入口（不起线程）。
 * @return 恒为 pdPASS
 */
BaseType_t xTaskCreatePinnedToCore(host_task_fn_t fn, const char *name,
                                   uint32_t stack, void *arg,
                                   unsigned prio, TaskHandle_t *out, int core);

/** 记录普通创建（vTaskDelay 等用不到，但保持 API 完整） */
BaseType_t xTaskCreate(host_task_fn_t fn, const char *name, uint32_t stack,
                       void *arg, unsigned prio, TaskHandle_t *out);

/**
 * @brief 取回被记录的任务入口。
 * @return true 表示确实有一个任务被创建过
 */
int  host_task_get(host_task_fn_t *fn, void **arg, uint32_t *stack,
                   unsigned *prio, int *core);

/** 任务是否已经调用过 `vTaskDelete(NULL)` */
int  host_task_exited(void);
void host_task_reset(void);

/** 推进模拟时钟（真的 delay） */
void vTaskDelay(TickType_t ticks);

/**
 * @brief 延时到 `*prev + period`，并更新 `*prev`。
 *
 * 桩实现：把模拟时钟直接推到那个时刻（若已经过了就不动）。
 * ⇒ 于是"任务循环 + 固定周期"在宿主上有了确定的时序。
 */
void vTaskDelayUntil(TickType_t *prev, TickType_t period);

void vTaskDelete(TaskHandle_t task);
