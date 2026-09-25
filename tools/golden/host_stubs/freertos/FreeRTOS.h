/**
 * @file    freertos/FreeRTOS.h
 * @brief   宿主测试用的 FreeRTOS 桩（配合 semphr.h / task.h）
 *
 * ⚠️ **单线程协作式模拟**：没有真正的抢占、优先级、调度。
 * 所以它能验**逻辑**（状态机、节拍、超时、门控），
 * **不能**验并发安全、优先级反转、栈深度、看门狗。
 *
 * `vTaskDelayUntil()` 会**推进模拟时钟** —— 这是让任务循环能被确定性驱动的关键。
 */
#pragma once

#include <stdint.h>

typedef uint32_t TickType_t;
typedef int      BaseType_t;

#define pdTRUE       ((BaseType_t)1)
#define pdFALSE      ((BaseType_t)0)
#define pdPASS       pdTRUE
#define pdFAIL       pdFALSE
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFu)

/** 本桩约定：**1 tick = 1 ms**（与 ESP-IDF 默认 `configTICK_RATE_HZ=1000` 一致） */
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portTICK_PERIOD_MS 1

/** 模拟时钟（微秒） */
int64_t host_now_us(void);

/** 推进模拟时钟 */
void host_advance_us(int64_t us);

/** 当前 tick 数（= 模拟毫秒） */
TickType_t xTaskGetTickCount(void);
