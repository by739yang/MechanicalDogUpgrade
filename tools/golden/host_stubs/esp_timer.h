/**
 * @file    esp_timer.h
 * @brief   宿主测试用的 ESP-IDF `esp_timer.h` 桩 —— **可控时钟**是这一层的核心
 *
 * `esp_timer_get_time()` 返回**模拟时间**，由测试通过 `host_advance_us()` 推进，
 * 或者在 `vTaskDelayUntil()` 里被自动推进（见 freertos/task.h 的桩）。
 *
 * 有了可控时钟，`motion` / `app_chain` 这类**依赖时间的逻辑**才能被确定性地测试：
 * 节拍门控、超时停车、急停延迟、超期计数 —— 这些在真机上是"看运气"才能复现的。
 */
#pragma once

#include <stdint.h>

/** 当前模拟时刻（微秒），从 0 开始 */
int64_t esp_timer_get_time(void);

/** 直接推进模拟时钟 */
void host_advance_us(int64_t us);
