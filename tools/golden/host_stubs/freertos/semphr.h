/**
 * @file    freertos/semphr.h
 * @brief   宿主测试用的互斥锁桩
 *
 * ⚠️ 因为宿主模拟是**单线程**的，这里取/放锁永远是"成功"，不做真正的互斥。
 * ⇒ 宿主测试**不能**发现死锁、优先级反转、锁粒度问题。
 * 那些只能靠 review 和真机。测试里会显式打印这条边界。
 */
#pragma once

#include "freertos/FreeRTOS.h"

typedef void *SemaphoreHandle_t;

/** 返回一个非 NULL 的哑句柄（调用方只判断是否为 NULL） */
SemaphoreHandle_t xSemaphoreCreateMutex(void);
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void);

/** 恒返回 pdTRUE */
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout);
BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t sem, TickType_t timeout);
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem);
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t sem);

/** 供测试断言用：统计取锁次数，证明代码路径真的走了锁 */
unsigned long host_sem_take_count(void);
void          host_sem_reset_count(void);
