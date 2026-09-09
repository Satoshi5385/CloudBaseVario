#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h"
typedef void *TaskHandle_t;
typedef int BaseType_t;
void vTaskDelay(uint32_t ticks);
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *task_woken);
#define portYIELD_FROM_ISR() ((void) 0)
