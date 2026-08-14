#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/** Register the current SAFE_STOP task for SW1 GPIO notifications. */
esp_err_t safe_stop_wake_init(TaskHandle_t task);

/** Arm SW1 as an active-high press wake or active-low release wake. */
esp_err_t safe_stop_wake_arm(bool wake_when_pressed);

/** Disable the SW1 interrupt and light-sleep wake source. */
void safe_stop_wake_disarm(void);

/** Remove the SW1 ISR handler. */
void safe_stop_wake_deinit(void);
