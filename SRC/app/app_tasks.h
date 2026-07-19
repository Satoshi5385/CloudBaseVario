#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/**
 * @brief Create application tasks in the order required by SW_spec.md.
 * @return ESP_OK when all five tasks were created, otherwise ESP_ERR_NO_MEM.
 */
esp_err_t app_tasks_start(void);

/**
 * @brief Return the acknowledgement mask for tasks created so far.
 * @return Event-group bits corresponding to active application tasks.
 */
EventBits_t app_tasks_active_ack_mask(void);

/**
 * @brief Report whether system_task was successfully created.
 * @return true after successful system_task creation.
 */
bool app_tasks_system_started(void);

/**
 * @brief Run the SW1-aware fatal loop when system_task cannot be created.
 * @note This function does not return while external power remains applied.
 */
void app_tasks_run_fatal_fallback(void);
