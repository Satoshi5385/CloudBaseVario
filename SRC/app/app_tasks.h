#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "platform/imu_calibration_storage.h"

/** Provide the startup mc_data.json result before app_tasks_start(). */
void app_tasks_set_imu_accel_calibration(
    const imu_accel_calibration_t *calibration,
    const imu_calibration_storage_diagnostics_t *diagnostics);

/**
 * @brief Create application tasks and wait for the required BMP581 startup initialization.
 * @return ESP_OK when all tasks and BMP581 are ready; ESP_ERR_NOT_FOUND for BMP581 startup
 *         failure; otherwise a task/resource error.
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

/** Report whether all five required long-lived software workers exist. */
bool app_tasks_required_workers_started(void);

/**
 * @brief Run the SW1-aware fatal loop when system_task cannot be created.
 * @note This function does not return while external power remains applied.
 */
void app_tasks_run_fatal_fallback(void);
