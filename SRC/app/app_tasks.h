#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "platform/switch_preferences.h"
#include "platform/imu_calibration_storage.h"

/** Required stable SW1 press duration before accepting a power-on request. */
#define POWER_ON_HOLD_MS UINT32_C(2000)

_Static_assert(POWER_ON_HOLD_MS > 0U,
               "Power-on hold duration must be nonzero");

/** Provide the startup mc_data.json result before app_tasks_start(). */
void app_tasks_set_imu_accel_calibration(
    const imu_accel_calibration_t *calibration,
    const imu_calibration_storage_diagnostics_t *diagnostics);

/** Provide the startup NVS/default switch state before app_tasks_start(). */
void app_tasks_set_switch_preferences(
    const switch_preferences_t *preferences, bool dirty);

/**
 * @brief Play the blocking power-on confirmation sound at the selected volume.
 * @return ESP_OK after complete playback or when muted; otherwise the audio error.
 */
esp_err_t app_tasks_play_startup_sound(
    audio_volume_level_t volume_level);

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
 * @brief Release power hold and remain safely stopped while external power remains.
 * @note This function does not return.
 */
void app_tasks_run_safe_stop(void);

/**
 * @brief Run the SW1-aware fatal loop when system_task cannot be created.
 * @note This function does not return while external power remains applied.
 */
void app_tasks_run_fatal_fallback(void);
