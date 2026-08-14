#pragma once

#include <stdbool.h>

#include "domain/app_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "platform/imu_calibration_storage.h"
#include "platform/switch_preferences.h"

/** Set the persisted accelerometer calibration before workers start. */
void app_workers_set_imu_accel_calibration(
    const imu_accel_calibration_t *calibration,
    const imu_calibration_storage_diagnostics_t *diagnostics);
/** Set the initial switch preferences and their pending-save state. */
void app_workers_set_switch_preferences(
    const switch_preferences_t *preferences, bool dirty);
/** Play the synchronous startup sound at the selected volume. */
esp_err_t app_workers_play_startup_sound(
    audio_volume_level_t volume_level);
/** Run the sensor acquisition and estimation worker. */
void app_sensor_worker_task(void *context);
/** Run the vario and notification audio worker. */
void app_audio_worker_task(void *context);
/** Run switch, lifecycle, and status policy processing. */
void app_system_worker_task(void *context);
/** Run the TinyUSB CDC command and telemetry worker. */
void app_console_worker_task(void *context);
/** Enter the coordinated safe-stop sequence. */
void app_workers_run_safe_stop(void);
/** Run the reduced fatal fallback when normal workers are unavailable. */
void app_workers_run_fatal_fallback(void);
