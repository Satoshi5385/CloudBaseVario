#pragma once

#include <stdbool.h>

#include "domain/app_config.h"
#include "domain/app_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#define APP_EVENT_STOP_REQUEST BIT0
#define APP_EVENT_FATAL_STATE BIT1
#define APP_EVENT_AUDIO_ACK BIT2
#define APP_EVENT_SENSOR_ACK BIT3
#define APP_EVENT_SYSTEM_ACK BIT4
#define APP_EVENT_CONSOLE_ACK BIT5
#define APP_EVENT_BLE_TX_ACK BIT6
#define APP_EVENT_SAFE_SLEEP_BLOCKED BIT7
#define APP_EVENT_BMP581_STARTUP_COMPLETE BIT8
#define APP_EVENT_FATAL_BMP581 BIT9
#define APP_EVENT_BMP581_RECOVERING BIT10
#define APP_EVENT_IMU_CALIBRATING BIT11
#define APP_EVENT_IMU_DEGRADED BIT12
#define APP_EVENT_AUDIO_QUIESCED BIT13
#define APP_EVENT_SHUTDOWN_SOUND_REQUEST BIT14
#define APP_EVENT_SHUTDOWN_SOUND_DONE BIT15
#define APP_EVENT_SHUTDOWN_SOUND_ABORT BIT16
#define APP_EVENT_IMU_ACCEL_CALIBRATION_SAVED BIT20
#define APP_EVENT_IMU_ACCEL_CALIBRATION_REQUIRED BIT21
#define APP_EVENT_IMU_ACCEL_CALIBRATION_SKIP_REQUEST BIT22
#define APP_EVENT_IMU_ACCEL_CALIBRATION_SKIPPED BIT23

#define APP_EVENT_ALL_TASK_ACKS                                                                    \
    (APP_EVENT_AUDIO_ACK | APP_EVENT_SENSOR_ACK | APP_EVENT_SYSTEM_ACK | APP_EVENT_CONSOLE_ACK |   \
     APP_EVENT_BLE_TX_ACK)

/**
 * @brief Allocate the startup-only queues, mutexes, and event group.
 * @return ESP_OK on success, otherwise ESP_ERR_NO_MEM.
 */
esp_err_t app_resources_init(void);

/**
 * @brief Obtain the length-one latest-value queue consumed by audio_task.
 * @return Queue handle, or NULL before successful initialization.
 */
QueueHandle_t app_resources_audio_queue(void);

/**
 * @brief Obtain the latest-value SW1-SW3 notification request queue.
 * @return Queue handle, or NULL before successful initialization.
 */
QueueHandle_t app_resources_button_sound_queue(void);

/**
 * @brief Obtain the fixed-size diagnostic event queue.
 * @return Queue handle, or NULL before successful initialization.
 */
QueueHandle_t app_resources_diagnostic_queue(void);

/**
 * @brief Obtain the application event group.
 * @return Event group handle, or NULL before successful initialization.
 */
EventGroupHandle_t app_resources_event_group(void);

/**
 * @brief Store the latest vario result and overwrite the audio queue.
 * @param[in] result Complete result snapshot to publish.
 * @return true when the snapshot mutex was acquired.
 */
bool app_resources_publish_vario(const vario_result_t *result);

/**
 * @brief Copy the latest complete vario snapshot.
 * @param[out] result Destination snapshot.
 * @return true when a snapshot was copied.
 */
bool app_resources_copy_vario(vario_result_t *result);

/**
 * @brief Replace the complete HXY IMU diagnostic snapshot.
 * @param[in] diagnostics Complete diagnostic state to publish.
 * @return true when the shared snapshot mutex was acquired.
 */
bool app_resources_publish_imu_diagnostics(const imu_diagnostics_t *diagnostics);

/**
 * @brief Copy the latest HXY IMU diagnostic snapshot.
 * @param[out] diagnostics Destination snapshot.
 * @return true when a snapshot was copied.
 */
bool app_resources_copy_imu_diagnostics(imu_diagnostics_t *diagnostics);

/**
 * @brief Replace the complete system snapshot.
 * @param[in] snapshot Complete system state to publish.
 * @return true when the snapshot mutex was acquired.
 */
bool app_resources_publish_system(const system_snapshot_t *snapshot);

/**
 * @brief Copy the latest complete system snapshot.
 * @param[out] snapshot Destination snapshot.
 * @return true when a snapshot was copied.
 */
bool app_resources_copy_system(system_snapshot_t *snapshot);

/**
 * @brief Atomically replace the complete runtime parameter configuration.
 * @param[in] config Fully validated candidate.
 * @return true when the configuration mutex was acquired.
 */
bool app_resources_publish_config_for_profile(const app_config_t *config,
                                              uint8_t parameter_number);

/** Copy the active configuration together with its stable profile number. */
bool app_resources_copy_active_config(app_config_t *config,
                                      uint8_t *parameter_number);

/**
 * @brief Copy one coherent runtime parameter configuration.
 * @param[out] config Destination configuration.
 * @return true when a configuration was copied.
 */
bool app_resources_copy_config(app_config_t *config);

/** Replace all parameter sets and select the requested number or the lowest. */
bool app_resources_publish_config_profiles(
    const app_config_profiles_t *profiles, uint8_t requested_number,
    uint8_t *selected_number);

/** Copy every in-RAM parameter set for an explicit PARAM SAVE. */
bool app_resources_copy_config_profiles(app_config_profiles_t *profiles);

/** Select the next parameter set in ascending-number cyclic order. */
bool app_resources_select_next_config(uint8_t *parameter_number,
                                      uint8_t *parameter_set_count);

/** Copy the active configuration and its change revision atomically. */
bool app_resources_copy_config_with_revision(app_config_t *config,
                                             uint32_t *revision);

/** Enable a persistent diagnostic climb-rate input. */
bool app_resources_set_debug_vario(float climb_rate_mps,
                                   bool pressure_override_valid,
                                   int32_t pressure_pa_x100);

/** Disable the diagnostic input and return consumers to sensor data. */
void app_resources_clear_debug_vario(void);

/** Apply the current diagnostic input to a private consumer snapshot. */
bool app_resources_apply_debug_vario(vario_result_t *result,
                                     int64_t current_time_us);

/**
 * @brief Post a fixed-size diagnostic event without blocking the caller.
 * @param[in] event Event to enqueue.
 * @return true when queued, false when unavailable or full.
 */
bool app_resources_post_diagnostic(const diagnostic_event_t *event);
