#pragma once

#include <stdbool.h>

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
 * @brief Post a fixed-size diagnostic event without blocking the caller.
 * @param[in] event Event to enqueue.
 * @return true when queued, false when unavailable or full.
 */
bool app_resources_post_diagnostic(const diagnostic_event_t *event);
