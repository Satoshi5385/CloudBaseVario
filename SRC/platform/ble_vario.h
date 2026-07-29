#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "domain/app_types.h"
#include "esp_err.h"

typedef struct {
    uint32_t sentence_count;
    uint32_t dropped_sentence_count;
    int32_t last_notify_error;
    int64_t last_notify_success_us;
    bool connected;
    bool subscribed;
} ble_vario_diagnostics_t;

typedef struct {
    char raw_pressure[16];
    char altitude[16];
    char vario[16];
    char temperature[16];
    char battery[16];
    bool sentence_available;
} ble_vario_lk8ex1_fields_t;

/**
 * @brief Initialize NimBLE, register the NUS service, and start its host task.
 * @return ESP_OK on success, otherwise an ESP-IDF or NimBLE setup error.
 */
esp_err_t ble_vario_init(void);

/**
 * @brief Immediately gate new BLE traffic and request advertising/disconnect stop.
 * @note This call does not wait for the NimBLE host task to terminate.
 */
void ble_vario_begin_shutdown(void);

/**
 * @brief Stop advertising, disconnect, and stop the NimBLE host.
 * @return ESP_OK when stopped or not initialized, otherwise ESP_FAIL.
 */
esp_err_t ble_vario_stop(void);

/**
 * @brief Report whether a peer currently enabled NUS TX notifications.
 * @return true only while connected and subscribed.
 */
bool ble_vario_can_notify(void);

/**
 * @brief Report recent successful NUS traffic for the lifecycle LED.
 * @return true for 500 ms after a successful notification while subscribed.
 */
bool ble_vario_notify_active(void);

/**
 * @brief Format the exact five LK8EX1 payload fields without sending them.
 * @return true when arguments and all formatted fields are valid.
 */
bool ble_vario_format_lk8ex1_fields(
    const vario_result_t *vario, const system_snapshot_t *system,
    ble_vario_lk8ex1_fields_t *fields);

/**
 * @brief Format and send one LK8EX1 sentence, fragmented at ATT_MTU-3.
 * @return ESP_OK when sent; ESP_ERR_INVALID_STATE when not connected/subscribed;
 *         ESP_ERR_NOT_FOUND when both measurement fields are invalid; otherwise
 *         ESP_FAIL for a dropped NimBLE notification.
 */
esp_err_t ble_vario_notify_lk8ex1(const vario_result_t *vario,
                                  const system_snapshot_t *system);

/** Copy notification counters without blocking the NimBLE host. */
void ble_vario_get_diagnostics(ble_vario_diagnostics_t *diagnostics);
