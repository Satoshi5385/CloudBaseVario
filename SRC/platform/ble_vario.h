#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "domain/app_types.h"
#include "domain/lk8ex1.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BLE_VARIO_BATTERY_LEVEL_STATUS_SIZE 3U

typedef struct {
    uint32_t sentence_count;
    uint32_t dropped_sentence_count;
    int32_t last_notify_error;
    int64_t last_notify_success_us;
    bool connected;
    bool subscribed;
} ble_vario_diagnostics_t;

typedef lk8ex1_fields_t ble_vario_lk8ex1_fields_t;

/**
 * @brief Initialize NimBLE, register NUS and Battery Service, and start its host task.
 * @param[in] tx_power Initial default, advertising, and connection TX power.
 * @return ESP_OK on success, otherwise an ESP-IDF or NimBLE setup error.
 */
esp_err_t ble_vario_init(app_bluetooth_tx_power_t tx_power);

/**
 * @brief Apply one TX power preset to default, advertising, and any active link.
 * @param[in] tx_power Valid public TX power preset.
 * @return ESP_OK when every applicable controller setting succeeded.
 */
esp_err_t ble_vario_apply_tx_power(app_bluetooth_tx_power_t tx_power);

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

/** Register or clear the BLE TX worker task notified by GAP state changes. */
void ble_vario_set_tx_wakeup_task(TaskHandle_t task);

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

/** Convert a valid battery voltage to a 0-100% level using 3.0-4.1 V endpoints. */
uint8_t ble_vario_battery_level_from_voltage(float battery_voltage_v);

/** Format the three-byte Battery Level Status value defined by GSS. */
void ble_vario_format_battery_level_status(
    bool external_power_present,
    uint8_t status[BLE_VARIO_BATTERY_LEVEL_STATUS_SIZE]);

/** Update Battery Service values from the latest system snapshot. */
void ble_vario_update_battery(const system_snapshot_t *system);

/**
 * @brief Format the exact five LK8EX1 payload fields without sending them.
 * @return true when arguments and all formatted fields are valid.
 */
bool ble_vario_format_lk8ex1_fields(
    const vario_result_t *vario, const system_snapshot_t *system,
    app_bluetooth_battery_mode_t battery_mode,
    ble_vario_lk8ex1_fields_t *fields);

/**
 * @brief Format and send one LK8EX1 sentence, fragmented at ATT_MTU-3.
 * @return ESP_OK when sent; ESP_ERR_INVALID_STATE when not connected/subscribed;
 *         ESP_ERR_NOT_FOUND when both measurement fields are invalid; otherwise
 *         ESP_FAIL for a dropped NimBLE notification.
 */
esp_err_t ble_vario_notify_lk8ex1(const vario_result_t *vario,
                                  const system_snapshot_t *system,
                                  app_bluetooth_battery_mode_t battery_mode);

/** Copy notification counters without blocking the NimBLE host. */
void ble_vario_get_diagnostics(ble_vario_diagnostics_t *diagnostics);
