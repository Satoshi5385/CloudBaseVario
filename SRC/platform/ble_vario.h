#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "domain/app_types.h"
#include "domain/ble_nus_tx.h"
#include "domain/lk8ex1.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BLE_VARIO_BATTERY_LEVEL_STATUS_SIZE 3U

typedef struct {
    uint32_t sentence_count;
    uint32_t dropped_sentence_count;
    uint32_t gps_pair_count;
    uint32_t gps_dropped_pair_count;
    uint32_t fragment_attempt_count;
    uint32_t fragment_accepted_count;
    uint32_t fragment_error_count;
    uint32_t lk8ex1_coalesced_count;
    uint32_t gps_coalesced_count;
    uint32_t partial_abort_count;
    uint32_t stream_resync_count;
    uint32_t link_generation;
    uint32_t connection_interval_us;
    uint16_t att_mtu;
    int32_t last_notify_error;
    int64_t last_notify_success_us;
    bool connected;
    bool subscribed;
} ble_vario_diagnostics_t;

typedef struct {
    uint32_t generation;
    uint32_t connection_interval_us;
    uint16_t att_mtu;
    uint16_t payload_capacity;
    bool connected;
    bool subscribed;
} ble_vario_nus_link_t;

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

/** Copy the current generation-checked NUS link parameters. */
void ble_vario_get_nus_link(ble_vario_nus_link_t *link);

/**
 * @brief Submit one NUS fragment for the specified link generation.
 * @param[in] generation Link generation returned by ble_vario_get_nus_link.
 * @param[in] data Fragment bytes whose length is at most ATT_MTU-3.
 * @param[in] length Number of fragment bytes.
 * @param[out] nimble_error Raw NimBLE result, or zero when accepted.
 * @return ESP_OK when accepted, ESP_ERR_NO_MEM for congestion,
 *         ESP_ERR_INVALID_STATE for a stale/unusable link, otherwise ESP_FAIL.
 */
esp_err_t ble_vario_notify_nus_fragment(uint32_t generation,
                                        const uint8_t *data, size_t length,
                                        int32_t *nimble_error);

/** Request termination of the still-current NUS connection. */
esp_err_t ble_vario_reset_nus_connection(uint32_t generation);

/** Publish scheduler counters and the last complete-transaction timestamp. */
void ble_vario_publish_nus_tx_diagnostics(
    const ble_nus_tx_statistics_t *statistics,
    int64_t last_notify_success_us);

/** Copy notification counters without blocking the NimBLE host. */
void ble_vario_get_diagnostics(ble_vario_diagnostics_t *diagnostics);
