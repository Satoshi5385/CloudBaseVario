#pragma once

#include <stdbool.h>

#include "esp_err.h"

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
