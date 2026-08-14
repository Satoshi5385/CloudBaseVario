#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint32_t current_cpu_frequency_mhz;
    uint32_t light_sleep_entry_count;
    uint32_t observed_frequency_switch_count;
    uint32_t sensor_lock_acquire_count;
    uint32_t lock_error_count;
    bool configured;
    bool fixed_frequency_fallback;
    bool sensor_cpu_lock_held;
    bool light_sleep_lock_held;
} app_power_diagnostics_t;

/**
 * @brief Configure 40/80 MHz DFS and keep automatic light sleep blocked while sensing.
 * @return ESP_OK when DFS and both application PM locks are available. On error, the module
 *         attempts to fall back to 80 MHz with automatic light sleep disabled.
 */
esp_err_t app_power_init(void);

/**
 * @brief Request the configured maximum CPU frequency for one sensor processing burst.
 * @return ESP_OK when the request is active, or when the fixed-80-MHz fallback is active.
 */
esp_err_t app_power_sensor_work_begin(void);

/**
 * @brief Release the sensor processing maximum-frequency request before blocking.
 * @return ESP_OK when the request was released, or when the fixed-frequency fallback is active.
 */
esp_err_t app_power_sensor_work_end(void);

/**
 * @brief Permit automatic light sleep after all sensing, audio, and BLE work has stopped.
 * @return ESP_OK when safe-stop sleep was enabled or is already disabled by fallback policy.
 */
esp_err_t app_power_enter_safe_stop(void);

/**
 * @brief Ensure PM is initialized and permit automatic light sleep in SAFE_STOP.
 *
 * This is safe to call from startup rejection paths which enter SAFE_STOP
 * before normal application power initialization.
 */
esp_err_t app_power_prepare_safe_stop(void);

/** Temporarily prevent light sleep while qualifying a SAFE_STOP SW1 action. */
esp_err_t app_power_safe_stop_interaction_begin(void);

/** Permit light sleep again after a SAFE_STOP SW1 action ends. */
esp_err_t app_power_safe_stop_interaction_end(void);

/**
 * @brief Copy a non-blocking snapshot of application power-management diagnostics.
 * @param[out] diagnostics Destination diagnostics structure.
 */
void app_power_get_diagnostics(app_power_diagnostics_t *diagnostics);
