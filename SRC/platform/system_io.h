#pragma once

#include <stdbool.h>

#include "esp_err.h"

typedef struct {
    int32_t last_raw;
    int32_t last_calibrated_mv;
    uint32_t valid_sample_count;
    uint32_t error_count;
    uint32_t saturation_count;
} system_io_battery_diagnostics_t;

/**
 * @brief Initialize the GPIO1 ADC oneshot channel and calibration handle.
 * @return ESP_OK when calibrated battery reads are available.
 */
esp_err_t system_io_init(void);

/**
 * @brief Add one calibrated sample to the 10 Hz rolling median window.
 * @param[out] battery_voltage_v Calibrated battery-side voltage in volts.
 * @return true when five valid samples are available.
 */
bool system_io_read_battery_voltage(float *battery_voltage_v);

/** Copy the latest ADC sample and cumulative battery-read counters. */
void system_io_get_battery_diagnostics(system_io_battery_diagnostics_t *diagnostics);

/**
 * @brief Read the external-power input.
 * @return true when USB external power is present.
 */
bool system_io_external_power_present(void);

/**
 * @brief Read the active-low SW2 input.
 * @return true while SW2 is pressed.
 */
bool system_io_sw2_pressed(void);

/**
 * @brief Read the active-low SW3 input.
 * @return true while SW3 is pressed.
 */
bool system_io_sw3_pressed(void);
