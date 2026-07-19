#pragma once

#include <stdbool.h>

#include "esp_err.h"

/**
 * @brief Initialize the GPIO1 ADC oneshot channel and calibration handle.
 * @return ESP_OK when calibrated battery reads are available.
 */
esp_err_t system_io_init(void);

/**
 * @brief Read five calibrated samples and calculate the divided battery input.
 * @param[out] battery_voltage_v Calibrated battery-side voltage in volts.
 * @return true when the complete conversion is valid.
 */
bool system_io_read_battery_voltage(float *battery_voltage_v);

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
