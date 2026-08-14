#pragma once

#include <stdbool.h>
#include <stdint.h>

#define BATTERY_DISPLAY_UPDATE_INTERVAL_US INT64_C(30000000)
#define BATTERY_STARTUP_MINIMUM_V 3.2f
#define BATTERY_RUNTIME_SHUTDOWN_V 3.1f

typedef struct {
    int64_t window_started_us;
    float window_min_voltage_v;
    float displayed_voltage_v;
    bool displayed_valid;
    bool window_has_valid_sample;
} battery_display_state_t;

/** Convert battery voltage to the shared 0-100% Battery Service level. */
uint8_t battery_level_percent_from_voltage(float battery_voltage_v);

/** Block normal startup only for a valid low battery without external power. */
bool battery_power_startup_blocked(bool external_power_present,
                                   bool battery_valid,
                                   float battery_voltage_v);

/** Request shutdown only for a valid low battery without external power. */
bool battery_power_shutdown_required(bool external_power_present,
                                     bool battery_valid,
                                     float battery_voltage_v);

/**
 * Update the BLE battery display from valid five-sample-median readings.
 * The first valid reading is published immediately. Afterwards the displayed
 * voltage is updated every 30 seconds to the minimum valid reading collected
 * in that interval. Invalid readings retain the previous displayed value.
 */
bool battery_display_update(battery_display_state_t *state,
                            bool sample_valid, float sample_voltage_v,
                            int64_t now_us, float *display_voltage_v);
