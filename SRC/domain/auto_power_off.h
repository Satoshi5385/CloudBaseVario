#pragma once

#include <stdbool.h>
#include <stdint.h>

#define AUTO_POWER_OFF_ALTITUDE_TOLERANCE_M 5.0f
#define AUTO_POWER_OFF_ALTITUDE_RANGE_M                                      \
    (2.0f * AUTO_POWER_OFF_ALTITUDE_TOLERANCE_M)

typedef struct {
    uint32_t configured_minutes;
    int64_t started_us;
    int64_t last_update_us;
    float minimum_altitude_m;
    float maximum_altitude_m;
    bool tracking;
    bool triggered;
} auto_power_off_state_t;

/** Clear all inactivity tracking state. */
void auto_power_off_reset(auto_power_off_state_t *state);

/**
 * Update the altitude-inactivity detector.
 *
 * Returns true once when the configured interval expires while the altitude
 * range remains at or below AUTO_POWER_OFF_ALTITUDE_RANGE_M.
 */
bool auto_power_off_update(auto_power_off_state_t *state,
                           uint32_t configured_minutes,
                           bool external_power_present,
                           bool altitude_valid,
                           float altitude_m,
                           int64_t now_us);
