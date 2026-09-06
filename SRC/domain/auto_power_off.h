#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t configured_minutes;
    int64_t started_us;
    int64_t candidate_since_us;
    int64_t last_update_us;
    int64_t blocked_stationary_since_us;
    bool tracking;
    bool triggered;
} auto_power_off_state_t;

/** Clear all stationary-duration tracking state. */
void auto_power_off_reset(auto_power_off_state_t *state);

/**
 * Update the stationary-duration shutdown detector.
 *
 * Returns true once when the configured interval expires while the motion
 * classifier remains stationary. stationary_since_us is the beginning of the
 * already-confirmed stationary candidate, so confirmation time is included.
 */
bool auto_power_off_update(auto_power_off_state_t *state,
                           uint32_t configured_minutes,
                           bool external_power_present,
                           bool stationary,
                           int64_t stationary_since_us,
                           int64_t now_us);
