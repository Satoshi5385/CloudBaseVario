#include "domain/auto_power_off.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define MICROSECONDS_PER_MINUTE INT64_C(60000000)

void auto_power_off_reset(auto_power_off_state_t *state) {
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
}

static void start_tracking(auto_power_off_state_t *state, float altitude_m,
                           int64_t now_us) {
    state->started_us = now_us;
    state->last_update_us = now_us;
    state->minimum_altitude_m = altitude_m;
    state->maximum_altitude_m = altitude_m;
    state->tracking = true;
    state->triggered = false;
}

static void reset_tracking(auto_power_off_state_t *state) {
    uint32_t configured_minutes = state->configured_minutes;

    auto_power_off_reset(state);
    state->configured_minutes = configured_minutes;
}

bool auto_power_off_update(auto_power_off_state_t *state,
                           uint32_t configured_minutes,
                           bool external_power_present,
                           bool altitude_valid,
                           float altitude_m,
                           int64_t now_us) {
    int64_t required_us = 0;

    if (state == NULL) {
        return false;
    }
    if (state->configured_minutes != configured_minutes) {
        auto_power_off_reset(state);
        state->configured_minutes = configured_minutes;
    }
    if (configured_minutes == 0U || external_power_present ||
        !altitude_valid || !isfinite(altitude_m) || now_us < 0) {
        reset_tracking(state);
        return false;
    }
    if (state->tracking && now_us < state->last_update_us) {
        reset_tracking(state);
    }
    if (!state->tracking) {
        start_tracking(state, altitude_m, now_us);
        return false;
    }

    state->last_update_us = now_us;
    if (altitude_m < state->minimum_altitude_m) {
        state->minimum_altitude_m = altitude_m;
    }
    if (altitude_m > state->maximum_altitude_m) {
        state->maximum_altitude_m = altitude_m;
    }
    if (state->maximum_altitude_m - state->minimum_altitude_m >
        AUTO_POWER_OFF_ALTITUDE_RANGE_M) {
        start_tracking(state, altitude_m, now_us);
        return false;
    }
    if (state->triggered) {
        return false;
    }

    required_us = (int64_t) configured_minutes *
                  MICROSECONDS_PER_MINUTE;
    if (now_us - state->started_us >= required_us) {
        state->triggered = true;
        return true;
    }
    return false;
}
