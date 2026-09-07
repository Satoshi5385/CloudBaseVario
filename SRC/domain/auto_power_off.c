#include "domain/auto_power_off.h"

#include <stddef.h>
#include <string.h>

#define MICROSECONDS_PER_MINUTE INT64_C(60000000)

void auto_power_off_reset(auto_power_off_state_t *state) {
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
}

static void start_tracking(auto_power_off_state_t *state,
                           int64_t started_us,
                           int64_t now_us) {
    state->started_us = started_us;
    state->last_update_us = now_us;
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
                           bool stationary,
                           int64_t stationary_since_us,
                           int64_t now_us) {
    int64_t required_us = 0;
    int64_t effective_stationary_since_us = stationary_since_us;

    if (state == NULL) {
        return false;
    }
    if (state->configured_minutes != configured_minutes) {
        auto_power_off_reset(state);
        state->configured_minutes = configured_minutes;
    }
    if (external_power_present) {
        reset_tracking(state);
        if (stationary) {
            state->blocked_stationary_since_us = stationary_since_us;
        }
        return false;
    }
    if (configured_minutes == 0U || !stationary ||
        stationary_since_us < 0 || stationary_since_us > now_us ||
        now_us < 0) {
        reset_tracking(state);
        state->blocked_stationary_since_us = 0;
        return false;
    }
    if (state->tracking && now_us < state->last_update_us) {
        reset_tracking(state);
        start_tracking(state, now_us, now_us);
        return false;
    }
    if (!state->tracking) {
        if (state->blocked_stationary_since_us == stationary_since_us) {
            effective_stationary_since_us = now_us;
        }
        start_tracking(state, effective_stationary_since_us, now_us);
        state->blocked_stationary_since_us = 0;
    } else if (state->started_us < stationary_since_us) {
        start_tracking(state, stationary_since_us, now_us);
    }

    state->last_update_us = now_us;
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

uint32_t auto_power_off_elapsed_seconds(
    const auto_power_off_state_t *state, int64_t now_us) {
    int64_t elapsed_us = 0;

    if (state == NULL || !state->tracking || state->started_us <= 0 ||
        now_us < state->started_us) {
        return 0U;
    }
    elapsed_us = now_us - state->started_us;
    if (elapsed_us / INT64_C(1000000) > (int64_t) UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t) (elapsed_us / INT64_C(1000000));
}
