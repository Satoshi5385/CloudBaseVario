#include "domain/battery_level.h"

#include <math.h>
#include <stddef.h>

typedef struct {
    float voltage_v;
    uint8_t percent;
} battery_level_point_t;

static const battery_level_point_t battery_level_curve[] = {
    {3.20f, 0U},
    {3.50f, 10U},
    {3.60f, 20U},
    {3.70f, 40U},
    {3.80f, 60U},
    {3.90f, 80U},
    {4.10f, 100U},
};

static bool battery_power_threshold_reached(bool external_power_present,
                                            bool battery_valid,
                                            float battery_voltage_v,
                                            float threshold_v) {
    return !external_power_present && battery_valid &&
           isfinite(battery_voltage_v) && battery_voltage_v <= threshold_v;
}

uint8_t battery_level_percent_from_voltage(float battery_voltage_v) {
    size_t point_count =
        sizeof(battery_level_curve) / sizeof(battery_level_curve[0]);

    if (!isfinite(battery_voltage_v) ||
        battery_voltage_v <= battery_level_curve[0].voltage_v) {
        return 0U;
    }
    if (battery_voltage_v >= battery_level_curve[point_count - 1U].voltage_v) {
        return 100U;
    }

    for (size_t index = 1U; index < point_count; index++) {
        const battery_level_point_t *lower = &battery_level_curve[index - 1U];
        const battery_level_point_t *upper = &battery_level_curve[index];

        if (battery_voltage_v <= upper->voltage_v) {
            float level = (float) lower->percent +
                (battery_voltage_v - lower->voltage_v) *
                    (float) (upper->percent - lower->percent) /
                    (upper->voltage_v - lower->voltage_v);
            return (uint8_t) lroundf(level);
        }
    }

    return 100U;
}

bool battery_power_startup_blocked(bool external_power_present,
                                   bool battery_valid,
                                   float battery_voltage_v) {
    return battery_power_threshold_reached(
        external_power_present, battery_valid, battery_voltage_v,
        BATTERY_STARTUP_MINIMUM_V);
}

bool battery_power_shutdown_required(bool external_power_present,
                                     bool battery_valid,
                                     float battery_voltage_v) {
    return battery_power_threshold_reached(
        external_power_present, battery_valid, battery_voltage_v,
        BATTERY_RUNTIME_SHUTDOWN_V);
}

bool battery_display_update(battery_display_state_t *state,
                            bool sample_valid, float sample_voltage_v,
                            int64_t now_us, float *display_voltage_v) {
    bool valid = sample_valid && isfinite(sample_voltage_v) &&
                 sample_voltage_v >= 0.0f;

    if (state == NULL || display_voltage_v == NULL) {
        return false;
    }
    if (!state->displayed_valid) {
        if (!valid) {
            return false;
        }
        state->window_started_us = now_us;
        state->window_min_voltage_v = sample_voltage_v;
        state->displayed_voltage_v = sample_voltage_v;
        state->displayed_valid = true;
        state->window_has_valid_sample = true;
        *display_voltage_v = state->displayed_voltage_v;
        return true;
    }

    if (now_us < state->window_started_us) {
        state->window_started_us = now_us;
        state->window_has_valid_sample = valid;
        if (valid) {
            state->window_min_voltage_v = sample_voltage_v;
        }
    } else {
        if (valid && (!state->window_has_valid_sample ||
                      sample_voltage_v < state->window_min_voltage_v)) {
            state->window_min_voltage_v = sample_voltage_v;
            state->window_has_valid_sample = true;
        }
        if (now_us - state->window_started_us >=
            BATTERY_DISPLAY_UPDATE_INTERVAL_US) {
            if (state->window_has_valid_sample) {
                state->displayed_voltage_v =
                    state->window_min_voltage_v;
            }
            state->window_started_us = now_us;
            state->window_has_valid_sample = valid;
            if (valid) {
                state->window_min_voltage_v = sample_voltage_v;
            }
        }
    }

    *display_voltage_v = state->displayed_voltage_v;
    return true;
}
