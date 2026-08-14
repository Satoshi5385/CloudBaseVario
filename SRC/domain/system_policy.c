#include "domain/system_policy.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#define FATAL_LED_PHASE_MS UINT32_C(500)
#define BMP_RECOVERY_LED_PHASE_MS UINT32_C(1000)
#define IMU_CALIBRATION_LED_CYCLE_MS UINT32_C(2000)
#define IMU_DEGRADED_LED_CYCLE_MS UINT32_C(1000)
#define LOW_BATTERY_LED_CYCLE_MS UINT32_C(1000)

static bool led_first_phase(uint32_t elapsed_ms, uint32_t phase_ms) {
    return elapsed_ms % (phase_ms * UINT32_C(2)) < phase_ms;
}

static uint32_t led_firefly_brightness(uint32_t elapsed_ms,
                                       uint32_t cycle_ms) {
    uint32_t position_ms = elapsed_ms % cycle_ms;
    uint32_t half_cycle_ms = cycle_ms / UINT32_C(2);

    if (position_ms < half_cycle_ms) {
        return UINT32_C(100) -
               position_ms * UINT32_C(100) / half_cycle_ms;
    }
    return (position_ms - half_cycle_ms) * UINT32_C(100) /
           half_cycle_ms;
}

bool system_policy_debounce(system_policy_button_t *button, bool pressed) {
    if (button == NULL) {
        return false;
    }
    if (pressed != button->candidate_pressed) {
        button->candidate_pressed = pressed;
        button->stable_time_ms = 0U;
        return button->stable_pressed;
    }
    if (button->stable_time_ms < SYSTEM_POLICY_SWITCH_DEBOUNCE_MS) {
        button->stable_time_ms = system_policy_increment_ms(
            button->stable_time_ms);
    }
    if (button->stable_time_ms >= SYSTEM_POLICY_SWITCH_DEBOUNCE_MS) {
        button->stable_pressed = button->candidate_pressed;
        button->stable_valid = true;
    }
    return button->stable_pressed;
}

uint32_t system_policy_increment_ms(uint32_t elapsed_ms) {
    if (UINT32_MAX - elapsed_ms < SYSTEM_POLICY_SAMPLE_PERIOD_MS) {
        return UINT32_MAX;
    }
    return elapsed_ms + SYSTEM_POLICY_SAMPLE_PERIOD_MS;
}

void system_policy_init(system_policy_state_t *state,
                        const system_policy_input_t *input) {
    if (state == NULL || input == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->sw1.candidate_pressed = input->sw1_pressed;
    state->sw1.stable_pressed = input->sw1_pressed;
    state->sw2.candidate_pressed = input->sw2_pressed;
    state->sw2.stable_pressed = input->sw2_pressed;
    state->sw3.candidate_pressed = input->sw3_pressed;
    state->sw3.stable_pressed = input->sw3_pressed;
    state->previous_sw1_pressed = input->sw1_pressed;
    state->previous_sw2_pressed = input->sw2_pressed;
    state->previous_sw3_pressed = input->sw3_pressed;
}

void system_policy_step(system_policy_state_t *state,
                        const system_policy_input_t *input,
                        system_policy_actions_t *actions) {
    bool sw1_pressed = false;
    bool sw2_pressed = false;
    bool sw3_pressed = false;

    if (state == NULL || input == NULL || actions == NULL) {
        return;
    }
    memset(actions, 0, sizeof(*actions));
    sw1_pressed = system_policy_debounce(&state->sw1, input->sw1_pressed);
    sw2_pressed = system_policy_debounce(&state->sw2, input->sw2_pressed);
    sw3_pressed = system_policy_debounce(&state->sw3, input->sw3_pressed);

    if (state->sw1.stable_valid && !sw1_pressed) {
        if (state->previous_sw1_pressed && state->sw1_was_released &&
            state->sw1_short_press_pending &&
            !state->sw1_power_off_issued) {
            actions->advance_volume = true;
        }
        state->sw1_was_released = true;
        state->sw1_hold_ms = 0U;
        state->sw1_short_press_pending = false;
        state->sw1_power_off_issued = false;
    } else if (state->sw1.stable_valid && state->sw1_was_released) {
        if (!state->previous_sw1_pressed) {
            state->sw1_hold_ms = 0U;
            state->sw1_short_press_pending = true;
            state->sw1_power_off_issued = false;
        }
        state->sw1_hold_ms = system_policy_increment_ms(state->sw1_hold_ms);
        if (state->sw1_hold_ms >= SYSTEM_POLICY_POWER_OFF_HOLD_MS) {
            state->sw1_short_press_pending = false;
            state->sw1_power_off_issued = true;
            actions->request_power_off = true;
        }
    } else {
        /* The press used to start the board is not a power-off request. */
    }

    if (state->sw2.stable_valid && sw2_pressed &&
        !state->previous_sw2_pressed) {
        actions->toggle_sink = true;
    }

    if (state->sw3.stable_valid && sw3_pressed) {
        if (!state->previous_sw3_pressed) {
            state->sw3_hold_ms = 0U;
            state->sw3_skip_request_issued = false;
            state->sw3_short_press_pending = input->imu_calibration_required;
            if (!input->imu_calibration_required) {
                actions->select_next_profile = true;
            }
        }
        if (input->imu_calibration_required &&
            state->sw3_short_press_pending &&
            !state->sw3_skip_request_issued) {
            state->sw3_hold_ms = system_policy_increment_ms(
                state->sw3_hold_ms);
            if (state->sw3_hold_ms >= SYSTEM_POLICY_IMU_SKIP_HOLD_MS) {
                actions->request_calibration_skip = true;
                state->sw3_skip_request_issued = true;
                state->sw3_short_press_pending = false;
            }
        }
    } else if (state->sw3.stable_valid && state->previous_sw3_pressed) {
        if (state->sw3_short_press_pending &&
            !state->sw3_skip_request_issued) {
            actions->select_next_profile = true;
        }
        state->sw3_hold_ms = 0U;
        state->sw3_short_press_pending = false;
        state->sw3_skip_request_issued = false;
    }

    state->previous_sw1_pressed = sw1_pressed;
    state->previous_sw2_pressed = sw2_pressed;
    state->previous_sw3_pressed = sw3_pressed;
    actions->sw1_pressed = sw1_pressed;
    actions->sw2_pressed = sw2_pressed;
    actions->sw3_pressed = sw3_pressed;
    actions->sw1_hold_ms = state->sw1_hold_ms;
    actions->sw3_hold_ms = state->sw3_hold_ms;
}

void system_policy_select_leds(const system_led_policy_input_t *input,
                               system_led_policy_output_t *output) {
    bool invalid_or_stale = false;

    if (input == NULL || output == NULL) {
        return;
    }
    memset(output, 0, sizeof(*output));
    output->green_brightness_percent = UINT32_C(100);

    if (input->fatal) {
        bool first_phase = led_first_phase(input->elapsed_ms,
                                           FATAL_LED_PHASE_MS);

        output->green_brightness_percent = 0U;
        if (!input->fatal_bmp581 && first_phase) {
            output->green_brightness_percent = 100U;
        }
        output->yellow_on = !first_phase;
        if (input->fatal_bmp581) {
            output->yellow_on = first_phase;
        }
        return;
    }
    if (!input->bmp581_startup_complete) {
        return;
    }

    if (input->storage_mode_active) {
        output->green_brightness_percent = UINT32_C(20);
        output->yellow_on =
            input->elapsed_ms % UINT32_C(200) < UINT32_C(100);
    } else {
        invalid_or_stale = input->vario_available &&
            (!input->pressure_valid ||
             (!input->climb_rate_valid &&
              !input->estimator_warming_up &&
              !input->bmp581_recovering));
        if (invalid_or_stale) {
            output->green_brightness_percent = 0U;
            output->yellow_on = led_first_phase(input->elapsed_ms,
                                                FATAL_LED_PHASE_MS);
        } else if (input->bmp581_recovering) {
            output->green_brightness_percent = 0U;
            if (led_first_phase(input->elapsed_ms,
                                BMP_RECOVERY_LED_PHASE_MS)) {
                output->green_brightness_percent = 100U;
            }
        } else if (input->imu_calibrating) {
            output->green_brightness_percent = led_firefly_brightness(
                input->elapsed_ms, IMU_CALIBRATION_LED_CYCLE_MS);
        } else if (input->estimator_warming_up) {
            output->green_brightness_percent = 0U;
            if (led_first_phase(input->elapsed_ms,
                                BMP_RECOVERY_LED_PHASE_MS)) {
                output->green_brightness_percent = 100U;
            }
        } else if (input->imu_degraded) {
            output->green_brightness_percent = led_firefly_brightness(
                input->elapsed_ms, IMU_DEGRADED_LED_CYCLE_MS);
        } else if (!input->external_power_present && input->battery_valid &&
                   isfinite(input->battery_voltage_v) &&
                   input->battery_voltage_v <=
                       SYSTEM_POLICY_LOW_BATTERY_THRESHOLD_V) {
            output->green_brightness_percent = UINT32_C(50) +
                led_firefly_brightness(input->elapsed_ms,
                                       LOW_BATTERY_LED_CYCLE_MS) /
                    UINT32_C(2);
        }

        if (!invalid_or_stale && input->ble_notify_active) {
            output->yellow_on =
                input->elapsed_ms % UINT32_C(1000) < UINT32_C(100);
        }
    }
    if (input->sw1_hold_ms > 0U) {
        output->green_brightness_percent =
            system_policy_power_off_brightness(input->sw1_hold_ms);
    }
}

uint32_t system_policy_power_on_brightness(uint32_t hold_ms,
                                           uint32_t required_hold_ms) {
    if (required_hold_ms == 0U || hold_ms >= required_hold_ms) {
        return UINT32_C(100);
    }
    return hold_ms * UINT32_C(100) / required_hold_ms;
}

uint32_t system_policy_power_off_brightness(uint32_t hold_ms) {
    if (hold_ms >= SYSTEM_POLICY_POWER_OFF_HOLD_MS) {
        return 0U;
    }
    return UINT32_C(100) - hold_ms * UINT32_C(100) /
           SYSTEM_POLICY_POWER_OFF_HOLD_MS;
}

uint32_t system_policy_shutdown_remaining_ms(int64_t deadline_us,
                                             int64_t now_us) {
    int64_t remaining_us = deadline_us - now_us;

    if (remaining_us <= 0) {
        return 0U;
    }
    if (remaining_us / INT64_C(1000) >= (int64_t) UINT32_MAX) {
        return UINT32_MAX;
    }
    remaining_us = (remaining_us + INT64_C(999)) / INT64_C(1000);
    if (remaining_us > (int64_t) UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t) remaining_us;
}

bool system_policy_can_start_shutdown_sound(bool workers_quiesced,
                                            uint32_t remaining_ms,
                                            uint32_t sound_duration_ms) {
    return workers_quiesced && remaining_ms > sound_duration_ms;
}
