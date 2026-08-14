#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "domain/system_policy.h"

static system_policy_actions_t sample(system_policy_state_t *state,
                                      bool sw1, bool sw2, bool sw3,
                                      bool calibration_required) {
    system_policy_input_t input = {
        .sw1_pressed = sw1,
        .sw2_pressed = sw2,
        .sw3_pressed = sw3,
        .imu_calibration_required = calibration_required,
    };
    system_policy_actions_t actions = {0};

    system_policy_step(state, &input, &actions);
    return actions;
}

static system_policy_actions_t settle(system_policy_state_t *state,
                                      bool sw1, bool sw2, bool sw3,
                                      bool calibration_required) {
    system_policy_actions_t actions = {0};

    for (uint32_t index = 0U; index < 4U; index++) {
        actions = sample(state, sw1, sw2, sw3,
                         calibration_required);
    }
    return actions;
}

static void init_released(system_policy_state_t *state) {
    system_policy_input_t input = {0};

    system_policy_init(state, &input);
    (void) settle(state, false, false, false, false);
}

static void test_startup_press_is_not_power_off(void) {
    system_policy_state_t state = {0};
    system_policy_input_t input = {.sw1_pressed = true};

    system_policy_init(&state, &input);
    for (uint32_t index = 0U; index < 250U; index++) {
        system_policy_actions_t actions = sample(
            &state, true, false, false, false);

        assert(!actions.request_power_off);
        assert(!actions.advance_volume);
    }
}

static void test_sw1_short_and_two_second_hold(void) {
    system_policy_state_t state = {0};
    system_policy_actions_t actions = {0};
    bool power_off_seen = false;

    init_released(&state);
    actions = settle(&state, true, false, false, false);
    assert(!actions.advance_volume);
    actions = settle(&state, false, false, false, false);
    assert(actions.advance_volume);

    (void) settle(&state, true, false, false, false);
    for (uint32_t index = 0U; index < 250U; index++) {
        actions = sample(&state, true, false, false, false);
        if (actions.request_power_off) {
            power_off_seen = true;
            assert(actions.sw1_hold_ms ==
                   SYSTEM_POLICY_POWER_OFF_HOLD_MS);
            break;
        }
    }
    assert(power_off_seen);
    actions = settle(&state, false, false, false, false);
    assert(!actions.advance_volume);
}

static void test_sw2_and_sw3_actions(void) {
    system_policy_state_t state = {0};
    system_policy_actions_t actions = {0};

    init_released(&state);
    actions = settle(&state, false, true, false, false);
    assert(actions.toggle_sink);
    actions = sample(&state, false, true, false, false);
    assert(!actions.toggle_sink);

    (void) settle(&state, false, false, false, false);
    actions = settle(&state, false, false, true, false);
    assert(actions.select_next_profile);
}

static void test_calibration_short_press_and_skip(void) {
    system_policy_state_t state = {0};
    system_policy_actions_t actions = {0};
    bool skip_seen = false;

    init_released(&state);
    actions = settle(&state, false, false, true, true);
    assert(!actions.select_next_profile);
    assert(!actions.request_calibration_skip);
    actions = settle(&state, false, false, false, true);
    assert(actions.select_next_profile);

    actions = settle(&state, false, false, true, true);
    assert(!actions.select_next_profile);
    for (uint32_t index = 0U; index < 350U; index++) {
        actions = sample(&state, false, false, true, true);
        if (actions.request_calibration_skip) {
            skip_seen = true;
            assert(actions.sw3_hold_ms ==
                   SYSTEM_POLICY_IMU_SKIP_HOLD_MS);
            break;
        }
    }
    assert(skip_seen);
    actions = settle(&state, false, false, false, true);
    assert(!actions.select_next_profile);
}

static void test_led_priority_table(void) {
    const struct {
        system_led_policy_input_t input;
        uint32_t green;
        bool yellow;
    } cases[] = {
        {{.elapsed_ms = 0U}, 100U, false},
        {{.elapsed_ms = 0U, .fatal = true, .fatal_bmp581 = true},
         0U, true},
        {{.elapsed_ms = 0U, .bmp581_startup_complete = true,
          .vario_available = true, .climb_rate_valid = true},
         0U, true},
        {{.elapsed_ms = 1000U, .bmp581_startup_complete = true,
          .vario_available = true, .pressure_valid = true,
          .climb_rate_valid = true, .bmp581_recovering = true,
          .imu_calibrating = true, .imu_degraded = true},
         0U, false},
        {{.elapsed_ms = 1000U, .bmp581_startup_complete = true,
          .vario_available = true, .pressure_valid = true,
          .climb_rate_valid = true, .imu_calibrating = true,
          .estimator_warming_up = true, .imu_degraded = true},
         0U, false},
        {{.elapsed_ms = 500U, .bmp581_startup_complete = true,
          .vario_available = true, .pressure_valid = true,
          .climb_rate_valid = true, .battery_voltage_v = 3.3f,
          .battery_valid = true},
         50U, false},
        {{.elapsed_ms = 500U, .bmp581_startup_complete = true,
          .vario_available = true, .pressure_valid = true,
          .climb_rate_valid = true, .battery_voltage_v = 3.3001f,
          .battery_valid = true},
         100U, false},
        {{.elapsed_ms = 0U, .bmp581_startup_complete = true,
          .vario_available = true, .pressure_valid = true,
          .climb_rate_valid = true, .external_power_present = true,
          .ble_notify_active = true},
         100U, true},
        {{.elapsed_ms = 0U, .bmp581_startup_complete = true,
          .storage_mode_active = true},
         20U, true},
        {{.elapsed_ms = 100U, .bmp581_startup_complete = true,
          .storage_mode_active = true},
         20U, false},
        {{.elapsed_ms = 0U, .sw1_hold_ms = 1000U,
          .bmp581_startup_complete = true, .storage_mode_active = true},
         50U, true},
        {{.elapsed_ms = 0U, .sw1_hold_ms = 1000U,
          .bmp581_startup_complete = true, .vario_available = true,
          .pressure_valid = true, .climb_rate_valid = true},
         50U, false},
    };

    for (size_t index = 0U; index < sizeof(cases) / sizeof(cases[0]);
         index++) {
        system_led_policy_output_t output = {0};

        system_policy_select_leds(&cases[index].input, &output);
        assert(output.green_brightness_percent == cases[index].green);
        assert(output.yellow_on == cases[index].yellow);
    }
}

static void test_shutdown_deadline_policy(void) {
    assert(system_policy_shutdown_remaining_ms(
               INT64_C(2000001), INT64_C(1000000)) == 1001U);
    assert(system_policy_shutdown_remaining_ms(
               INT64_C(1000000), INT64_C(1000000)) == 0U);
    assert(!system_policy_can_start_shutdown_sound(true, 380U, 380U));
    assert(system_policy_can_start_shutdown_sound(true, 381U, 380U));
    assert(!system_policy_can_start_shutdown_sound(false, 1000U, 380U));
    assert(system_policy_power_on_brightness(1000U, 2000U) == 50U);
    assert(system_policy_power_off_brightness(1000U) == 50U);
}

int main(void) {
    test_startup_press_is_not_power_off();
    test_sw1_short_and_two_second_hold();
    test_sw2_and_sw3_actions();
    test_calibration_short_press_and_skip();
    test_led_priority_table();
    test_shutdown_deadline_policy();
    puts("system_policy tests passed");
    return 0;
}
