#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "domain/auto_power_off.h"

#define MINUTE_US INT64_C(60000000)

static void test_disabled(void) {
    auto_power_off_state_t state = {0};

    assert(!auto_power_off_update(&state, 0U, false, true, 100.0f, 0));
    assert(!state.tracking);
}

static void test_boundary_and_single_trigger(void) {
    auto_power_off_state_t state = {0};

    assert(!auto_power_off_update(&state, 1U, false, true, 100.0f, 0));
    assert(!auto_power_off_update(&state, 1U, false, true, 110.0f,
                                  MINUTE_US - 1));
    assert(auto_power_off_update(&state, 1U, false, true, 105.0f,
                                 MINUTE_US));
    assert(!auto_power_off_update(&state, 1U, false, true, 105.0f,
                                  MINUTE_US + 1));
}

static void test_range_exceeded_restarts_window(void) {
    auto_power_off_state_t state = {0};

    assert(!auto_power_off_update(&state, 1U, false, true, 100.0f, 0));
    assert(!auto_power_off_update(&state, 1U, false, true, 110.01f,
                                  MINUTE_US / 2));
    assert(state.minimum_altitude_m == 110.01f);
    assert(state.maximum_altitude_m == 110.01f);
    assert(!auto_power_off_update(&state, 1U, false, true, 108.0f,
                                  MINUTE_US + MINUTE_US / 2 - 1));
    assert(auto_power_off_update(&state, 1U, false, true, 108.0f,
                                 MINUTE_US + MINUTE_US / 2));
}

static void test_invalid_and_external_power_reset(void) {
    auto_power_off_state_t state = {0};

    assert(!auto_power_off_update(&state, 1U, false, true, 100.0f, 0));
    assert(!auto_power_off_update(&state, 1U, false, false, 100.0f,
                                  MINUTE_US / 2));
    assert(!state.tracking);
    assert(!auto_power_off_update(&state, 1U, false, true, 101.0f,
                                  MINUTE_US));
    assert(!auto_power_off_update(&state, 1U, true, true, 101.0f,
                                  MINUTE_US + 1));
    assert(!state.tracking);
    assert(!auto_power_off_update(&state, 1U, false, true, 101.0f,
                                  MINUTE_US * 2));
    assert(!auto_power_off_update(&state, 1U, false, true, NAN,
                                  MINUTE_US * 3));
    assert(!state.tracking);
}

static void test_setting_change_and_time_reversal_reset(void) {
    auto_power_off_state_t state = {0};

    assert(!auto_power_off_update(&state, 1U, false, true, 100.0f,
                                  MINUTE_US));
    assert(!auto_power_off_update(&state, 2U, false, true, 100.0f,
                                  MINUTE_US * 2));
    assert(state.started_us == MINUTE_US * 2);
    assert(!auto_power_off_update(&state, 2U, false, true, 100.0f,
                                  MINUTE_US));
    assert(state.started_us == MINUTE_US);
}

static void test_multiple_movements_restart_each_time(void) {
    auto_power_off_state_t state = {0};

    assert(!auto_power_off_update(&state, 1U, false, true, 0.0f, 0));
    assert(!auto_power_off_update(&state, 1U, false, true, 10.1f,
                                  MINUTE_US / 4));
    assert(!auto_power_off_update(&state, 1U, false, true, -0.1f,
                                  MINUTE_US / 2));
    assert(!auto_power_off_update(&state, 1U, false, true, -0.1f,
                                  MINUTE_US + MINUTE_US / 2 - 1));
    assert(auto_power_off_update(&state, 1U, false, true, -0.1f,
                                 MINUTE_US + MINUTE_US / 2));
}

int main(void) {
    test_disabled();
    test_boundary_and_single_trigger();
    test_range_exceeded_restarts_window();
    test_invalid_and_external_power_reset();
    test_setting_change_and_time_reversal_reset();
    test_multiple_movements_restart_each_time();
    puts("auto_power_off tests passed");
    return 0;
}
