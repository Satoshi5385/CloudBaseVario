#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "domain/auto_power_off.h"

#define MINUTE_US INT64_C(60000000)

static void test_disabled_and_nonstationary_reset(void) {
    auto_power_off_state_t state = {0};

    assert(!auto_power_off_update(&state, 0U, false, true, 1, MINUTE_US));
    assert(!state.tracking);
    assert(!auto_power_off_update(&state, 1U, false, false, 0, MINUTE_US));
    assert(!state.tracking);
}

static void test_stationary_boundary_starts_after_confirmation(void) {
    auto_power_off_state_t state = {0};
    int64_t stationary_since_us = INT64_C(1000000);

    assert(!auto_power_off_update(&state, 1U, false, true,
                                  stationary_since_us,
                                  stationary_since_us + MINUTE_US - 1));
    assert(auto_power_off_elapsed_seconds(
               &state, stationary_since_us + MINUTE_US - 1) == 59U);
    assert(auto_power_off_update(&state, 1U, false, true,
                                 stationary_since_us,
                                 stationary_since_us + MINUTE_US));
    assert(!auto_power_off_update(&state, 1U, false, true,
                                  stationary_since_us,
                                  stationary_since_us + MINUTE_US + 1));
}

static void test_external_power_and_candidate_change_reset(void) {
    auto_power_off_state_t state = {0};

    assert(!auto_power_off_update(&state, 1U, false, true, 1,
                                  MINUTE_US / 2));
    assert(!auto_power_off_update(&state, 1U, true, true, 1,
                                  MINUTE_US));
    assert(!state.tracking);
    assert(!auto_power_off_update(&state, 1U, false, true, MINUTE_US,
                                  MINUTE_US + 1));
    assert(state.started_us == MINUTE_US);
    assert(!auto_power_off_update(&state, 1U, false, true,
                                  MINUTE_US + 1,
                                  MINUTE_US * 2));
    assert(state.started_us == MINUTE_US + 1);
}

static void test_external_power_does_not_count_stationary_time(void) {
    auto_power_off_state_t state = {0};
    int64_t stationary_since_us = 1;

    assert(!auto_power_off_update(&state, 1U, true, true,
                                  stationary_since_us, MINUTE_US));
    assert(!state.tracking);
    assert(!auto_power_off_update(&state, 1U, false, true,
                                  stationary_since_us, MINUTE_US + 1));
    assert(state.started_us == MINUTE_US + 1);
    assert(!auto_power_off_update(&state, 1U, false, true,
                                  stationary_since_us,
                                  2 * MINUTE_US));
    assert(auto_power_off_update(&state, 1U, false, true,
                                 stationary_since_us,
                                 2 * MINUTE_US + 1));
}

static void test_setting_change_and_invalid_time_reset(void) {
    auto_power_off_state_t state = {0};

    assert(!auto_power_off_update(&state, 1U, false, true, 1,
                                  MINUTE_US / 2));
    assert(!auto_power_off_update(&state, 2U, false, true, MINUTE_US,
                                  MINUTE_US));
    assert(state.started_us == MINUTE_US);
    assert(!auto_power_off_update(&state, 2U, false, true,
                                  MINUTE_US + 1, MINUTE_US - 1));
    assert(!state.tracking);
    assert(!auto_power_off_update(&state, 2U, false, true,
                                  MINUTE_US + 2, MINUTE_US + 1));
    assert(!state.tracking);

    auto_power_off_reset(&state);
    assert(!auto_power_off_update(&state, 1U, false, true, MINUTE_US,
                                  MINUTE_US + MINUTE_US / 2));
    assert(!auto_power_off_update(&state, 1U, false, true, MINUTE_US,
                                  MINUTE_US + MINUTE_US / 4));
    assert(state.started_us == MINUTE_US + MINUTE_US / 4);
}

int main(void) {
    test_disabled_and_nonstationary_reset();
    test_stationary_boundary_starts_after_confirmation();
    test_external_power_and_candidate_change_reset();
    test_external_power_does_not_count_stationary_time();
    test_setting_change_and_invalid_time_reset();
    puts("auto_power_off tests passed");
    return 0;
}
