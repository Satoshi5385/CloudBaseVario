#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "domain/flight_state.h"

#define SECOND_US INT64_C(1000000)

static flight_state_input_t base_input(int64_t now_us) {
    flight_state_input_t input = {
        .now_us = now_us,
        .gps_speed_threshold_kmh = 10.0f,
        .stationary_confirm_seconds = 60U,
        .vario_available = true,
        .vario_timestamp_us = now_us,
        .altitude_m = 100.0f,
    };

    return input;
}

static void update_time(flight_state_input_t *input, int64_t now_us) {
    input->now_us = now_us;
    input->vario_timestamp_us = now_us;
}

static void test_stationary_boot_and_five_metre_boundary(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(1);

    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    update_time(&input, 60 * SECOND_US + 1);
    input.altitude_m = 105.0f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_STATIONARY);
    assert(output.stationary_since_us == input.now_us);
    assert(output.altitude_range_m == 5.0f);

    update_time(&input, input.now_us + 1);
    input.altitude_m = 105.01f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    assert((output.evidence & FLIGHT_EVIDENCE_ALTITUDE_RANGE) != 0U);
}

static void test_altitude_range_detects_descent_and_round_trip(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    flight_state_update(&detector, &input, &output);
    update_time(&input, 2 * SECOND_US);
    input.altitude_m = 94.9f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);

    flight_state_reset(&detector);
    input = base_input(SECOND_US);
    flight_state_update(&detector, &input, &output);
    update_time(&input, 2 * SECOND_US);
    input.altitude_m = 103.0f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    update_time(&input, 3 * SECOND_US);
    input.altitude_m = 97.9f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
}

static void test_gps_two_updates_or_one_second(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    input.gps_available = true;
    input.gps_speed_kmh = 10.0f;
    input.gps_sequence = 1U;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    assert(output.gps_high_speed_pending);
    assert(output.gps_high_speed_updates == 1U);
    update_time(&input, SECOND_US + 100000);
    input.gps_sequence = 2U;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    assert((output.evidence & FLIGHT_EVIDENCE_GPS_SPEED) != 0U);

    flight_state_reset(&detector);
    input = base_input(SECOND_US);
    input.gps_available = true;
    input.gps_speed_kmh = 10.0f;
    input.gps_sequence = 1U;
    flight_state_update(&detector, &input, &output);
    update_time(&input, 2 * SECOND_US - 1);
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    update_time(&input, 2 * SECOND_US);
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    assert(output.gps_high_speed_elapsed_seconds == 1U);
}

static void test_gps_pending_blocks_stationary_confirmation(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(1);

    flight_state_update(&detector, &input, &output);
    update_time(&input, 60 * SECOND_US);
    input.gps_available = true;
    input.gps_speed_kmh = 10.0f;
    input.gps_sequence = 1U;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    assert(output.stationary_candidate_elapsed_seconds == 0U);

    update_time(&input, 60 * SECOND_US + 1);
    input.gps_speed_kmh = 9.9f;
    input.gps_sequence = 2U;
    flight_state_update(&detector, &input, &output);
    update_time(&input, 120 * SECOND_US + 1);
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_STATIONARY);
}

static void test_altitude_evidence_wins_during_gps_confirmation(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    flight_state_update(&detector, &input, &output);
    update_time(&input, 2 * SECOND_US);
    input.altitude_m = 105.1f;
    input.gps_available = true;
    input.gps_speed_kmh = 10.0f;
    input.gps_sequence = 1U;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    assert((output.evidence & FLIGHT_EVIDENCE_ALTITUDE_RANGE) != 0U);
    assert(output.gps_high_speed_pending);
}

static void test_flying_uses_configured_stationary_confirmation(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    input.stationary_confirm_seconds = 2U;
    flight_state_update(&detector, &input, &output);
    update_time(&input, 2 * SECOND_US);
    input.altitude_m = 105.1f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);

    update_time(&input, 3 * SECOND_US);
    flight_state_update(&detector, &input, &output);
    update_time(&input, 5 * SECOND_US - 1);
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    update_time(&input, 5 * SECOND_US);
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_STATIONARY);
    assert(output.stationary_since_us == 5 * SECOND_US);
}

static void test_invalid_vario_and_time_reversal_are_unknown(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    input.vario_available = false;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    assert(!output.vario_used);

    input.gps_available = true;
    input.gps_speed_kmh = 20.0f;
    input.gps_sequence = 1U;
    flight_state_update(&detector, &input, &output);
    update_time(&input, SECOND_US + 1);
    input.vario_available = false;
    input.gps_sequence = 2U;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);

    input.now_us--;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    assert(output.stationary_candidate_elapsed_seconds == 0U);
}

static void test_gps_loss_falls_back_to_altitude(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    input.stationary_confirm_seconds = 2U;
    input.gps_available = true;
    input.gps_speed_kmh = 20.0f;
    input.gps_sequence = 1U;
    flight_state_update(&detector, &input, &output);
    update_time(&input, SECOND_US + 1);
    input.gps_sequence = 2U;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);

    update_time(&input, 2 * SECOND_US);
    input.gps_available = false;
    flight_state_update(&detector, &input, &output);
    update_time(&input, 4 * SECOND_US);
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_STATIONARY);
}

int main(void) {
    test_stationary_boot_and_five_metre_boundary();
    test_altitude_range_detects_descent_and_round_trip();
    test_gps_two_updates_or_one_second();
    test_gps_pending_blocks_stationary_confirmation();
    test_altitude_evidence_wins_during_gps_confirmation();
    test_flying_uses_configured_stationary_confirmation();
    test_invalid_vario_and_time_reversal_are_unknown();
    test_gps_loss_falls_back_to_altitude();
    puts("flight_state tests passed");
    return 0;
}
