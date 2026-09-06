#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "domain/flight_state.h"

#define SECOND_US INT64_C(1000000)

static flight_state_input_t base_input(int64_t now_us) {
    flight_state_input_t input = {
        .now_us = now_us,
        .climb_rate_threshold_mps = 0.5f,
        .gps_speed_threshold_kmh = 10.0f,
        .stationary_confirm_seconds = 60U,
        .vario_available = true,
        .vario_timestamp_us = now_us,
        .altitude_m = 100.0f,
        .climb_rate_mps = 0.0f,
    };

    return input;
}

static void test_stationary_boot_and_altitude_boundary(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(1);

    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    input.now_us = 60 * SECOND_US;
    input.vario_timestamp_us = input.now_us;
    input.altitude_m = 110.0f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    input.now_us = 60 * SECOND_US + 1;
    input.vario_timestamp_us = input.now_us;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_STATIONARY);
    assert(output.stationary_since_us == 1);
    input.now_us++;
    input.vario_timestamp_us = input.now_us;
    input.altitude_m = 110.01f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    assert((output.evidence & FLIGHT_EVIDENCE_ALTITUDE_RANGE) != 0U);
}

static void test_sustained_vario_and_imu_evidence(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    input.climb_rate_mps = 0.5f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    input.now_us += FLIGHT_STATE_VARIO_CONFIRM_US;
    input.vario_timestamp_us = input.now_us;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    assert((output.evidence & FLIGHT_EVIDENCE_CLIMB_RATE) != 0U);

    flight_state_reset(&detector);
    input = base_input(SECOND_US);
    input.imu_available = true;
    input.imu_timestamp_us = input.now_us;
    input.imu_acceleration_rms_g = 0.03f;
    flight_state_update(&detector, &input, &output);
    input.now_us += FLIGHT_STATE_IMU_CONFIRM_US;
    input.vario_timestamp_us = input.now_us;
    input.imu_timestamp_us = input.now_us;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    assert((output.evidence & FLIGHT_EVIDENCE_IMU_ACTIVITY) != 0U);
}

static void test_two_gps_updates_and_invalid_baro(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    input.vario_available = false;
    input.gps_available = true;
    input.gps_speed_kmh = 10.0f;
    input.gps_sequence = 1U;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    input.now_us += SECOND_US;
    input.gps_sequence = 2U;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    assert((output.evidence & FLIGHT_EVIDENCE_GPS_SPEED) != 0U);
}

static void test_imu_evidence_with_invalid_baro(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    input.vario_available = false;
    input.imu_available = true;
    input.imu_timestamp_us = input.now_us;
    input.imu_gyro_rms_dps = 10.0f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    input.now_us += FLIGHT_STATE_IMU_CONFIRM_US;
    input.imu_timestamp_us = input.now_us;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    assert((output.evidence & FLIGHT_EVIDENCE_IMU_ACTIVITY) != 0U);
}

static void test_unavailable_optional_inputs_are_omitted(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(1);

    input.gps_available = false;
    input.imu_available = true;
    input.imu_timestamp_us = input.now_us;
    input.imu_acceleration_rms_g = 0.0f;
    input.imu_gyro_rms_dps = 0.0f;
    flight_state_update(&detector, &input, &output);
    input.now_us += 30 * SECOND_US;
    input.vario_timestamp_us = input.now_us;
    input.imu_available = false;
    flight_state_update(&detector, &input, &output);
    input.now_us += 30 * SECOND_US;
    input.vario_timestamp_us = input.now_us;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_STATIONARY);
    assert(!output.gps_used);
    assert(!output.imu_used);
}

static void test_landing_requires_120_seconds(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    input.gps_available = true;
    input.gps_speed_kmh = 20.0f;
    input.gps_sequence = 1U;
    flight_state_update(&detector, &input, &output);
    input.now_us += SECOND_US;
    input.vario_timestamp_us = input.now_us;
    input.gps_sequence = 2U;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);

    input.now_us += SECOND_US;
    input.vario_timestamp_us = input.now_us;
    input.gps_sequence = 3U;
    input.gps_speed_kmh = 0.0f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    input.now_us += FLIGHT_STATE_LANDING_CONFIRM_US - 1;
    input.vario_timestamp_us = input.now_us;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    input.now_us++;
    input.vario_timestamp_us = input.now_us;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_STATIONARY);
    assert(output.stationary_elapsed_seconds == 120U);
}

static void test_ambiguous_and_time_reversal_are_unknown(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    input.climb_rate_mps = 0.3f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    assert(output.stationary_since_us == 0);
    input.now_us--;
    input.vario_timestamp_us = input.now_us;
    input.climb_rate_mps = 0.0f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    assert(output.stationary_elapsed_seconds == 0U);
}

static void test_flying_to_ambiguous_is_unknown(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    input.gps_available = true;
    input.gps_speed_kmh = 10.0f;
    input.gps_sequence = 1U;
    flight_state_update(&detector, &input, &output);
    input.now_us += SECOND_US;
    input.vario_timestamp_us = input.now_us;
    input.gps_sequence = 2U;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);

    input.now_us += SECOND_US;
    input.vario_timestamp_us = input.now_us;
    input.gps_sequence = 3U;
    input.gps_speed_kmh = 7.0f;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_UNKNOWN);
    assert(output.stationary_since_us == 0);
}

static void test_evidence_bits_accumulate_while_flying(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    input.gps_available = true;
    input.gps_speed_kmh = 10.0f;
    input.gps_sequence = 1U;
    flight_state_update(&detector, &input, &output);
    input.now_us += SECOND_US;
    input.vario_timestamp_us = input.now_us;
    input.gps_sequence = 2U;
    flight_state_update(&detector, &input, &output);
    assert(output.evidence == FLIGHT_EVIDENCE_GPS_SPEED);

    input.gps_available = false;
    input.climb_rate_mps = 0.5f;
    input.now_us += SECOND_US;
    input.vario_timestamp_us = input.now_us;
    flight_state_update(&detector, &input, &output);
    input.now_us += FLIGHT_STATE_VARIO_CONFIRM_US;
    input.vario_timestamp_us = input.now_us;
    flight_state_update(&detector, &input, &output);
    assert((output.evidence & FLIGHT_EVIDENCE_GPS_SPEED) != 0U);
    assert((output.evidence & FLIGHT_EVIDENCE_CLIMB_RATE) != 0U);
}

static void test_gps_loss_uses_baro_landing_confirmation(void) {
    flight_state_detector_t detector = {0};
    flight_state_output_t output = {0};
    flight_state_input_t input = base_input(SECOND_US);

    input.gps_available = true;
    input.gps_speed_kmh = 10.0f;
    input.gps_sequence = 1U;
    flight_state_update(&detector, &input, &output);
    input.now_us += SECOND_US;
    input.vario_timestamp_us = input.now_us;
    input.gps_sequence = 2U;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);

    input.now_us += SECOND_US;
    input.vario_timestamp_us = input.now_us;
    input.gps_available = false;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_FLYING);
    assert(!output.gps_used);
    input.now_us += FLIGHT_STATE_LANDING_CONFIRM_US;
    input.vario_timestamp_us = input.now_us;
    flight_state_update(&detector, &input, &output);
    assert(output.state == FLIGHT_STATE_STATIONARY);
}

int main(void) {
    test_stationary_boot_and_altitude_boundary();
    test_sustained_vario_and_imu_evidence();
    test_two_gps_updates_and_invalid_baro();
    test_imu_evidence_with_invalid_baro();
    test_unavailable_optional_inputs_are_omitted();
    test_landing_requires_120_seconds();
    test_ambiguous_and_time_reversal_are_unknown();
    test_flying_to_ambiguous_is_unknown();
    test_evidence_bits_accumulate_while_flying();
    test_gps_loss_uses_baro_landing_confirmation();
    puts("flight_state tests passed");
    return 0;
}
