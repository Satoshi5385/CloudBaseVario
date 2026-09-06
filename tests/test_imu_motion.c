#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "domain/imu_motion.h"

static void test_stationary_and_rotating_samples(void) {
    imu_motion_state_t state = {0};
    imu_motion_output_t output = {0};
    const float stationary_accel[3] = {0.0f, 0.0f, 9.80665f};
    const float stationary_gyro[3] = {0.0f, 0.0f, 0.0f};
    const float rotating_gyro[3] = {0.0f, 0.0f, 1.0f};

    assert(imu_motion_update(&state, stationary_accel, stationary_gyro,
                             1000, &output));
    assert(output.valid);
    assert(output.acceleration_rms_g == 0.0f);
    assert(output.gyro_rms_dps == 0.0f);
    assert(imu_motion_update(&state, stationary_accel, rotating_gyro,
                             501000, &output));
    assert(output.gyro_rms_dps > 10.0f);
}

static void test_horizontal_acceleration_is_visible(void) {
    imu_motion_state_t state = {0};
    imu_motion_output_t output = {0};
    const float stationary[3] = {0.0f, 0.0f, 9.80665f};
    const float horizontal[3] = {0.980665f, 0.0f, 9.80665f};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};

    assert(imu_motion_update(&state, stationary, gyro, 1000, &output));
    assert(imu_motion_update(&state, horizontal, gyro, 101000, &output));
    assert(output.acceleration_rms_g > 0.03f);
}

static void test_invalid_sample_resets(void) {
    imu_motion_state_t state = {0};
    imu_motion_output_t output = {0};
    const float acceleration[3] = {0.0f, 0.0f, 9.80665f};
    const float invalid_gyro[3] = {NAN, 0.0f, 0.0f};

    assert(!imu_motion_update(&state, acceleration, invalid_gyro,
                              1000, &output));
    assert(!state.initialized);
    assert(!output.valid);
}

int main(void) {
    test_stationary_and_rotating_samples();
    test_horizontal_acceleration_is_visible();
    test_invalid_sample_resets();
    puts("imu_motion tests passed");
    return 0;
}
