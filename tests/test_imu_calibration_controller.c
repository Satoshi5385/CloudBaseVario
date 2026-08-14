#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "domain/imu_calibration_controller.h"

static imu_axis_map_t identity_axis_map(void) {
    imu_axis_map_t map = {0};

    for (uint32_t axis = 0U; axis < IMU_AXIS_COUNT; axis++) {
        map.accel_source[axis] = axis;
        map.accel_sign[axis] = 1.0f;
        map.gyro_source[axis] = axis;
        map.gyro_sign[axis] = 1.0f;
    }
    return map;
}

static void test_skip_is_boot_local(void) {
    imu_calibration_controller_t controller = {0};

    imu_calibration_controller_init(&controller, NULL);
    assert(imu_calibration_controller_required(&controller));
    assert(imu_calibration_controller_request_skip(&controller));
    assert(imu_calibration_controller_skipped(&controller));
    assert(!imu_calibration_controller_required(&controller));
    assert(!imu_calibration_controller_save_pending(&controller));
}

static void test_capture_retry_and_commit(void) {
    imu_calibration_controller_t controller = {0};
    imu_axis_map_t map = identity_axis_map();
    imu_sample_t sample = {
        .accel_mps2 = {0.0f, 0.0f, 9.80665f},
        .gyro_radps = {0.0f, 0.0f, 0.0f},
        .valid = true,
    };
    bool captured = false;

    imu_calibration_controller_init(&controller, NULL);
    for (uint32_t index = 0U;
         index < IMU_ACCEL_CALIBRATION_SAMPLE_COUNT; index++) {
        sample.timestamp_us = (int64_t) (index + 1U) * INT64_C(2500);
        captured = imu_calibration_controller_process_sample(
            &controller, &sample, &map, sample.timestamp_us);
        assert(captured ==
               (index + 1U == IMU_ACCEL_CALIBRATION_SAMPLE_COUNT));
    }
    assert(imu_calibration_controller_save_due(
        &controller, sample.timestamp_us));
    assert(imu_calibration_controller_pending(&controller) != NULL);

    imu_calibration_controller_save_failed(
        &controller, sample.timestamp_us, INT64_C(2000000));
    assert(!imu_calibration_controller_save_due(
        &controller, sample.timestamp_us + INT64_C(1999999)));
    assert(imu_calibration_controller_save_due(
        &controller, sample.timestamp_us + INT64_C(2000000)));

    imu_calibration_controller_save_succeeded(&controller);
    assert(!imu_calibration_controller_required(&controller));
    assert(!imu_calibration_controller_save_pending(&controller));
    assert(imu_calibration_controller_persisted(&controller) != NULL);
}

int main(void) {
    test_skip_is_boot_local();
    test_capture_retry_and_commit();
    puts("imu calibration controller tests passed");
    return 0;
}
