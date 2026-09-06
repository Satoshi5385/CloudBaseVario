#pragma once

#include <stdbool.h>
#include <stdint.h>

#define IMU_MOTION_AXIS_COUNT UINT32_C(3)

typedef struct {
    float acceleration_mean_g[IMU_MOTION_AXIS_COUNT];
    float acceleration_variance_g2[IMU_MOTION_AXIS_COUNT];
    float gyro_mean_square_dps2;
    int64_t previous_timestamp_us;
    bool initialized;
} imu_motion_state_t;

typedef struct {
    float acceleration_rms_g;
    float gyro_rms_dps;
    bool valid;
} imu_motion_output_t;

/** Reset the bounded, short-window IMU activity estimator. */
void imu_motion_reset(imu_motion_state_t *state);

/**
 * Update 0.5-second exponential activity estimates without integrating speed.
 * Acceleration is expressed in m/s^2 and angular rate in rad/s.
 */
bool imu_motion_update(imu_motion_state_t *state,
                       const float acceleration_mps2[IMU_MOTION_AXIS_COUNT],
                       const float gyro_radps[IMU_MOTION_AXIS_COUNT],
                       int64_t timestamp_us,
                       imu_motion_output_t *output);
