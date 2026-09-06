#include "domain/imu_motion.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_MOTION_GRAVITY_MPS2 9.80665f
#define IMU_MOTION_RADIANS_TO_DEGREES 57.29577951308232f
#define IMU_MOTION_TIME_CONSTANT_SECONDS 0.5f
#define IMU_MOTION_MAX_SAMPLE_GAP_US INT64_C(100000)

static bool vector_is_finite(const float vector[IMU_MOTION_AXIS_COUNT]) {
    if (vector == NULL) {
        return false;
    }
    for (size_t axis = 0U; axis < IMU_MOTION_AXIS_COUNT; axis++) {
        if (!isfinite(vector[axis])) {
            return false;
        }
    }
    return true;
}

static float vector_norm(const float vector[IMU_MOTION_AXIS_COUNT]) {
    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] +
                 vector[2] * vector[2]);
}

void imu_motion_reset(imu_motion_state_t *state) {
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

bool imu_motion_update(imu_motion_state_t *state,
                       const float acceleration_mps2[IMU_MOTION_AXIS_COUNT],
                       const float gyro_radps[IMU_MOTION_AXIS_COUNT],
                       int64_t timestamp_us,
                       imu_motion_output_t *output) {
    float gyro_norm_dps = 0.0f;
    float dt_seconds = 0.0f;
    float alpha = 0.0f;
    float acceleration_variance_g2 = 0.0f;

    if (state == NULL || output == NULL || timestamp_us <= 0 ||
        !vector_is_finite(acceleration_mps2) ||
        !vector_is_finite(gyro_radps)) {
        if (output != NULL) {
            memset(output, 0, sizeof(*output));
        }
        if (state != NULL) {
            imu_motion_reset(state);
        }
        return false;
    }
    memset(output, 0, sizeof(*output));
    gyro_norm_dps =
        vector_norm(gyro_radps) * IMU_MOTION_RADIANS_TO_DEGREES;
    if (!isfinite(gyro_norm_dps)) {
        imu_motion_reset(state);
        return false;
    }

    if (!state->initialized ||
        timestamp_us <= state->previous_timestamp_us ||
        timestamp_us - state->previous_timestamp_us >
            IMU_MOTION_MAX_SAMPLE_GAP_US) {
        for (size_t axis = 0U; axis < IMU_MOTION_AXIS_COUNT; axis++) {
            state->acceleration_mean_g[axis] =
                acceleration_mps2[axis] / IMU_MOTION_GRAVITY_MPS2;
            state->acceleration_variance_g2[axis] = 0.0f;
        }
        state->gyro_mean_square_dps2 = gyro_norm_dps * gyro_norm_dps;
        state->previous_timestamp_us = timestamp_us;
        state->initialized = true;
    } else {
        dt_seconds =
            (float) (timestamp_us - state->previous_timestamp_us) /
            1000000.0f;
        state->previous_timestamp_us = timestamp_us;
        alpha = dt_seconds /
                (IMU_MOTION_TIME_CONSTANT_SECONDS + dt_seconds);
        for (size_t axis = 0U; axis < IMU_MOTION_AXIS_COUNT; axis++) {
            float acceleration_g =
                acceleration_mps2[axis] / IMU_MOTION_GRAVITY_MPS2;
            float delta_g = acceleration_g - state->acceleration_mean_g[axis];

            state->acceleration_mean_g[axis] += alpha * delta_g;
            state->acceleration_variance_g2[axis] =
                (1.0f - alpha) *
                (state->acceleration_variance_g2[axis] +
                 alpha * delta_g * delta_g);
        }
        state->gyro_mean_square_dps2 +=
            alpha * (gyro_norm_dps * gyro_norm_dps -
                     state->gyro_mean_square_dps2);
    }
    for (size_t axis = 0U; axis < IMU_MOTION_AXIS_COUNT; axis++) {
        if (!isfinite(state->acceleration_variance_g2[axis]) ||
            state->acceleration_variance_g2[axis] < 0.0f) {
            imu_motion_reset(state);
            return false;
        }
        acceleration_variance_g2 +=
            state->acceleration_variance_g2[axis];
    }
    if (!isfinite(state->gyro_mean_square_dps2) ||
        state->gyro_mean_square_dps2 < 0.0f) {
        imu_motion_reset(state);
        return false;
    }
    output->acceleration_rms_g = sqrtf(acceleration_variance_g2);
    output->gyro_rms_dps = sqrtf(state->gyro_mean_square_dps2);
    output->valid = isfinite(output->acceleration_rms_g) &&
                    isfinite(output->gyro_rms_dps);
    return output->valid;
}
