#include "domain/imu_fusion.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define IMU_STANDARD_GRAVITY_MPS2 9.80665f
#define IMU_CALIBRATION_ACCEL_MIN_G 0.9f
#define IMU_CALIBRATION_ACCEL_MAX_G 1.1f
#define IMU_CALIBRATION_GYRO_MAX_RADPS 0.05235988f
#define IMU_ATTITUDE_MAX_DT_SECONDS 0.05f
#define IMU_RADIANS_TO_DEGREES 57.29577951308232f

static bool vector_is_finite(const float vector[IMU_AXIS_COUNT]) {
    if (vector == NULL) {
        return false;
    }
    for (size_t index = 0U; index < IMU_AXIS_COUNT; index++) {
        if (!isfinite(vector[index])) {
            return false;
        }
    }
    return true;
}

static float vector_norm(const float vector[IMU_AXIS_COUNT]) {
    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] +
                 vector[2] * vector[2]);
}

static bool quaternion_is_finite(const float quaternion[4]) {
    if (quaternion == NULL) {
        return false;
    }
    for (size_t index = 0U; index < 4U; index++) {
        if (!isfinite(quaternion[index])) {
            return false;
        }
    }
    return true;
}

static void reset_calibration_accumulators(imu_fusion_t *fusion) {
    if (fusion == NULL) {
        return;
    }
    memset(fusion->calibration_gyro_sum, 0,
           sizeof(fusion->calibration_gyro_sum));
    memset(fusion->calibration_accel_sum, 0,
           sizeof(fusion->calibration_accel_sum));
    fusion->calibration_samples = 0U;
}

void imu_fusion_reset(imu_fusion_t *fusion) {
    if (fusion == NULL) {
        return;
    }
    memset(fusion, 0, sizeof(*fusion));
    fusion->quaternion[0] = 1.0f;
}

static bool sample_is_stationary(const imu_sample_t *sample,
                                 float accel_norm_g) {
    if (sample == NULL || accel_norm_g < IMU_CALIBRATION_ACCEL_MIN_G ||
        accel_norm_g > IMU_CALIBRATION_ACCEL_MAX_G) {
        return false;
    }
    for (size_t index = 0U; index < IMU_AXIS_COUNT; index++) {
        if (fabsf(sample->gyro_radps[index]) >
            IMU_CALIBRATION_GYRO_MAX_RADPS) {
            return false;
        }
    }
    return true;
}

static bool initialize_attitude(imu_fusion_t *fusion) {
    float average_accel[IMU_AXIS_COUNT] = {0};
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float cos_roll = 0.0f;
    float sin_roll = 0.0f;
    float cos_pitch = 0.0f;
    float sin_pitch = 0.0f;

    if (fusion == NULL || fusion->calibration_samples == 0U) {
        return false;
    }
    for (size_t index = 0U; index < IMU_AXIS_COUNT; index++) {
        fusion->gyro_bias_radps[index] =
            (float) (fusion->calibration_gyro_sum[index] /
                     (double) fusion->calibration_samples);
        average_accel[index] =
            (float) (fusion->calibration_accel_sum[index] /
                     (double) fusion->calibration_samples);
    }

    roll_rad = atan2f(average_accel[1], average_accel[2]);
    pitch_rad = atan2f(
        -average_accel[0],
        sqrtf(average_accel[1] * average_accel[1] +
              average_accel[2] * average_accel[2]));
    cos_roll = cosf(roll_rad * 0.5f);
    sin_roll = sinf(roll_rad * 0.5f);
    cos_pitch = cosf(pitch_rad * 0.5f);
    sin_pitch = sinf(pitch_rad * 0.5f);

    fusion->quaternion[0] = cos_roll * cos_pitch;
    fusion->quaternion[1] = sin_roll * cos_pitch;
    fusion->quaternion[2] = cos_roll * sin_pitch;
    fusion->quaternion[3] = -sin_roll * sin_pitch;
    fusion->calibrated = true;
    fusion->attitude_valid = quaternion_is_finite(fusion->quaternion);
    return fusion->attitude_valid;
}

static bool update_calibration(imu_fusion_t *fusion,
                               const imu_sample_t *sample,
                               const app_config_t *config,
                               float accel_norm_g) {
    if (!sample_is_stationary(sample, accel_norm_g)) {
        reset_calibration_accumulators(fusion);
        return true;
    }

    for (size_t index = 0U; index < IMU_AXIS_COUNT; index++) {
        fusion->calibration_gyro_sum[index] += sample->gyro_radps[index];
        fusion->calibration_accel_sum[index] += sample->accel_mps2[index];
    }
    fusion->calibration_samples++;
    if (fusion->calibration_samples < config->imu_gyro_calibration_samples) {
        return true;
    }

    if (!initialize_attitude(fusion)) {
        imu_fusion_reset(fusion);
        return false;
    }
    fusion->previous_timestamp_us = sample->timestamp_us;
    return true;
}

static bool normalize_quaternion(imu_fusion_t *fusion) {
    float norm = 0.0f;

    norm = sqrtf(fusion->quaternion[0] * fusion->quaternion[0] +
                 fusion->quaternion[1] * fusion->quaternion[1] +
                 fusion->quaternion[2] * fusion->quaternion[2] +
                 fusion->quaternion[3] * fusion->quaternion[3]);
    if (!isfinite(norm) || norm <= 0.0f) {
        return false;
    }
    norm = 1.0f / norm;
    for (size_t index = 0U; index < 4U; index++) {
        fusion->quaternion[index] *= norm;
    }
    return quaternion_is_finite(fusion->quaternion);
}

static bool update_attitude(imu_fusion_t *fusion, const imu_sample_t *sample,
                            const app_config_t *config, float accel_norm_g) {
    float gyro[IMU_AXIS_COUNT] = {0};
    float accel[IMU_AXIS_COUNT] = {0};
    float half_gravity[IMU_AXIS_COUNT] = {0};
    float half_error[IMU_AXIS_COUNT] = {0};
    float dt_seconds = 0.0f;
    float q0 = fusion->quaternion[0];
    float q1 = fusion->quaternion[1];
    float q2 = fusion->quaternion[2];
    float q3 = fusion->quaternion[3];
    float previous_q0 = q0;
    float previous_q1 = q1;
    float previous_q2 = q2;
    int64_t delta_us = sample->timestamp_us - fusion->previous_timestamp_us;

    dt_seconds = (float) delta_us / 1000000.0f;
    if (delta_us <= 0 || !isfinite(dt_seconds) ||
        dt_seconds > IMU_ATTITUDE_MAX_DT_SECONDS) {
        return false;
    }
    fusion->previous_timestamp_us = sample->timestamp_us;

    for (size_t index = 0U; index < IMU_AXIS_COUNT; index++) {
        gyro[index] =
            sample->gyro_radps[index] - fusion->gyro_bias_radps[index];
        accel[index] = sample->accel_mps2[index];
    }

    if (accel_norm_g >= config->imu_accel_correction_min_g &&
        accel_norm_g <= config->imu_accel_correction_max_g) {
        float accel_norm_mps2 = accel_norm_g * IMU_STANDARD_GRAVITY_MPS2;

        for (size_t index = 0U; index < IMU_AXIS_COUNT; index++) {
            accel[index] /= accel_norm_mps2;
        }
        half_gravity[0] = q1 * q3 - q0 * q2;
        half_gravity[1] = q0 * q1 + q2 * q3;
        half_gravity[2] = q0 * q0 - 0.5f + q3 * q3;
        half_error[0] =
            accel[1] * half_gravity[2] - accel[2] * half_gravity[1];
        half_error[1] =
            accel[2] * half_gravity[0] - accel[0] * half_gravity[2];
        half_error[2] =
            accel[0] * half_gravity[1] - accel[1] * half_gravity[0];

        for (size_t index = 0U; index < IMU_AXIS_COUNT; index++) {
            if (config->imu_mahony_ki > 0.0f) {
                fusion->integral_feedback[index] +=
                    2.0f * config->imu_mahony_ki * half_error[index] *
                    dt_seconds;
            } else {
                fusion->integral_feedback[index] = 0.0f;
            }
            gyro[index] += fusion->integral_feedback[index] +
                           2.0f * config->imu_mahony_kp * half_error[index];
        }
    }

    for (size_t index = 0U; index < IMU_AXIS_COUNT; index++) {
        gyro[index] *= 0.5f * dt_seconds;
    }
    fusion->quaternion[0] +=
        -previous_q1 * gyro[0] - previous_q2 * gyro[1] - q3 * gyro[2];
    fusion->quaternion[1] +=
        previous_q0 * gyro[0] + previous_q2 * gyro[2] - q3 * gyro[1];
    fusion->quaternion[2] +=
        previous_q0 * gyro[1] - previous_q1 * gyro[2] + q3 * gyro[0];
    fusion->quaternion[3] +=
        previous_q0 * gyro[2] + previous_q1 * gyro[1] -
        previous_q2 * gyro[0];
    return normalize_quaternion(fusion);
}

static bool calculate_vertical_acceleration(const imu_fusion_t *fusion,
                                            const imu_sample_t *sample,
                                            float *vertical_accel_mps2) {
    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    float earth_z_mps2 = 0.0f;

    if (fusion == NULL || sample == NULL || vertical_accel_mps2 == NULL ||
        !fusion->attitude_valid) {
        return false;
    }
    q0 = fusion->quaternion[0];
    q1 = fusion->quaternion[1];
    q2 = fusion->quaternion[2];
    q3 = fusion->quaternion[3];
    earth_z_mps2 =
        2.0f * (q1 * q3 - q0 * q2) * sample->accel_mps2[0] +
        2.0f * (q0 * q1 + q2 * q3) * sample->accel_mps2[1] +
        (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) *
            sample->accel_mps2[2] -
        IMU_STANDARD_GRAVITY_MPS2;
    if (!isfinite(earth_z_mps2)) {
        return false;
    }
    *vertical_accel_mps2 = earth_z_mps2;
    return true;
}

static bool populate_attitude_output(const imu_fusion_t *fusion,
                                     imu_fusion_output_t *output) {
    float sin_pitch = 0.0f;
    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    if (fusion == NULL || output == NULL || !fusion->attitude_valid ||
        !quaternion_is_finite(fusion->quaternion)) {
        return false;
    }

    q0 = fusion->quaternion[0];
    q1 = fusion->quaternion[1];
    q2 = fusion->quaternion[2];
    q3 = fusion->quaternion[3];
    memcpy(output->quaternion, fusion->quaternion,
           sizeof(output->quaternion));

    sin_pitch = 2.0f * (q0 * q2 - q3 * q1);
    if (sin_pitch > 1.0f) {
        sin_pitch = 1.0f;
    } else if (sin_pitch < -1.0f) {
        sin_pitch = -1.0f;
    }
    output->roll_deg =
        atan2f(2.0f * (q0 * q1 + q2 * q3),
               1.0f - 2.0f * (q1 * q1 + q2 * q2)) *
        IMU_RADIANS_TO_DEGREES;
    output->pitch_deg = asinf(sin_pitch) * IMU_RADIANS_TO_DEGREES;
    output->yaw_deg =
        atan2f(2.0f * (q0 * q3 + q1 * q2),
               1.0f - 2.0f * (q2 * q2 + q3 * q3)) *
        IMU_RADIANS_TO_DEGREES;
    return isfinite(output->roll_deg) && isfinite(output->pitch_deg) &&
           isfinite(output->yaw_deg);
}

bool imu_fusion_apply_axis_map(const imu_sample_t *input,
                               const app_config_t *config,
                               imu_sample_t *output) {
    uint32_t accel_source[IMU_AXIS_COUNT] = {0};
    uint32_t gyro_source[IMU_AXIS_COUNT] = {0};
    float accel_sign[IMU_AXIS_COUNT] = {0};
    float gyro_sign[IMU_AXIS_COUNT] = {0};

    if (input == NULL || config == NULL || output == NULL || !input->valid ||
        !app_config_validate(config) ||
        !vector_is_finite(input->accel_mps2) ||
        !vector_is_finite(input->gyro_radps)) {
        return false;
    }
    accel_source[0] = config->imu_accel_x_source;
    accel_source[1] = config->imu_accel_y_source;
    accel_source[2] = config->imu_accel_z_source;
    gyro_source[0] = config->imu_gyro_x_source;
    gyro_source[1] = config->imu_gyro_y_source;
    gyro_source[2] = config->imu_gyro_z_source;
    accel_sign[0] = config->imu_accel_x_sign;
    accel_sign[1] = config->imu_accel_y_sign;
    accel_sign[2] = config->imu_accel_z_sign;
    gyro_sign[0] = config->imu_gyro_x_sign;
    gyro_sign[1] = config->imu_gyro_y_sign;
    gyro_sign[2] = config->imu_gyro_z_sign;

    memset(output, 0, sizeof(*output));
    for (size_t index = 0U; index < IMU_AXIS_COUNT; index++) {
        output->accel_mps2[index] =
            input->accel_mps2[accel_source[index]] * accel_sign[index];
        output->gyro_radps[index] =
            input->gyro_radps[gyro_source[index]] * gyro_sign[index];
    }
    output->timestamp_us = input->timestamp_us;
    output->valid = true;
    return true;
}

bool imu_fusion_update(imu_fusion_t *fusion, const imu_sample_t *sample,
                       const app_config_t *config,
                       imu_fusion_output_t *output) {
    float accel_norm_mps2 = 0.0f;

    if (fusion == NULL || sample == NULL || config == NULL || output == NULL) {
        return false;
    }
    memset(output, 0, sizeof(*output));
    if (!sample->valid || sample->timestamp_us <= 0 ||
        !vector_is_finite(sample->accel_mps2) ||
        !vector_is_finite(sample->gyro_radps)) {
        imu_fusion_reset(fusion);
        return false;
    }

    accel_norm_mps2 = vector_norm(sample->accel_mps2);
    output->accel_norm_g = accel_norm_mps2 / IMU_STANDARD_GRAVITY_MPS2;
    if (!isfinite(output->accel_norm_g)) {
        imu_fusion_reset(fusion);
        return false;
    }

    if (!fusion->calibrated) {
        if (!update_calibration(fusion, sample, config,
                                output->accel_norm_g)) {
            return false;
        }
        output->calibration_samples = fusion->calibration_samples;
        output->calibrated = fusion->calibrated;
        output->attitude_valid = fusion->attitude_valid;
        if (output->attitude_valid &&
            !populate_attitude_output(fusion, output)) {
            imu_fusion_reset(fusion);
            return false;
        }
        return true;
    }

    if (!update_attitude(fusion, sample, config, output->accel_norm_g)) {
        imu_fusion_reset(fusion);
        return false;
    }
    fusion->attitude_valid = true;
    output->calibration_samples = fusion->calibration_samples;
    output->calibrated = true;
    output->attitude_valid = true;
    if (!populate_attitude_output(fusion, output)) {
        imu_fusion_reset(fusion);
        return false;
    }
    output->vertical_accel_valid = calculate_vertical_acceleration(
        fusion, sample, &output->vertical_accel_mps2);
    if (!output->vertical_accel_valid) {
        imu_fusion_reset(fusion);
        return false;
    }
    return true;
}
