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
#define IMU_DEGREES_TO_RADIANS 0.017453292519943295f
#define IMU_ACCEL_OFFSET_MAX_G 0.20f
#define IMU_FACTORY_LEVEL_XY_MAX_G 0.10f
#define IMU_FACTORY_POSE_Z_MIN_G 0.75f
#define IMU_FACTORY_POSE_Z_MAX_G 1.25f
#define IMU_FACTORY_VIBRATION_MAX_G 0.02f
#define IMU_VIBRATION_TIME_CONSTANT_SECONDS 0.25f
#define IMU_ACCEL_CONFIDENCE_ZERO_ERROR_G 0.15f
#define IMU_GYRO_CONFIDENCE_FULL_RADPS (10.0f * IMU_DEGREES_TO_RADIANS)
#define IMU_GYRO_CONFIDENCE_ZERO_RADPS (90.0f * IMU_DEGREES_TO_RADIANS)
#define IMU_VIBRATION_CONFIDENCE_FULL_G 0.01f
#define IMU_VIBRATION_CONFIDENCE_ZERO_G 0.05f
#define IMU_KI_ACCEL_ERROR_MAX_G 0.03f
#define IMU_KI_GYRO_MAX_RADPS (3.0f * IMU_DEGREES_TO_RADIANS)
#define IMU_KI_VIBRATION_MAX_G 0.01f
#define IMU_KI_STATIONARY_TIME_US INT64_C(500000)
#define IMU_INTEGRAL_FEEDBACK_MAX_RADPS (5.0f * IMU_DEGREES_TO_RADIANS)

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

static float clamp_unit(float value) {
    if (value <= 0.0f) {
        return 0.0f;
    }
    if (value >= 1.0f) {
        return 1.0f;
    }
    return value;
}

static float descending_confidence(float value, float full_value,
                                   float zero_value) {
    if (!isfinite(value) || !(full_value < zero_value)) {
        return 0.0f;
    }
    if (value <= full_value) {
        return 1.0f;
    }
    if (value >= zero_value) {
        return 0.0f;
    }
    return (zero_value - value) / (zero_value - full_value);
}

static void update_vibration_estimate(float value_g, int64_t timestamp_us,
                                      float *mean_g, float *variance_g2,
                                      float *rms_g,
                                      int64_t *previous_timestamp_us,
                                      bool *initialized) {
    float dt_seconds = 0.0f;
    float alpha = 0.0f;
    float delta = 0.0f;

    if (!isfinite(value_g) || timestamp_us <= 0 || mean_g == NULL ||
        variance_g2 == NULL || rms_g == NULL ||
        previous_timestamp_us == NULL || initialized == NULL) {
        return;
    }
    if (!*initialized || timestamp_us <= *previous_timestamp_us) {
        *mean_g = value_g;
        *variance_g2 = 0.0f;
        *rms_g = 0.0f;
        *previous_timestamp_us = timestamp_us;
        *initialized = true;
        return;
    }
    dt_seconds =
        (float) (timestamp_us - *previous_timestamp_us) / 1000000.0f;
    *previous_timestamp_us = timestamp_us;
    if (!isfinite(dt_seconds) || dt_seconds <= 0.0f) {
        return;
    }
    if (dt_seconds > IMU_ATTITUDE_MAX_DT_SECONDS) {
        dt_seconds = IMU_ATTITUDE_MAX_DT_SECONDS;
    }
    alpha = dt_seconds /
            (IMU_VIBRATION_TIME_CONSTANT_SECONDS + dt_seconds);
    delta = value_g - *mean_g;
    *mean_g += alpha * delta;
    *variance_g2 =
        (1.0f - alpha) * (*variance_g2 + alpha * delta * delta);
    if (!isfinite(*variance_g2) || *variance_g2 < 0.0f) {
        *variance_g2 = 0.0f;
    }
    *rms_g = sqrtf(*variance_g2);
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

void imu_accel_calibrator_reset(imu_accel_calibrator_t *calibrator) {
    if (calibrator == NULL) {
        return;
    }
    memset(calibrator, 0, sizeof(*calibrator));
}

bool imu_accel_calibration_validate(
    const imu_accel_calibration_t *calibration) {
    if (calibration == NULL || !calibration->valid ||
        calibration->sample_count != IMU_ACCEL_CALIBRATION_SAMPLE_COUNT) {
        return false;
    }
    for (size_t axis = 0U; axis < IMU_AXIS_COUNT; axis++) {
        if (!isfinite(calibration->offset_mps2[axis]) ||
            fabsf(calibration->offset_mps2[axis]) >
                IMU_ACCEL_OFFSET_MAX_G * IMU_STANDARD_GRAVITY_MPS2) {
            return false;
        }
    }
    return true;
}

static void reset_accel_calibration_samples(
    imu_accel_calibrator_t *calibrator) {
    memset(calibrator->sensor_accel_sum, 0,
           sizeof(calibrator->sensor_accel_sum));
    calibrator->sample_count = 0U;
}

bool imu_accel_calibrator_update(
    imu_accel_calibrator_t *calibrator, const imu_sample_t *sensor_sample,
    const imu_axis_map_t *axis_map, imu_accel_calibration_t *calibration) {
    imu_sample_t board_sample = {0};
    float accel_norm_g = 0.0f;
    float expected_board_mps2[IMU_AXIS_COUNT] = {
        0.0f, 0.0f, IMU_STANDARD_GRAVITY_MPS2};
    float expected_sensor_mps2[IMU_AXIS_COUNT] = {0};

    if (calibrator == NULL || calibration == NULL) {
        return false;
    }
    if (sensor_sample == NULL || axis_map == NULL ||
        !imu_fusion_apply_axis_map(sensor_sample, axis_map, &board_sample)) {
        reset_accel_calibration_samples(calibrator);
        return false;
    }
    for (size_t board_axis = 0U; board_axis < IMU_AXIS_COUNT;
         board_axis++) {
        expected_sensor_mps2[axis_map->accel_source[board_axis]] =
            expected_board_mps2[board_axis] *
            axis_map->accel_sign[board_axis];
    }
    accel_norm_g =
        vector_norm(board_sample.accel_mps2) / IMU_STANDARD_GRAVITY_MPS2;
    calibrator->accel_norm_g = accel_norm_g;
    update_vibration_estimate(
        accel_norm_g, board_sample.timestamp_us,
        &calibrator->vibration_mean_g,
        &calibrator->vibration_variance_g2,
        &calibrator->vibration_rms_g,
        &calibrator->previous_timestamp_us,
        &calibrator->vibration_initialized);

    if (fabsf(board_sample.accel_mps2[0]) >
        IMU_FACTORY_LEVEL_XY_MAX_G * IMU_STANDARD_GRAVITY_MPS2) {
        reset_accel_calibration_samples(calibrator);
        return false;
    }
    if (fabsf(board_sample.accel_mps2[1]) >
        IMU_FACTORY_LEVEL_XY_MAX_G * IMU_STANDARD_GRAVITY_MPS2) {
        reset_accel_calibration_samples(calibrator);
        return false;
    }
    if (board_sample.accel_mps2[2] <
            IMU_FACTORY_POSE_Z_MIN_G * IMU_STANDARD_GRAVITY_MPS2 ||
        board_sample.accel_mps2[2] >
            IMU_FACTORY_POSE_Z_MAX_G * IMU_STANDARD_GRAVITY_MPS2) {
        reset_accel_calibration_samples(calibrator);
        return false;
    }
    if (calibrator->vibration_rms_g > IMU_FACTORY_VIBRATION_MAX_G) {
        reset_accel_calibration_samples(calibrator);
        return false;
    }
    for (size_t axis = 0U; axis < IMU_AXIS_COUNT; axis++) {
        if (fabsf(board_sample.gyro_radps[axis]) >
            IMU_CALIBRATION_GYRO_MAX_RADPS) {
            reset_accel_calibration_samples(calibrator);
            return false;
        }
    }
    for (size_t axis = 0U; axis < IMU_AXIS_COUNT; axis++) {
        calibrator->sensor_accel_sum[axis] +=
            sensor_sample->accel_mps2[axis];
    }
    calibrator->sample_count++;
    if (calibrator->sample_count < IMU_ACCEL_CALIBRATION_SAMPLE_COUNT) {
        return false;
    }

    memset(calibration, 0, sizeof(*calibration));
    for (size_t axis = 0U; axis < IMU_AXIS_COUNT; axis++) {
        calibration->offset_mps2[axis] =
            (float) (calibrator->sensor_accel_sum[axis] /
                     (double) calibrator->sample_count) -
            expected_sensor_mps2[axis];
    }
    calibration->sample_count = IMU_ACCEL_CALIBRATION_SAMPLE_COUNT;
    calibration->valid = true;
    if (!imu_accel_calibration_validate(calibration)) {
        memset(calibration, 0, sizeof(*calibration));
        reset_accel_calibration_samples(calibrator);
        return false;
    }
    return true;
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
    float gyro_norm_radps = 0.0f;
    float accel_confidence = 0.0f;
    float gyro_confidence = 0.0f;
    float vibration_confidence = 0.0f;
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

    gyro_norm_radps = vector_norm(gyro);
    accel_confidence = clamp_unit(
        1.0f - fabsf(accel_norm_g - 1.0f) /
                   IMU_ACCEL_CONFIDENCE_ZERO_ERROR_G);
    gyro_confidence = descending_confidence(
        gyro_norm_radps, IMU_GYRO_CONFIDENCE_FULL_RADPS,
        IMU_GYRO_CONFIDENCE_ZERO_RADPS);
    vibration_confidence = descending_confidence(
        fusion->vibration_rms_g, IMU_VIBRATION_CONFIDENCE_FULL_G,
        IMU_VIBRATION_CONFIDENCE_ZERO_G);
    fusion->confidence =
        fminf(accel_confidence,
              fminf(gyro_confidence, vibration_confidence));
    fusion->kp_effective = config->imu_mahony_kp * fusion->confidence;

    if (config->imu_mahony_ki > 0.0f &&
        fabsf(accel_norm_g - 1.0f) <= IMU_KI_ACCEL_ERROR_MAX_G &&
        gyro_norm_radps <= IMU_KI_GYRO_MAX_RADPS &&
        fusion->vibration_rms_g <= IMU_KI_VIBRATION_MAX_G) {
        if (fusion->quasi_stationary_since_us == 0) {
            fusion->quasi_stationary_since_us = sample->timestamp_us;
        }
        fusion->ki_active =
            sample->timestamp_us - fusion->quasi_stationary_since_us >=
            IMU_KI_STATIONARY_TIME_US;
    } else {
        fusion->quasi_stationary_since_us = 0;
        fusion->ki_active = false;
    }
    fusion->ki_effective = fusion->ki_active
                               ? config->imu_mahony_ki * fusion->confidence
                               : 0.0f;

    if (accel_norm_g <= 0.0f || !isfinite(accel_norm_g)) {
        return false;
    }
    {
        float accel_norm_mps2 = accel_norm_g * IMU_STANDARD_GRAVITY_MPS2;

        for (size_t index = 0U; index < IMU_AXIS_COUNT; index++) {
            accel[index] /= accel_norm_mps2;
        }
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
        if (fusion->ki_active) {
            fusion->integral_feedback[index] +=
                2.0f * fusion->ki_effective * half_error[index] *
                dt_seconds;
            if (fusion->integral_feedback[index] >
                IMU_INTEGRAL_FEEDBACK_MAX_RADPS) {
                fusion->integral_feedback[index] =
                    IMU_INTEGRAL_FEEDBACK_MAX_RADPS;
            } else if (fusion->integral_feedback[index] <
                       -IMU_INTEGRAL_FEEDBACK_MAX_RADPS) {
                fusion->integral_feedback[index] =
                    -IMU_INTEGRAL_FEEDBACK_MAX_RADPS;
            }
        }
        gyro[index] += fusion->integral_feedback[index] +
                       2.0f * fusion->kp_effective * half_error[index];
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

bool imu_axis_map_validate(const imu_axis_map_t *axis_map) {
    uint32_t accel_axes = 0U;
    uint32_t gyro_axes = 0U;

    if (axis_map == NULL) {
        return false;
    }
    for (size_t axis = 0U; axis < IMU_AXIS_COUNT; axis++) {
        if (axis_map->accel_source[axis] >= IMU_AXIS_COUNT ||
            axis_map->gyro_source[axis] >= IMU_AXIS_COUNT ||
            (axis_map->accel_sign[axis] != -1.0f &&
             axis_map->accel_sign[axis] != 1.0f) ||
            (axis_map->gyro_sign[axis] != -1.0f &&
             axis_map->gyro_sign[axis] != 1.0f)) {
            return false;
        }
        accel_axes |= UINT32_C(1) << axis_map->accel_source[axis];
        gyro_axes |= UINT32_C(1) << axis_map->gyro_source[axis];
    }
    return accel_axes == UINT32_C(0x07) && gyro_axes == UINT32_C(0x07);
}

bool imu_fusion_apply_axis_map(const imu_sample_t *input,
                               const imu_axis_map_t *axis_map,
                               imu_sample_t *output) {
    if (input == NULL || axis_map == NULL || output == NULL || !input->valid ||
        !imu_axis_map_validate(axis_map) ||
        !vector_is_finite(input->accel_mps2) ||
        !vector_is_finite(input->gyro_radps)) {
        return false;
    }

    memset(output, 0, sizeof(*output));
    for (size_t index = 0U; index < IMU_AXIS_COUNT; index++) {
        output->accel_mps2[index] =
            input->accel_mps2[axis_map->accel_source[index]] *
            axis_map->accel_sign[index];
        output->gyro_radps[index] =
            input->gyro_radps[axis_map->gyro_source[index]] *
            axis_map->gyro_sign[index];
    }
    output->timestamp_us = input->timestamp_us;
    output->valid = true;
    return true;
}

bool imu_fusion_apply_calibration_and_axis_map(
    const imu_sample_t *input, const imu_axis_map_t *axis_map,
    const imu_accel_calibration_t *calibration, imu_sample_t *output) {
    imu_sample_t corrected = {0};

    if (input == NULL || axis_map == NULL || calibration == NULL ||
        output == NULL || !imu_accel_calibration_validate(calibration)) {
        return false;
    }
    corrected = *input;
    for (size_t axis = 0U; axis < IMU_AXIS_COUNT; axis++) {
        corrected.accel_mps2[axis] -= calibration->offset_mps2[axis];
    }
    return imu_fusion_apply_axis_map(&corrected, axis_map, output);
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
    update_vibration_estimate(
        output->accel_norm_g, sample->timestamp_us,
        &fusion->vibration_mean_g, &fusion->vibration_variance_g2,
        &fusion->vibration_rms_g,
        &fusion->vibration_previous_timestamp_us,
        &fusion->vibration_initialized);
    output->vibration_rms_g = fusion->vibration_rms_g;

    if (!fusion->calibrated) {
        if (!update_calibration(fusion, sample, config,
                                output->accel_norm_g)) {
            return false;
        }
        output->calibration_samples = fusion->calibration_samples;
        output->confidence = fusion->confidence;
        output->kp_effective = fusion->kp_effective;
        output->ki_effective = fusion->ki_effective;
        output->ki_active = fusion->ki_active;
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
    output->confidence = fusion->confidence;
    output->vibration_rms_g = fusion->vibration_rms_g;
    output->kp_effective = fusion->kp_effective;
    output->ki_effective = fusion->ki_effective;
    output->ki_active = fusion->ki_active;
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
