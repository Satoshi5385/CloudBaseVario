#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "domain/app_config.h"

#define IMU_AXIS_COUNT UINT32_C(3)
#define IMU_ACCEL_CALIBRATION_SAMPLE_COUNT UINT32_C(800)

typedef struct {
    float accel_mps2[IMU_AXIS_COUNT];
    float gyro_radps[IMU_AXIS_COUNT];
    int64_t timestamp_us;
    bool valid;
} imu_sample_t;

typedef struct {
    uint32_t accel_source[IMU_AXIS_COUNT];
    float accel_sign[IMU_AXIS_COUNT];
    uint32_t gyro_source[IMU_AXIS_COUNT];
    float gyro_sign[IMU_AXIS_COUNT];
} imu_axis_map_t;

typedef struct {
    float offset_mps2[IMU_AXIS_COUNT];
    uint32_t sample_count;
    bool valid;
} imu_accel_calibration_t;

typedef struct {
    double sensor_accel_sum[IMU_AXIS_COUNT];
    float accel_norm_g;
    float vibration_mean_g;
    float vibration_variance_g2;
    float vibration_rms_g;
    uint32_t sample_count;
    int64_t previous_timestamp_us;
    bool vibration_initialized;
} imu_accel_calibrator_t;

typedef struct {
    float quaternion[4];
    float integral_feedback[IMU_AXIS_COUNT];
    float gyro_bias_radps[IMU_AXIS_COUNT];
    double calibration_gyro_sum[IMU_AXIS_COUNT];
    double calibration_accel_sum[IMU_AXIS_COUNT];
    uint32_t calibration_samples;
    float vibration_mean_g;
    float vibration_variance_g2;
    float vibration_rms_g;
    float confidence;
    float kp_effective;
    float ki_effective;
    int64_t quasi_stationary_since_us;
    int64_t vibration_previous_timestamp_us;
    int64_t previous_timestamp_us;
    bool vibration_initialized;
    bool ki_active;
    bool calibrated;
    bool attitude_valid;
} imu_fusion_t;

typedef struct {
    float vertical_accel_mps2;
    float accel_norm_g;
    float quaternion[4];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float confidence;
    float vibration_rms_g;
    float kp_effective;
    float ki_effective;
    uint32_t calibration_samples;
    bool ki_active;
    bool calibrated;
    bool attitude_valid;
    bool vertical_accel_valid;
} imu_fusion_output_t;

/**
 * @brief Reset gyro calibration, attitude, and vertical-acceleration state.
 * @param[out] fusion State to reset.
 */
void imu_fusion_reset(imu_fusion_t *fusion);

/** Reset a first-boot horizontal accelerometer calibration sequence. */
void imu_accel_calibrator_reset(imu_accel_calibrator_t *calibrator);

/** Validate persisted sensor-coordinate accelerometer offsets. */
bool imu_accel_calibration_validate(
    const imu_accel_calibration_t *calibration);

/**
 * Collect one uncorrected sample for the LEVEL_Z_UP factory calibration.
 * Returns true only when 800 consecutive accepted samples completed a valid
 * sensor-coordinate offset result.
 */
bool imu_accel_calibrator_update(
    imu_accel_calibrator_t *calibrator, const imu_sample_t *sensor_sample,
    const imu_axis_map_t *axis_map, imu_accel_calibration_t *calibration);

/** Validate a sensor-to-board axis permutation and its signs. */
bool imu_axis_map_validate(const imu_axis_map_t *axis_map);

/**
 * @brief Apply the configured sensor-to-board axis permutation and signs.
 * @param[in] input Physical sample in HXY sensor coordinates.
 * @param[in] axis_map Valid board-specific axis configuration.
 * @param[out] output Physical sample in board coordinates.
 * @return true when the map and sample are valid.
 */
bool imu_fusion_apply_axis_map(const imu_sample_t *input,
                               const imu_axis_map_t *axis_map,
                               imu_sample_t *output);

/** Subtract persisted sensor-coordinate acceleration offsets, then map axes. */
bool imu_fusion_apply_calibration_and_axis_map(
    const imu_sample_t *input, const imu_axis_map_t *axis_map,
    const imu_accel_calibration_t *calibration, imu_sample_t *output);

/**
 * @brief Calibrate the gyro and update the 6DoF attitude estimate.
 * @param[in,out] fusion Persistent calibration and quaternion state.
 * @param[in] sample Board-coordinate physical sample.
 * @param[in] config Valid runtime calibration and Mahony settings.
 * @param[out] output Calibration, attitude, and vertical-acceleration result.
 * @return true when the sample was accepted, including calibration samples.
 */
bool imu_fusion_update(imu_fusion_t *fusion, const imu_sample_t *sample,
                       const app_config_t *config,
                       imu_fusion_output_t *output);
