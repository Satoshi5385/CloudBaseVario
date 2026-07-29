#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "domain/app_config.h"

#define IMU_AXIS_COUNT UINT32_C(3)

typedef struct {
    float accel_mps2[IMU_AXIS_COUNT];
    float gyro_radps[IMU_AXIS_COUNT];
    int64_t timestamp_us;
    bool valid;
} imu_sample_t;

typedef struct {
    float quaternion[4];
    float integral_feedback[IMU_AXIS_COUNT];
    float gyro_bias_radps[IMU_AXIS_COUNT];
    double calibration_gyro_sum[IMU_AXIS_COUNT];
    double calibration_accel_sum[IMU_AXIS_COUNT];
    uint32_t calibration_samples;
    int64_t previous_timestamp_us;
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
    uint32_t calibration_samples;
    bool calibrated;
    bool attitude_valid;
    bool vertical_accel_valid;
} imu_fusion_output_t;

/**
 * @brief Reset gyro calibration, attitude, and vertical-acceleration state.
 * @param[out] fusion State to reset.
 */
void imu_fusion_reset(imu_fusion_t *fusion);

/**
 * @brief Apply the configured sensor-to-board axis permutation and signs.
 * @param[in] input Physical sample in HXY sensor coordinates.
 * @param[in] config Valid runtime axis configuration.
 * @param[out] output Physical sample in board coordinates.
 * @return true when the map and sample are valid.
 */
bool imu_fusion_apply_axis_map(const imu_sample_t *input,
                               const app_config_t *config,
                               imu_sample_t *output);

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
