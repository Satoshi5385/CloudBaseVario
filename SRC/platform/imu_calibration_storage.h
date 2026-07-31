#pragma once

#include <stdint.h>

#include "domain/imu_fusion.h"
#include "esp_err.h"

typedef enum {
    IMU_CALIBRATION_STORAGE_VALID = 0,
    IMU_CALIBRATION_STORAGE_MISSING,
    IMU_CALIBRATION_STORAGE_RECOVERED,
    IMU_CALIBRATION_STORAGE_INVALID,
    IMU_CALIBRATION_STORAGE_IO_ERROR,
} imu_calibration_storage_result_t;

typedef struct {
    imu_calibration_storage_result_t result;
    int32_t io_error;
} imu_calibration_storage_diagnostics_t;

imu_calibration_storage_result_t imu_calibration_storage_load(
    const char *base_path, imu_accel_calibration_t *calibration,
    imu_calibration_storage_diagnostics_t *diagnostics);

esp_err_t imu_calibration_storage_save(
    const char *base_path, const imu_accel_calibration_t *calibration);

const char *imu_calibration_storage_result_name(
    imu_calibration_storage_result_t result);
