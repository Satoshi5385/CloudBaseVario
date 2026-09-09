#pragma once

#include "domain/imu_calibration_controller.h"
#include "platform/imu_calibration_storage.h"

/** Provide boot-loaded accelerometer calibration to the sensor worker. */
void sensor_worker_set_imu_accel_calibration(
    const imu_accel_calibration_t *calibration,
    const imu_calibration_storage_diagnostics_t *diagnostics);

/** Run the acquisition, estimation, and publication sensor task. */
void app_sensor_worker_task(void *context);
