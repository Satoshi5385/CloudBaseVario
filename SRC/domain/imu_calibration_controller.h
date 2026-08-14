#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "domain/imu_fusion.h"

typedef struct {
    imu_accel_calibrator_t collector;
    imu_accel_calibration_t persisted;
    imu_accel_calibration_t pending;
    int64_t next_save_us;
    bool save_pending;
    bool skipped;
} imu_calibration_controller_t;

/** Initialize calibration collection from an optional persisted result. */
void imu_calibration_controller_init(
    imu_calibration_controller_t *controller,
    const imu_accel_calibration_t *persisted);

/** Report whether a valid calibration must still be collected. */
bool imu_calibration_controller_required(
    const imu_calibration_controller_t *controller);
/** Report whether the user skipped the required calibration. */
bool imu_calibration_controller_skipped(
    const imu_calibration_controller_t *controller);
/** Request a skip and report whether controller state changed. */
bool imu_calibration_controller_request_skip(
    imu_calibration_controller_t *controller);

/** Process one timestamped sensor sample and stage a valid result for saving. */
bool imu_calibration_controller_process_sample(
    imu_calibration_controller_t *controller,
    const imu_sample_t *sensor_sample, const imu_axis_map_t *axis_map,
    int64_t now_us);

/** Report whether a staged calibration save is due at now_us. */
bool imu_calibration_controller_save_due(
    const imu_calibration_controller_t *controller, int64_t now_us);
/** Return the staged calibration, or NULL when no save is pending. */
const imu_accel_calibration_t *imu_calibration_controller_pending(
    const imu_calibration_controller_t *controller);
/** Return the current persisted calibration view. */
const imu_accel_calibration_t *imu_calibration_controller_persisted(
    const imu_calibration_controller_t *controller);
/** Commit the pending calibration after a successful save. */
void imu_calibration_controller_save_succeeded(
    imu_calibration_controller_t *controller);
/** Schedule another save after a failed persistence attempt. */
void imu_calibration_controller_save_failed(
    imu_calibration_controller_t *controller, int64_t now_us,
    int64_t retry_interval_us);

/** Discard partial samples and restart calibration collection. */
void imu_calibration_controller_reset_collection(
    imu_calibration_controller_t *controller);
/** Return accepted calibration sample progress. */
uint32_t imu_calibration_controller_sample_count(
    const imu_calibration_controller_t *controller);
/** Return the latest acceleration norm in g. */
float imu_calibration_controller_accel_norm_g(
    const imu_calibration_controller_t *controller);
/** Return current calibration vibration RMS in g. */
float imu_calibration_controller_vibration_rms_g(
    const imu_calibration_controller_t *controller);
/** Report whether a valid calibration is waiting for persistence. */
bool imu_calibration_controller_save_pending(
    const imu_calibration_controller_t *controller);
/** Return the next allowed save timestamp in microseconds. */
int64_t imu_calibration_controller_next_save_us(
    const imu_calibration_controller_t *controller);
