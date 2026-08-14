#include "domain/imu_calibration_controller.h"

#include <string.h>

void imu_calibration_controller_init(
    imu_calibration_controller_t *controller,
    const imu_accel_calibration_t *persisted) {
    if (controller == NULL) {
        return;
    }
    memset(controller, 0, sizeof(*controller));
    if (imu_accel_calibration_validate(persisted)) {
        controller->persisted = *persisted;
    }
    imu_accel_calibrator_reset(&controller->collector);
}

bool imu_calibration_controller_required(
    const imu_calibration_controller_t *controller) {
    return controller != NULL && !controller->skipped &&
           !imu_accel_calibration_validate(&controller->persisted);
}

bool imu_calibration_controller_skipped(
    const imu_calibration_controller_t *controller) {
    return controller != NULL && controller->skipped;
}

bool imu_calibration_controller_request_skip(
    imu_calibration_controller_t *controller) {
    if (!imu_calibration_controller_required(controller)) {
        return false;
    }
    controller->skipped = true;
    controller->save_pending = false;
    controller->next_save_us = 0;
    memset(&controller->pending, 0, sizeof(controller->pending));
    imu_accel_calibrator_reset(&controller->collector);
    return true;
}

bool imu_calibration_controller_process_sample(
    imu_calibration_controller_t *controller,
    const imu_sample_t *sensor_sample, const imu_axis_map_t *axis_map,
    int64_t now_us) {
    if (!imu_calibration_controller_required(controller) ||
        controller->save_pending) {
        return false;
    }
    if (!imu_accel_calibrator_update(&controller->collector,
                                     sensor_sample, axis_map,
                                     &controller->pending)) {
        return false;
    }
    controller->save_pending = true;
    controller->next_save_us = now_us;
    return true;
}

bool imu_calibration_controller_save_due(
    const imu_calibration_controller_t *controller, int64_t now_us) {
    return controller != NULL && controller->save_pending &&
           now_us >= controller->next_save_us;
}

const imu_accel_calibration_t *imu_calibration_controller_pending(
    const imu_calibration_controller_t *controller) {
    if (controller == NULL || !controller->save_pending ||
        !imu_accel_calibration_validate(&controller->pending)) {
        return NULL;
    }
    return &controller->pending;
}

const imu_accel_calibration_t *imu_calibration_controller_persisted(
    const imu_calibration_controller_t *controller) {
    if (controller == NULL ||
        !imu_accel_calibration_validate(&controller->persisted)) {
        return NULL;
    }
    return &controller->persisted;
}

void imu_calibration_controller_save_succeeded(
    imu_calibration_controller_t *controller) {
    if (controller == NULL ||
        !imu_accel_calibration_validate(&controller->pending)) {
        return;
    }
    controller->persisted = controller->pending;
    memset(&controller->pending, 0, sizeof(controller->pending));
    controller->save_pending = false;
    controller->next_save_us = 0;
}

void imu_calibration_controller_save_failed(
    imu_calibration_controller_t *controller, int64_t now_us,
    int64_t retry_interval_us) {
    if (controller == NULL || !controller->save_pending) {
        return;
    }
    controller->next_save_us = now_us + retry_interval_us;
}

void imu_calibration_controller_reset_collection(
    imu_calibration_controller_t *controller) {
    if (controller != NULL && !controller->save_pending) {
        imu_accel_calibrator_reset(&controller->collector);
    }
}

uint32_t imu_calibration_controller_sample_count(
    const imu_calibration_controller_t *controller) {
    if (controller == NULL) {
        return 0U;
    }
    if (controller->save_pending) {
        return IMU_ACCEL_CALIBRATION_SAMPLE_COUNT;
    }
    return controller->collector.sample_count;
}

float imu_calibration_controller_accel_norm_g(
    const imu_calibration_controller_t *controller) {
    if (controller == NULL) {
        return 0.0f;
    }
    return controller->collector.accel_norm_g;
}

float imu_calibration_controller_vibration_rms_g(
    const imu_calibration_controller_t *controller) {
    if (controller == NULL) {
        return 0.0f;
    }
    return controller->collector.vibration_rms_g;
}

bool imu_calibration_controller_save_pending(
    const imu_calibration_controller_t *controller) {
    return controller != NULL && controller->save_pending;
}

int64_t imu_calibration_controller_next_save_us(
    const imu_calibration_controller_t *controller) {
    if (controller == NULL) {
        return 0;
    }
    return controller->next_save_us;
}
