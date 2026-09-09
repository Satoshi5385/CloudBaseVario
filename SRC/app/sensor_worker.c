#include "app/sensor_worker.h"

#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/app_resources.h"
#include "app/app_worker_support.h"
#include "domain/app_config.h"
#include "domain/app_types.h"
#include "domain/imu_calibration_controller.h"
#include "domain/imu_fusion.h"
#include "domain/imu_motion.h"
#include "domain/sensor_scheduler.h"
#include "domain/vario_estimator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "platform/app_power.h"
#include "platform/bmp581.h"
#include "platform/board.h"
#include "platform/icm42688_hxy.h"
#include "platform/imu_calibration_storage.h"
#include "platform/sensor_bus.h"
#include "platform/usb_device_service.h"
#include "platform/watchdog_service.h"

#define SENSOR_MINIMUM_WAIT_MS UINT32_C(1)
#define SENSOR_IDLE_RETRY_MS UINT32_C(100)
#define STORAGE_MODE_POLL_MS UINT32_C(10)
#define BMP581_SAMPLE_PERIOD_US INT64_C(10000)
#define SENSOR_PUBLICATION_PERIOD_US BMP581_SAMPLE_PERIOD_US
#define SENSOR_STALE_TIMEOUT_US INT64_C(100000)
#define IMU_STALE_TIMEOUT_US INT64_C(100000)
#define SENSOR_IDLE_WAKE_US INT64_C(100000)
#define SENSOR_RETRY_INTERVAL_US \
    ((int64_t) CONFIG_CBV_SENSOR_RETRY_INTERVAL_MS * INT64_C(1000))
#define SENSOR_CONSECUTIVE_ERROR_LIMIT UINT32_C(10)
#define IMU_CALIBRATION_SAVE_RETRY_US INT64_C(2000000)
#define SENSOR_IMU_INIT_STEP_BUDGET_US INT64_C(5000)

static const char *TAG = "sensor_worker";
typedef struct {
    vario_result_t result;
    bool bmp_ready;
    bool bmp_bus_failed;
    bool bus_timeout_detected;
    bool imu_ready;
    bool imu_initializing;
    bool imu_watermark_pending;
    sensor_scheduler_t scheduler;
    int64_t next_publication_us;
    int64_t next_bmp_retry_us;
    int64_t next_imu_retry_us;
    int64_t last_bmp_valid_us;
    int64_t last_imu_valid_us;
    uint32_t bmp_consecutive_errors;
    uint32_t imu_consecutive_errors;
    imu_diagnostics_t imu_diagnostics;
    imu_fusion_t imu_fusion;
    imu_motion_state_t imu_motion;
    imu_calibration_controller_t imu_calibration;
    app_config_t imu_config;
    app_config_t measurement_config;
    vario_estimator_t estimator;
    float estimator_reference_pressure_pa;
    bool imu_config_valid;
    bool estimator_reference_valid;
    bool publication_pending;
} sensor_task_state_t;

typedef struct {
    icm42688_hxy_batch_t imu_batch;
    bmp581_sample_t bmp_sample;
    app_config_t config;
    esp_err_t imu_result;
    esp_err_t bmp_result;
    bool imu_attempted;
    bool bmp_attempted;
} sensor_cycle_t;

static imu_accel_calibration_t initial_imu_accel_calibration;
static imu_calibration_storage_diagnostics_t
    initial_imu_accel_calibration_diagnostics = {
        .result = IMU_CALIBRATION_STORAGE_MISSING,
    };

void sensor_worker_set_imu_accel_calibration(
    const imu_accel_calibration_t *calibration,
    const imu_calibration_storage_diagnostics_t *diagnostics) {
    memset(&initial_imu_accel_calibration, 0,
           sizeof(initial_imu_accel_calibration));
    if (calibration != NULL &&
        imu_accel_calibration_validate(calibration)) {
        initial_imu_accel_calibration = *calibration;
    }
    if (diagnostics != NULL) {
        initial_imu_accel_calibration_diagnostics = *diagnostics;
    } else {
        initial_imu_accel_calibration_diagnostics.result =
            IMU_CALIBRATION_STORAGE_MISSING;
        initial_imu_accel_calibration_diagnostics.io_error = 0;
    }
}

static void set_bmp581_recovering(bool recovering) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (event_group == NULL) {
        return;
    }
    if (recovering) {
        (void) xEventGroupSetBits(event_group, APP_EVENT_BMP581_RECOVERING);
    } else {
        (void) xEventGroupClearBits(event_group, APP_EVENT_BMP581_RECOVERING);
    }
}

static void set_imu_lifecycle_state(bool calibrating, bool degraded) {
    EventGroupHandle_t event_group = app_resources_event_group();
    const EventBits_t lifecycle_mask =
        APP_EVENT_IMU_CALIBRATING | APP_EVENT_IMU_DEGRADED;
    EventBits_t desired_bits = 0U;
    EventBits_t current_bits = 0U;
    EventBits_t bits_to_set = 0U;
    EventBits_t bits_to_clear = 0U;

    if (event_group == NULL) {
        return;
    }
    if (calibrating) {
        desired_bits |= APP_EVENT_IMU_CALIBRATING;
    }
    if (degraded) {
        desired_bits |= APP_EVENT_IMU_DEGRADED;
    }

    current_bits = xEventGroupGetBits(event_group) & lifecycle_mask;
    bits_to_set = desired_bits & ~current_bits;
    bits_to_clear = current_bits & ~desired_bits;
    if (bits_to_set != 0U) {
        (void) xEventGroupSetBits(event_group, bits_to_set);
    }
    if (bits_to_clear != 0U) {
        (void) xEventGroupClearBits(event_group, bits_to_clear);
    }
}

static void sensor_record_bmp_error(sensor_task_state_t *state,
                                    esp_err_t error,
                                    bool active_transfer) {
    if (state == NULL) {
        return;
    }

    app_worker_add_saturating_u32(&state->result.i2c_error_count, 1U);
    app_worker_add_saturating_u32(&state->bmp_consecutive_errors, 1U);
    if (active_transfer) {
        state->bmp_bus_failed = true;
    }
    if (error == ESP_ERR_TIMEOUT) {
        state->bus_timeout_detected = true;
    }
}

static void sensor_record_imu_error(sensor_task_state_t *state,
                                    esp_err_t error) {
    if (state == NULL) {
        return;
    }
    app_worker_add_saturating_u32(&state->result.i2c_error_count, 1U);
    app_worker_add_saturating_u32(&state->imu_consecutive_errors, 1U);
    state->imu_diagnostics.consecutive_error_count =
        state->imu_consecutive_errors;
    state->imu_diagnostics.last_error = (int32_t) error;
    if (error == ESP_ERR_TIMEOUT) {
        state->bus_timeout_detected = true;
    }
}

static void sensor_reset_imu_motion(sensor_task_state_t *state) {
    if (state == NULL) {
        return;
    }
    imu_motion_reset(&state->imu_motion);
    state->imu_diagnostics.motion_timestamp_us = 0;
    state->imu_diagnostics.motion_acceleration_rms_g = 0.0f;
    state->imu_diagnostics.motion_gyro_rms_dps = 0.0f;
    state->imu_diagnostics.motion_valid = false;
}

static void sensor_sync_scheduler_diagnostics(sensor_task_state_t *state) {
    if (state == NULL) {
        return;
    }
    state->imu_diagnostics.cadence =
        (imu_diagnostic_cadence_t) state->scheduler.cadence;
    state->imu_diagnostics.wtm_cycle_count =
        state->scheduler.wtm_cycle_count;
    state->imu_diagnostics.bmp_timer_cycle_count =
        state->scheduler.bmp_timer_cycle_count;
    state->imu_diagnostics.missed_interrupt_count =
        state->scheduler.wtm_timeout_count;
    state->imu_diagnostics.last_cycle_interval_us =
        state->scheduler.last_cycle_interval_us;
    state->imu_diagnostics.max_cycle_interval_us =
        state->scheduler.max_cycle_interval_us;
}

static bool sensor_try_initialize_imu(sensor_task_state_t *state,
                                      i2c_master_bus_handle_t bus_handle,
                                      int64_t now_us) {
    icm42688_hxy_identity_t identity = {0};
    esp_err_t ret = ESP_OK;

    if (state == NULL || bus_handle == NULL || !state->bmp_ready ||
        state->imu_ready ||
        imu_calibration_controller_skipped(&state->imu_calibration) ||
        now_us < state->next_imu_retry_us) {
        return false;
    }
    if (!state->imu_initializing) {
        int64_t bmp_slack_us = state->scheduler.next_bmp_us - now_us;

        if (state->scheduler.cadence != SENSOR_CADENCE_IMU_WTM &&
            bmp_slack_us >= 0 && bmp_slack_us <
                SENSOR_IMU_INIT_STEP_BUDGET_US) {
            return false;
        }
        app_worker_add_saturating_u32(&state->imu_diagnostics.retry_count, 1U);
        ret = icm42688_hxy_init_begin(bus_handle,
                                      xTaskGetCurrentTaskHandle());
        if (ret != ESP_OK) {
            state->imu_diagnostics.last_error = (int32_t) ret;
            state->next_imu_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
            return true;
        }
        state->imu_initializing = true;
        sensor_scheduler_start_imu_init(&state->scheduler);
        sensor_sync_scheduler_diagnostics(state);
        return true;
    }
    if (now_us < icm42688_hxy_init_next_action_us()) {
        return false;
    }
    if (state->scheduler.next_bmp_us > now_us &&
        state->scheduler.next_bmp_us - now_us <
            SENSOR_IMU_INIT_STEP_BUDGET_US) {
        return false;
    }
    ret = icm42688_hxy_init_poll(now_us, &identity);
    if (ret == ESP_ERR_NOT_FINISHED) {
        return true;
    }
    state->imu_initializing = false;
    state->imu_diagnostics.who_am_i = identity.who_am_i;
    state->imu_diagnostics.last_error = (int32_t) ret;
    if (ret == ESP_OK) {
        state->imu_ready = true;
        state->imu_watermark_pending = false;
        state->imu_consecutive_errors = 0U;
        state->last_imu_valid_us = now_us;
        state->imu_diagnostics.online = true;
        state->imu_diagnostics.configured = true;
        state->imu_diagnostics.stale = false;
        state->imu_diagnostics.address = identity.address;
        sensor_reset_imu_motion(state);
        state->result.imu_online = true;
        state->result.imu_stale = false;
        imu_fusion_reset(&state->imu_fusion);
        vario_estimator_disable_fusion(&state->estimator);
        state->imu_config_valid = false;
        sensor_scheduler_imu_ready(&state->scheduler, now_us);
        sensor_sync_scheduler_diagnostics(state);
        set_imu_lifecycle_state(true, false);
    } else {
        state->imu_ready = false;
        state->imu_watermark_pending = false;
        state->imu_diagnostics.online = false;
        state->imu_diagnostics.configured = false;
        state->imu_diagnostics.calibrated = false;
        state->imu_diagnostics.attitude_valid = false;
        state->imu_diagnostics.fusion_active = false;
        state->imu_diagnostics.stale = true;
        state->imu_diagnostics.address = 0U;
        sensor_reset_imu_motion(state);
        state->result.imu_online = false;
        state->result.imu_calibrated = false;
        state->result.imu_stale = true;
        state->result.imu_fusion_active = false;
        state->result.vertical_accel_valid = false;
        state->next_imu_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
        sensor_scheduler_use_bmp_timer(&state->scheduler,
                                       state->scheduler.next_bmp_us);
        sensor_sync_scheduler_diagnostics(state);
        set_imu_lifecycle_state(false, true);
    }
    return true;
}

static void sensor_invalidate_estimate(sensor_task_state_t *state,
                                       bool invalidate_pressure);

static void sensor_shutdown_devices(void) {
    esp_err_t bmp_ret = bmp581_deinit();

    if (bmp_ret != ESP_OK) {
        ESP_LOGW(TAG, "sensor shutdown incomplete: bmp=%s", esp_err_to_name(bmp_ret));
    }
    {
        esp_err_t imu_ret = icm42688_hxy_deinit();

        if (imu_ret != ESP_OK) {
            ESP_LOGW(TAG, "sensor shutdown incomplete: hxy_imu=%s",
                     esp_err_to_name(imu_ret));
        }
    }
}

static void sensor_enter_storage_mode(sensor_task_state_t *state) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (state == NULL) {
        return;
    }
    sensor_shutdown_devices();
    (void) sensor_bus_deinit();
    state->bmp_ready = false;
    state->bmp_bus_failed = false;
    state->bus_timeout_detected = false;
    state->imu_ready = false;
    state->imu_initializing = false;
    state->imu_watermark_pending = false;
    state->imu_config_valid = false;
    state->bmp_consecutive_errors = 0U;
    state->imu_consecutive_errors = 0U;
    state->result.bmp581_online = false;
    state->result.imu_online = false;
    state->result.imu_calibrated = false;
    state->result.imu_stale = false;
    state->result.imu_fusion_active = false;
    state->result.vertical_accel_valid = false;
    sensor_invalidate_estimate(state, true);
    state->imu_diagnostics.online = false;
    state->imu_diagnostics.configured = false;
    state->imu_diagnostics.calibrated = false;
    state->imu_diagnostics.attitude_valid = false;
    state->imu_diagnostics.fusion_active = false;
    state->imu_diagnostics.stale = false;
    state->imu_diagnostics.consecutive_error_count = 0U;
    sensor_reset_imu_motion(state);
    set_bmp581_recovering(false);
    set_imu_lifecycle_state(false, false);
    (void) app_resources_publish_vario(&state->result);
    (void) app_resources_publish_imu_diagnostics(&state->imu_diagnostics);
    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group,
                                  APP_EVENT_SENSOR_QUIESCED);
    }
}

static void sensor_leave_storage_mode(sensor_task_state_t *state) {
    int64_t now_us = esp_timer_get_time();

    if (state == NULL) {
        return;
    }
    state->next_bmp_retry_us = now_us;
    state->next_imu_retry_us = now_us;
    state->last_bmp_valid_us = now_us;
    state->last_imu_valid_us = now_us;
    sensor_scheduler_init(&state->scheduler, now_us);
    sensor_sync_scheduler_diagnostics(state);
    imu_fusion_reset(&state->imu_fusion);
    vario_estimator_reset(&state->estimator);
}

static void sensor_invalidate_estimate(sensor_task_state_t *state,
                                       bool invalidate_pressure) {
    if (state == NULL) {
        return;
    }
    if (invalidate_pressure) {
        state->result.pressure_valid = false;
    }
    state->result.climb_rate_valid = false;
    state->result.estimate_valid = false;
    state->result.estimator_warming_up = false;
    state->result.altitude_m = 0.0f;
    state->result.climb_rate_mps = 0.0f;
    vario_estimator_reset(&state->estimator);
}

static void sensor_invalidate_imu(sensor_task_state_t *state, bool stale) {
    if (state == NULL) {
        return;
    }
    state->imu_ready = false;
    state->imu_initializing = false;
    state->imu_watermark_pending = false;
    state->imu_config_valid = false;
    state->imu_consecutive_errors = 0U;
    state->result.imu_online = false;
    state->result.imu_calibrated = false;
    state->result.imu_stale = stale;
    state->result.imu_fusion_active = false;
    state->result.vertical_accel_valid = false;
    state->result.vertical_accel_mps2 = 0.0f;
    state->imu_diagnostics.online = false;
    state->imu_diagnostics.configured = false;
    state->imu_diagnostics.calibrated = false;
    state->imu_diagnostics.attitude_valid = false;
    state->imu_diagnostics.fusion_active = false;
    state->imu_diagnostics.stale = stale;
    state->imu_diagnostics.consecutive_error_count = 0U;
    sensor_reset_imu_motion(state);
    memset(state->imu_diagnostics.quaternion, 0,
           sizeof(state->imu_diagnostics.quaternion));
    state->imu_diagnostics.roll_deg = 0.0f;
    state->imu_diagnostics.pitch_deg = 0.0f;
    state->imu_diagnostics.yaw_deg = 0.0f;
    imu_fusion_reset(&state->imu_fusion);
    if (imu_calibration_controller_required(&state->imu_calibration)) {
        imu_calibration_controller_reset_collection(
            &state->imu_calibration);
    }
    vario_estimator_disable_fusion(&state->estimator);
    set_imu_lifecycle_state(false, true);
}

static bool sensor_recover_shared_bus(sensor_task_state_t *state, int64_t now_us) {
    esp_err_t ret = ESP_OK;
    esp_err_t imu_ret = ESP_OK;
    bool imu_was_ready = false;

    if (state == NULL || !state->bus_timeout_detected) {
        return false;
    }

    ESP_LOGW(TAG, "recovering shared I2C bus after transaction timeout");
    (void) bmp581_deinit();
    imu_was_ready = state->imu_ready;
    imu_ret = icm42688_hxy_deinit();
    if (imu_ret != ESP_OK) {
        ESP_LOGW(TAG, "HXY IMU handle removal before bus recovery failed: %s",
                 esp_err_to_name(imu_ret));
    }
    sensor_invalidate_imu(state, true);
    if (imu_was_ready || imu_ret != ESP_OK) {
        if (imu_ret == ESP_OK) {
            state->imu_diagnostics.last_error =
                (int32_t) ESP_ERR_INVALID_STATE;
        } else {
            state->imu_diagnostics.last_error = (int32_t) imu_ret;
        }
    }
    ret = sensor_bus_recover();

    state->bmp_ready = false;
    state->result.bmp581_online = false;
    sensor_invalidate_estimate(state, true);
    set_bmp581_recovering(true);
    state->bmp_consecutive_errors = 0U;
    state->bmp_bus_failed = false;
    state->bus_timeout_detected = false;
    sensor_scheduler_use_bmp_timer(&state->scheduler, now_us);
    sensor_sync_scheduler_diagnostics(state);
    if (ret == ESP_OK) {
        state->next_bmp_retry_us = now_us;
        state->next_imu_retry_us = now_us;
    } else {
        ESP_LOGW(TAG, "I2C recovery failed: %s", esp_err_to_name(ret));
        state->next_bmp_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
        state->next_imu_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
    }
    return true;
}

static bool sensor_try_initialize_devices(sensor_task_state_t *state,
                                          int64_t now_us) {
    i2c_master_bus_handle_t bus_handle = sensor_bus_get_handle();
    bool changed = false;
    esp_err_t ret = ESP_OK;

    if (state == NULL) {
        return false;
    }

    if (bus_handle == NULL && now_us >= state->next_bmp_retry_us) {
        ret = sensor_bus_init();
        if (ret != ESP_OK) {
            app_worker_add_saturating_u32(&state->result.i2c_error_count, 1U);
            if (ret == ESP_ERR_TIMEOUT) {
                state->bus_timeout_detected = true;
            }
            state->next_bmp_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
            set_bmp581_recovering(true);
            return true;
        }
        bus_handle = sensor_bus_get_handle();
    }
    if (bus_handle == NULL) {
        return changed;
    }

    if (!state->bmp_ready && now_us >= state->next_bmp_retry_us) {
        ret = bmp581_init(bus_handle);
        if (ret == ESP_OK) {
            state->bmp_ready = true;
            state->bmp_consecutive_errors = 0U;
            state->bmp_bus_failed = false;
            state->last_bmp_valid_us = now_us;
            set_bmp581_recovering(false);
        } else {
            sensor_record_bmp_error(state, ret, false);
            state->next_bmp_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
            set_bmp581_recovering(true);
        }
        changed = true;
    }

    return changed;
}

static bool imu_configs_match(const app_config_t *left,
                              const app_config_t *right) {
    if (left == NULL || right == NULL) {
        return false;
    }
    return left->imu_gyro_calibration_samples ==
           right->imu_gyro_calibration_samples;
}

static void sensor_restart_imu_fusion(sensor_task_state_t *state,
                                      const app_config_t *config) {
    if (state == NULL || config == NULL) {
        return;
    }
    state->imu_config = *config;
    state->imu_config_valid = true;
    imu_fusion_reset(&state->imu_fusion);
    vario_estimator_disable_fusion(&state->estimator);
    state->result.imu_calibrated = false;
    state->result.imu_fusion_active = false;
    state->result.vertical_accel_valid = false;
    state->result.vertical_accel_mps2 = 0.0f;
    state->imu_diagnostics.calibrated = false;
    state->imu_diagnostics.attitude_valid = false;
    state->imu_diagnostics.fusion_active = false;
    state->imu_diagnostics.calibration_sample_count = 0U;
    memset(state->imu_diagnostics.quaternion, 0,
           sizeof(state->imu_diagnostics.quaternion));
    state->imu_diagnostics.roll_deg = 0.0f;
    state->imu_diagnostics.pitch_deg = 0.0f;
    state->imu_diagnostics.yaw_deg = 0.0f;
    set_imu_lifecycle_state(true, false);
}

static void sensor_refresh_configs(sensor_task_state_t *state) {
    app_config_t config = {0};

    if (state == NULL) {
        return;
    }
    if (!app_resources_copy_config(&config)) {
        app_config_set_defaults(&config);
    }
    state->measurement_config = config;
    if (state->imu_ready) {
        if (!state->imu_config_valid ||
            !imu_configs_match(&state->imu_config, &config)) {
            sensor_restart_imu_fusion(state, &config);
        } else {
            state->imu_config = config;
        }
    }
}

static void sensor_sync_accel_calibration_diagnostics(
    sensor_task_state_t *state) {
    const imu_accel_calibration_t *calibration = NULL;

    if (state == NULL) {
        return;
    }
    calibration = imu_calibration_controller_persisted(
        &state->imu_calibration);
    if (calibration == NULL) {
        calibration = imu_calibration_controller_pending(
            &state->imu_calibration);
    }
    state->imu_diagnostics.accel_calibrated =
        imu_calibration_controller_persisted(
            &state->imu_calibration) != NULL;
    state->imu_diagnostics.accel_calibration_persisted =
        state->imu_diagnostics.accel_calibrated;
    state->imu_diagnostics.accel_calibration_save_pending =
        imu_calibration_controller_save_pending(
            &state->imu_calibration);
    state->imu_diagnostics.accel_calibration_skipped =
        imu_calibration_controller_skipped(&state->imu_calibration);
    state->imu_diagnostics.accel_calibration_sample_count =
        imu_calibration_controller_sample_count(
            &state->imu_calibration);
    state->imu_diagnostics.accel_norm_g =
        imu_calibration_controller_accel_norm_g(
            &state->imu_calibration);
    memset(state->imu_diagnostics.accel_offset_mps2, 0,
           sizeof(state->imu_diagnostics.accel_offset_mps2));
    if (calibration != NULL) {
        memcpy(state->imu_diagnostics.accel_offset_mps2,
               calibration->offset_mps2,
               sizeof(state->imu_diagnostics.accel_offset_mps2));
    }
}

static bool sensor_try_save_accel_calibration(sensor_task_state_t *state,
                                               int64_t now_us) {
    EventGroupHandle_t event_group = app_resources_event_group();
    esp_err_t ret = ESP_OK;

    if (state == NULL ||
        !imu_calibration_controller_save_due(
            &state->imu_calibration, now_us)) {
        return false;
    }
    ret = usb_device_save_imu_calibration(
        imu_calibration_controller_pending(&state->imu_calibration));
    state->imu_diagnostics.accel_calibration_storage_error =
        (int32_t) ret;
    if (ret == ESP_OK) {
        imu_calibration_controller_save_succeeded(
            &state->imu_calibration);
        state->imu_diagnostics.accel_calibration_storage_result =
            (int32_t) IMU_CALIBRATION_STORAGE_VALID;
        state->imu_diagnostics.accel_calibration_storage_error = 0;
        imu_fusion_reset(&state->imu_fusion);
        state->imu_config_valid = false;
        if (event_group != NULL) {
            (void) xEventGroupClearBits(
                event_group,
                APP_EVENT_IMU_ACCEL_CALIBRATION_REQUIRED |
                    APP_EVENT_IMU_ACCEL_CALIBRATION_SKIP_REQUEST);
            (void) xEventGroupSetBits(
                event_group, APP_EVENT_IMU_ACCEL_CALIBRATION_SAVED);
        }
        ESP_LOGI(TAG, "IMU accelerometer calibration saved");
    } else {
        imu_calibration_controller_save_failed(
            &state->imu_calibration, now_us,
            IMU_CALIBRATION_SAVE_RETRY_US);
        state->imu_diagnostics.accel_calibration_storage_result =
            (int32_t) IMU_CALIBRATION_STORAGE_IO_ERROR;
        ESP_LOGW(TAG, "mc_data.json save failed: %s",
                 esp_err_to_name(ret));
    }
    sensor_sync_accel_calibration_diagnostics(state);
    return true;
}

static bool sensor_handle_accel_calibration_skip(
    sensor_task_state_t *state) {
    EventGroupHandle_t event_group = app_resources_event_group();
    EventBits_t bits = 0U;

    if (event_group != NULL) {
        bits = xEventGroupGetBits(event_group);
    }

    if (state == NULL ||
        (bits & APP_EVENT_IMU_ACCEL_CALIBRATION_SKIP_REQUEST) == 0U ||
        !imu_calibration_controller_request_skip(
            &state->imu_calibration)) {
        return false;
    }

    if (state->imu_ready || state->imu_initializing) {
        esp_err_t ret = icm42688_hxy_deinit();

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "IMU deinitialization after calibration skip failed: %s",
                     esp_err_to_name(ret));
        }
    }
    state->imu_ready = false;
    state->imu_initializing = false;
    state->imu_watermark_pending = false;
    state->imu_config_valid = false;
    imu_fusion_reset(&state->imu_fusion);
    vario_estimator_disable_fusion(&state->estimator);

    state->result.imu_online = false;
    state->result.imu_calibrated = false;
    state->result.imu_stale = true;
    state->result.imu_fusion_active = false;
    state->result.vertical_accel_valid = false;
    state->result.vertical_accel_mps2 = 0.0f;
    state->imu_diagnostics.online = false;
    state->imu_diagnostics.configured = false;
    state->imu_diagnostics.calibrated = false;
    state->imu_diagnostics.attitude_valid = false;
    state->imu_diagnostics.fusion_active = false;
    state->imu_diagnostics.stale = true;
    sensor_reset_imu_motion(state);
    sensor_sync_accel_calibration_diagnostics(state);
    set_imu_lifecycle_state(false, true);
    sensor_scheduler_use_bmp_timer(&state->scheduler,
                                   esp_timer_get_time());
    sensor_sync_scheduler_diagnostics(state);

    if (event_group != NULL) {
        (void) xEventGroupClearBits(
            event_group,
            APP_EVENT_IMU_ACCEL_CALIBRATION_REQUIRED |
                APP_EVENT_IMU_ACCEL_CALIBRATION_SKIP_REQUEST);
        (void) xEventGroupSetBits(
            event_group, APP_EVENT_IMU_ACCEL_CALIBRATION_SKIPPED);
    }
    ESP_LOGW(TAG,
             "IMU accelerometer calibration skipped for this boot; pressure-only mode active");
    return true;
}

static bool sensor_process_factory_accel_calibration(
    sensor_task_state_t *state, const imu_sample_t *sensor_sample,
    int64_t now_us) {
    if (state == NULL || sensor_sample == NULL ||
        !imu_calibration_controller_required(
            &state->imu_calibration)) {
        return false;
    }
    if (imu_calibration_controller_process_sample(
            &state->imu_calibration, sensor_sample,
            board_imu_axis_map(), now_us)) {
        ESP_LOGI(TAG,
                 "IMU accelerometer calibration captured; saving mc_data.json");
    }
    state->result.imu_calibrated = false;
    state->result.vertical_accel_valid = false;
    state->result.imu_fusion_active = false;
    state->imu_diagnostics.calibrated = false;
    state->imu_diagnostics.attitude_valid = false;
    state->imu_diagnostics.fusion_active = false;
    state->imu_diagnostics.vibration_rms_g =
        imu_calibration_controller_vibration_rms_g(
            &state->imu_calibration);
    sensor_sync_accel_calibration_diagnostics(state);
    set_imu_lifecycle_state(true, false);
    return true;
}

/* state and hxy_sample are non-NULL and the batch was validated by the driver. */
static bool sensor_process_imu_sample(sensor_task_state_t *state,
                                      const icm42688_hxy_sample_t *hxy_sample) {
    imu_sample_t sensor_sample = {0};
    imu_sample_t board_sample = {0};
    imu_fusion_output_t fusion_output = {0};
    imu_motion_output_t motion_output = {0};
    const imu_accel_calibration_t *accel_calibration = NULL;

    state->imu_consecutive_errors = 0U;
    state->last_imu_valid_us = hxy_sample->timestamp_us;
    state->result.imu_online = true;
    state->result.imu_stale = false;
    state->imu_diagnostics.online = true;
    state->imu_diagnostics.configured = true;
    state->imu_diagnostics.stale = false;
    state->imu_diagnostics.last_error = (int32_t) ESP_OK;
    state->imu_diagnostics.consecutive_error_count = 0U;
    state->imu_diagnostics.data_status = hxy_sample->data_status;
    app_worker_add_saturating_u32(&state->imu_diagnostics.sample_count, 1U);
    state->publication_pending = true;
    memcpy(sensor_sample.accel_mps2, hxy_sample->accel_mps2,
           sizeof(sensor_sample.accel_mps2));
    memcpy(sensor_sample.gyro_radps, hxy_sample->gyro_radps,
           sizeof(sensor_sample.gyro_radps));
    sensor_sample.timestamp_us = hxy_sample->timestamp_us;
    sensor_sample.valid = hxy_sample->valid;
    if (imu_motion_update(&state->imu_motion, sensor_sample.accel_mps2,
                          sensor_sample.gyro_radps,
                          sensor_sample.timestamp_us, &motion_output)) {
        state->imu_diagnostics.motion_timestamp_us =
            sensor_sample.timestamp_us;
        state->imu_diagnostics.motion_acceleration_rms_g =
            motion_output.acceleration_rms_g;
        state->imu_diagnostics.motion_gyro_rms_dps =
            motion_output.gyro_rms_dps;
        state->imu_diagnostics.motion_valid = motion_output.valid;
    } else {
        sensor_reset_imu_motion(state);
    }
    accel_calibration = imu_calibration_controller_persisted(
        &state->imu_calibration);
    if (accel_calibration == NULL) {
        if (imu_calibration_controller_skipped(
                &state->imu_calibration)) {
            state->result.imu_calibrated = false;
            state->result.vertical_accel_valid = false;
            state->result.imu_fusion_active = false;
            state->imu_diagnostics.calibrated = false;
            state->imu_diagnostics.attitude_valid = false;
            state->imu_diagnostics.fusion_active = false;
            set_imu_lifecycle_state(false, true);
            return false;
        }
        (void) sensor_process_factory_accel_calibration(
            state, &sensor_sample, sensor_sample.timestamp_us);
        return false;
    }
    if (!imu_fusion_apply_calibration_and_axis_map(
            &sensor_sample, board_imu_axis_map(),
            accel_calibration,
            &board_sample) ||
        !imu_fusion_update(&state->imu_fusion, &board_sample,
                           &state->imu_config,
                           &fusion_output)) {
        sensor_restart_imu_fusion(state, &state->imu_config);
        return true;
    }

    state->imu_diagnostics.accel_norm_g = fusion_output.accel_norm_g;
    state->imu_diagnostics.confidence = fusion_output.confidence;
    state->imu_diagnostics.vibration_rms_g =
        fusion_output.vibration_rms_g;
    state->imu_diagnostics.kp_effective = fusion_output.kp_effective;
    state->imu_diagnostics.ki_effective = fusion_output.ki_effective;
    state->imu_diagnostics.ki_active = fusion_output.ki_active;
    state->imu_diagnostics.calibration_sample_count =
        fusion_output.calibration_samples;
    state->imu_diagnostics.calibrated = fusion_output.calibrated;
    state->imu_diagnostics.attitude_valid =
        fusion_output.attitude_valid;
    state->result.imu_calibrated = fusion_output.calibrated;
    if (fusion_output.attitude_valid) {
        memcpy(state->imu_diagnostics.quaternion,
               fusion_output.quaternion,
               sizeof(state->imu_diagnostics.quaternion));
        state->imu_diagnostics.roll_deg = fusion_output.roll_deg;
        state->imu_diagnostics.pitch_deg = fusion_output.pitch_deg;
        state->imu_diagnostics.yaw_deg = fusion_output.yaw_deg;
    }
    for (size_t axis = 0U; axis < IMU_AXIS_COUNT; axis++) {
        state->imu_diagnostics.gyro_bias_radps[axis] =
            state->imu_fusion.gyro_bias_radps[axis];
    }

    if (fusion_output.vertical_accel_valid) {
        state->result.vertical_accel_mps2 =
            fusion_output.vertical_accel_mps2;
        state->result.vertical_accel_valid = true;
        if (state->imu_config.filter_mode == APP_FILTER_MODE_AUTO) {
            if (!vario_estimator_update_imu(
                    &state->estimator,
                    fusion_output.vertical_accel_mps2,
                    fusion_output.confidence,
                    fusion_output.vibration_rms_g,
                    board_sample.timestamp_us)) {
                state->result.vertical_accel_valid = false;
            }
        } else {
            vario_estimator_disable_fusion(&state->estimator);
        }
    } else {
        state->result.vertical_accel_valid = false;
        state->result.vertical_accel_mps2 = 0.0f;
        vario_estimator_disable_fusion(&state->estimator);
    }

    if (fusion_output.calibrated && fusion_output.attitude_valid) {
        set_imu_lifecycle_state(false, false);
    } else {
        set_imu_lifecycle_state(true, false);
    }
    state->imu_diagnostics.fusion_active =
        state->result.imu_fusion_active;
    return false;
}

static bool sensor_process_imu_batch(sensor_task_state_t *state,
                                     const icm42688_hxy_batch_t *batch) {
    bool changed = false;

    if (state == NULL || batch == NULL || !state->imu_ready) {
        return false;
    }
    for (size_t index = 0U; index < batch->sample_count; index++) {
        changed |= sensor_process_imu_sample(state, &batch->samples[index]);
    }
    return changed;
}

static bool sensor_apply_bmp_sample(sensor_task_state_t *state,
                                    const bmp581_sample_t *sample,
                                    const app_config_t *config) {
    vario_estimate_t estimate = {0};

    if (state == NULL || sample == NULL || config == NULL || !sample->valid) {
        return false;
    }
    app_worker_add_saturating_u32(&state->result.sequence, 1U);
    state->result.timestamp_us = sample->timestamp_us;
    state->result.raw_temperature = sample->raw_temperature;
    state->result.raw_pressure = sample->raw_pressure;
    state->result.temperature_c_x100 = sample->temperature_c_x100;
    state->result.pressure_pa_x100 = sample->pressure_pa_x100;
    state->result.pressure_valid = true;
    state->result.bmp581_online = true;
    if (!state->estimator_reference_valid ||
        fabsf(config->sea_level_pressure_pa -
              state->estimator_reference_pressure_pa) > 0.01f) {
        vario_estimator_reset(&state->estimator);
        state->estimator_reference_pressure_pa = config->sea_level_pressure_pa;
        state->estimator_reference_valid = true;
    }
    if (vario_estimator_update(&state->estimator, sample->pressure_pa_x100,
                               sample->timestamp_us,
                               config->sea_level_pressure_pa,
                               config->filter_mode == APP_FILTER_MODE_AUTO &&
                                   state->imu_ready &&
                                   state->result.imu_calibrated &&
                                   state->result.vertical_accel_valid &&
                                   !state->result.imu_stale,
                               &estimate)) {
        state->result.estimator_warming_up = estimate.warming_up;
        state->result.altitude_m = estimate.altitude_m;
        state->result.climb_rate_mps = estimate.climb_rate_mps;
        state->result.climb_rate_valid = estimate.climb_rate_valid;
        state->result.estimate_valid =
            estimate.altitude_valid && estimate.climb_rate_valid;
        state->result.imu_fusion_active = estimate.fusion_active;
    } else {
        state->result.estimator_warming_up = estimate.warming_up;
        state->result.climb_rate_valid = false;
        state->result.estimate_valid = false;
        state->result.imu_fusion_active = false;
    }
    state->imu_diagnostics.fusion_active = state->result.imu_fusion_active;
    return true;
}

static void sensor_acquire_cycle(sensor_task_state_t *state,
                                 sensor_trigger_t trigger,
                                 sensor_cycle_t *cycle) {
    if (state == NULL || cycle == NULL) {
        return;
    }
    memset(cycle, 0, sizeof(*cycle));
    cycle->imu_result = ESP_ERR_INVALID_STATE;
    cycle->bmp_result = ESP_ERR_INVALID_STATE;
    if (trigger == SENSOR_TRIGGER_IMU_WTM && state->imu_ready) {
        cycle->imu_attempted = true;
        cycle->imu_result = icm42688_hxy_read_fifo(&cycle->imu_batch);
    }
    if (state->bmp_ready) {
        cycle->bmp_attempted = true;
        cycle->bmp_result = bmp581_read_sample(&cycle->bmp_sample);
    }
    cycle->config = state->measurement_config;
}

static bool sensor_record_bmp_cycle(sensor_task_state_t *state,
                                    const sensor_cycle_t *cycle,
                                    int64_t now_us) {
    if (state == NULL || cycle == NULL || !cycle->bmp_attempted) {
        return false;
    }
    if (cycle->bmp_result != ESP_OK || !cycle->bmp_sample.valid) {
        esp_err_t error = cycle->bmp_result;

        if (error == ESP_OK) {
            error = ESP_ERR_INVALID_RESPONSE;
        }

        sensor_record_bmp_error(state, error, true);
        if (state->bmp_consecutive_errors >= SENSOR_CONSECUTIVE_ERROR_LIMIT) {
            ESP_LOGW(TAG, "BMP581 offline after %" PRIu32
                     " consecutive errors", state->bmp_consecutive_errors);
            (void) bmp581_deinit();
            state->bmp_ready = false;
            state->result.bmp581_online = false;
            sensor_invalidate_estimate(state, true);
            state->next_bmp_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
            set_bmp581_recovering(true);
        }
        return true;
    }
    state->bmp_consecutive_errors = 0U;
    state->bmp_bus_failed = false;
    state->last_bmp_valid_us = cycle->bmp_sample.timestamp_us;
    return true;
}

static bool sensor_record_imu_cycle(sensor_task_state_t *state,
                                    const sensor_cycle_t *cycle,
                                    int64_t now_us) {
    esp_err_t error;

    if (state == NULL || cycle == NULL || !cycle->imu_attempted) {
        return false;
    }
    app_worker_add_saturating_u32(&state->imu_diagnostics.fifo_read_count, 1U);
    state->imu_diagnostics.data_status = cycle->imu_batch.data_status;
    state->imu_diagnostics.fifo_last_sample_count =
        (uint32_t) cycle->imu_batch.sample_count;
    if (cycle->imu_batch.overflow) {
        app_worker_add_saturating_u32(&state->imu_diagnostics.fifo_overflow_count, 1U);
    }
    app_worker_add_saturating_u32(&state->result.missed_imu_sample_count,
                       cycle->imu_batch.discarded_samples);
    if (cycle->imu_result == ESP_OK &&
        cycle->imu_batch.sample_count >= 4U) {
        return false;
    }
    error = cycle->imu_result;
    if (error == ESP_OK) {
        error = ESP_ERR_INVALID_SIZE;
    }
    app_worker_add_saturating_u32(&state->imu_diagnostics.fifo_error_count, 1U);
    sensor_record_imu_error(state, error);
    (void) icm42688_hxy_deinit();
    sensor_invalidate_imu(state, true);
    state->imu_diagnostics.last_error = (int32_t) error;
    state->next_imu_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
    return true;
}

static bool sensor_apply_cycle(sensor_task_state_t *state,
                               sensor_trigger_t trigger,
                               const sensor_cycle_t *cycle,
                               int64_t completed_us) {
    bool changed = false;
    bool imu_failed = false;

    changed |= sensor_record_bmp_cycle(state, cycle, completed_us);
    if (trigger == SENSOR_TRIGGER_WTM_TIMEOUT) {
        if (state->imu_ready || state->imu_initializing) {
            (void) icm42688_hxy_deinit();
            sensor_invalidate_imu(state, true);
            state->next_imu_retry_us = completed_us +
                                       SENSOR_RETRY_INTERVAL_US;
        }
        imu_failed = true;
    } else {
        imu_failed = sensor_record_imu_cycle(state, cycle, completed_us);
    }
    if (!imu_failed && cycle->imu_attempted) {
        changed |= sensor_process_imu_batch(state, &cycle->imu_batch);
    }
    if (cycle->bmp_result == ESP_OK && cycle->bmp_sample.valid) {
        changed |= sensor_apply_bmp_sample(state, &cycle->bmp_sample,
                                           &cycle->config);
    }
    if (trigger == SENSOR_TRIGGER_IMU_WTM && !imu_failed) {
        sensor_scheduler_complete_wtm(&state->scheduler, completed_us);
    } else if (trigger == SENSOR_TRIGGER_IMU_WTM && imu_failed) {
        sensor_scheduler_fail_imu_after_bmp(&state->scheduler,
                                            completed_us);
    } else {
        uint32_t overruns = sensor_scheduler_complete_bmp_timer(
            &state->scheduler, completed_us);

        app_worker_add_saturating_u32(&state->result.bmp_period_overrun_count,
                           overruns);
    }
    sensor_sync_scheduler_diagnostics(state);
    return changed || cycle->imu_attempted || cycle->bmp_attempted;
}

static bool sensor_check_stale(sensor_task_state_t *state, int64_t now_us) {
    bool changed = false;

    if (state == NULL) {
        return false;
    }

    if (state->bmp_ready && now_us - state->last_bmp_valid_us > SENSOR_STALE_TIMEOUT_US) {
        ESP_LOGW(TAG, "BMP581 stale; scheduling device reinitialization");
        (void) bmp581_deinit();
        state->bmp_ready = false;
        state->result.bmp581_online = false;
        sensor_invalidate_estimate(state, true);
        state->next_bmp_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
        set_bmp581_recovering(true);
        changed = true;
    }
    if (state->imu_ready &&
        now_us - state->last_imu_valid_us > IMU_STALE_TIMEOUT_US) {
        ESP_LOGW(TAG, "HXY IMU stale; falling back to pressure-only mode");
        (void) icm42688_hxy_deinit();
        sensor_invalidate_imu(state, true);
        state->next_imu_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
        state->imu_diagnostics.last_error =
            (int32_t) ESP_ERR_INVALID_STATE;
        sensor_scheduler_use_bmp_timer(&state->scheduler, now_us);
        sensor_sync_scheduler_diagnostics(state);
        changed = true;
    }
    return changed;
}

static void sensor_sync_estimator_diagnostics(sensor_task_state_t *state) {
    vario_estimator_diagnostics_t diagnostics = {0};

    if (state == NULL ||
        !vario_estimator_get_diagnostics(&state->estimator,
                                         &diagnostics)) {
        return;
    }
    state->result.kalman_accel_bias_mps2 = diagnostics.accel_bias_mps2;
    state->result.kalman_baro_innovation_m =
        diagnostics.baro_innovation_m;
    state->result.kalman_accel_innovation_mps2 =
        diagnostics.accel_innovation_mps2;
    state->result.kalman_baro_r_m2 =
        diagnostics.baro_measurement_variance_m2;
    state->result.kalman_accel_r_m2_s4 =
        diagnostics.accel_measurement_variance_m2_s4;
    state->result.kalman_baro_innovation_valid =
        diagnostics.baro_innovation_valid;
    state->result.kalman_accel_innovation_valid =
        diagnostics.accel_innovation_valid;
}

static bool sensor_publication_due(const sensor_task_state_t *state,
                                   int64_t now_us) {
    return state != NULL && state->publication_pending &&
           now_us >= state->next_publication_us;
}

static void sensor_publish_snapshots(sensor_task_state_t *state,
                                     int64_t now_us) {
    bool vario_published = false;
    bool diagnostics_published = false;

    if (state == NULL) {
        return;
    }
    sensor_sync_scheduler_diagnostics(state);
    sensor_sync_estimator_diagnostics(state);
    vario_published = app_resources_publish_vario(&state->result);
    diagnostics_published = app_resources_publish_imu_diagnostics(
        &state->imu_diagnostics);
    state->publication_pending =
        !vario_published || !diagnostics_published;
    state->next_publication_us =
        now_us + SENSOR_PUBLICATION_PERIOD_US;
}

static TickType_t sensor_wait_ticks(const sensor_task_state_t *state, int64_t now_us) {
    int64_t wake_time_us = now_us + SENSOR_IDLE_WAKE_US;
    int64_t wait_us = 0;
    uint32_t wait_ms = 0U;

    if (state == NULL) {
        return pdMS_TO_TICKS(SENSOR_MINIMUM_WAIT_MS);
    }

    if (sensor_scheduler_next_wake_us(&state->scheduler) < wake_time_us) {
        wake_time_us = sensor_scheduler_next_wake_us(&state->scheduler);
    }
    if (state->imu_initializing) {
        int64_t init_us = icm42688_hxy_init_next_action_us();
        int64_t init_start_us = now_us;

        if (init_us > init_start_us) {
            init_start_us = init_us;
        }

        if (init_us < wake_time_us &&
            (state->scheduler.next_bmp_us <= now_us ||
             state->scheduler.next_bmp_us - init_start_us >=
                 SENSOR_IMU_INIT_STEP_BUDGET_US)) {
            wake_time_us = init_us;
        }
    }
    if (!state->bmp_ready && state->next_bmp_retry_us < wake_time_us) {
        wake_time_us = state->next_bmp_retry_us;
    }
    if (state->bmp_ready && state->last_bmp_valid_us + SENSOR_STALE_TIMEOUT_US < wake_time_us) {
        wake_time_us = state->last_bmp_valid_us + SENSOR_STALE_TIMEOUT_US;
    }
    if (!state->imu_ready && !state->bmp_ready && state->publication_pending &&
        state->next_publication_us < wake_time_us) {
        wake_time_us = state->next_publication_us;
    }
    if (imu_calibration_controller_save_pending(
            &state->imu_calibration) &&
        imu_calibration_controller_next_save_us(
            &state->imu_calibration) < wake_time_us) {
        wake_time_us = imu_calibration_controller_next_save_us(
            &state->imu_calibration);
    }
    if (state->imu_ready &&
        state->last_imu_valid_us + IMU_STALE_TIMEOUT_US < wake_time_us) {
        wake_time_us =
            state->last_imu_valid_us + IMU_STALE_TIMEOUT_US;
    } else if (!state->imu_ready && !state->imu_initializing &&
               state->next_imu_retry_us < wake_time_us) {
        int64_t retry_us = state->next_imu_retry_us;
        int64_t retry_start_us = now_us;

        if (retry_us > retry_start_us) {
            retry_start_us = retry_us;
        }
        if (state->scheduler.next_bmp_us <= now_us ||
            state->scheduler.next_bmp_us - retry_start_us >=
                SENSOR_IMU_INIT_STEP_BUDGET_US) {
            wake_time_us = retry_us;
        }
    }

    wait_us = wake_time_us - now_us;
    if (wait_us <= 0) {
        return 0U;
    }
    wait_ms = (uint32_t) ((wait_us + INT64_C(999)) / INT64_C(1000));
    return pdMS_TO_TICKS(wait_ms);
}

static bool sensor_measurement_work_due(const sensor_task_state_t *state,
                                        int64_t now_us) {
    return state != NULL &&
        sensor_scheduler_trigger(&state->scheduler,
                                 state->imu_watermark_pending,
                                 now_us) != SENSOR_TRIGGER_NONE;
}

static bool sensor_execute_measurement_work(sensor_task_state_t *state,
                                            int64_t now_us) {
    esp_err_t power_ret;
    sensor_cycle_t cycle;
    sensor_trigger_t trigger;
    bool changed = false;

    if (!sensor_measurement_work_due(state, now_us)) {
        return false;
    }
    trigger = sensor_scheduler_trigger(&state->scheduler,
                                       state->imu_watermark_pending,
                                       now_us);
    if (trigger == SENSOR_TRIGGER_WTM_TIMEOUT) {
        sensor_scheduler_expire_wtm(&state->scheduler, now_us);
    }
    state->imu_watermark_pending = false;

    power_ret = app_power_sensor_work_begin();
    if (power_ret != ESP_OK) {
        ESP_LOGW(TAG, "sensor CPU-frequency lock unavailable: %s", esp_err_to_name(power_ret));
        app_worker_block_safe_stop_light_sleep();
    }

    sensor_acquire_cycle(state, trigger, &cycle);
    changed |= sensor_apply_cycle(state, trigger, &cycle,
                                  esp_timer_get_time());
    if (power_ret == ESP_OK) {
        power_ret = app_power_sensor_work_end();
        if (power_ret != ESP_OK) {
            ESP_LOGW(TAG, "sensor CPU-frequency lock release failed: %s",
                     esp_err_to_name(power_ret));
            app_worker_block_safe_stop_light_sleep();
        }
    }
    return changed;
}

static bool sensor_execute_work(sensor_task_state_t *state, int64_t now_us) {
    bool changed = false;
    i2c_master_bus_handle_t bus_handle;

    changed |= sensor_handle_accel_calibration_skip(state);
    changed |= sensor_try_initialize_devices(state, now_us);
    changed |= sensor_execute_measurement_work(
        state, esp_timer_get_time());
    sensor_refresh_configs(state);
    bus_handle = sensor_bus_get_handle();
    if (!state->bus_timeout_detected) {
        changed |= sensor_try_initialize_imu(
            state, bus_handle, esp_timer_get_time());
    }
    changed |= sensor_try_save_accel_calibration(
        state, esp_timer_get_time());
    changed |= sensor_check_stale(state, esp_timer_get_time());
    changed |= sensor_recover_shared_bus(state, esp_timer_get_time());
    return changed;
}

void app_sensor_worker_task(void *context) {
    sensor_task_state_t state = {0};
    EventGroupHandle_t event_group = app_resources_event_group();
    bool watchdog_registered = false;
    bool fatal_shutdown_complete = false;
    bool storage_quiesced = false;

    (void) context;
    ESP_LOGI(TAG, "sensor_task started on core %d", xPortGetCoreID());
    watchdog_registered = app_worker_register_watchdog(
        WATCHDOG_ACTOR_SENSOR, "sensor_task");
    if (!watchdog_registered) {
        if (event_group != NULL) {
            (void) xEventGroupSetBits(
                event_group, APP_EVENT_BMP581_STARTUP_COMPLETE);
        }
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(SENSOR_IDLE_RETRY_MS));
        }
    }
    if (app_worker_fatal_state()) {
        app_worker_unregister_watchdog(WATCHDOG_ACTOR_SENSOR,
                                     &watchdog_registered);
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(SENSOR_IDLE_RETRY_MS));
        }
    }
    state.next_bmp_retry_us = esp_timer_get_time();
    state.next_publication_us = state.next_bmp_retry_us;
    state.next_imu_retry_us = state.next_bmp_retry_us;
    state.result.timestamp_us = state.next_bmp_retry_us;
    sensor_scheduler_init(&state.scheduler, state.next_bmp_retry_us);
    state.imu_diagnostics.enabled = true;
    state.imu_diagnostics.last_error = (int32_t) ESP_ERR_NOT_FOUND;
    app_config_set_defaults(&state.measurement_config);
    sensor_refresh_configs(&state);
    imu_calibration_controller_init(
        &state.imu_calibration, &initial_imu_accel_calibration);
    state.imu_diagnostics.accel_calibration_storage_result =
        (int32_t) initial_imu_accel_calibration_diagnostics.result;
    state.imu_diagnostics.accel_calibration_storage_error =
        initial_imu_accel_calibration_diagnostics.io_error;
    sensor_sync_accel_calibration_diagnostics(&state);
    imu_fusion_reset(&state.imu_fusion);
    imu_motion_reset(&state.imu_motion);
    state.publication_pending = true;
    {
        bool work_changed =
            sensor_execute_work(&state, state.next_bmp_retry_us);

        state.publication_pending =
            state.publication_pending || work_changed;
    }
    sensor_publish_snapshots(&state, esp_timer_get_time());
    if (event_group != NULL) {
        EventBits_t startup_bits = APP_EVENT_BMP581_STARTUP_COMPLETE;

        if (!state.bmp_ready) {
            startup_bits |= APP_EVENT_FATAL_STATE | APP_EVENT_FATAL_BMP581;
        }
        (void) xEventGroupSetBits(event_group, startup_bits);
    }
    if (!state.bmp_ready) {
        ESP_LOGE(TAG, "BMP581 startup initialization failed; entering fatal state");
    }

    for (;;) {
        bool work_changed;
        int64_t now_us = 0;
        uint32_t notification_count = 0U;

        if (app_worker_stop_requested()) {
            sensor_shutdown_devices();
            app_worker_unregister_watchdog(WATCHDOG_ACTOR_SENSOR,
                                         &watchdog_registered);
            app_worker_acknowledge_and_delete(APP_EVENT_SENSOR_ACK);
        }
        if (app_worker_fatal_state()) {
            if (!fatal_shutdown_complete) {
                sensor_shutdown_devices();
                fatal_shutdown_complete = true;
            }
            app_worker_unregister_watchdog(WATCHDOG_ACTOR_SENSOR,
                                         &watchdog_registered);
            vTaskDelay(pdMS_TO_TICKS(SENSOR_IDLE_RETRY_MS));
            continue;
        }
        if (event_group != NULL &&
            (xEventGroupGetBits(event_group) &
             APP_EVENT_STORAGE_MODE_REQUEST) != 0U) {
            if (!storage_quiesced) {
                sensor_enter_storage_mode(&state);
                storage_quiesced = true;
            }
            app_worker_feed_watchdog(WATCHDOG_ACTOR_SENSOR,
                                   watchdog_registered);
            vTaskDelay(pdMS_TO_TICKS(STORAGE_MODE_POLL_MS));
            continue;
        }
        if (storage_quiesced) {
            (void) xEventGroupClearBits(event_group,
                                        APP_EVENT_SENSOR_QUIESCED);
            sensor_leave_storage_mode(&state);
            storage_quiesced = false;
        }

        now_us = esp_timer_get_time();
        notification_count =
            ulTaskNotifyTake(pdTRUE, sensor_wait_ticks(&state, now_us));
        if (notification_count > 0U && state.imu_ready &&
            state.scheduler.cadence == SENSOR_CADENCE_IMU_WTM) {
            state.imu_watermark_pending = true;
        }
        now_us = esp_timer_get_time();

        work_changed = sensor_execute_work(&state, now_us);
        state.publication_pending =
            state.publication_pending || work_changed;
        now_us = esp_timer_get_time();
        if (sensor_publication_due(&state, now_us)) {
            sensor_publish_snapshots(&state, now_us);
        }

        app_worker_feed_watchdog(WATCHDOG_ACTOR_SENSOR,
                               watchdog_registered);
    }
}

