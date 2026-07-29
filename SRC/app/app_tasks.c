#include "app/app_tasks.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>

#include "app/app_resources.h"
#include "domain/app_config.h"
#include "domain/app_types.h"
#include "domain/imu_fusion.h"
#include "domain/vario_audio.h"
#include "domain/vario_estimator.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "platform/app_power.h"
#include "platform/audio_output.h"
#include "platform/ble_vario.h"
#include "platform/bmp581.h"
#include "platform/board.h"
#include "platform/firmware_update.h"
#include "platform/icm42688_hxy.h"
#include "platform/sensor_bus.h"
#include "platform/system_io.h"
#include "platform/usb_device_service.h"

#define SENSOR_TASK_PRIORITY ((UBaseType_t) 20U)
#define AUDIO_TASK_PRIORITY ((UBaseType_t) 18U)
#define SYSTEM_TASK_PRIORITY ((UBaseType_t) 12U)
#define BLE_TX_TASK_PRIORITY ((UBaseType_t) 8U)
#define CONSOLE_TASK_PRIORITY ((UBaseType_t) 5U)

#define SENSOR_TASK_STACK_BYTES UINT32_C(8192)
#define AUDIO_TASK_STACK_BYTES UINT32_C(4096)
#define SYSTEM_TASK_STACK_BYTES UINT32_C(4096)
#define BLE_TX_TASK_STACK_BYTES UINT32_C(6144)
#define CONSOLE_TASK_STACK_BYTES UINT32_C(6144)

#define HIGH_RATE_TASK_CORE ((BaseType_t) 1)
#define COMMUNICATION_TASK_CORE ((BaseType_t) 0)

#define AUDIO_EVALUATION_PERIOD_MS UINT32_C(10)
#define SYSTEM_SAMPLE_PERIOD_MS UINT32_C(10)
#define BLE_TX_PERIOD_MS UINT32_C(100)
#define SERIAL_MONITOR_PERIOD_US INT64_C(100000)
#define BATTERY_SAMPLE_PERIOD_MS UINT32_C(100)
#define SWITCH_DEBOUNCE_MS UINT32_C(30)
#define POWER_OFF_HOLD_MS UINT32_C(3000)
#define SHUTDOWN_DEADLINE_MS UINT32_C(15000)
#define SAFE_STOP_PERIOD_MS UINT32_C(10)
#define SYSTEM_SOUND_LOW_HZ UINT32_C(700)
#define SYSTEM_SOUND_LOW_MS UINT32_C(180)
#define SYSTEM_SOUND_SILENCE_MS UINT32_C(80)
#define SYSTEM_SOUND_HIGH_HZ UINT32_C(1200)
#define SYSTEM_SOUND_HIGH_MS UINT32_C(120)
#define SHUTDOWN_SOUND_TOTAL_MS                                                        \
    (SYSTEM_SOUND_HIGH_MS + SYSTEM_SOUND_SILENCE_MS + SYSTEM_SOUND_LOW_MS)
#define STARTUP_SOUND_WAIT_MS UINT32_C(1000)
#define SYSTEM_SOUND_DUTY_PERCENT UINT32_C(50)
#define SYSTEM_SOUND_AMPLIFIER_MODE UINT32_C(1)
#define FATAL_LED_PHASE_MS UINT32_C(500)
#define BMP_RECOVERY_LED_PHASE_MS UINT32_C(1000)
#define IMU_CALIBRATION_LED_CYCLE_MS UINT32_C(2000)
#define IMU_DEGRADED_LED_CYCLE_MS UINT32_C(1000)
#define LED_SOFTWARE_PWM_STEPS UINT32_C(10)

#define BMP581_SAMPLE_PERIOD_US INT64_C(10000)
#define SENSOR_STALE_TIMEOUT_US INT64_C(100000)
#define IMU_STALE_TIMEOUT_US INT64_C(100000)
#define SENSOR_IDLE_WAKE_US INT64_C(100000)
#define SENSOR_RETRY_INTERVAL_US ((int64_t) CONFIG_CBV_SENSOR_RETRY_INTERVAL_MS * INT64_C(1000))

typedef struct {
    bool candidate_pressed;
    bool stable_pressed;
    bool stable_valid;
    uint32_t stable_time_ms;
} button_debounce_t;

typedef struct {
    uint32_t frequency_hz;
    uint32_t duration_ms;
} system_sound_step_t;

typedef enum {
    SYSTEM_SOUND_COMPLETE = 0,
    SYSTEM_SOUND_ABORTED,
    SYSTEM_SOUND_OUTPUT_ERROR,
} system_sound_result_t;

typedef struct {
    vario_result_t result;
    bool bmp_ready;
    bool bmp_bus_failed;
    bool bus_timeout_detected;
    bool imu_ready;
    bool imu_interrupt_pending;
    int64_t next_bmp_deadline_us;
    int64_t next_bmp_retry_us;
    int64_t next_imu_retry_us;
    int64_t last_bmp_valid_us;
    int64_t last_imu_valid_us;
    uint32_t bmp_consecutive_errors;
    uint32_t imu_consecutive_errors;
    imu_diagnostics_t imu_diagnostics;
    imu_fusion_t imu_fusion;
    app_config_t imu_config;
    vario_estimator_t estimator;
    float estimator_reference_pressure_pa;
    bool imu_config_valid;
    bool estimator_reference_valid;
} sensor_task_state_t;

static const char *TAG = "app_tasks";

/* Application task handles are retained for diagnostics and future shutdown work. */
static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t audio_task_handle = NULL;
static TaskHandle_t system_task_handle = NULL;
static TaskHandle_t ble_tx_task_handle = NULL;
static TaskHandle_t console_task_handle = NULL;
static EventBits_t active_ack_mask = 0U;
static uint32_t serial_monitor_drop_count = 0U;

_Static_assert(configMAX_PRIORITIES >= 25, "SW_spec.md requires at least 25 priorities");
_Static_assert(SHUTDOWN_SOUND_TOTAL_MS < STARTUP_SOUND_WAIT_MS,
               "Startup sound wait must exceed the complete sound sequence");

static const system_sound_step_t startup_sound_steps[] = {
    {SYSTEM_SOUND_LOW_HZ, SYSTEM_SOUND_LOW_MS},
    {0U, SYSTEM_SOUND_SILENCE_MS},
    {SYSTEM_SOUND_HIGH_HZ, SYSTEM_SOUND_HIGH_MS},
};

static const system_sound_step_t shutdown_sound_steps[] = {
    {SYSTEM_SOUND_HIGH_HZ, SYSTEM_SOUND_HIGH_MS},
    {0U, SYSTEM_SOUND_SILENCE_MS},
    {SYSTEM_SOUND_LOW_HZ, SYSTEM_SOUND_LOW_MS},
};

static bool app_stop_requested(void) {
    EventGroupHandle_t event_group = app_resources_event_group();
    EventBits_t bits = 0U;

    if (event_group == NULL) {
        return false;
    }
    bits = xEventGroupGetBits(event_group);
    return (bits & APP_EVENT_STOP_REQUEST) != 0U;
}

static bool app_fatal_state(void) {
    EventGroupHandle_t event_group = app_resources_event_group();
    EventBits_t bits = 0U;

    if (event_group == NULL) {
        return true;
    }
    bits = xEventGroupGetBits(event_group);
    return (bits & APP_EVENT_FATAL_STATE) != 0U;
}

static EventBits_t app_event_bits(void) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (event_group == NULL) {
        return 0U;
    }
    return xEventGroupGetBits(event_group);
}

static void post_runtime_diagnostic(diagnostic_event_type_t type,
                                    esp_err_t detail) {
    diagnostic_event_t event = {
        .type = type,
        .timestamp_us = esp_timer_get_time(),
        .detail = (int32_t) detail,
    };

    (void) app_resources_post_diagnostic(&event);
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

#if CONFIG_CBV_IMU_HXY_ENABLE
static void set_imu_lifecycle_state(bool calibrating, bool degraded) {
    EventGroupHandle_t event_group = app_resources_event_group();
    EventBits_t set_bits = 0U;

    if (event_group == NULL) {
        return;
    }
    if (calibrating) {
        set_bits |= APP_EVENT_IMU_CALIBRATING;
    }
    if (degraded) {
        set_bits |= APP_EVENT_IMU_DEGRADED;
    }
    (void) xEventGroupClearBits(
        event_group, APP_EVENT_IMU_CALIBRATING | APP_EVENT_IMU_DEGRADED);
    if (set_bits != 0U) {
        (void) xEventGroupSetBits(event_group, set_bits);
    }
}
#endif

static bool led_first_phase(uint32_t elapsed_ms, uint32_t phase_ms) {
    uint32_t cycle_ms = phase_ms * UINT32_C(2);

    return elapsed_ms % cycle_ms < phase_ms;
}

static bool led_firefly_on(uint32_t elapsed_ms, uint32_t cycle_ms) {
    uint32_t position_ms = elapsed_ms % cycle_ms;
    uint32_t half_cycle_ms = cycle_ms / 2U;
    uint32_t brightness_steps = 0U;
    uint32_t pwm_step =
        (elapsed_ms / SYSTEM_SAMPLE_PERIOD_MS) % LED_SOFTWARE_PWM_STEPS;

    if (position_ms < half_cycle_ms) {
        brightness_steps =
            position_ms * LED_SOFTWARE_PWM_STEPS / half_cycle_ms;
    } else {
        brightness_steps =
            (cycle_ms - position_ms) * LED_SOFTWARE_PWM_STEPS /
            half_cycle_ms;
    }
    return pwm_step < brightness_steps;
}

static void set_fatal_leds(uint32_t elapsed_ms, bool bmp581_startup_failure) {
    bool first_phase = led_first_phase(elapsed_ms, FATAL_LED_PHASE_MS);

    if (bmp581_startup_failure) {
        board_set_status_leds(false, first_phase);
    } else {
        board_set_status_leds(first_phase, !first_phase);
    }
}

static uint32_t sw1_green_brightness_percent(uint32_t hold_ms) {
    if (hold_ms >= POWER_OFF_HOLD_MS) {
        return 0U;
    }
    return 100U - hold_ms * 100U / POWER_OFF_HOLD_MS;
}

static void set_lifecycle_leds(uint32_t elapsed_ms, uint32_t sw1_hold_ms) {
    EventBits_t bits = app_event_bits();
    vario_result_t result = {0};
    bool green = true;
    bool yellow = false;
    bool green_selected = false;
    bool yellow_selected = false;
    bool recovering = (bits & APP_EVENT_BMP581_RECOVERING) != 0U;

    if ((bits & APP_EVENT_FATAL_STATE) != 0U) {
        set_fatal_leds(elapsed_ms, (bits & APP_EVENT_FATAL_BMP581) != 0U);
        return;
    }
    if ((bits & APP_EVENT_BMP581_STARTUP_COMPLETE) == 0U) {
        board_set_status_leds(false, false);
        return;
    }

    if (app_resources_copy_vario(&result)) {
        bool invalid_or_stale =
            !result.pressure_valid ||
            (!result.climb_rate_valid && !result.estimator_warming_up &&
             !recovering);

        if (invalid_or_stale) {
            green = false;
            yellow = led_first_phase(elapsed_ms, FATAL_LED_PHASE_MS);
            green_selected = true;
            yellow_selected = true;
        }
        if (!green_selected && result.estimator_warming_up) {
            green =
                led_first_phase(elapsed_ms, BMP_RECOVERY_LED_PHASE_MS);
            green_selected = true;
        }
    }
    if (!green_selected && recovering) {
        green = led_first_phase(elapsed_ms, BMP_RECOVERY_LED_PHASE_MS);
        green_selected = true;
    }
    if (!green_selected && (bits & APP_EVENT_IMU_CALIBRATING) != 0U) {
        green =
            led_firefly_on(elapsed_ms, IMU_CALIBRATION_LED_CYCLE_MS);
        green_selected = true;
    }
    if (!green_selected && (bits & APP_EVENT_IMU_DEGRADED) != 0U) {
        green = led_firefly_on(elapsed_ms, IMU_DEGRADED_LED_CYCLE_MS);
        green_selected = true;
    }
    if (!yellow_selected && ble_vario_notify_active()) {
        yellow = elapsed_ms % UINT32_C(1000) < UINT32_C(100);
        yellow_selected = true;
    }
    if (!green_selected) {
        green = true;
    }
    if (!yellow_selected) {
        yellow = false;
    }
    if (sw1_hold_ms > 0U) {
        board_set_status_leds_brightness(
            sw1_green_brightness_percent(sw1_hold_ms), yellow);
    } else {
        board_set_status_leds(green, yellow);
    }
}

static audio_volume_level_t config_volume_level(
    const app_config_t *config) {
    if (config == NULL || !config->audio_enabled) {
        return AUDIO_VOLUME_MUTE;
    }
    switch (config->audio_amp_mode) {
    case 2U:
        return AUDIO_VOLUME_MEDIUM;
    case 3U:
        return AUDIO_VOLUME_LARGE;
    case 1U:
    default:
        return AUDIO_VOLUME_SMALL;
    }
}

static audio_volume_level_t next_volume_level(
    audio_volume_level_t current) {
    switch (current) {
    case AUDIO_VOLUME_SMALL:
        return AUDIO_VOLUME_MEDIUM;
    case AUDIO_VOLUME_MEDIUM:
        return AUDIO_VOLUME_LARGE;
    case AUDIO_VOLUME_LARGE:
        return AUDIO_VOLUME_MUTE;
    case AUDIO_VOLUME_MUTE:
    default:
        return AUDIO_VOLUME_SMALL;
    }
}

static void apply_audio_overrides(app_config_t *config,
                                  const system_snapshot_t *system) {
    if (config == NULL || system == NULL) {
        return;
    }
    if (system->volume_override_active) {
        if (system->volume_level == AUDIO_VOLUME_MUTE) {
            config->audio_enabled = false;
        } else {
            config->audio_enabled = true;
            config->audio_amp_mode =
                (uint32_t) system->volume_level + UINT32_C(1);
        }
    }
    if (system->sink_override_active) {
        config->sink_enabled = system->sink_enabled_override;
    }
}

static bool debounce_button(button_debounce_t *state, bool pressed) {
    if (state == NULL) {
        return false;
    }

    if (pressed != state->candidate_pressed) {
        state->candidate_pressed = pressed;
        state->stable_time_ms = SYSTEM_SAMPLE_PERIOD_MS;
        return state->stable_pressed;
    }

    if (state->stable_time_ms < SWITCH_DEBOUNCE_MS) {
        state->stable_time_ms += SYSTEM_SAMPLE_PERIOD_MS;
    }
    if (state->stable_time_ms >= SWITCH_DEBOUNCE_MS) {
        state->stable_pressed = state->candidate_pressed;
        state->stable_valid = true;
    }
    return state->stable_pressed;
}

static void acknowledge_and_delete(EventBits_t acknowledgement_bit) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group, acknowledgement_bit);
    }
    vTaskDelete(NULL);
}

static void block_safe_stop_light_sleep(void) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group, APP_EVENT_SAFE_SLEEP_BLOCKED);
    }
}

static void add_saturating_u32(uint32_t *counter, uint32_t increment) {
    if (counter == NULL) {
        return;
    }
    if (UINT32_MAX - *counter < increment) {
        *counter = UINT32_MAX;
    } else {
        *counter += increment;
    }
}

static void sensor_record_bmp_error(sensor_task_state_t *state,
                                    esp_err_t error,
                                    bool active_transfer) {
    if (state == NULL) {
        return;
    }

    add_saturating_u32(&state->result.i2c_error_count, 1U);
    add_saturating_u32(&state->bmp_consecutive_errors, 1U);
    if (active_transfer) {
        state->bmp_bus_failed = true;
    }
    if (error == ESP_ERR_TIMEOUT) {
        state->bus_timeout_detected = true;
    }
}

#if CONFIG_CBV_IMU_HXY_ENABLE
static void sensor_record_imu_error(sensor_task_state_t *state,
                                    esp_err_t error) {
    if (state == NULL) {
        return;
    }
    add_saturating_u32(&state->result.i2c_error_count, 1U);
    add_saturating_u32(&state->imu_consecutive_errors, 1U);
    state->imu_diagnostics.consecutive_error_count =
        state->imu_consecutive_errors;
    state->imu_diagnostics.last_error = (int32_t) error;
    if (error == ESP_ERR_TIMEOUT) {
        state->bus_timeout_detected = true;
    }
}

static void sensor_try_initialize_imu(sensor_task_state_t *state,
                                      i2c_master_bus_handle_t bus_handle,
                                      int64_t now_us) {
    icm42688_hxy_identity_t identity = {0};
    esp_err_t ret = ESP_OK;

    if (state == NULL || bus_handle == NULL || state->imu_ready ||
        now_us < state->next_imu_retry_us) {
        return;
    }

    add_saturating_u32(&state->imu_diagnostics.retry_count, 1U);
    ret = icm42688_hxy_init(bus_handle, xTaskGetCurrentTaskHandle(),
                            &identity);
    state->imu_diagnostics.who_am_i = identity.who_am_i;
    state->imu_diagnostics.last_error = (int32_t) ret;
    if (ret == ESP_OK) {
        state->imu_ready = true;
        state->imu_interrupt_pending = true;
        state->imu_consecutive_errors = 0U;
        state->last_imu_valid_us = now_us;
        state->imu_diagnostics.online = true;
        state->imu_diagnostics.configured = true;
        state->imu_diagnostics.stale = false;
        state->imu_diagnostics.address = identity.address;
        state->result.imu_online = true;
        state->result.imu_stale = false;
        imu_fusion_reset(&state->imu_fusion);
        vario_estimator_disable_fusion(&state->estimator);
        state->imu_config_valid = false;
        set_imu_lifecycle_state(true, false);
    } else {
        state->imu_ready = false;
        state->imu_diagnostics.online = false;
        state->imu_diagnostics.configured = false;
        state->imu_diagnostics.calibrated = false;
        state->imu_diagnostics.attitude_valid = false;
        state->imu_diagnostics.fusion_active = false;
        state->imu_diagnostics.stale = true;
        state->imu_diagnostics.address = 0U;
        state->result.imu_online = false;
        state->result.imu_calibrated = false;
        state->result.imu_stale = true;
        state->result.imu_fusion_active = false;
        state->result.vertical_accel_valid = false;
        state->next_imu_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
        set_imu_lifecycle_state(false, true);
    }
    (void) app_resources_publish_imu_diagnostics(&state->imu_diagnostics);
}
#endif

static void sensor_shutdown_devices(void) {
    esp_err_t bmp_ret = bmp581_deinit();

    if (bmp_ret != ESP_OK) {
        ESP_LOGW(TAG, "sensor shutdown incomplete: bmp=%s", esp_err_to_name(bmp_ret));
    }
#if CONFIG_CBV_IMU_HXY_ENABLE
    {
        esp_err_t imu_ret = icm42688_hxy_deinit();

        if (imu_ret != ESP_OK) {
            ESP_LOGW(TAG, "sensor shutdown incomplete: hxy_imu=%s",
                     esp_err_to_name(imu_ret));
        }
    }
#endif
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

#if CONFIG_CBV_IMU_HXY_ENABLE
static void sensor_invalidate_imu(sensor_task_state_t *state, bool stale) {
    if (state == NULL) {
        return;
    }
    state->imu_ready = false;
    state->imu_interrupt_pending = false;
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
    memset(state->imu_diagnostics.quaternion, 0,
           sizeof(state->imu_diagnostics.quaternion));
    state->imu_diagnostics.roll_deg = 0.0f;
    state->imu_diagnostics.pitch_deg = 0.0f;
    state->imu_diagnostics.yaw_deg = 0.0f;
    imu_fusion_reset(&state->imu_fusion);
    vario_estimator_disable_fusion(&state->estimator);
    set_imu_lifecycle_state(false, true);
}
#endif

static bool sensor_recover_shared_bus(sensor_task_state_t *state, int64_t now_us) {
    esp_err_t ret = ESP_OK;
#if CONFIG_CBV_IMU_HXY_ENABLE
    esp_err_t imu_ret = ESP_OK;
    bool imu_was_ready = false;
#endif

    if (state == NULL || !state->bus_timeout_detected) {
        return false;
    }

    ESP_LOGW(TAG, "recovering shared I2C bus after transaction timeout");
    (void) bmp581_deinit();
#if CONFIG_CBV_IMU_HXY_ENABLE
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
#endif
    ret = sensor_bus_recover();

    state->bmp_ready = false;
    state->result.bmp581_online = false;
    sensor_invalidate_estimate(state, true);
    set_bmp581_recovering(true);
    state->bmp_consecutive_errors = 0U;
    state->bmp_bus_failed = false;
    state->bus_timeout_detected = false;
    if (ret == ESP_OK) {
        state->next_bmp_retry_us = now_us;
#if CONFIG_CBV_IMU_HXY_ENABLE
        state->next_imu_retry_us = now_us;
#endif
    } else {
        ESP_LOGW(TAG, "I2C recovery failed: %s", esp_err_to_name(ret));
        state->next_bmp_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
#if CONFIG_CBV_IMU_HXY_ENABLE
        state->next_imu_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
#endif
    }
#if CONFIG_CBV_IMU_HXY_ENABLE
    (void) app_resources_publish_imu_diagnostics(&state->imu_diagnostics);
#endif
    return true;
}

static bool sensor_try_initialize_devices(sensor_task_state_t *state, int64_t now_us) {
    i2c_master_bus_handle_t bus_handle = sensor_bus_get_handle();
    bool changed = false;
    esp_err_t ret = ESP_OK;

    if (state == NULL) {
        return false;
    }

    if (bus_handle == NULL && now_us >= state->next_bmp_retry_us) {
        ret = sensor_bus_init();
        if (ret != ESP_OK) {
            add_saturating_u32(&state->result.i2c_error_count, 1U);
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
            state->next_bmp_deadline_us = now_us;
            set_bmp581_recovering(false);
        } else {
            sensor_record_bmp_error(state, ret, false);
            state->next_bmp_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
            set_bmp581_recovering(true);
        }
        changed = true;
    }

#if CONFIG_CBV_IMU_HXY_ENABLE
    if (!state->bus_timeout_detected) {
        sensor_try_initialize_imu(state, bus_handle, now_us);
    }
#endif

    return changed;
}

#if CONFIG_CBV_IMU_HXY_ENABLE
static bool imu_configs_match(const app_config_t *left,
                              const app_config_t *right) {
    if (left == NULL || right == NULL) {
        return false;
    }
    return left->imu_gyro_calibration_samples ==
               right->imu_gyro_calibration_samples &&
           left->imu_accel_correction_min_g ==
               right->imu_accel_correction_min_g &&
           left->imu_accel_correction_max_g ==
               right->imu_accel_correction_max_g &&
           left->imu_mahony_kp == right->imu_mahony_kp &&
           left->imu_mahony_ki == right->imu_mahony_ki &&
           left->imu_accel_x_source == right->imu_accel_x_source &&
           left->imu_accel_y_source == right->imu_accel_y_source &&
           left->imu_accel_z_source == right->imu_accel_z_source &&
           left->imu_accel_x_sign == right->imu_accel_x_sign &&
           left->imu_accel_y_sign == right->imu_accel_y_sign &&
           left->imu_accel_z_sign == right->imu_accel_z_sign &&
           left->imu_gyro_x_source == right->imu_gyro_x_source &&
           left->imu_gyro_y_source == right->imu_gyro_y_source &&
           left->imu_gyro_z_source == right->imu_gyro_z_source &&
           left->imu_gyro_x_sign == right->imu_gyro_x_sign &&
           left->imu_gyro_y_sign == right->imu_gyro_y_sign &&
           left->imu_gyro_z_sign == right->imu_gyro_z_sign;
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

static bool sensor_process_imu(sensor_task_state_t *state, int64_t now_us) {
    icm42688_hxy_sample_t hxy_sample = {0};
    imu_sample_t sensor_sample = {0};
    imu_sample_t board_sample = {0};
    imu_fusion_output_t fusion_output = {0};
    app_config_t config = {0};
    uint32_t error_limit = CONFIG_CBV_SENSOR_CONSECUTIVE_ERROR_LIMIT;
    esp_err_t ret = ESP_OK;

    if (state == NULL || !state->imu_ready ||
        !state->imu_interrupt_pending) {
        return false;
    }
    state->imu_interrupt_pending = false;
    ret = icm42688_hxy_read_sample(&hxy_sample);
    if (ret == ESP_ERR_NOT_FINISHED) {
        return false;
    }
    if (ret != ESP_OK) {
        sensor_record_imu_error(state, ret);
        if (app_resources_copy_config(&config)) {
            error_limit = config.i2c_reinit_error_count;
        }
        if (state->imu_consecutive_errors >= error_limit) {
            ESP_LOGW(TAG,
                     "HXY IMU offline after %" PRIu32
                     " consecutive errors",
                     state->imu_consecutive_errors);
            (void) icm42688_hxy_deinit();
            sensor_invalidate_imu(state, true);
            state->next_imu_retry_us =
                now_us + SENSOR_RETRY_INTERVAL_US;
        }
        (void) app_resources_publish_imu_diagnostics(
            &state->imu_diagnostics);
        return true;
    }

    state->imu_consecutive_errors = 0U;
    state->last_imu_valid_us = hxy_sample.timestamp_us;
    state->result.imu_online = true;
    state->result.imu_stale = false;
    state->imu_diagnostics.online = true;
    state->imu_diagnostics.configured = true;
    state->imu_diagnostics.stale = false;
    state->imu_diagnostics.last_error = (int32_t) ESP_OK;
    state->imu_diagnostics.consecutive_error_count = 0U;
    state->imu_diagnostics.data_status = hxy_sample.data_status;
    add_saturating_u32(&state->imu_diagnostics.sample_count, 1U);

    if (!app_resources_copy_config(&config)) {
        app_config_set_defaults(&config);
    }
    if (!state->imu_config_valid ||
        !imu_configs_match(&state->imu_config, &config)) {
        sensor_restart_imu_fusion(state, &config);
    }

    memcpy(sensor_sample.accel_mps2, hxy_sample.accel_mps2,
           sizeof(sensor_sample.accel_mps2));
    memcpy(sensor_sample.gyro_radps, hxy_sample.gyro_radps,
           sizeof(sensor_sample.gyro_radps));
    sensor_sample.timestamp_us = hxy_sample.timestamp_us;
    sensor_sample.valid = hxy_sample.valid;
    if (!imu_fusion_apply_axis_map(&sensor_sample, &config,
                                   &board_sample) ||
        !imu_fusion_update(&state->imu_fusion, &board_sample, &config,
                           &fusion_output)) {
        sensor_restart_imu_fusion(state, &config);
        (void) app_resources_publish_imu_diagnostics(
            &state->imu_diagnostics);
        return true;
    }

    state->imu_diagnostics.accel_norm_g = fusion_output.accel_norm_g;
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
        if (config.filter_mode == APP_FILTER_MODE_AUTO) {
            if (!vario_estimator_update_imu(
                    &state->estimator,
                    fusion_output.vertical_accel_mps2,
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
    (void) app_resources_publish_imu_diagnostics(
        &state->imu_diagnostics);
    return true;
}
#endif

static bool sensor_process_bmp581(sensor_task_state_t *state, int64_t now_us) {
    bmp581_sample_t sample = {0};
    app_config_t config = {0};
    vario_estimate_t estimate = {0};
    int64_t periods_elapsed = 0;
    uint32_t error_limit = CONFIG_CBV_SENSOR_CONSECUTIVE_ERROR_LIMIT;
    esp_err_t ret = ESP_OK;

    if (state == NULL || !state->bmp_ready || now_us < state->next_bmp_deadline_us) {
        return false;
    }

    periods_elapsed = (now_us - state->next_bmp_deadline_us) / BMP581_SAMPLE_PERIOD_US;
    if (periods_elapsed > 0) {
        uint32_t overrun_increment = UINT32_MAX;

        if (periods_elapsed <= (int64_t) UINT32_MAX) {
            overrun_increment = (uint32_t) periods_elapsed;
        }
        add_saturating_u32(&state->result.bmp_period_overrun_count,
                           overrun_increment);
    }
    state->next_bmp_deadline_us += (periods_elapsed + 1) * BMP581_SAMPLE_PERIOD_US;

    ret = bmp581_read_sample(&sample);
    if (ret != ESP_OK) {
        sensor_record_bmp_error(state, ret, true);
        if (app_resources_copy_config(&config)) {
            error_limit = config.i2c_reinit_error_count;
        }
        if (state->bmp_consecutive_errors >= error_limit) {
            ESP_LOGW(TAG, "BMP581 offline after %" PRIu32 " consecutive errors",
                     state->bmp_consecutive_errors);
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
    if (sample.valid) {
        state->last_bmp_valid_us = sample.timestamp_us;
        add_saturating_u32(&state->result.sequence, 1U);
        state->result.timestamp_us = sample.timestamp_us;
        state->result.raw_temperature = sample.raw_temperature;
        state->result.raw_pressure = sample.raw_pressure;
        state->result.temperature_c_x100 = sample.temperature_c_x100;
        state->result.pressure_pa_x100 = sample.pressure_pa_x100;
        state->result.pressure_valid = true;
        state->result.bmp581_online = true;
        if (!app_resources_copy_config(&config)) {
            app_config_set_defaults(&config);
        }
        if (!state->estimator_reference_valid ||
            fabsf(config.sea_level_pressure_pa -
                  state->estimator_reference_pressure_pa) > 0.01f) {
            vario_estimator_reset(&state->estimator);
            state->estimator_reference_pressure_pa =
                config.sea_level_pressure_pa;
            state->estimator_reference_valid = true;
        }
        if (vario_estimator_update(&state->estimator, sample.pressure_pa_x100,
                                   sample.timestamp_us,
                                   config.sea_level_pressure_pa,
                                   config.filter_mode == APP_FILTER_MODE_AUTO &&
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
        state->imu_diagnostics.fusion_active =
            state->result.imu_fusion_active;
        (void) app_resources_publish_imu_diagnostics(
            &state->imu_diagnostics);
    }
    return true;
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
#if CONFIG_CBV_IMU_HXY_ENABLE
    if (state->imu_ready &&
        now_us - state->last_imu_valid_us > IMU_STALE_TIMEOUT_US) {
        ESP_LOGW(TAG, "HXY IMU stale; falling back to pressure-only mode");
        (void) icm42688_hxy_deinit();
        sensor_invalidate_imu(state, true);
        state->next_imu_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
        state->imu_diagnostics.last_error =
            (int32_t) ESP_ERR_INVALID_STATE;
        (void) app_resources_publish_imu_diagnostics(
            &state->imu_diagnostics);
        changed = true;
    }
#endif
    return changed;
}

static TickType_t sensor_wait_ticks(const sensor_task_state_t *state, int64_t now_us) {
    int64_t wake_time_us = now_us + SENSOR_IDLE_WAKE_US;
    int64_t wait_us = 0;
    uint32_t wait_ms = 0U;

    if (state == NULL) {
        return pdMS_TO_TICKS(1U);
    }

    if (state->bmp_ready && state->next_bmp_deadline_us < wake_time_us) {
        wake_time_us = state->next_bmp_deadline_us;
    } else if (!state->bmp_ready && state->next_bmp_retry_us < wake_time_us) {
        wake_time_us = state->next_bmp_retry_us;
    }
    if (state->bmp_ready && state->last_bmp_valid_us + SENSOR_STALE_TIMEOUT_US < wake_time_us) {
        wake_time_us = state->last_bmp_valid_us + SENSOR_STALE_TIMEOUT_US;
    }
#if CONFIG_CBV_IMU_HXY_ENABLE
    if (state->imu_ready &&
        state->last_imu_valid_us + IMU_STALE_TIMEOUT_US < wake_time_us) {
        wake_time_us =
            state->last_imu_valid_us + IMU_STALE_TIMEOUT_US;
    } else if (!state->imu_ready &&
               state->next_imu_retry_us < wake_time_us) {
        wake_time_us = state->next_imu_retry_us;
    }
#endif

    wait_us = wake_time_us - now_us;
    if (wait_us <= 0) {
        return 0U;
    }
    wait_ms = (uint32_t) ((wait_us + INT64_C(999)) / INT64_C(1000));
    return pdMS_TO_TICKS(wait_ms);
}

static bool sensor_execute_work(sensor_task_state_t *state, int64_t now_us) {
    esp_err_t power_ret = app_power_sensor_work_begin();
    bool changed = false;

    if (power_ret != ESP_OK) {
        ESP_LOGW(TAG, "sensor CPU-frequency lock unavailable: %s", esp_err_to_name(power_ret));
        block_safe_stop_light_sleep();
    }

    changed |= sensor_try_initialize_devices(state, now_us);
#if CONFIG_CBV_IMU_HXY_ENABLE
    changed |= sensor_process_imu(state, esp_timer_get_time());
#endif
    changed |= sensor_process_bmp581(state, esp_timer_get_time());
    changed |= sensor_check_stale(state, esp_timer_get_time());
    changed |= sensor_recover_shared_bus(state, esp_timer_get_time());

    if (power_ret == ESP_OK) {
        power_ret = app_power_sensor_work_end();
        if (power_ret != ESP_OK) {
            ESP_LOGW(TAG, "sensor CPU-frequency lock release failed: %s",
                     esp_err_to_name(power_ret));
            block_safe_stop_light_sleep();
        }
    }
    return changed;
}

static void sensor_task(void *context) {
    sensor_task_state_t state = {0};
    EventGroupHandle_t event_group = app_resources_event_group();
    bool watchdog_registered = false;
    bool fatal_shutdown_complete = false;

    (void) context;
    ESP_LOGI(TAG, "sensor_task started on core %d", xPortGetCoreID());
    if (esp_task_wdt_add(NULL) == ESP_OK) {
        watchdog_registered = true;
    } else {
        ESP_LOGW(TAG, "sensor_task watchdog registration failed");
    }
    state.next_bmp_retry_us = esp_timer_get_time();
    state.next_imu_retry_us = state.next_bmp_retry_us;
    state.result.timestamp_us = state.next_bmp_retry_us;
#if CONFIG_CBV_IMU_HXY_ENABLE
    state.imu_diagnostics.enabled = true;
    state.imu_diagnostics.last_error = (int32_t) ESP_ERR_NOT_FOUND;
    imu_fusion_reset(&state.imu_fusion);
#else
    state.imu_diagnostics.enabled = false;
    state.imu_diagnostics.last_error = (int32_t) ESP_ERR_NOT_SUPPORTED;
#endif
    (void) app_resources_publish_imu_diagnostics(&state.imu_diagnostics);

    if (sensor_execute_work(&state, state.next_bmp_retry_us)) {
        (void) app_resources_publish_vario(&state.result);
    }
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
        bool snapshot_changed = false;
        int64_t now_us = 0;
        uint32_t notification_count = 0U;

        if (app_stop_requested()) {
            sensor_shutdown_devices();
            if (watchdog_registered) {
                (void) esp_task_wdt_delete(NULL);
            }
            acknowledge_and_delete(APP_EVENT_SENSOR_ACK);
        }
        if (app_fatal_state()) {
            if (!fatal_shutdown_complete) {
                sensor_shutdown_devices();
                fatal_shutdown_complete = true;
            }
            if (watchdog_registered) {
                (void) esp_task_wdt_delete(NULL);
                watchdog_registered = false;
            }
            vTaskDelay(pdMS_TO_TICKS(100U));
            continue;
        }

        now_us = esp_timer_get_time();
        notification_count =
            ulTaskNotifyTake(pdTRUE, sensor_wait_ticks(&state, now_us));
#if CONFIG_CBV_IMU_HXY_ENABLE
        if (notification_count > 0U) {
            state.imu_interrupt_pending = true;
            if (notification_count > 1U) {
                uint32_t missed_count = notification_count - 1U;

                add_saturating_u32(
                    &state.result.missed_imu_sample_count, missed_count);
                add_saturating_u32(
                    &state.imu_diagnostics.missed_interrupt_count,
                    missed_count);
            }
        }
#else
        (void) notification_count;
#endif
        now_us = esp_timer_get_time();

        snapshot_changed |= sensor_execute_work(&state, now_us);

        if (snapshot_changed) {
            (void) app_resources_publish_vario(&state.result);
        }

        if (watchdog_registered) {
            (void) esp_task_wdt_reset();
        }
    }
}

static bool system_sound_abort_requested(EventBits_t abort_mask) {
    EventGroupHandle_t event_group = app_resources_event_group();
    EventBits_t bits =
        event_group == NULL ? 0U : xEventGroupGetBits(event_group);

    return (bits & abort_mask) != 0U;
}

static bool system_sound_delay(uint32_t duration_ms,
                               EventBits_t abort_mask,
                               bool watchdog_registered) {
    uint32_t elapsed_ms = 0U;

    while (elapsed_ms < duration_ms) {
        uint32_t delay_ms =
            duration_ms - elapsed_ms > AUDIO_EVALUATION_PERIOD_MS
                ? AUDIO_EVALUATION_PERIOD_MS
                : duration_ms - elapsed_ms;

        if (system_sound_abort_requested(abort_mask)) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        elapsed_ms += delay_ms;
        if (watchdog_registered) {
            (void) esp_task_wdt_reset();
        }
    }
    return true;
}

static system_sound_result_t play_system_sound(
    const system_sound_step_t *steps, size_t step_count,
    EventBits_t abort_mask, bool watchdog_registered) {
    system_sound_result_t result = SYSTEM_SOUND_COMPLETE;

    audio_output_shutdown();
    for (size_t index = 0U; index < step_count; index++) {
        if (system_sound_abort_requested(abort_mask)) {
            result = SYSTEM_SOUND_ABORTED;
            break;
        }
        if (steps[index].frequency_hz == 0U) {
            audio_output_shutdown();
        } else {
            esp_err_t ret = audio_output_apply(
                steps[index].frequency_hz, SYSTEM_SOUND_DUTY_PERCENT,
                SYSTEM_SOUND_AMPLIFIER_MODE);

            if (ret != ESP_OK) {
                ESP_LOGW(TAG,
                         "system sound step %u could not start: %s",
                         (unsigned int) index, esp_err_to_name(ret));
                post_runtime_diagnostic(
                    DIAGNOSTIC_EVENT_PERIPHERAL_FAILURE, ret);
                result = SYSTEM_SOUND_OUTPUT_ERROR;
                break;
            }
        }
        if (!system_sound_delay(steps[index].duration_ms, abort_mask,
                                watchdog_registered)) {
            result = SYSTEM_SOUND_ABORTED;
            break;
        }
        audio_output_shutdown();
    }
    audio_output_shutdown();
    return result;
}

static void audio_task(void *context) {
    QueueHandle_t queue = app_resources_audio_queue();
    vario_result_t result = {0};
    system_snapshot_t system = {0};
    app_config_t config = {0};
    vario_audio_state_t audio_state = {0};
    vario_audio_command_t command = {0};
    bool watchdog_registered = false;

    (void) context;
    ESP_LOGI(TAG, "audio_task started on core %d", xPortGetCoreID());
    audio_output_shutdown();
    app_config_set_defaults(&config);
    vario_audio_reset(&audio_state);
    if (esp_task_wdt_add(NULL) == ESP_OK) {
        watchdog_registered = true;
    } else {
        ESP_LOGW(TAG, "audio_task watchdog registration failed");
    }

    for (;;) {
        if (app_stop_requested()) {
            EventGroupHandle_t event_group = app_resources_event_group();
            system_sound_result_t sound_result = SYSTEM_SOUND_ABORTED;

            audio_output_shutdown();
            if (event_group != NULL) {
                (void) xEventGroupClearBits(
                    event_group, APP_EVENT_STARTUP_SOUND_REQUEST);
                (void) xEventGroupSetBits(
                    event_group,
                    APP_EVENT_AUDIO_QUIESCED |
                        APP_EVENT_STARTUP_SOUND_ABORT |
                        APP_EVENT_STARTUP_SOUND_DONE);
            }
            for (;;) {
                EventBits_t bits = event_group == NULL
                                       ? APP_EVENT_SHUTDOWN_SOUND_ABORT
                                       : xEventGroupGetBits(event_group);

                if ((bits & APP_EVENT_SHUTDOWN_SOUND_ABORT) != 0U) {
                    break;
                }
                if ((bits & APP_EVENT_SHUTDOWN_SOUND_REQUEST) != 0U) {
                    sound_result = play_system_sound(
                        shutdown_sound_steps,
                        sizeof(shutdown_sound_steps) /
                            sizeof(shutdown_sound_steps[0]),
                        APP_EVENT_SHUTDOWN_SOUND_ABORT,
                        watchdog_registered);
                    break;
                }
                if (watchdog_registered) {
                    (void) esp_task_wdt_reset();
                }
                vTaskDelay(pdMS_TO_TICKS(AUDIO_EVALUATION_PERIOD_MS));
            }
            audio_output_shutdown();
            if (sound_result != SYSTEM_SOUND_ABORTED &&
                event_group != NULL) {
                (void) xEventGroupSetBits(event_group,
                                          APP_EVENT_SHUTDOWN_SOUND_DONE);
            }
            if (watchdog_registered) {
                (void) esp_task_wdt_delete(NULL);
            }
            acknowledge_and_delete(APP_EVENT_AUDIO_ACK);
            return;
        }
        if (app_fatal_state()) {
            audio_output_shutdown();
            if (watchdog_registered) {
                (void) esp_task_wdt_delete(NULL);
                watchdog_registered = false;
            }
            vTaskDelay(pdMS_TO_TICKS(AUDIO_EVALUATION_PERIOD_MS));
            continue;
        }
        EventGroupHandle_t event_group = app_resources_event_group();
        EventBits_t event_bits =
            event_group == NULL ? 0U : xEventGroupGetBits(event_group);

        if ((event_bits & APP_EVENT_STARTUP_SOUND_REQUEST) != 0U) {
            system_sound_result_t sound_result = play_system_sound(
                startup_sound_steps,
                sizeof(startup_sound_steps) /
                    sizeof(startup_sound_steps[0]),
                APP_EVENT_STARTUP_SOUND_ABORT | APP_EVENT_STOP_REQUEST |
                    APP_EVENT_FATAL_STATE,
                watchdog_registered);

            audio_output_shutdown();
            vario_audio_reset(&audio_state);
            if (sound_result == SYSTEM_SOUND_OUTPUT_ERROR) {
                ESP_LOGW(TAG, "startup sound output failed; continuing startup");
            }
            if (event_group != NULL) {
                (void) xEventGroupClearBits(
                    event_group,
                    APP_EVENT_STARTUP_SOUND_REQUEST |
                        APP_EVENT_STARTUP_SOUND_ABORT);
                (void) xEventGroupSetBits(event_group,
                                          APP_EVENT_STARTUP_SOUND_DONE);
            }
            continue;
        }

        if (queue != NULL) {
            (void) xQueueReceive(queue, &result, pdMS_TO_TICKS(AUDIO_EVALUATION_PERIOD_MS));
        } else {
            vTaskDelay(pdMS_TO_TICKS(AUDIO_EVALUATION_PERIOD_MS));
        }
        (void) app_resources_apply_debug_vario(&result,
                                               esp_timer_get_time());
        (void) app_resources_copy_config(&config);
        if (app_resources_copy_system(&system)) {
            apply_audio_overrides(&config, &system);
        }
        vario_audio_step(&audio_state, &config, &result, esp_timer_get_time(),
                         &command);
        if (command.sounding) {
            if (audio_output_apply(command.frequency_hz, command.duty_percent,
                                   command.amplifier_mode) != ESP_OK) {
                audio_output_shutdown();
            }
        } else {
            audio_output_shutdown();
        }
        if (watchdog_registered) {
            (void) esp_task_wdt_reset();
        }
    }
}

static uint32_t shutdown_remaining_ms(int64_t deadline_us) {
    int64_t remaining_us = deadline_us - esp_timer_get_time();

    if (remaining_us <= 0) {
        return 0U;
    }
    if (remaining_us / INT64_C(1000) >= (int64_t) UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t) ((remaining_us + INT64_C(999)) / INT64_C(1000));
}

static bool wait_for_shutdown_bits(EventGroupHandle_t event_group,
                                   EventBits_t wait_mask,
                                   int64_t deadline_us) {
    uint32_t remaining_ms = shutdown_remaining_ms(deadline_us);
    EventBits_t bits = 0U;

    if (wait_mask == 0U) {
        return true;
    }
    if (event_group == NULL || remaining_ms == 0U) {
        return false;
    }
    bits = xEventGroupWaitBits(event_group, wait_mask, pdFALSE, pdTRUE,
                               pdMS_TO_TICKS(remaining_ms));
    return (bits & wait_mask) == wait_mask;
}

static void run_safe_stop_loop(EventGroupHandle_t event_group,
                               EventBits_t wait_mask,
                               bool safe_sleep_enabled) {
    button_debounce_t sw1 = {0};
    uint32_t hold_time_ms = 0U;
    bool was_released = false;

    sw1.candidate_pressed = board_is_sw1_pressed();
    sw1.stable_pressed = sw1.candidate_pressed;

    for (;;) {
        bool pressed = debounce_button(&sw1, board_is_sw1_pressed());

        if (sw1.stable_valid && !pressed) {
            was_released = true;
            hold_time_ms = 0U;
        } else if (sw1.stable_valid && was_released) {
            if (UINT32_MAX - hold_time_ms < SYSTEM_SAMPLE_PERIOD_MS) {
                hold_time_ms = UINT32_MAX;
            } else {
                hold_time_ms += SYSTEM_SAMPLE_PERIOD_MS;
            }
        }
        if (was_released && hold_time_ms >= POWER_OFF_HOLD_MS) {
            esp_restart();
        }

        if (!safe_sleep_enabled && event_group != NULL &&
            wait_mask != 0U) {
            EventBits_t bits = xEventGroupGetBits(event_group);

            if ((bits & wait_mask) == wait_mask &&
                (bits & APP_EVENT_SAFE_SLEEP_BLOCKED) == 0U &&
                !usb_device_bus_active()) {
                safe_sleep_enabled = app_power_enter_safe_stop() == ESP_OK;
            }
        }
        board_set_safe_indicators();
        vTaskDelay(pdMS_TO_TICKS(SAFE_STOP_PERIOD_MS));
    }
}

static void request_power_off(system_snapshot_t *snapshot) {
    EventGroupHandle_t event_group = app_resources_event_group();
    EventBits_t wait_mask = active_ack_mask & ~APP_EVENT_SYSTEM_ACK;
    int64_t shutdown_started_us = esp_timer_get_time();
    int64_t shutdown_deadline_us =
        shutdown_started_us +
        (int64_t) SHUTDOWN_DEADLINE_MS * INT64_C(1000);
    EventBits_t non_audio_wait_mask =
        wait_mask & ~APP_EVENT_AUDIO_ACK;
    EventBits_t quiesce_wait_mask =
        non_audio_wait_mask | APP_EVENT_AUDIO_QUIESCED;
    bool workers_quiesced = false;
    bool shutdown_sound_done = false;
    bool all_workers_stopped = false;
    bool safe_sleep_enabled = false;

    if (snapshot != NULL) {
        snapshot->power_off_requested = true;
        snapshot->timestamp_us = shutdown_started_us;
        (void) app_resources_publish_system(snapshot);
    }

    board_set_status_leds(false, false);
    if (event_group != NULL) {
        (void) xEventGroupClearBits(
            event_group,
            APP_EVENT_AUDIO_QUIESCED | APP_EVENT_SHUTDOWN_SOUND_REQUEST |
                APP_EVENT_SHUTDOWN_SOUND_DONE |
                APP_EVENT_SHUTDOWN_SOUND_ABORT);
        (void) xEventGroupSetBits(event_group, APP_EVENT_STOP_REQUEST);
    }

    ble_vario_begin_shutdown();
    workers_quiesced = wait_for_shutdown_bits(
        event_group, quiesce_wait_mask, shutdown_deadline_us);
    if (workers_quiesced &&
        shutdown_remaining_ms(shutdown_deadline_us) >
            SHUTDOWN_SOUND_TOTAL_MS) {
        (void) xEventGroupSetBits(event_group,
                                  APP_EVENT_SHUTDOWN_SOUND_REQUEST);
        shutdown_sound_done = wait_for_shutdown_bits(
            event_group,
            APP_EVENT_SHUTDOWN_SOUND_DONE | APP_EVENT_AUDIO_ACK,
            shutdown_deadline_us);
    }
    if (!shutdown_sound_done && event_group != NULL) {
        (void) xEventGroupSetBits(event_group,
                                  APP_EVENT_SHUTDOWN_SOUND_ABORT);
    }
    if (event_group != NULL) {
        EventBits_t bits = xEventGroupGetBits(event_group);

        all_workers_stopped = (bits & wait_mask) == wait_mask;
    } else {
        all_workers_stopped = wait_mask == 0U;
    }
    if (event_group != NULL &&
        (xEventGroupGetBits(event_group) & APP_EVENT_SAFE_SLEEP_BLOCKED) != 0U) {
        all_workers_stopped = false;
    }
    if (usb_device_bus_active()) {
        all_workers_stopped = false;
    }
    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group, APP_EVENT_SYSTEM_ACK);
    }
    board_set_safe_indicators();
    (void) board_set_power_hold(false);
    if (all_workers_stopped) {
        esp_err_t power_ret = app_power_enter_safe_stop();

        if (power_ret != ESP_OK) {
            ESP_LOGW(TAG, "safe-stop light sleep unavailable: %s", esp_err_to_name(power_ret));
        } else {
            safe_sleep_enabled = true;
        }
    } else {
        ESP_LOGW(TAG, "safe-stop light sleep blocked because worker shutdown timed out");
    }
    run_safe_stop_loop(event_group, wait_mask, safe_sleep_enabled);
}

static void system_task(void *context) {
    button_debounce_t sw1 = {0};
    button_debounce_t sw2 = {0};
    button_debounce_t sw3 = {0};
    system_snapshot_t snapshot = {0};
    uint32_t sw1_hold_time_ms = 0U;
    uint32_t battery_elapsed_ms = BATTERY_SAMPLE_PERIOD_MS;
    uint32_t led_elapsed_ms = 0U;
    bool sw1_was_released = false;
    bool previous_sw2_pressed = false;
    bool previous_sw3_pressed = false;

    (void) context;
    ESP_LOGI(TAG, "system_task started on core %d", xPortGetCoreID());

    sw1.candidate_pressed = board_is_sw1_pressed();
    sw1.stable_pressed = sw1.candidate_pressed;
    sw2.candidate_pressed = system_io_sw2_pressed();
    sw2.stable_pressed = sw2.candidate_pressed;
    sw3.candidate_pressed = system_io_sw3_pressed();
    sw3.stable_pressed = sw3.candidate_pressed;
    previous_sw2_pressed = sw2.stable_pressed;
    previous_sw3_pressed = sw3.stable_pressed;

    for (;;) {
        snapshot.timestamp_us = esp_timer_get_time();
        snapshot.sw1_pressed = debounce_button(&sw1, board_is_sw1_pressed());
        snapshot.sw2_pressed = debounce_button(&sw2, system_io_sw2_pressed());
        snapshot.sw3_pressed = debounce_button(&sw3, system_io_sw3_pressed());
        snapshot.external_power_present = system_io_external_power_present();

        if (sw1.stable_valid && !snapshot.sw1_pressed) {
            sw1_was_released = true;
            sw1_hold_time_ms = 0U;
        } else if (sw1.stable_valid && sw1_was_released) {
            if (UINT32_MAX - sw1_hold_time_ms <
                SYSTEM_SAMPLE_PERIOD_MS) {
                sw1_hold_time_ms = UINT32_MAX;
            } else {
                sw1_hold_time_ms += SYSTEM_SAMPLE_PERIOD_MS;
            }
        } else {
            /* The switch used to start the board is not a power-off request. */
        }
        snapshot.sw1_hold_ms = sw1_hold_time_ms;

        if (sw2.stable_valid && snapshot.sw2_pressed &&
            !previous_sw2_pressed) {
            app_config_t config = {0};
            audio_volume_level_t current = snapshot.volume_level;

            if (!snapshot.volume_override_active) {
                if (!app_resources_copy_config(&config)) {
                    app_config_set_defaults(&config);
                }
                current = config_volume_level(&config);
            }
            snapshot.volume_level = next_volume_level(current);
            snapshot.volume_override_active = true;
        }
        if (sw3.stable_valid && snapshot.sw3_pressed &&
            !previous_sw3_pressed) {
            app_config_t config = {0};
            bool current = snapshot.sink_enabled_override;

            if (!snapshot.sink_override_active) {
                if (!app_resources_copy_config(&config)) {
                    app_config_set_defaults(&config);
                }
                current = config.sink_enabled;
            }
            snapshot.sink_enabled_override = !current;
            snapshot.sink_override_active = true;
        }
        previous_sw2_pressed = snapshot.sw2_pressed;
        previous_sw3_pressed = snapshot.sw3_pressed;

        battery_elapsed_ms += SYSTEM_SAMPLE_PERIOD_MS;
        if (battery_elapsed_ms >= BATTERY_SAMPLE_PERIOD_MS) {
            system_io_battery_diagnostics_t diagnostics = {0};

            snapshot.battery_valid = system_io_read_battery_voltage(&snapshot.battery_voltage_v);
            system_io_get_battery_diagnostics(&diagnostics);
            snapshot.battery_raw = diagnostics.last_raw;
            snapshot.battery_adc_mv =
                diagnostics.last_calibrated_mv;
            snapshot.battery_sample_count =
                diagnostics.valid_sample_count;
            snapshot.battery_error_count = diagnostics.error_count;
            snapshot.battery_saturation_count =
                diagnostics.saturation_count;
            battery_elapsed_ms = 0U;
        }

        set_lifecycle_leds(led_elapsed_ms, sw1_hold_time_ms);
        led_elapsed_ms += SYSTEM_SAMPLE_PERIOD_MS;

        (void) app_resources_publish_system(&snapshot);

        if (sw1_was_released && sw1_hold_time_ms >= POWER_OFF_HOLD_MS) {
            request_power_off(&snapshot);
        }

        vTaskDelay(pdMS_TO_TICKS(SYSTEM_SAMPLE_PERIOD_MS));
    }
}

static bool console_writef(const char *format, ...) {
    char output[896] = {0};
    va_list arguments;
    int written = 0;

    va_start(arguments, format);
    written = vsnprintf(output, sizeof(output), format, arguments);
    va_end(arguments);
    if (written > 0 && (size_t) written < sizeof(output)) {
        return usb_device_write(output);
    }
    return false;
}

static bool console_parameter_index(const char *name, size_t *index_out) {
    if (name == NULL || index_out == NULL) {
        return false;
    }
    for (size_t index = 0U; index < app_config_parameter_count(); index++) {
        app_parameter_info_t info = {0};

        if (app_config_parameter_info(index, &info) &&
            strcasecmp(name, info.name) == 0) {
            *index_out = index;
            return true;
        }
    }
    return false;
}

static void console_print_parameter(const app_config_t *config, size_t index) {
    app_parameter_info_t info = {0};
    char value[48] = {0};

    if (app_config_parameter_info(index, &info) &&
        app_config_format_value(config, index, value, sizeof(value))) {
        console_writef("%s=%s\r\n", info.name, value);
    }
}

static bool console_parse_float(const char *text, float *value) {
    char *end = NULL;
    float parsed = 0.0f;

    if (text == NULL || value == NULL || text[0] == '\0') {
        return false;
    }
    errno = 0;
    parsed = strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed)) {
        return false;
    }
    *value = parsed;
    return true;
}

static bool console_write_monitor_line(void) {
    vario_result_t vario = {0};
    imu_diagnostics_t imu = {0};
    system_snapshot_t system = {0};
    ble_vario_lk8ex1_fields_t ble_fields = {0};
    int64_t now_us = esp_timer_get_time();

    if (!app_resources_copy_vario(&vario) ||
        !app_resources_copy_imu_diagnostics(&imu) ||
        !app_resources_copy_system(&system)) {
        return false;
    }
    (void) app_resources_apply_debug_vario(&vario, now_us);
    if (!ble_vario_format_lk8ex1_fields(&vario, &system, &ble_fields)) {
        return false;
    }

    return console_writef(
        "BARO seq=%" PRIu32 " timestamp_us=%" PRId64
        " online=%d pressure_valid=%d raw_temp=%" PRId32
        " raw_pressure=%" PRIu32 " temp_c=%.2f pressure_pa=%.2f"
        " altitude_m=%.2f climb_mps=%.3f climb_valid=%d"
        " estimate_valid=%d i2c_errors=%" PRIu32
        " overruns=%" PRIu32 " ble_pressure_pa=%s"
        " ble_altitude_m=%s ble_vario_cm_s=%s"
        " ble_temperature_c=%s ble_battery=%s"
        " ble_available=%d ble_notify=%d"
        " imu_online=%d imu_calibrated=%d imu_attitude_valid=%d"
        " imu_stale=%d q_w=%.5f q_x=%.5f q_y=%.5f q_z=%.5f"
        " roll_deg=%.2f pitch_deg=%.2f yaw_deg=%.2f"
        " vertical_accel_mps2=%.3f vertical_accel_valid=%d"
        " fusion_active=%d imu_samples=%" PRIu32
        " imu_missed=%" PRIu32 " stream_drops=%" PRIu32 "\r\n",
        vario.sequence, vario.timestamp_us, vario.bmp581_online,
        vario.pressure_valid, vario.raw_temperature, vario.raw_pressure,
        (double) vario.temperature_c_x100 / 100.0,
        (double) vario.pressure_pa_x100 / 100.0,
        (double) vario.altitude_m, (double) vario.climb_rate_mps,
        vario.climb_rate_valid, vario.estimate_valid,
        vario.i2c_error_count, vario.bmp_period_overrun_count,
        ble_fields.raw_pressure, ble_fields.altitude, ble_fields.vario,
        ble_fields.temperature, ble_fields.battery,
        ble_fields.sentence_available, ble_vario_can_notify(),
        imu.online, imu.calibrated, imu.attitude_valid, imu.stale,
        (double) imu.quaternion[0], (double) imu.quaternion[1],
        (double) imu.quaternion[2], (double) imu.quaternion[3],
        (double) imu.roll_deg, (double) imu.pitch_deg,
        (double) imu.yaw_deg, (double) vario.vertical_accel_mps2,
        vario.vertical_accel_valid, vario.imu_fusion_active,
        imu.sample_count, vario.missed_imu_sample_count,
        serial_monitor_drop_count);
}

static void console_diag_status(void) {
    vario_result_t vario = {0};
    imu_diagnostics_t imu = {0};
    system_snapshot_t system = {0};
    usb_device_diagnostics_t usb = {0};
    firmware_update_diagnostics_t update = {0};
    ble_vario_diagnostics_t ble = {0};
    app_power_diagnostics_t power = {0};
    int64_t now_us = esp_timer_get_time();

    (void) app_resources_copy_vario(&vario);
    (void) app_resources_apply_debug_vario(&vario, now_us);
    (void) app_resources_copy_imu_diagnostics(&imu);
    (void) app_resources_copy_system(&system);
    usb_device_get_diagnostics(&usb);
    firmware_update_get_diagnostics(&update);
    ble_vario_get_diagnostics(&ble);
    app_power_get_diagnostics(&power);

    console_writef(
        "VARIO bmp=%d pressure_valid=%d climb_valid=%d fusion=%d "
        "vertical_accel_valid=%d debug=%d pressure_pa=%.2f "
        "raw_temp=%" PRId32 " raw_pressure=%" PRIu32
        " altitude_m=%.2f climb_mps=%.2f vertical_accel_mps2=%.3f "
        "i2c_errors=%" PRIu32 " bmp_overruns=%" PRIu32
        " imu_missed=%" PRIu32 "\r\n",
        vario.bmp581_online, vario.pressure_valid, vario.climb_rate_valid,
        vario.imu_fusion_active, vario.vertical_accel_valid,
        vario.debug_input_active,
        (double) vario.pressure_pa_x100 / 100.0,
        vario.raw_temperature, vario.raw_pressure,
        (double) vario.altitude_m, (double) vario.climb_rate_mps,
        (double) vario.vertical_accel_mps2, vario.i2c_error_count,
        vario.bmp_period_overrun_count, vario.missed_imu_sample_count);
    console_writef(
        "IMU enabled=%d online=%d configured=%d calibrated=%d "
        "attitude=%d stale=%d fusion=%d target_address=0x%02x "
        "address=0x%02x who_am_i=0x%02x status=0x%02x "
        "retries=%" PRIu32 " samples=%" PRIu32 " calibration=%" PRIu32
        " missed=%" PRIu32 " errors=%" PRIu32 " accel_norm_g=%.3f "
        "gyro_bias_radps=%.5f,%.5f,%.5f "
        "q=%.5f,%.5f,%.5f,%.5f roll_deg=%.2f pitch_deg=%.2f "
        "yaw_deg=%.2f last_error=%s(%" PRId32 ")\r\n",
        imu.enabled, imu.online, imu.configured, imu.calibrated,
        imu.attitude_valid, imu.stale, imu.fusion_active,
        ICM42688_HXY_I2C_ADDRESS, imu.address, imu.who_am_i,
        imu.data_status, imu.retry_count, imu.sample_count,
        imu.calibration_sample_count, imu.missed_interrupt_count,
        imu.consecutive_error_count, (double) imu.accel_norm_g,
        (double) imu.gyro_bias_radps[0],
        (double) imu.gyro_bias_radps[1],
        (double) imu.gyro_bias_radps[2],
        (double) imu.quaternion[0], (double) imu.quaternion[1],
        (double) imu.quaternion[2], (double) imu.quaternion[3],
        (double) imu.roll_deg, (double) imu.pitch_deg,
        (double) imu.yaw_deg,
        esp_err_to_name((esp_err_t) imu.last_error), imu.last_error);
    console_writef(
        "SYSTEM battery_valid=%d battery_v=%.2f battery_raw=%" PRId32
        " battery_adc_mv=%" PRId32 " battery_samples=%" PRIu32
        " battery_errors=%" PRIu32 " battery_saturations=%" PRIu32
        " ext_power=%d sw=%d,%d,%d sw1_hold_ms=%" PRIu32
        " volume_override=%d volume_level=%d sink_override=%d"
        " sink_enabled=%d power_off=%d\r\n",
        system.battery_valid, (double) system.battery_voltage_v,
        system.battery_raw, system.battery_adc_mv,
        system.battery_sample_count, system.battery_error_count,
        system.battery_saturation_count, system.external_power_present,
        system.sw1_pressed, system.sw2_pressed, system.sw3_pressed,
        system.sw1_hold_ms, system.volume_override_active,
        (int) system.volume_level, system.sink_override_active,
        system.sink_enabled_override, system.power_off_requested);
    console_writef(
        "USB tinyusb=%d cdc=%d msc_driver=%d msc_media=%d"
        " attached=%d dtr=%d vbus=%d storage=%d owner=%s load=%d"
        " config_source=%s config_validation=%s config_version=%" PRId32
        " config_key=%s config_io_error=%" PRId32 " "
        "storage_error=%s last_save=%s attach_count=%" PRIu32
        " detach_count=%" PRIu32 " mount_errors=%" PRIu32
        " format_required=%" PRIu32 " rx_errors=%" PRIu32
        " tx_errors=%" PRIu32
        " stream_drops=%" PRIu32 "\r\n",
        usb.driver_ready, usb.cdc_ready, usb.msc_driver_ready,
        usb.msc_media_ready, usb.device_attached, usb.cdc_connected,
        usb.vbus_present, usb.storage_ready,
        usb_device_storage_owner_name(usb.storage_owner),
        (int) usb.load_result,
        config_storage_source_name(usb.config.source),
        config_storage_validation_name(usb.config.validation),
        usb.config.format_version,
        usb.config.key[0] == '\0' ? "-" : usb.config.key,
        usb.config.io_error, esp_err_to_name(usb.last_storage_error),
        esp_err_to_name(usb.last_save_result),
        usb.attach_count, usb.detach_count, usb.mount_failure_count,
        usb.format_required_count, usb.rx_error_count, usb.tx_error_count,
        serial_monitor_drop_count);
    console_writef(
        "UPDATE state=%s error=%s size=%" PRIu32
        " written=%" PRIu32 " confirm=%d workers=%d target=%s"
        " version=%s fingerprint=%s\r\n",
        firmware_update_state_name(update.state),
        esp_err_to_name(update.last_error), update.image_size_bytes,
        update.bytes_written, update.confirmation_required,
        update.required_workers_started,
        update.target_partition[0] == '\0' ? "-" : update.target_partition,
        update.image_version[0] == '\0' ? "-" : update.image_version,
        update.image_fingerprint[0] == '\0'
            ? "-"
            : update.image_fingerprint);
    console_writef(
        "BLE connected=%d subscribed=%d notify=%d active=%d"
        " sent=%" PRIu32 " dropped=%" PRIu32
        " last_success_us=%" PRId64 " last_error=%" PRId32 "\r\n",
        ble.connected, ble.subscribed, ble_vario_can_notify(),
        ble_vario_notify_active(), ble.sentence_count,
        ble.dropped_sentence_count, ble.last_notify_success_us,
        ble.last_notify_error);
    console_writef(
        "POWER cpu=%" PRIu32 "MHz sensor_lock=%d sleep_lock=%d "
        "sleep_count=%" PRIu32 " freq_changes=%" PRIu32
        " lock_errors=%" PRIu32 "\r\n",
        power.current_cpu_frequency_mhz, power.sensor_cpu_lock_held,
        power.light_sleep_lock_held, power.light_sleep_entry_count,
        power.observed_frequency_switch_count, power.lock_error_count);
    console_writef(
        "STACK words sensor=%u audio=%u system=%u console=%u ble=%u\r\n",
        (unsigned int) uxTaskGetStackHighWaterMark(sensor_task_handle),
        (unsigned int) uxTaskGetStackHighWaterMark(audio_task_handle),
        (unsigned int) uxTaskGetStackHighWaterMark(system_task_handle),
        (unsigned int) uxTaskGetStackHighWaterMark(console_task_handle),
        (unsigned int) uxTaskGetStackHighWaterMark(ble_tx_task_handle));
    console_writef("OK\r\n");
}

static void console_handle_parameter(char *tokens[], size_t token_count) {
    app_config_t config = {0};

    if (token_count < 2U || !app_resources_copy_config(&config)) {
        console_writef("ERR PARAM\r\n");
        return;
    }
    if (strcasecmp(tokens[1], "LIST") == 0 && token_count == 2U) {
        for (size_t index = 0U; index < app_config_parameter_count(); index++) {
            console_print_parameter(&config, index);
        }
        console_writef("OK\r\n");
        return;
    }
    if (strcasecmp(tokens[1], "GET") == 0 && token_count == 3U) {
        size_t index = 0U;

        if (!console_parameter_index(tokens[2], &index)) {
            console_writef("ERR UNKNOWN_PARAMETER\r\n");
            return;
        }
        console_print_parameter(&config, index);
        console_writef("OK\r\n");
        return;
    }
    if (strcasecmp(tokens[1], "SET") == 0 && token_count == 4U) {
        if (!app_config_set_text(&config, tokens[2], tokens[3]) ||
            !app_resources_publish_config(&config)) {
            console_writef("ERR INVALID_VALUE\r\n");
            return;
        }
        console_writef("OK\r\n");
        return;
    }
    if (strcasecmp(tokens[1], "RESET") == 0 && token_count == 3U) {
        if (!app_config_reset(&config, tokens[2]) ||
            !app_resources_publish_config(&config)) {
            console_writef("ERR INVALID_RESET\r\n");
            return;
        }
        console_writef("OK\r\n");
        return;
    }
    if (strcasecmp(tokens[1], "SAVE") == 0 && token_count == 2U) {
        esp_err_t ret = ESP_OK;

        ret = usb_device_save_config(&config);
        if (ret == ESP_OK) {
            console_writef("OK\r\n");
        } else if (ret == ESP_ERR_INVALID_STATE) {
            console_writef("ERR SAVE BUSY\r\n");
        } else {
            console_writef("ERR SAVE %s\r\n", esp_err_to_name(ret));
        }
        return;
    }
    console_writef("ERR PARAM_COMMAND\r\n");
}

static void console_handle_debug(char *tokens[], size_t token_count) {
    if (token_count == 2U && strcasecmp(tokens[1], "CLEAR") == 0) {
        app_resources_clear_debug_vario();
        console_writef("OK\r\n");
        return;
    }
    if ((token_count == 3U || token_count == 4U) &&
        strcasecmp(tokens[1], "VARIO") == 0) {
        float climb_rate_mps = 0.0f;
        float pressure_pa = 0.0f;
        bool pressure_valid = token_count == 4U;
        int32_t pressure_pa_x100 = 0;

        if (!console_parse_float(tokens[2], &climb_rate_mps)) {
            console_writef("ERR INVALID_VARIO\r\n");
            return;
        }
        if (pressure_valid) {
            if (!console_parse_float(tokens[3], &pressure_pa) ||
                pressure_pa < 30000.0f || pressure_pa > 125000.0f) {
                console_writef("ERR INVALID_PRESSURE\r\n");
                return;
            }
            pressure_pa_x100 = (int32_t) lroundf(pressure_pa * 100.0f);
        }
        if (!app_resources_set_debug_vario(
                climb_rate_mps, pressure_valid, pressure_pa_x100)) {
            console_writef("ERR INVALID_VARIO\r\n");
            return;
        }
        console_writef("OK\r\n");
        return;
    }
    console_writef("ERR DEBUG_COMMAND\r\n");
}

static void console_process_line(char *line) {
    char *tokens[5] = {0};
    char *save_pointer = NULL;
    char *token = NULL;
    size_t token_count = 0U;

    token = strtok_r(line, " \t", &save_pointer);
    while (token != NULL && token_count < sizeof(tokens) / sizeof(tokens[0])) {
        tokens[token_count++] = token;
        token = strtok_r(NULL, " \t", &save_pointer);
    }
    if (token != NULL) {
        console_writef("ERR TOO_MANY_ARGUMENTS\r\n");
        return;
    }
    if (token_count == 0U) {
        return;
    }
    if (strcasecmp(tokens[0], "PARAM") == 0) {
        console_handle_parameter(tokens, token_count);
    } else if (strcasecmp(tokens[0], "DEBUG") == 0) {
        console_handle_debug(tokens, token_count);
    } else if (token_count == 2U && strcasecmp(tokens[0], "DIAG") == 0 &&
               strcasecmp(tokens[1], "STATUS") == 0) {
        console_diag_status();
    } else {
        console_writef("ERR UNKNOWN_COMMAND\r\n");
    }
}

static void console_task(void *context) {
    QueueHandle_t queue = app_resources_diagnostic_queue();
    diagnostic_event_t event = {0};
    uint8_t input[64] = {0};
    char line[129] = {0};
    size_t input_length = 0U;
    size_t line_length = 0U;
    bool line_overflow = false;
    bool previous_was_cr = false;
    int64_t next_monitor_us = 0;

    (void) context;
    ESP_LOGI(TAG, "console_task started on core %d", xPortGetCoreID());
    next_monitor_us = esp_timer_get_time() + SERIAL_MONITOR_PERIOD_US;

    for (;;) {
        int64_t now_us = esp_timer_get_time();

        if (app_stop_requested()) {
            acknowledge_and_delete(APP_EVENT_CONSOLE_ACK);
        }

        if (now_us >= next_monitor_us) {
            int64_t periods_elapsed =
                (now_us - next_monitor_us) / SERIAL_MONITOR_PERIOD_US;

            if (periods_elapsed > 0) {
                uint32_t skipped = UINT32_MAX;

                if (periods_elapsed <= (int64_t) UINT32_MAX) {
                    skipped = (uint32_t) periods_elapsed;
                }
                add_saturating_u32(&serial_monitor_drop_count, skipped);
            }
            next_monitor_us +=
                (periods_elapsed + 1) * SERIAL_MONITOR_PERIOD_US;
            if (usb_device_cdc_connected() &&
                !console_write_monitor_line()) {
                add_saturating_u32(&serial_monitor_drop_count, 1U);
            }
        }

        if (queue != NULL && xQueueReceive(queue, &event, 0U) == pdTRUE) {
            console_writef("EVENT type=%d detail=%" PRId32 "\r\n",
                           (int) event.type, event.detail);
        }

        while (usb_device_read(input, sizeof(input), &input_length)) {
            for (size_t index = 0U; index < input_length; index++) {
                char character = (char) input[index];
                bool terminator = character == '\r' || character == '\n';

                if (character == '\n' && previous_was_cr) {
                    previous_was_cr = false;
                    continue;
                }
                previous_was_cr = character == '\r';
                if (terminator) {
                    if (line_overflow) {
                        console_writef("ERR LINE_TOO_LONG\r\n");
                    } else if (line_length > 0U) {
                        line[line_length] = '\0';
                        console_process_line(line);
                    }
                    line_length = 0U;
                    line_overflow = false;
                } else if (!line_overflow) {
                    if (line_length < sizeof(line) - 1U) {
                        line[line_length++] = character;
                    } else {
                        line_overflow = true;
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

static void ble_tx_task(void *context) {
    vario_result_t vario = {0};
    system_snapshot_t system = {0};

    (void) context;
    ESP_LOGI(TAG, "ble_tx_task started on core %d", xPortGetCoreID());

    for (;;) {
        if (app_stop_requested()) {
            esp_err_t ret = ble_vario_stop();

            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "BLE shutdown incomplete: %s", esp_err_to_name(ret));
                block_safe_stop_light_sleep();
            }
            acknowledge_and_delete(APP_EVENT_BLE_TX_ACK);
        }

        if (app_resources_copy_vario(&vario) &&
            app_resources_copy_system(&system) &&
            ble_vario_can_notify()) {
            (void) app_resources_apply_debug_vario(
                &vario, esp_timer_get_time());
            esp_err_t ret = ble_vario_notify_lk8ex1(&vario, &system);

            if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND &&
                ret != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "LK8EX1 sentence dropped: %s",
                         esp_err_to_name(ret));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(BLE_TX_PERIOD_MS));
    }
}

static esp_err_t create_task(TaskFunction_t function, const char *name, uint32_t stack_size_bytes,
                             UBaseType_t priority, BaseType_t core_id, TaskHandle_t *task_handle,
                             EventBits_t acknowledgement_bit) {
    BaseType_t result = pdFAIL;

    result = xTaskCreatePinnedToCore(function, name, stack_size_bytes, NULL, priority, task_handle,
                                     core_id);
    if (result != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    active_ack_mask |= acknowledgement_bit;
    return ESP_OK;
}

esp_err_t app_tasks_start(void) {
    EventGroupHandle_t event_group = app_resources_event_group();
    esp_err_t ret = ESP_OK;

    active_ack_mask = 0U;

    ret = create_task(audio_task, "audio_task", AUDIO_TASK_STACK_BYTES, AUDIO_TASK_PRIORITY,
                      HIGH_RATE_TASK_CORE, &audio_task_handle, APP_EVENT_AUDIO_ACK);
    if (ret == ESP_OK) {
        ret = create_task(system_task, "system_task", SYSTEM_TASK_STACK_BYTES, SYSTEM_TASK_PRIORITY,
                          COMMUNICATION_TASK_CORE, &system_task_handle, APP_EVENT_SYSTEM_ACK);
    }
    if (ret == ESP_OK) {
        ret = create_task(sensor_task, "sensor_task", SENSOR_TASK_STACK_BYTES, SENSOR_TASK_PRIORITY,
                          HIGH_RATE_TASK_CORE, &sensor_task_handle, APP_EVENT_SENSOR_ACK);
    }
    if (ret == ESP_OK) {
        ret = create_task(console_task, "console_task", CONSOLE_TASK_STACK_BYTES,
                          CONSOLE_TASK_PRIORITY, COMMUNICATION_TASK_CORE, &console_task_handle,
                          APP_EVENT_CONSOLE_ACK);
    }
    if (ret == ESP_OK) {
        ret = create_task(ble_tx_task, "ble_tx_task", BLE_TX_TASK_STACK_BYTES, BLE_TX_TASK_PRIORITY,
                          COMMUNICATION_TASK_CORE, &ble_tx_task_handle, APP_EVENT_BLE_TX_ACK);
    }

    if (ret != ESP_OK) {
        board_set_safe_indicators();
        if (event_group != NULL) {
            (void) xEventGroupClearBits(event_group, APP_EVENT_FATAL_BMP581);
            (void) xEventGroupSetBits(event_group, APP_EVENT_FATAL_STATE);
        }
        ESP_LOGE(TAG, "task creation failed; entering fatal state");
        return ret;
    }

    if (event_group == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    firmware_update_mark_workers_started();
    EventBits_t startup_bits =
        xEventGroupWaitBits(event_group, APP_EVENT_BMP581_STARTUP_COMPLETE, pdFALSE, pdTRUE,
                            portMAX_DELAY);
    if ((startup_bits & APP_EVENT_FATAL_BMP581) != 0U) {
        return ESP_ERR_NOT_FOUND;
    }

    (void) xEventGroupClearBits(
        event_group,
        APP_EVENT_STARTUP_SOUND_REQUEST | APP_EVENT_STARTUP_SOUND_DONE |
            APP_EVENT_STARTUP_SOUND_ABORT);
    (void) xEventGroupSetBits(event_group,
                              APP_EVENT_STARTUP_SOUND_REQUEST);
    EventBits_t sound_bits = xEventGroupWaitBits(
        event_group,
        APP_EVENT_STARTUP_SOUND_DONE | APP_EVENT_STOP_REQUEST |
            APP_EVENT_FATAL_STATE,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(STARTUP_SOUND_WAIT_MS));
    if ((sound_bits &
         (APP_EVENT_STOP_REQUEST | APP_EVENT_FATAL_STATE)) != 0U) {
        (void) xEventGroupSetBits(event_group,
                                  APP_EVENT_STARTUP_SOUND_ABORT);
        (void) xEventGroupClearBits(event_group,
                                    APP_EVENT_STARTUP_SOUND_REQUEST);
        return ESP_ERR_INVALID_STATE;
    }
    if ((sound_bits & APP_EVENT_STARTUP_SOUND_DONE) == 0U) {
        ESP_LOGW(TAG, "startup sound timed out; continuing startup");
        post_runtime_diagnostic(DIAGNOSTIC_EVENT_PERIPHERAL_FAILURE,
                                ESP_ERR_TIMEOUT);
        (void) xEventGroupSetBits(event_group,
                                  APP_EVENT_STARTUP_SOUND_ABORT);
        (void) xEventGroupClearBits(event_group,
                                    APP_EVENT_STARTUP_SOUND_REQUEST);
    } else {
        (void) xEventGroupClearBits(event_group,
                                    APP_EVENT_STARTUP_SOUND_DONE);
    }

    return ESP_OK;
}

EventBits_t app_tasks_active_ack_mask(void) {
    return active_ack_mask;
}

bool app_tasks_system_started(void) {
    return system_task_handle != NULL;
}

bool app_tasks_required_workers_started(void) {
    return audio_task_handle != NULL && system_task_handle != NULL &&
           sensor_task_handle != NULL && console_task_handle != NULL &&
           ble_tx_task_handle != NULL;
}

void app_tasks_run_fatal_fallback(void) {
    button_debounce_t sw1 = {0};
    uint32_t hold_time_ms = 0U;
    uint32_t led_elapsed_ms = 0U;
    bool was_released = false;

    sw1.candidate_pressed = board_is_sw1_pressed();
    sw1.stable_pressed = sw1.candidate_pressed;

    for (;;) {
        bool pressed = debounce_button(&sw1, board_is_sw1_pressed());

        if (sw1.stable_valid && !pressed) {
            was_released = true;
            hold_time_ms = 0U;
        } else if (sw1.stable_valid && was_released) {
            hold_time_ms += SYSTEM_SAMPLE_PERIOD_MS;
        } else {
            /* Ignore the switch that was already held during startup. */
        }

        set_fatal_leds(led_elapsed_ms, false);
        led_elapsed_ms += SYSTEM_SAMPLE_PERIOD_MS;

        if (was_released && hold_time_ms >= POWER_OFF_HOLD_MS) {
            board_set_safe_indicators();
            (void) board_set_power_hold(false);
            run_safe_stop_loop(NULL, 0U, false);
        }

        vTaskDelay(pdMS_TO_TICKS(SYSTEM_SAMPLE_PERIOD_MS));
    }
}
