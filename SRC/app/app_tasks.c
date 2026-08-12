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
#include "domain/auto_power_off.h"
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
#include "platform/imu_calibration_storage.h"
#include "platform/sensor_bus.h"
#include "platform/system_io.h"
#include "platform/switch_preferences.h"
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
#define POWER_OFF_HOLD_MS UINT32_C(2000)
#define IMU_CALIBRATION_SKIP_HOLD_MS UINT32_C(3000)
#define SHUTDOWN_DEADLINE_MS UINT32_C(15000)
#define SAFE_STOP_PERIOD_MS UINT32_C(10)
#define SYSTEM_SOUND_LOW_HZ UINT32_C(700)
#define SYSTEM_SOUND_LOW_MS UINT32_C(180)
#define SYSTEM_SOUND_SILENCE_MS UINT32_C(80)
#define SYSTEM_SOUND_HIGH_HZ UINT32_C(1200)
#define SYSTEM_SOUND_HIGH_MS UINT32_C(120)
#define BUTTON_SOUND_HZ UINT32_C(1000)
#define BUTTON_SOUND_MS UINT32_C(80)
#define BUTTON_SOUND_SILENCE_MS UINT32_C(80)
#define SHUTDOWN_SOUND_TOTAL_MS (SYSTEM_SOUND_HIGH_MS + SYSTEM_SOUND_SILENCE_MS + SYSTEM_SOUND_LOW_MS)
#define SYSTEM_SOUND_DUTY_PERCENT UINT32_C(50)
#define FATAL_LED_PHASE_MS UINT32_C(500)
#define BMP_RECOVERY_LED_PHASE_MS UINT32_C(1000)
#define IMU_CALIBRATION_LED_CYCLE_MS UINT32_C(2000)
#define IMU_DEGRADED_LED_CYCLE_MS UINT32_C(1000)
#define LOW_BATTERY_LED_CYCLE_MS UINT32_C(1000)
#define LOW_BATTERY_THRESHOLD_V 3.4f

#define BMP581_SAMPLE_PERIOD_US INT64_C(10000)
#define SENSOR_STALE_TIMEOUT_US INT64_C(100000)
#define IMU_STALE_TIMEOUT_US INT64_C(100000)
#define SENSOR_IDLE_WAKE_US INT64_C(100000)
#define SENSOR_RETRY_INTERVAL_US ((int64_t) CONFIG_CBV_SENSOR_RETRY_INTERVAL_MS * INT64_C(1000))
#define IMU_CALIBRATION_SAVE_RETRY_US INT64_C(2000000)

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
    imu_accel_calibrator_t imu_accel_calibrator;
    imu_accel_calibration_t imu_accel_calibration;
    imu_accel_calibration_t pending_imu_accel_calibration;
    app_config_t imu_config;
    vario_estimator_t estimator;
    float estimator_reference_pressure_pa;
    bool imu_config_valid;
    bool imu_accel_calibration_save_pending;
    bool imu_accel_calibration_skipped;
    int64_t next_imu_accel_calibration_save_us;
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
static app_config_profiles_t system_profile_snapshot;
static app_config_profiles_t console_profile_snapshot;
static auto_power_off_state_t system_auto_power_off_state;
static app_config_t system_auto_power_off_config;
static vario_result_t system_auto_power_off_vario;
static uint32_t system_auto_power_off_config_revision = 0U;
static bool system_auto_power_off_config_revision_valid = false;
static switch_preferences_t initial_switch_preferences = {
    .volume_level = AUDIO_VOLUME_SMALL,
    .sink_enabled = true,
    .parameter_number = 1U,
};
static bool initial_switch_preferences_dirty = false;
static imu_accel_calibration_t initial_imu_accel_calibration;
static imu_calibration_storage_diagnostics_t
    initial_imu_accel_calibration_diagnostics = {
        .result = IMU_CALIBRATION_STORAGE_MISSING,
    };

void app_tasks_set_imu_accel_calibration(
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

void app_tasks_set_switch_preferences(
    const switch_preferences_t *preferences, bool dirty) {
    switch_preferences_set_defaults(&initial_switch_preferences);
    initial_switch_preferences_dirty = dirty;
    if (preferences != NULL &&
        preferences->volume_level >= AUDIO_VOLUME_SMALL &&
        preferences->volume_level <= AUDIO_VOLUME_MUTE &&
        preferences->parameter_number >= APP_CONFIG_PROFILE_MIN_NUMBER &&
        preferences->parameter_number <= APP_CONFIG_PROFILE_MAX_NUMBER) {
        initial_switch_preferences = *preferences;
    }
}

_Static_assert(configMAX_PRIORITIES >= 25, "SW_spec.md requires at least 25 priorities");

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

static const system_sound_step_t button_sound_steps[] = {
    {BUTTON_SOUND_HZ, BUTTON_SOUND_MS},
};

static const system_sound_step_t sink_enabled_sound_steps[] = {
    {SYSTEM_SOUND_HIGH_HZ, SYSTEM_SOUND_HIGH_MS},
};

static const system_sound_step_t sink_disabled_sound_steps[] = {
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

static bool led_first_phase(uint32_t elapsed_ms, uint32_t phase_ms) {
    uint32_t cycle_ms = phase_ms * UINT32_C(2);

    return elapsed_ms % cycle_ms < phase_ms;
}

static uint32_t led_firefly_brightness_percent(uint32_t elapsed_ms,
                                               uint32_t cycle_ms) {
    uint32_t position_ms = elapsed_ms % cycle_ms;
    uint32_t half_cycle_ms = cycle_ms / 2U;

    if (position_ms < half_cycle_ms) {
        return 100U - position_ms * 100U / half_cycle_ms;
    }
    return (position_ms - half_cycle_ms) * 100U / half_cycle_ms;
}

static uint32_t low_battery_led_brightness_percent(uint32_t elapsed_ms) {
    uint32_t firefly_brightness_percent =
        led_firefly_brightness_percent(elapsed_ms,
                                       LOW_BATTERY_LED_CYCLE_MS);

    return 50U + firefly_brightness_percent / 2U;
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

static uint32_t power_on_green_brightness_percent(uint32_t hold_ms) {
    if (hold_ms >= POWER_ON_HOLD_MS) {
        return 100U;
    }
    return hold_ms * 100U / POWER_ON_HOLD_MS;
}

static void set_lifecycle_leds(uint32_t elapsed_ms, uint32_t sw1_hold_ms,
                               bool external_power_present,
                               bool battery_valid, float battery_voltage_v) {
    EventBits_t bits = app_event_bits();
    vario_result_t result = {0};
    uint32_t green_brightness_percent = 100U;
    bool yellow = false;
    bool green_selected = false;
    bool yellow_selected = false;
    bool recovering = (bits & APP_EVENT_BMP581_RECOVERING) != 0U;

    if ((bits & APP_EVENT_FATAL_STATE) != 0U) {
        set_fatal_leds(elapsed_ms, (bits & APP_EVENT_FATAL_BMP581) != 0U);
        return;
    }
    if ((bits & APP_EVENT_BMP581_STARTUP_COMPLETE) == 0U) {
        board_set_status_leds_brightness(100U, false);
        return;
    }

    if (app_resources_copy_vario(&result)) {
        bool invalid_or_stale =
            !result.pressure_valid ||
            (!result.climb_rate_valid && !result.estimator_warming_up &&
             !recovering);

        if (invalid_or_stale) {
            green_brightness_percent = 0U;
            yellow = led_first_phase(elapsed_ms, FATAL_LED_PHASE_MS);
            green_selected = true;
            yellow_selected = true;
        }
    }
    if (!green_selected && recovering) {
        green_brightness_percent =
            led_first_phase(elapsed_ms, BMP_RECOVERY_LED_PHASE_MS)
                ? 100U
                : 0U;
        green_selected = true;
    }
    if (!green_selected && (bits & APP_EVENT_IMU_CALIBRATING) != 0U) {
        green_brightness_percent = led_firefly_brightness_percent(
            elapsed_ms, IMU_CALIBRATION_LED_CYCLE_MS);
        green_selected = true;
    }
    if (!green_selected && result.estimator_warming_up) {
        green_brightness_percent =
            led_first_phase(elapsed_ms, BMP_RECOVERY_LED_PHASE_MS)
                ? 100U
                : 0U;
        green_selected = true;
    }
    if (!green_selected && (bits & APP_EVENT_IMU_DEGRADED) != 0U) {
        green_brightness_percent = led_firefly_brightness_percent(
            elapsed_ms, IMU_DEGRADED_LED_CYCLE_MS);
        green_selected = true;
    }
    if (!green_selected && !external_power_present && battery_valid &&
        isfinite(battery_voltage_v) &&
        battery_voltage_v <= LOW_BATTERY_THRESHOLD_V) {
        green_brightness_percent =
            low_battery_led_brightness_percent(elapsed_ms);
        green_selected = true;
    }
    if (!yellow_selected && ble_vario_notify_active()) {
        yellow = elapsed_ms % UINT32_C(1000) < UINT32_C(100);
        yellow_selected = true;
    }
    if (!yellow_selected) {
        yellow = false;
    }
    if (sw1_hold_ms > 0U) {
        board_set_status_leds_brightness(
            sw1_green_brightness_percent(sw1_hold_ms), yellow);
    } else {
        board_set_status_leds_brightness(green_brightness_percent,
                                         yellow);
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

static audio_volume_level_t selected_volume_level(
    const system_snapshot_t *system) {
    app_config_t config = {0};

    if (system != NULL && system->volume_override_active) {
        switch (system->volume_level) {
        case AUDIO_VOLUME_SMALL:
        case AUDIO_VOLUME_MEDIUM:
        case AUDIO_VOLUME_LARGE:
        case AUDIO_VOLUME_MUTE:
            return system->volume_level;
        default:
            return AUDIO_VOLUME_MUTE;
        }
    }
    if (!app_resources_copy_config(&config)) {
        app_config_set_defaults(&config);
    }
    return config_volume_level(&config);
}

static uint32_t volume_amplifier_mode(audio_volume_level_t volume_level) {
    switch (volume_level) {
    case AUDIO_VOLUME_SMALL:
    case AUDIO_VOLUME_MEDIUM:
    case AUDIO_VOLUME_LARGE:
        return (uint32_t) volume_level + UINT32_C(1);
    case AUDIO_VOLUME_MUTE:
    default:
        return 0U;
    }
}

static void request_button_sound(audio_volume_level_t volume_level,
                                 uint8_t repeat_count) {
    QueueHandle_t queue = app_resources_button_sound_queue();
    audio_notification_request_t request = {
        .kind = AUDIO_NOTIFICATION_BUTTON,
        .volume_level = volume_level,
        .repeat_count = repeat_count,
    };

    if (volume_level == AUDIO_VOLUME_MUTE || repeat_count == 0U ||
        repeat_count > APP_CONFIG_PROFILE_MAX_NUMBER) {
        return;
    }
    if (queue == NULL) {
        ESP_LOGW(TAG, "button sound request queue unavailable");
        return;
    }
    (void) xQueueOverwrite(queue, &request);
}

static void request_sink_status_sound(audio_volume_level_t volume_level,
                                      bool sink_enabled) {
    QueueHandle_t queue = app_resources_button_sound_queue();
    audio_notification_request_t request = {
        .kind = sink_enabled ? AUDIO_NOTIFICATION_SINK_ENABLED
                             : AUDIO_NOTIFICATION_SINK_DISABLED,
        .volume_level = volume_level,
        .repeat_count = 1U,
    };

    if (volume_level == AUDIO_VOLUME_MUTE) {
        return;
    }
    if (queue == NULL) {
        ESP_LOGW(TAG, "button sound request queue unavailable");
        return;
    }
    (void) xQueueOverwrite(queue, &request);
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

static void toggle_sink_override(system_snapshot_t *snapshot) {
    app_config_t config = {0};
    bool current;

    if (snapshot == NULL) {
        return;
    }
    current = snapshot->sink_enabled_override;
    if (!snapshot->sink_override_active) {
        if (!app_resources_copy_config(&config)) {
            app_config_set_defaults(&config);
        }
        current = config.sink_enabled;
    }
    snapshot->sink_enabled_override = !current;
    snapshot->sink_override_active = true;
}

static void update_switch_preferences_dirty(
    system_snapshot_t *snapshot,
    const switch_preferences_t *baseline, bool force_dirty) {
    if (snapshot == NULL || baseline == NULL) {
        return;
    }
    snapshot->switch_preferences_dirty =
        force_dirty ||
        snapshot->volume_level != baseline->volume_level ||
        snapshot->sink_enabled_override != baseline->sink_enabled ||
        snapshot->parameter_number != baseline->parameter_number;
}

static bool select_next_parameter_set(system_snapshot_t *snapshot) {
    if (snapshot == NULL ||
        !app_resources_select_next_config(&snapshot->parameter_number,
                                          &snapshot->parameter_set_count)) {
        ESP_LOGW(TAG, "parameter set switch failed");
        return false;
    }
    request_button_sound(selected_volume_level(snapshot),
                         snapshot->parameter_number);
    return true;
}

static bool debounce_button(button_debounce_t *state, bool pressed) {
    if (state == NULL) {
        return false;
    }

    if (pressed != state->candidate_pressed) {
        state->candidate_pressed = pressed;
        /* The edge sample starts the interval; it does not consume 10 ms. */
        state->stable_time_ms = 0U;
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
        state->imu_accel_calibration_skipped ||
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
    if (!imu_accel_calibration_validate(&state->imu_accel_calibration)) {
        imu_accel_calibrator_reset(&state->imu_accel_calibrator);
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
    if (ret == ESP_OK) {
        state->next_bmp_retry_us = now_us;
        state->next_imu_retry_us = now_us;
    } else {
        ESP_LOGW(TAG, "I2C recovery failed: %s", esp_err_to_name(ret));
        state->next_bmp_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
        state->next_imu_retry_us = now_us + SENSOR_RETRY_INTERVAL_US;
    }
    (void) app_resources_publish_imu_diagnostics(&state->imu_diagnostics);
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

    if (!state->bus_timeout_detected) {
        sensor_try_initialize_imu(state, bus_handle, now_us);
    }

    return changed;
}

static bool imu_configs_match(const app_config_t *left,
                              const app_config_t *right) {
    if (left == NULL || right == NULL) {
        return false;
    }
    return left->imu_gyro_calibration_samples ==
               right->imu_gyro_calibration_samples &&
           left->imu_mahony_kp == right->imu_mahony_kp &&
           left->imu_mahony_ki == right->imu_mahony_ki;
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

static void sensor_sync_accel_calibration_diagnostics(
    sensor_task_state_t *state) {
    const imu_accel_calibration_t *calibration = NULL;

    if (state == NULL) {
        return;
    }
    if (imu_accel_calibration_validate(&state->imu_accel_calibration)) {
        calibration = &state->imu_accel_calibration;
    } else if (imu_accel_calibration_validate(
                   &state->pending_imu_accel_calibration)) {
        calibration = &state->pending_imu_accel_calibration;
    }
    state->imu_diagnostics.accel_calibrated =
        imu_accel_calibration_validate(&state->imu_accel_calibration);
    state->imu_diagnostics.accel_calibration_persisted =
        state->imu_diagnostics.accel_calibrated;
    state->imu_diagnostics.accel_calibration_save_pending =
        state->imu_accel_calibration_save_pending;
    state->imu_diagnostics.accel_calibration_skipped =
        state->imu_accel_calibration_skipped;
    state->imu_diagnostics.accel_calibration_sample_count =
        state->imu_accel_calibration_save_pending
            ? IMU_ACCEL_CALIBRATION_SAMPLE_COUNT
            : state->imu_accel_calibrator.sample_count;
    state->imu_diagnostics.accel_norm_g =
        state->imu_accel_calibrator.accel_norm_g;
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

    if (state == NULL || !state->imu_accel_calibration_save_pending ||
        now_us < state->next_imu_accel_calibration_save_us) {
        return false;
    }
    ret = usb_device_save_imu_calibration(
        &state->pending_imu_accel_calibration);
    state->imu_diagnostics.accel_calibration_storage_error =
        (int32_t) ret;
    if (ret == ESP_OK) {
        state->imu_accel_calibration =
            state->pending_imu_accel_calibration;
        memset(&state->pending_imu_accel_calibration, 0,
               sizeof(state->pending_imu_accel_calibration));
        state->imu_accel_calibration_save_pending = false;
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
        state->next_imu_accel_calibration_save_us =
            now_us + IMU_CALIBRATION_SAVE_RETRY_US;
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
    EventBits_t bits =
        event_group == NULL ? 0U : xEventGroupGetBits(event_group);

    if (state == NULL || state->imu_accel_calibration_skipped ||
        (bits & APP_EVENT_IMU_ACCEL_CALIBRATION_SKIP_REQUEST) == 0U ||
        imu_accel_calibration_validate(&state->imu_accel_calibration)) {
        return false;
    }

    if (state->imu_ready) {
        esp_err_t ret = icm42688_hxy_deinit();

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "IMU deinitialization after calibration skip failed: %s",
                     esp_err_to_name(ret));
        }
    }
    state->imu_ready = false;
    state->imu_interrupt_pending = false;
    state->imu_config_valid = false;
    state->imu_accel_calibration_skipped = true;
    state->imu_accel_calibration_save_pending = false;
    state->next_imu_accel_calibration_save_us = 0;
    memset(&state->pending_imu_accel_calibration, 0,
           sizeof(state->pending_imu_accel_calibration));
    imu_accel_calibrator_reset(&state->imu_accel_calibrator);
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
    sensor_sync_accel_calibration_diagnostics(state);
    (void) app_resources_publish_imu_diagnostics(
        &state->imu_diagnostics);
    set_imu_lifecycle_state(false, true);

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
        imu_accel_calibration_validate(&state->imu_accel_calibration)) {
        return false;
    }
    if (!state->imu_accel_calibration_save_pending &&
        imu_accel_calibrator_update(
            &state->imu_accel_calibrator, sensor_sample,
            board_imu_axis_map(),
            &state->pending_imu_accel_calibration)) {
        state->imu_accel_calibration_save_pending = true;
        state->next_imu_accel_calibration_save_us = now_us;
        ESP_LOGI(TAG,
                 "IMU accelerometer calibration captured; saving mc_data.json");
    }
    (void) sensor_try_save_accel_calibration(state, now_us);
    state->result.imu_calibrated = false;
    state->result.vertical_accel_valid = false;
    state->result.imu_fusion_active = false;
    state->imu_diagnostics.calibrated = false;
    state->imu_diagnostics.attitude_valid = false;
    state->imu_diagnostics.fusion_active = false;
    state->imu_diagnostics.vibration_rms_g =
        state->imu_accel_calibrator.vibration_rms_g;
    sensor_sync_accel_calibration_diagnostics(state);
    set_imu_lifecycle_state(true, false);
    return true;
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
    if (!imu_accel_calibration_validate(&state->imu_accel_calibration)) {
        if (state->imu_accel_calibration_skipped) {
            state->result.imu_calibrated = false;
            state->result.vertical_accel_valid = false;
            state->result.imu_fusion_active = false;
            state->imu_diagnostics.calibrated = false;
            state->imu_diagnostics.attitude_valid = false;
            state->imu_diagnostics.fusion_active = false;
            set_imu_lifecycle_state(false, true);
            (void) app_resources_publish_imu_diagnostics(
                &state->imu_diagnostics);
            return true;
        }
        (void) sensor_process_factory_accel_calibration(
            state, &sensor_sample, now_us);
        (void) app_resources_publish_imu_diagnostics(
            &state->imu_diagnostics);
        return true;
    }
    if (!imu_fusion_apply_calibration_and_axis_map(
            &sensor_sample, board_imu_axis_map(),
            &state->imu_accel_calibration,
            &board_sample) ||
        !imu_fusion_update(&state->imu_fusion, &board_sample, &config,
                           &fusion_output)) {
        sensor_restart_imu_fusion(state, &config);
        (void) app_resources_publish_imu_diagnostics(
            &state->imu_diagnostics);
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
        if (config.filter_mode == APP_FILTER_MODE_AUTO) {
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
    (void) app_resources_publish_imu_diagnostics(
        &state->imu_diagnostics);
    return true;
}

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
    if (state->imu_accel_calibration_save_pending &&
        state->next_imu_accel_calibration_save_us < wake_time_us) {
        wake_time_us = state->next_imu_accel_calibration_save_us;
    }
    if (state->imu_ready &&
        state->last_imu_valid_us + IMU_STALE_TIMEOUT_US < wake_time_us) {
        wake_time_us =
            state->last_imu_valid_us + IMU_STALE_TIMEOUT_US;
    } else if (!state->imu_ready &&
               state->next_imu_retry_us < wake_time_us) {
        wake_time_us = state->next_imu_retry_us;
    }

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

    changed |= sensor_handle_accel_calibration_skip(state);
    changed |= sensor_try_initialize_devices(state, now_us);
    changed |= sensor_process_imu(state, esp_timer_get_time());
    changed |= sensor_try_save_accel_calibration(
        state, esp_timer_get_time());
    changed |= sensor_process_bmp581(state, esp_timer_get_time());
    changed |= sensor_check_stale(state, esp_timer_get_time());
    changed |= sensor_recover_shared_bus(state, esp_timer_get_time());
    sensor_sync_estimator_diagnostics(state);

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
    state.imu_diagnostics.enabled = true;
    state.imu_diagnostics.last_error = (int32_t) ESP_ERR_NOT_FOUND;
    state.imu_accel_calibration = initial_imu_accel_calibration;
    state.imu_diagnostics.accel_calibration_storage_result =
        (int32_t) initial_imu_accel_calibration_diagnostics.result;
    state.imu_diagnostics.accel_calibration_storage_error =
        initial_imu_accel_calibration_diagnostics.io_error;
    imu_accel_calibrator_reset(&state.imu_accel_calibrator);
    sensor_sync_accel_calibration_diagnostics(&state);
    imu_fusion_reset(&state.imu_fusion);
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
    EventBits_t abort_mask, bool watchdog_registered,
    uint32_t amplifier_mode, esp_err_t *output_error) {
    system_sound_result_t result = SYSTEM_SOUND_COMPLETE;

    audio_output_shutdown();
    if (output_error != NULL) {
        *output_error = ESP_OK;
    }
    if (amplifier_mode == 0U) {
        return result;
    }
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
                amplifier_mode);

            if (ret != ESP_OK) {
                if (output_error != NULL) {
                    *output_error = ret;
                }
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

esp_err_t app_tasks_play_startup_sound(
    audio_volume_level_t volume_level) {
    esp_err_t output_error = ESP_OK;
    system_sound_result_t result = play_system_sound(
        startup_sound_steps,
        sizeof(startup_sound_steps) / sizeof(startup_sound_steps[0]),
        0U, false, volume_amplifier_mode(volume_level), &output_error);

    if (result == SYSTEM_SOUND_OUTPUT_ERROR) {
        return output_error == ESP_OK ? ESP_FAIL : output_error;
    }
    return result == SYSTEM_SOUND_COMPLETE ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static void play_latest_button_notification(
    QueueHandle_t queue, audio_notification_request_t request,
    bool watchdog_registered) {
    uint8_t completed = 0U;

    while (completed < request.repeat_count) {
        audio_notification_request_t newer = {0};
        const system_sound_step_t *steps = button_sound_steps;
        size_t step_count =
            sizeof(button_sound_steps) / sizeof(button_sound_steps[0]);

        if (request.kind == AUDIO_NOTIFICATION_SINK_ENABLED) {
            steps = sink_enabled_sound_steps;
            step_count = sizeof(sink_enabled_sound_steps) /
                         sizeof(sink_enabled_sound_steps[0]);
        } else if (request.kind == AUDIO_NOTIFICATION_SINK_DISABLED) {
            steps = sink_disabled_sound_steps;
            step_count = sizeof(sink_disabled_sound_steps) /
                         sizeof(sink_disabled_sound_steps[0]);
        }

        if (play_system_sound(
                steps, step_count,
                APP_EVENT_STOP_REQUEST | APP_EVENT_FATAL_STATE,
                watchdog_registered,
                volume_amplifier_mode(request.volume_level), NULL) !=
            SYSTEM_SOUND_COMPLETE) {
            break;
        }
        completed++;
        if (xQueueReceive(queue, &newer, 0U) == pdTRUE) {
            request = newer;
            completed = 0U;
            continue;
        }
        if (completed < request.repeat_count &&
            !system_sound_delay(BUTTON_SOUND_SILENCE_MS,
                                APP_EVENT_STOP_REQUEST |
                                    APP_EVENT_FATAL_STATE,
                                watchdog_registered)) {
            break;
        }
        if (xQueueReceive(queue, &newer, 0U) == pdTRUE) {
            request = newer;
            completed = 0U;
        }
    }
    audio_output_shutdown();
}

static void audio_task(void *context) {
    QueueHandle_t queue = app_resources_audio_queue();
    QueueHandle_t button_sound_queue = app_resources_button_sound_queue();
    vario_result_t result = {0};
    system_snapshot_t system = {0};
    app_config_t config = {0};
    static vario_audio_state_t audio_state;
    vario_audio_command_t command = {0};
    uint32_t config_revision = 0U;
    uint32_t previous_config_revision = 0U;
    bool config_revision_valid = false;
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
                (void) xEventGroupSetBits(
                    event_group,
                    APP_EVENT_AUDIO_QUIESCED);
            }
            for (;;) {
                EventBits_t bits = event_group == NULL
                                       ? APP_EVENT_SHUTDOWN_SOUND_ABORT
                                       : xEventGroupGetBits(event_group);

                if ((bits & APP_EVENT_SHUTDOWN_SOUND_ABORT) != 0U) {
                    break;
                }
                if ((bits & APP_EVENT_SHUTDOWN_SOUND_REQUEST) != 0U) {
                    (void) app_resources_copy_system(&system);
                    sound_result = play_system_sound(
                        shutdown_sound_steps,
                        sizeof(shutdown_sound_steps) /
                            sizeof(shutdown_sound_steps[0]),
                        APP_EVENT_SHUTDOWN_SOUND_ABORT,
                        watchdog_registered,
                        volume_amplifier_mode(
                            selected_volume_level(&system)), NULL);
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
        if (button_sound_queue != NULL) {
            audio_notification_request_t notification = {0};

            if (xQueueReceive(button_sound_queue, &notification, 0U) ==
                pdTRUE) {
                play_latest_button_notification(
                    button_sound_queue, notification,
                    watchdog_registered);
                vario_audio_reset(&audio_state);
                continue;
            }
        }

        if (queue != NULL) {
            (void) xQueueReceive(queue, &result, pdMS_TO_TICKS(AUDIO_EVALUATION_PERIOD_MS));
        } else {
            vTaskDelay(pdMS_TO_TICKS(AUDIO_EVALUATION_PERIOD_MS));
        }
        (void) app_resources_apply_debug_vario(&result,
                                               esp_timer_get_time());
        if (app_resources_copy_config_with_revision(&config,
                                                    &config_revision)) {
            if (config_revision_valid &&
                config_revision != previous_config_revision) {
                vario_audio_reset(&audio_state);
            }
            previous_config_revision = config_revision;
            config_revision_valid = true;
        }
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
    vTaskDelay(pdMS_TO_TICKS(SYSTEM_SAMPLE_PERIOD_MS));

    for (;;) {
        bool pressed = debounce_button(&sw1, board_is_sw1_pressed());
        bool power_on_hold_active = false;

        if (sw1.stable_valid && !pressed) {
            was_released = true;
            hold_time_ms = 0U;
        } else if (sw1.stable_valid && was_released) {
            if (UINT32_MAX - hold_time_ms < SYSTEM_SAMPLE_PERIOD_MS) {
                hold_time_ms = UINT32_MAX;
            } else {
                hold_time_ms += SYSTEM_SAMPLE_PERIOD_MS;
            }
            power_on_hold_active = true;
        }
        if (power_on_hold_active) {
            board_set_status_leds_brightness(
                power_on_green_brightness_percent(hold_time_ms), false);
        } else {
            board_set_safe_indicators();
        }
        if (power_on_hold_active && hold_time_ms >= POWER_ON_HOLD_MS) {
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
        vTaskDelay(pdMS_TO_TICKS(SAFE_STOP_PERIOD_MS));
    }
}

void app_tasks_run_safe_stop(void) {
    board_set_safe_indicators();
    (void) board_set_power_hold(false);
    run_safe_stop_loop(NULL, 0U, false);
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
    EventGroupHandle_t event_group = app_resources_event_group();
    button_debounce_t sw1 = {0};
    button_debounce_t sw2 = {0};
    button_debounce_t sw3 = {0};
    system_snapshot_t snapshot = {0};
    switch_preferences_t persisted_preferences =
        initial_switch_preferences;
    bool force_preferences_save = initial_switch_preferences_dirty;
    uint32_t sw1_hold_time_ms = 0U;
    uint32_t sw3_hold_time_ms = 0U;
    uint32_t battery_elapsed_ms = BATTERY_SAMPLE_PERIOD_MS;
    uint32_t led_elapsed_ms = 0U;
    bool sw1_was_released = false;
    bool previous_sw1_pressed = false;
    bool previous_sw2_pressed = false;
    bool previous_sw3_pressed = false;
    bool sw1_short_press_pending = false;
    bool sw1_power_off_issued = false;
    bool sw3_short_press_pending = false;
    bool sw3_skip_request_issued = false;

    (void) context;
    ESP_LOGI(TAG, "system_task started on core %d", xPortGetCoreID());

    sw1.candidate_pressed = board_is_sw1_pressed();
    sw1.stable_pressed = sw1.candidate_pressed;
    sw2.candidate_pressed = system_io_sw2_pressed();
    sw2.stable_pressed = sw2.candidate_pressed;
    sw3.candidate_pressed = system_io_sw3_pressed();
    sw3.stable_pressed = sw3.candidate_pressed;
    previous_sw1_pressed = sw1.stable_pressed;
    previous_sw2_pressed = sw2.stable_pressed;
    previous_sw3_pressed = sw3.stable_pressed;
    snapshot.volume_override_active = true;
    snapshot.volume_level = persisted_preferences.volume_level;
    snapshot.sink_override_active = true;
    snapshot.sink_enabled_override = persisted_preferences.sink_enabled;
    snapshot.parameter_number = persisted_preferences.parameter_number;
    if (app_resources_copy_config_profiles(&system_profile_snapshot)) {
        snapshot.parameter_set_count =
            (uint8_t) system_profile_snapshot.count;
    }
    snapshot.switch_preferences_dirty = initial_switch_preferences_dirty;
    vTaskDelay(pdMS_TO_TICKS(SYSTEM_SAMPLE_PERIOD_MS));

    for (;;) {
        EventBits_t event_bits =
            event_group == NULL ? 0U : xEventGroupGetBits(event_group);
        bool imu_accel_calibration_required =
            (event_bits & APP_EVENT_IMU_ACCEL_CALIBRATION_REQUIRED) != 0U;
        bool auto_power_off_issued = false;
        bool auto_power_off_config_valid = false;
        bool auto_power_off_altitude_valid = false;
        uint32_t auto_power_off_minutes = 0U;
        uint32_t auto_power_off_config_revision = 0U;
        float auto_power_off_altitude_m = 0.0f;

        snapshot.timestamp_us = esp_timer_get_time();
        snapshot.sw1_pressed = debounce_button(&sw1, board_is_sw1_pressed());
        snapshot.sw2_pressed = debounce_button(&sw2, system_io_sw2_pressed());
        snapshot.sw3_pressed = debounce_button(&sw3, system_io_sw3_pressed());
        snapshot.external_power_present = system_io_external_power_present();

        auto_power_off_config_valid = app_resources_copy_config_with_revision(
            &system_auto_power_off_config,
            &auto_power_off_config_revision);
        if (auto_power_off_config_valid) {
            if (system_auto_power_off_config_revision_valid &&
                auto_power_off_config_revision !=
                    system_auto_power_off_config_revision) {
                auto_power_off_reset(&system_auto_power_off_state);
            }
            system_auto_power_off_config_revision =
                auto_power_off_config_revision;
            system_auto_power_off_config_revision_valid = true;
            auto_power_off_minutes =
                system_auto_power_off_config.auto_power_off_minutes;
        } else {
            system_auto_power_off_config_revision_valid = false;
        }
        if (app_resources_copy_vario(&system_auto_power_off_vario)) {
            auto_power_off_altitude_valid =
                system_auto_power_off_vario.estimate_valid;
            auto_power_off_altitude_m =
                system_auto_power_off_vario.altitude_m;
        }
        auto_power_off_issued = auto_power_off_update(
            &system_auto_power_off_state, auto_power_off_minutes,
            snapshot.external_power_present,
            auto_power_off_config_valid && auto_power_off_altitude_valid,
            auto_power_off_altitude_m, snapshot.timestamp_us);

        if (sw1.stable_valid && !snapshot.sw1_pressed) {
            if (previous_sw1_pressed && sw1_was_released &&
                sw1_short_press_pending && !sw1_power_off_issued) {
                snapshot.volume_level =
                    next_volume_level(snapshot.volume_level);
                snapshot.volume_override_active = true;
                update_switch_preferences_dirty(
                    &snapshot, &persisted_preferences,
                    force_preferences_save);
                request_button_sound(snapshot.volume_level, 1U);
            }
            sw1_was_released = true;
            sw1_hold_time_ms = 0U;
            sw1_short_press_pending = false;
            sw1_power_off_issued = false;
        } else if (sw1.stable_valid && sw1_was_released) {
            if (!previous_sw1_pressed) {
                sw1_hold_time_ms = 0U;
                sw1_short_press_pending = true;
                sw1_power_off_issued = false;
            }
            if (UINT32_MAX - sw1_hold_time_ms <
                SYSTEM_SAMPLE_PERIOD_MS) {
                sw1_hold_time_ms = UINT32_MAX;
            } else {
                sw1_hold_time_ms += SYSTEM_SAMPLE_PERIOD_MS;
            }
            if (sw1_hold_time_ms >= POWER_OFF_HOLD_MS) {
                sw1_short_press_pending = false;
                sw1_power_off_issued = true;
            }
        } else {
            /* The switch used to start the board is not a power-off request. */
        }
        snapshot.sw1_hold_ms = sw1_hold_time_ms;

        if (sw2.stable_valid && snapshot.sw2_pressed &&
            !previous_sw2_pressed) {
            toggle_sink_override(&snapshot);
            update_switch_preferences_dirty(&snapshot,
                                            &persisted_preferences,
                                            force_preferences_save);
            request_sink_status_sound(selected_volume_level(&snapshot),
                                      snapshot.sink_enabled_override);
        }
        if (sw3.stable_valid && snapshot.sw3_pressed) {
            if (!previous_sw3_pressed) {
                sw3_hold_time_ms = 0U;
                sw3_skip_request_issued = false;
                sw3_short_press_pending =
                    imu_accel_calibration_required;
                if (!imu_accel_calibration_required) {
                    if (select_next_parameter_set(&snapshot)) {
                        update_switch_preferences_dirty(
                            &snapshot, &persisted_preferences,
                            force_preferences_save);
                    }
                }
            }
            if (imu_accel_calibration_required &&
                sw3_short_press_pending &&
                !sw3_skip_request_issued) {
                if (UINT32_MAX - sw3_hold_time_ms <
                    SYSTEM_SAMPLE_PERIOD_MS) {
                    sw3_hold_time_ms = UINT32_MAX;
                } else {
                    sw3_hold_time_ms += SYSTEM_SAMPLE_PERIOD_MS;
                }
                if (sw3_hold_time_ms >=
                    IMU_CALIBRATION_SKIP_HOLD_MS) {
                    if (event_group != NULL) {
                        (void) xEventGroupSetBits(
                            event_group,
                            APP_EVENT_IMU_ACCEL_CALIBRATION_SKIP_REQUEST);
                    }
                    sw3_skip_request_issued = true;
                    sw3_short_press_pending = false;
                    ESP_LOGW(TAG,
                             "SW3 long press requested initial IMU calibration skip");
                }
            }
        } else if (sw3.stable_valid && previous_sw3_pressed) {
            if (sw3_short_press_pending &&
                !sw3_skip_request_issued) {
                if (select_next_parameter_set(&snapshot)) {
                    update_switch_preferences_dirty(
                        &snapshot, &persisted_preferences,
                        force_preferences_save);
                }
            }
            sw3_hold_time_ms = 0U;
            sw3_short_press_pending = false;
            sw3_skip_request_issued = false;
        }
        snapshot.sw3_hold_ms = sw3_hold_time_ms;
        previous_sw1_pressed = snapshot.sw1_pressed;
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

        set_lifecycle_leds(
            led_elapsed_ms, sw1_hold_time_ms,
            snapshot.external_power_present, snapshot.battery_valid,
            snapshot.battery_voltage_v);
        if ((event_bits & APP_EVENT_BMP581_STARTUP_COMPLETE) != 0U) {
            led_elapsed_ms += SYSTEM_SAMPLE_PERIOD_MS;
        }

        (void) app_resources_publish_system(&snapshot);

        if (auto_power_off_issued) {
            ESP_LOGI(TAG,
                     "automatic power-off requested after %" PRIu32
                     " minutes within %.1f m altitude range",
                     auto_power_off_minutes,
                     (double) AUTO_POWER_OFF_ALTITUDE_RANGE_M);
        }
        if (sw1_power_off_issued || auto_power_off_issued) {
            request_power_off(&snapshot);
        }

        vTaskDelay(pdMS_TO_TICKS(SYSTEM_SAMPLE_PERIOD_MS));
    }
}

static bool console_writef(const char *format, ...) {
    char output[1280] = {0};
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
        " imu_accel_calibrated=%d imu_accel_cal_persisted=%d"
        " imu_accel_cal_skipped=%d"
        " imu_stale=%d q_w=%.5f q_x=%.5f q_y=%.5f q_z=%.5f"
        " roll_deg=%.2f pitch_deg=%.2f yaw_deg=%.2f"
        " vertical_accel_mps2=%.3f vertical_accel_valid=%d"
        " fusion_active=%d"
        " kalman_accel_bias_mps2=%.4f"
        " kalman_baro_innovation_m=%.4f"
        " kalman_baro_innovation_valid=%d"
        " kalman_accel_innovation_mps2=%.4f"
        " kalman_accel_innovation_valid=%d"
        " kalman_baro_r_m2=%.5f kalman_accel_r_m2_s4=%.5f"
        " imu_samples=%" PRIu32
        " imu_missed=%" PRIu32
        " imu_confidence=%.3f imu_vibration_rms_g=%.4f"
        " imu_kp_effective=%.4f imu_ki_effective=%.4f"
        " imu_ki_active=%d"
        " imu_cal_samples=%" PRIu32
        " imu_cal_save_pending=%d imu_cal_storage=%s"
        " imu_cal_storage_error=%" PRId32
        " stream_drops=%" PRIu32 "\r\n",
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
        imu.online, imu.calibrated, imu.attitude_valid,
        imu.accel_calibrated, imu.accel_calibration_persisted,
        imu.accel_calibration_skipped,
        imu.stale,
        (double) imu.quaternion[0], (double) imu.quaternion[1],
        (double) imu.quaternion[2], (double) imu.quaternion[3],
        (double) imu.roll_deg, (double) imu.pitch_deg,
        (double) imu.yaw_deg, (double) vario.vertical_accel_mps2,
        vario.vertical_accel_valid, vario.imu_fusion_active,
        (double) vario.kalman_accel_bias_mps2,
        (double) vario.kalman_baro_innovation_m,
        vario.kalman_baro_innovation_valid,
        (double) vario.kalman_accel_innovation_mps2,
        vario.kalman_accel_innovation_valid,
        (double) vario.kalman_baro_r_m2,
        (double) vario.kalman_accel_r_m2_s4,
        imu.sample_count, vario.missed_imu_sample_count,
        (double) imu.confidence, (double) imu.vibration_rms_g,
        (double) imu.kp_effective, (double) imu.ki_effective,
        imu.ki_active,
        imu.accel_calibration_sample_count,
        imu.accel_calibration_save_pending,
        imu_calibration_storage_result_name(
            (imu_calibration_storage_result_t)
                imu.accel_calibration_storage_result),
        imu.accel_calibration_storage_error,
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
    switch_preferences_diagnostics_t switch_diagnostics = {0};
    int64_t now_us = esp_timer_get_time();

    (void) app_resources_copy_vario(&vario);
    (void) app_resources_apply_debug_vario(&vario, now_us);
    (void) app_resources_copy_imu_diagnostics(&imu);
    (void) app_resources_copy_system(&system);
    usb_device_get_diagnostics(&usb);
    firmware_update_get_diagnostics(&update);
    ble_vario_get_diagnostics(&ble);
    app_power_get_diagnostics(&power);
    switch_preferences_get_diagnostics(&switch_diagnostics);

    console_writef(
        "VARIO bmp=%d pressure_valid=%d climb_valid=%d fusion=%d "
        "vertical_accel_valid=%d debug=%d pressure_pa=%.2f "
        "raw_temp=%" PRId32 " raw_pressure=%" PRIu32
        " altitude_m=%.2f climb_mps=%.2f vertical_accel_mps2=%.3f "
        "kalman_accel_bias_mps2=%.4f "
        "kalman_baro_innovation_m=%.4f kalman_baro_innovation_valid=%d "
        "kalman_accel_innovation_mps2=%.4f "
        "kalman_accel_innovation_valid=%d "
        "kalman_baro_r_m2=%.5f kalman_accel_r_m2_s4=%.5f "
        "i2c_errors=%" PRIu32 " bmp_overruns=%" PRIu32
        " imu_missed=%" PRIu32 "\r\n",
        vario.bmp581_online, vario.pressure_valid, vario.climb_rate_valid,
        vario.imu_fusion_active, vario.vertical_accel_valid,
        vario.debug_input_active,
        (double) vario.pressure_pa_x100 / 100.0,
        vario.raw_temperature, vario.raw_pressure,
        (double) vario.altitude_m, (double) vario.climb_rate_mps,
        (double) vario.vertical_accel_mps2,
        (double) vario.kalman_accel_bias_mps2,
        (double) vario.kalman_baro_innovation_m,
        vario.kalman_baro_innovation_valid,
        (double) vario.kalman_accel_innovation_mps2,
        vario.kalman_accel_innovation_valid,
        (double) vario.kalman_baro_r_m2,
        (double) vario.kalman_accel_r_m2_s4,
        vario.i2c_error_count,
        vario.bmp_period_overrun_count, vario.missed_imu_sample_count);
    console_writef(
        "IMU enabled=%d online=%d configured=%d calibrated=%d "
        "accel_calibrated=%d accel_persisted=%d accel_save_pending=%d "
        "accel_skipped=%d "
        "attitude=%d stale=%d fusion=%d target_address=0x%02x "
        "address=0x%02x who_am_i=0x%02x status=0x%02x "
        "retries=%" PRIu32 " samples=%" PRIu32 " calibration=%" PRIu32
        " accel_calibration=%" PRIu32
        " missed=%" PRIu32 " errors=%" PRIu32 " accel_norm_g=%.3f "
        "accel_offset_mps2=%.5f,%.5f,%.5f "
        "gyro_bias_radps=%.5f,%.5f,%.5f "
        "confidence=%.3f vibration_rms_g=%.4f "
        "kp_effective=%.4f ki_effective=%.4f ki_active=%d "
        "mc_data=%s mc_error=%" PRId32 " "
        "q=%.5f,%.5f,%.5f,%.5f roll_deg=%.2f pitch_deg=%.2f "
        "yaw_deg=%.2f last_error=%s(%" PRId32 ")\r\n",
        imu.enabled, imu.online, imu.configured, imu.calibrated,
        imu.accel_calibrated, imu.accel_calibration_persisted,
        imu.accel_calibration_save_pending,
        imu.accel_calibration_skipped, imu.attitude_valid,
        imu.stale, imu.fusion_active,
        ICM42688_HXY_I2C_ADDRESS, imu.address, imu.who_am_i,
        imu.data_status, imu.retry_count, imu.sample_count,
        imu.calibration_sample_count,
        imu.accel_calibration_sample_count,
        imu.missed_interrupt_count,
        imu.consecutive_error_count, (double) imu.accel_norm_g,
        (double) imu.accel_offset_mps2[0],
        (double) imu.accel_offset_mps2[1],
        (double) imu.accel_offset_mps2[2],
        (double) imu.gyro_bias_radps[0],
        (double) imu.gyro_bias_radps[1],
        (double) imu.gyro_bias_radps[2],
        (double) imu.confidence, (double) imu.vibration_rms_g,
        (double) imu.kp_effective, (double) imu.ki_effective,
        imu.ki_active,
        imu_calibration_storage_result_name(
            (imu_calibration_storage_result_t)
                imu.accel_calibration_storage_result),
        imu.accel_calibration_storage_error,
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
        " sw3_hold_ms=%" PRIu32
        " volume_override=%d volume_level=%d sink_override=%d"
        " sink_enabled=%d parameter_number=%u parameter_sets=%u"
        " switch_dirty=%d power_off=%d\r\n",
        system.battery_valid, (double) system.battery_voltage_v,
        system.battery_raw, system.battery_adc_mv,
        system.battery_sample_count, system.battery_error_count,
        system.battery_saturation_count, system.external_power_present,
        system.sw1_pressed, system.sw2_pressed, system.sw3_pressed,
        system.sw1_hold_ms, system.sw3_hold_ms,
        system.volume_override_active,
        (int) system.volume_level, system.sink_override_active,
        system.sink_enabled_override,
        (unsigned int) system.parameter_number,
        (unsigned int) system.parameter_set_count,
        system.switch_preferences_dirty,
        system.power_off_requested);
    console_writef(
        "SWITCH source=%s load=%s load_error=%s save_result=%s"
        " clear_result=%s load_errors=%" PRIu32
        " saves=%" PRIu32 " save_errors=%" PRIu32
        " clear_errors=%" PRIu32 "\r\n",
        switch_preferences_source_name(switch_diagnostics.source),
        switch_preferences_load_result_name(switch_diagnostics.load_result),
        esp_err_to_name(switch_diagnostics.last_load_error),
        esp_err_to_name(switch_diagnostics.last_save_result),
        esp_err_to_name(switch_diagnostics.last_clear_result),
        switch_diagnostics.load_error_count,
        switch_diagnostics.save_count,
        switch_diagnostics.save_error_count,
        switch_diagnostics.clear_error_count);
    console_writef(
        "USB tinyusb=%d cdc=%d msc_driver=%d msc_enabled=%d msc_media=%d"
        " attached=%d dtr=%d vbus=%d storage=%d owner=%s load=%d"
        " config_source=%s config_validation=%s config_version=%" PRId32
        " config_key=%s config_io_error=%" PRId32 " "
        "storage_error=%s last_save=%s attach_count=%" PRIu32
        " detach_count=%" PRIu32 " mount_errors=%" PRIu32
        " format_required=%" PRIu32 " rx_errors=%" PRIu32
        " tx_errors=%" PRIu32
        " stream_drops=%" PRIu32 "\r\n",
        usb.driver_ready, usb.cdc_ready, usb.msc_driver_ready,
        usb.msc_enabled,
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
    uint8_t parameter_number = 0U;

    if (token_count < 2U ||
        !app_resources_copy_active_config(&config, &parameter_number)) {
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
            !app_resources_publish_config_for_profile(&config,
                                                      parameter_number)) {
            console_writef("ERR INVALID_VALUE\r\n");
            return;
        }
        console_writef("OK\r\n");
        return;
    }
    if (strcasecmp(tokens[1], "RESET") == 0 && token_count == 3U) {
        if (!app_config_reset(&config, parameter_number, tokens[2]) ||
            !app_resources_publish_config_for_profile(&config,
                                                      parameter_number)) {
            console_writef("ERR INVALID_RESET\r\n");
            return;
        }
        console_writef("OK\r\n");
        return;
    }
    if (strcasecmp(tokens[1], "SAVE") == 0 && token_count == 2U) {
        esp_err_t ret = ESP_OK;

        if (!app_resources_copy_config_profiles(&console_profile_snapshot)) {
            console_writef("ERR PARAM\r\n");
            return;
        }
        ret = usb_device_save_config(&console_profile_snapshot);
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
    EventGroupHandle_t event_group = app_resources_event_group();
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
            system_snapshot_t final_system = {0};

            if (event_group != NULL) {
                (void) xEventGroupWaitBits(
                    event_group, APP_EVENT_AUDIO_QUIESCED,
                    pdFALSE, pdFALSE, portMAX_DELAY);
            }
            if (app_resources_copy_system(&final_system) &&
                final_system.switch_preferences_dirty) {
                switch_preferences_t preferences = {
                    .volume_level = final_system.volume_level,
                    .sink_enabled = final_system.sink_enabled_override,
                    .parameter_number = final_system.parameter_number,
                };
                esp_err_t save_ret =
                    switch_preferences_save(&preferences);

                if (save_ret != ESP_OK) {
                    ESP_LOGW(TAG,
                             "switch preferences shutdown save failed: %s",
                             esp_err_to_name(save_ret));
                    post_runtime_diagnostic(
                        DIAGNOSTIC_EVENT_PERIPHERAL_FAILURE, save_ret);
                }
            }
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

        if (app_resources_copy_system(&system)) {
            ble_vario_update_battery(&system);
        }
        if (app_resources_copy_vario(&vario) &&
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
    vTaskDelay(pdMS_TO_TICKS(SYSTEM_SAMPLE_PERIOD_MS));

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
            app_tasks_run_safe_stop();
        }

        vTaskDelay(pdMS_TO_TICKS(SYSTEM_SAMPLE_PERIOD_MS));
    }
}
