#include "app/startup.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app/app_resources.h"
#include "app/app_tasks.h"
#include "domain/app_config.h"
#include "domain/battery_level.h"
#include "domain/system_policy.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "platform/app_power.h"
#include "platform/audio_output.h"
#include "platform/ble_vario.h"
#include "platform/board.h"
#include "platform/board_identity_storage.h"
#include "platform/firmware_update.h"
#include "platform/sensor_bus.h"
#include "platform/system_io.h"
#include "platform/switch_preferences.h"
#include "platform/usb_device_service.h"
#include "platform/watchdog_service.h"

#if !CONFIG_IDF_TARGET_ESP32S3
#error "CloudBaseVario initial firmware targets ESP32-S3"
#endif

#if CONFIG_FREERTOS_UNICORE
#error "CloudBaseVario requires the standard dual-core ESP-IDF FreeRTOS configuration"
#endif

#if CONFIG_FREERTOS_SMP
#error "CloudBaseVario does not use the experimental Amazon SMP FreeRTOS kernel"
#endif

#if CONFIG_FREERTOS_NUMBER_OF_CORES != 2
#error "CloudBaseVario requires FreeRTOS on both ESP32-S3 cores"
#endif

#if CONFIG_FREERTOS_HZ != 1000
#error "CloudBaseVario requires a 1000 Hz FreeRTOS tick"
#endif

#if !CONFIG_ESP_CONSOLE_NONE
#error "CloudBaseVario routes its console through the TinyUSB CDC VFS"
#endif

#if CONFIG_ESP_CONSOLE_USB_CDC || CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG || \
    CONFIG_ESP_CONSOLE_UART_DEFAULT || CONFIG_ESP_CONSOLE_UART_CUSTOM
#error "CloudBaseVario must not reserve a second ESP-IDF console transport"
#endif

#if !CONFIG_TINYUSB_CDC_ENABLED || !CONFIG_TINYUSB_MSC_ENABLED
#error "CloudBaseVario requires the TinyUSB CDC + MSC composite classes"
#endif

#if !CONFIG_TINYUSB_DFU_MODE_NONE
#error "CloudBaseVario uses file-based MSC OTA, not a USB DFU interface"
#endif

#if !CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
#error "CloudBaseVario file-based OTA requires bootloader rollback"
#endif

#if !CONFIG_BOOTLOADER_WDT_ENABLE || CONFIG_BOOTLOADER_WDT_TIME_MS != 9000
#error "CloudBaseVario requires the 9 second bootloader RTC watchdog"
#endif

#if CONFIG_BOOTLOADER_WDT_DISABLE_IN_USER_CODE
#error "The bootloader RTC watchdog must stop before app_main"
#endif

#if !CONFIG_ESP_INT_WDT || CONFIG_ESP_INT_WDT_TIMEOUT_MS != 300 || \
    !CONFIG_ESP_INT_WDT_CHECK_CPU1
#error "CloudBaseVario requires the 300 ms interrupt watchdog on both CPUs"
#endif

#if !CONFIG_ESP_TASK_WDT_EN || !CONFIG_ESP_TASK_WDT_INIT || \
    !CONFIG_ESP_TASK_WDT_PANIC || CONFIG_ESP_TASK_WDT_TIMEOUT_S != 5 || \
    !CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0 || \
    !CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
#error "CloudBaseVario requires the 5 second panic Task Watchdog"
#endif

#if !CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT || \
    CONFIG_ESP_SYSTEM_PANIC_REBOOT_DELAY_SECONDS != 0
#error "CloudBaseVario requires immediate reboot after panic diagnostics"
#endif

#if CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ != 80
#error "CloudBaseVario requires an 80 MHz default and maximum CPU frequency"
#endif

#if !CONFIG_PM_ENABLE || !CONFIG_FREERTOS_USE_TICKLESS_IDLE
#error "CloudBaseVario requires DFS power management and tickless idle"
#endif

#if !CONFIG_FATFS_SECTOR_512 || !CONFIG_WL_SECTOR_SIZE_512 || \
    !CONFIG_WL_SECTOR_MODE_SAFE
#error "CloudBaseVario requires 512-byte FAT/WL sectors in Safety mode"
#endif

#define EXPECTED_FLASH_SIZE_BYTES UINT32_C(16777216)
#define EXPECTED_PSRAM_SIZE_BYTES UINT32_C(8388608)
#define STARTUP_FORMAT_SAMPLE_PERIOD_MS SYSTEM_POLICY_SAMPLE_PERIOD_MS
#define STARTUP_POWER_SAMPLE_PERIOD_MS SYSTEM_POLICY_SAMPLE_PERIOD_MS
#define STARTUP_MODE_HOLD_MS UINT32_C(2000)
#define STARTUP_RECOVERY_HOLD_MS \
    (STARTUP_MODE_HOLD_MS + SYSTEM_POLICY_SWITCH_DEBOUNCE_MS)
#define RECOVERY_STORAGE_RELEASE_WAIT_MS UINT32_C(100)
#define STARTUP_PREP_TASK_STACK_BYTES UINT32_C(4096)
#define STARTUP_PREP_TASK_PRIORITY ((UBaseType_t) 5U)
#define STARTUP_PREP_TASK_CORE ((BaseType_t) 1)
#define UPDATE_CONFIRMATION_USB_WAIT_MS UINT32_C(15000)
#define STARTUP_BATTERY_SAMPLE_PERIOD_MS UINT32_C(100)
#define STARTUP_BATTERY_SAMPLE_COUNT UINT32_C(5)
#define STARTUP_PREPARATION_WAIT_MS UINT32_C(250)

typedef struct {
    bool confirmed;
    bool config_format_requested;
} startup_power_on_result_t;

typedef enum {
    STARTUP_BOOT_GESTURE_NONE = 0,
    STARTUP_BOOT_GESTURE_AMBIGUOUS,
    STARTUP_BOOT_GESTURE_RECOVERY,
} startup_boot_gesture_t;

typedef struct {
    esp_err_t audio_result;
    esp_err_t nvs_result;
    switch_preferences_load_result_t switch_load_result;
    switch_preferences_t switch_preferences;
} startup_preparation_result_t;

static const char *TAG = "startup";
static app_config_profiles_t startup_runtime_profiles;
static startup_preparation_result_t startup_preparation_result;
static TaskHandle_t startup_preparation_waiter = NULL;
static volatile bool startup_preparation_watchdog_failed;
static void feed_startup_watchdog(void);

static void board_identity_service_safe_stop(
    const board_identity_storage_diagnostics_t *diagnostics) {
    board_identity_load_result_t result = BOARD_IDENTITY_LOAD_IO_ERROR;
    esp_err_t error = ESP_FAIL;

    if (diagnostics != NULL) {
        result = diagnostics->result;
        error = diagnostics->error;
    }
    ESP_LOGE(TAG, "board identity unavailable: status=%s error=%s",
             board_identity_load_result_name(result),
             esp_err_to_name(error));
    ESP_LOGE(TAG, "service-safe-stop: board-specific services remain disabled");
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        feed_startup_watchdog();
    }
}

static void feed_startup_watchdog(void) {
    (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
}

static void report_startup_storage_progress(void *arg) {
    (void) arg;
    feed_startup_watchdog();
}

static startup_boot_gesture_t startup_recovery_gesture(void) {
    uint32_t stable_time_ms = 0U;

    if (!system_io_sw2_pressed() || !system_io_sw3_pressed()) {
        return STARTUP_BOOT_GESTURE_NONE;
    }
    while (system_io_sw2_pressed() && system_io_sw3_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(STARTUP_POWER_SAMPLE_PERIOD_MS));
        feed_startup_watchdog();
        stable_time_ms += STARTUP_POWER_SAMPLE_PERIOD_MS;
        if (stable_time_ms >= STARTUP_RECOVERY_HOLD_MS) {
            return STARTUP_BOOT_GESTURE_RECOVERY;
        }
    }
    return STARTUP_BOOT_GESTURE_AMBIGUOUS;
}

static void recovery_stop(esp_err_t error, const char *message) {
    if (message != NULL) {
        ESP_LOGE(TAG, "%s: %s", message, esp_err_to_name(error));
    }
    (void) usb_device_stop();
    board_set_status_leds(false, true);
    (void) board_set_power_hold(false);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(RECOVERY_STORAGE_RELEASE_WAIT_MS));
        feed_startup_watchdog();
    }
}

static void run_recovery_mode(void) {
    usb_device_diagnostics_t diagnostics = {0};
    esp_err_t ret;

    watchdog_service_mark_stage(WATCHDOG_STAGE_INITIALIZING);
    board_set_status_leds(false, true);
    if (!system_io_external_power_present()) {
        recovery_stop(ESP_ERR_INVALID_STATE,
                      "MSC recovery requires USB VBUS");
    }

    ret = usb_device_recovery_storage_init(
        report_startup_storage_progress, NULL);
    feed_startup_watchdog();
    if (ret != ESP_OK) {
        recovery_stop(ret, "recovery config FAT initialization failed");
    }

    ret = firmware_update_process_recovery(true);
    feed_startup_watchdog();
    if (ret != ESP_ERR_NOT_FOUND && ret != ESP_OK) {
        ESP_LOGW(TAG, "stored recovery update rejected: %s",
                 esp_err_to_name(ret));
    }

    ret = usb_device_start_recovery();
    if (ret != ESP_OK) {
        recovery_stop(ret, "recovery USB initialization failed");
    }
    ESP_LOGW(TAG,
             "MSC recovery active; copy signed UPDATE.BIN and safely eject");

    for (;;) {
        uint32_t release_count;

        board_set_status_leds(false, true);
        usb_device_get_diagnostics(&diagnostics);
        release_count = diagnostics.host_release_count;
        ret = usb_device_enable_msc();
        if (ret != ESP_OK) {
            recovery_stop(ret, "recovery MSC exposure failed");
        }

        do {
            ret = usb_device_wait_for_host_release(
                release_count, RECOVERY_STORAGE_RELEASE_WAIT_MS);
            feed_startup_watchdog();
            if (!system_io_external_power_present()) {
                recovery_stop(ESP_ERR_INVALID_STATE,
                              "USB VBUS lost during recovery wait");
            }
        } while (ret == ESP_ERR_TIMEOUT);
        if (ret != ESP_OK) {
            recovery_stop(ret, "recovery MSC ownership failed");
        }

        ret = firmware_update_process_recovery(true);
        feed_startup_watchdog();
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "safe eject completed without UPDATE.BIN");
        } else if (ret != ESP_OK) {
            ESP_LOGW(TAG, "recovery update rejected: %s",
                     esp_err_to_name(ret));
        }
    }
}

static bool nvs_recovery_required(esp_err_t result) {
    return result == ESP_ERR_NVS_NO_FREE_PAGES ||
           result == ESP_ERR_NVS_NEW_VERSION_FOUND;
}

static esp_err_t recover_nvs_after_power_confirmation(
    esp_err_t initial_result) {
    if (!nvs_recovery_required(initial_result)) {
        return initial_result;
    }

    ESP_LOGW(TAG, "NVS requires one-time erase and recovery");
    esp_err_t result = nvs_flash_erase();

    if (result == ESP_OK) {
        result = nvs_flash_init();
    }
    return result;
}

static void run_startup_preparation(void) {
    switch_preferences_set_defaults(
        &startup_preparation_result.switch_preferences);
    startup_preparation_result.switch_load_result =
        SWITCH_PREFERENCES_LOAD_NOT_FOUND;
    startup_preparation_result.audio_result = audio_output_init();
    startup_preparation_result.nvs_result = nvs_flash_init();
    if (startup_preparation_result.nvs_result == ESP_OK) {
        startup_preparation_result.switch_load_result =
            switch_preferences_load(
                &startup_preparation_result.switch_preferences);
    }
}

static void startup_preparation_task(void *context) {
    TaskHandle_t waiter = startup_preparation_waiter;
    esp_err_t watchdog_result;

    (void) context;
    watchdog_result = watchdog_service_register_current(
        WATCHDOG_ACTOR_STARTUP_PREP);
    if (watchdog_result == ESP_OK) {
        run_startup_preparation();
        (void) watchdog_service_unregister_current(
            WATCHDOG_ACTOR_STARTUP_PREP);
    } else {
        startup_preparation_watchdog_failed = true;
        ESP_LOGE(TAG, "startup_prep watchdog registration failed: %s",
                 esp_err_to_name(watchdog_result));
    }
    if (waiter != NULL) {
        xTaskNotifyGive(waiter);
    }
    vTaskDelete(NULL);
}

static bool start_startup_preparation(void) {
    startup_preparation_waiter = xTaskGetCurrentTaskHandle();
    startup_preparation_watchdog_failed = false;
    return xTaskCreatePinnedToCore(
               startup_preparation_task, "startup_prep",
               STARTUP_PREP_TASK_STACK_BYTES, NULL,
               STARTUP_PREP_TASK_PRIORITY, NULL,
               STARTUP_PREP_TASK_CORE) == pdPASS;
}

static void wait_for_startup_preparation(bool started_async) {
    if (started_async) {
        while (ulTaskNotifyTake(
                   pdTRUE, pdMS_TO_TICKS(STARTUP_PREPARATION_WAIT_MS)) == 0U) {
            feed_startup_watchdog();
        }
    } else {
        run_startup_preparation();
    }
    feed_startup_watchdog();
}

static void log_hardware_configuration(void) {
    uint32_t flash_size_bytes = 0U;
    size_t psram_size_bytes = esp_psram_get_size();
    esp_err_t ret = esp_flash_get_physical_size(NULL, &flash_size_bytes);

    ESP_LOGI(TAG, "ESP-IDF %s", esp_get_idf_version());

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "flash size detection failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "flash size=%" PRIu32 " bytes", flash_size_bytes);
        if (flash_size_bytes != EXPECTED_FLASH_SIZE_BYTES) {
            ESP_LOGW(TAG, "expected 16 MB flash, detected %" PRIu32 " bytes", flash_size_bytes);
        }
    }

    ESP_LOGI(TAG, "PSRAM size=%u bytes", (unsigned int) psram_size_bytes);
    if (psram_size_bytes != EXPECTED_PSRAM_SIZE_BYTES) {
        ESP_LOGW(TAG, "expected 8 MB PSRAM, detected %u bytes", (unsigned int) psram_size_bytes);
    }
}

static void post_peripheral_failure(esp_err_t detail) {
    diagnostic_event_t event = {
        .type = DIAGNOSTIC_EVENT_PERIPHERAL_FAILURE,
        .timestamp_us = esp_timer_get_time(),
        .detail = (int32_t) detail,
    };

    (void) app_resources_post_diagnostic(&event);
}

static bool startup_read_battery(float *battery_voltage_v) {
    for (uint32_t sample = 0U; sample < STARTUP_BATTERY_SAMPLE_COUNT;
         sample++) {
        if (system_io_read_battery_voltage(battery_voltage_v)) {
            return true;
        }
        if (sample + 1U < STARTUP_BATTERY_SAMPLE_COUNT) {
            vTaskDelay(pdMS_TO_TICKS(STARTUP_BATTERY_SAMPLE_PERIOD_MS));
            feed_startup_watchdog();
        }
    }
    return false;
}

static startup_power_on_result_t startup_power_on_confirmed(
    bool format_allowed) {
    system_policy_button_t sw1 = {0};
    startup_power_on_result_t result = {0};
    uint32_t hold_time_ms = 0U;
    uint32_t format_hold_time_ms = 0U;
    bool format_eligible = format_allowed &&
                           !system_io_sw2_pressed() &&
                           system_io_sw3_pressed();

    sw1.candidate_pressed = board_is_sw1_pressed();
    sw1.stable_pressed = sw1.candidate_pressed;
    vTaskDelay(pdMS_TO_TICKS(STARTUP_POWER_SAMPLE_PERIOD_MS));
    feed_startup_watchdog();

    for (;;) {
        bool pressed = system_policy_debounce(&sw1,
                                              board_is_sw1_pressed());
        bool format_pressed =
            !system_io_sw2_pressed() && system_io_sw3_pressed();

        if (!format_pressed) {
            format_eligible = false;
        }
        if (format_eligible) {
            if (UINT32_MAX - format_hold_time_ms <
                STARTUP_FORMAT_SAMPLE_PERIOD_MS) {
                format_hold_time_ms = UINT32_MAX;
            } else {
                format_hold_time_ms += STARTUP_FORMAT_SAMPLE_PERIOD_MS;
            }
        } else {
            format_hold_time_ms = 0U;
        }

        if (sw1.stable_valid && !pressed) {
            board_set_status_leds(false, false);
            return result;
        }
        if (sw1.stable_valid) {
            if (UINT32_MAX - hold_time_ms < STARTUP_POWER_SAMPLE_PERIOD_MS) {
                hold_time_ms = UINT32_MAX;
            } else {
                hold_time_ms += STARTUP_POWER_SAMPLE_PERIOD_MS;
            }
            board_set_status_leds_brightness(
                system_policy_power_on_brightness(
                    hold_time_ms, POWER_ON_HOLD_MS),
                false);
            if (hold_time_ms >= POWER_ON_HOLD_MS &&
                (!format_eligible ||
                 format_hold_time_ms >= STARTUP_MODE_HOLD_MS)) {
                result.confirmed = true;
                result.config_format_requested =
                    format_eligible &&
                    format_hold_time_ms >= STARTUP_MODE_HOLD_MS;
                return result;
            }
        } else {
            board_set_status_leds(false, false);
        }
        vTaskDelay(pdMS_TO_TICKS(STARTUP_POWER_SAMPLE_PERIOD_MS));
        feed_startup_watchdog();
    }
}

static bool startup_config_format_requested(bool format_allowed) {
    uint32_t stable_time_ms = 0U;

    if (!format_allowed || system_io_sw2_pressed() ||
        !system_io_sw3_pressed()) {
        return false;
    }
    for (;;) {
        if (system_io_sw2_pressed() || !system_io_sw3_pressed()) {
            return false;
        }
        if (stable_time_ms >= STARTUP_MODE_HOLD_MS) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(STARTUP_FORMAT_SAMPLE_PERIOD_MS));
        feed_startup_watchdog();
        stable_time_ms += STARTUP_FORMAT_SAMPLE_PERIOD_MS;
    }
}

void app_startup_run(void) {
    esp_err_t ret = board_init_power_hold();
    board_identity_t board_identity = {0};
    board_identity_storage_diagnostics_t board_identity_diagnostics = {0};
    bool board_valid = false;
    bool config_format_requested = false;
    bool nvs_ready = false;
    bool update_confirmation_required = false;
    bool usb_msc_gate_required = false;
    bool usb_composite_active = false;
    bool imu_accel_calibration_required = false;
    bool ota_confirmation_boot = false;
    startup_boot_gesture_t boot_gesture = STARTUP_BOOT_GESTURE_NONE;
    watchdog_boot_action_t watchdog_boot_action =
        WATCHDOG_BOOT_REQUIRE_SW1;
    bool startup_preparation_started = false;
    bool external_power_present = false;
    bool startup_battery_valid = false;
    float startup_battery_voltage_v = 0.0f;
    startup_power_on_result_t power_on_result = {0};
    switch_preferences_t switch_preferences = {0};
    switch_preferences_load_result_t switch_load_result =
        SWITCH_PREFERENCES_LOAD_NOT_FOUND;
    bool switch_preferences_dirty = false;
    esp_err_t storage_result = ESP_OK;
    esp_err_t usb_result = ESP_OK;
    esp_err_t startup_gate_result = ESP_OK;
    esp_err_t startup_sound_result = ESP_OK;
    esp_err_t switch_clear_result = ESP_OK;
    esp_err_t nvs_result = ESP_OK;
    esp_err_t system_io_result = ESP_ERR_INVALID_STATE;
    firmware_update_diagnostics_t update_diagnostics = {0};
    app_config_t ble_config = {0};
    imu_accel_calibration_t imu_accel_calibration = {0};
    imu_calibration_storage_diagnostics_t imu_calibration_diagnostics = {
        .result = IMU_CALIBRATION_STORAGE_MISSING,
    };

    if (ret != ESP_OK) {
        app_tasks_run_fatal_fallback();
    }

    ret = board_init_safe_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "safe GPIO initialization failed: %s", esp_err_to_name(ret));
        app_tasks_run_fatal_fallback();
    }

    ota_confirmation_boot =
        firmware_update_running_image_pending_verify();
    ret = watchdog_service_begin_boot(ota_confirmation_boot,
                                      &watchdog_boot_action);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "watchdog service initialization failed: %s",
                 esp_err_to_name(ret));
        app_tasks_run_fatal_fallback();
    }
    ret = watchdog_service_register_current(WATCHDOG_ACTOR_STARTUP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "startup watchdog registration failed: %s",
                 esp_err_to_name(ret));
        watchdog_service_mark_stage(WATCHDOG_STAGE_FATAL);
        app_tasks_run_fatal_fallback();
    }
    if (!ota_confirmation_boot) {
        boot_gesture = startup_recovery_gesture();
        if (boot_gesture == STARTUP_BOOT_GESTURE_RECOVERY) {
            ESP_LOGW(TAG, "SW2+SW3 startup request: entering MSC recovery");
            watchdog_service_mark_user_confirmed();
            run_recovery_mode();
        }
    }

    if (board_identity_storage_load(
            &board_identity, &board_identity_diagnostics) !=
            BOARD_IDENTITY_LOAD_VALID ||
        !board_select_identity(&board_identity)) {
        board_identity_service_safe_stop(&board_identity_diagnostics);
    }

    startup_preparation_started = start_startup_preparation();
    if (ota_confirmation_boot) {
        board_set_status_leds_brightness(100U, false);
    } else if (watchdog_boot_action != WATCHDOG_BOOT_REQUIRE_SW1) {
        board_set_status_leds_brightness(100U, false);
        config_format_requested = startup_config_format_requested(
            boot_gesture == STARTUP_BOOT_GESTURE_NONE);
    } else {
        watchdog_service_mark_stage(WATCHDOG_STAGE_POWER_ON_WAIT);
        power_on_result = startup_power_on_confirmed(
            boot_gesture == STARTUP_BOOT_GESTURE_NONE);
        config_format_requested =
            power_on_result.config_format_requested;
        if (!power_on_result.confirmed) {
            wait_for_startup_preparation(startup_preparation_started);
            app_tasks_run_safe_stop();
        }
        watchdog_service_mark_user_confirmed();
    }
    watchdog_service_mark_stage(WATCHDOG_STAGE_INITIALIZING);
    wait_for_startup_preparation(startup_preparation_started);
    if (startup_preparation_watchdog_failed) {
        watchdog_service_mark_stage(WATCHDOG_STAGE_FATAL);
        app_tasks_run_fatal_fallback();
    }

    switch_preferences =
        startup_preparation_result.switch_preferences;
    switch_load_result =
        startup_preparation_result.switch_load_result;
    if (config_format_requested) {
        switch_preferences_set_defaults(&switch_preferences);
        switch_load_result = SWITCH_PREFERENCES_LOAD_NOT_FOUND;
    }

    board_valid = board_config_is_valid();
    if (!board_valid) {
        ESP_LOGE(TAG, "board configuration validation failed");
    }
    log_hardware_configuration();

    external_power_present = system_io_external_power_present();
    if (board_valid) {
        system_io_result = system_io_init();
        if (system_io_result != ESP_OK) {
            ESP_LOGW(TAG, "battery ADC unavailable: %s",
                     esp_err_to_name(system_io_result));
        } else if (!external_power_present) {
            startup_battery_valid = startup_read_battery(
                &startup_battery_voltage_v);
        }
    }
    if (battery_power_startup_blocked(
            external_power_present, startup_battery_valid,
            startup_battery_voltage_v)) {
        ESP_LOGW(TAG,
                 "startup blocked by low battery: voltage=%.2f V threshold=%.2f V",
                 (double) startup_battery_voltage_v,
                 (double) BATTERY_STARTUP_MINIMUM_V);
        app_tasks_run_safe_stop();
    }

    if (startup_preparation_result.audio_result == ESP_OK) {
        startup_sound_result = app_tasks_play_startup_sound(
            switch_preferences.volume_level);
    } else {
        startup_sound_result =
            startup_preparation_result.audio_result;
    }

    nvs_result = recover_nvs_after_power_confirmation(
        startup_preparation_result.nvs_result);
    feed_startup_watchdog();
    if (nvs_result == ESP_OK) {
        nvs_ready = true;
        if (nvs_recovery_required(
                startup_preparation_result.nvs_result)) {
            switch_preferences_set_defaults(&switch_preferences);
            switch_load_result = SWITCH_PREFERENCES_LOAD_NOT_FOUND;
        }
    }
    if (nvs_ready && config_format_requested) {
        switch_clear_result = switch_preferences_clear();
        if (switch_clear_result != ESP_OK) {
            switch_preferences_dirty = true;
        }
    }

    if (config_format_requested) {
        ESP_LOGW(TAG, "SW3 startup request: config FAT will be formatted");
    }

    ret = app_power_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "power management entered safe fallback: %s", esp_err_to_name(ret));
    }

    storage_result =
        usb_device_storage_init(&startup_runtime_profiles,
                                config_format_requested,
                                report_startup_storage_progress, NULL);
    feed_startup_watchdog();
    if (storage_result != ESP_OK) {
        ESP_LOGW(TAG, "config FAT/MSC storage degraded: %s",
                 esp_err_to_name(storage_result));
    }

    if (storage_result == ESP_OK) {
        imu_calibration_storage_result_t calibration_result =
            usb_device_load_imu_calibration(
                &imu_accel_calibration, &imu_calibration_diagnostics);

        imu_accel_calibration_required =
            calibration_result != IMU_CALIBRATION_STORAGE_VALID &&
            calibration_result != IMU_CALIBRATION_STORAGE_RECOVERED;
        ESP_LOGI(TAG, "mc_data.json=%s",
                 imu_calibration_storage_result_name(calibration_result));
        feed_startup_watchdog();
    } else {
        imu_calibration_diagnostics.result =
            IMU_CALIBRATION_STORAGE_IO_ERROR;
        imu_calibration_diagnostics.io_error = (int32_t) storage_result;
        imu_accel_calibration_required = true;
    }
    app_tasks_set_imu_accel_calibration(&imu_accel_calibration,
                                        &imu_calibration_diagnostics);

    ret = firmware_update_process_boot(
        external_power_present, startup_battery_valid,
        startup_battery_voltage_v);
    feed_startup_watchdog();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE &&
        ret != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "boot firmware-update processing failed: %s",
                 esp_err_to_name(ret));
    }
    firmware_update_get_diagnostics(&update_diagnostics);
    update_confirmation_required =
        update_diagnostics.confirmation_required;
    usb_msc_gate_required =
        update_confirmation_required ||
        update_diagnostics.state == FIRMWARE_UPDATE_PENDING_CONFIRMATION ||
        imu_accel_calibration_required;
    ret = firmware_update_begin_confirmation();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA confirmation task unavailable: %s",
                 esp_err_to_name(ret));
        startup_gate_result = ret;
    }

    usb_result = usb_device_start();
    usb_composite_active = usb_result == ESP_OK;
    if (usb_result != ESP_OK) {
        ESP_LOGW(TAG, "early TinyUSB CDC diagnostics unavailable: %s",
                 esp_err_to_name(usb_result));
    } else if (usb_msc_gate_required) {
        ESP_LOGI(TAG,
                 "USB CDC started; MSC medium remains APP-owned until startup gates clear");
    }

    ret = app_resources_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTOS resource initialization failed: %s", esp_err_to_name(ret));
        app_tasks_run_fatal_fallback();
    }
    if (imu_accel_calibration_required) {
        EventGroupHandle_t event_group = app_resources_event_group();

        if (event_group != NULL) {
            (void) xEventGroupSetBits(
                event_group,
            APP_EVENT_IMU_ACCEL_CALIBRATION_REQUIRED);
        }
    }

    if (!nvs_ready) {
        ESP_LOGW(TAG, "NVS unavailable; BLE will remain disabled: %s",
                 esp_err_to_name(nvs_result));
        post_peripheral_failure(nvs_result);
    }
    if (switch_clear_result != ESP_OK) {
            ESP_LOGW(TAG, "switch preferences could not be cleared: %s",
                     esp_err_to_name(switch_clear_result));
            post_peripheral_failure(switch_clear_result);
    }
    if (nvs_ready && !config_format_requested) {
        if (switch_load_result != SWITCH_PREFERENCES_LOAD_OK &&
            switch_load_result != SWITCH_PREFERENCES_LOAD_LEGACY &&
            switch_load_result != SWITCH_PREFERENCES_LOAD_NOT_FOUND) {
            switch_preferences_diagnostics_t diagnostics = {0};

            switch_preferences_get_diagnostics(&diagnostics);
            ESP_LOGW(TAG, "switch preferences ignored: %s (%s)",
                     switch_preferences_load_result_name(switch_load_result),
                     esp_err_to_name(diagnostics.last_load_error));
            esp_err_t load_error = diagnostics.last_load_error;

            if (load_error == ESP_OK) {
                load_error = ESP_ERR_INVALID_RESPONSE;
            }
            post_peripheral_failure(load_error);
        }
        if (switch_load_result == SWITCH_PREFERENCES_LOAD_LEGACY) {
            switch_preferences_dirty = true;
        }
    }
    {
        size_t selected_index = 0U;

        if (!app_config_profiles_find(&startup_runtime_profiles,
                                      switch_preferences.parameter_number,
                                      &selected_index)) {
            switch_preferences.parameter_number =
                startup_runtime_profiles.profiles[0].parameter_number;
            switch_preferences_dirty = true;
        }
    }
    if (!app_resources_publish_config_profiles(
            &startup_runtime_profiles, switch_preferences.parameter_number,
            &switch_preferences.parameter_number)) {
        ESP_LOGW(TAG,
                 "runtime parameter profile collection rejected; defaults retained");
        app_config_profiles_set_defaults(&startup_runtime_profiles);
        switch_preferences.parameter_number = 1U;
        switch_preferences_dirty = true;
        (void) app_resources_publish_config_profiles(
            &startup_runtime_profiles, switch_preferences.parameter_number,
            &switch_preferences.parameter_number);
    }
    app_tasks_set_switch_preferences(&switch_preferences,
                                     switch_preferences_dirty);
    ESP_LOGI(TAG,
             "switch preferences=%s volume=%d sink=%d parameter=%u sets=%u",
             switch_preferences_load_result_name(switch_load_result),
             (int) switch_preferences.volume_level,
             switch_preferences.sink_enabled,
             (unsigned int) switch_preferences.parameter_number,
             (unsigned int) startup_runtime_profiles.count);

    if (storage_result != ESP_OK) {
        post_peripheral_failure(storage_result);
    }
    if (usb_result != ESP_OK && usb_result != storage_result) {
        post_peripheral_failure(usb_result);
    }
    if (startup_gate_result != ESP_OK &&
        startup_gate_result != storage_result &&
        startup_gate_result != usb_result) {
        post_peripheral_failure(startup_gate_result);
    }
    if (startup_sound_result != ESP_OK &&
        startup_sound_result != storage_result &&
        startup_sound_result != usb_result &&
        startup_sound_result != startup_gate_result) {
        ESP_LOGW(TAG, "startup sound unavailable: %s",
                 esp_err_to_name(startup_sound_result));
        post_peripheral_failure(startup_sound_result);
    }
    ret = sensor_bus_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "shared sensor I2C bus unavailable: %s",
                 esp_err_to_name(ret));
        post_peripheral_failure(ret);
    }

    if (board_valid && system_io_result != ESP_OK) {
        post_peripheral_failure(system_io_result);
    }

    ret = app_tasks_start();
    if (app_tasks_required_workers_started()) {
        firmware_update_mark_workers_started();
    }
    if (ret != ESP_OK) {
        if (!app_tasks_system_started()) {
            watchdog_service_mark_stage(WATCHDOG_STAGE_FATAL);
            app_tasks_run_fatal_fallback();
        }
        ESP_LOGE(TAG, "application startup did not complete: %s", esp_err_to_name(ret));
        (void) watchdog_service_unregister_current(WATCHDOG_ACTOR_STARTUP);
        return;
    }
    (void) watchdog_service_unregister_current(WATCHDOG_ACTOR_STARTUP);
    usb_device_set_storage_mode_callbacks(
        app_tasks_begin_storage_mode, app_tasks_end_storage_mode, NULL);

    if (usb_msc_gate_required && update_confirmation_required &&
        startup_gate_result == ESP_OK) {
        startup_gate_result = firmware_update_wait_for_confirmation(
            UPDATE_CONFIRMATION_USB_WAIT_MS);
        if (startup_gate_result != ESP_OK) {
            ESP_LOGE(TAG,
                     "MSC remains disabled until OTA cleanup succeeds: %s",
                     esp_err_to_name(startup_gate_result));
            post_peripheral_failure(startup_gate_result);
        }
    }

    if (nvs_ready) {
        if (!app_resources_copy_config(&ble_config)) {
            app_config_set_defaults(&ble_config);
        }
        ret = ble_vario_init(ble_config.bluetooth_tx_power);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "NimBLE initialization failed: %s", esp_err_to_name(ret));
            post_peripheral_failure(ret);
        }
    }

    if (usb_msc_gate_required && imu_accel_calibration_required &&
        startup_gate_result == ESP_OK) {
        EventGroupHandle_t event_group = app_resources_event_group();
        EventBits_t bits = APP_EVENT_FATAL_STATE;

        if (event_group != NULL) {
            bits = xEventGroupWaitBits(
                event_group,
                APP_EVENT_IMU_ACCEL_CALIBRATION_SAVED |
                    APP_EVENT_IMU_ACCEL_CALIBRATION_SKIPPED |
                    APP_EVENT_STOP_REQUEST | APP_EVENT_FATAL_STATE,
                pdFALSE, pdFALSE, portMAX_DELAY);
        }

        if ((bits & (APP_EVENT_IMU_ACCEL_CALIBRATION_SAVED |
                     APP_EVENT_IMU_ACCEL_CALIBRATION_SKIPPED)) == 0U) {
            startup_gate_result = ESP_ERR_INVALID_STATE;
            post_peripheral_failure(startup_gate_result);
        } else if ((bits & APP_EVENT_IMU_ACCEL_CALIBRATION_SKIPPED) != 0U) {
            ESP_LOGW(TAG,
                     "initial IMU calibration skipped; enabling MSC in pressure-only mode");
        }
    }

    if (usb_result == ESP_OK && startup_gate_result == ESP_OK &&
        usb_composite_active) {
        usb_result = usb_device_enable_msc();
        if (usb_result != ESP_OK) {
            ESP_LOGE(TAG, "USB MSC enable failed: %s",
                     esp_err_to_name(usb_result));
            post_peripheral_failure(usb_result);
        }
    }

    ESP_LOGI(TAG, "foundation initialization complete");
}
