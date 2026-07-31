#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app/app_resources.h"
#include "app/app_tasks.h"
#include "domain/app_config.h"
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
#include "platform/firmware_update.h"
#include "platform/sensor_bus.h"
#include "platform/system_io.h"
#include "platform/usb_device_service.h"

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

#if CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ != 80
#error "CloudBaseVario requires an 80 MHz default and maximum CPU frequency"
#endif

#if !CONFIG_PM_ENABLE || !CONFIG_FREERTOS_USE_TICKLESS_IDLE
#error "CloudBaseVario requires DFS power management and tickless idle"
#endif

#if !CONFIG_WL_SECTOR_SIZE_512 || !CONFIG_WL_SECTOR_MODE_SAFE
#error "CloudBaseVario requires 512-byte wear-levelling sectors in Safety mode"
#endif

#define EXPECTED_FLASH_SIZE_BYTES UINT32_C(16777216)
#define EXPECTED_PSRAM_SIZE_BYTES UINT32_C(8388608)
#define STARTUP_FORMAT_SAMPLE_PERIOD_MS UINT32_C(10)
#define STARTUP_FORMAT_DEBOUNCE_MS UINT32_C(30)
#define UPDATE_CONFIRMATION_USB_WAIT_MS UINT32_C(15000)

static const char *TAG = "main";

static esp_err_t initialize_nvs(void) {
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS requires one-time erase and recovery");
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            return ret;
        }
        ret = nvs_flash_init();
    }

    return ret;
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

static bool startup_config_format_requested(void) {
    uint32_t stable_time_ms = 0U;

    for (;;) {
        if (!system_io_sw2_pressed() || !system_io_sw3_pressed()) {
            return false;
        }
        if (stable_time_ms >= STARTUP_FORMAT_DEBOUNCE_MS) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(STARTUP_FORMAT_SAMPLE_PERIOD_MS));
        stable_time_ms += STARTUP_FORMAT_SAMPLE_PERIOD_MS;
    }
}

void app_main(void) {
    esp_err_t ret = board_init_power_hold();
    app_config_t runtime_config = {0};
    bool board_valid = false;
    bool config_format_requested = false;
    bool nvs_ready = false;
    bool update_confirmation_required = false;
    bool usb_start_deferred = false;
    bool imu_accel_calibration_required = false;
    esp_err_t storage_result = ESP_OK;
    esp_err_t usb_result = ESP_OK;
    firmware_update_diagnostics_t update_diagnostics = {0};
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

    config_format_requested = startup_config_format_requested();
    if (config_format_requested) {
        ESP_LOGW(TAG, "SW2+SW3 startup request: config FAT will be formatted");
    }

    board_valid = board_config_is_valid();
    if (!board_valid) {
        ESP_LOGE(TAG, "board configuration validation failed");
    }

    log_hardware_configuration();

    ret = app_power_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "power management entered safe fallback: %s", esp_err_to_name(ret));
    }

    storage_result =
        usb_device_storage_init(&runtime_config, config_format_requested);
    if (storage_result != ESP_OK) {
        ESP_LOGW(TAG, "config FAT/MSC storage degraded: %s",
                 esp_err_to_name(storage_result));
    }

#if CONFIG_CBV_IMU_HXY_ENABLE
    if (storage_result == ESP_OK) {
        imu_calibration_storage_result_t calibration_result =
            usb_device_load_imu_calibration(
                &imu_accel_calibration, &imu_calibration_diagnostics);

        imu_accel_calibration_required =
            calibration_result != IMU_CALIBRATION_STORAGE_VALID &&
            calibration_result != IMU_CALIBRATION_STORAGE_RECOVERED;
        ESP_LOGI(TAG, "mc_data.json=%s",
                 imu_calibration_storage_result_name(calibration_result));
    } else {
        imu_calibration_diagnostics.result =
            IMU_CALIBRATION_STORAGE_IO_ERROR;
        imu_calibration_diagnostics.io_error = (int32_t) storage_result;
        imu_accel_calibration_required = true;
    }
    app_tasks_set_imu_accel_calibration(&imu_accel_calibration,
                                        &imu_calibration_diagnostics);
#endif

    ret = firmware_update_process_boot(system_io_external_power_present());
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE &&
        ret != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "boot firmware-update processing failed: %s",
                 esp_err_to_name(ret));
    }
    firmware_update_get_diagnostics(&update_diagnostics);
    update_confirmation_required =
        update_diagnostics.confirmation_required;
    usb_start_deferred =
        update_confirmation_required ||
        update_diagnostics.state == FIRMWARE_UPDATE_PENDING_CONFIRMATION ||
        imu_accel_calibration_required;
    ret = firmware_update_begin_confirmation();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "OTA confirmation task unavailable: %s",
                 esp_err_to_name(ret));
        usb_result = ret;
    }

    if (!usb_start_deferred) {
        usb_result = usb_device_start();
        if (usb_result != ESP_OK) {
            ESP_LOGW(TAG, "TinyUSB CDC+MSC degraded: %s",
                     esp_err_to_name(usb_result));
        }
    } else {
        ESP_LOGI(TAG,
                 "TinyUSB start deferred until OTA and IMU calibration gates clear");
    }

    ret = app_resources_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTOS resource initialization failed: %s", esp_err_to_name(ret));
        app_tasks_run_fatal_fallback();
    }

    ret = initialize_nvs();
    if (ret == ESP_OK) {
        nvs_ready = true;
    } else {
        ESP_LOGW(TAG, "NVS unavailable; BLE will remain disabled: %s", esp_err_to_name(ret));
        post_peripheral_failure(ret);
    }

    if (storage_result != ESP_OK) {
        post_peripheral_failure(storage_result);
    }
    if (usb_result != ESP_OK && usb_result != storage_result) {
        post_peripheral_failure(usb_result);
    }
    if (!app_resources_publish_config(&runtime_config)) {
        ESP_LOGW(TAG, "runtime parameter configuration rejected; defaults retained");
    }

    ret = sensor_bus_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "shared sensor I2C bus unavailable: %s",
                 esp_err_to_name(ret));
        post_peripheral_failure(ret);
    }

    ret = audio_output_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "audio output unavailable: %s", esp_err_to_name(ret));
        board_set_safe_indicators();
        post_peripheral_failure(ret);
    }

    if (board_valid) {
        ret = system_io_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "battery ADC unavailable: %s", esp_err_to_name(ret));
            post_peripheral_failure(ret);
        }
    }

    ret = app_tasks_start();
    if (app_tasks_required_workers_started()) {
        firmware_update_mark_workers_started();
    }
    if (ret != ESP_OK) {
        if (!app_tasks_system_started()) {
            app_tasks_run_fatal_fallback();
        }
        ESP_LOGE(TAG, "application startup did not complete: %s", esp_err_to_name(ret));
        return;
    }

    if (usb_start_deferred && update_confirmation_required &&
        usb_result == ESP_OK) {
        usb_result = firmware_update_wait_for_confirmation(
            UPDATE_CONFIRMATION_USB_WAIT_MS);
        if (usb_result != ESP_OK) {
            ESP_LOGE(TAG,
                     "TinyUSB remains disabled until OTA cleanup succeeds: %s",
                     esp_err_to_name(usb_result));
            post_peripheral_failure(usb_result);
        }
    }

    if (nvs_ready) {
        ret = ble_vario_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "NimBLE initialization failed: %s", esp_err_to_name(ret));
            post_peripheral_failure(ret);
        }
    }

#if CONFIG_CBV_IMU_HXY_ENABLE
    if (usb_start_deferred && imu_accel_calibration_required &&
        usb_result == ESP_OK) {
        EventGroupHandle_t event_group = app_resources_event_group();
        EventBits_t bits = event_group == NULL
                               ? APP_EVENT_FATAL_STATE
                               : xEventGroupWaitBits(
                                     event_group,
                                     APP_EVENT_IMU_ACCEL_CALIBRATION_SAVED |
                                         APP_EVENT_STOP_REQUEST |
                                         APP_EVENT_FATAL_STATE,
                                     pdFALSE, pdFALSE, portMAX_DELAY);

        if ((bits & APP_EVENT_IMU_ACCEL_CALIBRATION_SAVED) == 0U) {
            usb_result = ESP_ERR_INVALID_STATE;
        }
    }
#endif

    if (usb_start_deferred && usb_result == ESP_OK) {
        usb_result = usb_device_start();
        if (usb_result != ESP_OK) {
            ESP_LOGE(TAG, "TinyUSB deferred start failed: %s",
                     esp_err_to_name(usb_result));
            post_peripheral_failure(usb_result);
        }
    }

    ESP_LOGI(TAG, "foundation initialization complete");
}
