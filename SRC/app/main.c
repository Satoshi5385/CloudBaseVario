#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app/app_resources.h"
#include "app/app_tasks.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "platform/app_power.h"
#include "platform/audio_output.h"
#include "platform/ble_vario.h"
#include "platform/board.h"
#include "platform/sensor_bus.h"
#include "platform/system_io.h"

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

#if !CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#error "CloudBaseVario requires the USB Serial/JTAG primary console"
#endif

#if CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ != 80
#error "CloudBaseVario requires an 80 MHz default and maximum CPU frequency"
#endif

#if !CONFIG_PM_ENABLE || !CONFIG_FREERTOS_USE_TICKLESS_IDLE
#error "CloudBaseVario requires DFS power management and tickless idle"
#endif

#define EXPECTED_FLASH_SIZE_BYTES UINT32_C(16777216)
#define EXPECTED_PSRAM_SIZE_BYTES UINT32_C(8388608)

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

void app_main(void) {
    esp_err_t ret = board_init_power_hold();
    bool board_valid = false;
    bool nvs_ready = false;

    if (ret != ESP_OK) {
        app_tasks_run_fatal_fallback();
    }

    ret = board_init_safe_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "safe GPIO initialization failed: %s", esp_err_to_name(ret));
        app_tasks_run_fatal_fallback();
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

    ESP_LOGI(TAG, "USB Serial/JTAG primary console is configured");

    ret = sensor_bus_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "shared I2C bus unavailable: %s", esp_err_to_name(ret));
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
    if (ret != ESP_OK) {
        if (!app_tasks_system_started()) {
            app_tasks_run_fatal_fallback();
        }
        return;
    }

    if (nvs_ready) {
        ret = ble_vario_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "NimBLE initialization failed: %s", esp_err_to_name(ret));
            post_peripheral_failure(ret);
        }
    }

    ESP_LOGI(TAG, "foundation initialization complete");
}
