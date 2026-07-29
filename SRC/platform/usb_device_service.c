#include "platform/usb_device_service.h"

#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_partition.h"
#include "esp_vfs_fat.h"
#include "ff.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "platform/board.h"
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"
#include "tinyusb_default_config.h"
#include "tinyusb_msc.h"
#include "tusb.h"
#include "wear_levelling.h"

#define CONFIG_PARTITION_LABEL "config"
#define CONFIG_MOUNT_PATH "/config"
#define CONFIG_VOLUME_LABEL "CBVARIO"
#define STORAGE_MUTEX_TIMEOUT_MS UINT32_C(100)
#define USB_SERIAL_NUMBER_LENGTH 13U
#define USB_CONFIG_TOTAL_LENGTH \
    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN)

static const char *TAG = "usb_device";
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t storage_io_mutex;
static bool storage_transition_locked;
static wl_handle_t wear_levelling_handle = WL_INVALID_HANDLE;
static tinyusb_msc_storage_handle_t msc_storage;
static char serial_number[USB_SERIAL_NUMBER_LENGTH];
static const char *usb_strings[] = {
    (const char[]) {0x09, 0x04},
    "CloudBaseVario",
    "CloudBaseVario CDC+MSC",
    serial_number,
    "CloudBaseVario CDC",
    "CloudBaseVario MSC",
};

static usb_device_diagnostics_t usb_diagnostics = {
    .storage_owner = USB_STORAGE_UNAVAILABLE,
    .load_result = CONFIG_LOAD_IO_ERROR,
    .config = {
        .source = CONFIG_SOURCE_BUILTIN_DEFAULT,
        .validation = CONFIG_VALIDATION_IO_ERROR,
        .format_version = 1,
    },
    .last_storage_error = ESP_ERR_INVALID_STATE,
    .last_save_result = ESP_ERR_INVALID_STATE,
};

static const tusb_desc_device_t usb_device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = TINYUSB_ESPRESSIF_VID,
    .idProduct = 0x4003,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const uint8_t usb_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 3, 0, USB_CONFIG_TOTAL_LENGTH,
                          TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),
    TUD_CDC_DESCRIPTOR(0, 4, 0x81, 8, 0x02, 0x82, 64),
    TUD_MSC_DESCRIPTOR(2, 5, 0x03, 0x83, 64),
};

static void increment_counter(uint32_t *counter) {
    portENTER_CRITICAL(&state_lock);
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
    portEXIT_CRITICAL(&state_lock);
}

static void set_storage_unavailable(esp_err_t error) {
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.storage_ready = false;
    usb_diagnostics.msc_media_ready = false;
    usb_diagnostics.storage_owner = USB_STORAGE_UNAVAILABLE;
    usb_diagnostics.load_result = CONFIG_LOAD_IO_ERROR;
    usb_diagnostics.config.source = CONFIG_SOURCE_BUILTIN_DEFAULT;
    usb_diagnostics.config.validation = CONFIG_VALIDATION_IO_ERROR;
    usb_diagnostics.config.io_error = (int32_t) error;
    usb_diagnostics.config.key[0] = '\0';
    usb_diagnostics.last_storage_error = error;
    if (usb_diagnostics.mount_failure_count < UINT32_MAX) {
        usb_diagnostics.mount_failure_count++;
    }
    portEXIT_CRITICAL(&state_lock);
}

static void log_config_load_result(void) {
    if (usb_diagnostics.load_result == CONFIG_LOAD_INVALID_FILE) {
        ESP_LOGW(TAG,
                 "parameters.json invalid: reason=%s key=%s version=%" PRId32,
                 config_storage_validation_name(
                     usb_diagnostics.config.validation),
                 usb_diagnostics.config.key[0] == '\0'
                     ? "-"
                     : usb_diagnostics.config.key,
                 usb_diagnostics.config.format_version);
    } else if (usb_diagnostics.load_result == CONFIG_LOAD_IO_ERROR) {
        ESP_LOGW(TAG, "parameters.json read failed: reason=%s io_error=%" PRId32,
                 config_storage_validation_name(
                     usb_diagnostics.config.validation),
                 usb_diagnostics.config.io_error);
    } else if (usb_diagnostics.load_result == CONFIG_LOAD_RECOVERED_FILE) {
        ESP_LOGW(TAG, "parameters.json recovered from verified backup");
    }
}

static void make_serial_number(void) {
    uint8_t mac[6] = {0};

    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        (void) snprintf(serial_number, sizeof(serial_number), "000000000000");
        return;
    }
    (void) snprintf(serial_number, sizeof(serial_number),
                    "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2],
                    mac[3], mac[4], mac[5]);
}

static void tinyusb_device_event(tinyusb_event_t *event, void *arg) {
    (void) arg;
    if (event == NULL) {
        return;
    }

    portENTER_CRITICAL(&state_lock);
    if (event->id == TINYUSB_EVENT_ATTACHED) {
        usb_diagnostics.device_attached = true;
        if (usb_diagnostics.attach_count < UINT32_MAX) {
            usb_diagnostics.attach_count++;
        }
    } else if (event->id == TINYUSB_EVENT_DETACHED) {
        usb_diagnostics.device_attached = false;
        usb_diagnostics.cdc_connected = false;
        if (usb_diagnostics.detach_count < UINT32_MAX) {
            usb_diagnostics.detach_count++;
        }
    }
    portEXIT_CRITICAL(&state_lock);
}

static void cdc_line_state_changed(int itf, cdcacm_event_t *event) {
    (void) itf;
    if (event == NULL || event->type != CDC_EVENT_LINE_STATE_CHANGED) {
        return;
    }
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.cdc_connected =
        event->line_state_changed_data.dtr;
    portEXIT_CRITICAL(&state_lock);
}

static void msc_storage_event(tinyusb_msc_storage_handle_t handle,
                              tinyusb_msc_event_t *event, void *arg) {
    (void) handle;
    (void) arg;
    if (event == NULL || storage_io_mutex == NULL) {
        return;
    }

    if (event->id == TINYUSB_MSC_EVENT_MOUNT_START) {
        (void) xSemaphoreTake(storage_io_mutex, portMAX_DELAY);
        storage_transition_locked = true;
        portENTER_CRITICAL(&state_lock);
        usb_diagnostics.storage_owner = USB_STORAGE_SWITCHING;
        portEXIT_CRITICAL(&state_lock);
        return;
    }

    portENTER_CRITICAL(&state_lock);
    if (event->id == TINYUSB_MSC_EVENT_MOUNT_COMPLETE) {
        usb_diagnostics.storage_ready = true;
        usb_diagnostics.msc_media_ready = true;
        usb_diagnostics.last_storage_error = ESP_OK;
        usb_diagnostics.storage_owner =
            event->mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP
                ? USB_STORAGE_APP_OWNED
                : USB_STORAGE_HOST_OWNED;
    } else {
        usb_diagnostics.storage_ready = false;
        usb_diagnostics.msc_media_ready = false;
        usb_diagnostics.storage_owner = USB_STORAGE_UNAVAILABLE;
        usb_diagnostics.last_storage_error = ESP_FAIL;
        if (usb_diagnostics.mount_failure_count < UINT32_MAX) {
            usb_diagnostics.mount_failure_count++;
        }
        if (event->id == TINYUSB_MSC_EVENT_FORMAT_REQUIRED &&
            usb_diagnostics.format_required_count < UINT32_MAX) {
            usb_diagnostics.format_required_count++;
        }
    }
    portEXIT_CRITICAL(&state_lock);

    if (storage_transition_locked) {
        storage_transition_locked = false;
        (void) xSemaphoreGive(storage_io_mutex);
    }
}

esp_err_t usb_device_storage_init(app_config_t *config,
                                  bool format_config_storage) {
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 6,
        .allocation_unit_size = 4096,
        .use_one_fat = false,
    };
    const esp_partition_t *partition;
    wl_handle_t preflight_handle = WL_INVALID_HANDLE;
    bool media_ready;
    esp_err_t storage_error;
    esp_err_t ret;

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    app_config_set_defaults(config);

    storage_io_mutex = xSemaphoreCreateMutex();
    if (storage_io_mutex == NULL) {
        set_storage_unavailable(ESP_ERR_NO_MEM);
        return ESP_ERR_NO_MEM;
    }

    const tinyusb_msc_driver_config_t driver_config = {
        .user_flags = {.val = 0},
        .callback = msc_storage_event,
        .callback_arg = NULL,
    };
    ret = tinyusb_msc_install_driver(&driver_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MSC driver initialization failed: %s",
                 esp_err_to_name(ret));
        set_storage_unavailable(ret);
        return ret;
    }
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.msc_driver_ready = true;
    portEXIT_CRITICAL(&state_lock);

    if (format_config_storage) {
        esp_vfs_fat_mount_config_t format_config = mount_config;
        format_config.format_if_mount_failed = true;
        ret = esp_vfs_fat_spiflash_format_cfg_rw_wl(
            CONFIG_MOUNT_PATH, CONFIG_PARTITION_LABEL, &format_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "explicit config FAT format failed: %s",
                     esp_err_to_name(ret));
            set_storage_unavailable(ret);
            return ret;
        }
        ESP_LOGW(TAG, "config FAT formatted by SW2+SW3 startup request");
    }

    ret = esp_vfs_fat_spiflash_mount_rw_wl(
        CONFIG_MOUNT_PATH, CONFIG_PARTITION_LABEL, &mount_config,
        &preflight_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "config FAT unavailable; automatic format prohibited: %s",
                 esp_err_to_name(ret));
        set_storage_unavailable(ret);
        if (ret == ESP_ERR_NOT_FOUND) {
            increment_counter(&usb_diagnostics.format_required_count);
        }
        return ret;
    }

    usb_diagnostics.load_result =
        config_storage_load(CONFIG_MOUNT_PATH, config, &usb_diagnostics.config);
    if (usb_diagnostics.load_result == CONFIG_LOAD_DEFAULT_NO_FILE) {
        ret = config_storage_save(CONFIG_MOUNT_PATH, config);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "default parameters.json generation failed: %s",
                     esp_err_to_name(ret));
        }
    } else {
        log_config_load_result();
    }
    if (f_setlabel(CONFIG_VOLUME_LABEL) != FR_OK) {
        ESP_LOGW(TAG, "FAT volume label could not be set");
    }

    ret = esp_vfs_fat_spiflash_unmount_rw_wl(CONFIG_MOUNT_PATH,
                                              preflight_handle);
    if (ret != ESP_OK) {
        set_storage_unavailable(ret);
        return ret;
    }

    partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT,
        CONFIG_PARTITION_LABEL);
    if (partition == NULL) {
        set_storage_unavailable(ESP_ERR_NOT_FOUND);
        return ESP_ERR_NOT_FOUND;
    }
    ret = wl_mount(partition, &wear_levelling_handle);
    if (ret != ESP_OK) {
        set_storage_unavailable(ret);
        return ret;
    }

    tinyusb_msc_storage_config_t storage_config = {
        .medium = {.wl_handle = wear_levelling_handle},
        .fat_fs = {
            .base_path = CONFIG_MOUNT_PATH,
            .config = {
                .format_if_mount_failed = false,
                .max_files = 6,
                .allocation_unit_size = 4096,
                .use_one_fat = false,
            },
            .do_not_format = true,
            .format_flags = FM_ANY,
        },
        /*
         * Create the LUN without mounting it first.  esp_tinyusb 2.2.1 does
         * not unmap a LUN if its initial APP mount fails, so mounting in a
         * second step keeps every storage-creation failure at zero LUNs.
         */
        .mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB,
    };

    ret = tinyusb_msc_new_storage_spiflash(&storage_config, &msc_storage);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MSC storage initialization failed: %s",
                 esp_err_to_name(ret));
        if (wear_levelling_handle != WL_INVALID_HANDLE) {
            (void) wl_unmount(wear_levelling_handle);
            wear_levelling_handle = WL_INVALID_HANDLE;
        }
        set_storage_unavailable(ret);
        return ret;
    }
    ret = tinyusb_msc_set_storage_mount_point(
        msc_storage, TINYUSB_MSC_STORAGE_MOUNT_APP);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MSC application mount request failed: %s",
                 esp_err_to_name(ret));
        set_storage_unavailable(ret);
        return ret;
    }
    portENTER_CRITICAL(&state_lock);
    media_ready = usb_diagnostics.msc_media_ready;
    storage_error = usb_diagnostics.last_storage_error;
    portEXIT_CRITICAL(&state_lock);
    if (!media_ready) {
        ESP_LOGW(TAG, "MSC application mount failed: %s",
                 esp_err_to_name(storage_error));
        return storage_error == ESP_OK ? ESP_FAIL : storage_error;
    }
    return ESP_OK;
}

esp_err_t usb_device_start(void) {
    esp_err_t first_error = ESP_OK;
    esp_err_t ret;
    bool msc_driver_ready;
    bool media_ready;
    esp_err_t storage_error;

    portENTER_CRITICAL(&state_lock);
    msc_driver_ready = usb_diagnostics.msc_driver_ready;
    portEXIT_CRITICAL(&state_lock);
    if (!msc_driver_ready) {
        ESP_LOGW(TAG,
                 "TinyUSB composite not started because MSC driver is unavailable");
        return ESP_ERR_INVALID_STATE;
    }

    make_serial_number();
    tinyusb_config_t tinyusb_config =
        TINYUSB_DEFAULT_CONFIG(tinyusb_device_event, NULL);
    tinyusb_config.phy.self_powered = true;
    tinyusb_config.phy.vbus_monitor_io = PIN_PWR_EXT;
    tinyusb_config.task = TINYUSB_TASK_CUSTOM(4096, 6, 0);
    tinyusb_config.descriptor.device = &usb_device_descriptor;
    tinyusb_config.descriptor.string = usb_strings;
    tinyusb_config.descriptor.string_count =
        sizeof(usb_strings) / sizeof(usb_strings[0]);
    tinyusb_config.descriptor.full_speed_config =
        usb_configuration_descriptor;

    ret = tinyusb_driver_install(&tinyusb_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TinyUSB driver initialization failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.driver_ready = true;
    portEXIT_CRITICAL(&state_lock);

    const tinyusb_config_cdcacm_t cdc_config = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = NULL,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = cdc_line_state_changed,
        .callback_line_coding_changed = NULL,
    };
    ret = tinyusb_cdcacm_init(&cdc_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TinyUSB CDC initialization failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.cdc_ready = true;
    portEXIT_CRITICAL(&state_lock);

    ret = tinyusb_console_init(TINYUSB_CDC_ACM_0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TinyUSB log console redirect failed: %s",
                 esp_err_to_name(ret));
        if (first_error == ESP_OK) {
            first_error = ret;
        }
    }
    ESP_LOGI(TAG, "TinyUSB CDC+MSC composite device initialized");
    portENTER_CRITICAL(&state_lock);
    media_ready = usb_diagnostics.msc_media_ready;
    storage_error = usb_diagnostics.last_storage_error;
    portEXIT_CRITICAL(&state_lock);
    if (!media_ready) {
        ESP_LOGW(TAG,
                 "MSC media unavailable (%s); CDC and vario operation continue",
                 esp_err_to_name(storage_error));
    }
    return first_error;
}

esp_err_t usb_device_storage_begin_app_io(uint32_t timeout_ms) {
    usb_storage_owner_t owner;

    if (storage_io_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&state_lock);
    owner = usb_diagnostics.storage_owner;
    portEXIT_CRITICAL(&state_lock);
    if (owner != USB_STORAGE_APP_OWNED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(storage_io_mutex, pdMS_TO_TICKS(timeout_ms)) !=
        pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    portENTER_CRITICAL(&state_lock);
    owner = usb_diagnostics.storage_owner;
    portEXIT_CRITICAL(&state_lock);
    if (owner != USB_STORAGE_APP_OWNED) {
        (void) xSemaphoreGive(storage_io_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

void usb_device_storage_end_app_io(void) {
    if (storage_io_mutex != NULL) {
        (void) xSemaphoreGive(storage_io_mutex);
    }
}

esp_err_t usb_device_save_config(const app_config_t *config) {
    esp_err_t ret;

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ret = usb_device_storage_begin_app_io(STORAGE_MUTEX_TIMEOUT_MS);
    if (ret == ESP_OK) {
        ret = config_storage_save(CONFIG_MOUNT_PATH, config);
        usb_device_storage_end_app_io();
    }
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.last_save_result = ret;
    portEXIT_CRITICAL(&state_lock);
    return ret;
}

bool usb_device_read(uint8_t *buffer, size_t capacity, size_t *length) {
    esp_err_t ret;

    if (buffer == NULL || length == NULL || capacity == 0U) {
        return false;
    }
    *length = 0U;
    if (!usb_device_cdc_connected()) {
        return false;
    }
    ret = tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, buffer, capacity, length);
    if (ret != ESP_OK) {
        increment_counter(&usb_diagnostics.rx_error_count);
        return false;
    }
    return *length > 0U;
}

bool usb_device_write(const char *text) {
    size_t length;
    size_t queued;

    if (text == NULL || !usb_device_cdc_connected()) {
        return false;
    }
    length = strlen(text);
    queued = tinyusb_cdcacm_write_queue(
        TINYUSB_CDC_ACM_0, (const uint8_t *) text, length);
    esp_err_t flush_result =
        tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
    if (queued != length ||
        (flush_result != ESP_OK &&
         flush_result != ESP_ERR_NOT_FINISHED)) {
        increment_counter(&usb_diagnostics.tx_error_count);
        return false;
    }
    return true;
}

bool usb_device_cdc_connected(void) {
    bool connected;
    portENTER_CRITICAL(&state_lock);
    connected = usb_diagnostics.cdc_ready &&
                usb_diagnostics.cdc_connected;
    portEXIT_CRITICAL(&state_lock);
    return connected;
}

bool usb_device_bus_active(void) {
    bool attached;
    portENTER_CRITICAL(&state_lock);
    attached = usb_diagnostics.device_attached;
    portEXIT_CRITICAL(&state_lock);
    return attached || gpio_get_level(PIN_PWR_EXT) != 0;
}

const char *usb_device_storage_mount_path(void) {
    return CONFIG_MOUNT_PATH;
}

void usb_device_get_diagnostics(usb_device_diagnostics_t *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.vbus_present = gpio_get_level(PIN_PWR_EXT) != 0;
    *diagnostics = usb_diagnostics;
    portEXIT_CRITICAL(&state_lock);
}

const char *usb_device_storage_owner_name(usb_storage_owner_t owner) {
    switch (owner) {
        case USB_STORAGE_APP_OWNED:
            return "APP";
        case USB_STORAGE_SWITCHING:
            return "SWITCHING";
        case USB_STORAGE_HOST_OWNED:
            return "HOST";
        case USB_STORAGE_UNAVAILABLE:
        default:
            return "UNAVAILABLE";
    }
}
