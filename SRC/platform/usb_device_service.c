#include "platform/usb_device_service.h"

#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "diskio_wl.h"
#include "domain/board_info.h"
#include "domain/firmware_metadata.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "ff.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "platform/board.h"
#include "platform/imu_calibration_storage.h"
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
#define INFO_FILENAME "INFO.TXT"
#define INFO_PATH_CAPACITY 16U
#define STORAGE_MUTEX_TIMEOUT_MS UINT32_C(100)
#define STORAGE_MODE_QUIESCE_TIMEOUT_MS UINT32_C(100)
#define STORAGE_MODE_IDLE_US INT64_C(1000000)
#define USB_SERIAL_NUMBER_LENGTH BOARD_SERIAL_BUFFER_SIZE
#define USB_CONFIG_TOTAL_LENGTH \
    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN)

static const char *TAG = "usb_device";
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t storage_io_mutex;
static SemaphoreHandle_t msc_policy_mutex;
static bool storage_transition_locked;
static bool msc_exposure_enabled;
static bool usb_stopping;
static bool console_redirect_ready;
static wl_handle_t wear_levelling_handle = WL_INVALID_HANDLE;
static tinyusb_msc_storage_handle_t msc_storage;
static esp_timer_handle_t storage_mode_idle_timer;
static usb_storage_mode_begin_cb_t storage_mode_begin_cb;
static usb_storage_mode_end_cb_t storage_mode_end_cb;
static void *storage_mode_callback_arg;
static int64_t current_write_started_us;
static int64_t last_write_completed_us;
static bool storage_mode_force_exit;
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
        .format_version = CONFIG_FORMAT_VERSION,
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

static void storage_mode_finish_if_idle(void) {
    usb_storage_mode_end_cb_t end_cb = NULL;
    void *callback_arg = NULL;

    portENTER_CRITICAL(&state_lock);
    if (usb_storage_policy_write_session_forced_exit(
            usb_diagnostics.storage_mode_active,
            usb_diagnostics.pending_write_count)) {
        usb_diagnostics.storage_mode_active = false;
        storage_mode_force_exit = false;
        if (usb_diagnostics.storage_mode_end_count < UINT32_MAX) {
            usb_diagnostics.storage_mode_end_count++;
        }
        end_cb = storage_mode_end_cb;
        callback_arg = storage_mode_callback_arg;
    }
    portEXIT_CRITICAL(&state_lock);
    if (end_cb != NULL) {
        end_cb(callback_arg);
    }
}

static void storage_mode_request_forced_exit(void) {
    portENTER_CRITICAL(&state_lock);
    storage_mode_force_exit = true;
    portEXIT_CRITICAL(&state_lock);
    storage_mode_finish_if_idle();
}

static void storage_mode_idle_timer_cb(void *arg) {
    bool idle = false;
    int64_t now_us = esp_timer_get_time();

    (void) arg;
    portENTER_CRITICAL(&state_lock);
    idle = usb_storage_policy_write_session_idle(
        usb_diagnostics.storage_mode_active,
        usb_diagnostics.pending_write_count,
        now_us - last_write_completed_us, STORAGE_MODE_IDLE_US);
    portEXIT_CRITICAL(&state_lock);
    if (idle) {
        storage_mode_finish_if_idle();
    }
}

static void msc_write_event(tinyusb_msc_storage_handle_t handle,
                            const tinyusb_msc_write_event_t *event,
                            void *arg) {
    bool start_mode = false;
    bool quiesced = true;
    int64_t now_us = esp_timer_get_time();
    uint32_t duration_us = 0U;

    (void) handle;
    (void) arg;
    if (event == NULL) {
        return;
    }
    if (event->id == TINYUSB_MSC_WRITE_EVENT_BEGIN) {
        if (storage_mode_idle_timer != NULL) {
            (void) esp_timer_stop(storage_mode_idle_timer);
        }
        portENTER_CRITICAL(&state_lock);
        start_mode = !usb_diagnostics.storage_mode_active;
        usb_diagnostics.storage_mode_active = true;
        storage_mode_force_exit = false;
        usb_diagnostics.pending_write_count = event->pending_count;
        current_write_started_us = now_us;
        if (start_mode &&
            usb_diagnostics.storage_mode_start_count < UINT32_MAX) {
            usb_diagnostics.storage_mode_start_count++;
        }
        portEXIT_CRITICAL(&state_lock);
        if (start_mode && storage_mode_begin_cb != NULL) {
            quiesced = storage_mode_begin_cb(
                STORAGE_MODE_QUIESCE_TIMEOUT_MS,
                storage_mode_callback_arg);
            if (!quiesced) {
                portENTER_CRITICAL(&state_lock);
                if (usb_diagnostics.storage_mode_quiesce_timeout_count <
                    UINT32_MAX) {
                    usb_diagnostics.storage_mode_quiesce_timeout_count++;
                }
                portEXIT_CRITICAL(&state_lock);
            }
        }
        return;
    }

    if (now_us > current_write_started_us) {
        int64_t measured_us = now_us - current_write_started_us;
        duration_us = UINT32_MAX;
        if (measured_us <= (int64_t) UINT32_MAX) {
            duration_us = (uint32_t) measured_us;
        }
    }
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.pending_write_count = event->pending_count;
    usb_diagnostics.last_msc_write_duration_us = duration_us;
    if (duration_us > usb_diagnostics.max_msc_write_duration_us) {
        usb_diagnostics.max_msc_write_duration_us = duration_us;
    }
    if (usb_diagnostics.msc_write_count < UINT32_MAX) {
        usb_diagnostics.msc_write_count++;
    }
    if (event->result == ESP_OK) {
        if (UINT64_MAX - usb_diagnostics.msc_written_bytes < event->size) {
            usb_diagnostics.msc_written_bytes = UINT64_MAX;
        } else {
            usb_diagnostics.msc_written_bytes += event->size;
        }
    } else if (usb_diagnostics.msc_write_error_count < UINT32_MAX) {
        usb_diagnostics.msc_write_error_count++;
    }
    last_write_completed_us = now_us;
    portEXIT_CRITICAL(&state_lock);
    if (event->pending_count == 0U) {
        bool force_exit = false;

        portENTER_CRITICAL(&state_lock);
        force_exit = storage_mode_force_exit;
        portEXIT_CRITICAL(&state_lock);
        if (force_exit) {
            storage_mode_finish_if_idle();
        } else if (storage_mode_idle_timer != NULL) {
            (void) esp_timer_start_once(storage_mode_idle_timer,
                                        STORAGE_MODE_IDLE_US);
        }
    }
}

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
        const char *key = usb_diagnostics.config.key;

        if (key[0] == '\0') {
            key = "-";
        }
        ESP_LOGW(TAG,
                 "setting.json invalid: reason=%s key=%s version=%" PRId32,
                 config_storage_validation_name(
                     usb_diagnostics.config.validation),
                 key,
                 usb_diagnostics.config.format_version);
    } else if (usb_diagnostics.load_result == CONFIG_LOAD_IO_ERROR) {
        ESP_LOGW(TAG, "setting.json read failed: reason=%s io_error=%" PRId32,
                 config_storage_validation_name(
                     usb_diagnostics.config.validation),
                 usb_diagnostics.config.io_error);
    } else if (usb_diagnostics.load_result == CONFIG_LOAD_RECOVERED_FILE) {
        ESP_LOGW(TAG, "setting.json recovered from verified backup");
    }
}

static esp_err_t write_info_file(wl_handle_t wl_handle) {
    const board_identity_t *identity = board_active_identity();
    const board_descriptor_t *descriptor = board_active_descriptor();
    const esp_app_desc_t *app = esp_app_get_description();
    firmware_metadata_t firmware = {0};
    char contents[BOARD_INFO_TEXT_CAPACITY] = {0};
    char fat_path[INFO_PATH_CAPACITY] = {0};
    BYTE pdrv;
    FRESULT attribute_result;
    FILE *file;
    size_t content_length;
    int path_length;
    bool write_succeeded;

    if (app != NULL) {
        (void) firmware_metadata_parse(app->version, sizeof(app->version),
                                       &firmware);
    } else {
        (void) firmware_metadata_parse(NULL, 0U, &firmware);
    }
    if (!board_info_format(identity, descriptor, &firmware, contents)) {
        return ESP_ERR_INVALID_STATE;
    }

    pdrv = ff_diskio_get_pdrv_wl(wl_handle);
    if (pdrv > 9U) {
        return ESP_ERR_INVALID_STATE;
    }
    path_length = snprintf(fat_path, sizeof(fat_path), "%u:/%s",
                           (unsigned int) pdrv, INFO_FILENAME);
    if (path_length <= 0 || (size_t) path_length >= sizeof(fat_path)) {
        return ESP_ERR_INVALID_SIZE;
    }

    attribute_result = f_chmod(fat_path, 0U, AM_RDO);
    if (attribute_result != FR_OK && attribute_result != FR_NO_FILE) {
        ESP_LOGE(TAG, "INFO.TXT read-only attribute clear failed: %d",
                 (int) attribute_result);
        return ESP_FAIL;
    }

    file = fopen(CONFIG_MOUNT_PATH "/" INFO_FILENAME, "wb");
    if (file == NULL) {
        ESP_LOGE(TAG, "INFO.TXT create failed");
        return ESP_FAIL;
    }
    content_length = strlen(contents);
    write_succeeded =
        fwrite(contents, 1U, content_length, file) == content_length &&
        fflush(file) == 0 && fsync(fileno(file)) == 0;
    if (fclose(file) != 0) {
        write_succeeded = false;
    }
    if (!write_succeeded) {
        ESP_LOGE(TAG, "INFO.TXT write or sync failed");
        return ESP_FAIL;
    }

    attribute_result = f_chmod(fat_path, AM_RDO, AM_RDO);
    if (attribute_result != FR_OK) {
        ESP_LOGE(TAG, "INFO.TXT read-only attribute set failed: %d",
                 (int) attribute_result);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static bool make_serial_number(void) {
    const board_identity_t *identity = board_active_identity();

    if (identity == NULL || !board_identity_validate(identity)) {
        serial_number[0] = '\0';
        return false;
    }
    (void) snprintf(serial_number, sizeof(serial_number), "%s",
                    identity->serial);
    return true;
}

static void tinyusb_device_event(tinyusb_event_t *event, void *arg) {
    bool restore_app_ownership = false;
    bool policy_locked = false;
    usb_storage_owner_t owner;
    esp_err_t storage_error;
    esp_err_t ret;

    (void) arg;
    if (event == NULL) {
        return;
    }
    if (msc_policy_mutex != NULL) {
        policy_locked =
            xSemaphoreTake(msc_policy_mutex, portMAX_DELAY) == pdTRUE;
    }

    portENTER_CRITICAL(&state_lock);
    if (event->id == TINYUSB_EVENT_ATTACHED) {
        usb_diagnostics.device_attached = true;
        restore_app_ownership = usb_storage_policy_restore_on_attach(
            msc_exposure_enabled, msc_storage != NULL);
        if (usb_diagnostics.attach_count < UINT32_MAX) {
            usb_diagnostics.attach_count++;
        }
    } else if (event->id == TINYUSB_EVENT_DETACHED) {
        usb_diagnostics.device_attached = false;
        usb_diagnostics.cdc_connected = false;
        restore_app_ownership = usb_storage_policy_restore_on_detach(
            msc_storage != NULL, usb_diagnostics.storage_owner);
        if (usb_diagnostics.detach_count < UINT32_MAX) {
            usb_diagnostics.detach_count++;
        }
    }
    portEXIT_CRITICAL(&state_lock);

    /*
     * esp_tinyusb globally moves every registered MSC storage to USB ownership
     * from tud_mount_cb(). Keep startup storage application-owned until the
     * firmware explicitly opens the MSC gate.
     */
    if (restore_app_ownership) {
        ret = tinyusb_msc_set_storage_mount_point(
            msc_storage, TINYUSB_MSC_STORAGE_MOUNT_APP);
        portENTER_CRITICAL(&state_lock);
        owner = usb_diagnostics.storage_owner;
        storage_error = usb_diagnostics.last_storage_error;
        portEXIT_CRITICAL(&state_lock);
        if (ret == ESP_ERR_NOT_FINISHED &&
            event->id == TINYUSB_EVENT_DETACHED) {
            ESP_LOGI(TAG,
                     "MSC detach ownership transition deferred until writes drain");
        } else if (ret != ESP_OK || owner != USB_STORAGE_APP_OWNED) {
            esp_err_t effective_error = ret;

            if (ret == ESP_OK) {
                effective_error = storage_error;
                if (effective_error == ESP_OK) {
                    effective_error = ESP_FAIL;
                }
            }
            ESP_LOGE(TAG, "failed to retain APP storage ownership: %s",
                     esp_err_to_name(effective_error));
            set_storage_unavailable(effective_error);
        }
    }
    if (policy_locked) {
        (void) xSemaphoreGive(msc_policy_mutex);
    }
    if (event->id == TINYUSB_EVENT_DETACHED) {
        storage_mode_request_forced_exit();
    }
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
        storage_mode_request_forced_exit();
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
        usb_diagnostics.storage_owner = usb_storage_policy_mount_owner(
            event->mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP);
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

esp_err_t usb_device_storage_init(app_config_profiles_t *profiles,
                                  bool format_config_storage) {
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 6,
        .allocation_unit_size = 4096,
        .use_one_fat = false,
    };
    const esp_partition_t *partition = NULL;
    wl_handle_t preflight_handle = WL_INVALID_HANDLE;
    bool media_ready;
    esp_err_t storage_error;
    esp_err_t ret;

    if (profiles == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    app_config_profiles_set_defaults(profiles);

    storage_io_mutex = xSemaphoreCreateMutex();
    if (storage_io_mutex == NULL) {
        set_storage_unavailable(ESP_ERR_NO_MEM);
        return ESP_ERR_NO_MEM;
    }
    msc_policy_mutex = xSemaphoreCreateMutex();
    if (msc_policy_mutex == NULL) {
        vSemaphoreDelete(storage_io_mutex);
        storage_io_mutex = NULL;
        set_storage_unavailable(ESP_ERR_NO_MEM);
        return ESP_ERR_NO_MEM;
    }

    if (storage_mode_idle_timer == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = storage_mode_idle_timer_cb,
            .name = "msc_idle",
        };

        ret = esp_timer_create(&timer_args, &storage_mode_idle_timer);
        if (ret != ESP_OK) {
            set_storage_unavailable(ret);
            return ret;
        }
    }

    const tinyusb_msc_driver_config_t driver_config = {
        .user_flags = {.val = 0},
        .callback = msc_storage_event,
        .callback_arg = NULL,
        .write_callback = msc_write_event,
        .write_callback_arg = NULL,
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

    ret = write_info_file(preflight_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INFO.TXT generation failed: %s", esp_err_to_name(ret));
        (void) esp_vfs_fat_spiflash_unmount_rw_wl(CONFIG_MOUNT_PATH,
                                                   preflight_handle);
        set_storage_unavailable(ret);
        return ret;
    }

    usb_diagnostics.load_result =
        config_storage_load(CONFIG_MOUNT_PATH, profiles,
                            &usb_diagnostics.config);
    if (usb_diagnostics.load_result == CONFIG_LOAD_DEFAULT_NO_FILE) {
        ret = config_storage_save(CONFIG_MOUNT_PATH, profiles);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "default setting.json generation failed: %s",
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
        if (storage_error == ESP_OK) {
            return ESP_FAIL;
        }
        return storage_error;
    }
    return ESP_OK;
}

esp_err_t usb_device_start(void) {
    esp_err_t first_error = ESP_OK;
    esp_err_t ret;
    bool driver_ready;
    bool stopping;
    bool msc_driver_ready;
    bool media_ready;
    esp_err_t storage_error;

    portENTER_CRITICAL(&state_lock);
    driver_ready = usb_diagnostics.driver_ready;
    stopping = usb_stopping;
    msc_driver_ready = usb_diagnostics.msc_driver_ready;
    portEXIT_CRITICAL(&state_lock);
    if (driver_ready || stopping) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!msc_driver_ready) {
        ESP_LOGW(TAG,
                 "TinyUSB composite not started because MSC driver is unavailable");
        return ESP_ERR_INVALID_STATE;
    }

    if (!make_serial_number()) {
        ESP_LOGE(TAG, "TinyUSB requires a valid board product serial");
        return ESP_ERR_INVALID_STATE;
    }
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
    } else {
        portENTER_CRITICAL(&state_lock);
        console_redirect_ready = true;
        portEXIT_CRITICAL(&state_lock);
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

esp_err_t usb_device_stop(void) {
    usb_storage_owner_t owner;
    bool driver_ready;
    bool cdc_ready;
    bool console_ready;
    bool exposure_was_enabled;
    uint32_t internal_pending_writes = 0U;
    esp_err_t ret;

    if (msc_policy_mutex == NULL) {
        portENTER_CRITICAL(&state_lock);
        driver_ready = usb_diagnostics.driver_ready;
        portEXIT_CRITICAL(&state_lock);
        if (driver_ready) {
            return ESP_ERR_INVALID_STATE;
        }
        return ESP_OK;
    }
    if (xSemaphoreTake(msc_policy_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&state_lock);
    driver_ready = usb_diagnostics.driver_ready;
    if (!driver_ready) {
        usb_stopping = false;
        portEXIT_CRITICAL(&state_lock);
        (void) xSemaphoreGive(msc_policy_mutex);
        return ESP_OK;
    }
    if (usb_diagnostics.storage_mode_active ||
        usb_diagnostics.pending_write_count != 0U ||
        usb_diagnostics.storage_owner == USB_STORAGE_SWITCHING) {
        portEXIT_CRITICAL(&state_lock);
        (void) xSemaphoreGive(msc_policy_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    usb_stopping = true;
    exposure_was_enabled = msc_exposure_enabled;
    msc_exposure_enabled = false;
    owner = usb_diagnostics.storage_owner;
    portEXIT_CRITICAL(&state_lock);

    if (msc_storage != NULL) {
        ret = tinyusb_msc_stop_host_io(msc_storage,
                                       &internal_pending_writes);
        if (ret != ESP_OK || internal_pending_writes != 0U) {
            if (ret == ESP_OK) {
                (void) tinyusb_msc_set_storage_mount_point(
                    msc_storage, TINYUSB_MSC_STORAGE_MOUNT_USB);
                ret = ESP_ERR_INVALID_STATE;
            }
            portENTER_CRITICAL(&state_lock);
            usb_stopping = false;
            msc_exposure_enabled = exposure_was_enabled;
            portEXIT_CRITICAL(&state_lock);
            (void) xSemaphoreGive(msc_policy_mutex);
            return ret;
        }
    }

    if (owner == USB_STORAGE_HOST_OWNED && msc_storage != NULL) {
        ret = tinyusb_msc_set_storage_mount_point(
            msc_storage, TINYUSB_MSC_STORAGE_MOUNT_APP);
        if (ret != ESP_OK) {
            portENTER_CRITICAL(&state_lock);
            usb_stopping = false;
            msc_exposure_enabled = exposure_was_enabled;
            portEXIT_CRITICAL(&state_lock);
            (void) xSemaphoreGive(msc_policy_mutex);
            return ret;
        }
    }
    (void) xSemaphoreGive(msc_policy_mutex);

    portENTER_CRITICAL(&state_lock);
    console_ready = console_redirect_ready;
    cdc_ready = usb_diagnostics.cdc_ready;
    portEXIT_CRITICAL(&state_lock);
    if (console_ready) {
        ret = tinyusb_console_deinit(TINYUSB_CDC_ACM_0);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "TinyUSB console deinitialization failed: %s",
                     esp_err_to_name(ret));
        }
    }

    ret = tinyusb_driver_uninstall();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TinyUSB driver shutdown failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }
    if (cdc_ready) {
        esp_err_t cdc_ret = tinyusb_cdcacm_deinit(TINYUSB_CDC_ACM_0);

        if (cdc_ret != ESP_OK) {
            ESP_LOGW(TAG, "TinyUSB CDC deinitialization failed: %s",
                     esp_err_to_name(cdc_ret));
        }
    }

    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.driver_ready = false;
    usb_diagnostics.cdc_ready = false;
    usb_diagnostics.msc_enabled = false;
    usb_diagnostics.device_attached = false;
    usb_diagnostics.cdc_connected = false;
    console_redirect_ready = false;
    usb_stopping = false;
    portEXIT_CRITICAL(&state_lock);
    ESP_LOGI(TAG, "application TinyUSB task and PHY stopped");
    return ESP_OK;
}

esp_err_t usb_device_enable_msc(void) {
    bool attached;
    bool driver_ready;
    bool msc_driver_ready;
    usb_storage_owner_t owner;
    esp_err_t storage_error;
    esp_err_t ret;

    if (msc_policy_mutex == NULL ||
        xSemaphoreTake(msc_policy_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&state_lock);
    attached = usb_diagnostics.device_attached;
    driver_ready = usb_diagnostics.driver_ready;
    msc_driver_ready = usb_diagnostics.msc_driver_ready;
    owner = usb_diagnostics.storage_owner;
    if (driver_ready && msc_driver_ready &&
        msc_storage != NULL &&
        (owner == USB_STORAGE_APP_OWNED ||
         owner == USB_STORAGE_HOST_OWNED)) {
        msc_exposure_enabled = true;
        usb_diagnostics.msc_enabled = true;
    }
    portEXIT_CRITICAL(&state_lock);
    if (!driver_ready || !msc_driver_ready ||
        msc_storage == NULL ||
        (owner != USB_STORAGE_APP_OWNED &&
         owner != USB_STORAGE_HOST_OWNED)) {
        (void) xSemaphoreGive(msc_policy_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (!attached) {
        ESP_LOGI(TAG, "MSC medium enabled; waiting for USB host attachment");
        (void) xSemaphoreGive(msc_policy_mutex);
        return ESP_OK;
    }
    if (owner == USB_STORAGE_HOST_OWNED) {
        ESP_LOGI(TAG, "MSC medium already owned by USB host");
        (void) xSemaphoreGive(msc_policy_mutex);
        return ESP_OK;
    }

    ret = tinyusb_msc_set_storage_mount_point(
        msc_storage, TINYUSB_MSC_STORAGE_MOUNT_USB);
    portENTER_CRITICAL(&state_lock);
    owner = usb_diagnostics.storage_owner;
    storage_error = usb_diagnostics.last_storage_error;
    if (ret != ESP_OK || owner != USB_STORAGE_HOST_OWNED) {
        msc_exposure_enabled = false;
        usb_diagnostics.msc_enabled = false;
    }
    portEXIT_CRITICAL(&state_lock);
    (void) xSemaphoreGive(msc_policy_mutex);
    if (ret != ESP_OK) {
        return ret;
    }
    if (owner != USB_STORAGE_HOST_OWNED) {
        if (storage_error == ESP_OK) {
            return ESP_FAIL;
        }
        return storage_error;
    }
    ESP_LOGI(TAG, "MSC medium enabled for USB host");
    return ESP_OK;
}

void usb_device_set_storage_mode_callbacks(
    usb_storage_mode_begin_cb_t begin_cb,
    usb_storage_mode_end_cb_t end_cb, void *arg) {
    portENTER_CRITICAL(&state_lock);
    storage_mode_begin_cb = begin_cb;
    storage_mode_end_cb = end_cb;
    storage_mode_callback_arg = arg;
    portEXIT_CRITICAL(&state_lock);
}

bool usb_device_storage_mode_active(void) {
    bool active = false;

    portENTER_CRITICAL(&state_lock);
    active = usb_diagnostics.storage_mode_active;
    portEXIT_CRITICAL(&state_lock);
    return active;
}

esp_err_t usb_device_storage_begin_app_io(uint32_t timeout_ms) {
    usb_storage_owner_t owner;

    if (storage_io_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&state_lock);
    owner = usb_diagnostics.storage_owner;
    portEXIT_CRITICAL(&state_lock);
    if (!usb_storage_policy_app_io_allowed(owner)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(storage_io_mutex, pdMS_TO_TICKS(timeout_ms)) !=
        pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    portENTER_CRITICAL(&state_lock);
    owner = usb_diagnostics.storage_owner;
    portEXIT_CRITICAL(&state_lock);
    if (!usb_storage_policy_app_io_allowed(owner)) {
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

esp_err_t usb_device_save_config(const app_config_profiles_t *profiles) {
    esp_err_t ret;

    if (profiles == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ret = usb_device_storage_begin_app_io(STORAGE_MUTEX_TIMEOUT_MS);
    if (ret == ESP_OK) {
        ret = config_storage_save(CONFIG_MOUNT_PATH, profiles);
        usb_device_storage_end_app_io();
    }
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.last_save_result = ret;
    portEXIT_CRITICAL(&state_lock);
    return ret;
}

imu_calibration_storage_result_t usb_device_load_imu_calibration(
    imu_accel_calibration_t *calibration,
    imu_calibration_storage_diagnostics_t *diagnostics) {
    esp_err_t ret = usb_device_storage_begin_app_io(
        STORAGE_MUTEX_TIMEOUT_MS);
    imu_calibration_storage_result_t result =
        IMU_CALIBRATION_STORAGE_IO_ERROR;

    if (calibration == NULL) {
        return IMU_CALIBRATION_STORAGE_IO_ERROR;
    }
    if (ret != ESP_OK) {
        memset(calibration, 0, sizeof(*calibration));
        if (diagnostics != NULL) {
            diagnostics->result = IMU_CALIBRATION_STORAGE_IO_ERROR;
            diagnostics->io_error = (int32_t) ret;
        }
        return IMU_CALIBRATION_STORAGE_IO_ERROR;
    }
    result = imu_calibration_storage_load(CONFIG_MOUNT_PATH, calibration,
                                          diagnostics);
    usb_device_storage_end_app_io();
    return result;
}

esp_err_t usb_device_save_imu_calibration(
    const imu_accel_calibration_t *calibration) {
    esp_err_t ret = usb_device_storage_begin_app_io(
        STORAGE_MUTEX_TIMEOUT_MS);

    if (ret == ESP_OK) {
        ret = imu_calibration_storage_save(CONFIG_MOUNT_PATH, calibration);
        usb_device_storage_end_app_io();
    }
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
    connected = !usb_stopping && usb_diagnostics.cdc_ready &&
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
