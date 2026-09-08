#include "platform/usb_device_service.h"

#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "diskio_wl.h"
#include "domain/board_info.h"
#include "domain/firmware_metadata.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "ff.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mbedtls/platform_util.h"
#include "platform/board.h"
#include "platform/imu_calibration_storage.h"
#include "platform/firmware_auth.h"
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
#define RECOVERY_MOUNT_PATH "/update"
#define RECOVERY_VOLUME_LABEL "CBVUPDATE"
#define RECOVERY_README_FILENAME "README.TXT"
#define RECOVERY_RESULT_FILENAME "UPDATE_RESULT.TXT"
#define RECOVERY_RESULT_TEMP_FILENAME "UPDATE.TMP"
#define RECOVERY_DISK_BYTES UINT32_C(4194304)
#define RECOVERY_SECTOR_BYTES UINT32_C(512)
#define INFO_FILENAME "INFO.TXT"
#define SETTING_EDITOR_FILENAME "setting_editor.html"
#define GENERATED_FILE_PATH_CAPACITY 32U
#define GENERATED_FILE_IO_CHUNK_BYTES UINT32_C(4096)
#define STORAGE_MUTEX_TIMEOUT_MS UINT32_C(100)
#define STORAGE_MODE_QUIESCE_TIMEOUT_MS UINT32_C(100)
#define STORAGE_MODE_IDLE_US INT64_C(1000000)
#define STORAGE_RELEASE_POLL_MS UINT32_C(10)
#define USB_SERIAL_NUMBER_LENGTH BOARD_SERIAL_BUFFER_SIZE
#define USB_CONFIG_TOTAL_LENGTH \
    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN)

static const char *TAG = "usb_device";
extern const uint8_t setting_editor_html_start[]
    asm("_binary_setting_editor_html_start");
extern const uint8_t setting_editor_html_end[]
    asm("_binary_setting_editor_html_end");
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t storage_io_mutex;
static SemaphoreHandle_t msc_policy_mutex;
static bool storage_transition_locked;
static usb_storage_owner_t storage_transition_from =
    USB_STORAGE_UNAVAILABLE;
static bool msc_exposure_enabled;
static bool msc_exposure_requested;
static bool usb_stopping;
static bool console_redirect_ready;
static wl_handle_t wear_levelling_handle = WL_INVALID_HANDLE;
static tinyusb_msc_storage_handle_t msc_storage;
static uint8_t *recovery_psram_buffer;
static bool recovery_config_mounted;
static bool recovery_storage_active;
static esp_timer_handle_t storage_mode_idle_timer;
static usb_storage_mode_begin_cb_t storage_mode_begin_cb;
static usb_storage_mode_end_cb_t storage_mode_end_cb;
static void *storage_mode_callback_arg;
static int64_t current_write_started_us;
static int64_t last_write_completed_us;
static bool storage_mode_force_exit;
static char serial_number[USB_SERIAL_NUMBER_LENGTH];
static uint8_t generated_file_io_buffer[GENERATED_FILE_IO_CHUNK_BYTES];
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
    .recovery_config_error = ESP_ERR_INVALID_STATE,
    .psram_allocation_error = ESP_ERR_INVALID_STATE,
};

static const char recovery_readme[] =
    "CloudBaseVario PSRAM recovery volume\r\n"
    "\r\n"
    "Copy the signed UPDATE.BIN to this drive, then use the operating\r\n"
    "system's safe-eject action. Keep USB connected until the device\r\n"
    "restarts. Files on this volume are lost when USB power is removed.\r\n";

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

static void report_storage_progress(usb_storage_progress_cb_t progress_cb,
                                    void *progress_arg) {
    if (progress_cb != NULL) {
        progress_cb(progress_arg);
    }
}

static esp_err_t write_recovery_text_file_at(const char *mount_path,
                                             const char *filename,
                                             const char *contents) {
    char path[GENERATED_FILE_PATH_CAPACITY];
    FILE *file;
    size_t length;
    int path_length;
    esp_err_t ret = ESP_OK;

    if (mount_path == NULL || filename == NULL || contents == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    path_length = snprintf(path, sizeof(path), "%s/%s",
                           mount_path, filename);
    if (path_length <= 0 || (size_t) path_length >= sizeof(path)) {
        return ESP_ERR_INVALID_SIZE;
    }
    file = fopen(path, "wb");
    if (file == NULL) {
        return ESP_FAIL;
    }
    length = strlen(contents);
    if (fwrite(contents, 1U, length, file) != length ||
        fflush(file) != 0 || fsync(fileno(file)) != 0) {
        ret = ESP_FAIL;
    }
    if (fclose(file) != 0 && ret == ESP_OK) {
        ret = ESP_FAIL;
    }
    return ret;
}

static esp_err_t write_recovery_text_file(const char *filename,
                                          const char *contents) {
    return write_recovery_text_file_at(RECOVERY_MOUNT_PATH, filename,
                                       contents);
}

static esp_err_t write_recovery_init_failure_status(esp_err_t error) {
    char status[160];
    char result_path[GENERATED_FILE_PATH_CAPACITY];
    char temp_path[GENERATED_FILE_PATH_CAPACITY];
    int status_length;
    int result_path_length;
    int temp_path_length;
    esp_err_t ret;

    status_length = snprintf(
        status, sizeof(status),
        "state=REJECTED\r\n"
        "reason=recovery storage initialization failed\r\n"
        "error=%s\r\nsource=PSRAM\r\n",
        esp_err_to_name(error));
    result_path_length = snprintf(
        result_path, sizeof(result_path), "%s/%s", CONFIG_MOUNT_PATH,
        RECOVERY_RESULT_FILENAME);
    temp_path_length = snprintf(
        temp_path, sizeof(temp_path), "%s/%s", CONFIG_MOUNT_PATH,
        RECOVERY_RESULT_TEMP_FILENAME);
    if (status_length <= 0 || (size_t) status_length >= sizeof(status) ||
        result_path_length <= 0 ||
        (size_t) result_path_length >= sizeof(result_path) ||
        temp_path_length <= 0 ||
        (size_t) temp_path_length >= sizeof(temp_path)) {
        return ESP_ERR_INVALID_SIZE;
    }
    ret = write_recovery_text_file_at(
        CONFIG_MOUNT_PATH, RECOVERY_RESULT_TEMP_FILENAME, status);
    if (ret != ESP_OK) {
        return ret;
    }
    (void) unlink(result_path);
    if (rename(temp_path, result_path) != 0) {
        (void) unlink(temp_path);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t seed_recovery_volume(void) {
    const esp_app_desc_t *app = esp_app_get_description();
    firmware_metadata_t metadata = {0};
    tinyusb_msc_psram_diagnostics_t psram = {0};
    char label[16];
    char info[192];
    int length;
    esp_err_t ret;

    tinyusb_msc_get_psram_diagnostics(&psram);
    if (psram.drive_number == TINYUSB_MSC_PDRV_INVALID) {
        return ESP_ERR_INVALID_STATE;
    }
    length = snprintf(label, sizeof(label), "%u:%s",
                      (unsigned int) psram.drive_number,
                      RECOVERY_VOLUME_LABEL);
    if (length <= 0 || (size_t) length >= sizeof(label) ||
        f_setlabel(label) != FR_OK) {
        return ESP_FAIL;
    }
    if (app != NULL) {
        (void) firmware_metadata_parse(app->version, sizeof(app->version),
                                       &metadata);
    } else {
        (void) firmware_metadata_parse(NULL, 0U, &metadata);
    }
    length = snprintf(
        info, sizeof(info),
        "CloudBaseVario recovery\r\nmedium=PSRAM\r\nvolatile=1\r\n"
        "version=%s\r\nhash=%s\r\n",
        metadata.version, metadata.git_hash);
    if (length <= 0 || (size_t) length >= sizeof(info)) {
        return ESP_ERR_INVALID_SIZE;
    }
    ret = write_recovery_text_file(INFO_FILENAME, info);
    if (ret == ESP_OK) {
        ret = write_recovery_text_file(RECOVERY_README_FILENAME,
                                       recovery_readme);
    }
    return ret;
}

static esp_err_t generated_file_matches(
    const char *vfs_path, const uint8_t *contents, size_t content_length,
    usb_storage_progress_cb_t progress_cb, void *progress_arg,
    bool *matches) {
    FILE *file;
    size_t offset = 0U;
    esp_err_t result = ESP_OK;

    if (matches == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *matches = false;
    errno = 0;
    file = fopen(vfs_path, "rb");
    if (file == NULL) {
        if (errno == ENOENT) {
            return ESP_OK;
        }
        ESP_LOGE(TAG, "%s comparison open failed: errno=%d", vfs_path,
                 errno);
        return ESP_FAIL;
    }

    *matches = true;
    while (offset < content_length) {
        size_t remaining = content_length - offset;
        size_t chunk_size = remaining;
        size_t read_size;

        if (chunk_size > sizeof(generated_file_io_buffer)) {
            chunk_size = sizeof(generated_file_io_buffer);
        }
        read_size = fread(generated_file_io_buffer, 1U, chunk_size, file);
        report_storage_progress(progress_cb, progress_arg);
        if (read_size != chunk_size) {
            if (ferror(file)) {
                ESP_LOGE(TAG, "%s comparison read failed", vfs_path);
                result = ESP_FAIL;
            }
            *matches = false;
            break;
        }
        if (memcmp(generated_file_io_buffer, contents + offset,
                   chunk_size) != 0) {
            *matches = false;
            break;
        }
        offset += chunk_size;
    }
    if (result == ESP_OK && *matches) {
        int trailing_byte = fgetc(file);

        report_storage_progress(progress_cb, progress_arg);
        if (trailing_byte != EOF) {
            *matches = false;
        } else if (ferror(file)) {
            ESP_LOGE(TAG, "%s trailing-byte check failed", vfs_path);
            *matches = false;
            result = ESP_FAIL;
        }
    }
    if (fclose(file) != 0 && result == ESP_OK) {
        ESP_LOGE(TAG, "%s comparison close failed", vfs_path);
        result = ESP_FAIL;
    }
    report_storage_progress(progress_cb, progress_arg);
    return result;
}

static esp_err_t write_read_only_file(wl_handle_t wl_handle,
                                      const char *filename,
                                      const uint8_t *contents,
                                      size_t content_length,
                                      usb_storage_progress_cb_t progress_cb,
                                      void *progress_arg) {
    char fat_path[GENERATED_FILE_PATH_CAPACITY] = {0};
    char vfs_path[GENERATED_FILE_PATH_CAPACITY] = {0};
    BYTE pdrv;
    FRESULT attribute_result;
    FILE *file;
    int path_length;
    size_t offset = 0U;
    bool matches = false;
    esp_err_t ret;

    if (filename == NULL || filename[0] == '\0' || contents == NULL ||
        content_length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    pdrv = ff_diskio_get_pdrv_wl(wl_handle);
    if (pdrv > 9U) {
        return ESP_ERR_INVALID_STATE;
    }
    path_length = snprintf(fat_path, sizeof(fat_path), "%u:/%s",
                           (unsigned int) pdrv, filename);
    if (path_length <= 0 || (size_t) path_length >= sizeof(fat_path)) {
        return ESP_ERR_INVALID_SIZE;
    }
    path_length = snprintf(vfs_path, sizeof(vfs_path), "%s/%s",
                           CONFIG_MOUNT_PATH, filename);
    if (path_length <= 0 || (size_t) path_length >= sizeof(vfs_path)) {
        return ESP_ERR_INVALID_SIZE;
    }

    ret = generated_file_matches(vfs_path, contents, content_length,
                                 progress_cb, progress_arg, &matches);
    if (ret != ESP_OK) {
        return ret;
    }
    if (matches) {
        report_storage_progress(progress_cb, progress_arg);
        attribute_result = f_chmod(fat_path, AM_RDO, AM_RDO);
        report_storage_progress(progress_cb, progress_arg);
        if (attribute_result != FR_OK) {
            ESP_LOGE(TAG, "%s read-only attribute set failed: %d", filename,
                     (int) attribute_result);
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    report_storage_progress(progress_cb, progress_arg);
    attribute_result = f_chmod(fat_path, 0U, AM_RDO);
    report_storage_progress(progress_cb, progress_arg);
    if (attribute_result != FR_OK && attribute_result != FR_NO_FILE) {
        ESP_LOGE(TAG, "%s read-only attribute clear failed: %d", filename,
                 (int) attribute_result);
        return ESP_FAIL;
    }

    file = fopen(vfs_path, "wb");
    if (file == NULL) {
        ESP_LOGE(TAG, "%s create failed", filename);
        return ESP_FAIL;
    }
    while (offset < content_length) {
        size_t remaining = content_length - offset;
        size_t chunk_size = remaining;

        if (chunk_size > GENERATED_FILE_IO_CHUNK_BYTES) {
            chunk_size = GENERATED_FILE_IO_CHUNK_BYTES;
        }
        report_storage_progress(progress_cb, progress_arg);
        if (fwrite(contents + offset, 1U, chunk_size, file) != chunk_size) {
            ESP_LOGE(TAG, "%s write failed", filename);
            (void) fclose(file);
            return ESP_FAIL;
        }
        offset += chunk_size;
        report_storage_progress(progress_cb, progress_arg);
    }
    report_storage_progress(progress_cb, progress_arg);
    if (fflush(file) != 0) {
        ESP_LOGE(TAG, "%s flush failed", filename);
        (void) fclose(file);
        return ESP_FAIL;
    }
    report_storage_progress(progress_cb, progress_arg);
    if (fsync(fileno(file)) != 0) {
        ESP_LOGE(TAG, "%s sync failed", filename);
        (void) fclose(file);
        return ESP_FAIL;
    }
    report_storage_progress(progress_cb, progress_arg);
    if (fclose(file) != 0) {
        ESP_LOGE(TAG, "%s close failed", filename);
        return ESP_FAIL;
    }

    report_storage_progress(progress_cb, progress_arg);
    attribute_result = f_chmod(fat_path, AM_RDO, AM_RDO);
    report_storage_progress(progress_cb, progress_arg);
    if (attribute_result != FR_OK) {
        ESP_LOGE(TAG, "%s read-only attribute set failed: %d", filename,
                 (int) attribute_result);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t write_info_file(wl_handle_t wl_handle,
                                 usb_storage_progress_cb_t progress_cb,
                                 void *progress_arg) {
    const board_identity_t *identity = board_active_identity();
    const board_descriptor_t *descriptor = board_active_descriptor();
    const esp_app_desc_t *app = esp_app_get_description();
    firmware_metadata_t firmware = {0};
    firmware_authentication_t authentication = {0};
    const esp_partition_t *running_partition;
    char contents[BOARD_INFO_TEXT_CAPACITY] = {0};

    if (app != NULL) {
        (void) firmware_metadata_parse(app->version, sizeof(app->version),
                                       &firmware);
    } else {
        (void) firmware_metadata_parse(NULL, 0U, &firmware);
    }
    running_partition = esp_ota_get_running_partition();
    if (running_partition == NULL ||
        firmware_authenticate_partition(running_partition,
                                        "CloudBaseVario-Aohazuku",
                                        &authentication, progress_cb,
                                        progress_arg) != ESP_OK) {
        authentication.authenticity = FIRMWARE_AUTH_UNKNOWN;
    }
    if (!board_info_format(identity, descriptor, &firmware, &authentication,
                           contents)) {
        return ESP_ERR_INVALID_STATE;
    }

    return write_read_only_file(wl_handle, INFO_FILENAME,
                                (const uint8_t *) contents,
                                strlen(contents), progress_cb,
                                progress_arg);
}

static esp_err_t write_setting_editor_file(
    wl_handle_t wl_handle, usb_storage_progress_cb_t progress_cb,
    void *progress_arg) {
    size_t content_length;

    if (&setting_editor_html_end[0] <= &setting_editor_html_start[0]) {
        return ESP_ERR_INVALID_SIZE;
    }
    content_length = (size_t) (setting_editor_html_end -
                               setting_editor_html_start);
    return write_read_only_file(wl_handle, SETTING_EDITOR_FILENAME,
                                setting_editor_html_start, content_length,
                                progress_cb, progress_arg);
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

static bool make_recovery_serial_number(void) {
    uint8_t mac[6] = {0};

    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        serial_number[0] = '\0';
        return false;
    }
    (void) snprintf(serial_number, sizeof(serial_number),
                    "REC-%02X%02X%02X%02X%02X%02X",
                    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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
        storage_transition_from = usb_diagnostics.storage_owner;
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
        if (event->mount_point == TINYUSB_MSC_STORAGE_MOUNT_APP &&
            storage_transition_from == USB_STORAGE_HOST_OWNED &&
            usb_diagnostics.host_release_count < UINT32_MAX) {
            usb_diagnostics.host_release_count++;
        }
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
    storage_transition_from = USB_STORAGE_UNAVAILABLE;
    portEXIT_CRITICAL(&state_lock);

    if (storage_transition_locked) {
        storage_transition_locked = false;
        (void) xSemaphoreGive(storage_io_mutex);
    }
}

static esp_err_t initialize_storage_service(void) {
    esp_err_t ret;

    if (storage_io_mutex != NULL || msc_policy_mutex != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
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
    return ESP_OK;
}

static esp_err_t create_msc_storage(
    usb_storage_progress_cb_t progress_cb, void *progress_arg) {
    const esp_partition_t *partition = NULL;
    bool media_ready;
    esp_err_t storage_error;
    esp_err_t ret;

    partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT,
        CONFIG_PARTITION_LABEL);
    if (partition == NULL) {
        set_storage_unavailable(ESP_ERR_NOT_FOUND);
        return ESP_ERR_NOT_FOUND;
    }
    report_storage_progress(progress_cb, progress_arg);
    ret = wl_mount(partition, &wear_levelling_handle);
    report_storage_progress(progress_cb, progress_arg);
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

    report_storage_progress(progress_cb, progress_arg);
    ret = tinyusb_msc_new_storage_spiflash(&storage_config, &msc_storage);
    report_storage_progress(progress_cb, progress_arg);
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
    report_storage_progress(progress_cb, progress_arg);
    ret = tinyusb_msc_set_storage_mount_point(
        msc_storage, TINYUSB_MSC_STORAGE_MOUNT_APP);
    report_storage_progress(progress_cb, progress_arg);
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

esp_err_t usb_device_storage_init(app_config_profiles_t *profiles,
                                  usb_storage_progress_cb_t progress_cb,
                                  void *progress_arg) {
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 6,
        .allocation_unit_size = 4096,
        .use_one_fat = false,
    };
    wl_handle_t preflight_handle = WL_INVALID_HANDLE;
    esp_err_t ret;

    if (profiles == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    app_config_profiles_set_defaults(profiles);
    report_storage_progress(progress_cb, progress_arg);
    ret = initialize_storage_service();
    if (ret != ESP_OK) {
        return ret;
    }

    report_storage_progress(progress_cb, progress_arg);
    ret = esp_vfs_fat_spiflash_mount_rw_wl(
        CONFIG_MOUNT_PATH, CONFIG_PARTITION_LABEL, &mount_config,
        &preflight_handle);
    report_storage_progress(progress_cb, progress_arg);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "config FAT unavailable; automatic format prohibited: %s",
                 esp_err_to_name(ret));
        set_storage_unavailable(ret);
        if (ret == ESP_ERR_NOT_FOUND) {
            increment_counter(&usb_diagnostics.format_required_count);
        }
        return ret;
    }

    ret = write_info_file(preflight_handle, progress_cb, progress_arg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INFO.TXT generation failed: %s", esp_err_to_name(ret));
        (void) esp_vfs_fat_spiflash_unmount_rw_wl(CONFIG_MOUNT_PATH,
                                                   preflight_handle);
        set_storage_unavailable(ret);
        return ret;
    }
    ret = write_setting_editor_file(preflight_handle, progress_cb,
                                    progress_arg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "setting_editor.html generation failed: %s",
                 esp_err_to_name(ret));
        (void) esp_vfs_fat_spiflash_unmount_rw_wl(CONFIG_MOUNT_PATH,
                                                   preflight_handle);
        set_storage_unavailable(ret);
        return ret;
    }

    report_storage_progress(progress_cb, progress_arg);
    usb_diagnostics.load_result =
        config_storage_load(CONFIG_MOUNT_PATH, profiles,
                            &usb_diagnostics.config);
    report_storage_progress(progress_cb, progress_arg);
    if (usb_diagnostics.load_result == CONFIG_LOAD_DEFAULT_NO_FILE) {
        report_storage_progress(progress_cb, progress_arg);
        ret = config_storage_save(CONFIG_MOUNT_PATH, profiles);
        report_storage_progress(progress_cb, progress_arg);
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
    report_storage_progress(progress_cb, progress_arg);

    report_storage_progress(progress_cb, progress_arg);
    ret = esp_vfs_fat_spiflash_unmount_rw_wl(CONFIG_MOUNT_PATH,
                                              preflight_handle);
    report_storage_progress(progress_cb, progress_arg);
    if (ret != ESP_OK) {
        set_storage_unavailable(ret);
        return ret;
    }
    return create_msc_storage(progress_cb, progress_arg);
}

esp_err_t usb_device_recovery_storage_init(
    usb_storage_progress_cb_t progress_cb, void *progress_arg) {
    esp_vfs_fat_mount_config_t config_mount = {
        .format_if_mount_failed = false,
        .max_files = 6,
        .allocation_unit_size = 4096,
        .use_one_fat = false,
    };
    tinyusb_msc_storage_config_t storage_config = {
        .fat_fs = {
            .base_path = RECOVERY_MOUNT_PATH,
            .config = {
                .format_if_mount_failed = true,
                .max_files = 4,
                .allocation_unit_size = 4096,
                .use_one_fat = false,
            },
            .do_not_format = false,
            .format_flags = FM_ANY,
        },
        .mount_point = TINYUSB_MSC_STORAGE_MOUNT_APP,
    };
    tinyusb_msc_psram_config_t psram_config = {
        .buffer = NULL,
        .size_bytes = RECOVERY_DISK_BYTES,
        .sector_size = RECOVERY_SECTOR_BYTES,
    };
    char config_error_status[160];
    int status_length;
    esp_err_t ret;

    if (gpio_get_level(PIN_PWR_EXT) == 0 || recovery_storage_active) {
        return ESP_ERR_INVALID_STATE;
    }
    report_storage_progress(progress_cb, progress_arg);
    ret = initialize_storage_service();
    if (ret != ESP_OK) {
        return ret;
    }

    report_storage_progress(progress_cb, progress_arg);
    ret = esp_vfs_fat_spiflash_mount_rw_wl(
        CONFIG_MOUNT_PATH, CONFIG_PARTITION_LABEL, &config_mount,
        &wear_levelling_handle);
    recovery_config_mounted = ret == ESP_OK;
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.recovery_config_ready = recovery_config_mounted;
    usb_diagnostics.recovery_config_error = ret;
    if (!recovery_config_mounted) {
        usb_diagnostics.last_storage_error = ret;
    }
    usb_diagnostics.psram_free_before_bytes = (uint32_t)
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    usb_diagnostics.psram_largest_before_bytes = (uint32_t)
        heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM |
                                         MALLOC_CAP_8BIT);
    portEXIT_CRITICAL(&state_lock);

    if (!esp_psram_is_initialized() || gpio_get_level(PIN_PWR_EXT) == 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto fail;
    }
    if (heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) <
            RECOVERY_DISK_BYTES ||
        heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM |
                                         MALLOC_CAP_8BIT) <
            RECOVERY_DISK_BYTES) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    /*
     * CODING_RULES_DYNAMIC_MEMORY: recovery owns one bounded, short-lived
     * four-MiB PSRAM disk. It is allocated only with USB VBUS present and is
     * released when recovery stops or the device restarts.
     */
    recovery_psram_buffer = heap_caps_calloc(
        1U, RECOVERY_DISK_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (recovery_psram_buffer == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    if (gpio_get_level(PIN_PWR_EXT) == 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto fail;
    }
    psram_config.buffer = recovery_psram_buffer;
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.psram_free_after_bytes = (uint32_t)
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    usb_diagnostics.psram_largest_after_bytes = (uint32_t)
        heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM |
                                         MALLOC_CAP_8BIT);
    usb_diagnostics.psram_allocation_error = ESP_OK;
    portEXIT_CRITICAL(&state_lock);

    report_storage_progress(progress_cb, progress_arg);
    ret = tinyusb_msc_new_storage_psram(&storage_config, &psram_config,
                                        &msc_storage);
    if (ret != ESP_OK) {
        goto fail;
    }
    report_storage_progress(progress_cb, progress_arg);
    ret = seed_recovery_volume();
    if (ret != ESP_OK) {
        goto fail;
    }
    if (gpio_get_level(PIN_PWR_EXT) == 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto fail;
    }
    if (!recovery_config_mounted) {
        status_length = snprintf(
            config_error_status, sizeof(config_error_status),
            "state=REJECTED\r\nreason=config storage unavailable\r\n"
            "error=%s\r\nsource=PSRAM\r\n",
            esp_err_to_name(usb_diagnostics.recovery_config_error));
        if (status_length > 0 &&
            (size_t) status_length < sizeof(config_error_status)) {
            (void) write_recovery_text_file(
                "UPDATE_RESULT.TXT", config_error_status);
        }
    }
    recovery_storage_active = true;
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.recovery_medium = USB_RECOVERY_MEDIUM_PSRAM;
    usb_diagnostics.recovery_disk_size_bytes = RECOVERY_DISK_BYTES;
    portEXIT_CRITICAL(&state_lock);
    return ESP_OK;

fail:
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.psram_allocation_error = ret;
    usb_diagnostics.last_storage_error = ret;
    portEXIT_CRITICAL(&state_lock);
    if (recovery_config_mounted) {
        esp_err_t status_result =
            write_recovery_init_failure_status(ret);

        if (status_result != ESP_OK) {
            ESP_LOGW(TAG, "recovery failure status write failed: %s",
                     esp_err_to_name(status_result));
        }
    }
    (void) usb_device_recovery_storage_deinit();
    return ret;
}

esp_err_t usb_device_recovery_storage_deinit(void) {
    esp_err_t first_error = ESP_OK;
    esp_err_t ret;

    if (msc_storage != NULL) {
        ret = tinyusb_msc_delete_storage(msc_storage);
        if (ret == ESP_OK) {
            msc_storage = NULL;
        } else {
            first_error = ret;
        }
    }
    if (msc_storage == NULL) {
        ret = tinyusb_msc_uninstall_driver();
        if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED &&
            first_error == ESP_OK) {
            first_error = ret;
        }
    }
    if (recovery_config_mounted) {
        ret = esp_vfs_fat_spiflash_unmount_rw_wl(
            CONFIG_MOUNT_PATH, wear_levelling_handle);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        if (ret == ESP_OK) {
            recovery_config_mounted = false;
            wear_levelling_handle = WL_INVALID_HANDLE;
        }
    }
    if (recovery_psram_buffer != NULL && msc_storage == NULL) {
        mbedtls_platform_zeroize(recovery_psram_buffer,
                                 RECOVERY_DISK_BYTES);
        heap_caps_free(recovery_psram_buffer);
        recovery_psram_buffer = NULL;
    }
    recovery_storage_active = false;
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.msc_driver_ready = false;
    usb_diagnostics.msc_media_ready = false;
    usb_diagnostics.storage_ready = false;
    usb_diagnostics.storage_owner = USB_STORAGE_UNAVAILABLE;
    usb_diagnostics.recovery_medium = USB_RECOVERY_MEDIUM_NONE;
    usb_diagnostics.recovery_config_ready = false;
    usb_diagnostics.recovery_disk_size_bytes = 0U;
    portEXIT_CRITICAL(&state_lock);
    return first_error;
}

bool usb_device_recovery_storage_ready(void) {
    return recovery_storage_active && recovery_config_mounted;
}

static esp_err_t start_usb_device(bool recovery_mode) {
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

    if ((!recovery_mode && !make_serial_number()) ||
        (recovery_mode && !make_recovery_serial_number())) {
        ESP_LOGE(TAG, "TinyUSB serial number initialization failed");
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
    if (recovery_mode && first_error != ESP_OK) {
        ESP_LOGW(TAG,
                 "recovery MSC continuing without CDC console redirect: %s",
                 esp_err_to_name(first_error));
        return ESP_OK;
    }
    return first_error;
}

esp_err_t usb_device_start(void) {
    return start_usb_device(false);
}

esp_err_t usb_device_start_recovery(void) {
    return start_usb_device(true);
}

esp_err_t usb_device_update_vbus(void) {
    bool driver_ready;
    bool exposure_enabled;
    bool exposure_requested;
    bool stopping;
    esp_err_t ret;

    portENTER_CRITICAL(&state_lock);
    driver_ready = usb_diagnostics.driver_ready;
    exposure_enabled = msc_exposure_enabled;
    exposure_requested = msc_exposure_requested;
    stopping = usb_stopping;
    portEXIT_CRITICAL(&state_lock);

    if (!usb_device_vbus_present()) {
        if (!driver_ready) {
            return ESP_OK;
        }
        return usb_device_stop();
    }
    if (stopping) {
        return ESP_ERR_NOT_FINISHED;
    }
    if (!driver_ready) {
        ret = usb_device_start();
        if (ret != ESP_OK) {
            return ret;
        }
    }
    if (exposure_requested && !exposure_enabled) {
        return usb_device_enable_msc();
    }
    return ESP_OK;
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

esp_err_t usb_device_request_msc(void) {
    bool driver_ready;

    portENTER_CRITICAL(&state_lock);
    msc_exposure_requested = true;
    driver_ready = usb_diagnostics.driver_ready;
    portEXIT_CRITICAL(&state_lock);
    if (!driver_ready) {
        ESP_LOGI(TAG, "MSC exposure deferred until USB VBUS is present");
        return ESP_OK;
    }
    return usb_device_enable_msc();
}

esp_err_t usb_device_wait_for_host_release(uint32_t release_count,
                                           uint32_t timeout_ms) {
    TickType_t started = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    for (;;) {
        usb_storage_owner_t owner;
        uint32_t pending_writes;
        uint32_t current_release_count;
        esp_err_t storage_error;

        portENTER_CRITICAL(&state_lock);
        owner = usb_diagnostics.storage_owner;
        pending_writes = usb_diagnostics.pending_write_count;
        current_release_count = usb_diagnostics.host_release_count;
        storage_error = usb_diagnostics.last_storage_error;
        portEXIT_CRITICAL(&state_lock);
        if (current_release_count != release_count &&
            owner == USB_STORAGE_APP_OWNED && pending_writes == 0U) {
            return ESP_OK;
        }
        if (owner == USB_STORAGE_UNAVAILABLE) {
            if (storage_error == ESP_OK) {
                return ESP_FAIL;
            }
            return storage_error;
        }
        if (xTaskGetTickCount() - started >= timeout_ticks) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(STORAGE_RELEASE_POLL_MS));
    }
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

bool usb_device_vbus_present(void) {
    return gpio_get_level(PIN_PWR_EXT) != 0;
}

const char *usb_device_storage_mount_path(void) {
    return CONFIG_MOUNT_PATH;
}

const char *usb_device_recovery_mount_path(void) {
    return RECOVERY_MOUNT_PATH;
}

void usb_device_get_diagnostics(usb_device_diagnostics_t *diagnostics) {
    tinyusb_msc_psram_diagnostics_t psram = {0};

    if (diagnostics == NULL) {
        return;
    }
    tinyusb_msc_get_psram_diagnostics(&psram);
    portENTER_CRITICAL(&state_lock);
    usb_diagnostics.vbus_present = gpio_get_level(PIN_PWR_EXT) != 0;
    usb_diagnostics.psram_read_count = psram.read_count;
    usb_diagnostics.psram_read_error_count = psram.read_error_count;
    usb_diagnostics.psram_write_count = psram.write_count;
    usb_diagnostics.psram_write_error_count = psram.write_error_count;
    usb_diagnostics.psram_read_bytes = psram.read_bytes;
    usb_diagnostics.psram_written_bytes = psram.written_bytes;
    *diagnostics = usb_diagnostics;
    portEXIT_CRITICAL(&state_lock);
}

const char *usb_device_recovery_medium_name(usb_recovery_medium_t medium) {
    if (medium == USB_RECOVERY_MEDIUM_PSRAM) {
        return "PSRAM";
    }
    return "NONE";
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
