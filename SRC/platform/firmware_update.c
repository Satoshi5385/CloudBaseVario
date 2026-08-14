#include "platform/firmware_update.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "domain/firmware_update_policy.h"
#include "platform/board.h"
#include "platform/usb_device_service.h"
#include "platform/watchdog_service.h"

#ifndef CBV_FIRMWARE_PROJECT_NAME
#error "The build must provide the Aohazuku firmware project identity"
#endif

#define UPDATE_INPUT_NAME "UPDATE.BIN"
#define UPDATE_PENDING_NAME "UPDATE.PND"
#define UPDATE_BAD_NAME "UPDATE.BAD"
#define UPDATE_STATUS_NAME "UPDATE.TXT"
#define UPDATE_STATUS_TEMP_NAME "UPDATE.TMP"
#define UPDATE_MAX_IMAGE_BYTES UINT32_C(0x380000)
#define UPDATE_IO_BUFFER_BYTES 8192U
#define UPDATE_STORAGE_TIMEOUT_MS UINT32_C(1000)
#define UPDATE_LED_PERIOD_MS UINT32_C(100)
#define UPDATE_CONFIRMATION_DELAY_MS UINT32_C(10000)
#define UPDATE_CONFIRMATION_TASK_STACK 3072U
#define UPDATE_CONFIRMATION_TASK_PRIORITY 2U
#define UPDATE_LED_TASK_STACK 2048U
#define UPDATE_LED_TASK_PRIORITY 1U
#define UPDATE_LED_STOP_MARGIN_MS UINT32_C(10)
#define UPDATE_CONFIRMATION_POLL_MS UINT32_C(10)
#define UPDATE_PATH_BUFFER_SIZE 64U
#define UPDATE_STATUS_TEXT_SIZE 384U
#define UPDATE_PRINTABLE_ASCII_MIN UINT8_C(0x20)
#define UPDATE_PRINTABLE_ASCII_MAX UINT8_C(0x7e)

static const char *TAG = "firmware_update";
static portMUX_TYPE update_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool update_led_running;
static firmware_update_diagnostics_t update_diagnostics = {
    .state = FIRMWARE_UPDATE_IDLE,
    .last_error = ESP_OK,
};

typedef struct {
    size_t size;
    esp_image_header_t header;
    esp_app_desc_t descriptor;
} update_image_info_t;

static void make_path(const char *name, char *path, size_t capacity) {
    (void) snprintf(path, capacity, "%s/%s",
                    usb_device_storage_mount_path(), name);
}

static bool file_exists(const char *name) {
    char path[UPDATE_PATH_BUFFER_SIZE];
    struct stat info;
    make_path(name, path, sizeof(path));
    return stat(path, &info) == 0 && S_ISREG(info.st_mode);
}

static esp_err_t remove_if_present(const char *name) {
    char path[UPDATE_PATH_BUFFER_SIZE];
    make_path(name, path, sizeof(path));
    if (unlink(path) == 0 || errno == ENOENT) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

static esp_err_t move_state_file(const char *from_name, const char *to_name) {
    char from_path[UPDATE_PATH_BUFFER_SIZE];
    char to_path[UPDATE_PATH_BUFFER_SIZE];
    esp_err_t result = ESP_FAIL;

    make_path(from_name, from_path, sizeof(from_path));
    make_path(to_name, to_path, sizeof(to_path));
    (void) unlink(to_path);
    if (rename(from_path, to_path) == 0) {
        result = ESP_OK;
    }
    return result;
}

static esp_err_t write_status(const char *format, ...) {
    char status_path[UPDATE_PATH_BUFFER_SIZE];
    char temp_path[UPDATE_PATH_BUFFER_SIZE];
    char text[UPDATE_STATUS_TEXT_SIZE];
    va_list arguments;
    FILE *file = NULL;
    int length;
    esp_err_t ret = ESP_OK;

    va_start(arguments, format);
    length = vsnprintf(text, sizeof(text), format, arguments);
    va_end(arguments);
    if (length < 0 || (size_t) length >= sizeof(text)) {
        return ESP_ERR_INVALID_SIZE;
    }

    make_path(UPDATE_STATUS_NAME, status_path, sizeof(status_path));
    make_path(UPDATE_STATUS_TEMP_NAME, temp_path, sizeof(temp_path));
    file = fopen(temp_path, "wb");
    if (file == NULL) {
        return ESP_FAIL;
    }
    if (fwrite(text, 1, (size_t) length, file) != (size_t) length ||
        fflush(file) != 0 || fsync(fileno(file)) != 0) {
        ret = ESP_FAIL;
    }
    if (fclose(file) != 0) {
        ret = ESP_FAIL;
    }
    if (ret == ESP_OK) {
        (void) unlink(status_path);
        if (rename(temp_path, status_path) != 0) {
            ret = ESP_FAIL;
        }
    } else {
        (void) unlink(temp_path);
    }
    return ret;
}

static void bytes_to_hex(const uint8_t *bytes, size_t length,
                         char *output, size_t capacity) {
    static const char hex[] = "0123456789abcdef";
    if (capacity < length * 2U + 1U) {
        if (capacity > 0U) {
            output[0] = '\0';
        }
        return;
    }
    for (size_t index = 0U; index < length; index++) {
        output[index * 2U] = hex[bytes[index] >> 4U];
        output[index * 2U + 1U] = hex[bytes[index] & 0x0fU];
    }
    output[length * 2U] = '\0';
}

static void record_image_info(const update_image_info_t *info) {
    portENTER_CRITICAL(&update_lock);
    update_diagnostics.image_size_bytes = (uint32_t) info->size;
    (void) snprintf(update_diagnostics.image_version,
                    sizeof(update_diagnostics.image_version), "%s",
                    info->descriptor.version);
    bytes_to_hex(info->descriptor.app_elf_sha256,
                 sizeof(info->descriptor.app_elf_sha256),
                 update_diagnostics.image_fingerprint,
                 sizeof(update_diagnostics.image_fingerprint));
    portEXIT_CRITICAL(&update_lock);
}

static void set_state(firmware_update_state_t state, esp_err_t error) {
    portENTER_CRITICAL(&update_lock);
    update_diagnostics.state = state;
    update_diagnostics.last_error = error;
    portEXIT_CRITICAL(&update_lock);
}

static esp_err_t inspect_image(const char *name, update_image_info_t *info,
                               bool *project_mismatch) {
    char path[UPDATE_PATH_BUFFER_SIZE];
    struct stat file_info;
    FILE *file = NULL;
    size_t descriptor_offset =
        sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);
    esp_err_t ret = ESP_OK;

    if (name == NULL || info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (project_mismatch != NULL) {
        *project_mismatch = false;
    }
    memset(info, 0, sizeof(*info));
    make_path(name, path, sizeof(path));
    if (stat(path, &file_info) != 0 || !S_ISREG(file_info.st_mode)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (file_info.st_size <= 0 ||
        (uint64_t) file_info.st_size > UPDATE_MAX_IMAGE_BYTES) {
        return ESP_ERR_INVALID_SIZE;
    }
    info->size = (size_t) file_info.st_size;
    file = fopen(path, "rb");
    if (file == NULL) {
        return ESP_FAIL;
    }
    if (fread(&info->header, 1, sizeof(info->header), file) !=
            sizeof(info->header) ||
        fseek(file, (long) descriptor_offset, SEEK_SET) != 0 ||
        fread(&info->descriptor, 1, sizeof(info->descriptor), file) !=
            sizeof(info->descriptor)) {
        ret = ESP_ERR_INVALID_SIZE;
    }
    if (fclose(file) != 0 && ret == ESP_OK) {
        ret = ESP_FAIL;
    }
    if (ret != ESP_OK) {
        return ret;
    }
    if (info->header.magic != ESP_IMAGE_HEADER_MAGIC ||
        info->header.chip_id != ESP_CHIP_ID_ESP32S3 ||
        info->descriptor.magic_word != ESP_APP_DESC_MAGIC_WORD ||
        memchr(info->descriptor.project_name, '\0',
               sizeof(info->descriptor.project_name)) == NULL ||
        memchr(info->descriptor.version, '\0',
               sizeof(info->descriptor.version)) == NULL) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (!firmware_update_policy_project_name_matches(
            info->descriptor.project_name,
            sizeof(info->descriptor.project_name),
            CBV_FIRMWARE_PROJECT_NAME)) {
        if (project_mismatch != NULL) {
            *project_mismatch = true;
        }
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static void update_led_task(void *argument) {
    bool yellow = false;
    (void) argument;
    while (update_led_running) {
        yellow = !yellow;
        board_set_status_leds(false, yellow);
        vTaskDelay(pdMS_TO_TICKS(UPDATE_LED_PERIOD_MS));
    }
    board_set_status_leds(false, false);
    vTaskDelete(NULL);
}

static void start_update_indicator(void) {
    update_led_running = true;
    board_set_status_leds(false, true);
    if (xTaskCreatePinnedToCore(update_led_task, "ota_led",
                                UPDATE_LED_TASK_STACK, NULL,
                                UPDATE_LED_TASK_PRIORITY,
                                NULL, 0) != pdPASS) {
        update_led_running = false;
    }
}

static void stop_update_indicator(void) {
    update_led_running = false;
    vTaskDelay(pdMS_TO_TICKS(UPDATE_LED_PERIOD_MS +
                            UPDATE_LED_STOP_MARGIN_MS));
    board_set_status_leds(false, false);
}

static esp_err_t reject_input(esp_err_t reason, const char *message) {
    esp_err_t move_result = move_state_file(UPDATE_INPUT_NAME,
                                            UPDATE_BAD_NAME);
    set_state(FIRMWARE_UPDATE_REJECTED, reason);
    (void) write_status("state=REJECTED\r\nreason=%s\r\nerror=%s\r\n",
                        message, esp_err_to_name(reason));
    if (move_result != ESP_OK) {
        return move_result;
    }
    return reason;
}

static void sanitize_project_name(const char *input, size_t input_capacity,
                                  char *output, size_t output_capacity) {
    size_t index = 0U;

    if (output_capacity == 0U) {
        return;
    }
    while (index + 1U < output_capacity && index < input_capacity &&
           input[index] != '\0') {
        unsigned char value = (unsigned char) input[index];
        output[index] = '?';
        if (value >= UPDATE_PRINTABLE_ASCII_MIN &&
            value <= UPDATE_PRINTABLE_ASCII_MAX) {
            output[index] = (char) value;
        }
        index++;
    }
    output[index] = '\0';
}

static esp_err_t reject_project_mismatch(const update_image_info_t *info) {
    char actual_project[sizeof(info->descriptor.project_name) + 1U];
    esp_err_t reason = ESP_ERR_INVALID_RESPONSE;
    esp_err_t move_result = ESP_FAIL;

    sanitize_project_name(info->descriptor.project_name,
                          sizeof(info->descriptor.project_name),
                          actual_project, sizeof(actual_project));
    move_result = move_state_file(UPDATE_INPUT_NAME, UPDATE_BAD_NAME);
    set_state(FIRMWARE_UPDATE_REJECTED, reason);
    (void) write_status(
        "state=REJECTED\r\n"
        "reason=firmware target mismatch\r\n"
        "expected_project=%s\r\n"
        "actual_project=%s\r\n"
        "error=%s\r\n",
        CBV_FIRMWARE_PROJECT_NAME, actual_project, esp_err_to_name(reason));
    if (move_result != ESP_OK) {
        return move_result;
    }
    return reason;
}

static esp_err_t reconcile_pending_file(const esp_app_desc_t *running_desc) {
    update_image_info_t pending_info;
    esp_err_t ret;

    if (!file_exists(UPDATE_PENDING_NAME)) {
        return ESP_OK;
    }
    ret = inspect_image(UPDATE_PENDING_NAME, &pending_info, NULL);
    if (ret == ESP_OK &&
        memcmp(pending_info.descriptor.app_elf_sha256,
               running_desc->app_elf_sha256,
               sizeof(running_desc->app_elf_sha256)) == 0) {
        ret = write_status("state=CONFIRMED\r\nversion=%s\r\n",
                           running_desc->version);
        if (ret == ESP_OK) {
            ret = remove_if_present(UPDATE_PENDING_NAME);
        }
        if (ret != ESP_OK) {
            set_state(FIRMWARE_UPDATE_PENDING_CONFIRMATION, ret);
            ESP_LOGE(TAG, "pending update cleanup failed: %s",
                     esp_err_to_name(ret));
            return ret;
        }
        set_state(FIRMWARE_UPDATE_CONFIRMED, ESP_OK);
        return ESP_OK;
    }

    (void) move_state_file(UPDATE_PENDING_NAME, UPDATE_BAD_NAME);
    set_state(FIRMWARE_UPDATE_ROLLED_BACK, ret);
    (void) write_status(
        "state=ROLLED_BACK\r\nreason=staged image is not running\r\n");
    return ESP_OK;
}

static esp_err_t apply_update(const update_image_info_t *info) {
    char input_path[UPDATE_PATH_BUFFER_SIZE];
    uint8_t *buffer = NULL;
    FILE *file = NULL;
    const esp_partition_t *target = NULL;
    esp_ota_handle_t ota_handle = 0;
    esp_err_t ret;
    bool ota_started = false;

    target = esp_ota_get_next_update_partition(NULL);
    if (target == NULL || info->size > target->size) {
        return ESP_ERR_INVALID_SIZE;
    }
    portENTER_CRITICAL(&update_lock);
    (void) snprintf(update_diagnostics.target_partition,
                    sizeof(update_diagnostics.target_partition), "%s",
                    target->label);
    portEXIT_CRITICAL(&update_lock);

    make_path(UPDATE_INPUT_NAME, input_path, sizeof(input_path));
    file = fopen(input_path, "rb");
    if (file == NULL) {
        return ESP_FAIL;
    }
    /*
     * CODING_RULES_DYNAMIC_MEMORY: OTA is a serialized, low-frequency
     * transaction. Reserving this internal-RAM-only buffer permanently would
     * reduce normal runtime headroom; it is bounded and freed before exit.
     */
    buffer = heap_caps_malloc(UPDATE_IO_BUFFER_BYTES,
                              MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (buffer == NULL) {
        (void) fclose(file);
        return ESP_ERR_NO_MEM;
    }

    start_update_indicator();
    set_state(FIRMWARE_UPDATE_WRITING, ESP_OK);
    (void) write_status(
        "state=WRITING\r\nversion=%s\r\nsize=%u\r\ntarget=%s\r\n",
        info->descriptor.version, (unsigned int) info->size, target->label);

    (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
    ret = esp_ota_begin(target, info->size, &ota_handle);
    (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
    if (ret == ESP_OK) {
        ota_started = true;
    }
    while (ret == ESP_OK) {
        size_t read_length = fread(buffer, 1, UPDATE_IO_BUFFER_BYTES, file);
        if (read_length > 0U) {
            ret = esp_ota_write(ota_handle, buffer, read_length);
            if (ret == ESP_OK) {
                portENTER_CRITICAL(&update_lock);
                update_diagnostics.bytes_written +=
                    (uint32_t) read_length;
                portEXIT_CRITICAL(&update_lock);
            }
        }
        if (read_length < UPDATE_IO_BUFFER_BYTES) {
            if (ferror(file)) {
                ret = ESP_FAIL;
            }
            break;
        }
        (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
    }
    (void) fclose(file);
    heap_caps_free(buffer);

    if (ret == ESP_OK) {
        (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
        ret = esp_ota_end(ota_handle);
        (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
        ota_started = false;
    }
    if (ota_started) {
        (void) esp_ota_abort(ota_handle);
    }
    stop_update_indicator();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = move_state_file(UPDATE_INPUT_NAME, UPDATE_PENDING_NAME);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = write_status(
        "state=STAGED\r\nversion=%s\r\nsize=%u\r\ntarget=%s\r\n",
        info->descriptor.version, (unsigned int) info->size, target->label);
    if (ret != ESP_OK) {
        (void) move_state_file(UPDATE_PENDING_NAME, UPDATE_BAD_NAME);
        return ret;
    }
    ret = esp_ota_set_boot_partition(target);
    if (ret != ESP_OK) {
        (void) move_state_file(UPDATE_PENDING_NAME, UPDATE_BAD_NAME);
        return ret;
    }

    set_state(FIRMWARE_UPDATE_STAGED, ESP_OK);
    usb_device_storage_end_app_io();
    ESP_LOGI(TAG, "firmware update staged in %s; restarting", target->label);
    esp_restart();
    return ESP_OK;
}

bool firmware_update_running_image_pending_verify(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state = ESP_OTA_IMG_UNDEFINED;

    return running != NULL &&
           esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
           ota_state == ESP_OTA_IMG_PENDING_VERIFY;
}

esp_err_t firmware_update_process_boot(bool external_power_present,
                                       bool battery_valid,
                                       float battery_voltage_v) {
    const esp_app_desc_t *running_desc = esp_app_get_description();
    update_image_info_t image_info;
    esp_err_t ret;
    bool project_mismatch = false;
    bool update_power_allowed = firmware_update_policy_power_allowed(
        external_power_present, battery_valid, battery_voltage_v);

    portENTER_CRITICAL(&update_lock);
    update_diagnostics.external_power_present = external_power_present;
    update_diagnostics.battery_valid = battery_valid;
    update_diagnostics.battery_voltage_v = battery_voltage_v;
    update_diagnostics.minimum_battery_voltage_v =
        FIRMWARE_UPDATE_MIN_BATTERY_V;
    update_diagnostics.update_power_allowed = update_power_allowed;
    portEXIT_CRITICAL(&update_lock);

    if (firmware_update_running_image_pending_verify()) {
        portENTER_CRITICAL(&update_lock);
        update_diagnostics.state = FIRMWARE_UPDATE_PENDING_CONFIRMATION;
        update_diagnostics.confirmation_required = true;
        portEXIT_CRITICAL(&update_lock);
        return ESP_OK;
    }

    ret = usb_device_storage_begin_app_io(UPDATE_STORAGE_TIMEOUT_MS);
    if (ret != ESP_OK) {
        set_state(FIRMWARE_UPDATE_STORAGE_BUSY, ret);
        return ret;
    }

    ret = reconcile_pending_file(running_desc);
    (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
    if (ret != ESP_OK) {
        usb_device_storage_end_app_io();
        return ret;
    }
    if (!file_exists(UPDATE_INPUT_NAME)) {
        usb_device_storage_end_app_io();
        return ESP_OK;
    }
    if (!update_power_allowed) {
        set_state(FIRMWARE_UPDATE_DEFERRED_POWER, ESP_OK);
        (void) write_status(
            "state=DEFERRED\r\n"
            "reason=USB power absent and battery not above threshold\r\n"
            "external_power=%d\r\n"
            "battery_valid=%d\r\n"
            "battery_v=%.2f\r\n"
            "threshold_v=%.2f\r\n",
            external_power_present, battery_valid,
            (double) battery_voltage_v,
            (double) FIRMWARE_UPDATE_MIN_BATTERY_V);
        usb_device_storage_end_app_io();
        return ESP_OK;
    }

    set_state(FIRMWARE_UPDATE_VALIDATING, ESP_OK);
    ret = inspect_image(UPDATE_INPUT_NAME, &image_info, &project_mismatch);
    (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
    if (ret != ESP_OK) {
        if (project_mismatch) {
            ret = reject_project_mismatch(&image_info);
        } else {
            ret = reject_input(ret, "invalid ESP32-S3 application image");
        }
        usb_device_storage_end_app_io();
        return ret;
    }
    record_image_info(&image_info);
    portENTER_CRITICAL(&update_lock);
    update_diagnostics.bytes_written = 0U;
    portEXIT_CRITICAL(&update_lock);

    ret = apply_update(&image_info);
    if (ret != ESP_OK) {
        (void) reject_input(ret, "OTA write or verification failed");
        usb_device_storage_end_app_io();
    }
    return ret;
}

static void confirmation_task(void *argument) {
    bool workers_started;
    const esp_app_desc_t *running_desc = NULL;
    esp_err_t ret;
    (void) argument;

    vTaskDelay(pdMS_TO_TICKS(UPDATE_CONFIRMATION_DELAY_MS));
    portENTER_CRITICAL(&update_lock);
    workers_started = update_diagnostics.required_workers_started;
    portEXIT_CRITICAL(&update_lock);

    if (!workers_started) {
        set_state(FIRMWARE_UPDATE_ROLLED_BACK, ESP_ERR_INVALID_STATE);
        ESP_LOGE(TAG, "required application workers did not start; rollback");
        (void) esp_ota_mark_app_invalid_rollback_and_reboot();
        vTaskDelete(NULL);
        return;
    }

    ret = esp_ota_mark_app_valid_cancel_rollback();
    if (ret != ESP_OK) {
        set_state(FIRMWARE_UPDATE_PENDING_CONFIRMATION, ret);
        ESP_LOGE(TAG, "OTA confirmation failed: %s", esp_err_to_name(ret));
        (void) esp_ota_mark_app_invalid_rollback_and_reboot();
        vTaskDelete(NULL);
        return;
    }

    ret = usb_device_storage_begin_app_io(UPDATE_STORAGE_TIMEOUT_MS);
    if (ret == ESP_OK) {
        running_desc = esp_app_get_description();
        ret = write_status("state=CONFIRMED\r\nversion=%s\r\n",
                           running_desc->version);
        if (ret == ESP_OK) {
            ret = remove_if_present(UPDATE_PENDING_NAME);
        }
        usb_device_storage_end_app_io();
    }
    if (ret != ESP_OK) {
        set_state(FIRMWARE_UPDATE_PENDING_CONFIRMATION, ret);
        ESP_LOGE(TAG, "OTA confirmed but pending-file cleanup failed: %s",
                 esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    set_state(FIRMWARE_UPDATE_CONFIRMED, ESP_OK);
    portENTER_CRITICAL(&update_lock);
    update_diagnostics.confirmation_required = false;
    portEXIT_CRITICAL(&update_lock);
    ESP_LOGI(TAG, "firmware update confirmed");
    vTaskDelete(NULL);
}

esp_err_t firmware_update_begin_confirmation(void) {
    bool confirmation_required;

    portENTER_CRITICAL(&update_lock);
    confirmation_required = update_diagnostics.confirmation_required;
    portEXIT_CRITICAL(&update_lock);
    if (!confirmation_required) {
        return ESP_OK;
    }
    if (xTaskCreatePinnedToCore(
            confirmation_task, "ota_confirm",
            UPDATE_CONFIRMATION_TASK_STACK, NULL,
            UPDATE_CONFIRMATION_TASK_PRIORITY, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "confirmation task creation failed; rollback");
        (void) esp_ota_mark_app_invalid_rollback_and_reboot();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t firmware_update_wait_for_confirmation(uint32_t timeout_ms) {
    const TickType_t poll_ticks = pdMS_TO_TICKS(UPDATE_CONFIRMATION_POLL_MS);
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    const TickType_t start_ticks = xTaskGetTickCount();

    for (;;) {
        firmware_update_state_t state;
        esp_err_t last_error;
        bool confirmation_required;

        portENTER_CRITICAL(&update_lock);
        state = update_diagnostics.state;
        last_error = update_diagnostics.last_error;
        confirmation_required = update_diagnostics.confirmation_required;
        portEXIT_CRITICAL(&update_lock);

        if (!confirmation_required) {
            if (state == FIRMWARE_UPDATE_CONFIRMED) {
                return ESP_OK;
            }
            return last_error;
        }
        if (state == FIRMWARE_UPDATE_PENDING_CONFIRMATION &&
            last_error != ESP_OK) {
            return last_error;
        }
        if ((xTaskGetTickCount() - start_ticks) >= timeout_ticks) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(poll_ticks);
    }
}

void firmware_update_mark_workers_started(void) {
    portENTER_CRITICAL(&update_lock);
    update_diagnostics.required_workers_started = true;
    portEXIT_CRITICAL(&update_lock);
}

void firmware_update_get_diagnostics(
    firmware_update_diagnostics_t *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }
    portENTER_CRITICAL(&update_lock);
    *diagnostics = update_diagnostics;
    portEXIT_CRITICAL(&update_lock);
}

const char *firmware_update_state_name(firmware_update_state_t state) {
    switch (state) {
        case FIRMWARE_UPDATE_DEFERRED_POWER:
            return "DEFERRED";
        case FIRMWARE_UPDATE_VALIDATING:
            return "VALIDATING";
        case FIRMWARE_UPDATE_WRITING:
            return "WRITING";
        case FIRMWARE_UPDATE_STAGED:
            return "STAGED";
        case FIRMWARE_UPDATE_PENDING_CONFIRMATION:
            return "PENDING_CONFIRMATION";
        case FIRMWARE_UPDATE_CONFIRMED:
            return "CONFIRMED";
        case FIRMWARE_UPDATE_REJECTED:
            return "REJECTED";
        case FIRMWARE_UPDATE_ROLLED_BACK:
            return "ROLLED_BACK";
        case FIRMWARE_UPDATE_STORAGE_BUSY:
            return "STORAGE_BUSY";
        case FIRMWARE_UPDATE_IDLE:
        default:
            return "IDLE";
    }
}
