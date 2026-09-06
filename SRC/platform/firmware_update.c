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
#include "psa/crypto.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "domain/firmware_metadata.h"
#include "domain/firmware_pending_record.h"
#include "domain/firmware_update_policy.h"
#include "platform/board.h"
#include "platform/firmware_auth.h"
#include "platform/usb_device_service.h"
#include "platform/watchdog_service.h"

#ifndef CBV_FIRMWARE_PROJECT_NAME
#error "The build must provide the Aohazuku firmware project identity"
#endif

#define UPDATE_INPUT_NAME "UPDATE.BIN"
#define UPDATE_PENDING_NAME "UPDATE.PND"
#define UPDATE_BAD_NAME "UPDATE.BAD"
#define UPDATE_STATUS_NAME "UPDATE_RESULT.TXT"
#define UPDATE_STATUS_TEMP_NAME "UPDATE.TMP"
#define UPDATE_PENDING_TEMP_NAME "UPDATE.PTM"
#define UPDATE_MAX_IMAGE_BYTES FIRMWARE_AUTH_MAX_PAYLOAD_SIZE
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

typedef struct {
    const char *input_mount_path;
    const char *state_mount_path;
    firmware_update_source_t source;
} update_source_context_t;

static const update_source_context_t flash_source_context = {
    .input_mount_path = "/config",
    .state_mount_path = "/config",
    .source = FIRMWARE_UPDATE_SOURCE_FLASH,
};
static const update_source_context_t psram_source_context = {
    .input_mount_path = "/update",
    .state_mount_path = "/config",
    .source = FIRMWARE_UPDATE_SOURCE_PSRAM,
};
static const update_source_context_t *active_source = &flash_source_context;
static volatile bool authentication_vbus_lost;

static const char *TAG = "firmware_update";
static portMUX_TYPE update_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool update_led_running;
static firmware_update_diagnostics_t update_diagnostics = {
    .state = FIRMWARE_UPDATE_IDLE,
    .last_error = ESP_OK,
    .source = FIRMWARE_UPDATE_SOURCE_NONE,
};

static void report_authentication_progress(void *arg) {
    (void) arg;
    if (active_source->source == FIRMWARE_UPDATE_SOURCE_PSRAM &&
        !usb_device_vbus_present()) {
        authentication_vbus_lost = true;
    }
    (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
}

typedef struct {
    size_t size;
    size_t payload_offset;
    esp_image_header_t header;
    esp_app_desc_t descriptor;
    firmware_auth_header_t authentication;
    firmware_auth_failure_t auth_failure;
} update_image_info_t;

static void make_path_at(const char *mount_path, const char *name,
                         char *path, size_t capacity) {
    (void) snprintf(path, capacity, "%s/%s", mount_path, name);
}

static void make_input_path(const char *name, char *path, size_t capacity) {
    make_path_at(active_source->input_mount_path, name, path, capacity);
}

static void make_state_path(const char *name, char *path, size_t capacity) {
    make_path_at(active_source->state_mount_path, name, path, capacity);
}

static bool file_exists_at(const char *mount_path, const char *name) {
    char path[UPDATE_PATH_BUFFER_SIZE];
    struct stat info;
    make_path_at(mount_path, name, path, sizeof(path));
    return stat(path, &info) == 0 && S_ISREG(info.st_mode);
}

static bool input_file_exists(const char *name) {
    return file_exists_at(active_source->input_mount_path, name);
}

static bool state_file_exists(const char *name) {
    return file_exists_at(active_source->state_mount_path, name);
}

static esp_err_t remove_state_if_present(const char *name) {
    char path[UPDATE_PATH_BUFFER_SIZE];
    make_state_path(name, path, sizeof(path));
    if (unlink(path) == 0 || errno == ENOENT) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

static esp_err_t move_file_at(const char *mount_path, const char *from_name,
                              const char *to_name) {
    char from_path[UPDATE_PATH_BUFFER_SIZE];
    char to_path[UPDATE_PATH_BUFFER_SIZE];
    esp_err_t result = ESP_FAIL;

    make_path_at(mount_path, from_name, from_path, sizeof(from_path));
    make_path_at(mount_path, to_name, to_path, sizeof(to_path));
    (void) unlink(to_path);
    if (rename(from_path, to_path) == 0) {
        result = ESP_OK;
    }
    return result;
}

static esp_err_t move_input_file(const char *from_name, const char *to_name) {
    return move_file_at(active_source->input_mount_path, from_name, to_name);
}

static esp_err_t move_state_file(const char *from_name, const char *to_name) {
    return move_file_at(active_source->state_mount_path, from_name, to_name);
}

static esp_err_t write_text_atomic_at(const char *mount_path,
                                      const char *filename,
                                      const char *temp_filename,
                                      const void *contents,
                                      size_t content_length) {
    char path[UPDATE_PATH_BUFFER_SIZE];
    char temp_path[UPDATE_PATH_BUFFER_SIZE];
    FILE *file;
    esp_err_t ret = ESP_OK;

    make_path_at(mount_path, filename, path, sizeof(path));
    make_path_at(mount_path, temp_filename, temp_path, sizeof(temp_path));
    file = fopen(temp_path, "wb");
    if (file == NULL) {
        return ESP_FAIL;
    }
    if (fwrite(contents, 1U, content_length, file) != content_length ||
        fflush(file) != 0 || fsync(fileno(file)) != 0) {
        ret = ESP_FAIL;
    }
    if (fclose(file) != 0) {
        ret = ESP_FAIL;
    }
    if (ret == ESP_OK) {
        (void) unlink(path);
        if (rename(temp_path, path) != 0) {
            ret = ESP_FAIL;
        }
    } else {
        (void) unlink(temp_path);
    }
    return ret;
}

static esp_err_t write_status(const char *format, ...) {
    char text[UPDATE_STATUS_TEXT_SIZE];
    va_list arguments;
    int length;
    int source_length;
    esp_err_t ret;

    va_start(arguments, format);
    length = vsnprintf(text, sizeof(text), format, arguments);
    va_end(arguments);
    if (length < 0 || (size_t) length >= sizeof(text)) {
        return ESP_ERR_INVALID_SIZE;
    }
    source_length = snprintf(text + length, sizeof(text) - (size_t) length,
                             "source=%s\r\n",
                             firmware_update_source_name(
                                 active_source->source));
    if (source_length < 0 ||
        (size_t) source_length >= sizeof(text) - (size_t) length) {
        return ESP_ERR_INVALID_SIZE;
    }
    length += source_length;
    ret = write_text_atomic_at(active_source->state_mount_path,
                               UPDATE_STATUS_NAME, UPDATE_STATUS_TEMP_NAME,
                               text, (size_t) length);
    if (ret == ESP_OK &&
        active_source->source == FIRMWARE_UPDATE_SOURCE_PSRAM) {
        (void) write_text_atomic_at(active_source->input_mount_path,
                                    UPDATE_STATUS_NAME,
                                    UPDATE_STATUS_TEMP_NAME,
                                    text, (size_t) length);
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

static void record_descriptor_info(const esp_app_desc_t *descriptor) {
    firmware_metadata_t metadata;

    (void) firmware_metadata_parse(descriptor->version,
                                   sizeof(descriptor->version),
                                   &metadata);
    portENTER_CRITICAL(&update_lock);
    (void) snprintf(update_diagnostics.image_version,
                    sizeof(update_diagnostics.image_version), "%s",
                    metadata.version);
    (void) snprintf(update_diagnostics.image_hash,
                    sizeof(update_diagnostics.image_hash), "%s",
                    metadata.git_hash);
    bytes_to_hex(descriptor->app_elf_sha256,
                 sizeof(descriptor->app_elf_sha256),
                 update_diagnostics.image_fingerprint,
                 sizeof(update_diagnostics.image_fingerprint));
    portEXIT_CRITICAL(&update_lock);
}

static void record_image_info(const update_image_info_t *info) {
    record_descriptor_info(&info->descriptor);
    portENTER_CRITICAL(&update_lock);
    update_diagnostics.image_size_bytes = (uint32_t) info->size;
    portEXIT_CRITICAL(&update_lock);
}

static void set_state(firmware_update_state_t state, esp_err_t error) {
    portENTER_CRITICAL(&update_lock);
    update_diagnostics.state = state;
    update_diagnostics.last_error = error;
    portEXIT_CRITICAL(&update_lock);
}

static esp_err_t inspect_image_at(const char *mount_path, const char *name,
                                  update_image_info_t *info,
                                  bool *project_mismatch) {
    char path[UPDATE_PATH_BUFFER_SIZE];
    struct stat file_info;
    FILE *file = NULL;
    size_t descriptor_offset;
    esp_err_t ret = ESP_OK;

    if (name == NULL || info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (project_mismatch != NULL) {
        *project_mismatch = false;
    }
    memset(info, 0, sizeof(*info));
    make_path_at(mount_path, name, path, sizeof(path));
    if (stat(path, &file_info) != 0 || !S_ISREG(file_info.st_mode)) {
        return ESP_ERR_NOT_FOUND;
    }
    if (file_info.st_size <= (off_t) FIRMWARE_AUTH_HEADER_SIZE ||
        (uint64_t) file_info.st_size >
            (uint64_t) UPDATE_MAX_IMAGE_BYTES + FIRMWARE_AUTH_HEADER_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }
    file = fopen(path, "rb");
    if (file == NULL) {
        return ESP_FAIL;
    }
    ret = firmware_auth_verify_package(file, (size_t) file_info.st_size,
                                       CBV_FIRMWARE_PROJECT_NAME,
                                       &info->authentication,
                                       &info->auth_failure,
                                       report_authentication_progress, NULL);
    if (ret == ESP_OK) {
        info->size = info->authentication.payload_size;
        info->payload_offset = info->authentication.header_size;
        descriptor_offset = info->payload_offset + sizeof(esp_image_header_t) +
                            sizeof(esp_image_segment_header_t);
    }
    if (ret == ESP_OK &&
        (fseek(file, (long) info->payload_offset, SEEK_SET) != 0 ||
         fread(&info->header, 1, sizeof(info->header), file) !=
             sizeof(info->header) ||
         fseek(file, (long) descriptor_offset, SEEK_SET) != 0 ||
         fread(&info->descriptor, 1, sizeof(info->descriptor), file) !=
             sizeof(info->descriptor))) {
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

static esp_err_t inspect_input_image(const char *name,
                                     update_image_info_t *info,
                                     bool *project_mismatch) {
    return inspect_image_at(active_source->input_mount_path, name, info,
                            project_mismatch);
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

static esp_err_t reject_input(esp_err_t reason, const char *message,
                              const update_image_info_t *info) {
    esp_err_t move_result = move_input_file(UPDATE_INPUT_NAME,
                                            UPDATE_BAD_NAME);
    firmware_metadata_t metadata;

    if (info != NULL) {
        (void) firmware_metadata_parse(info->descriptor.version,
                                       sizeof(info->descriptor.version),
                                       &metadata);
    } else {
        (void) firmware_metadata_parse(NULL, 0U, &metadata);
    }
    set_state(FIRMWARE_UPDATE_REJECTED, reason);
    (void) write_status(
        "state=REJECTED\r\nreason=%s\r\nerror=%s\r\n"
        "version=%s\r\nhash=%s\r\n",
        message, esp_err_to_name(reason), metadata.version,
        metadata.git_hash);
    if (move_result != ESP_OK) {
        return move_result;
    }
    return reason;
}

static const char *authentication_failure_reason(
    firmware_auth_failure_t failure) {
    switch (failure) {
    case FIRMWARE_AUTH_FAILURE_PAYLOAD_HASH_MISMATCH:
        return "firmware payload hash mismatch";
    case FIRMWARE_AUTH_FAILURE_SIGNATURE_INVALID:
        return "firmware signature verification failed";
    case FIRMWARE_AUTH_FAILURE_NONE:
    case FIRMWARE_AUTH_FAILURE_OTHER:
        return "firmware authentication failed";
    }
    return "firmware authentication failed";
}

static esp_err_t reject_authentication(
    esp_err_t reason, firmware_auth_failure_t auth_failure) {
    set_state(FIRMWARE_UPDATE_REJECTED, reason);
    (void) write_status(
        "state=REJECTED\r\n"
        "reason=%s\r\n"
        "error=%s\r\n"
        "version=-\r\nhash=-\r\n",
        authentication_failure_reason(auth_failure),
        esp_err_to_name(reason));
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
    firmware_metadata_t metadata;
    esp_err_t reason = ESP_ERR_INVALID_RESPONSE;
    esp_err_t move_result = ESP_FAIL;

    sanitize_project_name(info->descriptor.project_name,
                          sizeof(info->descriptor.project_name),
                          actual_project, sizeof(actual_project));
    (void) firmware_metadata_parse(info->descriptor.version,
                                   sizeof(info->descriptor.version),
                                   &metadata);
    move_result = move_input_file(UPDATE_INPUT_NAME, UPDATE_BAD_NAME);
    set_state(FIRMWARE_UPDATE_REJECTED, reason);
    (void) write_status(
        "state=REJECTED\r\n"
        "reason=firmware target mismatch\r\n"
        "expected_project=%s\r\n"
        "actual_project=%s\r\n"
        "error=%s\r\n"
        "version=%s\r\nhash=%s\r\n",
        CBV_FIRMWARE_PROJECT_NAME, actual_project, esp_err_to_name(reason),
        metadata.version, metadata.git_hash);
    if (move_result != ESP_OK) {
        return move_result;
    }
    return reason;
}

static esp_err_t read_pending_record(firmware_pending_record_t *record,
                                     bool *is_record) {
    char path[UPDATE_PATH_BUFFER_SIZE];
    struct stat info;
    FILE *file;
    esp_err_t ret = ESP_OK;

    if (record == NULL || is_record == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *is_record = false;
    make_state_path(UPDATE_PENDING_NAME, path, sizeof(path));
    if (stat(path, &info) != 0 || !S_ISREG(info.st_mode)) {
        return ESP_ERR_NOT_FOUND;
    }
    if ((size_t) info.st_size != sizeof(*record)) {
        return ESP_OK;
    }
    *is_record = true;
    file = fopen(path, "rb");
    if (file == NULL) {
        return ESP_FAIL;
    }
    if (fread(record, 1U, sizeof(*record), file) != sizeof(*record)) {
        ret = ESP_FAIL;
    }
    if (fclose(file) != 0) {
        ret = ESP_FAIL;
    }
    if (ret != ESP_OK) {
        return ret;
    }
    if (!firmware_pending_record_validate(record,
                                          UPDATE_MAX_IMAGE_BYTES)) {
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
}

static bool pending_record_matches_partition(
    const firmware_pending_record_t *record,
    const esp_partition_t *partition,
    const esp_app_desc_t *descriptor) {
    if (record == NULL || partition == NULL || descriptor == NULL ||
        partition->size < FIRMWARE_AUTH_RECORD_SIZE) {
        return false;
    }
    return firmware_pending_record_matches(
        record, partition->address,
        partition->size - FIRMWARE_AUTH_RECORD_SIZE, partition->label,
        descriptor->app_elf_sha256);
}

static esp_err_t write_pending_record(const esp_partition_t *target,
                                      const update_image_info_t *info) {
    firmware_pending_record_t record;

    if (target == NULL || info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (state_file_exists(UPDATE_PENDING_NAME)) {
        return ESP_ERR_INVALID_STATE;
    }
    firmware_pending_record_initialize(
        &record, (uint32_t) info->size, target->address,
        info->descriptor.app_elf_sha256, info->descriptor.version,
        target->label);
    return write_text_atomic_at(active_source->state_mount_path,
                                UPDATE_PENDING_NAME,
                                UPDATE_PENDING_TEMP_NAME,
                                &record, sizeof(record));
}

static esp_err_t reconcile_pending_file(const esp_app_desc_t *running_desc) {
    update_image_info_t pending_info;
    firmware_pending_record_t pending_record = {0};
    esp_app_desc_t invalid_desc = {0};
    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    const esp_partition_t *last_invalid = esp_ota_get_last_invalid_partition();
    firmware_metadata_t metadata;
    esp_err_t ret;
    bool is_record = false;

    if (!state_file_exists(UPDATE_PENDING_NAME)) {
        return ESP_OK;
    }
    ret = read_pending_record(&pending_record, &is_record);
    if (is_record) {
        active_source = &psram_source_context;
        portENTER_CRITICAL(&update_lock);
        update_diagnostics.source = FIRMWARE_UPDATE_SOURCE_PSRAM;
        portEXIT_CRITICAL(&update_lock);
        if (ret != ESP_OK) {
            (void) move_state_file(UPDATE_PENDING_NAME, UPDATE_BAD_NAME);
            set_state(FIRMWARE_UPDATE_REJECTED, ret);
            (void) write_status(
                "state=REJECTED\r\nreason=pending record invalid\r\n"
                "error=%s\r\nversion=-\r\nhash=-\r\n",
                esp_err_to_name(ret));
            return ESP_OK;
        }
        if (pending_record_matches_partition(
                &pending_record, running_partition, running_desc)) {
            (void) firmware_metadata_parse(running_desc->version,
                                           sizeof(running_desc->version),
                                           &metadata);
            ret = write_status(
                "state=CONFIRMED\r\nversion=%s\r\nhash=%s\r\n",
                metadata.version, metadata.git_hash);
            record_descriptor_info(running_desc);
            if (ret == ESP_OK) {
                ret = remove_state_if_present(UPDATE_PENDING_NAME);
            }
            if (ret != ESP_OK) {
                set_state(FIRMWARE_UPDATE_PENDING_CONFIRMATION, ret);
                return ret;
            }
            set_state(FIRMWARE_UPDATE_CONFIRMED, ESP_OK);
            return ESP_OK;
        }
        if (last_invalid != NULL &&
            esp_ota_get_partition_description(last_invalid,
                                              &invalid_desc) == ESP_OK &&
            pending_record_matches_partition(
                &pending_record, last_invalid, &invalid_desc)) {
            (void) move_state_file(UPDATE_PENDING_NAME, UPDATE_BAD_NAME);
            set_state(FIRMWARE_UPDATE_ROLLED_BACK, ESP_OK);
            (void) firmware_metadata_parse(pending_record.version,
                                           sizeof(pending_record.version),
                                           &metadata);
            (void) write_status(
                "state=ROLLED_BACK\r\nreason=staged image rolled back\r\n"
                "version=%s\r\nhash=%s\r\n",
                metadata.version, metadata.git_hash);
            return ESP_OK;
        }
        (void) move_state_file(UPDATE_PENDING_NAME, UPDATE_BAD_NAME);
        set_state(FIRMWARE_UPDATE_ROLLED_BACK, ESP_ERR_INVALID_STATE);
        (void) write_status(
            "state=ROLLED_BACK\r\n"
            "reason=staged image is not running\r\n"
            "version=%s\r\nhash=-\r\n",
            pending_record.version);
        return ESP_OK;
    }
    if (ret != ESP_OK) {
        return ret;
    }
    ret = inspect_image_at(active_source->state_mount_path,
                           UPDATE_PENDING_NAME, &pending_info, NULL);
    if (ret == ESP_OK &&
        memcmp(pending_info.descriptor.app_elf_sha256,
               running_desc->app_elf_sha256,
               sizeof(running_desc->app_elf_sha256)) == 0) {
        (void) firmware_metadata_parse(running_desc->version,
                                       sizeof(running_desc->version),
                                       &metadata);
        ret = write_status(
            "state=CONFIRMED\r\nversion=%s\r\nhash=%s\r\n",
            metadata.version, metadata.git_hash);
        record_descriptor_info(running_desc);
        if (ret == ESP_OK) {
            ret = remove_state_if_present(UPDATE_PENDING_NAME);
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
    if (ret == ESP_OK) {
        (void) firmware_metadata_parse(pending_info.descriptor.version,
                                       sizeof(pending_info.descriptor.version),
                                       &metadata);
    } else {
        (void) firmware_metadata_parse(NULL, 0U, &metadata);
    }
    (void) write_status(
        "state=ROLLED_BACK\r\nreason=staged image is not running\r\n"
        "version=%s\r\nhash=%s\r\n",
        metadata.version, metadata.git_hash);
    return ESP_OK;
}

static esp_err_t apply_update(const update_image_info_t *info) {
    char input_path[UPDATE_PATH_BUFFER_SIZE];
    uint8_t digest[FIRMWARE_AUTH_SHA256_LENGTH];
    uint8_t *buffer = NULL;
    FILE *file = NULL;
    const esp_partition_t *target = NULL;
    esp_ota_handle_t ota_handle = 0;
    psa_hash_operation_t hash_operation = PSA_HASH_OPERATION_INIT;
    size_t digest_length = 0U;
    size_t remaining;
    esp_err_t ret;
    bool hash_started = false;
    bool ota_started = false;
    firmware_metadata_t metadata;

    (void) firmware_metadata_parse(info->descriptor.version,
                                   sizeof(info->descriptor.version),
                                   &metadata);

    target = esp_ota_get_next_update_partition(NULL);
    if (target == NULL || target->size < FIRMWARE_AUTH_RECORD_SIZE ||
        info->size > target->size - FIRMWARE_AUTH_RECORD_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }
    portENTER_CRITICAL(&update_lock);
    (void) snprintf(update_diagnostics.target_partition,
                    sizeof(update_diagnostics.target_partition), "%s",
                    target->label);
    portEXIT_CRITICAL(&update_lock);

    make_input_path(UPDATE_INPUT_NAME, input_path, sizeof(input_path));
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
    if (psa_crypto_init() != PSA_SUCCESS ||
        psa_hash_setup(&hash_operation, PSA_ALG_SHA_256) != PSA_SUCCESS) {
        (void) fclose(file);
        heap_caps_free(buffer);
        return ESP_FAIL;
    }
    hash_started = true;

    set_state(FIRMWARE_UPDATE_WRITING, ESP_OK);
    ret = write_status(
        "state=WRITING\r\nversion=%s\r\nhash=%s\r\n"
        "size=%u\r\ntarget=%s\r\n",
        metadata.version, metadata.git_hash, (unsigned int) info->size,
        target->label);
    if (ret != ESP_OK) {
        (void) psa_hash_abort(&hash_operation);
        (void) fclose(file);
        heap_caps_free(buffer);
        return ret;
    }
    start_update_indicator();

    (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
    ret = esp_ota_begin(target, info->size, &ota_handle);
    (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
    if (ret == ESP_OK) {
        ota_started = true;
    }
    if (ret == ESP_OK && fseek(file, (long) info->payload_offset, SEEK_SET) != 0) {
        ret = ESP_FAIL;
    }
    remaining = info->size;
    while (ret == ESP_OK && remaining > 0U) {
        size_t read_length = remaining;

        if (active_source->source == FIRMWARE_UPDATE_SOURCE_PSRAM &&
            !usb_device_vbus_present()) {
            ret = ESP_ERR_INVALID_STATE;
            break;
        }
        if (read_length > UPDATE_IO_BUFFER_BYTES) {
            read_length = UPDATE_IO_BUFFER_BYTES;
        }
        if (fread(buffer, 1U, read_length, file) != read_length) {
            ret = ESP_FAIL;
            break;
        }
        if (psa_hash_update(&hash_operation, buffer, read_length) !=
            PSA_SUCCESS) {
            ret = ESP_FAIL;
            break;
        }
        ret = esp_ota_write(ota_handle, buffer, read_length);
        if (ret == ESP_OK) {
            remaining -= read_length;
            portENTER_CRITICAL(&update_lock);
            update_diagnostics.bytes_written += (uint32_t) read_length;
            portEXIT_CRITICAL(&update_lock);
        }
        (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
    }
    if (fclose(file) != 0 && ret == ESP_OK) {
        ret = ESP_FAIL;
    }
    heap_caps_free(buffer);

    if (ret == ESP_OK) {
        if (psa_hash_finish(&hash_operation, digest, sizeof(digest),
                            &digest_length) != PSA_SUCCESS ||
            digest_length != sizeof(digest)) {
            ret = ESP_FAIL;
        } else {
            hash_started = false;
            if (memcmp(digest, info->authentication.payload_sha256,
                       sizeof(digest)) != 0) {
                ret = ESP_ERR_INVALID_CRC;
            } else {
                portENTER_CRITICAL(&update_lock);
                update_diagnostics.transfer_digest_verified = true;
                portEXIT_CRITICAL(&update_lock);
            }
        }
    }
    if (hash_started) {
        (void) psa_hash_abort(&hash_operation);
    }
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
    if (active_source->source == FIRMWARE_UPDATE_SOURCE_PSRAM &&
        !usb_device_vbus_present()) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = esp_partition_erase_range(target,
                                    target->size - FIRMWARE_AUTH_RECORD_SIZE,
                                    FIRMWARE_AUTH_RECORD_SIZE);
    if (ret == ESP_OK) {
        ret = esp_partition_write(target,
                                  target->size - FIRMWARE_AUTH_RECORD_SIZE,
                                  &info->authentication,
                                  sizeof(info->authentication));
    }
    if (ret != ESP_OK) {
        return ret;
    }

    if (active_source->source == FIRMWARE_UPDATE_SOURCE_PSRAM) {
        if (file_exists_at(active_source->state_mount_path,
                           UPDATE_INPUT_NAME)) {
            ret = move_file_at(active_source->state_mount_path,
                               UPDATE_INPUT_NAME, UPDATE_BAD_NAME);
            if (ret != ESP_OK) {
                return ret;
            }
        }
        ret = write_pending_record(target, info);
    } else {
        ret = move_input_file(UPDATE_INPUT_NAME, UPDATE_PENDING_NAME);
    }
    if (ret != ESP_OK) {
        return ret;
    }
    ret = write_status(
        "state=STAGED\r\nversion=%s\r\nhash=%s\r\n"
        "size=%u\r\ntarget=%s\r\n",
        metadata.version, metadata.git_hash, (unsigned int) info->size,
        target->label);
    if (ret != ESP_OK) {
        (void) move_state_file(UPDATE_PENDING_NAME, UPDATE_BAD_NAME);
        return ret;
    }
    if (active_source->source == FIRMWARE_UPDATE_SOURCE_PSRAM &&
        !usb_device_vbus_present()) {
        (void) move_state_file(UPDATE_PENDING_NAME, UPDATE_BAD_NAME);
        return ESP_ERR_INVALID_STATE;
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

static esp_err_t process_update(const update_source_context_t *source,
                                bool external_power_present,
                                bool battery_valid,
                                float battery_voltage_v,
                                bool report_missing_input) {
    const esp_app_desc_t *running_desc = esp_app_get_description();
    update_image_info_t image_info;
    esp_err_t ret;
    bool project_mismatch = false;
    bool update_power_allowed;

    if (source == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    active_source = source;
    update_power_allowed = firmware_update_policy_power_allowed(
        external_power_present, battery_valid, battery_voltage_v);

    portENTER_CRITICAL(&update_lock);
    update_diagnostics.external_power_present = external_power_present;
    update_diagnostics.battery_valid = battery_valid;
    update_diagnostics.battery_voltage_v = battery_voltage_v;
    update_diagnostics.minimum_battery_voltage_v =
        FIRMWARE_UPDATE_MIN_BATTERY_V;
    update_diagnostics.update_power_allowed = update_power_allowed;
    update_diagnostics.transfer_digest_verified = false;
    update_diagnostics.source = source->source;
    portEXIT_CRITICAL(&update_lock);

    if (firmware_update_running_image_pending_verify()) {
        record_descriptor_info(running_desc);
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
    active_source = source;
    portENTER_CRITICAL(&update_lock);
    update_diagnostics.source = source->source;
    portEXIT_CRITICAL(&update_lock);
    if (!input_file_exists(UPDATE_INPUT_NAME)) {
        usb_device_storage_end_app_io();
        if (report_missing_input) {
            return ESP_ERR_NOT_FOUND;
        }
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
            "threshold_v=%.2f\r\n"
            "version=-\r\nhash=-\r\n",
            external_power_present, battery_valid,
            (double) battery_voltage_v,
            (double) FIRMWARE_UPDATE_MIN_BATTERY_V);
        usb_device_storage_end_app_io();
        return ESP_OK;
    }

    set_state(FIRMWARE_UPDATE_VALIDATING, ESP_OK);
    authentication_vbus_lost = false;
    ret = inspect_input_image(UPDATE_INPUT_NAME, &image_info,
                              &project_mismatch);
    (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
    if (authentication_vbus_lost) {
        ret = ESP_ERR_INVALID_STATE;
        set_state(FIRMWARE_UPDATE_REJECTED, ret);
        (void) write_status(
            "state=REJECTED\r\nreason=USB VBUS lost during validation\r\n"
            "error=%s\r\nversion=-\r\nhash=-\r\n",
            esp_err_to_name(ret));
        usb_device_storage_end_app_io();
        return ret;
    }
    if (ret != ESP_OK) {
        if (project_mismatch) {
            ret = reject_project_mismatch(&image_info);
        } else {
            ret = reject_authentication(ret, image_info.auth_failure);
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
        (void) reject_input(ret, "OTA write or verification failed",
                            &image_info);
        usb_device_storage_end_app_io();
    }
    return ret;
}

esp_err_t firmware_update_process_boot(bool external_power_present,
                                       bool battery_valid,
                                       float battery_voltage_v) {
    return process_update(&flash_source_context, external_power_present,
                          battery_valid,
                          battery_voltage_v, false);
}

esp_err_t firmware_update_process_recovery(bool external_power_present) {
    if (!external_power_present || !usb_device_vbus_present() ||
        !usb_device_recovery_storage_ready()) {
        return ESP_ERR_INVALID_STATE;
    }
    return process_update(&psram_source_context, true, false, 0.0f, true);
}

static void confirmation_task(void *argument) {
    bool workers_started;
    bool is_record = false;
    const esp_app_desc_t *running_desc = NULL;
    const esp_partition_t *running_partition = NULL;
    firmware_pending_record_t pending_record = {0};
    esp_err_t record_result;
    esp_err_t ret;
    firmware_metadata_t metadata;
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

    ret = usb_device_storage_begin_app_io(UPDATE_STORAGE_TIMEOUT_MS);
    if (ret != ESP_OK) {
        set_state(FIRMWARE_UPDATE_PENDING_CONFIRMATION, ret);
        ESP_LOGE(TAG, "OTA confirmation storage unavailable: %s",
                 esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    active_source = &flash_source_context;
    running_desc = esp_app_get_description();
    running_partition = esp_ota_get_running_partition();
    record_result = read_pending_record(&pending_record, &is_record);
    if (is_record) {
        active_source = &psram_source_context;
        portENTER_CRITICAL(&update_lock);
        update_diagnostics.source = FIRMWARE_UPDATE_SOURCE_PSRAM;
        portEXIT_CRITICAL(&update_lock);
        if (record_result != ESP_OK ||
            !pending_record_matches_partition(
                &pending_record, running_partition, running_desc)) {
            if (record_result == ESP_OK) {
                record_result = ESP_ERR_INVALID_STATE;
            }
            (void) move_state_file(UPDATE_PENDING_NAME, UPDATE_BAD_NAME);
            (void) write_status(
                "state=REJECTED\r\nreason=pending record invalid\r\n"
                "error=%s\r\nversion=-\r\nhash=-\r\n",
                esp_err_to_name(record_result));
            usb_device_storage_end_app_io();
            set_state(FIRMWARE_UPDATE_PENDING_CONFIRMATION, record_result);
            ESP_LOGE(TAG, "OTA pending record validation failed: %s",
                     esp_err_to_name(record_result));
            vTaskDelete(NULL);
            return;
        }
    }

    ret = esp_ota_mark_app_valid_cancel_rollback();
    if (ret != ESP_OK) {
        usb_device_storage_end_app_io();
        set_state(FIRMWARE_UPDATE_PENDING_CONFIRMATION, ret);
        ESP_LOGE(TAG, "OTA confirmation failed: %s", esp_err_to_name(ret));
        (void) esp_ota_mark_app_invalid_rollback_and_reboot();
        vTaskDelete(NULL);
        return;
    }

    (void) firmware_metadata_parse(running_desc->version,
                                   sizeof(running_desc->version),
                                   &metadata);
    ret = write_status(
        "state=CONFIRMED\r\nversion=%s\r\nhash=%s\r\n",
        metadata.version, metadata.git_hash);
    if (ret == ESP_OK) {
        ret = remove_state_if_present(UPDATE_PENDING_NAME);
    }
    usb_device_storage_end_app_io();
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

const char *firmware_update_source_name(firmware_update_source_t source) {
    switch (source) {
        case FIRMWARE_UPDATE_SOURCE_FLASH:
            return "FLASH";
        case FIRMWARE_UPDATE_SOURCE_PSRAM:
            return "PSRAM";
        case FIRMWARE_UPDATE_SOURCE_NONE:
        default:
            return "NONE";
    }
}
