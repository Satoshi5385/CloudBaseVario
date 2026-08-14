#include "platform/board_identity_storage.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"

#define BOARD_DATA_PARTITION "board_data"
#define BOARD_DATA_NAMESPACE "identity"
#define BOARD_DATA_KEY_SCHEMA "schema_ver"
#define BOARD_DATA_KEY_BOARD_ID "board_id"
#define BOARD_DATA_KEY_SERIAL "serial"
#define BOARD_DATA_EXPECTED_KEY_COUNT 3U

static void set_diagnostics(board_identity_storage_diagnostics_t *diagnostics,
                            board_identity_load_result_t result,
                            esp_err_t error) {
    if (diagnostics != NULL) {
        diagnostics->result = result;
        diagnostics->error = error;
    }
}

static bool key_is_expected(const nvs_entry_info_t *info) {
    if (info == NULL) {
        return false;
    }
    return (strcmp(info->key, BOARD_DATA_KEY_SCHEMA) == 0 &&
            info->type == NVS_TYPE_U8) ||
           (strcmp(info->key, BOARD_DATA_KEY_BOARD_ID) == 0 &&
            info->type == NVS_TYPE_U16) ||
           (strcmp(info->key, BOARD_DATA_KEY_SERIAL) == 0 &&
            info->type == NVS_TYPE_STR);
}

static bool namespace_has_exact_keys(void) {
    nvs_iterator_t iterator = NULL;
    esp_err_t result = nvs_entry_find(BOARD_DATA_PARTITION,
                                      BOARD_DATA_NAMESPACE,
                                      NVS_TYPE_ANY, &iterator);
    size_t count = 0U;

    while (result == ESP_OK) {
        nvs_entry_info_t info = {0};

        nvs_entry_info(iterator, &info);
        if (!key_is_expected(&info)) {
            nvs_release_iterator(iterator);
            return false;
        }
        count++;
        result = nvs_entry_next(&iterator);
    }
    nvs_release_iterator(iterator);
    return result == ESP_ERR_NVS_NOT_FOUND &&
           count == BOARD_DATA_EXPECTED_KEY_COUNT;
}

board_identity_load_result_t board_identity_storage_load(
    board_identity_t *identity,
    board_identity_storage_diagnostics_t *diagnostics) {
    nvs_handle_t handle = 0;
    size_t serial_size = BOARD_SERIAL_BUFFER_SIZE;
    esp_err_t result;

    if (identity == NULL) {
        set_diagnostics(diagnostics, BOARD_IDENTITY_LOAD_INVALID,
                        ESP_ERR_INVALID_ARG);
        return BOARD_IDENTITY_LOAD_INVALID;
    }
    memset(identity, 0, sizeof(*identity));
    result = nvs_flash_init_partition(BOARD_DATA_PARTITION);
    if (result != ESP_OK) {
        board_identity_load_result_t load_result =
            BOARD_IDENTITY_LOAD_IO_ERROR;

        if (result == ESP_ERR_NOT_FOUND) {
            load_result = BOARD_IDENTITY_LOAD_MISSING;
        }

        set_diagnostics(diagnostics, load_result, result);
        return load_result;
    }
    result = nvs_open_from_partition(BOARD_DATA_PARTITION,
                                     BOARD_DATA_NAMESPACE,
                                     NVS_READONLY, &handle);
    if (result != ESP_OK) {
        board_identity_load_result_t load_result =
            BOARD_IDENTITY_LOAD_IO_ERROR;

        if (result == ESP_ERR_NVS_NOT_FOUND) {
            load_result = BOARD_IDENTITY_LOAD_MISSING;
        }

        set_diagnostics(diagnostics, load_result, result);
        return load_result;
    }
    result = nvs_get_u8(handle, BOARD_DATA_KEY_SCHEMA,
                        &identity->schema_version);
    if (result == ESP_OK) {
        result = nvs_get_u16(handle, BOARD_DATA_KEY_BOARD_ID,
                             &identity->board_id);
    }
    if (result == ESP_OK) {
        result = nvs_get_str(handle, BOARD_DATA_KEY_SERIAL,
                             identity->serial, &serial_size);
    }
    nvs_close(handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        set_diagnostics(diagnostics, BOARD_IDENTITY_LOAD_MISSING, result);
        return BOARD_IDENTITY_LOAD_MISSING;
    }
    if (result != ESP_OK) {
        set_diagnostics(diagnostics, BOARD_IDENTITY_LOAD_INVALID, result);
        return BOARD_IDENTITY_LOAD_INVALID;
    }
    if (serial_size != BOARD_SERIAL_BUFFER_SIZE ||
        !namespace_has_exact_keys()) {
        set_diagnostics(diagnostics, BOARD_IDENTITY_LOAD_INVALID,
                        ESP_ERR_INVALID_RESPONSE);
        return BOARD_IDENTITY_LOAD_INVALID;
    }
    if (board_identity_descriptor(identity->board_id) == NULL) {
        set_diagnostics(diagnostics, BOARD_IDENTITY_LOAD_UNSUPPORTED,
                        ESP_ERR_NOT_SUPPORTED);
        return BOARD_IDENTITY_LOAD_UNSUPPORTED;
    }
    if (!board_identity_validate(identity)) {
        set_diagnostics(diagnostics, BOARD_IDENTITY_LOAD_INVALID,
                        ESP_ERR_INVALID_RESPONSE);
        return BOARD_IDENTITY_LOAD_INVALID;
    }
    set_diagnostics(diagnostics, BOARD_IDENTITY_LOAD_VALID, ESP_OK);
    return BOARD_IDENTITY_LOAD_VALID;
}

const char *board_identity_load_result_name(
    board_identity_load_result_t result) {
    switch (result) {
        case BOARD_IDENTITY_LOAD_VALID:
            return "VALID";
        case BOARD_IDENTITY_LOAD_MISSING:
            return "MISSING";
        case BOARD_IDENTITY_LOAD_INVALID:
            return "INVALID";
        case BOARD_IDENTITY_LOAD_UNSUPPORTED:
            return "UNSUPPORTED";
        case BOARD_IDENTITY_LOAD_IO_ERROR:
            return "IO_ERROR";
        default:
            return "UNKNOWN";
    }
}
