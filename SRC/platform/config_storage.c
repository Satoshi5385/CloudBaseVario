
#include "platform/config_storage.h"

#include "platform/config_json.h"

#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define CONFIG_MAX_FILE_BYTES (32U * 1024U)
#define CONFIG_PATH_BUFFER_SIZE 96U

/*
 * Keep the multi-profile work buffers out of the caller task's stack.
 * usb_device_service serializes storage I/O, and startup loading completes
 * before worker tasks are created.
 */
static struct {
    app_config_profiles_t recovered;
    app_config_profiles_t canonical;
    app_config_profiles_t verified;
} config_storage_workspace;

static bool make_path(const char *base_path, const char *filename,
                      char path[CONFIG_PATH_BUFFER_SIZE]) {
    int written = 0;

    if (base_path == NULL || filename == NULL || path == NULL) {
        return false;
    }
    written = snprintf(path, CONFIG_PATH_BUFFER_SIZE, "%s/%s", base_path,
                       filename);
    return written > 0 && (size_t) written < CONFIG_PATH_BUFFER_SIZE;
}
static config_load_result_t load_path(
    const char *path, app_config_profiles_t *profiles,
    config_storage_diagnostics_t *diagnostics) {
    struct stat file_status = {0};
    FILE *file = NULL;
    char *contents = NULL;
    size_t bytes_read = 0U;
    config_load_result_t result = CONFIG_LOAD_IO_ERROR;

    if (stat(path, &file_status) != 0) {
        if (errno == ENOENT) {
            return CONFIG_LOAD_DEFAULT_NO_FILE;
        }
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_IO_ERROR, NULL,
                        errno);
        return CONFIG_LOAD_IO_ERROR;
    }
    if (file_status.st_size <= 0) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_EMPTY_FILE, NULL, 0);
        return CONFIG_LOAD_INVALID_FILE;
    }
    if ((uint64_t) file_status.st_size > CONFIG_MAX_FILE_BYTES) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_FILE_TOO_LARGE, NULL,
                        0);
        return CONFIG_LOAD_INVALID_FILE;
    }

    /*
     * CODING_RULES_DYNAMIC_MEMORY: configuration files are bounded by
     * CONFIG_MAX_FILE_BYTES and loaded only during serialized storage
     * transactions. The exact-size buffer is released before every return.
     */
    contents = malloc((size_t) file_status.st_size + 1U);
    if (contents == NULL) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_NO_MEMORY, NULL,
                        ENOMEM);
        return CONFIG_LOAD_IO_ERROR;
    }
    file = fopen(path, "rb");
    if (file == NULL) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_IO_ERROR, NULL,
                        errno);
        free(contents);
        return CONFIG_LOAD_IO_ERROR;
    }
    bytes_read =
        fread(contents, 1U, (size_t) file_status.st_size, file);
    if (ferror(file) != 0 || bytes_read != (size_t) file_status.st_size) {
        int io_error = EIO;

        if (ferror(file) != 0) {
            io_error = errno;
        }
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_IO_ERROR, NULL,
                        io_error);
        result = CONFIG_LOAD_IO_ERROR;
    } else {
        contents[bytes_read] = '\0';
        result = CONFIG_LOAD_INVALID_FILE;
        if (config_json_parse(contents, bytes_read, profiles, diagnostics)) {
            result = CONFIG_LOAD_VALID_FILE;
        }
    }
    (void) fclose(file);
    free(contents);
    return result;
}

config_load_result_t config_storage_load(const char *base_path,
                                         app_config_profiles_t *profiles,
                                         config_storage_diagnostics_t *diagnostics) {
    char path[CONFIG_PATH_BUFFER_SIZE] = {0};
    char backup_path[CONFIG_PATH_BUFFER_SIZE] = {0};
    app_config_profiles_t *recovered =
        &config_storage_workspace.recovered;

    config_load_result_t result = CONFIG_LOAD_IO_ERROR;
    config_storage_diagnostics_t backup_diagnostics = {
        .source = CONFIG_SOURCE_RECOVERED_BACKUP,
        .validation = CONFIG_VALIDATION_OK,
        .format_version = -1,
    };

    if (diagnostics != NULL) {
        memset(diagnostics, 0, sizeof(*diagnostics));
        diagnostics->source = CONFIG_SOURCE_BUILTIN_DEFAULT;
        diagnostics->validation = CONFIG_VALIDATION_OK;
        diagnostics->format_version = CONFIG_FORMAT_VERSION;
    }

    if (profiles == NULL ||
        !make_path(base_path, "setting.json", path) ||
        !make_path(base_path, "setting.bak", backup_path)) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_IO_ERROR, NULL,
                        EINVAL);
        return CONFIG_LOAD_IO_ERROR;
    }
    app_config_profiles_set_defaults(profiles);
    if (diagnostics != NULL) {
        diagnostics->source = CONFIG_SOURCE_FILE;
        diagnostics->format_version = -1;
    }
    result = load_path(path, profiles, diagnostics);
    if (result == CONFIG_LOAD_VALID_FILE) {
        (void) unlink(backup_path);
        return result;
    }
    if (result != CONFIG_LOAD_DEFAULT_NO_FILE) {
        if (diagnostics != NULL) {
            diagnostics->source = CONFIG_SOURCE_BUILTIN_DEFAULT;
        }
        return result;
    }

    /*
     * A backup exists only during the FAT-compatible replacement sequence.
     * Recover the last fully validated canonical file after a reset between
     * the two rename operations. A temporary file is never loaded.
    */
    app_config_profiles_set_defaults(recovered);
    result = load_path(backup_path, recovered, &backup_diagnostics);
    if (result == CONFIG_LOAD_VALID_FILE &&
        rename(backup_path, path) == 0) {
        *profiles = *recovered;
        if (diagnostics != NULL) {
            *diagnostics = backup_diagnostics;
        }
        return CONFIG_LOAD_RECOVERED_FILE;
    }
    if (result == CONFIG_LOAD_DEFAULT_NO_FILE) {
        if (diagnostics != NULL) {
            diagnostics->source = CONFIG_SOURCE_BUILTIN_DEFAULT;
            diagnostics->validation = CONFIG_VALIDATION_OK;
            diagnostics->format_version = CONFIG_FORMAT_VERSION;
            diagnostics->io_error = 0;
            diagnostics->key[0] = '\0';
        }
        return CONFIG_LOAD_DEFAULT_NO_FILE;
    }
    if (diagnostics != NULL) {
        *diagnostics = backup_diagnostics;
        diagnostics->source = CONFIG_SOURCE_BUILTIN_DEFAULT;
        if (result == CONFIG_LOAD_VALID_FILE) {
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_IO_ERROR, NULL,
                            errno);
        }
    }
    return CONFIG_LOAD_IO_ERROR;
}

static bool configs_equal(const app_config_t *left, const app_config_t *right,
                          app_parameter_scope_t scope) {
    if (left == NULL || right == NULL) {
        return false;
    }
    for (size_t index = 0U; index < app_config_parameter_count(); index++) {
        app_parameter_info_t info = {0};
        app_parameter_value_t left_value = {0};
        app_parameter_value_t right_value = {0};

        if (!app_config_parameter_info(index, &info) ||
            !app_config_get_value(left, index, &left_value) ||
            !app_config_get_value(right, index, &right_value)) {
            return false;
        }
        if (info.scope != scope) {
            continue;
        }
        switch (info.type) {
        case APP_PARAMETER_BOOL:
            if (left_value.boolean != right_value.boolean) {
                return false;
            }

            break;
        case APP_PARAMETER_UINT32:
            if (left_value.uint32 != right_value.uint32) {
                return false;
            }
            break;
        case APP_PARAMETER_FLOAT:
            if (left_value.real != right_value.real) {
                return false;
            }
            break;
        case APP_PARAMETER_ENUM:
            if (left_value.enumeration != right_value.enumeration) {
                return false;
            }
            break;
        default:
            return false;
        }
    }
    return true;
}

static bool profiles_equal(const app_config_profiles_t *left,
                           const app_config_profiles_t *right) {
    if (left == NULL || right == NULL || left->count != right->count) {
        return false;
    }
    if (!configs_equal(&left->shared_config, &right->shared_config,
                       APP_PARAMETER_SCOPE_SHARED)) {
        return false;
    }
    for (size_t index = 0U; index < left->count; index++) {
        if (left->profiles[index].parameter_number !=
                right->profiles[index].parameter_number ||
            !configs_equal(&left->profiles[index].config,
                           &right->profiles[index].config,
                           APP_PARAMETER_SCOPE_PROFILE)) {
            return false;
        }
    }
    return true;
}

esp_err_t config_storage_save(const char *base_path,
                              const app_config_profiles_t *profiles) {
    char target_path[CONFIG_PATH_BUFFER_SIZE] = {0};
    char temporary_path[CONFIG_PATH_BUFFER_SIZE] = {0};
    char backup_path[CONFIG_PATH_BUFFER_SIZE] = {0};
    FILE *file = NULL;
    app_config_profiles_t *canonical =
        &config_storage_workspace.canonical;
    app_config_profiles_t *verified =
        &config_storage_workspace.verified;
    config_storage_diagnostics_t verification_diagnostics = {0};
    config_load_result_t verification_result = CONFIG_LOAD_IO_ERROR;
    int file_descriptor = -1;
    struct stat target_status = {0};
    bool target_exists = false;

    if (profiles == NULL || !app_config_profiles_validate(profiles) ||
        !make_path(base_path, "setting.json", target_path) ||
        !make_path(base_path, "setting.tmp", temporary_path) ||
        !make_path(base_path, "setting.bak", backup_path)) {
        return ESP_ERR_INVALID_ARG;
    }
    *canonical = *profiles;
    app_config_profiles_sort(canonical);

    file = fopen(temporary_path, "wb");
    if (file == NULL) {
        return ESP_FAIL;
    }
    if (!config_json_write(file, canonical) || fflush(file) != 0) {
        (void) fclose(file);
        (void) unlink(temporary_path);
        return ESP_FAIL;
    }
    file_descriptor = fileno(file);
    if (file_descriptor < 0 || fsync(file_descriptor) != 0 ||
        fclose(file) != 0) {
        (void) unlink(temporary_path);
        return ESP_FAIL;
    }

    verification_result = load_path(temporary_path, verified,
                                    &verification_diagnostics);
    if (verification_result != CONFIG_LOAD_VALID_FILE ||
        !profiles_equal(verified, canonical)) {
        (void) unlink(temporary_path);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (stat(target_path, &target_status) == 0) {
        target_exists = true;
    } else if (errno != ENOENT) {
        (void) unlink(temporary_path);
        return ESP_FAIL;
    }

    if (target_exists) {
        if (unlink(backup_path) != 0 && errno != ENOENT) {
            (void) unlink(temporary_path);
            return ESP_FAIL;
        }
        if (rename(target_path, backup_path) != 0) {
            (void) unlink(temporary_path);
            return ESP_FAIL;
        }
    }

    if (rename(temporary_path, target_path) != 0) {
        if (target_exists) {
            (void) rename(backup_path, target_path);
        }
        (void) unlink(temporary_path);
        return ESP_FAIL;
    }
    if (target_exists) {
        (void) unlink(backup_path);
    }
    return ESP_OK;
}

const char *config_storage_source_name(config_source_t source) {
    switch (source) {
    case CONFIG_SOURCE_BUILTIN_DEFAULT:
        return "BUILTIN_DEFAULT";
    case CONFIG_SOURCE_FILE:
        return "FILE";
    case CONFIG_SOURCE_RECOVERED_BACKUP:
        return "RECOVERED_BACKUP";
    default:
        return "UNKNOWN";
    }
}

const char *config_storage_validation_name(
    config_validation_result_t validation) {
    switch (validation) {
    case CONFIG_VALIDATION_OK:
        return "OK";
    case CONFIG_VALIDATION_EMPTY_FILE:
        return "EMPTY_FILE";
    case CONFIG_VALIDATION_FILE_TOO_LARGE:
        return "FILE_TOO_LARGE";
    case CONFIG_VALIDATION_JSON_SYNTAX:
        return "JSON_SYNTAX";
    case CONFIG_VALIDATION_ROOT_NOT_OBJECT:
        return "ROOT_NOT_OBJECT";
    case CONFIG_VALIDATION_UNKNOWN_TOP_LEVEL_KEY:
        return "UNKNOWN_TOP_LEVEL_KEY";
    case CONFIG_VALIDATION_DUPLICATE_KEY:
        return "DUPLICATE_KEY";
    case CONFIG_VALIDATION_MISSING_FORMAT_VERSION:
        return "MISSING_FORMAT_VERSION";
    case CONFIG_VALIDATION_FORMAT_VERSION_TYPE:
        return "FORMAT_VERSION_TYPE";
    case CONFIG_VALIDATION_UNSUPPORTED_FORMAT_VERSION:
        return "UNSUPPORTED_FORMAT_VERSION";
    case CONFIG_VALIDATION_MISSING_PARAMETERS:
        return "MISSING_PARAMETERS";
    case CONFIG_VALIDATION_PARAMETERS_NOT_OBJECT:
        return "PARAMETERS_NOT_OBJECT";
    case CONFIG_VALIDATION_MISSING_PARAMETER_SETS:
        return "MISSING_PARAMETER_SETS";
    case CONFIG_VALIDATION_PARAMETER_SETS_NOT_ARRAY:
        return "PARAMETER_SETS_NOT_ARRAY";
    case CONFIG_VALIDATION_PROFILE_COUNT:
        return "PROFILE_COUNT";
    case CONFIG_VALIDATION_PROFILE_NOT_OBJECT:
        return "PROFILE_NOT_OBJECT";
    case CONFIG_VALIDATION_MISSING_PARAMETER_NUMBER:
        return "MISSING_PARAMETER_NUMBER";
    case CONFIG_VALIDATION_PARAMETER_NUMBER_TYPE:
        return "PARAMETER_NUMBER_TYPE";
    case CONFIG_VALIDATION_PARAMETER_NUMBER_RANGE:
        return "PARAMETER_NUMBER_RANGE";
    case CONFIG_VALIDATION_DUPLICATE_PARAMETER_NUMBER:
        return "DUPLICATE_PARAMETER_NUMBER";

    case CONFIG_VALIDATION_UNKNOWN_PARAMETER:
        return "UNKNOWN_PARAMETER";
    case CONFIG_VALIDATION_PARAMETER_TYPE:
        return "PARAMETER_TYPE";
    case CONFIG_VALIDATION_NONFINITE_VALUE:
        return "NONFINITE_VALUE";
    case CONFIG_VALIDATION_PARAMETER_RANGE:
        return "PARAMETER_RANGE";
    case CONFIG_VALIDATION_RELATION:
        return "RELATION";
    case CONFIG_VALIDATION_NO_MEMORY:
        return "NO_MEMORY";
    case CONFIG_VALIDATION_IO_ERROR:
        return "IO_ERROR";
    default:
        return "UNKNOWN";
    }
}
