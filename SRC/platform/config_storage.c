#include "platform/config_storage.h"

#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "cJSON.h"

#define CONFIG_MAX_FILE_BYTES (32U * 1024U)
#define CONFIG_PATH_BUFFER_SIZE 96U

/*
 * Keep the multi-profile work buffers out of the caller task's stack.
 * usb_device_service serializes storage I/O, and startup loading completes
 * before worker tasks are created.
 */
static struct {
    app_config_profiles_t parse_candidate;
    app_config_profiles_t recovered;
    app_config_profiles_t canonical;
    app_config_profiles_t verified;
} config_storage_workspace;

static void set_diagnostics(config_storage_diagnostics_t *diagnostics,
                            config_validation_result_t validation,
                            const char *key, int32_t io_error) {
    if (diagnostics == NULL) {
        return;
    }
    diagnostics->validation = validation;
    diagnostics->io_error = io_error;
    diagnostics->key[0] = '\0';
    if (key != NULL) {
        (void) snprintf(diagnostics->key, sizeof(diagnostics->key), "%s",
                        key);
    }
}

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

static bool parameter_index_by_name(const char *name, size_t *index_out,
                                    app_parameter_info_t *info_out) {
    if (name == NULL) {
        return false;
    }

    for (size_t index = 0U; index < app_config_parameter_count(); index++) {
        app_parameter_info_t info = {0};

        if (app_config_parameter_info(index, &info) &&
            strcmp(name, info.name) == 0) {
            if (index_out != NULL) {
                *index_out = index;
            }
            if (info_out != NULL) {
                *info_out = info;
            }
            return true;
        }
    }
    return false;
}

static bool parse_json_parameter(app_config_t *candidate, const char *name,
                                 const cJSON *item,
                                 app_parameter_type_t type,
                                 config_storage_diagnostics_t *diagnostics) {
    app_parameter_value_t value = {0};

    switch (type) {
    case APP_PARAMETER_BOOL:
        if (!cJSON_IsBool(item)) {
            set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_TYPE,
                            name, 0);
            return false;
        }
        value.boolean = cJSON_IsTrue(item);
        break;
    case APP_PARAMETER_UINT32:
        if (!cJSON_IsNumber(item)) {
            set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_TYPE,
                            name, 0);
            return false;
        }
        if (!isfinite(item->valuedouble)) {
            set_diagnostics(diagnostics, CONFIG_VALIDATION_NONFINITE_VALUE,
                            name, 0);
            return false;
        }
        if (item->valuedouble < 0.0 ||
            item->valuedouble > (double) UINT32_MAX ||
            floor(item->valuedouble) != item->valuedouble) {
            set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_RANGE,
                            name, 0);
            return false;
        }
        value.uint32 = (uint32_t) item->valuedouble;
        break;
    case APP_PARAMETER_FLOAT:
        if (!cJSON_IsNumber(item)) {
            set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_TYPE,
                            name, 0);
            return false;
        }
        if (!isfinite(item->valuedouble)) {
            set_diagnostics(diagnostics, CONFIG_VALIDATION_NONFINITE_VALUE,
                            name, 0);
            return false;
        }
        if (item->valuedouble < -(double) FLT_MAX ||
            item->valuedouble > (double) FLT_MAX) {
            set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_RANGE,
                            name, 0);
            return false;
        }
        value.real = (float) item->valuedouble;
        break;
    case APP_PARAMETER_ENUM:
        if (!cJSON_IsString(item) || item->valuestring == NULL) {
            set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_TYPE,
                            name, 0);
            return false;
        }
        if (strcmp(item->valuestring, "AUTO") == 0) {
            value.filter_mode = APP_FILTER_MODE_AUTO;
        } else if (strcmp(item->valuestring, "BARO_ONLY") == 0) {
            value.filter_mode = APP_FILTER_MODE_BARO_ONLY;
        } else {
            set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_RANGE,
                            name, 0);
            return false;
        }
        break;
    default:
        set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_TYPE, name,
                        0);
        return false;
    }
    if (!app_config_assign_value(candidate, name, type, value)) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_RANGE, name,
                        0);
        return false;
    }
    return true;
}

static bool parse_parameters_object(
    const cJSON *parameters, app_parameter_scope_t expected_scope,
    app_config_t *config,
    config_storage_diagnostics_t *diagnostics) {
    app_config_t candidate = {0};
    bool *seen = NULL;
    bool valid = false;

    if (!cJSON_IsObject(parameters) || config == NULL) {
        set_diagnostics(diagnostics,
                        CONFIG_VALIDATION_PARAMETERS_NOT_OBJECT,
                        "parameters", 0);
        return false;
    }
    seen = calloc(app_config_parameter_count(), sizeof(*seen));
    if (seen == NULL) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_NO_MEMORY, NULL,
                        ENOMEM);
        return false;
    }
    app_config_set_defaults(&candidate);

    for (const cJSON *child = parameters->child; child != NULL;
         child = child->next) {
        size_t index = 0U;
        app_parameter_info_t info = {0};

        if (child->string == NULL) {
            set_diagnostics(diagnostics,
                            CONFIG_VALIDATION_UNKNOWN_PARAMETER, NULL, 0);
            goto cleanup;
        }
        if (!parameter_index_by_name(child->string, &index, &info)) {
            set_diagnostics(diagnostics,
                            CONFIG_VALIDATION_UNKNOWN_PARAMETER,
                            child->string, 0);
            goto cleanup;
        }
        if (info.scope != expected_scope) {
            set_diagnostics(diagnostics,
                            CONFIG_VALIDATION_UNKNOWN_PARAMETER,
                            child->string, 0);
            goto cleanup;
        }
        if (seen[index]) {
            set_diagnostics(diagnostics, CONFIG_VALIDATION_DUPLICATE_KEY,
                            child->string, 0);
            goto cleanup;
        }
        if (!parse_json_parameter(&candidate, child->string, child,
                                  info.type, diagnostics)) {
            goto cleanup;
        }
        seen[index] = true;
    }
    for (size_t index = 0U; index < app_config_parameter_count(); index++) {
        app_parameter_info_t info = {0};

        if (app_config_parameter_info(index, &info) &&
            info.scope == expected_scope && !seen[index]) {
            set_diagnostics(diagnostics, CONFIG_VALIDATION_MISSING_PARAMETERS,
                            info.name, 0);
            goto cleanup;
        }
    }
    *config = candidate;
    valid = true;

cleanup:
    free(seen);
    return valid;
}

static bool parse_config_json(const char *json, size_t json_length,
                              app_config_profiles_t *profiles,
                              config_storage_diagnostics_t *diagnostics) {
    const char *parse_end = NULL;
    const char *document = json;
    size_t document_length = json_length;
    cJSON *root = NULL;
    cJSON *version = NULL;
    cJSON *shared_parameters = NULL;
    cJSON *parameter_sets = NULL;
    app_config_profiles_t *candidate =
        &config_storage_workspace.parse_candidate;
    bool valid = false;
    unsigned int version_count = 0U;
    unsigned int shared_parameters_count = 0U;
    unsigned int parameter_sets_count = 0U;
    int32_t parsed_version = -1;

    memset(candidate, 0, sizeof(*candidate));

    if (json == NULL || profiles == NULL || json_length == 0U) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_EMPTY_FILE, NULL, 0);
        return false;
    }
    if (json_length >= 3U && (uint8_t) json[0] == UINT8_C(0xEF) &&
        (uint8_t) json[1] == UINT8_C(0xBB) &&
        (uint8_t) json[2] == UINT8_C(0xBF)) {
        document += 3;
        document_length -= 3U;
    }

    root = cJSON_ParseWithLengthOpts(document, document_length + 1U, &parse_end,
                                     true);
    if (root == NULL || parse_end != document + document_length) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_JSON_SYNTAX, NULL, 0);
        goto cleanup;
    }
    if (!cJSON_IsObject(root)) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_ROOT_NOT_OBJECT, NULL,
                        0);
        goto cleanup;
    }

    for (cJSON *child = root->child; child != NULL; child = child->next) {
        if (child->string == NULL) {
            set_diagnostics(diagnostics,
                            CONFIG_VALIDATION_UNKNOWN_TOP_LEVEL_KEY, NULL, 0);
            goto cleanup;
        }
        if (strcmp(child->string, "format_version") == 0) {
            version_count++;
            version = child;
        } else if (strcmp(child->string, "parameters") == 0) {
            shared_parameters_count++;
            shared_parameters = child;
        } else if (strcmp(child->string, "parameter_sets") == 0) {
            parameter_sets_count++;
            parameter_sets = child;
        } else {
            set_diagnostics(diagnostics,
                            CONFIG_VALIDATION_UNKNOWN_TOP_LEVEL_KEY,
                            child->string, 0);
            goto cleanup;
        }
    }
    if (version_count == 0U) {
        set_diagnostics(diagnostics,
                        CONFIG_VALIDATION_MISSING_FORMAT_VERSION,
                        "format_version", 0);
        goto cleanup;
    }
    if (version_count > 1U) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_DUPLICATE_KEY,
                        "format_version", 0);
        goto cleanup;
    }
    if (!cJSON_IsNumber(version) || !isfinite(version->valuedouble) ||
        floor(version->valuedouble) != version->valuedouble) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_FORMAT_VERSION_TYPE,
                        "format_version", 0);
        goto cleanup;
    }
    if (version->valuedouble >= (double) INT32_MIN &&
        version->valuedouble <= (double) INT32_MAX) {
        parsed_version = (int32_t) version->valuedouble;
        if (diagnostics != NULL) {
            diagnostics->format_version = parsed_version;
        }
    }
    if (parsed_version != CONFIG_FORMAT_VERSION) {
        set_diagnostics(diagnostics,
                        CONFIG_VALIDATION_UNSUPPORTED_FORMAT_VERSION,
                        "format_version", 0);
        goto cleanup;
    }
    if (shared_parameters_count == 0U) {
        set_diagnostics(diagnostics,
                        CONFIG_VALIDATION_MISSING_PARAMETERS,
                        "parameters", 0);
        goto cleanup;
    }
    if (shared_parameters_count > 1U) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_DUPLICATE_KEY,
                        "parameters", 0);
        goto cleanup;
    }
    if (!parse_parameters_object(shared_parameters,
                                 APP_PARAMETER_SCOPE_SHARED,
                                 &candidate->shared_config, diagnostics)) {
        goto cleanup;
    }
    if (parameter_sets_count == 0U) {
        set_diagnostics(diagnostics,
                        CONFIG_VALIDATION_MISSING_PARAMETER_SETS,
                        "parameter_sets", 0);
        goto cleanup;
    }
    if (parameter_sets_count > 1U) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_DUPLICATE_KEY,
                        "parameter_sets", 0);
        goto cleanup;
    }
    {
        int profile_count = 0;
        bool seen_numbers[APP_CONFIG_PROFILE_MAX_NUMBER + 1U] = {false};

        if (!cJSON_IsArray(parameter_sets)) {
            set_diagnostics(diagnostics,
                            CONFIG_VALIDATION_PARAMETER_SETS_NOT_ARRAY,
                            "parameter_sets", 0);
            goto cleanup;
        }
        profile_count = cJSON_GetArraySize(parameter_sets);
        if (profile_count < 1 ||
            profile_count > (int) APP_CONFIG_PROFILE_MAX_COUNT) {
            set_diagnostics(diagnostics, CONFIG_VALIDATION_PROFILE_COUNT,
                            "parameter_sets", 0);
            goto cleanup;
        }
        candidate->count = (size_t) profile_count;
        for (size_t index = 0U; index < candidate->count; index++) {
            cJSON *profile = cJSON_GetArrayItem(parameter_sets, (int) index);
            cJSON *number = NULL;
            cJSON *parameters = NULL;
            unsigned int number_count = 0U;
            unsigned int parameters_count = 0U;

            if (!cJSON_IsObject(profile)) {
                set_diagnostics(diagnostics,
                                CONFIG_VALIDATION_PROFILE_NOT_OBJECT,
                                "parameter_sets", 0);
                goto cleanup;
            }
            for (cJSON *child = profile->child; child != NULL;
                 child = child->next) {
                if (child->string == NULL) {
                    set_diagnostics(diagnostics,
                                    CONFIG_VALIDATION_UNKNOWN_PARAMETER,
                                    "parameter_sets", 0);
                    goto cleanup;
                }
                if (strcmp(child->string, "parameter_number") == 0) {
                    number_count++;
                    number = child;
                } else if (strcmp(child->string, "parameters") == 0) {
                    parameters_count++;
                    parameters = child;
                } else {
                    set_diagnostics(
                        diagnostics,
                        CONFIG_VALIDATION_UNKNOWN_TOP_LEVEL_KEY,
                        child->string, 0);
                    goto cleanup;
                }
            }
            if (number_count == 0U) {
                set_diagnostics(diagnostics,
                                CONFIG_VALIDATION_MISSING_PARAMETER_NUMBER,
                                "parameter_number", 0);
                goto cleanup;
            }
            if (number_count > 1U || parameters_count > 1U) {
                set_diagnostics(diagnostics,
                                CONFIG_VALIDATION_DUPLICATE_KEY,
                                number_count > 1U ? "parameter_number"
                                                  : "parameters",
                                0);
                goto cleanup;
            }
            if (parameters_count == 0U) {
                set_diagnostics(diagnostics,
                                CONFIG_VALIDATION_MISSING_PARAMETERS,
                                "parameters", 0);
                goto cleanup;
            }
            if (!cJSON_IsNumber(number) ||
                !isfinite(number->valuedouble) ||
                floor(number->valuedouble) != number->valuedouble) {
                set_diagnostics(diagnostics,
                                CONFIG_VALIDATION_PARAMETER_NUMBER_TYPE,
                                "parameter_number", 0);
                goto cleanup;
            }
            if (number->valuedouble < APP_CONFIG_PROFILE_MIN_NUMBER ||
                number->valuedouble > APP_CONFIG_PROFILE_MAX_NUMBER) {
                set_diagnostics(diagnostics,
                                CONFIG_VALIDATION_PARAMETER_NUMBER_RANGE,
                                "parameter_number", 0);
                goto cleanup;
            }
            candidate->profiles[index].parameter_number =
                (uint8_t) number->valuedouble;
            if (seen_numbers[candidate->profiles[index].parameter_number]) {
                set_diagnostics(
                    diagnostics,
                    CONFIG_VALIDATION_DUPLICATE_PARAMETER_NUMBER,
                    "parameter_number", 0);
                goto cleanup;
            }
            seen_numbers[candidate->profiles[index].parameter_number] = true;
            if (!parse_parameters_object(
                    parameters, APP_PARAMETER_SCOPE_PROFILE,
                    &candidate->profiles[index].config,
                    diagnostics)) {
                goto cleanup;
            }
        }
        app_config_profiles_sort(candidate);
    }
    if (!app_config_profiles_validate(candidate)) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_RELATION, NULL, 0);
        goto cleanup;
    }
    *profiles = *candidate;
    set_diagnostics(diagnostics, CONFIG_VALIDATION_OK, NULL, 0);
    valid = true;

cleanup:
    cJSON_Delete(root);
    return valid;
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
        set_diagnostics(diagnostics, CONFIG_VALIDATION_IO_ERROR, NULL,
                        errno);
        return CONFIG_LOAD_IO_ERROR;
    }
    if (file_status.st_size <= 0) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_EMPTY_FILE, NULL, 0);
        return CONFIG_LOAD_INVALID_FILE;
    }
    if ((uint64_t) file_status.st_size > CONFIG_MAX_FILE_BYTES) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_FILE_TOO_LARGE, NULL,
                        0);
        return CONFIG_LOAD_INVALID_FILE;
    }

    contents = malloc((size_t) file_status.st_size + 1U);
    if (contents == NULL) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_NO_MEMORY, NULL,
                        ENOMEM);
        return CONFIG_LOAD_IO_ERROR;
    }
    file = fopen(path, "rb");
    if (file == NULL) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_IO_ERROR, NULL,
                        errno);
        free(contents);
        return CONFIG_LOAD_IO_ERROR;
    }
    bytes_read =
        fread(contents, 1U, (size_t) file_status.st_size, file);
    if (ferror(file) != 0 || bytes_read != (size_t) file_status.st_size) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_IO_ERROR, NULL,
                        ferror(file) != 0 ? errno : EIO);
        result = CONFIG_LOAD_IO_ERROR;
    } else {
        contents[bytes_read] = '\0';
        result = parse_config_json(contents, bytes_read, profiles, diagnostics)
                     ? CONFIG_LOAD_VALID_FILE
                     : CONFIG_LOAD_INVALID_FILE;
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
        !make_path(base_path, "parameters.json", path) ||
        !make_path(base_path, "parameters.bak", backup_path)) {
        set_diagnostics(diagnostics, CONFIG_VALIDATION_IO_ERROR, NULL,
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
            set_diagnostics(diagnostics, CONFIG_VALIDATION_IO_ERROR, NULL,
                            errno);
        }
    }
    return CONFIG_LOAD_IO_ERROR;
}

static bool write_parameter_object(FILE *file, const app_config_t *config,
                                   app_parameter_scope_t scope,
                                   const char *indent) {
    size_t remaining = 0U;

    for (size_t index = 0U; index < app_config_parameter_count(); index++) {
        app_parameter_info_t info = {0};

        if (app_config_parameter_info(index, &info) && info.scope == scope) {
            remaining++;
        }
    }

    for (size_t index = 0U; index < app_config_parameter_count(); index++) {
        app_parameter_info_t info = {0};
        app_parameter_value_t value = {0};
        char value_text[40] = {0};
        int written = -1;

        if (!app_config_parameter_info(index, &info) ||
            !app_config_get_value(config, index, &value)) {
            return false;
        }
        if (info.scope != scope) {
            continue;
        }
        switch (info.type) {
        case APP_PARAMETER_BOOL:
            written = snprintf(value_text, sizeof(value_text), "%s",
                               value.boolean ? "true" : "false");
            break;
        case APP_PARAMETER_UINT32:
            written = snprintf(value_text, sizeof(value_text), "%lu",
                               (unsigned long) value.uint32);
            break;
        case APP_PARAMETER_FLOAT:
            written = snprintf(value_text, sizeof(value_text), "%.9g",
                               (double) value.real);
            break;
        case APP_PARAMETER_ENUM:
            written = snprintf(
                value_text, sizeof(value_text), "\"%s\"",
                app_config_filter_mode_name(value.filter_mode));
            break;
        default:
            return false;
        }
        if (written <= 0 || (size_t) written >= sizeof(value_text) ||
            fprintf(file, "%s\"%s\": %s%s\n", indent, info.name, value_text,
                    --remaining > 0U ? "," : "") < 0) {
            return false;
        }
    }
    return true;
}

static bool write_json(FILE *file, const app_config_profiles_t *profiles) {
    if (fprintf(file, "{\n  \"format_version\": %d,\n"
                      "  \"parameters\": {\n",
                CONFIG_FORMAT_VERSION) < 0 ||
        !write_parameter_object(file, &profiles->shared_config,
                                APP_PARAMETER_SCOPE_SHARED, "    ") ||
        fprintf(file, "  },\n"
                      "  \"parameter_sets\": [\n") < 0) {
        return false;
    }
    for (size_t index = 0U; index < profiles->count; index++) {
        const app_config_profile_t *profile = &profiles->profiles[index];

        if (fprintf(file,
                    "    {\n      \"parameter_number\": %u,\n"
                    "      \"parameters\": {\n",
                    (unsigned int) profile->parameter_number) < 0 ||
            !write_parameter_object(file, &profile->config,
                                    APP_PARAMETER_SCOPE_PROFILE, "        ") ||
            fprintf(file, "      }\n    }%s\n",
                    index + 1U < profiles->count ? "," : "") < 0) {
            return false;
        }
    }
    return fprintf(file, "  ]\n}\n") >= 0;
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
            if (left_value.filter_mode != right_value.filter_mode) {
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
        !make_path(base_path, "parameters.json", target_path) ||
        !make_path(base_path, "parameters.tmp", temporary_path) ||
        !make_path(base_path, "parameters.bak", backup_path)) {
        return ESP_ERR_INVALID_ARG;
    }
    *canonical = *profiles;
    app_config_profiles_sort(canonical);

    file = fopen(temporary_path, "wb");
    if (file == NULL) {
        return ESP_FAIL;
    }
    if (!write_json(file, canonical) || fflush(file) != 0) {
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
