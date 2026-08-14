#include "platform/config_json.h"

#include <errno.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"

static struct {
    app_config_profiles_t parse_candidate;
} config_json_workspace;

void config_json_set_diagnostics(config_storage_diagnostics_t *diagnostics,
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
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_TYPE,
                            name, 0);
            return false;
        }
        value.boolean = cJSON_IsTrue(item);
        break;
    case APP_PARAMETER_UINT32:
        if (!cJSON_IsNumber(item)) {
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_TYPE,
                            name, 0);
            return false;
        }
        if (!isfinite(item->valuedouble)) {
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_NONFINITE_VALUE,
                            name, 0);
            return false;
        }
        if (item->valuedouble < 0.0 ||
            item->valuedouble > (double) UINT32_MAX ||
            floor(item->valuedouble) != item->valuedouble) {
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_RANGE,
                            name, 0);
            return false;
        }
        value.uint32 = (uint32_t) item->valuedouble;
        break;
    case APP_PARAMETER_FLOAT:
        if (!cJSON_IsNumber(item)) {
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_TYPE,
                            name, 0);
            return false;
        }
        if (!isfinite(item->valuedouble)) {
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_NONFINITE_VALUE,
                            name, 0);
            return false;
        }
        if (item->valuedouble < -(double) FLT_MAX ||
            item->valuedouble > (double) FLT_MAX) {
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_RANGE,
                            name, 0);
            return false;
        }
        value.real = (float) item->valuedouble;
        break;
    case APP_PARAMETER_ENUM:
        if (!cJSON_IsString(item) || item->valuestring == NULL) {
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_TYPE,
                            name, 0);
            return false;
        }
        if (!app_config_parse_enum_value(name, item->valuestring,
                                         &value.enumeration) ||
            strcmp(item->valuestring,
                   app_config_enum_value_name(name,
                                              value.enumeration)) != 0) {
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_RANGE,
                            name, 0);
            return false;
        }
        break;
    default:
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_TYPE, name,
                        0);
        return false;
    }
    if (!app_config_assign_value(candidate, name, type, value)) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_PARAMETER_RANGE, name,
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
        config_json_set_diagnostics(diagnostics,
                        CONFIG_VALIDATION_PARAMETERS_NOT_OBJECT,
                        "mc_parameters", 0);
        return false;
    }
    /*
     * CODING_RULES_DYNAMIC_MEMORY: the parameter count is table-driven and
     * bounded by the strict document schema. Parsing is serialized, and this
     * temporary duplicate-key map is released at cleanup on every path.
     */
    seen = calloc(app_config_parameter_count(), sizeof(*seen));
    if (seen == NULL) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_NO_MEMORY, NULL,
                        ENOMEM);
        return false;

    }
    app_config_set_defaults(&candidate);

    for (const cJSON *child = parameters->child; child != NULL;
         child = child->next) {
        size_t index = 0U;
        app_parameter_info_t info = {0};

        if (child->string == NULL) {
            config_json_set_diagnostics(diagnostics,
                            CONFIG_VALIDATION_UNKNOWN_PARAMETER, NULL, 0);
            goto cleanup;
        }
        if (!parameter_index_by_name(child->string, &index, &info)) {
            config_json_set_diagnostics(diagnostics,
                            CONFIG_VALIDATION_UNKNOWN_PARAMETER,
                            child->string, 0);
            goto cleanup;
        }
        if (info.scope != expected_scope) {
            config_json_set_diagnostics(diagnostics,
                            CONFIG_VALIDATION_UNKNOWN_PARAMETER,
                            child->string, 0);
            goto cleanup;
        }
        if (seen[index]) {
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_DUPLICATE_KEY,
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
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_MISSING_PARAMETERS,
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


bool config_json_parse(const char *json, size_t json_length,
                              app_config_profiles_t *profiles,
                              config_storage_diagnostics_t *diagnostics) {
    const char *parse_end = NULL;
    const char *document = json;
    size_t document_length = json_length;
    cJSON *root = NULL;
    cJSON *version = NULL;
    cJSON *mc_parameters = NULL;
    cJSON *vario_parameter_sets = NULL;
    app_config_profiles_t *candidate =
        &config_json_workspace.parse_candidate;
    bool valid = false;
    unsigned int version_count = 0U;
    unsigned int mc_parameters_count = 0U;
    unsigned int vario_parameter_sets_count = 0U;
    int32_t parsed_version = -1;

    memset(candidate, 0, sizeof(*candidate));

    if (json == NULL || profiles == NULL || json_length == 0U) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_EMPTY_FILE, NULL, 0);
        return false;
    }
    if (json_length >= 3U && (uint8_t) json[0] == UINT8_C(0xEF) &&
        (uint8_t) json[1] == UINT8_C(0xBB) &&
        (uint8_t) json[2] == UINT8_C(0xBF)) {
        document += 3;
        document_length -= 3U;
    }

    /*
     * CODING_RULES_DYNAMIC_MEMORY: cJSON owns a bounded parse tree for this
     * low-frequency, serialized configuration transaction. cJSON_Delete at
     * cleanup releases the complete tree on every exit path.
     */
    root = cJSON_ParseWithLengthOpts(document, document_length + 1U, &parse_end,
                                     true);
    if (root == NULL || parse_end != document + document_length) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_JSON_SYNTAX, NULL, 0);
        goto cleanup;
    }
    if (!cJSON_IsObject(root)) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_ROOT_NOT_OBJECT, NULL,
                        0);
        goto cleanup;
    }

    for (cJSON *child = root->child; child != NULL; child = child->next) {
        if (child->string == NULL) {
            config_json_set_diagnostics(diagnostics,
                            CONFIG_VALIDATION_UNKNOWN_TOP_LEVEL_KEY, NULL, 0);
            goto cleanup;
        }
        if (strcmp(child->string, "format_version") == 0) {
            version_count++;
            version = child;
        } else if (strcmp(child->string, "mc_parameters") == 0) {
            mc_parameters_count++;
            mc_parameters = child;
        } else if (strcmp(child->string, "vario_parameter_sets") == 0) {
            vario_parameter_sets_count++;
            vario_parameter_sets = child;
        } else {
            config_json_set_diagnostics(diagnostics,
                            CONFIG_VALIDATION_UNKNOWN_TOP_LEVEL_KEY,
                            child->string, 0);
            goto cleanup;
        }
    }
    if (version_count == 0U) {
        config_json_set_diagnostics(diagnostics,
                        CONFIG_VALIDATION_MISSING_FORMAT_VERSION,
                        "format_version", 0);
        goto cleanup;
    }
    if (version_count > 1U) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_DUPLICATE_KEY,
                        "format_version", 0);
        goto cleanup;
    }
    if (!cJSON_IsNumber(version) || !isfinite(version->valuedouble) ||
        floor(version->valuedouble) != version->valuedouble) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_FORMAT_VERSION_TYPE,
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
        config_json_set_diagnostics(diagnostics,
                        CONFIG_VALIDATION_UNSUPPORTED_FORMAT_VERSION,
                        "format_version", 0);
        goto cleanup;
    }
    if (mc_parameters_count == 0U) {
        config_json_set_diagnostics(diagnostics,
                        CONFIG_VALIDATION_MISSING_PARAMETERS,
                        "mc_parameters", 0);
        goto cleanup;
    }
    if (mc_parameters_count > 1U) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_DUPLICATE_KEY,
                        "mc_parameters", 0);
        goto cleanup;
    }
    if (!parse_parameters_object(mc_parameters,
                                 APP_PARAMETER_SCOPE_SHARED,
                                 &candidate->shared_config, diagnostics)) {
        goto cleanup;
    }
    if (vario_parameter_sets_count == 0U) {
        config_json_set_diagnostics(diagnostics,
                        CONFIG_VALIDATION_MISSING_PARAMETER_SETS,
                        "vario_parameter_sets", 0);
        goto cleanup;
    }
    if (vario_parameter_sets_count > 1U) {
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_DUPLICATE_KEY,
                        "vario_parameter_sets", 0);
        goto cleanup;
    }
    {
        int profile_count = 0;
        bool seen_numbers[APP_CONFIG_PROFILE_MAX_NUMBER + 1U] = {false};


        if (!cJSON_IsArray(vario_parameter_sets)) {
            config_json_set_diagnostics(diagnostics,
                            CONFIG_VALIDATION_PARAMETER_SETS_NOT_ARRAY,
                            "vario_parameter_sets", 0);
            goto cleanup;
        }
        profile_count = cJSON_GetArraySize(vario_parameter_sets);
        if (profile_count < 1 ||
            profile_count > (int) APP_CONFIG_PROFILE_MAX_COUNT) {
            config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_PROFILE_COUNT,
                            "vario_parameter_sets", 0);
            goto cleanup;
        }
        candidate->count = (size_t) profile_count;
        for (size_t index = 0U; index < candidate->count; index++) {
            cJSON *profile = cJSON_GetArrayItem(vario_parameter_sets, (int) index);
            cJSON *number = NULL;
            cJSON *parameters = NULL;
            unsigned int number_count = 0U;
            unsigned int parameters_count = 0U;

            if (!cJSON_IsObject(profile)) {
                config_json_set_diagnostics(diagnostics,
                                CONFIG_VALIDATION_PROFILE_NOT_OBJECT,
                                "vario_parameter_sets", 0);
                goto cleanup;
            }
            for (cJSON *child = profile->child; child != NULL;
                 child = child->next) {
                if (child->string == NULL) {
                    config_json_set_diagnostics(diagnostics,
                                    CONFIG_VALIDATION_UNKNOWN_PARAMETER,
                                    "vario_parameter_sets", 0);
                    goto cleanup;
                }
                if (strcmp(child->string, "parameter_number") == 0) {
                    number_count++;
                    number = child;
                } else if (strcmp(child->string, "parameters") == 0) {
                    parameters_count++;
                    parameters = child;
                } else {
                    config_json_set_diagnostics(
                        diagnostics,
                        CONFIG_VALIDATION_UNKNOWN_TOP_LEVEL_KEY,
                        child->string, 0);
                    goto cleanup;
                }
            }
            if (number_count == 0U) {
                config_json_set_diagnostics(diagnostics,
                                CONFIG_VALIDATION_MISSING_PARAMETER_NUMBER,
                                "parameter_number", 0);
                goto cleanup;
            }
            if (number_count > 1U || parameters_count > 1U) {
                const char *duplicate_key = "parameters";

                if (number_count > 1U) {
                    duplicate_key = "parameter_number";
                }
                config_json_set_diagnostics(diagnostics,
                                CONFIG_VALIDATION_DUPLICATE_KEY,
                                duplicate_key, 0);
                goto cleanup;
            }
            if (parameters_count == 0U) {
                config_json_set_diagnostics(diagnostics,
                                CONFIG_VALIDATION_MISSING_PARAMETERS,
                                "parameters", 0);
                goto cleanup;
            }
            if (!cJSON_IsNumber(number) ||
                !isfinite(number->valuedouble) ||
                floor(number->valuedouble) != number->valuedouble) {
                config_json_set_diagnostics(diagnostics,
                                CONFIG_VALIDATION_PARAMETER_NUMBER_TYPE,
                                "parameter_number", 0);
                goto cleanup;
            }
            if (number->valuedouble < APP_CONFIG_PROFILE_MIN_NUMBER ||
                number->valuedouble > APP_CONFIG_PROFILE_MAX_NUMBER) {
                config_json_set_diagnostics(diagnostics,
                                CONFIG_VALIDATION_PARAMETER_NUMBER_RANGE,
                                "parameter_number", 0);
                goto cleanup;
            }
            candidate->profiles[index].parameter_number =
                (uint8_t) number->valuedouble;
            if (seen_numbers[candidate->profiles[index].parameter_number]) {
                config_json_set_diagnostics(
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
        config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_RELATION, NULL, 0);
        goto cleanup;
    }
    *profiles = *candidate;
    config_json_set_diagnostics(diagnostics, CONFIG_VALIDATION_OK, NULL, 0);
    valid = true;

cleanup:
    cJSON_Delete(root);
    return valid;
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
        case APP_PARAMETER_BOOL: {
            const char *boolean_text = "false";

            if (value.boolean) {
                boolean_text = "true";
            }
            written = snprintf(value_text, sizeof(value_text), "%s",
                               boolean_text);
            break;
        }
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
                app_config_enum_value_name(info.name, value.enumeration));
            break;
        default:
            return false;
        }
        remaining--;
        const char *separator = "";
        if (remaining > 0U) {
            separator = ",";
        }
        if (written <= 0 || (size_t) written >= sizeof(value_text) ||
            fprintf(file, "%s\"%s\": %s%s\n", indent, info.name, value_text,
                    separator) < 0) {
            return false;
        }
    }
    return true;
}


bool config_json_write(FILE *file, const app_config_profiles_t *profiles) {
    if (fprintf(file, "{\n  \"format_version\": %d,\n"
                      "  \"mc_parameters\": {\n",
                CONFIG_FORMAT_VERSION) < 0 ||
        !write_parameter_object(file, &profiles->shared_config,
                                APP_PARAMETER_SCOPE_SHARED, "    ") ||
        fprintf(file, "  },\n"
                      "  \"vario_parameter_sets\": [\n") < 0) {
        return false;
    }
    for (size_t index = 0U; index < profiles->count; index++) {
        const app_config_profile_t *profile = &profiles->profiles[index];
        const char *separator = "";

        if (index + 1U < profiles->count) {
            separator = ",";
        }

        if (fprintf(file,
                    "    {\n      \"parameter_number\": %u,\n"
                    "      \"parameters\": {\n",
                    (unsigned int) profile->parameter_number) < 0 ||
            !write_parameter_object(file, &profile->config,
                                    APP_PARAMETER_SCOPE_PROFILE, "        ") ||
            fprintf(file, "      }\n    }%s\n", separator) < 0) {
            return false;
        }
    }
    return fprintf(file, "  ]\n}\n") >= 0;
}
