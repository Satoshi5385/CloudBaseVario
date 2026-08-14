#pragma once

#include <stdbool.h>

#include "domain/app_config.h"
#include "esp_err.h"

#define CONFIG_FORMAT_VERSION 1

typedef enum {
    CONFIG_LOAD_DEFAULT_NO_FILE = 0,
    CONFIG_LOAD_VALID_FILE,
    CONFIG_LOAD_RECOVERED_FILE,
    CONFIG_LOAD_INVALID_FILE,
    CONFIG_LOAD_IO_ERROR,
} config_load_result_t;

#define CONFIG_DIAGNOSTIC_KEY_LENGTH 48U

typedef enum {
    CONFIG_SOURCE_BUILTIN_DEFAULT = 0,
    CONFIG_SOURCE_FILE,
    CONFIG_SOURCE_RECOVERED_BACKUP,
} config_source_t;

typedef enum {
    CONFIG_VALIDATION_OK = 0,
    CONFIG_VALIDATION_EMPTY_FILE,
    CONFIG_VALIDATION_FILE_TOO_LARGE,
    CONFIG_VALIDATION_JSON_SYNTAX,
    CONFIG_VALIDATION_ROOT_NOT_OBJECT,
    CONFIG_VALIDATION_UNKNOWN_TOP_LEVEL_KEY,
    CONFIG_VALIDATION_DUPLICATE_KEY,
    CONFIG_VALIDATION_MISSING_FORMAT_VERSION,
    CONFIG_VALIDATION_FORMAT_VERSION_TYPE,
    CONFIG_VALIDATION_UNSUPPORTED_FORMAT_VERSION,
    CONFIG_VALIDATION_MISSING_PARAMETERS,
    CONFIG_VALIDATION_PARAMETERS_NOT_OBJECT,
    CONFIG_VALIDATION_MISSING_PARAMETER_SETS,
    CONFIG_VALIDATION_PARAMETER_SETS_NOT_ARRAY,
    CONFIG_VALIDATION_PROFILE_COUNT,
    CONFIG_VALIDATION_PROFILE_NOT_OBJECT,
    CONFIG_VALIDATION_MISSING_PARAMETER_NUMBER,
    CONFIG_VALIDATION_PARAMETER_NUMBER_TYPE,
    CONFIG_VALIDATION_PARAMETER_NUMBER_RANGE,
    CONFIG_VALIDATION_DUPLICATE_PARAMETER_NUMBER,
    CONFIG_VALIDATION_UNKNOWN_PARAMETER,
    CONFIG_VALIDATION_PARAMETER_TYPE,
    CONFIG_VALIDATION_NONFINITE_VALUE,
    CONFIG_VALIDATION_PARAMETER_RANGE,
    CONFIG_VALIDATION_RELATION,
    CONFIG_VALIDATION_NO_MEMORY,
    CONFIG_VALIDATION_IO_ERROR,
} config_validation_result_t;

typedef struct {
    config_source_t source;
    config_validation_result_t validation;
    int32_t format_version;
    int32_t io_error;
    char key[CONFIG_DIAGNOSTIC_KEY_LENGTH];
} config_storage_diagnostics_t;

/**
 * @brief Load and strictly validate setting.json from a mounted FAT volume.
 *
 * All nine shared parameters and all 22 parameters in every version-1
 * profile are required. An invalid file never partially changes the returned
 * configuration.
 */
config_load_result_t config_storage_load(const char *base_path,
                                         app_config_profiles_t *profiles,
                                         config_storage_diagnostics_t *diagnostics);

/**
 * @brief Atomically write, sync, read back, and rename setting.json.
 * @return ESP_OK on verified save, otherwise an I/O or validation error.
 */
esp_err_t config_storage_save(const char *base_path,
                              const app_config_profiles_t *profiles);

/** Return the diagnostic name of a configuration source. */
const char *config_storage_source_name(config_source_t source);
/** Return the diagnostic name of a validation result. */
const char *config_storage_validation_name(
    config_validation_result_t validation);
