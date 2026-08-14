#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "platform/config_storage.h"

/** Parse one complete strict version-5 configuration document. */
bool config_json_parse(const char *json, size_t json_length,
                       app_config_profiles_t *profiles,
                       config_storage_diagnostics_t *diagnostics);

/** Write one canonical version-5 configuration document. */
bool config_json_write(FILE *file, const app_config_profiles_t *profiles);

/** Set one stable configuration validation diagnostic. */
void config_json_set_diagnostics(config_storage_diagnostics_t *diagnostics,
                                 config_validation_result_t validation,
                                 const char *key, int32_t io_error);

