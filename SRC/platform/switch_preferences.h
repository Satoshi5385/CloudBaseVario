#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "domain/app_types.h"
#include "esp_err.h"

typedef enum {
    SWITCH_PREFERENCES_SOURCE_DEFAULT = 0,
    SWITCH_PREFERENCES_SOURCE_NVS,
} switch_preferences_source_t;

typedef enum {
    SWITCH_PREFERENCES_LOAD_OK = 0,
    SWITCH_PREFERENCES_LOAD_LEGACY,
    SWITCH_PREFERENCES_LOAD_NOT_FOUND,
    SWITCH_PREFERENCES_LOAD_INVALID_SIZE,
    SWITCH_PREFERENCES_LOAD_UNSUPPORTED_VERSION,
    SWITCH_PREFERENCES_LOAD_INVALID_VALUE,
    SWITCH_PREFERENCES_LOAD_IO_ERROR,
} switch_preferences_load_result_t;

typedef struct {
    switch_preferences_source_t source;
    switch_preferences_load_result_t load_result;
    esp_err_t last_load_error;
    esp_err_t last_save_result;
    esp_err_t last_clear_result;
    uint32_t load_error_count;
    uint32_t save_count;
    uint32_t save_error_count;
    uint32_t clear_error_count;
} switch_preferences_diagnostics_t;

/** Set first-boot defaults: small volume, sink enabled, parameter number 1. */
void switch_preferences_set_defaults(switch_preferences_t *preferences);

/** Load and validate the coherent switch-state blob from NVS. */
switch_preferences_load_result_t switch_preferences_load(
    switch_preferences_t *preferences);

/** Save one coherent switch-state blob and commit it to NVS. */
esp_err_t switch_preferences_save(const switch_preferences_t *preferences);

/** Remove only the switch-state key, preserving all other NVS users. */
esp_err_t switch_preferences_clear(void);

/** Copy persistence diagnostics for DIAG STATUS. */
void switch_preferences_get_diagnostics(
    switch_preferences_diagnostics_t *diagnostics);

/** Return the diagnostic name of a switch-preference source. */
const char *switch_preferences_source_name(
    switch_preferences_source_t source);
/** Return the diagnostic name of a switch-preference load result. */
const char *switch_preferences_load_result_name(
    switch_preferences_load_result_t result);
