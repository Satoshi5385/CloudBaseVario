#pragma once

#include <stdint.h>

#include "domain/board_identity.h"
#include "esp_err.h"

typedef enum {
    BOARD_IDENTITY_LOAD_VALID = 0,
    BOARD_IDENTITY_LOAD_MISSING,
    BOARD_IDENTITY_LOAD_INVALID,
    BOARD_IDENTITY_LOAD_UNSUPPORTED,
    BOARD_IDENTITY_LOAD_IO_ERROR,
} board_identity_load_result_t;

typedef struct {
    board_identity_load_result_t result;
    esp_err_t error;
} board_identity_storage_diagnostics_t;

/**
 * @brief Load and strictly validate the immutable manufacturing identity.
 * @param[out] identity Destination identity.
 * @param[out] diagnostics Optional load diagnostics.
 * @return Detailed load result.
 */
board_identity_load_result_t board_identity_storage_load(
    board_identity_t *identity,
    board_identity_storage_diagnostics_t *diagnostics);

/** Return a stable diagnostic name for an identity load result. */
const char *board_identity_load_result_name(
    board_identity_load_result_t result);
