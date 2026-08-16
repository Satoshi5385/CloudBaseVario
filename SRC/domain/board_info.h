#pragma once

#include <stdbool.h>

#include "domain/board_identity.h"
#include "domain/firmware_metadata.h"

#define BOARD_INFO_TEXT_CAPACITY 192U

/**
 * @brief Format the canonical user-visible board-information text file.
 *
 * The caller must provide a valid identity, its matching descriptor, and
 * parsed firmware metadata. The resulting ASCII text uses CRLF line endings
 * for host-friendly display on the USB FAT volume.
 */
bool board_info_format(const board_identity_t *identity,
                       const board_descriptor_t *descriptor,
                       const firmware_metadata_t *firmware,
                       char output[BOARD_INFO_TEXT_CAPACITY]);
