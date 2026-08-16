#include "domain/board_info.h"

#include <inttypes.h>
#include <stdio.h>

bool board_info_format(const board_identity_t *identity,
                       const board_descriptor_t *descriptor,
                       const firmware_metadata_t *firmware,
                       char output[BOARD_INFO_TEXT_CAPACITY]) {
    int written;

    if (identity == NULL || descriptor == NULL || firmware == NULL ||
        output == NULL || !board_identity_validate(identity) ||
        board_identity_descriptor(identity->board_id) != descriptor) {
        return false;
    }

    written = snprintf(
        output, BOARD_INFO_TEXT_CAPACITY,
        "Board name: %s\r\n"
        "Board ID: 0x%04" PRIx16 "\r\n"
        "Serial number: %s\r\n"
        "Firmware version: %s\r\n"
        "Firmware git hash: %s\r\n",
        descriptor->model, identity->board_id, identity->serial,
        firmware->version, firmware->git_hash);
    return written > 0 && (size_t) written < BOARD_INFO_TEXT_CAPACITY;
}
