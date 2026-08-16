#include "domain/board_info.h"

#include <inttypes.h>
#include <stdio.h>

bool board_info_format(const board_identity_t *identity,
                       const board_descriptor_t *descriptor,
                       const firmware_metadata_t *firmware,
                       const firmware_authentication_t *authentication,
                       char output[BOARD_INFO_TEXT_CAPACITY]) {
    int written;
    const char *key_id;
    const char *image_sha256;

    if (identity == NULL || descriptor == NULL || firmware == NULL ||
        authentication == NULL ||
        output == NULL || !board_identity_validate(identity) ||
        board_identity_descriptor(identity->board_id) != descriptor) {
        return false;
    }

    key_id = authentication->key_id;
    image_sha256 = authentication->payload_sha256;
    if (key_id[0] == '\0') {
        key_id = "-";
    }
    if (image_sha256[0] == '\0') {
        image_sha256 = "-";
    }
    written = snprintf(
        output, BOARD_INFO_TEXT_CAPACITY,
        "Board name: %s\r\n"
        "Board ID: 0x%04" PRIx16 "\r\n"
        "Serial number: %s\r\n"
        "Firmware version: %s\r\n"
        "Firmware git hash: %s\r\n"
        "Firmware authenticity: %s\r\n"
        "Authenticity key ID: %s\r\n"
        "Firmware image SHA-256: %s\r\n",
        descriptor->model, identity->board_id, identity->serial,
        firmware->version, firmware->git_hash,
        firmware_authenticity_name(authentication->authenticity),
        key_id, image_sha256);
    return written > 0 && (size_t) written < BOARD_INFO_TEXT_CAPACITY;
}
