#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include "domain/board_identity.h"
#include "domain/board_info.h"
#include "domain/firmware_metadata.h"
#include "domain/firmware_authentication.h"

static board_identity_t valid_identity(void) {
    return (board_identity_t){
        .schema_version = BOARD_IDENTITY_SCHEMA_VERSION,
        .board_id = BOARD_ID_AOHAZUKU_REV0,
        .serial = "CBV_A0_73I0j_0009",
    };
}

int main(void) {
    board_identity_t identity = valid_identity();
    const board_descriptor_t *descriptor =
        board_identity_descriptor(identity.board_id);
    firmware_metadata_t firmware = {0};
    firmware_authentication_t authentication = {
        .authenticity = FIRMWARE_AUTH_OFFICIAL,
        .key_id = "owner-key-2026!!",
        .payload_sha256 =
            "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef",
    };
    char text[BOARD_INFO_TEXT_CAPACITY] = {0};

    assert(firmware_metadata_parse("0.1.0+48a4472",
                                   sizeof("0.1.0+48a4472"), &firmware));
    assert(board_info_format(&identity, descriptor, &firmware, &authentication,
                             text));
    assert(strcmp(text,
                  "Board name: Aohazuku-Rev0\r\n"
                  "Board ID: 0x0100\r\n"
                  "Serial number: CBV_A0_73I0j_0009\r\n"
                  "Firmware version: 0.1.0\r\n"
                  "Firmware git hash: 48a4472\r\n"
                  "Firmware authenticity: OFFICIAL\r\n"
                  "Authenticity key ID: owner-key-2026!!\r\n"
                  "Firmware image SHA-256: "
                  "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\r\n") ==
           0);

    assert(!firmware_metadata_parse("0.1+48a4472", sizeof("0.1+48a4472"),
                                    &firmware));
    assert(board_info_format(&identity, descriptor, &firmware, &authentication,
                             text));
    assert(strstr(text, "Firmware version: -\r\n") != NULL);
    assert(strstr(text, "Firmware git hash: -\r\n") != NULL);

    identity.board_id = UINT16_C(0x9999);
    assert(!board_info_format(&identity, descriptor, &firmware, &authentication,
                              text));
    return 0;
}
