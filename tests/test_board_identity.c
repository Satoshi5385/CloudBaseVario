#include <assert.h>
#include <string.h>

#include "domain/board_identity.h"

int main(void) {
    board_identity_t identity = {
        .schema_version = BOARD_IDENTITY_SCHEMA_VERSION,
        .board_id = BOARD_ID_AOHAZUKU_REV0,
        .serial = "CBV_A0_73I0j_0009",
    };

    assert(board_identity_validate(&identity));
    assert(board_serial_validate(identity.serial, "A0"));
    assert(!board_serial_validate("CBV_A0_73I0j_0000", "A0"));
    assert(!board_serial_validate("CBV_A1_73I0j_0009", "A0"));
    assert(!board_serial_validate("CBV_A0_73I0-_0009", "A0"));
    assert(!board_serial_validate(" CBV_A0_73I0j_0009", "A0"));

    identity.schema_version++;
    assert(!board_identity_validate(&identity));
    identity.schema_version = BOARD_IDENTITY_SCHEMA_VERSION;
    identity.board_id = UINT16_C(0xFFFF);
    assert(!board_identity_validate(&identity));
    identity.board_id = BOARD_ID_AOHAZUKU_REV0;
    (void) memcpy(identity.serial, "CBV_A1_73I0j_0009",
                  sizeof("CBV_A1_73I0j_0009"));
    assert(!board_identity_validate(&identity));
    return 0;
}
