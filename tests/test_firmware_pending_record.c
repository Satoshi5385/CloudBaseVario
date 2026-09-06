#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "domain/firmware_pending_record.h"

#define MAX_PAYLOAD UINT32_C(0x37f000)
#define TARGET_ADDRESS UINT32_C(0x820000)

static void initialize(firmware_pending_record_t *record,
                       const uint8_t digest[FIRMWARE_PENDING_DIGEST_SIZE]) {
    firmware_pending_record_initialize(
        record, UINT32_C(0x123456), TARGET_ADDRESS, digest,
        "0.1.3+abcdef0", "ota_0");
}

int main(void) {
    firmware_pending_record_t record;
    uint8_t digest[FIRMWARE_PENDING_DIGEST_SIZE];
    uint8_t other_digest[FIRMWARE_PENDING_DIGEST_SIZE];

    for (size_t index = 0U; index < sizeof(digest); index++) {
        digest[index] = (uint8_t) index;
        other_digest[index] = (uint8_t) (index + 1U);
    }
    initialize(&record, digest);
    assert(firmware_pending_record_validate(&record, MAX_PAYLOAD));
    assert(firmware_pending_record_matches(
        &record, TARGET_ADDRESS, MAX_PAYLOAD, "ota_0", digest));
    assert(!firmware_pending_record_matches(
        &record, TARGET_ADDRESS + 1U, MAX_PAYLOAD, "ota_0", digest));
    assert(!firmware_pending_record_matches(
        &record, TARGET_ADDRESS, record.payload_size - 1U, "ota_0", digest));
    assert(!firmware_pending_record_matches(
        &record, TARGET_ADDRESS, MAX_PAYLOAD, "ota_1", digest));
    assert(!firmware_pending_record_matches(
        &record, TARGET_ADDRESS, MAX_PAYLOAD, "ota_0", other_digest));

    record.crc32 ^= UINT32_C(1);
    assert(!firmware_pending_record_validate(&record, MAX_PAYLOAD));
    initialize(&record, digest);
    record.magic[0] ^= UINT8_C(1);
    assert(!firmware_pending_record_validate(&record, MAX_PAYLOAD));
    initialize(&record, digest);
    record.record_size = 0U;
    assert(!firmware_pending_record_validate(&record, MAX_PAYLOAD));
    initialize(&record, digest);
    assert(!firmware_pending_record_validate(
        &record, record.payload_size - 1U));

    puts("firmware pending record tests passed");
    return 0;
}
