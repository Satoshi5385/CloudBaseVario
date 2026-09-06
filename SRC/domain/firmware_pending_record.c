#include "domain/firmware_pending_record.h"

#include <stddef.h>
#include <string.h>

static const uint8_t pending_magic[8] = {
    'C', 'B', 'V', 'P', 'N', 'D', '0', '1',
};

_Static_assert(sizeof(firmware_pending_record_t) == 108U,
               "firmware pending record format drift");

static uint32_t record_crc32(const void *data, size_t length) {
    const uint8_t *bytes = (const uint8_t *) data;
    uint32_t crc = UINT32_MAX;

    for (size_t index = 0U; index < length; index++) {
        crc ^= bytes[index];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            uint32_t mask = (uint32_t) -(int32_t) (crc & 1U);
            crc = (crc >> 1U) ^ (UINT32_C(0xedb88320) & mask);
        }
    }
    return ~crc;
}

static void copy_text(char *destination, size_t capacity,
                      const char *source) {
    size_t index = 0U;

    if (destination == NULL || capacity == 0U) {
        return;
    }
    if (source != NULL) {
        while (index + 1U < capacity && source[index] != '\0') {
            destination[index] = source[index];
            index += 1U;
        }
    }
    destination[index] = '\0';
}

void firmware_pending_record_initialize(
    firmware_pending_record_t *record, uint32_t payload_size,
    uint32_t target_address,
    const uint8_t app_elf_sha256[FIRMWARE_PENDING_DIGEST_SIZE],
    const char *version, const char *target_partition) {
    if (record == NULL || app_elf_sha256 == NULL) {
        return;
    }
    memset(record, 0, sizeof(*record));
    memcpy(record->magic, pending_magic, sizeof(record->magic));
    record->format_version = FIRMWARE_PENDING_RECORD_VERSION;
    record->record_size = (uint16_t) sizeof(*record);
    record->payload_size = payload_size;
    record->target_address = target_address;
    memcpy(record->app_elf_sha256, app_elf_sha256,
           sizeof(record->app_elf_sha256));
    copy_text(record->version, sizeof(record->version), version);
    copy_text(record->target_partition,
              sizeof(record->target_partition), target_partition);
    record->crc32 = record_crc32(
        record, offsetof(firmware_pending_record_t, crc32));
}

bool firmware_pending_record_validate(
    const firmware_pending_record_t *record, uint32_t maximum_payload_size) {
    return record != NULL &&
           memcmp(record->magic, pending_magic, sizeof(pending_magic)) == 0 &&
           record->format_version == FIRMWARE_PENDING_RECORD_VERSION &&
           record->record_size == sizeof(*record) &&
           record->payload_size > 0U &&
           record->payload_size <= maximum_payload_size &&
           memchr(record->version, '\0', sizeof(record->version)) != NULL &&
           memchr(record->target_partition, '\0',
                  sizeof(record->target_partition)) != NULL &&
           record->crc32 == record_crc32(
                                record,
                                offsetof(firmware_pending_record_t, crc32));
}

bool firmware_pending_record_matches(
    const firmware_pending_record_t *record, uint32_t partition_address,
    uint32_t partition_payload_capacity, const char *partition_label,
    const uint8_t app_elf_sha256[FIRMWARE_PENDING_DIGEST_SIZE]) {
    return record != NULL && partition_label != NULL &&
           app_elf_sha256 != NULL &&
           record->target_address == partition_address &&
           record->payload_size <= partition_payload_capacity &&
           strncmp(record->target_partition, partition_label,
                   sizeof(record->target_partition)) == 0 &&
           memcmp(record->app_elf_sha256, app_elf_sha256,
                  sizeof(record->app_elf_sha256)) == 0;
}
