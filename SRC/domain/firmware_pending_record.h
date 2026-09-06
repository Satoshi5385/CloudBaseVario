#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FIRMWARE_PENDING_RECORD_VERSION UINT16_C(1)
#define FIRMWARE_PENDING_DIGEST_SIZE 32U
#define FIRMWARE_PENDING_VERSION_SIZE 32U
#define FIRMWARE_PENDING_PARTITION_SIZE 17U

typedef struct {
    uint8_t magic[8];
    uint16_t format_version;
    uint16_t record_size;
    uint32_t payload_size;
    uint32_t target_address;
    uint8_t app_elf_sha256[FIRMWARE_PENDING_DIGEST_SIZE];
    char version[FIRMWARE_PENDING_VERSION_SIZE];
    char target_partition[FIRMWARE_PENDING_PARTITION_SIZE];
    uint8_t reserved[3];
    uint32_t crc32;
} firmware_pending_record_t;

/** Initialize a fixed-size PSRAM-update pending record and its CRC32. */
void firmware_pending_record_initialize(
    firmware_pending_record_t *record, uint32_t payload_size,
    uint32_t target_address,
    const uint8_t app_elf_sha256[FIRMWARE_PENDING_DIGEST_SIZE],
    const char *version, const char *target_partition);

/** Validate record identity, bounds, string termination, and CRC32. */
bool firmware_pending_record_validate(
    const firmware_pending_record_t *record, uint32_t maximum_payload_size);

/** Match a valid record to an OTA partition and application digest. */
bool firmware_pending_record_matches(
    const firmware_pending_record_t *record, uint32_t partition_address,
    uint32_t partition_payload_capacity, const char *partition_label,
    const uint8_t app_elf_sha256[FIRMWARE_PENDING_DIGEST_SIZE]);
