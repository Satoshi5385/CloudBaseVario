#pragma once

#include <stdbool.h>
#include <stddef.h>

#define FIRMWARE_METADATA_VERSION_CAPACITY 32U
#define FIRMWARE_METADATA_HASH_CAPACITY 8U

typedef struct {
    char version[FIRMWARE_METADATA_VERSION_CAPACITY];
    char git_hash[FIRMWARE_METADATA_HASH_CAPACITY];
} firmware_metadata_t;

/**
 * @brief Split an embedded application version into release version and hash.
 *
 * Current images use "major.minor.patch+hhhhhhh". Legacy images containing
 * only a seven-character lowercase hexadecimal Git hash remain recognizable.
 * Unknown or malformed values return false and produce "-" placeholders.
 */
bool firmware_metadata_parse(const char *embedded, size_t capacity,
                             firmware_metadata_t *metadata);
