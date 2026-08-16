#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_partition.h"
#include "domain/firmware_authentication.h"

#define FIRMWARE_AUTH_HEADER_SIZE 164U
#define FIRMWARE_AUTH_RECORD_SIZE 4096U
#define FIRMWARE_AUTH_MAX_PAYLOAD_SIZE UINT32_C(0x37f000)
#define FIRMWARE_AUTH_SIGNATURE_LENGTH 64U

typedef struct __attribute__((packed)) {
    uint8_t magic[8];
    uint16_t format_version;
    uint16_t header_size;
    char project_name[32];
    uint16_t chip_id;
    uint16_t reserved;
    uint32_t payload_size;
    uint8_t payload_sha256[FIRMWARE_AUTH_SHA256_LENGTH];
    char key_id[FIRMWARE_AUTH_KEY_ID_LENGTH];
    uint8_t signature[FIRMWARE_AUTH_SIGNATURE_LENGTH];
} firmware_auth_header_t;

/** Validate an UPDATE.BIN container at its current file position. */
esp_err_t firmware_auth_verify_package(FILE *file, size_t file_size,
                                       const char *expected_project,
                                       firmware_auth_header_t *header);

/** Hash raw application bytes and verify their final-sector auth record. */
esp_err_t firmware_authenticate_partition(
    const esp_partition_t *partition, const char *expected_project,
    firmware_authentication_t *authentication);
