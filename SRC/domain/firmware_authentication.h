#pragma once

#include <stdint.h>

#define FIRMWARE_AUTH_KEY_ID_LENGTH 16U
#define FIRMWARE_AUTH_SHA256_LENGTH 32U

typedef enum {
    FIRMWARE_AUTH_OFFICIAL = 0,
    FIRMWARE_AUTH_NON_OFFICIAL,
    FIRMWARE_AUTH_UNKNOWN,
} firmware_authenticity_t;

typedef struct {
    firmware_authenticity_t authenticity;
    char key_id[FIRMWARE_AUTH_KEY_ID_LENGTH + 1U];
    char payload_sha256[FIRMWARE_AUTH_SHA256_LENGTH * 2U + 1U];
} firmware_authentication_t;

/** Return the stable ASCII value used in INFO.TXT for an auth result. */
const char *firmware_authenticity_name(firmware_authenticity_t authenticity);
