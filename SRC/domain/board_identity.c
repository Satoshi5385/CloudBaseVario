#include "domain/board_identity.h"

#include <stddef.h>
#include <string.h>

static const board_descriptor_t board_descriptors[] = {{
    .id = BOARD_ID_AOHAZUKU_REV0,
    .code = "A0",
    .model = "Aohazuku-Rev0",
    .imu_model = "ICM-42688P-HXY",
    .imu_address = UINT8_C(0x18),
    .imu_who_am_i = UINT8_C(0x6A),
    .usb_power_mode = BOARD_USB_POWER_SELF_POWERED_VBUS_GPIO,
    .audio_driver = BOARD_AUDIO_DRIVER_PAM8904E,
}};

static bool is_ascii_digit(char value) {
    return value >= '0' && value <= '9';
}

static bool is_ascii_alphanumeric(char value) {
    return is_ascii_digit(value) ||
           (value >= 'A' && value <= 'Z') ||
           (value >= 'a' && value <= 'z');
}

const board_descriptor_t *board_identity_descriptor(uint16_t board_id) {
    for (size_t index = 0U;
         index < sizeof(board_descriptors) / sizeof(board_descriptors[0]);
         index++) {
        if (board_descriptors[index].id == board_id) {
            return &board_descriptors[index];
        }
    }
    return NULL;
}

bool board_serial_validate(const char *serial, const char *board_code) {
    unsigned int sequence = 0U;

    if (serial == NULL || board_code == NULL || strlen(serial) != BOARD_SERIAL_TEXT_LENGTH ||
        strlen(board_code) != 2U || strncmp(serial, "CBV_", 4U) != 0 ||
        serial[4] != board_code[0] || serial[5] != board_code[1] ||
        serial[6] != '_' || serial[12] != '_') {
        return false;
    }
    for (size_t index = 7U; index <= 11U; index++) {
        if (!is_ascii_alphanumeric(serial[index])) {
            return false;
        }
    }
    for (size_t index = 13U; index <= 16U; index++) {
        if (!is_ascii_digit(serial[index])) {
            return false;
        }
        sequence = sequence * 10U + (unsigned int) (serial[index] - '0');
    }
    return sequence >= 1U && sequence <= 9999U;
}

bool board_identity_validate(const board_identity_t *identity) {
    const board_descriptor_t *descriptor = NULL;

    if (identity == NULL ||
        identity->schema_version != BOARD_IDENTITY_SCHEMA_VERSION) {
        return false;
    }
    descriptor = board_identity_descriptor(identity->board_id);
    return descriptor != NULL &&
           board_serial_validate(identity->serial, descriptor->code);
}
