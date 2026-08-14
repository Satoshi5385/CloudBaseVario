#pragma once

#include <stdbool.h>
#include <stdint.h>

#define BOARD_IDENTITY_SCHEMA_VERSION UINT8_C(1)
#define BOARD_ID_AOHAZUKU_REV0 UINT16_C(0x0100)
#define BOARD_SERIAL_TEXT_LENGTH 17U
#define BOARD_SERIAL_BUFFER_SIZE (BOARD_SERIAL_TEXT_LENGTH + 1U)

typedef enum {
    BOARD_USB_POWER_SELF_POWERED_VBUS_GPIO = 0,
} board_usb_power_mode_t;

typedef enum {
    BOARD_AUDIO_DRIVER_PAM8904E = 0,
} board_audio_driver_t;

typedef struct {
    uint16_t id;
    const char *code;
    const char *model;
    const char *imu_model;
    uint8_t imu_address;
    uint8_t imu_who_am_i;
    board_usb_power_mode_t usb_power_mode;
    board_audio_driver_t audio_driver;
} board_descriptor_t;

typedef struct {
    uint8_t schema_version;
    uint16_t board_id;
    char serial[BOARD_SERIAL_BUFFER_SIZE];
} board_identity_t;

/** Find an immutable descriptor for a supported board identifier. */
const board_descriptor_t *board_identity_descriptor(uint16_t board_id);

/** Validate the complete identity, including the serial's board code. */
bool board_identity_validate(const board_identity_t *identity);

/** Validate and split a serial into its fixed board, lot and sequence fields. */
bool board_serial_validate(const char *serial, const char *board_code);

