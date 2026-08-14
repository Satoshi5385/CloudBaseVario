#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    FIRMWARE_UPDATE_IDLE = 0,
    FIRMWARE_UPDATE_DEFERRED_POWER,
    FIRMWARE_UPDATE_VALIDATING,
    FIRMWARE_UPDATE_WRITING,
    FIRMWARE_UPDATE_STAGED,
    FIRMWARE_UPDATE_PENDING_CONFIRMATION,
    FIRMWARE_UPDATE_CONFIRMED,
    FIRMWARE_UPDATE_REJECTED,
    FIRMWARE_UPDATE_ROLLED_BACK,
    FIRMWARE_UPDATE_STORAGE_BUSY,
} firmware_update_state_t;

typedef struct {
    firmware_update_state_t state;
    esp_err_t last_error;
    uint32_t image_size_bytes;
    uint32_t bytes_written;
    bool confirmation_required;
    bool required_workers_started;
    bool external_power_present;
    bool battery_valid;
    bool update_power_allowed;
    float battery_voltage_v;
    float minimum_battery_voltage_v;
    char target_partition[17];
    char image_version[33];
    char image_fingerprint[65];
} firmware_update_diagnostics_t;

/**
 * @brief Check whether the running image is in its rollback-validation boot.
 *
 * This read-only check is safe before storage, USB, and application resources
 * are initialized. It is used to bypass the physical power-on hold only for
 * the first boot of a newly staged OTA image.
 */
bool firmware_update_running_image_pending_verify(void);

/**
 * @brief Reconcile update state files and apply UPDATE.BIN when permitted.
 *
 * Must run before TinyUSB is exposed and before normal application tasks.
 * USB power permits an update without a battery reading. Otherwise the
 * battery reading must be valid and strictly above the policy threshold.
 * A successful staging operation restarts and does not return.
 */
esp_err_t firmware_update_process_boot(bool external_power_present,
                                       bool battery_valid,
                                       float battery_voltage_v);

/** Start the ten-second first-boot rollback confirmation gate if required. */
esp_err_t firmware_update_begin_confirmation(void);

/**
 * @brief Wait until first-boot confirmation and state-file cleanup complete.
 *
 * This must complete successfully before TinyUSB is exposed after an update.
 * Returns immediately when no confirmation is required.
 */
esp_err_t firmware_update_wait_for_confirmation(uint32_t timeout_ms);

/** Report that all required long-lived application workers were created. */
void firmware_update_mark_workers_started(void);

/** Snapshot firmware-update state, power policy, and image diagnostics. */
void firmware_update_get_diagnostics(
    firmware_update_diagnostics_t *diagnostics);

/** Return the diagnostic name of a firmware update state. */
const char *firmware_update_state_name(firmware_update_state_t state);
