#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "domain/app_config.h"
#include "domain/usb_storage_policy.h"
#include "esp_err.h"
#include "platform/config_storage.h"
#include "platform/imu_calibration_storage.h"

typedef enum {
    USB_RECOVERY_MEDIUM_NONE = 0,
    USB_RECOVERY_MEDIUM_PSRAM,
} usb_recovery_medium_t;

typedef struct {
    bool driver_ready;
    bool cdc_ready;
    bool msc_driver_ready;
    bool msc_enabled;
    bool msc_media_ready;
    bool device_attached;
    bool cdc_connected;
    bool vbus_present;
    bool storage_ready;
    usb_storage_owner_t storage_owner;
    config_load_result_t load_result;
    config_storage_diagnostics_t config;
    esp_err_t last_storage_error;
    esp_err_t last_save_result;
    uint32_t attach_count;
    uint32_t detach_count;
    uint32_t host_release_count;
    uint32_t mount_failure_count;
    uint32_t format_required_count;
    uint32_t rx_error_count;
    uint32_t tx_error_count;
    bool storage_mode_active;
    uint32_t pending_write_count;
    uint32_t storage_mode_start_count;
    uint32_t storage_mode_end_count;
    uint32_t storage_mode_quiesce_timeout_count;
    uint32_t msc_write_count;
    uint32_t msc_write_error_count;
    uint64_t msc_written_bytes;
    uint32_t last_msc_write_duration_us;
    uint32_t max_msc_write_duration_us;
    usb_recovery_medium_t recovery_medium;
    bool recovery_config_ready;
    esp_err_t recovery_config_error;
    uint32_t recovery_disk_size_bytes;
    uint32_t psram_free_before_bytes;
    uint32_t psram_largest_before_bytes;
    uint32_t psram_free_after_bytes;
    uint32_t psram_largest_after_bytes;
    esp_err_t psram_allocation_error;
    uint32_t psram_read_count;
    uint32_t psram_read_error_count;
    uint32_t psram_write_count;
    uint32_t psram_write_error_count;
    uint64_t psram_read_bytes;
    uint64_t psram_written_bytes;
} usb_device_diagnostics_t;

typedef bool (*usb_storage_mode_begin_cb_t)(uint32_t timeout_ms, void *arg);
typedef void (*usb_storage_mode_end_cb_t)(void *arg);
typedef void (*usb_storage_progress_cb_t)(void *arg);

/**
 * @brief Prepare the shared FAT volume and load parameters before USB starts.
 *
 * A blank or damaged volume is never formatted by runtime firmware.
 * @p progress_cb is called between bounded storage operations and may be NULL.
 */
esp_err_t usb_device_storage_init(app_config_profiles_t *profiles,
                                  usb_storage_progress_cb_t progress_cb,
                                  void *progress_arg);

/**
 * @brief Prepare the private config state mount and PSRAM recovery MSC.
 *
 * This path never formats or exposes config Flash. It requires VBUS, creates a
 * zero-filled 4 MiB PSRAM FAT volume at /update, and leaves both volumes owned
 * by the application until recovery explicitly exposes the PSRAM LUN.
 */
esp_err_t usb_device_recovery_storage_init(
    usb_storage_progress_cb_t progress_cb, void *progress_arg);

/** Release the recovery-only PSRAM disk and its private config mount. */
esp_err_t usb_device_recovery_storage_deinit(void);

/** Return true when recovery may persist status and pending state to config. */
bool usb_device_recovery_storage_ready(void);

/** Start the self-powered TinyUSB CDC + MSC composite device. */
esp_err_t usb_device_start(void);

/** Start TinyUSB for recovery with an eFuse-MAC-derived serial number. */
esp_err_t usb_device_start_recovery(void);

/**
 * @brief Match the normal application TinyUSB lifecycle to the raw VBUS input.
 *
 * Starts the TinyUSB task, PHY, and CDC when VBUS is present. When VBUS is
 * absent, stops them only after the existing MSC write/ownership checks allow
 * shutdown. The Flash-backed application FAT and its sleeping MSC write worker
 * remain initialized.
 *
 * @return ESP_OK when the requested state is reached, or a retryable/error
 *         result from USB startup or the safe shutdown path.
 */
esp_err_t usb_device_update_vbus(void);

/**
 * @brief Stop the application TinyUSB task and PHY without deleting FAT/MSC storage.
 *
 * The call is idempotent, but refuses to stop while an MSC write session is
 * active or a write remains pending.
 */
esp_err_t usb_device_stop(void);

/**
 * Allow the already-enumerated composite device to expose its MSC medium.
 * Until this is called, USB attach events return the FAT volume to APP ownership
 * so startup calibration, OTA cleanup, and configuration writes can complete.
 */
esp_err_t usb_device_enable_msc(void);

/**
 * @brief Permit normal MSC exposure now or after the next VBUS connection.
 *
 * This records the completed startup gate even when TinyUSB is stopped because
 * VBUS is absent. A later usb_device_update_vbus() call restores MSC exposure.
 */
esp_err_t usb_device_request_msc(void);

/**
 * Wait for a completed HOST-to-APP ownership transition after @p release_count.
 * A successful result also guarantees that no accepted MSC write remains.
 */
esp_err_t usb_device_wait_for_host_release(uint32_t release_count,
                                           uint32_t timeout_ms);

/** Register application quiesce/resume hooks used around actual MSC writes. */
void usb_device_set_storage_mode_callbacks(
    usb_storage_mode_begin_cb_t begin_cb,
    usb_storage_mode_end_cb_t end_cb, void *arg);

/** Return true from the first WRITE(10) until one second of write inactivity. */
bool usb_device_storage_mode_active(void);

/** Save parameters while the application owns the FAT volume. */
esp_err_t usb_device_save_config(const app_config_profiles_t *profiles);

/** Load the private per-unit IMU calibration while the app owns the FAT. */
imu_calibration_storage_result_t usb_device_load_imu_calibration(
    imu_accel_calibration_t *calibration,
    imu_calibration_storage_diagnostics_t *diagnostics);

/** Atomically save the private per-unit IMU calibration. */
esp_err_t usb_device_save_imu_calibration(
    const imu_accel_calibration_t *calibration);

/** Receive available CDC bytes without blocking. */
bool usb_device_read(uint8_t *buffer, size_t capacity, size_t *length);

/** Queue and flush CDC text without blocking normal firmware tasks. */
bool usb_device_write(const char *text);

/** Return true while the CDC host has asserted DTR. */
bool usb_device_cdc_connected(void);

/** Return true while VBUS or an enumerated USB connection is present. */
bool usb_device_bus_active(void);

/** Return the raw USB VBUS GPIO state. */
bool usb_device_vbus_present(void);

/**
 * @brief Lock the application-owned FAT volume for a bounded operation.
 *
 * Returns ESP_ERR_INVALID_STATE while the host owns the MSC LUN.
 */
esp_err_t usb_device_storage_begin_app_io(uint32_t timeout_ms);

/** Release a successful usb_device_storage_begin_app_io() call. */
void usb_device_storage_end_app_io(void);

/** Return the mounted application path. */
const char *usb_device_storage_mount_path(void);

/** Return the recovery-only PSRAM FAT mount path. */
const char *usb_device_recovery_mount_path(void);

/** Snapshot TinyUSB and shared-storage diagnostics. */
void usb_device_get_diagnostics(usb_device_diagnostics_t *diagnostics);

/** Return the diagnostic name of a shared-storage owner. */
const char *usb_device_storage_owner_name(usb_storage_owner_t owner);

/** Return the diagnostic name of the recovery storage medium. */
const char *usb_device_recovery_medium_name(usb_recovery_medium_t medium);
