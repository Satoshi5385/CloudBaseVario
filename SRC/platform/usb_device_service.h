#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "domain/app_config.h"
#include "esp_err.h"
#include "platform/config_storage.h"

typedef enum {
    USB_STORAGE_UNAVAILABLE = 0,
    USB_STORAGE_APP_OWNED,
    USB_STORAGE_SWITCHING,
    USB_STORAGE_HOST_OWNED,
} usb_storage_owner_t;

typedef struct {
    bool driver_ready;
    bool cdc_ready;
    bool msc_driver_ready;
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
    uint32_t mount_failure_count;
    uint32_t format_required_count;
    uint32_t rx_error_count;
    uint32_t tx_error_count;
} usb_device_diagnostics_t;

/**
 * @brief Prepare the shared FAT volume and load parameters before USB starts.
 *
 * A blank or damaged volume is never formatted implicitly. Formatting occurs
 * only when @p format_config_storage is true (the SW2+SW3 boot gesture).
 */
esp_err_t usb_device_storage_init(app_config_t *config,
                                  bool format_config_storage);

/** Start the self-powered TinyUSB CDC + MSC composite device. */
esp_err_t usb_device_start(void);

/** Save parameters while the application owns the FAT volume. */
esp_err_t usb_device_save_config(const app_config_t *config);

/** Receive available CDC bytes without blocking. */
bool usb_device_read(uint8_t *buffer, size_t capacity, size_t *length);

/** Queue and flush CDC text without blocking normal firmware tasks. */
bool usb_device_write(const char *text);

/** Return true while the CDC host has asserted DTR. */
bool usb_device_cdc_connected(void);

/** Return true while VBUS or an enumerated USB connection is present. */
bool usb_device_bus_active(void);

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

/** Snapshot TinyUSB and shared-storage diagnostics. */
void usb_device_get_diagnostics(usb_device_diagnostics_t *diagnostics);

const char *usb_device_storage_owner_name(usb_storage_owner_t owner);
