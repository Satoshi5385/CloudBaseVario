#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    USB_STORAGE_UNAVAILABLE = 0,
    USB_STORAGE_APP_OWNED,
    USB_STORAGE_SWITCHING,
    USB_STORAGE_HOST_OWNED,
} usb_storage_owner_t;

/** Decide whether attach must immediately restore application ownership. */
bool usb_storage_policy_restore_on_attach(bool exposure_enabled,
                                          bool storage_registered);

/** Decide whether detach must restore application ownership. */
bool usb_storage_policy_restore_on_detach(bool storage_registered,
                                          usb_storage_owner_t owner);

/** Convert a completed MSC mount target into the stable owner state. */
usb_storage_owner_t usb_storage_policy_mount_owner(bool app_mount);

/** Return whether a bounded application FAT operation may start. */
bool usb_storage_policy_app_io_allowed(usb_storage_owner_t owner);

/** Return true when an active write session may end after its idle timeout. */
bool usb_storage_policy_write_session_idle(bool active,
                                           uint32_t pending_writes,
                                           int64_t idle_us,
                                           int64_t required_idle_us);

/** Return true when eject/detach may end an active write session immediately. */
bool usb_storage_policy_write_session_forced_exit(bool active,
                                                  uint32_t pending_writes);
