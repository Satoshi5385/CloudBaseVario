#include "domain/usb_storage_policy.h"

bool usb_storage_policy_restore_on_attach(bool exposure_enabled,
                                          bool storage_registered) {
    return !exposure_enabled && storage_registered;
}

bool usb_storage_policy_restore_on_detach(bool storage_registered,
                                          usb_storage_owner_t owner) {
    return storage_registered && owner != USB_STORAGE_APP_OWNED &&
           owner != USB_STORAGE_UNAVAILABLE;
}

usb_storage_owner_t usb_storage_policy_mount_owner(bool app_mount) {
    if (app_mount) {
        return USB_STORAGE_APP_OWNED;
    }
    return USB_STORAGE_HOST_OWNED;
}

bool usb_storage_policy_app_io_allowed(usb_storage_owner_t owner) {
    return owner == USB_STORAGE_APP_OWNED;
}

bool usb_storage_policy_write_session_idle(bool active,
                                           uint32_t pending_writes,
                                           int64_t idle_us,
                                           int64_t required_idle_us) {
    return active && pending_writes == 0U && required_idle_us >= 0 &&
           idle_us >= required_idle_us;
}

bool usb_storage_policy_write_session_forced_exit(bool active,
                                                  uint32_t pending_writes) {
    return active && pending_writes == 0U;
}
