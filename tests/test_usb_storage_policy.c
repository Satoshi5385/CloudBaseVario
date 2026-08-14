#include <assert.h>
#include <stdio.h>

#include "domain/usb_storage_policy.h"

int main(void) {
    assert(usb_storage_policy_restore_on_attach(false, true));
    assert(!usb_storage_policy_restore_on_attach(true, true));
    assert(!usb_storage_policy_restore_on_attach(false, false));
    assert(usb_storage_policy_restore_on_detach(
        true, USB_STORAGE_HOST_OWNED));
    assert(usb_storage_policy_restore_on_detach(
        true, USB_STORAGE_SWITCHING));
    assert(!usb_storage_policy_restore_on_detach(
        true, USB_STORAGE_APP_OWNED));
    assert(!usb_storage_policy_restore_on_detach(
        true, USB_STORAGE_UNAVAILABLE));
    assert(usb_storage_policy_mount_owner(true) ==
           USB_STORAGE_APP_OWNED);
    assert(usb_storage_policy_mount_owner(false) ==
           USB_STORAGE_HOST_OWNED);
    assert(usb_storage_policy_app_io_allowed(USB_STORAGE_APP_OWNED));
    assert(!usb_storage_policy_app_io_allowed(USB_STORAGE_HOST_OWNED));
    assert(!usb_storage_policy_write_session_idle(true, 0U, 999999, 1000000));
    assert(usb_storage_policy_write_session_idle(true, 0U, 1000000, 1000000));
    assert(!usb_storage_policy_write_session_idle(true, 1U, 1000000, 1000000));
    assert(!usb_storage_policy_write_session_idle(false, 0U, 1000000, 1000000));
    assert(usb_storage_policy_write_session_forced_exit(true, 0U));
    assert(!usb_storage_policy_write_session_forced_exit(true, 1U));
    puts("usb_storage_policy tests passed");
    return 0;
}
