import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN_SOURCE = (ROOT / "SRC/app/startup.c").read_text(encoding="utf-8")
USB_SOURCE = (ROOT / "SRC/platform/usb_device_service.c").read_text(
    encoding="utf-8"
)
USB_HEADER = (ROOT / "SRC/platform/usb_device_service.h").read_text(
    encoding="utf-8"
)
USB_POLICY_SOURCE = (ROOT / "SRC/domain/usb_storage_policy.c").read_text(
    encoding="utf-8"
)


class UsbStartupPolicyTests(unittest.TestCase):
    def test_formal_cdc_starts_before_application_workers(self) -> None:
        composite_start = MAIN_SOURCE.index("usb_result = usb_device_start();")
        workers_start = MAIN_SOURCE.index("ret = app_tasks_start();")

        self.assertLess(composite_start, workers_start)

    def test_msc_is_enabled_only_after_startup_waits(self) -> None:
        workers_start = MAIN_SOURCE.index("ret = app_tasks_start();")
        ota_wait = MAIN_SOURCE.index("firmware_update_wait_for_confirmation(")
        calibration_wait = MAIN_SOURCE.index("xEventGroupWaitBits(")
        msc_enable = MAIN_SOURCE.index("usb_result = usb_device_enable_msc();")

        self.assertLess(workers_start, ota_wait)
        self.assertLess(ota_wait, msc_enable)
        self.assertLess(calibration_wait, msc_enable)
        self.assertIn("startup_gate_result == ESP_OK", MAIN_SOURCE)

    def test_attach_retains_app_ownership_while_msc_gate_is_closed(self) -> None:
        self.assertIn("msc_policy_mutex", USB_SOURCE)
        self.assertIn(
            "usb_storage_policy_restore_on_attach(", USB_SOURCE
        )
        self.assertIn("return !exposure_enabled && storage_registered", USB_POLICY_SOURCE)
        self.assertIn(
            "TINYUSB_MSC_STORAGE_MOUNT_APP", USB_SOURCE
        )
        self.assertIn("usb_storage_policy_restore_on_detach(", USB_SOURCE)

    def test_msc_exposure_is_explicit_and_diagnosable(self) -> None:
        self.assertIn("esp_err_t usb_device_enable_msc(void);", USB_HEADER)
        self.assertIn("bool msc_enabled;", USB_HEADER)
        self.assertIn("usb_diagnostics.msc_enabled = true;", USB_SOURCE)
        self.assertIn(
            "TINYUSB_MSC_STORAGE_MOUNT_USB", USB_SOURCE
        )

    def test_calibration_debug_usb_variant_is_removed(self) -> None:
        combined = MAIN_SOURCE + USB_SOURCE + USB_HEADER

        self.assertNotIn("start_calibration_" + "debug", combined)
        self.assertNotIn("calibration_" + "debug_mode", combined)
        self.assertNotIn("USB_CDC_" + "ONLY", combined)


if __name__ == "__main__":
    unittest.main()
