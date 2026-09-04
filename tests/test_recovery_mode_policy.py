from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
STARTUP = (ROOT / "SRC/app/startup.c").read_text(encoding="utf-8")
USB = (ROOT / "SRC/platform/usb_device_service.c").read_text(encoding="utf-8")
USB_HEADER = (ROOT / "SRC/platform/usb_device_service.h").read_text(
    encoding="utf-8"
)
UPDATE = (ROOT / "SRC/platform/firmware_update.c").read_text(encoding="utf-8")
UPDATE_HEADER = (ROOT / "SRC/platform/firmware_update.h").read_text(
    encoding="utf-8"
)
SPEC = (ROOT / "DOC/SW_spec.md").read_text(encoding="utf-8")


def body(source: str, start_text: str, end_text: str) -> str:
    start = source.index(start_text)
    return source[start : source.index(end_text, start)]


class RecoveryModePolicyTests(unittest.TestCase):
    def test_recovery_gesture_is_early_bounded_and_pending_verify_wins(self) -> None:
        gesture = body(
            STARTUP,
            "static startup_boot_gesture_t startup_recovery_gesture(void)",
            "static void recovery_stop",
        )
        startup = STARTUP[STARTUP.index("void app_startup_run(void)") :]
        safe_gpio = startup.index("board_init_safe_gpio();")
        pending = startup.index("firmware_update_running_image_pending_verify();")
        recovery = startup.index("boot_gesture = startup_recovery_gesture();")
        identity = startup.index("board_identity_storage_load(")
        preparation = startup.index("start_startup_preparation();")

        self.assertIn("system_io_sw2_pressed()", gesture)
        self.assertIn("system_io_sw3_pressed()", gesture)
        self.assertIn("STARTUP_RECOVERY_HOLD_MS", gesture)
        self.assertIn("feed_startup_watchdog();", gesture)
        self.assertIn("#define STARTUP_MODE_HOLD_MS UINT32_C(2000)", STARTUP)
        self.assertIn("if (!ota_confirmation_boot)", startup)
        self.assertLess(safe_gpio, pending)
        self.assertLess(pending, recovery)
        self.assertLess(recovery, identity)
        self.assertLess(identity, preparation)

    def test_sw3_only_is_the_destructive_format_gesture(self) -> None:
        power_wait = body(
            STARTUP,
            "static startup_power_on_result_t startup_power_on_confirmed(",
            "static bool startup_config_format_requested",
        )
        automatic = body(
            STARTUP,
            "static bool startup_config_format_requested",
            "void app_startup_run(void)",
        )

        for helper in (power_wait, automatic):
            self.assertIn("system_io_sw2_pressed()", helper)
            self.assertIn("system_io_sw3_pressed()", helper)
            self.assertIn("STARTUP_MODE_HOLD_MS", helper)
        self.assertIn("!system_io_sw2_pressed()", power_wait)
        self.assertIn("boot_gesture == STARTUP_BOOT_GESTURE_NONE", STARTUP)
        self.assertIn("SW3 startup request: config FAT will be formatted", STARTUP)

    def test_recovery_skips_normal_startup_services(self) -> None:
        recovery = body(
            STARTUP,
            "static void run_recovery_mode(void)",
            "static bool nvs_recovery_required",
        )

        self.assertIn("usb_device_recovery_storage_init(", recovery)
        self.assertIn("firmware_update_process_recovery(true)", recovery)
        self.assertIn("usb_device_start_recovery();", recovery)
        for forbidden in (
            "nvs_flash_init",
            "audio_output_init",
            "config_storage_load",
            "sensor_bus_init",
            "ble_vario_init",
            "app_tasks_start",
        ):
            self.assertNotIn(forbidden, recovery)

    def test_recovery_storage_does_not_generate_or_format_files(self) -> None:
        recovery_storage = body(
            USB,
            "esp_err_t usb_device_recovery_storage_init(",
            "static esp_err_t start_usb_device",
        )

        self.assertIn("initialize_storage_service();", recovery_storage)
        self.assertIn("create_msc_storage(progress_cb, progress_arg)", recovery_storage)
        for forbidden in (
            "write_info_file",
            "write_setting_editor_file",
            "config_storage_load",
            "config_storage_save",
            "esp_vfs_fat_spiflash_format_cfg_rw_wl",
            "f_setlabel",
        ):
            self.assertNotIn(forbidden, recovery_storage)

    def test_recovery_usb_uses_unique_efuse_mac_serial(self) -> None:
        serial = body(
            USB,
            "static bool make_recovery_serial_number(void)",
            "static void tinyusb_device_event",
        )

        self.assertIn("esp_read_mac(mac, ESP_MAC_WIFI_STA)", serial)
        self.assertIn('"REC-%02X%02X%02X%02X%02X%02X"', serial)
        self.assertIn("esp_err_t usb_device_start_recovery(void);", USB_HEADER)

    def test_host_release_requires_a_real_drained_host_session(self) -> None:
        events = body(
            USB,
            "static void msc_storage_event(",
            "static esp_err_t initialize_storage_service",
        )
        wait = body(
            USB,
            "esp_err_t usb_device_wait_for_host_release(",
            "void usb_device_set_storage_mode_callbacks",
        )

        self.assertIn("storage_transition_from = usb_diagnostics.storage_owner", events)
        self.assertIn("storage_transition_from == USB_STORAGE_HOST_OWNED", events)
        self.assertIn("usb_diagnostics.host_release_count++", events)
        self.assertIn("current_release_count != release_count", wait)
        self.assertIn("owner == USB_STORAGE_APP_OWNED", wait)
        self.assertIn("pending_writes == 0U", wait)

    def test_recovery_waits_for_safe_eject_and_feeds_wdt(self) -> None:
        recovery = body(
            STARTUP,
            "static void run_recovery_mode(void)",
            "static bool nvs_recovery_required",
        )
        initial_update = recovery.index("firmware_update_process_recovery(true)")
        usb_start = recovery.index("usb_device_start_recovery();")
        release_wait = recovery.index("usb_device_wait_for_host_release(")
        later_update = recovery.index(
            "firmware_update_process_recovery(true)", initial_update + 1
        )

        self.assertLess(initial_update, usb_start)
        self.assertLess(usb_start, release_wait)
        self.assertLess(release_wait, later_update)
        self.assertIn("RECOVERY_STORAGE_RELEASE_WAIT_MS UINT32_C(100)", STARTUP)
        self.assertIn("feed_startup_watchdog();", recovery)
        self.assertGreaterEqual(recovery.count("system_io_external_power_present()"), 2)

    def test_recovery_ota_reuses_boot_authentication_path(self) -> None:
        process = body(
            UPDATE,
            "static esp_err_t process_update(",
            "static void confirmation_task",
        )
        recovery_wrapper = body(
            UPDATE,
            "esp_err_t firmware_update_process_recovery(",
            "static void confirmation_task",
        )

        self.assertIn("inspect_image(UPDATE_INPUT_NAME", process)
        self.assertIn("apply_update(&image_info)", process)
        self.assertIn("report_missing_input", process)
        self.assertIn("return ESP_ERR_NOT_FOUND;", process)
        self.assertIn("return process_update(true, false, 0.0f, true);", recovery_wrapper)
        self.assertIn(
            "esp_err_t firmware_update_process_recovery(bool external_power_present);",
            UPDATE_HEADER,
        )

    def test_spec_records_recovery_boundary_and_gestures(self) -> None:
        self.assertIn("SW2+SW3", SPEC)
        self.assertIn("SW3", SPEC)
        self.assertIn("RECOVERY", SPEC)
        self.assertIn("ROM Download Mode", SPEC)


if __name__ == "__main__":
    unittest.main()
