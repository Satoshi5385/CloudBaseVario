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
README = (ROOT / "README.md").read_text(encoding="utf-8")
DEVELOPER_README = (ROOT / "README2.md").read_text(encoding="utf-8")
SETTING_GUIDE = (ROOT / "DOC/setting_json.md").read_text(encoding="utf-8")


def body(source: str, start_text: str, end_text: str) -> str:
    start = source.index(start_text)
    return source[start : source.index(end_text, start)]


class RecoveryModePolicyTests(unittest.TestCase):
    def test_recovery_gesture_is_early_bounded_and_manual_recovery_wins(self) -> None:
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
        self.assertNotIn("if (!ota_confirmation_boot)", startup)
        self.assertLess(safe_gpio, pending)
        self.assertLess(pending, recovery)
        self.assertLess(recovery, identity)
        self.assertLess(identity, preparation)

    def test_sw3_only_cannot_format_runtime_storage(self) -> None:
        power_wait = body(
            STARTUP,
            "static startup_power_on_result_t startup_power_on_confirmed(",
            "void app_startup_run(void)",
        )
        normal_storage = body(
            USB,
            "esp_err_t usb_device_storage_init(",
            "esp_err_t usb_device_recovery_storage_init(",
        )

        self.assertNotIn("system_io_sw3_pressed()", power_wait)
        self.assertNotIn("config_format_requested", STARTUP)
        self.assertNotIn("esp_vfs_fat_spiflash_format_cfg_rw_wl", normal_storage)
        self.assertNotIn("format_if_mount_failed = true", normal_storage)
        self.assertNotIn("switch_preferences_clear()", STARTUP)

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

    def test_recovery_storage_uses_only_zeroed_psram_for_the_lun(self) -> None:
        recovery_storage = body(
            USB,
            "esp_err_t usb_device_recovery_storage_init(",
            "static esp_err_t start_usb_device",
        )

        self.assertIn("initialize_storage_service();", recovery_storage)
        self.assertIn("tinyusb_msc_new_storage_psram", recovery_storage)
        self.assertIn("heap_caps_calloc(", recovery_storage)
        self.assertIn("MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT", recovery_storage)
        self.assertIn("RECOVERY_DISK_BYTES", recovery_storage)
        self.assertIn("RECOVERY_SECTOR_BYTES", recovery_storage)
        self.assertIn("seed_recovery_volume();", recovery_storage)
        self.assertIn("esp_vfs_fat_spiflash_mount_rw_wl(", recovery_storage)
        self.assertNotIn("create_msc_storage(", recovery_storage)
        self.assertNotIn("tinyusb_msc_new_storage_spiflash", recovery_storage)
        for forbidden in (
            "write_info_file",
            "write_setting_editor_file",
            "config_storage_load",
            "config_storage_save",
            "esp_vfs_fat_spiflash_format_cfg_rw_wl",
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
        usb_start = recovery.index("usb_device_start_recovery();")
        release_wait = recovery.index("usb_device_wait_for_host_release(")
        update = recovery.index("firmware_update_process_recovery(true)")

        self.assertLess(usb_start, release_wait)
        self.assertLess(release_wait, update)
        self.assertEqual(1, recovery.count("firmware_update_process_recovery(true)"))
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

        self.assertIn("inspect_input_image(UPDATE_INPUT_NAME", process)
        self.assertIn("apply_update(&image_info)", process)
        self.assertIn("report_missing_input", process)
        self.assertIn("return ESP_ERR_NOT_FOUND;", process)
        self.assertIn("usb_device_recovery_storage_ready()", recovery_wrapper)
        self.assertIn(
            "return process_update(&psram_source_context, true, false, 0.0f, true);",
            recovery_wrapper,
        )
        self.assertIn(
            "esp_err_t firmware_update_process_recovery(bool external_power_present);",
            UPDATE_HEADER,
        )

    def test_spec_records_recovery_boundary_and_gestures(self) -> None:
        self.assertIn("SW2+SW3", SPEC)
        self.assertIn("SW3", SPEC)
        self.assertIn("RECOVERY", SPEC)
        self.assertIn("ROM Download Mode", SPEC)
        self.assertIn("CBVUPDATE", SPEC)
        self.assertIn("PSRAM-backed", SPEC)

    def test_user_documents_do_not_offer_switch_formatting(self) -> None:
        documents = (README, DEVELOPER_README, SETTING_GUIDE)

        for document in documents:
            self.assertIn("CBVUPDATE", document)
            self.assertIn("PSRAM-backed", document)
            self.assertNotIn("SW2とSW3による起動時初期化", document)
            self.assertNotIn(
                "SW2とSW3を同時に押したまま電源ONすると設定FATをformat",
                document,
            )
        self.assertIn("SW3単独の起動操作では設定用ドライブを初期化しません", README)
        self.assertIn("SW3単独にも設定FATの初期化機能はありません", DEVELOPER_README)
        self.assertIn("SW3単独に初期化機能はありません", SETTING_GUIDE)


if __name__ == "__main__":
    unittest.main()
