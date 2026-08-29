from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
USB_SOURCE = (ROOT / "SRC/platform/usb_device_service.c").read_text(
    encoding="utf-8"
)
COMPONENT_CMAKE = (ROOT / "SRC/CMakeLists.txt").read_text(encoding="utf-8")
SDKCONFIG_DEFAULTS = (ROOT / "sdkconfig.defaults").read_text(encoding="utf-8")


class InfoFilePolicyTests(unittest.TestCase):
    def test_info_file_is_generated_before_configuration_and_msc_exposure(self) -> None:
        generation = USB_SOURCE.index("ret = write_info_file(preflight_handle);")
        config_load = USB_SOURCE.index("config_storage_load(", generation)
        msc_storage = USB_SOURCE.index("tinyusb_msc_new_storage_spiflash", generation)

        self.assertLess(generation, config_load)
        self.assertLess(config_load, msc_storage)

    def test_info_file_is_synced_and_marked_read_only(self) -> None:
        start = USB_SOURCE.index("static esp_err_t write_read_only_file")
        end = USB_SOURCE.index("static esp_err_t write_info_file", start)
        writer = USB_SOURCE[start:end]

        self.assertIn("f_chmod(fat_path, 0U, AM_RDO)", writer)
        self.assertIn("fflush(file) == 0", writer)
        self.assertIn("fsync(fileno(file)) == 0", writer)
        self.assertIn("f_chmod(fat_path, AM_RDO, AM_RDO)", writer)

    def test_info_file_failure_keeps_msc_unavailable(self) -> None:
        start = USB_SOURCE.index("ret = write_info_file(preflight_handle);")
        end = USB_SOURCE.index("usb_diagnostics.load_result", start)
        failure = USB_SOURCE[start:end]

        self.assertIn("set_storage_unavailable(ret);", failure)
        self.assertIn("return ret;", failure)

    def test_info_file_reports_running_firmware_authenticity(self) -> None:
        start = USB_SOURCE.index("static esp_err_t write_info_file")
        end = USB_SOURCE.index("static bool make_serial_number", start)
        writer = USB_SOURCE[start:end]

        self.assertIn("firmware_authenticate_partition", writer)
        self.assertIn("FIRMWARE_AUTH_UNKNOWN", writer)


class SettingEditorFilePolicyTests(unittest.TestCase):
    def test_editor_is_embedded_from_canonical_doc_file(self) -> None:
        self.assertIn(
            '"${CMAKE_SOURCE_DIR}/DOC/setting_editor.html"', COMPONENT_CMAKE
        )
        self.assertIn('RENAME_TO "setting_editor_html"', COMPONENT_CMAKE)
        self.assertIn("_binary_setting_editor_html_start", USB_SOURCE)
        self.assertIn("_binary_setting_editor_html_end", USB_SOURCE)

    def test_editor_is_generated_before_configuration_and_msc_exposure(self) -> None:
        generation = USB_SOURCE.index(
            "ret = write_setting_editor_file(preflight_handle);"
        )
        config_load = USB_SOURCE.index("config_storage_load(", generation)
        msc_storage = USB_SOURCE.index(
            "tinyusb_msc_new_storage_spiflash", generation
        )

        self.assertLess(generation, config_load)
        self.assertLess(config_load, msc_storage)

    def test_editor_uses_shared_read_only_synced_writer(self) -> None:
        start = USB_SOURCE.index("static esp_err_t write_setting_editor_file")
        end = USB_SOURCE.index("static bool make_serial_number", start)
        writer = USB_SOURCE[start:end]

        self.assertIn("SETTING_EDITOR_FILENAME", writer)
        self.assertIn("write_read_only_file", writer)

    def test_editor_failure_keeps_msc_unavailable(self) -> None:
        start = USB_SOURCE.index(
            "ret = write_setting_editor_file(preflight_handle);"
        )
        end = USB_SOURCE.index("usb_diagnostics.load_result", start)
        failure = USB_SOURCE[start:end]

        self.assertIn("set_storage_unavailable(ret);", failure)
        self.assertIn("return ret;", failure)

    def test_long_filename_support_is_enabled(self) -> None:
        self.assertIn("CONFIG_FATFS_LFN_HEAP=y", SDKCONFIG_DEFAULTS)
        self.assertIn("CONFIG_FATFS_MAX_LFN=255", SDKCONFIG_DEFAULTS)


if __name__ == "__main__":
    unittest.main()
