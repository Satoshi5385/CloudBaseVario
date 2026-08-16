from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
USB_SOURCE = (ROOT / "SRC/platform/usb_device_service.c").read_text(
    encoding="utf-8"
)


class InfoFilePolicyTests(unittest.TestCase):
    def test_info_file_is_generated_before_configuration_and_msc_exposure(self) -> None:
        generation = USB_SOURCE.index("ret = write_info_file(preflight_handle);")
        config_load = USB_SOURCE.index("config_storage_load(", generation)
        msc_storage = USB_SOURCE.index("tinyusb_msc_new_storage_spiflash", generation)

        self.assertLess(generation, config_load)
        self.assertLess(config_load, msc_storage)

    def test_info_file_is_synced_and_marked_read_only(self) -> None:
        start = USB_SOURCE.index("static esp_err_t write_info_file")
        end = USB_SOURCE.index("static bool make_serial_number", start)
        writer = USB_SOURCE[start:end]

        self.assertIn("INFO_FILENAME", writer)
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


if __name__ == "__main__":
    unittest.main()
