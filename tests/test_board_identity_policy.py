import csv
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
STARTUP = (ROOT / "SRC/app/startup.c").read_text(encoding="utf-8")
PARTITIONS = ROOT / "partitions.csv"
USB = (ROOT / "SRC/platform/usb_device_service.c").read_text(encoding="utf-8")
IDENTITY_STORAGE = (ROOT / "SRC/platform/board_identity_storage.c").read_text(
    encoding="utf-8"
)


class BoardIdentityPolicyTests(unittest.TestCase):
    def test_recovery_gate_precedes_identity_and_normal_services(self) -> None:
        load = STARTUP.index("board_identity_storage_load(")
        select = STARTUP.index("board_select_identity(&board_identity)")
        safe_gpio = STARTUP.index("ret = board_init_safe_gpio();")
        recovery = STARTUP.index("boot_gesture = startup_recovery_gesture();")
        preparation = STARTUP.index("start_startup_preparation();")
        self.assertLess(safe_gpio, recovery)
        self.assertLess(recovery, load)
        self.assertLess(load, select)
        self.assertLess(select, preparation)

    def test_board_data_is_dedicated_readonly_partition(self) -> None:
        with PARTITIONS.open(encoding="utf-8", newline="") as stream:
            rows = [row for row in csv.reader(stream) if row and not row[0].startswith("#")]
        board_data = next(row for row in rows if row[0].strip() == "board_data")
        self.assertEqual(board_data[3].strip(), "0xf20000")
        self.assertEqual(board_data[4].strip(), "0x1000")
        self.assertEqual(board_data[5].strip(), "readonly")

    def test_usb_serial_comes_from_validated_board_identity(self) -> None:
        normal_start = USB[
            USB.index("static bool make_serial_number(void)") :
            USB.index("static bool make_recovery_serial_number(void)")
        ]
        self.assertIn("board_active_identity()", normal_start)
        self.assertIn("board_identity_validate(identity)", normal_start)
        self.assertNotIn("esp_read_mac", normal_start)

    def test_schema_one_requires_the_fourth_gps_key(self) -> None:
        self.assertIn('#define BOARD_DATA_KEY_GPS_INSTALLED "gps_inst"', IDENTITY_STORAGE)
        self.assertIn("#define BOARD_DATA_EXPECTED_KEY_COUNT 4U", IDENTITY_STORAGE)
        missing_key = IDENTITY_STORAGE.index(
            "if (result == ESP_ERR_NVS_NOT_FOUND)",
            IDENTITY_STORAGE.index("nvs_close(handle);"),
        )
        invalid_return = IDENTITY_STORAGE.index(
            "return BOARD_IDENTITY_LOAD_INVALID;", missing_key
        )
        self.assertLess(missing_key, invalid_return)


if __name__ == "__main__":
    unittest.main()
