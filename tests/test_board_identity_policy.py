import csv
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
STARTUP = (ROOT / "SRC/app/startup.c").read_text(encoding="utf-8")
PARTITIONS = ROOT / "partitions.csv"
USB = (ROOT / "SRC/platform/usb_device_service.c").read_text(encoding="utf-8")


class BoardIdentityPolicyTests(unittest.TestCase):
    def test_identity_precedes_every_board_specific_initialization(self) -> None:
        load = STARTUP.index("board_identity_storage_load(")
        select = STARTUP.index("board_select_identity(&board_identity)")
        safe_gpio = STARTUP.index("ret = board_init_safe_gpio();")
        preparation = STARTUP.index("start_startup_preparation();")
        self.assertLess(load, select)
        self.assertLess(select, safe_gpio)
        self.assertLess(safe_gpio, preparation)
        self.assertNotIn("BOARD_IDENTITY_LOAD_MISSING", STARTUP[safe_gpio:])

    def test_board_data_is_dedicated_readonly_partition(self) -> None:
        with PARTITIONS.open(encoding="utf-8", newline="") as stream:
            rows = [row for row in csv.reader(stream) if row and not row[0].startswith("#")]
        board_data = next(row for row in rows if row[0].strip() == "board_data")
        self.assertEqual(board_data[3].strip(), "0xf20000")
        self.assertEqual(board_data[4].strip(), "0x1000")
        self.assertEqual(board_data[5].strip(), "readonly")

    def test_usb_serial_comes_from_validated_board_identity(self) -> None:
        self.assertIn("board_active_identity()", USB)
        self.assertIn("board_identity_validate(identity)", USB)
        self.assertNotIn("ESP_MAC_WIFI_STA", USB)


if __name__ == "__main__":
    unittest.main()

