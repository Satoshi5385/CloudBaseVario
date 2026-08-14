import json
import contextlib
import io
from pathlib import Path
import tempfile
import unittest

from manufacturing_tools.camera_qr import StableQRDetector
from manufacturing_tools.manufacturing_cli import _read_product, build_parser
from manufacturing_tools.serial_model import (
    SerialValidationError,
    load_catalog,
    parse_product_serial,
)


ROOT = Path(__file__).resolve().parents[1]


class ManufacturingSerialTests(unittest.TestCase):
    def test_valid_serial_and_catalog_match_firmware(self) -> None:
        product = parse_product_serial("CBV_A0_73I0j_0009")
        self.assertEqual(product.board_code, "A0")
        self.assertEqual(product.lot, "73I0j")
        self.assertEqual(product.sequence, 9)
        self.assertEqual(product.board.board_id, 0x0100)
        source = (ROOT / "SRC/domain/board_identity.c").read_text(encoding="utf-8")
        header = (ROOT / "SRC/domain/board_identity.h").read_text(encoding="utf-8")
        self.assertIn('.code = "A0"', source)
        self.assertIn('.model = "Aohazuku-Rev0"', source)
        self.assertIn("BOARD_ID_AOHAZUKU_REV0 UINT16_C(0x0100)", header)

    def test_rejects_noncanonical_serials(self) -> None:
        invalid = (
            "CBV_A0_73I0j_0000",
            "CBV_A0_73I0j_10000",
            "CBV_A0_73I0-_0009",
            "CBV_A0_73I0j_0009 ",
            "cbv_A0_73I0j_0009",
            "CBV_A1_73I0j_0009",
        )
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(SerialValidationError):
                parse_product_serial(value)

    def test_catalog_rejects_duplicate_codes(self) -> None:
        document = json.loads(
            (ROOT / "manufacturing_tools/board_catalog.json").read_text(
                encoding="utf-8"
            )
        )
        document["boards"].append(document["boards"][0])
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / "catalog.json"
            path.write_text(json.dumps(document), encoding="utf-8")
            with self.assertRaises(SerialValidationError):
                load_catalog(path)

    def test_stability_requires_same_single_qr_for_time_and_frames(self) -> None:
        detector = StableQRDetector(frames_required=3, duration_s=0.5)
        serial = "CBV_A0_73I0j_0009"
        self.assertIsNone(detector.update((serial,), now=1.0)[1])
        self.assertIsNone(detector.update((serial,), now=1.3)[1])
        self.assertEqual(detector.update((serial,), now=1.5)[1].value, serial)

    def test_stability_rejects_multiple_or_invalid_qr(self) -> None:
        detector = StableQRDetector()
        _, confirmed, message = detector.update(
            ("CBV_A0_73I0j_0009", "CBV_A0_73I0j_0010"), now=1.0
        )
        self.assertIsNone(confirmed)
        self.assertIn("複数", message)
        _, confirmed, message = detector.update(("https://example.com",), now=2.0)
        self.assertIsNone(confirmed)
        self.assertIn("serial", message)

    def test_cli_accepts_strict_manual_serial(self) -> None:
        args = build_parser().parse_args(
            [
                "dry-run",
                "--serial",
                "CBV_A0_73I0j_0009",
                "--build-dir",
                "build",
            ]
        )
        self.assertEqual(_read_product(args).value, "CBV_A0_73I0j_0009")

    def test_cli_manual_serial_does_not_trim_or_correct(self) -> None:
        args = build_parser().parse_args(
            [
                "dry-run",
                "--serial",
                " CBV_A0_73I0j_0009",
                "--build-dir",
                "build",
            ]
        )
        with self.assertRaises(SerialValidationError):
            _read_product(args)

    def test_cli_serial_sources_are_mutually_exclusive(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                build_parser().parse_args(
                    [
                        "dry-run",
                        "--serial",
                        "CBV_A0_73I0j_0009",
                        "--camera-index",
                        "0",
                        "--build-dir",
                        "build",
                    ]
                )

    def test_gui_manual_path_uses_unmodified_strict_input(self) -> None:
        source = (ROOT / "manufacturing_tools/manufacturing_tool.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("value = self.manual_serial.get()", source)
        self.assertIn("product = parse_product_serial(value)", source)
        self.assertNotIn("self.manual_serial.get().strip()", source)
        self.assertIn('if self.accepted_source == "QR"', source)


if __name__ == "__main__":
    unittest.main()
