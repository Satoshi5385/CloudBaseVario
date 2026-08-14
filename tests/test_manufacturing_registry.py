from pathlib import Path
import tempfile
import unittest

from manufacturing_tools.registry import ManufacturingRegistry, RegistryError
from manufacturing_tools.serial_model import parse_product_serial


FIRMWARE = {
    "project": "CloudBaseVario-Aohazuku",
    "version": "test",
    "application_sha256": "a" * 64,
    "board_data_sha256": "b" * 64,
}


class ManufacturingRegistryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.registry = ManufacturingRegistry(Path(self.temp.name))
        self.product = parse_product_serial("CBV_A0_73I0j_0009")

    def tearDown(self) -> None:
        self.temp.cleanup()

    def begin(self, mac="001122334455", **kwargs):
        return self.registry.begin_attempt(
            self.product, mac, "operator", "COM5", 0, FIRMWARE, **kwargs
        )

    def test_register_finish_and_explicit_rework(self) -> None:
        attempt = self.begin()
        self.registry.finish_attempt(attempt, success=True, stage="PASS")
        self.assertEqual(self.registry.list_devices()[0]["status"], "PASS")
        with self.assertRaises(RegistryError):
            self.begin()
        retry = self.begin(rework=True)
        self.registry.finish_attempt(retry, success=False, stage="WAIT_CDC", error="timeout")
        self.assertEqual(self.registry.list_devices()[0]["status"], "FAIL")

    def test_rebind_requires_reason_and_keeps_attempt_history(self) -> None:
        attempt = self.begin()
        self.registry.finish_attempt(attempt, success=True, stage="PASS")
        with self.assertRaises(RegistryError):
            self.begin(mac="AABBCCDDEEFF", rework=True)
        rebound = self.begin(
            mac="AABBCCDDEEFF", rework=True, rebind_reason="module replacement"
        )
        attempts = self.registry.list_attempts(self.product.value)
        self.assertEqual(attempts[-1]["rebind_reason"], "module replacement")
        self.registry.finish_attempt(rebound, success=True, stage="PASS")

    def test_mac_cannot_be_bound_to_another_serial(self) -> None:
        attempt = self.begin()
        self.registry.finish_attempt(attempt, success=True, stage="PASS")
        other = parse_product_serial("CBV_A0_73I0j_0010")
        with self.assertRaises(RegistryError):
            self.registry.begin_attempt(
                other, "001122334455", "operator", "COM5", 0, FIRMWARE
            )

    def test_interrupted_attempt_is_recovered(self) -> None:
        self.begin()
        self.assertEqual(self.registry.recover_interrupted(), 1)
        self.assertEqual(self.registry.list_devices()[0]["status"], "FAIL")
        self.assertEqual(self.registry.list_attempts()[0]["result"], "INTERRUPTED")


if __name__ == "__main__":
    unittest.main()

