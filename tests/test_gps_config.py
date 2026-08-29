import copy
import json
import unittest
from pathlib import Path

from tools.vario_sound_simulator.parameters_model import (
    ConfigError,
    parse_config_document_text,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SETTING = ROOT / "DOC" / "default_setting.json"


class GpsConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.raw = json.loads(DEFAULT_SETTING.read_text(encoding="utf-8"))

    def parse(self, document: dict) -> object:
        return parse_config_document_text(json.dumps(document))

    def test_interval_default_and_boundaries(self) -> None:
        parsed = self.parse(self.raw)
        self.assertEqual(parsed.mc_parameters["gps_send_interval_ms"], 1000)
        self.assertEqual(len(parsed.mc_parameters), 8)
        for interval in (200, 1000, 10000):
            candidate = copy.deepcopy(self.raw)
            candidate["mc_parameters"]["gps_send_interval_ms"] = interval
            self.assertEqual(
                self.parse(candidate).mc_parameters["gps_send_interval_ms"],
                interval,
            )

    def test_invalid_or_presence_keys_are_rejected(self) -> None:
        for value in (199, 10001, True):
            candidate = copy.deepcopy(self.raw)
            candidate["mc_parameters"]["gps_send_interval_ms"] = value
            with self.assertRaises(ConfigError):
                self.parse(candidate)
        for name in (
            "gps_module_installed",
            "gps_installed",
            "gps_inst",
            "i2c_reinit_error_count",
            "imu_mahony_kp",
            "imu_mahony_ki",
        ):
            candidate = copy.deepcopy(self.raw)
            candidate["mc_parameters"][name] = True
            with self.assertRaises(ConfigError):
                self.parse(candidate)


if __name__ == "__main__":
    unittest.main()
