import copy
import json
import unittest
from pathlib import Path

from tools.vario_sound_simulator.parameters_model import (
    default_config_document,
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
        self.assertEqual(len(parsed.mc_parameters), 11)
        for interval in (200, 1000, 10000):
            candidate = copy.deepcopy(self.raw)
            candidate["mc_parameters"]["gps_send_interval_ms"] = interval
            self.assertEqual(
                self.parse(candidate).mc_parameters["gps_send_interval_ms"],
                interval,
            )

    def test_invalid_values_use_defaults_and_unknown_keys_are_ignored(self) -> None:
        defaults = default_config_document()
        for value in (199, 10001, True):
            candidate = copy.deepcopy(self.raw)
            candidate["mc_parameters"]["gps_send_interval_ms"] = value
            self.assertEqual(self.parse(candidate), defaults)
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
            parsed = self.parse(candidate)
            self.assertEqual(parsed.mc_parameters["gps_send_interval_ms"], 1000)
            self.assertEqual(parsed.sorted_numbers(), (1, 2, 3))


if __name__ == "__main__":
    unittest.main()
