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
SETTING_EDITOR = (ROOT / "DOC" / "setting_editor.html").read_text(
    encoding="utf-8"
)
APP_WORKERS = (ROOT / "SRC" / "app" / "app_workers.c").read_text(
    encoding="utf-8"
)


class FlightStateConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.raw = json.loads(DEFAULT_SETTING.read_text(encoding="utf-8"))

    def parse(self, document: dict):
        return parse_config_document_text(json.dumps(document))

    def test_defaults_and_boundaries(self) -> None:
        parsed = self.parse(self.raw)
        self.assertEqual(
            parsed.mc_parameters["flight_climb_rate_threshold_mps"], 0.5
        )
        self.assertEqual(
            parsed.mc_parameters["flight_gps_speed_threshold_kmh"], 10.0
        )
        self.assertEqual(parsed.mc_parameters["stationary_confirm_seconds"], 60)

        boundaries = {
            "flight_climb_rate_threshold_mps": (0.1, 5.0),
            "flight_gps_speed_threshold_kmh": (1.0, 100.0),
            "stationary_confirm_seconds": (10, 600),
        }
        for name, values in boundaries.items():
            for value in values:
                candidate = copy.deepcopy(self.raw)
                candidate["mc_parameters"][name] = value
                self.assertEqual(self.parse(candidate).mc_parameters[name], value)

    def test_missing_or_invalid_required_value_uses_defaults(self) -> None:
        defaults = default_config_document()
        for name, invalid in (
            ("flight_climb_rate_threshold_mps", 0.09),
            ("flight_gps_speed_threshold_kmh", 100.1),
            ("stationary_confirm_seconds", 9),
        ):
            missing = copy.deepcopy(self.raw)
            del missing["mc_parameters"][name]
            self.assertEqual(self.parse(missing), defaults)
            candidate = copy.deepcopy(self.raw)
            candidate["mc_parameters"][name] = invalid
            self.assertEqual(self.parse(candidate), defaults)

    def test_format_version_remains_one(self) -> None:
        self.assertEqual(self.raw["format_version"], 1)

    def test_editor_and_param_console_share_the_contract(self) -> None:
        for name in (
            "flight_climb_rate_threshold_mps",
            "flight_gps_speed_threshold_kmh",
            "stationary_confirm_seconds",
        ):
            self.assertIn(name, SETTING_EDITOR)
        for command in (
            'strcasecmp(tokens[1], "GET")',
            'strcasecmp(tokens[1], "SET")',
            'strcasecmp(tokens[1], "SAVE")',
        ):
            self.assertIn(command, APP_WORKERS)


if __name__ == "__main__":
    unittest.main()
