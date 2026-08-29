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


class SettingJsonRobustnessTests(unittest.TestCase):
    def setUp(self) -> None:
        self.raw = json.loads(DEFAULT_SETTING.read_text(encoding="utf-8"))
        self.defaults = default_config_document()

    def parse(self, document: object):
        return parse_config_document_text(json.dumps(document))

    def test_unknown_items_are_ignored_at_every_level(self) -> None:
        candidate = copy.deepcopy(self.raw)
        candidate["future_top"] = {"enabled": True}
        candidate["mc_parameters"]["future_shared"] = 123
        candidate["mc_parameters"]["sea_level_pressure_pa"] = 100000.0
        candidate["vario_parameter_sets"][0]["future_set"] = "ignored"
        candidate["vario_parameter_sets"][0]["parameters"][
            "future_profile"
        ] = 456
        candidate["vario_parameter_sets"][0]["parameters"][
            "lift_start_mps"
        ] = 0.15

        parsed = self.parse(candidate)

        self.assertEqual(parsed.mc_parameters["sea_level_pressure_pa"], 100000.0)
        self.assertEqual(parsed.vario_parameter_sets[1]["lift_start_mps"], 0.15)

    def test_missing_required_top_or_shared_item_uses_defaults(self) -> None:
        for path in ("format_version", "mc_parameters", "vario_parameter_sets"):
            candidate = copy.deepcopy(self.raw)
            del candidate[path]
            self.assertEqual(self.parse(candidate), self.defaults)

        candidate = copy.deepcopy(self.raw)
        del candidate["mc_parameters"]["sea_level_pressure_pa"]
        self.assertEqual(self.parse(candidate), self.defaults)

    def test_only_complete_valid_parameter_sets_are_loaded(self) -> None:
        candidate = copy.deepcopy(self.raw)
        valid_one = candidate["vario_parameter_sets"][0]
        incomplete = candidate["vario_parameter_sets"][1]
        invalid_relation = candidate["vario_parameter_sets"][2]
        del incomplete["parameters"]["lift_start_mps"]
        invalid_relation["parameters"]["lift_end_mps"] = 1.0
        valid_four = copy.deepcopy(valid_one)
        valid_four["parameter_number"] = 4
        valid_four["parameters"]["lift_start_mps"] = 0.4
        valid_four["parameters"]["lift_end_mps"] = 0.35
        candidate["vario_parameter_sets"].extend(
            [None, {"parameter_number": 9, "parameters": {}}, valid_four]
        )

        parsed = self.parse(candidate)

        self.assertEqual(parsed.sorted_numbers(), (1, 4))
        self.assertEqual(
            parsed.vario_parameter_sets[4]["lift_start_mps"], 0.4
        )

    def test_duplicate_numbers_are_all_skipped(self) -> None:
        candidate = copy.deepcopy(self.raw)
        duplicate = copy.deepcopy(candidate["vario_parameter_sets"][0])
        candidate["vario_parameter_sets"].append(duplicate)

        parsed = self.parse(candidate)

        self.assertEqual(parsed.sorted_numbers(), (2, 3))

    def test_no_valid_set_and_malformed_json_use_defaults(self) -> None:
        candidate = copy.deepcopy(self.raw)
        candidate["mc_parameters"]["sea_level_pressure_pa"] = 100000.0
        for profile in candidate["vario_parameter_sets"]:
            del profile["parameters"]["lift_start_mps"]

        self.assertEqual(self.parse(candidate), self.defaults)
        self.assertEqual(parse_config_document_text("{"), self.defaults)

    def test_duplicate_known_keys_use_matching_fallback_scope(self) -> None:
        source = DEFAULT_SETTING.read_text(encoding="utf-8")
        duplicate_shared = source.replace(
            '"sea_level_pressure_pa": 101325.0,',
            '"sea_level_pressure_pa": 100000.0,\n'
            '    "sea_level_pressure_pa": 101325.0,',
            1,
        )
        self.assertEqual(
            parse_config_document_text(duplicate_shared), self.defaults
        )

        duplicate_profile = source.replace(
            '"parameter_number": 1,',
            '"parameter_number": 1,\n      "parameter_number": 1,',
            1,
        )
        parsed = parse_config_document_text(duplicate_profile)
        self.assertEqual(parsed.sorted_numbers(), (2, 3))


if __name__ == "__main__":
    unittest.main()
