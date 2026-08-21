import copy
import json
import unittest
from pathlib import Path

from tools.vario_sound_simulator.parameters_model import (
    ConfigError,
    config_document_json_text,
    parse_config_document_text,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SETTING = ROOT / "DOC" / "default_setting.json"


class BluetoothTxPowerConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.document_text = DEFAULT_SETTING.read_text(encoding="utf-8")
        self.raw_document = json.loads(self.document_text)

    def test_default_is_low_and_all_presets_round_trip(self) -> None:
        document = parse_config_document_text(self.document_text)
        self.assertEqual(document.mc_parameters["bluetooth_tx_power"], "LOW")
        self.assertEqual(len(document.mc_parameters), 10)

        for preset in ("MIN", "LOW", "NORMAL", "HIGH"):
            candidate = copy.deepcopy(self.raw_document)
            candidate["mc_parameters"]["bluetooth_tx_power"] = preset
            parsed = parse_config_document_text(json.dumps(candidate))
            rendered = config_document_json_text(parsed)
            reparsed = parse_config_document_text(rendered)
            self.assertEqual(
                reparsed.mc_parameters["bluetooth_tx_power"], preset
            )

    def test_missing_invalid_and_numeric_values_are_rejected(self) -> None:
        missing = copy.deepcopy(self.raw_document)
        del missing["mc_parameters"]["bluetooth_tx_power"]
        with self.assertRaises(ConfigError):
            parse_config_document_text(json.dumps(missing))

        for invalid in ("MAX", "INVALID", 20):
            candidate = copy.deepcopy(self.raw_document)
            candidate["mc_parameters"]["bluetooth_tx_power"] = invalid
            with self.assertRaises(ConfigError):
                parse_config_document_text(json.dumps(candidate))


if __name__ == "__main__":
    unittest.main()
