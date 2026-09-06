import copy
import json
import unittest
from pathlib import Path

from tools.vario_sound_simulator.parameters_model import (
    default_config_document,
    parse_config_document_text,
)
from tools.vario_sound_simulator.vario_sound_model import (
    AudioMode,
    VarioAudioState,
    VarioSample,
    vario_audio_step,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SETTING = ROOT / "DOC" / "default_setting.json"
APP_WORKERS = (ROOT / "SRC" / "app" / "app_workers.c").read_text(
    encoding="utf-8"
)


class StationaryAudioConfigTests(unittest.TestCase):
    def setUp(self) -> None:
        self.raw = json.loads(DEFAULT_SETTING.read_text(encoding="utf-8"))

    def test_default_is_disabled_for_every_profile(self) -> None:
        parsed = parse_config_document_text(json.dumps(self.raw))
        self.assertEqual(parsed.sorted_numbers(), (1, 2, 3))
        for number in parsed.sorted_numbers():
            self.assertFalse(
                parsed.vario_parameter_sets[number][
                    "audio_mute_when_stationary"
                ]
            )

    def test_option_is_independent_per_profile(self) -> None:
        candidate = copy.deepcopy(self.raw)
        candidate["vario_parameter_sets"][1]["parameters"][
            "audio_mute_when_stationary"
        ] = True
        parsed = parse_config_document_text(json.dumps(candidate))
        self.assertFalse(
            parsed.effective_parameters(1)["audio_mute_when_stationary"]
        )
        self.assertTrue(
            parsed.effective_parameters(2)["audio_mute_when_stationary"]
        )
        self.assertFalse(
            parsed.effective_parameters(3)["audio_mute_when_stationary"]
        )

    def test_missing_or_invalid_option_skips_only_that_profile(self) -> None:
        for invalid in (None, 1, "false"):
            candidate = copy.deepcopy(self.raw)
            profile = candidate["vario_parameter_sets"][0]["parameters"]
            if invalid is None:
                del profile["audio_mute_when_stationary"]
            else:
                profile["audio_mute_when_stationary"] = invalid
            parsed = parse_config_document_text(json.dumps(candidate))
            self.assertEqual(parsed.sorted_numbers(), (2, 3))

    def test_stationary_gate_matches_firmware_model(self) -> None:
        values = default_config_document().effective_parameters(1)
        values["audio_climb_rate_average_s"] = 0.0
        sample = VarioSample(1.0, 1.0, 1.0)

        enabled_state = VarioAudioState()
        values["audio_mute_when_stationary"] = True
        command = vario_audio_step(
            enabled_state, values, sample, 1.0, stationary=True
        )
        self.assertEqual(command.mode, AudioMode.SILENT)
        self.assertFalse(command.sounding)

        command = vario_audio_step(
            enabled_state, values, sample, 1.0, stationary=False
        )
        self.assertEqual(command.mode, AudioMode.LIFT)
        self.assertTrue(command.sounding)

        disabled_state = VarioAudioState()
        values["audio_mute_when_stationary"] = False
        command = vario_audio_step(
            disabled_state, values, sample, 1.0, stationary=True
        )
        self.assertEqual(command.mode, AudioMode.LIFT)
        self.assertTrue(command.sounding)

    def test_audio_worker_uses_published_flight_state(self) -> None:
        self.assertIn(
            "system.motion_state == FLIGHT_STATE_STATIONARY", APP_WORKERS
        )


if __name__ == "__main__":
    unittest.main()
