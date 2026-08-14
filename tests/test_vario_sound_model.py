import json
import math
from pathlib import Path
import re
import tempfile
import unittest
from unittest import mock

from tools.vario_sound_model import (
    AUDIO_PARAMETER_NAMES,
    PARAMETER_SPECS,
    PROFILE_PARAMETER_SPECS,
    RUNTIME_CONTROL_SPECS,
    SHARED_PARAMETER_SPECS,
    AudioMode,
    ConfigDocument,
    ConfigError,
    PwmWaveform,
    VarioAudioState,
    VarioSample,
    config_document_json_text,
    config_json_text,
    default_parameters,
    default_config_document,
    lift_frequency_hz,
    lift_phase_time_ms,
    load_config_document_file,
    load_config_file,
    parse_config_document_text,
    parse_config_text,
    save_config_document_file,
    save_config_file,
    sink_frequency_hz,
    vario_audio_step,
)
from tools.vario_sound_simulator import CLIMB_RATE_MAX_MPS, CLIMB_RATE_MIN_MPS


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


class ParameterContractTests(unittest.TestCase):
    def test_simulator_climb_rate_input_range_is_symmetric_15_mps(self) -> None:
        self.assertEqual(CLIMB_RATE_MIN_MPS, -15.0)
        self.assertEqual(CLIMB_RATE_MAX_MPS, 15.0)

    def test_python_schema_matches_current_firmware_table(self) -> None:
        source = (REPOSITORY_ROOT / "SRC/domain/app_config.c").read_text(
            encoding="utf-8"
        )
        table = source.split("parameter_table[] = {", 1)[1].split("};", 1)[0]
        entries = re.findall(r"PARAM_(BOOL|UINT|FLOAT|ENUM)\(([^)]*)\)", table)
        self.assertEqual(len(entries), 31)
        self.assertEqual(len(PARAMETER_SPECS), 31)
        self.assertEqual(len(SHARED_PARAMETER_SPECS), 9)
        self.assertEqual(len(PROFILE_PARAMETER_SPECS), 22)
        self.assertEqual(len(AUDIO_PARAMETER_NAMES), 25)

        source_names = []
        kind_map = {"BOOL": "bool", "UINT": "uint", "FLOAT": "float", "ENUM": "enum"}
        for macro_kind, arguments_text in entries:
            arguments = [item.strip() for item in arguments_text.split(",")]
            name = arguments[0]
            source_names.append(name)
            spec = PARAMETER_SPECS[name]
            expected_scope = (
                "APP_PARAMETER_SCOPE_PROFILE"
                if spec.audio
                else "APP_PARAMETER_SCOPE_SHARED"
            )
            self.assertEqual(arguments[-1], expected_scope, name)
            self.assertEqual(spec.kind, kind_map[macro_kind], name)
            if macro_kind == "BOOL":
                self.assertEqual(spec.default, arguments[1] == "true", name)
            elif macro_kind == "ENUM":
                expected_default = {
                    "APP_FILTER_MODE_AUTO": "AUTO",
                    "APP_BLUETOOTH_BATTERY_MODE_VOLTAGE": "VOLTAGE",
                }[arguments[1]]
                self.assertEqual(spec.default, expected_default, name)
            else:
                default_text = arguments[1].removesuffix("f")
                expected_default = (
                    int(default_text) if macro_kind == "UINT" else float(default_text)
                )
                self.assertEqual(spec.default, expected_default, name)
                self.assertEqual(spec.minimum, float(arguments[2]), name)
                self.assertEqual(spec.maximum, float(arguments[3]), name)
        self.assertEqual(source_names, list(PARAMETER_SPECS))

    def test_new_document_contains_complete_version_1_configuration(self) -> None:
        root = json.loads(config_document_json_text(default_config_document()))
        self.assertEqual(root["format_version"], 1)
        self.assertEqual(len(root["mc_parameters"]), 9)
        self.assertEqual(root["mc_parameters"]["auto_power_off_minutes"], 60)
        self.assertEqual(root["mc_parameters"]["bluetooth_battery_mode"], "VOLTAGE")
        self.assertEqual(root["mc_parameters"]["bluetooth_notify_rate_hz"], 10)
        self.assertEqual(list(root["mc_parameters"]), list(SHARED_PARAMETER_SPECS))
        self.assertEqual(len(root["vario_parameter_sets"]), 3)
        profile = root["vario_parameter_sets"][0]
        self.assertEqual(profile["parameter_number"], 1)
        self.assertEqual(len(profile["parameters"]), 22)
        self.assertEqual(
            list(profile["parameters"]), list(PROFILE_PARAMETER_SPECS)
        )

    def test_documented_default_json_matches_firmware_model(self) -> None:
        document = (REPOSITORY_ROOT / "DOC/setting_json.md").read_text(
            encoding="utf-8"
        )
        example = document.split("```json", 1)[1].split("```", 1)[0]
        self.assertEqual(
            json.loads(example),
            json.loads(config_document_json_text(default_config_document())),
        )

    def test_default_setting_and_sw_spec_excerpt_use_version_1_contract(self) -> None:
        default_setting_path = REPOSITORY_ROOT / "DOC/default_setting.json"
        default_setting = load_config_document_file(default_setting_path)
        self.assertEqual(
            default_setting.mc_parameters["bluetooth_battery_mode"], "VOLTAGE"
        )
        self.assertEqual(
            default_setting.mc_parameters["bluetooth_notify_rate_hz"], 10
        )

        sw_spec = (REPOSITORY_ROOT / "DOC/SW_spec.md").read_text(
            encoding="utf-8"
        )
        file_format_section = sw_spec.split("#### ファイル形式", 1)[1]
        excerpt = file_format_section.split("```json", 1)[1].split("```", 1)[0]
        excerpt_root = json.loads(excerpt)
        self.assertEqual(excerpt_root["format_version"], 1)
        self.assertEqual(
            excerpt_root["mc_parameters"]["bluetooth_battery_mode"], "VOLTAGE"
        )
        self.assertEqual(
            excerpt_root["mc_parameters"]["bluetooth_notify_rate_hz"], 10
        )

    def test_accepts_integral_json_numbers_for_firmware_uint_fields(self) -> None:
        root = json.loads(config_json_text(default_parameters()))
        root["format_version"] = 1.0
        root["vario_parameter_sets"][0]["parameters"]["audio_state_hold_ms"] = 60.0
        values = parse_config_text(json.dumps(root))
        self.assertEqual(values["audio_state_hold_ms"], 60)
        self.assertIs(type(values["audio_state_hold_ms"]), int)

    def test_rejects_duplicate_unknown_range_and_relationship(self) -> None:
        rendered = config_json_text(default_parameters())
        with self.assertRaisesRegex(ConfigError, "duplicate key"):
            parse_config_text(rendered.replace('"format_version": 1', '"format_version": 1, "format_version": 1'))
        root = json.loads(rendered)
        parameters = root["vario_parameter_sets"][0]["parameters"]
        parameters["not_a_parameter"] = 1
        with self.assertRaisesRegex(ConfigError, "unknown parameter"):
            parse_config_text(json.dumps(root))
        parameters.pop("not_a_parameter")
        parameters["audio_climb_rate_average_s"] = 10.1
        with self.assertRaisesRegex(ConfigError, "between 0 and 10"):
            parse_config_text(json.dumps(root))
        parameters["audio_climb_rate_average_s"] = 5.0
        root["mc_parameters"]["bluetooth_notify_rate_hz"] = 0
        with self.assertRaisesRegex(ConfigError, "between 1 and 50"):
            parse_config_text(json.dumps(root))
        root["mc_parameters"]["bluetooth_notify_rate_hz"] = 51
        with self.assertRaisesRegex(ConfigError, "between 1 and 50"):
            parse_config_text(json.dumps(root))
        root["mc_parameters"]["bluetooth_notify_rate_hz"] = 10
        parameters["sink_start_mps"] = -0.5
        parameters["sink_end_mps"] = -1.0
        with self.assertRaisesRegex(ConfigError, "sink_start_mps"):
            parse_config_text(json.dumps(root))

    def test_rejects_missing_and_misplaced_parameters(self) -> None:
        root = json.loads(config_json_text(default_parameters()))
        root["mc_parameters"].pop("filter_mode")
        with self.assertRaisesRegex(ConfigError, "missing parameter: filter_mode"):
            parse_config_document_text(json.dumps(root))

        root = json.loads(config_json_text(default_parameters()))
        root["mc_parameters"].pop("bluetooth_battery_mode")
        with self.assertRaisesRegex(
            ConfigError, "missing parameter: bluetooth_battery_mode"
        ):
            parse_config_document_text(json.dumps(root))

        root = json.loads(config_json_text(default_parameters()))
        root["mc_parameters"]["bluetooth_battery_mode"] = "PERCENTAGE"
        with self.assertRaisesRegex(
            ConfigError, "expected VOLTAGE or PERCENT"
        ):
            parse_config_document_text(json.dumps(root))

        root = json.loads(config_json_text(default_parameters()))
        root["mc_parameters"]["lift_start_mps"] = root["vario_parameter_sets"][0][
            "parameters"
        ].pop("lift_start_mps")
        with self.assertRaisesRegex(ConfigError, "unknown parameter: lift_start_mps"):
            parse_config_document_text(json.dumps(root))

        root = json.loads(config_json_text(default_parameters()))
        root["vario_parameter_sets"][0]["parameters"]["filter_mode"] = root[
            "mc_parameters"
        ]["filter_mode"]
        with self.assertRaisesRegex(ConfigError, "unknown parameter: filter_mode"):
            parse_config_document_text(json.dumps(root))

    def test_rejects_full_configuration_per_set_old_draft(self) -> None:
        root = json.loads(config_json_text(default_parameters()))
        root.pop("mc_parameters")
        root["vario_parameter_sets"][0]["parameters"] = {
            name: default_parameters()[name] for name in PARAMETER_SPECS
        }
        with self.assertRaisesRegex(ConfigError, "missing top-level key: mc_parameters"):
            parse_config_document_text(json.dumps(root))

    def test_runtime_switch_keys_are_never_saved(self) -> None:
        values = default_parameters()
        values["audio_enabled"] = False
        values["audio_amp_mode"] = 3
        values["sink_enabled"] = False
        rendered = config_json_text(values)
        root = json.loads(rendered)
        for name in RUNTIME_CONTROL_SPECS:
            self.assertNotIn(name, root["vario_parameter_sets"][0]["parameters"])

    def test_versions_2_through_7_are_rejected(self) -> None:
        for version in (2, 3, 4, 5, 6, 7):
            with self.subTest(version=version):
                with self.assertRaisesRegex(ConfigError, "unsupported format_version"):
                    parse_config_document_text(
                        json.dumps({"format_version": version, "vario_parameter_sets": []})
                    )

    def test_old_key_names_are_rejected_without_compatibility(self) -> None:
        root = json.loads(config_json_text(default_parameters()))
        root["parameters"] = root.pop("mc_parameters")
        with self.assertRaisesRegex(ConfigError, "unknown top-level key: parameters"):
            parse_config_document_text(json.dumps(root))

        root = json.loads(config_json_text(default_parameters()))
        root["parameter_sets"] = root.pop("vario_parameter_sets")
        with self.assertRaisesRegex(ConfigError, "unknown top-level key: parameter_sets"):
            parse_config_document_text(json.dumps(root))

        root = json.loads(config_json_text(default_parameters()))
        profile = root["vario_parameter_sets"][0]
        profile["mc_parameters"] = profile.pop("parameters")
        with self.assertRaisesRegex(ConfigError, "unknown profile key: mc_parameters"):
            parse_config_document_text(json.dumps(root))

        for old_name, new_name in (
            ("lk8ex1_battery_mode", "bluetooth_battery_mode"),
            ("lk8ex1_notify_rate_hz", "bluetooth_notify_rate_hz"),
        ):
            with self.subTest(old_name=old_name):
                root = json.loads(config_json_text(default_parameters()))
                root["mc_parameters"][old_name] = root["mc_parameters"].pop(new_name)
                with self.assertRaisesRegex(
                    ConfigError, f"unknown parameter: {old_name}"
                ):
                    parse_config_document_text(json.dumps(root))

    def test_overwrite_preserves_non_audio_values(self) -> None:
        values = default_parameters()
        values["sea_level_pressure_pa"] = 100123.5
        values["auto_power_off_minutes"] = 120
        values["filter_mode"] = "BARO_ONLY"
        values["bluetooth_battery_mode"] = "PERCENT"
        values["bluetooth_notify_rate_hz"] = 50
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "custom.json"
            save_config_file(path, values)
            loaded = load_config_file(path)
            loaded["lift_freq_base_hz"] = 950
            save_config_file(path, loaded)
            result = load_config_file(path)
        self.assertEqual(result["sea_level_pressure_pa"], 100123.5)
        self.assertEqual(result["auto_power_off_minutes"], 120)
        self.assertEqual(result["filter_mode"], "BARO_ONLY")
        self.assertEqual(result["bluetooth_battery_mode"], "PERCENT")
        self.assertEqual(result["bluetooth_notify_rate_hz"], 50)
        self.assertEqual(result["lift_freq_base_hz"], 950)

    def test_multiple_parameter_sets_are_sorted_and_preserved(self) -> None:
        one = default_parameters()
        three = default_parameters()
        three["lift_freq_base_hz"] = 1200
        document = ConfigDocument(
            {name: one[name] for name in SHARED_PARAMETER_SPECS},
            {
                3: {name: three[name] for name in PROFILE_PARAMETER_SPECS},
                1: {name: one[name] for name in PROFILE_PARAMETER_SPECS},
            },
        )
        rendered = config_document_json_text(document)
        root = json.loads(rendered)
        self.assertEqual(
            [item["parameter_number"] for item in root["vario_parameter_sets"]],
            [1, 3],
        )
        parsed = parse_config_document_text(rendered)
        self.assertEqual(parsed.vario_parameter_sets[3]["lift_freq_base_hz"], 1200)
        self.assertEqual(parsed.mc_parameters["sea_level_pressure_pa"], 101325.0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "setting.json"
            save_config_document_file(path, parsed)
            loaded = load_config_document_file(path)
        self.assertEqual(loaded, parsed)

    def test_accepts_five_parameter_sets_with_one_shared_configuration(self) -> None:
        values = default_parameters()
        document = ConfigDocument(
            {name: values[name] for name in SHARED_PARAMETER_SPECS},
            {
                number: {
                    **{name: values[name] for name in PROFILE_PARAMETER_SPECS},
                    "lift_freq_base_hz": 900 + number * 100,
                }
                for number in range(1, 6)
            },
        )
        parsed = parse_config_document_text(config_document_json_text(document))
        self.assertEqual(parsed.sorted_numbers(), (1, 2, 3, 4, 5))
        self.assertEqual(parsed.mc_parameters["filter_mode"], "AUTO")
        self.assertEqual(parsed.vario_parameter_sets[5]["lift_freq_base_hz"], 1400)

    def test_rejects_duplicate_out_of_range_and_too_many_sets(self) -> None:
        root = json.loads(config_json_text(default_parameters()))
        parameters = root["vario_parameter_sets"][0]["parameters"]
        with self.assertRaisesRegex(ConfigError, "duplicate parameter_number"):
            parse_config_document_text(
                json.dumps(
                    {
                        "format_version": 1,
                        "mc_parameters": root["mc_parameters"],
                        "vario_parameter_sets": [
                            {"parameter_number": 1, "parameters": parameters},
                            {"parameter_number": 1, "parameters": parameters},
                        ],
                    }
                )
            )
        with self.assertRaisesRegex(ConfigError, "between 1 and 5"):
            root["vario_parameter_sets"][0]["parameter_number"] = 6
            parse_config_document_text(json.dumps(root))
        with self.assertRaisesRegex(ConfigError, "1 to 5"):
            root["vario_parameter_sets"] = []
            parse_config_document_text(json.dumps(root))

    def test_failed_replace_leaves_existing_file_untouched(self) -> None:
        values = default_parameters()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "setting.json"
            path.write_text("original\n", encoding="utf-8")
            with mock.patch(
                "tools.parameters_model.os.replace",
                side_effect=OSError("injected failure"),
            ):
                with self.assertRaisesRegex(ConfigError, "injected failure"):
                    save_config_file(path, values)
            self.assertEqual(path.read_text(encoding="utf-8"), "original\n")
            self.assertEqual(list(Path(directory).iterdir()), [path])

    def test_rejects_files_larger_than_firmware_limit(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "large.json"
            path.write_bytes(b" " * (32 * 1024 + 1))
            with self.assertRaisesRegex(ConfigError, "exceeds 32768 bytes"):
                load_config_file(path)


class VarioStateTests(unittest.TestCase):
    @staticmethod
    def step(
        state: VarioAudioState,
        config: dict,
        now_s: float,
        altitude_m: float,
        climb_rate_mps: float,
        *,
        timestamp_s: float | None = None,
    ):
        sample = VarioSample(
            now_s if timestamp_s is None else timestamp_s,
            altitude_m,
            climb_rate_mps,
        )
        return vario_audio_step(state, config, sample, now_s)

    def test_zero_average_bypass_and_strict_lift_threshold(self) -> None:
        config = default_parameters()
        config["audio_climb_rate_average_s"] = 0.0
        config["audio_state_hold_ms"] = 0
        state = VarioAudioState()
        command = self.step(
            state, config, 1.0, math.nan, config["lift_start_mps"]
        )
        self.assertEqual(command.mode, AudioMode.SILENT)
        command = self.step(
            state, config, 1.1, math.nan, config["lift_start_mps"] + 0.001
        )
        self.assertEqual(command.mode, AudioMode.LIFT)
        self.assertEqual(state.averaged_climb_rate_mps, 0.101)
        self.assertEqual(state.history, [])

    def test_partial_window_duplicate_expiry_and_source_reset(self) -> None:
        config = default_parameters()
        config["audio_state_hold_ms"] = 0
        config["audio_climb_rate_average_s"] = 5.0
        state = VarioAudioState()
        self.step(state, config, 1.0, 0.0, 0.0)
        self.step(state, config, 2.0, 0.0, 1.0)
        self.step(state, config, 3.0, 0.0, 2.0)
        self.assertAlmostEqual(state.averaged_climb_rate_mps, 1.0)
        self.step(state, config, 3.0, 0.0, 20.0)
        self.assertAlmostEqual(state.averaged_climb_rate_mps, 1.0)
        self.step(state, config, 6.000001, 0.0, 4.0)
        self.assertAlmostEqual(state.averaged_climb_rate_mps, 7.0 / 3.0)

        sample = VarioSample(7.0, 0.0, 6.0, debug_input_active=True)
        vario_audio_step(state, config, sample, 7.0)
        self.assertAlmostEqual(state.averaged_climb_rate_mps, 6.0)
        self.assertEqual(len(state.history), 1)

    def test_stale_input_forces_silence_and_resets_history(self) -> None:
        config = default_parameters()
        state = VarioAudioState()
        self.step(state, config, 1.0, 0.0, 1.0)
        self.assertTrue(state.history)
        command = self.step(
            state,
            config,
            2.0,
            1.0,
            1.0,
            timestamp_s=2.0 - (config["audio_stale_ms"] + 1) / 1000.0,
        )
        self.assertEqual(command.mode, AudioMode.SILENT)
        self.assertFalse(state.history)
        self.assertFalse(state.averaged_climb_rate_valid)

    def test_predictive_lift_transitions_are_immediate_and_share_pitch(self) -> None:
        config = default_parameters()
        config["audio_climb_rate_average_s"] = 0.0
        config["predictive_buzzer_enabled"] = True
        config["audio_state_hold_ms"] = 1000
        state = VarioAudioState()

        command = self.step(state, config, 1.0, 0.0, 0.1)
        self.assertEqual(command.mode, AudioMode.PREDICTIVE)
        self.assertEqual(command.frequency_hz, lift_frequency_hz(config, 0.1))
        command = self.step(state, config, 1.01, 0.0, 0.21)
        self.assertEqual(command.mode, AudioMode.LIFT)
        self.assertTrue(command.sounding)
        command = self.step(state, config, 1.02, 0.0, 0.01)
        self.assertEqual(command.mode, AudioMode.PREDICTIVE)
        self.assertTrue(command.sounding)

        state = VarioAudioState()
        command = self.step(state, config, 2.0, 0.0, config["lift_start_mps"])
        self.assertEqual(command.mode, AudioMode.PREDICTIVE)

    def test_predictive_fixed_interval_and_duration(self) -> None:
        config = default_parameters()
        config["audio_climb_rate_average_s"] = 0.0
        config["predictive_buzzer_enabled"] = True
        state = VarioAudioState()

        self.assertTrue(self.step(state, config, 1.0, 0.0, 0.1).sounding)
        self.assertTrue(self.step(state, config, 1.149999, 0.0, 0.1).sounding)
        self.assertFalse(self.step(state, config, 1.150001, 0.0, 0.1).sounding)
        self.assertFalse(self.step(state, config, 1.999999, 0.0, 0.1).sounding)
        self.assertTrue(self.step(state, config, 2.0, 0.0, 0.1).sounding)
        self.assertTrue(self.step(state, config, 7.0, 0.0, 0.1).sounding)

    def test_frequency_and_tempo_curves_match_firmware_formulas(self) -> None:
        config = default_parameters()
        self.assertEqual(lift_frequency_hz(config, 0.2), 1067)
        self.assertEqual(lift_frequency_hz(config, 20.0), 2600)
        self.assertEqual(sink_frequency_hz(config, -3.0), 443)
        self.assertEqual(sink_frequency_hz(config, -20.0), 240)
        self.assertEqual(lift_phase_time_ms(config, 0.2), 400)
        self.assertEqual(lift_phase_time_ms(config, 0.6), 400)
        self.assertEqual(lift_phase_time_ms(config, 5.0), 100)


class PwmWaveformTests(unittest.TestCase):
    def test_rectangular_wave_is_zero_mean_at_supported_duties(self) -> None:
        waveform = PwmWaveform(sample_rate=1000, ramp_ms=0.0)
        samples = waveform.render(
            100,
            sounding=True,
            frequency_hz=50,
            duty_percent=25,
            amplifier_mode=3,
            volume=1.0,
        )
        self.assertAlmostEqual(sum(samples) / len(samples), 0.0, places=12)
        self.assertEqual(sum(sample > 0.0 for sample in samples), 25)

    def test_amplifier_modes_and_volume_scale_waveform(self) -> None:
        peaks = []
        for mode in (1, 2, 3):
            waveform = PwmWaveform(sample_rate=1000, ramp_ms=0.0)
            samples = waveform.render(
                20,
                sounding=True,
                frequency_hz=50,
                duty_percent=50,
                amplifier_mode=mode,
                volume=1.0,
            )
            peaks.append(max(abs(sample) for sample in samples))
        self.assertAlmostEqual(peaks[0], 1.0 / 3.0)
        self.assertAlmostEqual(peaks[1], 2.0 / 3.0)
        self.assertAlmostEqual(peaks[2], 1.0)

        waveform = PwmWaveform(sample_rate=1000, ramp_ms=0.0)
        silent = waveform.render(
            20,
            sounding=False,
            frequency_hz=0,
            duty_percent=50,
            amplifier_mode=3,
            volume=1.0,
        )
        self.assertEqual(silent, [0.0] * 20)


if __name__ == "__main__":
    unittest.main()
