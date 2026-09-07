"""Firmware-compatible setting.json model for the sound simulator."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable, Mapping


FORMAT_VERSION = 1
MAX_CONFIG_FILE_BYTES = 32 * 1024


class ConfigError(ValueError):
    """Raised when a setting.json document violates the firmware contract."""


@dataclass(frozen=True)
class ParameterSpec:
    kind: str
    default: bool | int | float | str
    minimum: float | None = None
    maximum: float | None = None
    audio: bool = False
    choices: tuple[str, ...] = ()


@dataclass
class ConfigDocument:
    mc_parameters: dict[str, Any]
    vario_parameter_sets: dict[int, dict[str, Any]]

    def sorted_numbers(self) -> tuple[int, ...]:
        return tuple(sorted(self.vario_parameter_sets))

    def effective_parameters(self, parameter_number: int) -> dict[str, Any]:
        if parameter_number not in self.vario_parameter_sets:
            raise ConfigError(f"unknown parameter_number: {parameter_number}")
        values = default_parameters()
        values.update(self.mc_parameters)
        values.update(self.vario_parameter_sets[parameter_number])
        return validate_parameters(values)

    def update_profile(
        self, parameter_number: int, values: Mapping[str, Any]
    ) -> None:
        checked = validate_parameters(values)
        self.mc_parameters = {
            name: checked[name] for name in SHARED_PARAMETER_SPECS
        }
        self.vario_parameter_sets[parameter_number] = {
            name: checked[name] for name in PROFILE_PARAMETER_SPECS
        }


PARAMETER_SPECS: dict[str, ParameterSpec] = {
    "sea_level_pressure_pa": ParameterSpec("float", 101325.0, 80000.0, 110000.0),
    "auto_power_off_minutes": ParameterSpec("uint", 60, 0.0, 1440.0),
    "flight_gps_speed_threshold_kmh": ParameterSpec("float", 10.0, 1.0, 100.0),
    "stationary_confirm_seconds": ParameterSpec("uint", 60, 10.0, 600.0),
    "filter_mode": ParameterSpec(
        "enum", "AUTO", choices=("AUTO", "BARO_ONLY")
    ),
    "bluetooth_battery_mode": ParameterSpec(
        "enum", "PERCENT", choices=("VOLTAGE", "PERCENT")
    ),
    "bluetooth_tx_power": ParameterSpec(
        "enum", "LOW", choices=("MIN", "LOW", "NORMAL", "HIGH")
    ),
    "bluetooth_notify_rate_hz": ParameterSpec("uint", 10, 1.0, 50.0),
    "gps_send_interval_ms": ParameterSpec("uint", 1000, 200.0, 10000.0),
    "imu_gyro_calibration_samples": ParameterSpec("uint", 200, 50.0, 2000.0),
    "audio_mute_when_stationary": ParameterSpec("bool", False, audio=True),
    "predictive_buzzer_enabled": ParameterSpec("bool", False, audio=True),
    "audio_climb_rate_average_s": ParameterSpec("float", 1.0, 0.0, 10.0, True),
    "lift_start_mps": ParameterSpec("float", 0.10, -1.0, 5.0, True),
    "lift_end_mps": ParameterSpec("float", 0.08, -1.0, 5.0, True),
    "sink_start_mps": ParameterSpec("float", -1.80, -10.0, 0.0, True),
    "sink_end_mps": ParameterSpec("float", -1.70, -10.0, 0.0, True),
    "audio_state_hold_ms": ParameterSpec("uint", 200, 0.0, 1000.0, True),
    "audio_stale_ms": ParameterSpec("uint", 500, 100.0, 500.0, True),
    "lift_freq_base_hz": ParameterSpec("uint", 1047, 200.0, 5000.0, True),
    "lift_freq_rate_hz_per_mps": ParameterSpec(
        "float", 100.0, 0.0, 1000.0, True
    ),
    "lift_freq_max_hz": ParameterSpec("uint", 2600, 200.0, 5000.0, True),
    "lift_time_ms_at_0p2": ParameterSpec("uint", 400, 20.0, 2000.0, True),
    "lift_time_ms_at_1p0": ParameterSpec("uint", 400, 20.0, 2000.0, True),
    "lift_time_ms_at_2p5": ParameterSpec("uint", 300, 20.0, 2000.0, True),
    "lift_time_ms_at_5p0": ParameterSpec("uint", 100, 70.0, 2000.0, True),
    "sink_freq_start_hz": ParameterSpec("uint", 523, 130.0, 2000.0, True),
    "sink_freq_rate_hz_per_mps": ParameterSpec(
        "float", 40.0, 0.0, 500.0, True
    ),
    "sink_freq_min_hz": ParameterSpec("uint", 240, 130.0, 2000.0, True),
    "audio_duty_percent": ParameterSpec("uint", 50, 10.0, 90.0, True),
    "predictive_interval_ms": ParameterSpec("uint", 1000, 20.0, 2000.0, True),
    "predictive_duration_ms": ParameterSpec("uint", 150, 10.0, 1000.0, True),
    "predictive_min_mps": ParameterSpec("float", 0.01, -2.0, 1.0, True),
}

RUNTIME_CONTROL_SPECS: dict[str, ParameterSpec] = {
    "audio_enabled": ParameterSpec("bool", True, audio=True),
    "sink_enabled": ParameterSpec("bool", True, audio=True),
    "audio_amp_mode": ParameterSpec("uint", 1, 1.0, 3.0, True),
}

MODEL_PARAMETER_SPECS = {**PARAMETER_SPECS, **RUNTIME_CONTROL_SPECS}

SHARED_PARAMETER_SPECS = {
    name: spec for name, spec in PARAMETER_SPECS.items() if not spec.audio
}
PROFILE_PARAMETER_SPECS = {
    name: spec for name, spec in PARAMETER_SPECS.items() if spec.audio
}

AUDIO_PARAMETER_NAMES = tuple(
    name for name, spec in MODEL_PARAMETER_SPECS.items() if spec.audio
)

def default_parameters() -> dict[str, Any]:
    return {
        name: spec.default for name, spec in MODEL_PARAMETER_SPECS.items()
    }


def default_config_document() -> ConfigDocument:
    values = default_parameters()
    shared = {name: values[name] for name in SHARED_PARAMETER_SPECS}
    profile = {name: values[name] for name in PROFILE_PARAMETER_SPECS}
    return ConfigDocument(
        shared,
        {
            1: dict(profile),
            2: {
                **profile,
                "lift_start_mps": 0.20,
                "lift_end_mps": 0.18,
                "sink_start_mps": -2.00,
                "sink_end_mps": -1.90,
            },
            3: {
                **profile,
                "lift_start_mps": 0.30,
                "lift_end_mps": 0.29,
                "sink_start_mps": -2.20,
                "sink_end_mps": -2.10,
            },
        },
    )


def _duplicate_rejecting_object(pairs: Iterable[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ConfigError(f"duplicate key: {key}")
        result[key] = value
    return result


@dataclass(frozen=True)
class _ParsedObject:
    pairs: tuple[tuple[str, Any], ...]


def _preserve_object_pairs(pairs: Iterable[tuple[str, Any]]) -> _ParsedObject:
    return _ParsedObject(tuple(pairs))


def _recognized_object(
    value: Any, recognized_keys: Iterable[str]
) -> dict[str, Any] | None:
    if not isinstance(value, _ParsedObject):
        return None
    recognized = set(recognized_keys)
    result: dict[str, Any] = {}
    for key, item in value.pairs:
        if key not in recognized:
            continue
        if key in result:
            return None
        result[key] = item
    return result


def _reject_constant(value: str) -> None:
    raise ConfigError(f"non-finite JSON number: {value}")


def _validate_scalar(name: str, value: Any, spec: ParameterSpec) -> Any:
    if spec.kind == "bool":
        if type(value) is not bool:
            raise ConfigError(f"{name}: expected bool")
        return value
    if spec.kind == "uint":
        if type(value) not in (int, float) or not math.isfinite(float(value)):
            raise ConfigError(f"{name}: expected integer")
        if float(value) < 0.0 or not float(value).is_integer():
            raise ConfigError(f"{name}: expected integer")
        converted: int | float = int(value)
    elif spec.kind == "float":
        if type(value) not in (int, float):
            raise ConfigError(f"{name}: expected number")
        converted = float(value)
        if not math.isfinite(converted):
            raise ConfigError(f"{name}: expected finite number")
    elif spec.kind == "enum":
        if value not in spec.choices:
            expected = " or ".join(spec.choices)
            raise ConfigError(f"{name}: expected {expected}")
        return value

    else:
        raise ConfigError(f"{name}: unsupported parameter type")

    assert spec.minimum is not None and spec.maximum is not None
    if converted < spec.minimum or converted > spec.maximum:
        raise ConfigError(
            f"{name}: value must be between {spec.minimum:g} and {spec.maximum:g}"
        )
    return converted


def validate_parameters(values: Mapping[str, Any]) -> dict[str, Any]:
    unknown = set(values) - set(MODEL_PARAMETER_SPECS)
    missing = set(MODEL_PARAMETER_SPECS) - set(values)
    if unknown:
        raise ConfigError(f"unknown parameter: {sorted(unknown)[0]}")
    if missing:
        raise ConfigError(f"missing parameter: {sorted(missing)[0]}")

    checked = {
        name: _validate_scalar(name, values[name], spec)
        for name, spec in MODEL_PARAMETER_SPECS.items()
    }
    if not (
        checked["sink_start_mps"]
        <= checked["sink_end_mps"]
        < checked["lift_end_mps"]
        <= checked["lift_start_mps"]
    ):
        raise ConfigError(
            "expected sink_start_mps <= sink_end_mps < "
            "lift_end_mps <= lift_start_mps"
        )
    if checked["lift_freq_base_hz"] > checked["lift_freq_max_hz"]:
        raise ConfigError("lift_freq_base_hz must not exceed lift_freq_max_hz")
    if checked["sink_freq_min_hz"] > checked["sink_freq_start_hz"]:
        raise ConfigError("sink_freq_min_hz must not exceed sink_freq_start_hz")
    if not (
        checked["lift_time_ms_at_0p2"]
        >= checked["lift_time_ms_at_1p0"]
        >= checked["lift_time_ms_at_2p5"]
        >= checked["lift_time_ms_at_5p0"]
    ):
        raise ConfigError("lift timing control points must be non-increasing")
    if checked["predictive_min_mps"] > checked["lift_start_mps"]:
        raise ConfigError("predictive_min_mps must not exceed lift_start_mps")
    if checked["predictive_duration_ms"] > checked["predictive_interval_ms"]:
        raise ConfigError(
            "predictive_duration_ms must not exceed predictive_interval_ms"
        )
    return checked


def _parse_parameter_values(
    parameters: Any, expected_specs: Mapping[str, ParameterSpec]
) -> dict[str, Any]:
    if not isinstance(parameters, dict):
        raise ConfigError("parameters must be an object")
    unknown = set(parameters) - set(expected_specs)
    missing = set(expected_specs) - set(parameters)
    if unknown:
        raise ConfigError(f"unknown parameter: {sorted(unknown)[0]}")
    if missing:
        raise ConfigError(f"missing parameter: {sorted(missing)[0]}")
    return {
        name: _validate_scalar(name, parameters[name], spec)
        for name, spec in expected_specs.items()
    }


def _read_parameter_values(
    raw: Any, specs: Mapping[str, ParameterSpec]
) -> dict[str, Any] | None:
    values = _recognized_object(raw, specs)
    if values is None or set(values) != set(specs):
        return None
    try:
        return {
            name: _validate_scalar(name, values[name], spec)
            for name, spec in specs.items()
        }
    except ConfigError:
        return None


def _read_profile_number(raw: Any) -> int | None:
    profile = _recognized_object(raw, ("parameter_number",))
    if profile is None or "parameter_number" not in profile:
        return None
    number = profile["parameter_number"]
    if type(number) not in (int, float) or not math.isfinite(float(number)):
        return None
    if not float(number).is_integer() or not 1 <= int(number) <= 5:
        return None
    return int(number)


def parse_config_document_text(text: str) -> ConfigDocument:
    try:
        root = json.loads(
            text.lstrip("\ufeff"),
            object_pairs_hook=_preserve_object_pairs,
            parse_constant=_reject_constant,
        )
    except (ConfigError, json.JSONDecodeError, TypeError):
        return default_config_document()

    root_values = _recognized_object(
        root, ("format_version", "mc_parameters", "vario_parameter_sets")
    )
    if root_values is None or set(root_values) != {
        "format_version",
        "mc_parameters",
        "vario_parameter_sets",
    }:
        return default_config_document()
    raw_version = root_values["format_version"]
    if (
        type(raw_version) not in (int, float)
        or not math.isfinite(float(raw_version))
        or not float(raw_version).is_integer()
    ):
        return default_config_document()
    version = int(raw_version)
    if version != FORMAT_VERSION:
        return default_config_document()
    shared_parameters = _read_parameter_values(
        root_values["mc_parameters"], SHARED_PARAMETER_SPECS
    )
    if shared_parameters is None:
        return default_config_document()
    raw_sets = root_values["vario_parameter_sets"]
    if not isinstance(raw_sets, list):
        return default_config_document()
    number_counts: dict[int, int] = {}
    for raw_set in raw_sets:
        number = _read_profile_number(raw_set)
        if number is not None:
            number_counts[number] = number_counts.get(number, 0) + 1
    parameter_sets: dict[int, dict[str, Any]] = {}
    for raw_set in raw_sets:
        number = _read_profile_number(raw_set)
        profile = _recognized_object(
            raw_set, ("parameter_number", "parameters")
        )
        if (
            number is None
            or number_counts.get(number) != 1
            or profile is None
            or "parameters" not in profile
        ):
            continue
        profile_parameters = _read_parameter_values(
            profile["parameters"], PROFILE_PARAMETER_SPECS
        )
        if profile_parameters is None:
            continue
        try:
            validate_parameters(
                {
                    **default_parameters(),
                    **shared_parameters,
                    **profile_parameters,
                }
            )
        except ConfigError:
            continue
        parameter_sets[number] = profile_parameters
    if not parameter_sets:
        return default_config_document()
    return ConfigDocument(
        shared_parameters, dict(sorted(parameter_sets.items()))
    )


def parse_config_text(text: str) -> dict[str, Any]:
    document = parse_config_document_text(text)
    return document.effective_parameters(document.sorted_numbers()[0])


def load_config_file(path: str | os.PathLike[str]) -> dict[str, Any]:
    document = load_config_document_file(path)
    return document.effective_parameters(document.sorted_numbers()[0])


def load_config_document_file(path: str | os.PathLike[str]) -> ConfigDocument:
    try:
        contents = Path(path).read_bytes()
    except (OSError, UnicodeError) as exc:
        raise ConfigError(f"could not read file: {exc}") from exc
    if not contents:
        return default_config_document()
    if len(contents) > MAX_CONFIG_FILE_BYTES:
        return default_config_document()
    try:
        text = contents.decode("utf-8-sig")
    except UnicodeError:
        return default_config_document()
    return parse_config_document_text(text)


def config_json_text(values: Mapping[str, Any]) -> str:
    checked = validate_parameters(values)
    return config_document_json_text(
        ConfigDocument(
            {name: checked[name] for name in SHARED_PARAMETER_SPECS},
            {1: {name: checked[name] for name in PROFILE_PARAMETER_SPECS}},
        )

    )


def config_document_json_text(document: ConfigDocument) -> str:
    if not 1 <= len(document.vario_parameter_sets) <= 5:
        raise ConfigError("vario_parameter_sets must contain 1 to 5 sets")
    checked_shared = _parse_parameter_values(
        document.mc_parameters, SHARED_PARAMETER_SPECS
    )
    checked_sets: dict[int, dict[str, Any]] = {}
    for number, values in document.vario_parameter_sets.items():
        if type(number) is not int or not 1 <= number <= 5:
            raise ConfigError("parameter_number must be between 1 and 5")
        checked_profile = _parse_parameter_values(
            values, PROFILE_PARAMETER_SPECS
        )
        validate_parameters(
            {**default_parameters(), **checked_shared, **checked_profile}
        )
        checked_sets[number] = checked_profile
    rendered_document = {
        "format_version": FORMAT_VERSION,
        "mc_parameters": {
            name: checked_shared[name] for name in SHARED_PARAMETER_SPECS
        },
        "vario_parameter_sets": [
            {
                "parameter_number": number,
                "parameters": {
                    name: checked_sets[number][name]
                    for name in PROFILE_PARAMETER_SPECS
                },
            }
            for number in sorted(checked_sets)
        ],
    }
    return json.dumps(rendered_document, ensure_ascii=False, indent=2) + "\n"


def save_config_file(
    path: str | os.PathLike[str], values: Mapping[str, Any]
) -> None:
    checked = validate_parameters(values)
    save_config_document_file(
        path,
        ConfigDocument(
            {name: checked[name] for name in SHARED_PARAMETER_SPECS},
            {1: {name: checked[name] for name in PROFILE_PARAMETER_SPECS}},
        ),
    )


def save_config_document_file(
    path: str | os.PathLike[str], document: ConfigDocument
) -> None:
    target = Path(path)
    parent = target.parent
    if not parent.is_dir():
        raise ConfigError(f"destination directory does not exist: {parent}")
    rendered = config_document_json_text(document)
    temporary_path: Path | None = None
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{target.name}.", suffix=".tmp", dir=parent
        )
        temporary_path = Path(temporary_name)
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(rendered)
            stream.flush()
            os.fsync(stream.fileno())
        verified_bytes = temporary_path.read_bytes()
        if verified_bytes != rendered.encode("utf-8"):
            raise ConfigError("saved file verification failed")
        try:
            json.loads(
                verified_bytes.decode("utf-8"),
                object_pairs_hook=_duplicate_rejecting_object,
                parse_constant=_reject_constant,
            )
        except (ConfigError, json.JSONDecodeError, UnicodeError) as exc:
            raise ConfigError("saved file verification failed") from exc
        os.replace(temporary_path, target)
        temporary_path = None
    except ConfigError:
        raise
    except OSError as exc:
        raise ConfigError(f"could not save file: {exc}") from exc
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except OSError:
                pass
