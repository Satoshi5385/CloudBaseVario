"""Firmware-compatible vario sound model and parameters.json handling."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable, Mapping


FORMAT_VERSION = 2
LEGACY_FORMAT_VERSION = 1
MAX_CONFIG_FILE_BYTES = 32 * 1024


class ConfigError(ValueError):
    """Raised when a parameters.json document violates the firmware contract."""


@dataclass(frozen=True)
class ParameterSpec:
    kind: str
    default: bool | int | float | str
    minimum: float | None = None
    maximum: float | None = None
    audio: bool = False


PARAMETER_SPECS: dict[str, ParameterSpec] = {
    "sea_level_pressure_pa": ParameterSpec("float", 101325.0, 80000.0, 110000.0),
    "filter_mode": ParameterSpec("enum", "AUTO"),
    "i2c_reinit_error_count": ParameterSpec("uint", 10, 1.0, 100.0),
    "imu_gyro_calibration_samples": ParameterSpec("uint", 200, 50.0, 2000.0),
    "imu_mahony_kp": ParameterSpec("float", 5.0, 0.0, 20.0),
    "imu_mahony_ki": ParameterSpec("float", 0.05, 0.0, 5.0),
    "audio_enabled": ParameterSpec("bool", True, audio=True),
    "sink_enabled": ParameterSpec("bool", True, audio=True),
    "predictive_buzzer_enabled": ParameterSpec("bool", False, audio=True),
    "lift_start_mps": ParameterSpec("float", 0.10, -1.0, 5.0, True),
    "lift_end_mps": ParameterSpec("float", 0.05, -1.0, 5.0, True),
    "lift_confirm_distance_m": ParameterSpec("float", 0.5, 0.0, 10.0, True),
    "sink_start_mps": ParameterSpec("float", -1.00, -10.0, 0.0, True),
    "sink_end_mps": ParameterSpec("float", -0.80, -10.0, 0.0, True),
    "sink_confirm_distance_m": ParameterSpec("float", 0.5, 0.0, 10.0, True),
    "audio_state_hold_ms": ParameterSpec("uint", 60, 0.0, 1000.0, True),
    "audio_stale_ms": ParameterSpec("uint", 500, 100.0, 500.0, True),
    "lift_freq_base_hz": ParameterSpec("uint", 600, 200.0, 5000.0, True),
    "lift_freq_rate_hz_per_mps": ParameterSpec(
        "float", 100.0, 0.0, 1000.0, True
    ),
    "lift_freq_max_hz": ParameterSpec("uint", 1800, 200.0, 5000.0, True),
    "lift_time_ms_at_0p2": ParameterSpec("uint", 480, 20.0, 2000.0, True),
    "lift_time_ms_at_1p0": ParameterSpec("uint", 220, 20.0, 2000.0, True),
    "lift_time_ms_at_2p5": ParameterSpec("uint", 100, 20.0, 2000.0, True),
    "lift_time_ms_at_5p0": ParameterSpec("uint", 70, 70.0, 2000.0, True),
    "sink_freq_start_hz": ParameterSpec("uint", 400, 130.0, 2000.0, True),
    "sink_freq_rate_hz_per_mps": ParameterSpec(
        "float", 70.0, 0.0, 500.0, True
    ),
    "sink_freq_min_hz": ParameterSpec("uint", 130, 130.0, 2000.0, True),
    "audio_duty_percent": ParameterSpec("uint", 50, 10.0, 90.0, True),
    "audio_amp_mode": ParameterSpec("uint", 1, 1.0, 3.0, True),
    "predictive_freq_hz": ParameterSpec("uint", 800, 200.0, 5000.0, True),
    "predictive_on_ms": ParameterSpec("uint", 20, 10.0, 1000.0, True),
    "predictive_off_ms": ParameterSpec("uint", 20, 10.0, 1000.0, True),
    "predictive_min_mps": ParameterSpec("float", -0.10, -2.0, 1.0, True),
    "predictive_max_mps": ParameterSpec("float", 0.10, -1.0, 2.0, True),
}

AUDIO_PARAMETER_NAMES = tuple(
    name for name, spec in PARAMETER_SPECS.items() if spec.audio
)

LEGACY_BOARD_AXIS_PARAMETERS = frozenset(
    {
        "imu_accel_x_source",
        "imu_accel_y_source",
        "imu_accel_z_source",
        "imu_accel_x_sign",
        "imu_accel_y_sign",
        "imu_accel_z_sign",
        "imu_gyro_x_source",
        "imu_gyro_y_source",
        "imu_gyro_z_source",
        "imu_gyro_x_sign",
        "imu_gyro_y_sign",
        "imu_gyro_z_sign",
    }
)

LEGACY_VERSION_1_PARAMETERS = frozenset(
    {"imu_accel_correction_min_g", "imu_accel_correction_max_g"}
)


def default_parameters(*, legacy_version: bool = False) -> dict[str, Any]:
    values = {name: spec.default for name, spec in PARAMETER_SPECS.items()}
    if legacy_version:
        values["imu_mahony_ki"] = 0.0
    return values


def _duplicate_rejecting_object(pairs: Iterable[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ConfigError(f"duplicate key: {key}")
        result[key] = value
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
        if value not in ("AUTO", "BARO_ONLY"):
            raise ConfigError(f"{name}: expected AUTO or BARO_ONLY")
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
    unknown = set(values) - set(PARAMETER_SPECS)
    missing = set(PARAMETER_SPECS) - set(values)
    if unknown:
        raise ConfigError(f"unknown parameter: {sorted(unknown)[0]}")
    if missing:
        raise ConfigError(f"missing parameter: {sorted(missing)[0]}")

    checked = {
        name: _validate_scalar(name, values[name], spec)
        for name, spec in PARAMETER_SPECS.items()
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
    if checked["predictive_min_mps"] > checked["predictive_max_mps"]:
        raise ConfigError("predictive_min_mps must not exceed predictive_max_mps")
    return checked


def parse_config_text(text: str) -> dict[str, Any]:
    try:
        root = json.loads(
            text.lstrip("\ufeff"),
            object_pairs_hook=_duplicate_rejecting_object,
            parse_constant=_reject_constant,
        )
    except ConfigError:
        raise
    except (json.JSONDecodeError, TypeError) as exc:
        raise ConfigError(f"invalid JSON: {exc}") from exc

    if not isinstance(root, dict):
        raise ConfigError("top-level value must be an object")
    unknown_top = set(root) - {"format_version", "parameters"}
    if unknown_top:
        raise ConfigError(f"unknown top-level key: {sorted(unknown_top)[0]}")
    if set(root) != {"format_version", "parameters"}:
        missing = {"format_version", "parameters"} - set(root)
        raise ConfigError(f"missing top-level key: {sorted(missing)[0]}")

    raw_version = root["format_version"]
    if (
        type(raw_version) not in (int, float)
        or not math.isfinite(float(raw_version))
        or not float(raw_version).is_integer()
    ):
        raise ConfigError("format_version must be an integer")
    version = int(raw_version)
    if version not in (LEGACY_FORMAT_VERSION, FORMAT_VERSION):
        raise ConfigError(f"unsupported format_version: {version}")
    parameters = root["parameters"]
    if not isinstance(parameters, dict):
        raise ConfigError("parameters must be an object")

    values = default_parameters(legacy_version=version == LEGACY_FORMAT_VERSION)
    for name, value in parameters.items():
        if name in LEGACY_BOARD_AXIS_PARAMETERS:
            continue
        if version == LEGACY_FORMAT_VERSION and name in LEGACY_VERSION_1_PARAMETERS:
            if type(value) not in (int, float) or not math.isfinite(float(value)):
                raise ConfigError(f"{name}: expected finite number")
            continue
        spec = PARAMETER_SPECS.get(name)
        if spec is None:
            raise ConfigError(f"unknown parameter: {name}")
        values[name] = _validate_scalar(name, value, spec)
    return validate_parameters(values)


def load_config_file(path: str | os.PathLike[str]) -> dict[str, Any]:
    try:
        contents = Path(path).read_bytes()
    except (OSError, UnicodeError) as exc:
        raise ConfigError(f"could not read file: {exc}") from exc
    if not contents:
        raise ConfigError("configuration file is empty")
    if len(contents) > MAX_CONFIG_FILE_BYTES:
        raise ConfigError(
            f"configuration file exceeds {MAX_CONFIG_FILE_BYTES} bytes"
        )
    try:
        text = contents.decode("utf-8-sig")
    except UnicodeError as exc:
        raise ConfigError(f"configuration file is not UTF-8: {exc}") from exc
    return parse_config_text(text)


def config_json_text(values: Mapping[str, Any]) -> str:
    checked = validate_parameters(values)
    document = {
        "format_version": FORMAT_VERSION,
        "parameters": checked,
    }
    return json.dumps(document, ensure_ascii=False, indent=2) + "\n"


def save_config_file(
    path: str | os.PathLike[str], values: Mapping[str, Any]
) -> None:
    target = Path(path)
    parent = target.parent
    if not parent.is_dir():
        raise ConfigError(f"destination directory does not exist: {parent}")
    rendered = config_json_text(values)
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
        verified = load_config_file(temporary_path)
        if verified != validate_parameters(values):
            raise ConfigError("saved file verification failed")
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


class AudioMode(str, Enum):
    SILENT = "SILENT"
    LIFT = "LIFT"
    SINK = "SINK"
    PREDICTIVE = "PREDICTIVE"


@dataclass
class VarioSample:
    timestamp_s: float
    altitude_m: float
    climb_rate_mps: float
    climb_rate_valid: bool = True
    estimate_valid: bool = True
    debug_input_active: bool = False


@dataclass
class VarioAudioState:
    mode: AudioMode = AudioMode.SILENT
    mode_started_s: float = 0.0
    phase_started_s: float = 0.0
    phase_on: bool = False
    reference_altitude_m: float = 0.0
    vertical_direction: int = 0
    reference_altitude_valid: bool = False
    last_debug_input_active: bool = False
    input_source_valid: bool = False


@dataclass(frozen=True)
class VarioAudioCommand:
    mode: AudioMode = AudioMode.SILENT
    sounding: bool = False
    frequency_hz: int = 0
    duty_percent: int = 0
    amplifier_mode: int = 0
    phase_time_ms: int = 0


def reset_audio_state(state: VarioAudioState) -> None:
    state.mode = AudioMode.SILENT
    state.mode_started_s = 0.0
    state.phase_started_s = 0.0
    state.phase_on = False
    state.reference_altitude_m = 0.0
    state.vertical_direction = 0
    state.reference_altitude_valid = False
    state.last_debug_input_active = False
    state.input_source_valid = False


def _lround_positive(value: float) -> int:
    return math.floor(value + 0.5)


def _interpolate_u32(x: float, x0: float, x1: float, y0: int, y1: int) -> int:
    if x <= x0 or x1 <= x0:
        return y0
    if x >= x1:
        return y1
    value = y0 + ((x - x0) / (x1 - x0)) * (y1 - y0)
    return _lround_positive(value)


def lift_phase_time_ms(config: Mapping[str, Any], climb_rate_mps: float) -> int:
    if climb_rate_mps <= 0.2:
        return int(config["lift_time_ms_at_0p2"])
    if climb_rate_mps <= 1.0:
        return _interpolate_u32(
            climb_rate_mps,
            0.2,
            1.0,
            int(config["lift_time_ms_at_0p2"]),
            int(config["lift_time_ms_at_1p0"]),
        )
    if climb_rate_mps <= 2.5:
        return _interpolate_u32(
            climb_rate_mps,
            1.0,
            2.5,
            int(config["lift_time_ms_at_1p0"]),
            int(config["lift_time_ms_at_2p5"]),
        )
    return _interpolate_u32(
        climb_rate_mps,
        2.5,
        5.0,
        int(config["lift_time_ms_at_2p5"]),
        int(config["lift_time_ms_at_5p0"]),
    )


def lift_frequency_hz(config: Mapping[str, Any], climb_rate_mps: float) -> int:
    frequency = float(config["lift_freq_base_hz"]) + float(
        config["lift_freq_rate_hz_per_mps"]
    ) * max(climb_rate_mps, 0.0)
    return _lround_positive(min(frequency, float(config["lift_freq_max_hz"])))


def sink_frequency_hz(config: Mapping[str, Any], climb_rate_mps: float) -> int:
    stronger_sink = max((-climb_rate_mps) - 1.0, 0.0)
    frequency = float(config["sink_freq_start_hz"]) - float(
        config["sink_freq_rate_hz_per_mps"]
    ) * stronger_sink
    return _lround_positive(max(frequency, float(config["sink_freq_min_hz"])))


def _vertical_direction(climb_rate_mps: float) -> int:
    if climb_rate_mps > 0.0:
        return 1
    if climb_rate_mps < 0.0:
        return -1
    return 0


def _clear_altitude_reference(state: VarioAudioState) -> None:
    state.reference_altitude_m = 0.0
    state.vertical_direction = 0
    state.reference_altitude_valid = False


def _update_altitude_reference(state: VarioAudioState, sample: VarioSample) -> None:
    if (
        not state.input_source_valid
        or state.last_debug_input_active != sample.debug_input_active
    ):
        _clear_altitude_reference(state)
        state.last_debug_input_active = sample.debug_input_active
        state.input_source_valid = True
    if (
        not sample.estimate_valid
        or not sample.climb_rate_valid
        or not math.isfinite(sample.altitude_m)
        or not math.isfinite(sample.climb_rate_mps)
    ):
        _clear_altitude_reference(state)
        return

    direction = _vertical_direction(sample.climb_rate_mps)
    if not state.reference_altitude_valid:
        state.reference_altitude_m = sample.altitude_m
        state.vertical_direction = direction
        state.reference_altitude_valid = True
        return
    if direction == 0:
        return
    if state.vertical_direction != 0 and direction != state.vertical_direction:
        state.reference_altitude_m = sample.altitude_m
    state.vertical_direction = direction


def _lift_distance_confirmed(
    state: VarioAudioState, config: Mapping[str, Any], sample: VarioSample
) -> bool:
    distance = float(config["lift_confirm_distance_m"])
    if distance == 0.0:
        return True
    return (
        state.reference_altitude_valid
        and sample.estimate_valid
        and math.isfinite(sample.altitude_m)
        and sample.altitude_m - state.reference_altitude_m > distance
    )


def _sink_distance_confirmed(
    state: VarioAudioState, config: Mapping[str, Any], sample: VarioSample
) -> bool:
    distance = float(config["sink_confirm_distance_m"])
    if distance == 0.0:
        return True
    return (
        state.reference_altitude_valid
        and sample.estimate_valid
        and math.isfinite(sample.altitude_m)
        and state.reference_altitude_m - sample.altitude_m > distance
    )


def _requested_mode(
    state: VarioAudioState, config: Mapping[str, Any], sample: VarioSample
) -> AudioMode:
    rate = sample.climb_rate_mps
    if state.mode == AudioMode.LIFT:
        return AudioMode.SILENT if rate < config["lift_end_mps"] else AudioMode.LIFT
    if state.mode == AudioMode.SINK:
        if not config["sink_enabled"] or rate > config["sink_end_mps"]:
            return AudioMode.SILENT
        return AudioMode.SINK
    if state.mode == AudioMode.PREDICTIVE:
        if (
            not config["predictive_buzzer_enabled"]
            or rate < config["predictive_min_mps"]
            or rate > config["predictive_max_mps"]
        ):
            return AudioMode.SILENT
        return AudioMode.PREDICTIVE

    if rate > config["lift_start_mps"] and _lift_distance_confirmed(
        state, config, sample
    ):
        return AudioMode.LIFT
    if (
        config["sink_enabled"]
        and rate < config["sink_start_mps"]
        and _sink_distance_confirmed(state, config, sample)
    ):
        return AudioMode.SINK
    if (
        config["predictive_buzzer_enabled"]
        and rate >= config["predictive_min_mps"]
        and rate <= config["predictive_max_mps"]
    ):
        return AudioMode.PREDICTIVE
    return AudioMode.SILENT


def vario_audio_step(
    state: VarioAudioState,
    config: Mapping[str, Any],
    sample: VarioSample,
    now_s: float,
) -> VarioAudioCommand:
    sample_age_s = now_s - sample.timestamp_s
    force_silent = (
        not config["audio_enabled"]
        or not sample.climb_rate_valid
        or not math.isfinite(sample.climb_rate_mps)
        or sample_age_s < 0.0
        or sample_age_s > float(config["audio_stale_ms"]) / 1000.0
    )
    if force_silent:
        reset_audio_state(state)
        return VarioAudioCommand()

    _update_altitude_reference(state, sample)
    requested = _requested_mode(state, config, sample)
    if state.mode == AudioMode.LIFT and requested != AudioMode.LIFT and state.phase_on:
        phase_s = lift_phase_time_ms(config, sample.climb_rate_mps) / 1000.0
        elapsed_s = now_s - state.phase_started_s
        if 0.0 <= elapsed_s < phase_s:
            requested = AudioMode.LIFT

    if requested != state.mode:
        held_s = now_s - state.mode_started_s
        if state.mode_started_s == 0.0 or held_s >= config["audio_state_hold_ms"] / 1000.0:
            state.mode = requested
            state.mode_started_s = now_s
            state.phase_started_s = now_s
            state.phase_on = requested != AudioMode.SILENT

    duty = int(config["audio_duty_percent"])
    amplifier = int(config["audio_amp_mode"])
    if state.mode == AudioMode.LIFT:
        phase_ms = lift_phase_time_ms(config, sample.climb_rate_mps)
        phase_s = phase_ms / 1000.0
        elapsed_s = now_s - state.phase_started_s
        if elapsed_s >= phase_s:
            phases_elapsed = math.floor(elapsed_s / phase_s)
            if phases_elapsed & 1:
                state.phase_on = not state.phase_on
            state.phase_started_s += phases_elapsed * phase_s
        return VarioAudioCommand(
            state.mode,
            state.phase_on,
            lift_frequency_hz(config, sample.climb_rate_mps),
            duty,
            amplifier,
            phase_ms,
        )
    if state.mode == AudioMode.SINK:
        return VarioAudioCommand(
            state.mode,
            True,
            sink_frequency_hz(config, sample.climb_rate_mps),
            duty,
            amplifier,
            0,
        )
    if state.mode == AudioMode.PREDICTIVE:
        phase_ms = int(
            config["predictive_on_ms"]
            if state.phase_on
            else config["predictive_off_ms"]
        )
        if now_s - state.phase_started_s >= phase_ms / 1000.0:
            state.phase_on = not state.phase_on
            state.phase_started_s = now_s
        return VarioAudioCommand(
            state.mode,
            state.phase_on,
            int(config["predictive_freq_hz"]),
            duty,
            amplifier,
            phase_ms,
        )
    return VarioAudioCommand(
        AudioMode.SILENT, False, 0, duty, amplifier, 0
    )


@dataclass
class PwmWaveform:
    """Generate a zero-mean rectangular waveform with a short gain ramp."""

    sample_rate: int = 48000
    ramp_ms: float = 3.0
    phase: float = 0.0
    gain: float = 0.0
    _was_sounding: bool = field(default=False, init=False)
    _frequency_hz: float = field(default=0.0, init=False)
    _duty: float = field(default=0.5, init=False)

    def reset(self) -> None:
        self.phase = 0.0
        self.gain = 0.0
        self._was_sounding = False
        self._frequency_hz = 0.0
        self._duty = 0.5

    def render(
        self,
        frames: int,
        *,
        sounding: bool,
        frequency_hz: int,
        duty_percent: int,
        amplifier_mode: int,
        volume: float,
    ) -> list[float]:
        if frames < 0:
            raise ValueError("frames must not be negative")
        requested_frequency = max(0.0, float(frequency_hz))
        requested_duty = min(max(float(duty_percent) / 100.0, 0.01), 0.99)
        if sounding and requested_frequency > 0.0:
            self._frequency_hz = requested_frequency
            self._duty = requested_duty
        frequency = self._frequency_hz
        duty = self._duty
        volume_gain = min(max(float(volume), 0.0), 1.0)
        mode_gain = min(max(int(amplifier_mode), 1), 3) / 3.0
        target_gain = volume_gain * mode_gain if sounding and frequency > 0.0 else 0.0
        ramp_samples = max(1, round(self.sample_rate * self.ramp_ms / 1000.0))
        gain_step = 1.0 / ramp_samples
        scale = max(duty, 1.0 - duty)
        high = (1.0 - duty) / scale
        low = -duty / scale
        output: list[float] = []

        if sounding and not self._was_sounding and self.gain == 0.0:
            self.phase = 0.0
        for _ in range(frames):
            if self.gain < target_gain:
                self.gain = min(target_gain, self.gain + gain_step)
            elif self.gain > target_gain:
                self.gain = max(target_gain, self.gain - gain_step)
            level = high if self.phase < duty else low
            output.append(level * self.gain)
            if sounding or self.gain > 0.0:
                self.phase = (self.phase + frequency / self.sample_rate) % 1.0
        if not sounding and self.gain == 0.0:
            self.phase = 0.0
            self._frequency_hz = 0.0
        self._was_sounding = sounding
        return output
