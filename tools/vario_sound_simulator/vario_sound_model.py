"""Firmware-compatible vario sound state and waveform model."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from pathlib import Path
import sys
from typing import Any, Mapping

# Keep the supported ``python tools/vario_sound_simulator/vario_sound_simulator.py`` entry point
# working while the shared parameter contract is maintained with the tests.
if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

try:
    from tests.parameters_model import (
        AUDIO_PARAMETER_NAMES,
        FORMAT_VERSION,
        MAX_CONFIG_FILE_BYTES,
        MODEL_PARAMETER_SPECS,
        PARAMETER_SPECS,
        PROFILE_PARAMETER_SPECS,
        RUNTIME_CONTROL_SPECS,
        SHARED_PARAMETER_SPECS,
        ConfigDocument,
        ConfigError,
        ParameterSpec,
        config_document_json_text,
        config_json_text,
        default_config_document,
        default_parameters,
        load_config_document_file,
        load_config_file,
        parse_config_document_text,
        parse_config_text,
        save_config_document_file,
        save_config_file,
        validate_parameters,
    )
except ImportError:
    from tests.parameters_model import (
        AUDIO_PARAMETER_NAMES,
        FORMAT_VERSION,
        MAX_CONFIG_FILE_BYTES,
        MODEL_PARAMETER_SPECS,
        PARAMETER_SPECS,
        PROFILE_PARAMETER_SPECS,
        RUNTIME_CONTROL_SPECS,
        SHARED_PARAMETER_SPECS,
        ConfigDocument,
        ConfigError,
        ParameterSpec,
        config_document_json_text,
        config_json_text,
        default_config_document,
        default_parameters,
        load_config_document_file,
        load_config_file,
        parse_config_document_text,
        parse_config_text,
        save_config_document_file,
        save_config_file,
        validate_parameters,
    )


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
    history: list[tuple[float, float]] = field(default_factory=list)
    history_sum_mps: float = 0.0
    averaged_climb_rate_mps: float = 0.0
    averaged_climb_rate_valid: bool = False
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
    state.history.clear()
    state.history_sum_mps = 0.0
    state.averaged_climb_rate_mps = 0.0
    state.averaged_climb_rate_valid = False
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


def _clear_history(state: VarioAudioState) -> None:
    state.history.clear()
    state.history_sum_mps = 0.0
    state.averaged_climb_rate_mps = 0.0
    state.averaged_climb_rate_valid = False


def _averaged_climb_rate(
    state: VarioAudioState, config: Mapping[str, Any], sample: VarioSample
) -> float:
    window_s = float(config["audio_climb_rate_average_s"])
    if window_s == 0.0:
        _clear_history(state)
        state.averaged_climb_rate_mps = sample.climb_rate_mps
        state.averaged_climb_rate_valid = True
        return sample.climb_rate_mps

    if state.history:
        last_timestamp_s = state.history[-1][0]
        if sample.timestamp_s < last_timestamp_s:
            _clear_history(state)
        elif sample.timestamp_s == last_timestamp_s:
            return state.averaged_climb_rate_mps
    state.history.append((sample.timestamp_s, sample.climb_rate_mps))
    state.history_sum_mps += sample.climb_rate_mps
    while len(state.history) > 1 and sample.timestamp_s - state.history[0][0] > window_s:
        _, oldest_rate = state.history.pop(0)
        state.history_sum_mps -= oldest_rate
    state.averaged_climb_rate_mps = state.history_sum_mps / len(state.history)
    state.averaged_climb_rate_valid = True
    return state.averaged_climb_rate_mps


def _requested_mode(
    state: VarioAudioState, config: Mapping[str, Any], rate: float
) -> AudioMode:
    if state.mode == AudioMode.LIFT:
        if rate < config["lift_end_mps"]:
            if (
                config["predictive_buzzer_enabled"]
                and rate >= config["predictive_min_mps"]
            ):
                return AudioMode.PREDICTIVE
            return AudioMode.SILENT
        return AudioMode.LIFT
    if state.mode == AudioMode.SINK:
        if not config["sink_enabled"] or rate > config["sink_end_mps"]:
            return AudioMode.SILENT
        return AudioMode.SINK
    if state.mode == AudioMode.PREDICTIVE:
        if rate > config["lift_start_mps"]:
            return AudioMode.LIFT
        if (
            not config["predictive_buzzer_enabled"]
            or rate < config["predictive_min_mps"]
        ):
            return AudioMode.SILENT
        return AudioMode.PREDICTIVE

    if rate > config["lift_start_mps"]:
        return AudioMode.LIFT
    if (
        config["sink_enabled"]
        and rate < config["sink_start_mps"]
    ):
        return AudioMode.SINK
    if (
        config["predictive_buzzer_enabled"]
        and rate >= config["predictive_min_mps"]
        and rate <= config["lift_start_mps"]
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

    if (
        not state.input_source_valid
        or state.last_debug_input_active != sample.debug_input_active
    ):
        _clear_history(state)
        state.last_debug_input_active = sample.debug_input_active
        state.input_source_valid = True
    rate = _averaged_climb_rate(state, config, sample)
    requested = _requested_mode(state, config, rate)
    if (
        state.mode == AudioMode.LIFT
        and requested not in (AudioMode.LIFT, AudioMode.PREDICTIVE)
        and state.phase_on
    ):
        phase_s = lift_phase_time_ms(config, rate) / 1000.0
        elapsed_s = now_s - state.phase_started_s
        if 0.0 <= elapsed_s < phase_s:
            requested = AudioMode.LIFT

    if requested != state.mode:
        held_s = now_s - state.mode_started_s
        predictive_lift_transition = {
            state.mode,
            requested,
        } == {AudioMode.PREDICTIVE, AudioMode.LIFT}
        if (
            predictive_lift_transition
            or state.mode_started_s == 0.0
            or held_s >= config["audio_state_hold_ms"] / 1000.0
        ):
            state.mode = requested
            state.mode_started_s = now_s
            state.phase_started_s = now_s
            state.phase_on = requested != AudioMode.SILENT

    duty = int(config["audio_duty_percent"])
    amplifier = int(config["audio_amp_mode"])
    if state.mode == AudioMode.LIFT:
        phase_ms = lift_phase_time_ms(config, rate)
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
            lift_frequency_hz(config, rate),
            duty,
            amplifier,
            phase_ms,
        )
    if state.mode == AudioMode.SINK:
        return VarioAudioCommand(
            state.mode,
            True,
            sink_frequency_hz(config, rate),
            duty,
            amplifier,
            0,
        )
    if state.mode == AudioMode.PREDICTIVE:
        interval_ms = int(config["predictive_interval_ms"])
        duration_ms = int(config["predictive_duration_ms"])
        elapsed_ms = max((now_s - state.phase_started_s) * 1000.0, 0.0)
        state.phase_on = elapsed_ms % interval_ms < duration_ms
        return VarioAudioCommand(

            state.mode,
            state.phase_on,
            lift_frequency_hz(config, rate),
            duty,
            amplifier,
            duration_ms if state.phase_on else interval_ms - duration_ms,
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
