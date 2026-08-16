"""Parsing helpers for the CloudBaseVario TinyUSB CDC protocol."""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import TypeAlias

Scalar: TypeAlias = int | float | str

DISPLAY_NORMAL = "normal"
DISPLAY_INACTIVE = "inactive"
DISPLAY_WARNING = "warning"
DISPLAY_ERROR = "error"
DISPLAY_UNAVAILABLE = "unavailable"

TELEMETRY_PREFIX = "BARO"
MAX_COMMAND_BYTES = 128

_INTEGER_PATTERN = re.compile(r"^[+-]?\d+$")
_FLOAT_PATTERN = re.compile(
    r"^[+-]?(?:\d+\.\d*|\d*\.\d+)(?:[eE][+-]?\d+)?$"
)
_PARAMETER_NAME_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


@dataclass(frozen=True)
class TelemetrySample:
    """One decoded BARO telemetry line."""

    fields: dict[str, Scalar]
    raw_line: str

    def integer(self, name: str, default: int = 0) -> int:
        value = self.fields.get(name, default)
        if isinstance(value, bool):
            return int(value)
        if isinstance(value, int):
            return value
        if isinstance(value, float):
            return int(value)
        try:
            return int(str(value), 10)
        except (TypeError, ValueError):
            return default

    def number(self, name: str, default: float = 0.0) -> float:
        value = self.fields.get(name, default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def flag(self, name: str) -> bool:
        return self.integer(name) != 0

    def text(self, name: str, default: str = "") -> str:
        return str(self.fields.get(name, default))


@dataclass(frozen=True)
class TelemetryFieldSpec:
    """Presentation metadata for one known BARO telemetry field."""

    name: str
    label: str
    unit: str = ""
    precision: int | None = None


@dataclass(frozen=True)
class DisplayItem:
    """One GUI-ready value with a semantic state for coloring."""

    key: str
    label: str
    value: str
    state: str = DISPLAY_NORMAL


@dataclass(frozen=True)
class TelemetryGroup:
    """A named diagnostic group in the current BARO contract."""

    key: str
    title: str
    items: tuple[DisplayItem, ...]


@dataclass(frozen=True)
class TelemetryViewModel:
    """Pure presentation model derived from one complete BARO record."""

    flight: tuple[DisplayItem, ...]
    statuses: tuple[DisplayItem, ...]
    diagnostics: tuple[TelemetryGroup, ...]


FLIGHT_FIELD_SPECS = (
    TelemetryFieldSpec("pressure_pa", "Pressure", "Pa", 2),
    TelemetryFieldSpec("altitude_m", "Altitude", "m", 2),
    TelemetryFieldSpec("climb_mps", "Climb rate", "m/s", 3),
    TelemetryFieldSpec("vertical_accel_mps2", "Vertical accel", "m/s²", 3),
    TelemetryFieldSpec("temp_c", "Temperature", "°C", 2),
)


QUALITY_FIELD_SPECS = (
    TelemetryFieldSpec("raw_temp", "BMP581 raw temperature", "count", 0),
    TelemetryFieldSpec("raw_pressure", "BMP581 raw pressure", "count", 0),
    TelemetryFieldSpec("i2c_errors", "I²C errors", "", 0),
    TelemetryFieldSpec("overruns", "BMP581 overruns", "", 0),
    TelemetryFieldSpec("kalman_accel_bias_mps2", "Kalman accel bias", "m/s²", 4),
    TelemetryFieldSpec("kalman_baro_innovation_m", "Baro innovation", "m", 4),
    TelemetryFieldSpec(
        "kalman_accel_innovation_mps2", "Accel innovation", "m/s²", 4
    ),
    TelemetryFieldSpec("kalman_baro_r_m2", "Baro measurement variance", "m²", 5),
    TelemetryFieldSpec(
        "kalman_accel_r_m2_s4", "Accel measurement variance", "m²/s⁴", 5
    ),
)


IMU_FIELD_SPECS = (
    TelemetryFieldSpec("imu_samples", "IMU samples", "", 0),
    TelemetryFieldSpec("imu_missed", "Missed IMU samples", "", 0),
    TelemetryFieldSpec("imu_confidence", "IMU confidence", "%", 1),
    TelemetryFieldSpec("imu_vibration_rms_g", "IMU vibration RMS", "g", 4),
    TelemetryFieldSpec("imu_kp_effective", "Effective Mahony Kp", "", 4),
    TelemetryFieldSpec("imu_ki_effective", "Effective Mahony Ki", "", 4),
    TelemetryFieldSpec("imu_cal_samples", "Accel calibration samples", "", 0),
    TelemetryFieldSpec("imu_cal_storage", "Calibration storage", ""),
    TelemetryFieldSpec("imu_cal_storage_error", "Calibration storage error", "", 0),
)


BLE_FIELD_SPECS = (
    TelemetryFieldSpec("ble_pressure_pa", "LK8EX1 pressure", "Pa", 0),
    TelemetryFieldSpec("ble_altitude_m", "LK8EX1 altitude", "m", 0),
    TelemetryFieldSpec("ble_vario_cm_s", "LK8EX1 vario", "cm/s", 0),
    TelemetryFieldSpec("ble_temperature_c", "LK8EX1 temperature", "°C", 0),
    TelemetryFieldSpec("ble_battery", "LK8EX1 battery", ""),
    TelemetryFieldSpec("seq", "BARO sequence", "", 0),
    TelemetryFieldSpec("timestamp_us", "Source timestamp", "µs", 0),
    TelemetryFieldSpec("stream_drops", "Serial stream drops", "", 0),
)


_BLE_SENTINELS = {
    "ble_pressure_pa": {"999999"},
    "ble_altitude_m": {"99999"},
    "ble_vario_cm_s": {"9999"},
    "ble_temperature_c": {"99"},
    "ble_battery": {"999"},
}


def _optional_flag(sample: TelemetrySample, name: str) -> bool | None:
    if name not in sample.fields:
        return None
    return sample.flag(name)


def _number_item(
    sample: TelemetrySample,
    spec: TelemetryFieldSpec,
    *,
    valid: bool | None = True,
    multiplier: float = 1.0,
    warning_if_nonzero: bool = False,
) -> DisplayItem:
    if valid is not True or spec.name not in sample.fields:
        return DisplayItem(spec.name, spec.label, "--", DISPLAY_UNAVAILABLE)
    value = sample.number(spec.name) * multiplier
    if spec.precision is None:
        rendered = sample.text(spec.name)
    elif spec.precision == 0:
        rendered = str(sample.integer(spec.name))
    else:
        rendered = f"{value:.{spec.precision}f}"
    if spec.unit:
        rendered = f"{rendered} {spec.unit}"
    state = DISPLAY_WARNING if warning_if_nonzero and value != 0.0 else DISPLAY_NORMAL
    return DisplayItem(spec.name, spec.label, rendered, state)


def _flag_item(
    sample: TelemetrySample,
    name: str,
    label: str,
    *,
    true_text: str = "YES",
    false_text: str = "NO",
    true_state: str = DISPLAY_NORMAL,
    false_state: str = DISPLAY_INACTIVE,
) -> DisplayItem:
    value = _optional_flag(sample, name)
    if value is None:
        return DisplayItem(name, label, "--", DISPLAY_UNAVAILABLE)
    return DisplayItem(
        name,
        label,
        true_text if value else false_text,
        true_state if value else false_state,
    )


def _status_item(
    key: str, label: str, value: str, state: str
) -> DisplayItem:
    return DisplayItem(key, label, value, state)


def _calibration_status(sample: TelemetrySample) -> DisplayItem:
    skipped = _optional_flag(sample, "imu_accel_cal_skipped")
    persisted = _optional_flag(sample, "imu_accel_cal_persisted")
    save_pending = _optional_flag(sample, "imu_cal_save_pending")
    imu_online = _optional_flag(sample, "imu_online")
    storage_error = (
        sample.integer("imu_cal_storage_error")
        if "imu_cal_storage_error" in sample.fields
        else None
    )
    if skipped:
        return _status_item("calibration", "CAL", "SKIPPED", DISPLAY_WARNING)
    if storage_error not in (None, 0):
        return _status_item("calibration", "CAL", "SAVE ERROR", DISPLAY_ERROR)
    if save_pending:
        return _status_item("calibration", "CAL", "SAVING", DISPLAY_WARNING)
    if persisted:
        return _status_item("calibration", "CAL", "READY", DISPLAY_NORMAL)
    if imu_online:
        return _status_item("calibration", "CAL", "CALIBRATING", DISPLAY_WARNING)
    return _status_item("calibration", "CAL", "--", DISPLAY_UNAVAILABLE)


def _ble_item(sample: TelemetrySample, spec: TelemetryFieldSpec) -> DisplayItem:
    if spec.name not in sample.fields:
        return DisplayItem(spec.name, spec.label, "--", DISPLAY_UNAVAILABLE)
    raw_value = sample.text(spec.name)
    if raw_value in _BLE_SENTINELS.get(spec.name, set()):
        return DisplayItem(spec.name, spec.label, "--", DISPLAY_UNAVAILABLE)
    return _number_item(sample, spec)


def build_telemetry_view(sample: TelemetrySample) -> TelemetryViewModel:
    """Interpret the current firmware BARO contract for the GUI.

    The raw sample remains intentionally open-ended; fields unknown to this
    version are still available in the GUI's all-fields view.
    """

    pressure_valid = _optional_flag(sample, "pressure_valid")
    estimate_valid = _optional_flag(sample, "estimate_valid")
    climb_valid = _optional_flag(sample, "climb_valid")
    vertical_accel_valid = _optional_flag(sample, "vertical_accel_valid")
    imu_online = _optional_flag(sample, "imu_online")
    attitude_valid = _optional_flag(sample, "imu_attitude_valid")
    fusion_active = _optional_flag(sample, "fusion_active")
    ble_notify = _optional_flag(sample, "ble_notify")
    online = _optional_flag(sample, "online")

    flight = (
        _number_item(sample, FLIGHT_FIELD_SPECS[0], valid=pressure_valid),
        _number_item(sample, FLIGHT_FIELD_SPECS[1], valid=estimate_valid),
        _number_item(sample, FLIGHT_FIELD_SPECS[2], valid=climb_valid),
        _number_item(
            sample, FLIGHT_FIELD_SPECS[3], valid=vertical_accel_valid
        ),
        _number_item(sample, FLIGHT_FIELD_SPECS[4], valid=pressure_valid),
    )

    if online is None or pressure_valid is None:
        baro_status = _status_item("baro", "BARO", "--", DISPLAY_UNAVAILABLE)
    elif online and pressure_valid:
        baro_status = _status_item("baro", "BARO", "READY", DISPLAY_NORMAL)
    else:
        baro_status = _status_item("baro", "BARO", "OFFLINE", DISPLAY_ERROR)

    def flag_status(
        key: str, label: str, value: bool | None, active_text: str = "ON"
    ) -> DisplayItem:
        if value is None:
            return _status_item(key, label, "--", DISPLAY_UNAVAILABLE)
        return _status_item(
            key,
            label,
            active_text if value else "OFF",
            DISPLAY_NORMAL if value else DISPLAY_INACTIVE,
        )

    drops = (
        sample.integer("stream_drops")
        if "stream_drops" in sample.fields
        else None
    )
    if drops is None:
        stream_status = _status_item("stream", "STREAM", "--", DISPLAY_UNAVAILABLE)
    elif drops == 0:
        stream_status = _status_item("stream", "STREAM", "CLEAN", DISPLAY_NORMAL)
    else:
        stream_status = _status_item(
            "stream", "STREAM", f"DROPS {drops}", DISPLAY_WARNING
        )

    statuses = (
        baro_status,
        flag_status("estimate", "EST", estimate_valid, "VALID"),
        flag_status("imu", "IMU", imu_online),
        _calibration_status(sample),
        flag_status("attitude", "ATT", attitude_valid, "VALID"),
        flag_status("fusion", "FUSION", fusion_active),
        flag_status("ble", "BLE", ble_notify, "NOTIFY"),
        stream_status,
    )

    quality = (
        _number_item(sample, QUALITY_FIELD_SPECS[0], valid=pressure_valid),
        _number_item(sample, QUALITY_FIELD_SPECS[1], valid=pressure_valid),
        _number_item(sample, QUALITY_FIELD_SPECS[2], warning_if_nonzero=True),
        _number_item(sample, QUALITY_FIELD_SPECS[3], warning_if_nonzero=True),
        _number_item(sample, QUALITY_FIELD_SPECS[4], valid=estimate_valid),
        _number_item(
            sample,
            QUALITY_FIELD_SPECS[5],
            valid=_optional_flag(sample, "kalman_baro_innovation_valid"),
        ),
        _number_item(
            sample,
            QUALITY_FIELD_SPECS[6],
            valid=_optional_flag(sample, "kalman_accel_innovation_valid"),
        ),
        _number_item(sample, QUALITY_FIELD_SPECS[7], valid=estimate_valid),
        _number_item(sample, QUALITY_FIELD_SPECS[8], valid=estimate_valid),
    )

    imu = (
        _flag_item(sample, "imu_online", "IMU online"),
        _flag_item(
            sample,
            "imu_stale",
            "IMU stale",
            true_text="STALE",
            false_text="FRESH",
            true_state=DISPLAY_WARNING,
            false_state=DISPLAY_NORMAL,
        ),
        _flag_item(sample, "imu_calibrated", "IMU calibrated"),
        _flag_item(sample, "imu_accel_calibrated", "Accel calibrated"),
        _flag_item(sample, "imu_accel_cal_persisted", "Accel calibration persisted"),
        _flag_item(
            sample,
            "imu_accel_cal_skipped",
            "Accel calibration skipped",
            true_text="SKIPPED",
            false_text="NO",
            true_state=DISPLAY_WARNING,
            false_state=DISPLAY_NORMAL,
        ),
        _number_item(sample, IMU_FIELD_SPECS[0], valid=imu_online),
        _number_item(sample, IMU_FIELD_SPECS[1], warning_if_nonzero=True),
        _number_item(sample, IMU_FIELD_SPECS[2], valid=imu_online, multiplier=100.0),
        _number_item(sample, IMU_FIELD_SPECS[3], valid=imu_online),
        _number_item(sample, IMU_FIELD_SPECS[4], valid=imu_online),
        _number_item(sample, IMU_FIELD_SPECS[5], valid=imu_online),
        _flag_item(sample, "imu_ki_active", "Mahony Ki active", true_text="ACTIVE", false_text="OFF"),
        _number_item(sample, IMU_FIELD_SPECS[6]),
        _flag_item(
            sample,
            "imu_cal_save_pending",
            "Calibration save",
            true_text="PENDING",
            false_text="IDLE",
            true_state=DISPLAY_WARNING,
            false_state=DISPLAY_NORMAL,
        ),
        _number_item(sample, IMU_FIELD_SPECS[7]),
        _number_item(sample, IMU_FIELD_SPECS[8], warning_if_nonzero=True),
    )

    ble = (
        *(_ble_item(sample, spec) for spec in BLE_FIELD_SPECS[:5]),
        _flag_item(sample, "ble_available", "LK8EX1 available", true_text="AVAILABLE", false_text="UNAVAILABLE"),
        _flag_item(sample, "ble_notify", "BLE notify", true_text="NOTIFY", false_text="IDLE", false_state=DISPLAY_NORMAL),
        _number_item(sample, BLE_FIELD_SPECS[5]),
        _number_item(sample, BLE_FIELD_SPECS[6]),
        _number_item(sample, BLE_FIELD_SPECS[7], warning_if_nonzero=True),
    )

    return TelemetryViewModel(
        flight=flight,
        statuses=statuses,
        diagnostics=(
            TelemetryGroup("quality", "Sensor / estimator quality", quality),
            TelemetryGroup("imu", "IMU / calibration", imu),
            TelemetryGroup("ble", "BLE / stream health", ble),
        ),
    )


def _parse_scalar(text: str) -> Scalar:
    if _INTEGER_PATTERN.fullmatch(text):
        return int(text, 10)
    if _FLOAT_PATTERN.fullmatch(text):
        return float(text)
    return text


def parse_telemetry_line(line: str) -> TelemetrySample | None:
    """Parse a complete fixed-line BARO record.

    Malformed or duplicate fields are rejected so the GUI never presents an
    ambiguous value as current telemetry.
    """

    stripped = line.strip()
    parts = stripped.split()
    if not parts or parts[0] != TELEMETRY_PREFIX:
        return None

    fields: dict[str, Scalar] = {}
    for token in parts[1:]:
        name, separator, value = token.partition("=")
        if (
            separator != "="
            or not name
            or not value
            or name in fields
            or not _PARAMETER_NAME_PATTERN.fullmatch(name)
        ):
            return None
        fields[name] = _parse_scalar(value)

    if not fields:
        return None
    return TelemetrySample(fields=fields, raw_line=stripped)


def parse_parameter_line(line: str) -> tuple[str, str] | None:
    """Parse one PARAM LIST/GET response line (`name=value`)."""

    stripped = line.strip()
    if not stripped or " " in stripped or "\t" in stripped:
        return None
    name, separator, value = stripped.partition("=")
    if (
        separator != "="
        or not value
        or not _PARAMETER_NAME_PATTERN.fullmatch(name)
    ):
        return None
    return name, value


def format_command(command: str) -> bytes:
    """Validate and encode one firmware console command with CRLF."""

    normalized = command.strip()
    if not normalized or "\r" in normalized or "\n" in normalized:
        raise ValueError("command must contain exactly one non-empty line")
    try:
        encoded = normalized.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError("command must contain ASCII characters only") from exc
    if len(encoded) > MAX_COMMAND_BYTES:
        raise ValueError(
            f"command exceeds {MAX_COMMAND_BYTES} ASCII bytes"
        )
    return encoded + b"\r\n"
