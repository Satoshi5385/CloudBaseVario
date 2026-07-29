"""Parsing helpers for the CloudBaseVario TinyUSB CDC protocol."""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import TypeAlias

Scalar: TypeAlias = int | float | str

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
