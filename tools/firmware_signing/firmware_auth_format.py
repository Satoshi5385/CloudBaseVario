"""Shared, byte-exact format for CloudBaseVario signed firmware."""

from __future__ import annotations

import hashlib
import struct


MAGIC = b"CBVOTA01"
FORMAT_VERSION = 1
PROJECT_NAME = "CloudBaseVario-Aohazuku"
CHIP_ID_ESP32S3 = 9
KEY_ID_LENGTH = 16
SHA256_LENGTH = 32
SIGNATURE_LENGTH = 64
HEADER = struct.Struct("<8sHH32sHHI32s16s64s")
HEADER_SIZE = HEADER.size
SIGNATURE_PREFIX_SIZE = HEADER_SIZE - SIGNATURE_LENGTH
RECORD_SIZE = 4096
MAX_PAYLOAD_SIZE = 0x37F000
SIGNATURE_DOMAIN = b"CloudBaseVario OTA authentication v1"


def fixed_ascii(value: str, size: int, label: str) -> bytes:
    encoded = value.encode("ascii")
    if not encoded or len(encoded) >= size or b"\0" in encoded:
        raise ValueError(f"{label} must be non-empty ASCII shorter than {size} bytes")
    return encoded + b"\0" * (size - len(encoded))


def fixed_key_id(value: str) -> bytes:
    encoded = value.encode("ascii")
    if len(encoded) != KEY_ID_LENGTH:
        raise ValueError(f"key ID must be exactly {KEY_ID_LENGTH} ASCII bytes")
    return encoded


def unsigned_header(project: str, chip_id: int, payload: bytes, key_id: str) -> bytes:
    if len(payload) == 0 or len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"payload must be 1..0x{MAX_PAYLOAD_SIZE:x} bytes")
    return HEADER.pack(
        MAGIC,
        FORMAT_VERSION,
        HEADER_SIZE,
        fixed_ascii(project, 32, "project name"),
        chip_id,
        0,
        len(payload),
        hashlib.sha256(payload).digest(),
        fixed_key_id(key_id),
        b"\0" * SIGNATURE_LENGTH,
    )[:SIGNATURE_PREFIX_SIZE]


def build_header(unsigned: bytes, signature: bytes) -> bytes:
    if len(unsigned) != SIGNATURE_PREFIX_SIZE or len(signature) != SIGNATURE_LENGTH:
        raise ValueError("invalid signed header fields")
    return unsigned + signature


def record_from_header(header: bytes) -> bytes:
    if len(header) != HEADER_SIZE:
        raise ValueError("invalid auth header")
    return header + b"\xff" * (RECORD_SIZE - len(header))
