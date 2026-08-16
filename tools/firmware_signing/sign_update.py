"""Create an owner-signed single-file UPDATE.BIN and factory auth record."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

try:
    from tools.firmware_signing.firmware_auth_format import (
        CHIP_ID_ESP32S3,
        PROJECT_NAME,
        SIGNATURE_DOMAIN,
        build_header,
        record_from_header,
        unsigned_header,
    )
except ModuleNotFoundError:
    from firmware_auth_format import (
        CHIP_ID_ESP32S3,
        PROJECT_NAME,
        SIGNATURE_DOMAIN,
        build_header,
        record_from_header,
        unsigned_header,
    )


def _crypto():
    try:
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import ec, utils
    except ImportError as error:
        raise SystemExit(
            "cryptography is required; install tools/firmware_signing/requirements-firmware-signing.txt"
        ) from error
    return hashes, serialization, ec, utils


def _verify_raw_payload(payload: bytes, project: str, chip_id: int) -> None:
    descriptor_offset = 24 + 8
    project_offset = descriptor_offset + 48
    if len(payload) < project_offset + 32 or payload[0] != 0xE9:
        raise ValueError("input is not a complete ESP application image")
    actual_chip_id = int.from_bytes(payload[12:14], "little")
    if actual_chip_id != chip_id:
        raise ValueError(f"input chip ID {actual_chip_id} does not match {chip_id}")
    embedded_project = payload[project_offset : project_offset + 32].split(b"\0", 1)[0]
    if embedded_project != project.encode("ascii"):
        raise ValueError(
            f"input project name {embedded_project!r} does not match {project!r}"
        )


def _raw_signature(private_key: object, message: bytes) -> bytes:
    hashes, _, ec, utils = _crypto()
    der = private_key.sign(message, ec.ECDSA(hashes.SHA256()))
    r, s = utils.decode_dss_signature(der)
    return r.to_bytes(32, "big") + s.to_bytes(32, "big")


def create_signed_artifacts(
    input_path: Path,
    private_key_path: Path,
    output_path: Path,
    record_path: Path,
    key_id: str,
    project: str = PROJECT_NAME,
    chip_id: int = CHIP_ID_ESP32S3,
) -> None:
    _, serialization, ec, _ = _crypto()
    payload = input_path.read_bytes()
    _verify_raw_payload(payload, project, chip_id)
    private_key = serialization.load_pem_private_key(
        private_key_path.read_bytes(), password=None
    )
    if not isinstance(private_key, ec.EllipticCurvePrivateKey) or not isinstance(
        private_key.curve, ec.SECP256R1
    ):
        raise ValueError("private key must be ECDSA P-256 PEM")
    unsigned = unsigned_header(project, chip_id, payload, key_id)
    header = build_header(unsigned, _raw_signature(private_key, SIGNATURE_DOMAIN + unsigned))
    output_path.write_bytes(header + payload)
    record_path.write_bytes(record_from_header(header))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="raw application bin")
    parser.add_argument("--private-key", type=Path, required=True, help="owner P-256 PEM")
    parser.add_argument("--output", type=Path, required=True, help="signed UPDATE.BIN")
    parser.add_argument(
        "--auth-record", type=Path, required=True, help="4 KiB factory auth record"
    )
    parser.add_argument("--key-id", required=True, help="exactly 16 ASCII bytes")
    args = parser.parse_args()
    try:
        create_signed_artifacts(
            args.input,
            args.private_key,
            args.output,
            args.auth_record,
            args.key_id,
        )
    except (OSError, ValueError) as error:
        print(f"sign_update: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
