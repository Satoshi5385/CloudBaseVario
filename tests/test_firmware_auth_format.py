from __future__ import annotations

import hashlib
from pathlib import Path
import tempfile
import unittest

from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec, utils

from tools.firmware_signing.firmware_auth_format import (
    CHIP_ID_ESP32S3,
    HEADER,
    HEADER_SIZE,
    MAX_PAYLOAD_SIZE,
    PROJECT_NAME,
    RECORD_SIZE,
    SIGNATURE_DOMAIN,
)
from tools.firmware_signing.sign_update import create_signed_artifacts


def raw_application(project: str = PROJECT_NAME) -> bytes:
    image = bytearray(24 + 8 + 48 + 32)
    image[0] = 0xE9
    image[12:14] = CHIP_ID_ESP32S3.to_bytes(2, "little")
    offset = 24 + 8 + 48
    image[offset : offset + len(project)] = project.encode("ascii")
    return bytes(image)


class FirmwareAuthFormatTests(unittest.TestCase):
    def test_signer_creates_single_container_and_matching_record(self) -> None:
        private_key = ec.generate_private_key(ec.SECP256R1())
        payload = raw_application()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            input_path = root / "app.bin"
            key_path = root / "owner.pem"
            update_path = root / "UPDATE.BIN"
            record_path = root / "FIRMWARE.AUTH"
            input_path.write_bytes(payload)
            key_path.write_bytes(
                private_key.private_bytes(
                    serialization.Encoding.PEM,
                    serialization.PrivateFormat.PKCS8,
                    serialization.NoEncryption(),
                )
            )
            create_signed_artifacts(
                input_path, key_path, update_path, record_path, "owner-key-2026!!"
            )

            update = update_path.read_bytes()
            record = record_path.read_bytes()
            fields = HEADER.unpack(update[:HEADER_SIZE])
            self.assertEqual(len(record), RECORD_SIZE)
            self.assertEqual(record[:HEADER_SIZE], update[:HEADER_SIZE])
            self.assertEqual(update[HEADER_SIZE:], payload)
            self.assertEqual(fields[0], b"CBVOTA01")
            self.assertEqual(fields[4], CHIP_ID_ESP32S3)
            self.assertEqual(fields[6], len(payload))
            self.assertEqual(fields[7], hashlib.sha256(payload).digest())

            der_signature = utils.encode_dss_signature(
                int.from_bytes(fields[9][:32], "big"),
                int.from_bytes(fields[9][32:], "big"),
            )
            private_key.public_key().verify(
                der_signature,
                SIGNATURE_DOMAIN + update[: HEADER_SIZE - 64],
                ec.ECDSA(hashes.SHA256()),
            )

    def test_signer_rejects_wrong_target_and_maximum_overflow(self) -> None:
        private_key = ec.generate_private_key(ec.SECP256R1())
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            key_path = root / "owner.pem"
            key_path.write_bytes(
                private_key.private_bytes(
                    serialization.Encoding.PEM,
                    serialization.PrivateFormat.PKCS8,
                    serialization.NoEncryption(),
                )
            )
            wrong_target = root / "wrong.bin"
            wrong_target.write_bytes(raw_application("OtherProject"))
            with self.assertRaises(ValueError):
                create_signed_artifacts(
                    wrong_target,
                    key_path,
                    root / "UPDATE.BIN",
                    root / "FIRMWARE.AUTH",
                    "owner-key-2026!!",
                )
            self.assertEqual(MAX_PAYLOAD_SIZE, 0x37F000)


if __name__ == "__main__":
    unittest.main()
