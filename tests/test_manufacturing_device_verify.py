from types import SimpleNamespace
import unittest
from unittest import mock

from manufacturing_tools.device_verify import (
    DeviceVerificationError,
    TINYUSB_PID,
    TINYUSB_VID,
    wait_and_verify_device,
)
from manufacturing_tools.serial_model import parse_product_serial


PRODUCT = parse_product_serial("CBV_A0_73l0j_0009")
MAC = "001122334455"
ARTIFACTS = SimpleNamespace(
    project="CloudBaseVario-Aohazuku",
    version="test-version",
)


def board_response(serial: str) -> list[bytes]:
    return [
        (
            "BOARD status=VALID schema=1 id=0x0100 code=A0 "
            f"model=Aohazuku-Rev0 serial={serial} mac={MAC} "
            "firmware_project=CloudBaseVario-Aohazuku "
            "firmware_version=test-version\r\n"
        ).encode("ascii"),
        b"OK\r\n",
    ]


DIAG_RESPONSE = [
    b"IMU online=1 configured=1 address=0x18 who_am_i=0x6a\r\n",
    (
        b"USB msc_driver=1 msc_media=1 storage=1 owner=APP "
        b"storage_error=ESP_OK\r\n"
    ),
    b"OK\r\n",
]


class FakeConnection:
    def __init__(self, serial_value: str):
        self.serial_value = serial_value
        self.lines: list[bytes] = []
        self.dtr = False

    def __enter__(self):
        return self

    def __exit__(self, _kind, _value, _traceback):
        return False

    def reset_input_buffer(self):
        self.lines.clear()

    def write(self, command: bytes):
        if command == b"BOARD INFO\r\n":
            self.lines.extend(board_response(self.serial_value))
        elif command == b"DIAG STATUS\r\n":
            self.lines.extend(DIAG_RESPONSE)
        return len(command)

    def flush(self):
        return None

    def readline(self):
        return self.lines.pop(0) if self.lines else b""


class FakeSerialModule:
    def __init__(self, serial_value: str, error: Exception | None = None):
        self.serial_value = serial_value
        self.error = error

    def Serial(self, *_args, **_kwargs):
        if self.error is not None:
            raise self.error
        return FakeConnection(self.serial_value)


class FakeListPorts:
    def __init__(self, ports):
        self.ports = ports

    def comports(self):
        return self.ports


def port(serial_number="CBV_A0_73L0J_0009"):
    return SimpleNamespace(
        device="COM10",
        vid=TINYUSB_VID,
        pid=TINYUSB_PID,
        serial_number=serial_number,
    )


class ManufacturingDeviceVerifyTests(unittest.TestCase):
    def test_windows_uppercase_usb_serial_uses_exact_board_info(self) -> None:
        messages = []
        modules = (
            FakeSerialModule(PRODUCT.value),
            FakeListPorts([port()]),
        )
        with mock.patch(
            "manufacturing_tools.device_verify._serial_modules",
            return_value=modules,
        ):
            result = wait_and_verify_device(
                PRODUCT,
                MAC,
                ARTIFACTS,
                timeout_s=1.0,
                progress=lambda stage, message: messages.append((stage, message)),
            )
        self.assertEqual(result.port, "COM10")
        self.assertIn(f"serial={PRODUCT.value}", result.board_line)
        self.assertIn("storage=1", result.usb_line)
        self.assertTrue(any("CBV_A0_73L0J_0009" in item[1] for item in messages))

    def test_unavailable_config_fat_is_rejected(self) -> None:
        modules = (
            FakeSerialModule(PRODUCT.value),
            FakeListPorts([port()]),
        )
        failed_diag = [
            b"IMU online=1 configured=1 address=0x18 who_am_i=0x6a\r\n",
            (
                b"USB msc_driver=1 msc_media=0 storage=0 owner=UNAVAILABLE "
                b"storage_error=ESP_FAIL\r\n"
            ),
            b"OK\r\n",
        ]
        with mock.patch(
            "manufacturing_tools.device_verify._serial_modules",
            return_value=modules,
        ), mock.patch(f"{__name__}.DIAG_RESPONSE", failed_diag):
            with self.assertRaisesRegex(
                DeviceVerificationError, "USB msc_media mismatch"
            ):
                wait_and_verify_device(PRODUCT, MAC, ARTIFACTS, timeout_s=1.0)

    def test_other_board_is_not_accepted_case_insensitively(self) -> None:
        modules = (
            FakeSerialModule("CBV_A0_73L0J_0010"),
            FakeListPorts([port()]),
        )
        with mock.patch(
            "manufacturing_tools.device_verify._serial_modules",
            return_value=modules,
        ):
            with self.assertRaisesRegex(
                DeviceVerificationError, "BOARD serial mismatch"
            ):
                wait_and_verify_device(PRODUCT, MAC, ARTIFACTS, timeout_s=0.02)

    def test_busy_candidate_is_reported_at_timeout(self) -> None:
        modules = (
            FakeSerialModule(PRODUCT.value, PermissionError("access denied")),
            FakeListPorts([port()]),
        )
        with mock.patch(
            "manufacturing_tools.device_verify._serial_modules",
            return_value=modules,
        ):
            with self.assertRaisesRegex(
                DeviceVerificationError, "COM10: PermissionError: access denied"
            ):
                wait_and_verify_device(PRODUCT, MAC, ARTIFACTS, timeout_s=0.02)


if __name__ == "__main__":
    unittest.main()
