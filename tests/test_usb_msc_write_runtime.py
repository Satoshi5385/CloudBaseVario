"""Execute production MSC functions with deterministic task/USB interleavings.

The adapter's real C functions and reset bridge are compiled, not reimplemented
in Python. Only the RTOS, medium and TinyUSB protocol machinery are fakes.
The firmware build separately compiles the bridge against pinned TinyUSB.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "components/esp_tinyusb"


def function(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end] + "\n"


class MscWriteRuntimeTests(unittest.TestCase):
    def test_write_reservation_and_reset_interleavings(self):
        source = (COMPONENT / "tinyusb_msc.c").read_text()
        signatures = [
            "static inline esp_err_t msc_storage_write_sector_deferred(",
            "static void msc_storage_complete_write(",
            "static void msc_storage_process_write(",
            "static void msc_storage_write_worker_task(",
            "esp_err_t tinyusb_msc_stop_host_io(",
        ]
        with tempfile.TemporaryDirectory() as temp:
            temp = Path(temp)
            (temp / "class/msc").mkdir(parents=True)
            (temp / "class/msc/msc_device.h").write_text("")
            (temp / "production_functions.inc").write_text(
                "\n".join(function(source, sig) for sig in signatures)
            )
            # Types stay aligned with the production adapter.
            start = source.index("typedef struct {")
            end = source.index("static portMUX_TYPE", start)
            (temp / "production_types.inc").write_text(source[start:end])
            bridge = (COMPONENT / "msc_device_bridge.c.in").read_text()
            bridge = bridge.replace(
                "@tinyusb_msc_source@",
                (ROOT / "tests/host_stubs/msc_core_fake.inc").as_posix(),
            )
            (temp / "msc_device_bridge.inc").write_text(bridge)
            executable = temp / "test_msc"
            subprocess.run([
                os.environ.get("CC", "cc"), "-std=c11", "-Wall", "-Wextra",
                "-Werror", "-pedantic", "-I", str(temp), "-I",
                str(COMPONENT / "include_private"),
                str(ROOT / "tests/test_usb_msc_write_runtime.c"),
                "-o", str(executable),
            ], check=True, capture_output=True, text=True)
            subprocess.run([str(executable)], check=True,
                           capture_output=True, text=True)
