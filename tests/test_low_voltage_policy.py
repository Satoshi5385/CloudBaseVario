import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BATTERY_HEADER = (ROOT / "SRC/domain/battery_level.h").read_text(
    encoding="utf-8"
)
BATTERY_SOURCE = (ROOT / "SRC/domain/battery_level.c").read_text(
    encoding="utf-8"
)
STARTUP_SOURCE = (ROOT / "SRC/app/startup.c").read_text(encoding="utf-8")
WORKER_SOURCE = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
SW_SPEC = (ROOT / "DOC/SW_spec.md").read_text(encoding="utf-8")


class LowVoltagePolicyTests(unittest.TestCase):
    def test_thresholds_are_shared_domain_constants(self) -> None:
        self.assertIn("#define BATTERY_STARTUP_MINIMUM_V 3.2f", BATTERY_HEADER)
        self.assertIn("#define BATTERY_RUNTIME_SHUTDOWN_V 3.1f", BATTERY_HEADER)
        self.assertNotIn("#define BATTERY_STARTUP_MINIMUM_V", STARTUP_SOURCE)
        self.assertNotIn("#define BATTERY_RUNTIME_SHUTDOWN_V", WORKER_SOURCE)

    def test_policy_requires_valid_battery_power(self) -> None:
        self.assertIn("!external_power_present && battery_valid", BATTERY_SOURCE)
        self.assertIn("isfinite(battery_voltage_v)", BATTERY_SOURCE)
        self.assertIn("battery_voltage_v <= threshold_v", BATTERY_SOURCE)

    def test_startup_gate_uses_safe_stop_before_services(self) -> None:
        gate = STARTUP_SOURCE.index("battery_power_startup_blocked(")
        safe_stop = STARTUP_SOURCE.index("app_tasks_run_safe_stop();", gate)
        sound = STARTUP_SOURCE.index("app_tasks_play_startup_sound(")
        storage = STARTUP_SOURCE.index("usb_device_storage_init(")
        update = STARTUP_SOURCE.index("firmware_update_process_boot(")
        usb = STARTUP_SOURCE.index("usb_device_start();")
        self.assertLess(gate, safe_stop)
        self.assertLess(safe_stop, sound)
        self.assertLess(safe_stop, storage)
        self.assertLess(safe_stop, update)
        self.assertLess(safe_stop, usb)

    def test_runtime_shutdown_is_raw_and_latched_through_storage(self) -> None:
        self.assertRegex(
            WORKER_SOURCE,
            re.compile(
                r"battery_power_shutdown_required\(\s*"
                r"snapshot\.external_power_present, snapshot\.battery_valid,\s*"
                r"snapshot\.battery_voltage_v\)",
                re.DOTALL,
            ),
        )
        self.assertIn("low_battery_power_off_pending = true;", WORKER_SOURCE)
        self.assertIn("storage_power_off_pending = true;", WORKER_SOURCE)
        self.assertIn("request_power_off(&snapshot);", WORKER_SOURCE)
        self.assertNotIn(
            "snapshot.battery_display_voltage_v)) {\n"
            "                low_battery_power_off_pending",
            WORKER_SOURCE,
        )

    def test_specification_records_both_protection_boundaries(self) -> None:
        self.assertIn("3.2 V以下", SW_SPEC)
        self.assertIn("3.1 V以下", SW_SPEC)


if __name__ == "__main__":
    unittest.main()
