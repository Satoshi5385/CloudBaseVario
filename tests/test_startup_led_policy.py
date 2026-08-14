import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
POLICY_HEADER = (ROOT / "SRC/domain/system_policy.h").read_text(encoding="utf-8")
POLICY_SOURCE = (ROOT / "SRC/domain/system_policy.c").read_text(encoding="utf-8")
WORKER_SOURCE = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
STARTUP_SOURCE = (ROOT / "SRC/app/startup.c").read_text(encoding="utf-8")
SW_SPEC = (ROOT / "DOC/SW_spec.md").read_text(encoding="utf-8")


class StartupLedPolicyTests(unittest.TestCase):
    """Keep only SDK wiring checks here; host C tests own LED behavior."""

    def test_led_policy_is_public_sdk_independent_domain_api(self):
        self.assertIn("typedef struct {", POLICY_HEADER)
        self.assertIn("system_led_policy_input_t", POLICY_HEADER)
        self.assertIn("system_led_policy_output_t", POLICY_HEADER)
        self.assertIn("void system_policy_select_leds(", POLICY_HEADER)

    def test_worker_maps_runtime_snapshot_to_policy(self):
        for field in (
            ".bmp581_startup_complete",
            ".bmp581_recovering",
            ".imu_calibrating",
            ".imu_degraded",
            ".external_power_present",
            ".battery_valid",
            ".ble_notify_active",
        ):
            self.assertIn(field, WORKER_SOURCE)

    def test_worker_applies_only_policy_output_to_board(self):
        self.assertIn("system_policy_select_leds(&input, &output);", WORKER_SOURCE)
        self.assertIn("output.green_brightness_percent, output.yellow_on", WORKER_SOURCE)

    def test_low_battery_threshold_has_one_policy_definition(self):
        self.assertIn(
            "#define SYSTEM_POLICY_LOW_BATTERY_THRESHOLD_V 3.3f",
            POLICY_HEADER,
        )
        self.assertNotIn("#define LOW_BATTERY_THRESHOLD_V", WORKER_SOURCE)

    def test_led_priority_inputs_are_explicit(self):
        for field in (
            "bool fatal;",
            "bool fatal_bmp581;",
            "bool estimator_warming_up;",
            "bool bmp581_recovering;",
            "bool imu_calibrating;",
            "bool imu_degraded;",
        ):
            self.assertIn(field, POLICY_HEADER)

    def test_power_on_fade_uses_shared_policy(self):
        self.assertIn("system_policy_power_on_brightness(", POLICY_HEADER)
        self.assertIn("system_policy_power_on_brightness(", STARTUP_SOURCE)
        self.assertIn("POWER_ON_HOLD_MS", STARTUP_SOURCE)

    def test_policy_owns_led_math(self):
        self.assertIn("static uint32_t led_firefly_brightness", POLICY_SOURCE)
        self.assertIn("system_policy_power_off_brightness", POLICY_SOURCE)
        self.assertNotIn("led_firefly_brightness_percent", WORKER_SOURCE)

    def test_specification_defines_led_priority(self):
        self.assertIn("表の上から順に優先度が高い", SW_SPEC)
        self.assertIn("低電池残量", SW_SPEC)
        self.assertIn("BLE", SW_SPEC)


if __name__ == "__main__":
    unittest.main()
