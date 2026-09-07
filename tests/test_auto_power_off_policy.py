import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TASK_SOURCE = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
POLICY_HEADER = (ROOT / "SRC/domain/auto_power_off.h").read_text(
    encoding="utf-8"
)
POLICY_SOURCE = (ROOT / "SRC/domain/auto_power_off.c").read_text(
    encoding="utf-8"
)
FLIGHT_HEADER = (ROOT / "SRC/domain/flight_state.h").read_text(
    encoding="utf-8"
)
CONFIG_SOURCE = (ROOT / "SRC/domain/app_config.c").read_text(
    encoding="utf-8"
)
CONFIG_HEADER = (ROOT / "SRC/platform/config_storage.h").read_text(
    encoding="utf-8"
)
class AutoPowerOffPolicyTests(unittest.TestCase):
    def test_motion_thresholds_and_public_settings(self) -> None:
        self.assertIn(
            "#define FLIGHT_STATE_ALTITUDE_RANGE_M 10.0f",
            FLIGHT_HEADER,
        )
        self.assertIn(
            "PARAM_UINT(auto_power_off_minutes, 60, 0.0, 1440.0, "
            "APP_PARAMETER_SCOPE_SHARED)",
            CONFIG_SOURCE,
        )
        for expression in (
            "PARAM_FLOAT(flight_climb_rate_threshold_mps, 0.5f, 0.1, 5.0,",
            "PARAM_FLOAT(flight_gps_speed_threshold_kmh, 10.0f, 1.0, 100.0,",
            "PARAM_UINT(stationary_confirm_seconds, 60, 10.0, 600.0,",
        ):
            self.assertIn(expression, CONFIG_SOURCE)
        self.assertIn("#define CONFIG_FORMAT_VERSION 1", CONFIG_HEADER)

    def test_system_task_uses_raw_shared_vario_and_normal_shutdown(self) -> None:
        system_task = TASK_SOURCE[
            TASK_SOURCE.index("void app_system_worker_task(void *context)") :
            TASK_SOURCE.index("static bool console_writef(")
        ]
        self.assertIn("app_resources_copy_vario(", system_task)
        self.assertIn("flight_state_update(", system_task)
        self.assertIn("FLIGHT_STATE_STATIONARY", system_task)
        self.assertIn("auto_power_off_update(", system_task)
        self.assertIn("request_power_off(&snapshot);", system_task)
        self.assertNotIn("app_resources_apply_debug_vario", system_task)

    def test_reset_conditions_are_explicit(self) -> None:
        for expression in (
            "configured_minutes == 0U",
            "external_power_present",
            "!stationary",
            "stationary_since_us > now_us",
            "now_us < state->last_update_us",
        ):
            self.assertIn(expression, POLICY_SOURCE)
        self.assertIn("auto_power_off_config_revision", TASK_SOURCE)

if __name__ == "__main__":
    unittest.main()
