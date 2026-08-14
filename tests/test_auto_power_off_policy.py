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
CONFIG_SOURCE = (ROOT / "SRC/domain/app_config.c").read_text(
    encoding="utf-8"
)
CONFIG_HEADER = (ROOT / "SRC/platform/config_storage.h").read_text(
    encoding="utf-8"
)
SW_SPEC = (ROOT / "DOC/SW_spec.md").read_text(encoding="utf-8")
PARAMETER_SPEC = (ROOT / "DOC/setting_json.md").read_text(
    encoding="utf-8"
)


class AutoPowerOffPolicyTests(unittest.TestCase):
    def test_fixed_altitude_tolerance_and_public_setting(self) -> None:
        self.assertIn(
            "#define AUTO_POWER_OFF_ALTITUDE_TOLERANCE_M 5.0f",
            POLICY_HEADER,
        )
        self.assertIn(
            "PARAM_UINT(auto_power_off_minutes, 60, 0.0, 1440.0, "
            "APP_PARAMETER_SCOPE_SHARED)",
            CONFIG_SOURCE,
        )
        self.assertIn("#define CONFIG_FORMAT_VERSION 1", CONFIG_HEADER)

    def test_system_task_uses_raw_shared_vario_and_normal_shutdown(self) -> None:
        system_task = TASK_SOURCE[
            TASK_SOURCE.index("void app_system_worker_task(void *context)") :
            TASK_SOURCE.index("static bool console_writef(")
        ]
        self.assertIn("app_resources_copy_vario(", system_task)
        self.assertIn("system_auto_power_off_vario.estimate_valid", system_task)
        self.assertIn("auto_power_off_update(", system_task)
        self.assertIn("request_power_off(&snapshot);", system_task)
        self.assertNotIn("app_resources_apply_debug_vario", system_task)

    def test_reset_conditions_are_explicit(self) -> None:
        for expression in (
            "configured_minutes == 0U",
            "external_power_present",
            "!altitude_valid",
            "!isfinite(altitude_m)",
            "now_us < state->last_update_us",
        ):
            self.assertIn(expression, POLICY_SOURCE)
        self.assertIn("auto_power_off_config_revision", TASK_SOURCE)

    def test_documentation_matches_runtime_contract(self) -> None:
        for text in (
            "`auto_power_off_minutes`",
            "0～1440 min",
            "期間内変動幅が10 m以下",
            "デバッグ高度",
        ):
            self.assertIn(text, PARAMETER_SPEC)
        self.assertIn("高度変動幅10 m以下", SW_SPEC)


if __name__ == "__main__":
    unittest.main()
