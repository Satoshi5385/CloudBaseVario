import json
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PREFERENCES_SOURCE = (ROOT / "SRC/platform/switch_preferences.c").read_text(
    encoding="utf-8"
)
PREFERENCES_HEADER = (ROOT / "SRC/platform/switch_preferences.h").read_text(
    encoding="utf-8"
)
MAIN_SOURCE = (ROOT / "SRC/app/startup.c").read_text(encoding="utf-8")
TASK_SOURCE = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
CONFIG_SOURCE = (ROOT / "SRC/domain/app_config.c").read_text(encoding="utf-8")
STORAGE_SOURCE = (ROOT / "SRC/platform/config_storage.c").read_text(
    encoding="utf-8"
)
PARAMETER_SPEC = (ROOT / "DOC/setting_json.md").read_text(encoding="utf-8")


class SwitchPreferencesPolicyTests(unittest.TestCase):
    def test_nvs_blob_contract_and_defaults(self) -> None:
        self.assertIn('#define SWITCH_PREFERENCES_NAMESPACE "switch_pref"', PREFERENCES_SOURCE)
        self.assertIn('#define SWITCH_PREFERENCES_KEY "state"', PREFERENCES_SOURCE)
        self.assertIn("#define SWITCH_PREFERENCES_PAYLOAD_SIZE 5U", PREFERENCES_SOURCE)
        self.assertIn("#define SWITCH_PREFERENCES_FORMAT_VERSION UINT8_C(2)", PREFERENCES_SOURCE)
        self.assertIn("#define SWITCH_PREFERENCES_LEGACY_PAYLOAD_SIZE 4U", PREFERENCES_SOURCE)
        self.assertRegex(
            PREFERENCES_SOURCE,
            re.compile(
                r"preferences->volume_level = AUDIO_VOLUME_SMALL;\s*"
                r"preferences->sink_enabled = true;\s*"
                r"preferences->parameter_number = APP_CONFIG_PROFILE_MIN_NUMBER;",
                re.DOTALL,
            ),
        )

    def test_load_checks_legacy_and_current_blob_fields(self) -> None:
        self.assertIn("payload_size != sizeof(payload)", PREFERENCES_SOURCE)
        self.assertIn(
            "payload[0] != SWITCH_PREFERENCES_FORMAT_VERSION",
            PREFERENCES_SOURCE,
        )
        self.assertIn("payload[1] > (uint8_t) AUDIO_VOLUME_MUTE", PREFERENCES_SOURCE)
        self.assertIn("payload[2] > 1U", PREFERENCES_SOURCE)
        self.assertIn("payload[3] < APP_CONFIG_PROFILE_MIN_NUMBER", PREFERENCES_SOURCE)
        self.assertIn("payload[4] != 0U", PREFERENCES_SOURCE)
        self.assertIn("SWITCH_PREFERENCES_LOAD_LEGACY", PREFERENCES_SOURCE)
        for result in (
            "SWITCH_PREFERENCES_LOAD_NOT_FOUND",
            "SWITCH_PREFERENCES_LOAD_INVALID_SIZE",
            "SWITCH_PREFERENCES_LOAD_UNSUPPORTED_VERSION",
            "SWITCH_PREFERENCES_LOAD_INVALID_VALUE",
            "SWITCH_PREFERENCES_LOAD_IO_ERROR",
        ):
            self.assertIn(result, PREFERENCES_SOURCE)
        self.assertIn("load_error_count", PREFERENCES_HEADER)

    def test_save_is_one_blob_commit_and_clear_is_key_scoped(self) -> None:
        save = PREFERENCES_SOURCE[
            PREFERENCES_SOURCE.index("esp_err_t switch_preferences_save(") :
            PREFERENCES_SOURCE.index("esp_err_t switch_preferences_clear(")
        ]
        clear = PREFERENCES_SOURCE[
            PREFERENCES_SOURCE.index("esp_err_t switch_preferences_clear(") :
            PREFERENCES_SOURCE.index("void switch_preferences_get_diagnostics(")
        ]
        self.assertEqual(save.count("nvs_set_blob("), 1)
        self.assertEqual(save.count("nvs_commit("), 1)
        self.assertIn("nvs_erase_key(handle, SWITCH_PREFERENCES_KEY)", clear)
        self.assertNotIn("nvs_flash_erase", clear)

    def test_boot_loads_or_clears_only_switch_preferences(self) -> None:
        self.assertIn("switch_preferences_set_defaults(&switch_preferences)", MAIN_SOURCE)
        self.assertIn("nvs_ready && config_format_requested", MAIN_SOURCE)
        self.assertIn("switch_preferences_clear()", MAIN_SOURCE)
        self.assertIn(
            "switch_preferences_load(\n                &startup_preparation_result.switch_preferences)",
            MAIN_SOURCE,
        )
        self.assertIn(
            "app_tasks_set_switch_preferences(&switch_preferences,",
            MAIN_SOURCE,
        )

    def test_switches_update_dirty_without_writing_flash(self) -> None:
        system_task = TASK_SOURCE[
            TASK_SOURCE.index("void app_system_worker_task(void *context)") :
            TASK_SOURCE.index("static bool console_writef(")
        ]
        self.assertIn("update_switch_preferences_dirty(", system_task)
        self.assertIn("snapshot->volume_level != baseline->volume_level", TASK_SOURCE)
        self.assertIn(
            "snapshot->sink_enabled_override != baseline->sink_enabled",
            TASK_SOURCE,
        )
        self.assertIn(
            "snapshot->parameter_number != baseline->parameter_number",
            TASK_SOURCE,
        )
        self.assertNotIn("switch_preferences_save(", system_task)

    def test_shutdown_waits_for_audio_then_saves_final_snapshot(self) -> None:
        console_task = TASK_SOURCE[
            TASK_SOURCE.index("void app_console_worker_task(void *context)") :
            TASK_SOURCE.index("void app_workers_run_fatal_fallback(")
        ]
        quiesced = console_task.index("APP_EVENT_AUDIO_QUIESCED")
        copy = console_task.index("app_resources_copy_system(&final_system)")
        save = console_task.index("switch_preferences_save(&preferences)")
        ack = console_task.index("acknowledge_and_delete(APP_EVENT_CONSOLE_ACK)")
        self.assertLess(quiesced, copy)
        self.assertLess(copy, save)
        self.assertLess(save, ack)
        self.assertIn("final_system.switch_preferences_dirty", console_task)

        shutdown = TASK_SOURCE[
            TASK_SOURCE.index("static void request_power_off(") :
            TASK_SOURCE.index("void app_system_worker_task(")
        ]
        self.assertLess(
            shutdown.index("app_resources_publish_system(snapshot)"),
            shutdown.index("APP_EVENT_STOP_REQUEST"),
        )
        self.assertIn("SHUTDOWN_DEADLINE_MS", shutdown)

    def test_removed_keys_are_not_public_or_migrated(self) -> None:
        table = CONFIG_SOURCE.split("parameter_table[] = {", 1)[1].split("};", 1)[0]
        for name in ("audio_enabled", "audio_amp_mode", "sink_enabled"):
            self.assertNotIn(name, table)
            self.assertNotIn(f'"{name}"', STORAGE_SOURCE)

        example = PARAMETER_SPEC.split("```json", 1)[1].split("```", 1)[0]
        document = json.loads(example)
        shared_parameters = document["mc_parameters"]
        profile_parameters = document["vario_parameter_sets"][0]["parameters"]
        self.assertEqual(len(shared_parameters), 10)
        self.assertEqual(len(profile_parameters), 22)
        for name in ("audio_enabled", "audio_amp_mode", "sink_enabled"):
            self.assertNotIn(name, shared_parameters)
            self.assertNotIn(name, profile_parameters)


if __name__ == "__main__":
    unittest.main()
