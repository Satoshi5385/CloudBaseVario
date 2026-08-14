import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TASK_SOURCE = "\n".join(
    (ROOT / path).read_text(encoding="utf-8")
    for path in ("SRC/app/app_workers.c", "SRC/domain/system_policy.c")
)
POLICY_SOURCE = (ROOT / "SRC/domain/system_policy.c").read_text(
    encoding="utf-8"
)
POLICY_HEADER = (ROOT / "SRC/domain/system_policy.h").read_text(
    encoding="utf-8"
)
RESOURCE_SOURCE = (ROOT / "SRC/app/app_resources.c").read_text(
    encoding="utf-8"
)
RESOURCE_HEADER = (ROOT / "SRC/app/app_resources.h").read_text(
    encoding="utf-8"
)
SW_SPEC = (ROOT / "DOC/SW_spec.md").read_text(encoding="utf-8")
PARAMETER_SPEC = (ROOT / "DOC/setting_json.md").read_text(
    encoding="utf-8"
)


class SystemSoundPolicyTests(unittest.TestCase):
    def test_switch_debounce_waits_thirty_ms_after_the_edge_sample(self) -> None:
        self.assertIn("SYSTEM_POLICY_SAMPLE_PERIOD_MS UINT32_C(10)", POLICY_HEADER)
        self.assertIn("SYSTEM_POLICY_SWITCH_DEBOUNCE_MS UINT32_C(30)", POLICY_HEADER)
        self.assertIn("button->stable_time_ms = 0U;", POLICY_SOURCE)
        self.assertNotIn(
            "button->stable_time_ms = SYSTEM_POLICY_SAMPLE_PERIOD_MS;",
            POLICY_SOURCE,
        )
        self.assertIn(
            "system_policy_increment_ms(\n            button->stable_time_ms)",
            POLICY_SOURCE,
        )

    def test_button_sound_contract(self) -> None:
        self.assertIn("#define BUTTON_SOUND_HZ UINT32_C(1000)", TASK_SOURCE)
        self.assertIn("#define BUTTON_SOUND_MS UINT32_C(80)", TASK_SOURCE)
        self.assertIn(
            "#define SYSTEM_SOUND_DUTY_PERCENT UINT32_C(50)", TASK_SOURCE
        )
        self.assertIn("{BUTTON_SOUND_HZ, BUTTON_SOUND_MS}", TASK_SOURCE)

    def test_sink_status_sounds_use_two_tone_system_sound_steps(self) -> None:
        self.assertIn("AUDIO_NOTIFICATION_SINK_ENABLED", TASK_SOURCE)
        self.assertIn("AUDIO_NOTIFICATION_SINK_DISABLED", TASK_SOURCE)
        self.assertRegex(
            TASK_SOURCE,
            re.compile(
                r"sink_enabled_sound_steps\[\]\s*=\s*\{\s*"
                r"\{SYSTEM_SOUND_LOW_HZ, SYSTEM_SOUND_LOW_MS\},\s*"
                r"\{0U, SYSTEM_SOUND_SILENCE_MS\},\s*"
                r"\{SYSTEM_SOUND_HIGH_HZ, SYSTEM_SOUND_HIGH_MS\},\s*"
                r"\};",
                re.DOTALL,
            ),
        )
        self.assertRegex(
            TASK_SOURCE,
            re.compile(
                r"sink_disabled_sound_steps\[\]\s*=\s*\{\s*"
                r"\{SYSTEM_SOUND_HIGH_HZ, SYSTEM_SOUND_HIGH_MS\},\s*"
                r"\{0U, SYSTEM_SOUND_SILENCE_MS\},\s*"
                r"\{SYSTEM_SOUND_LOW_HZ, SYSTEM_SOUND_LOW_MS\},\s*"
                r"\};",
                re.DOTALL,
            ),
        )

    def test_notifications_use_a_nonblocking_latest_value_queue(self) -> None:
        self.assertIn(
            "#define BUTTON_SOUND_QUEUE_LENGTH ((UBaseType_t) 1U)",
            RESOURCE_SOURCE,
        )
        self.assertIn(
            "sizeof(audio_notification_request_t)", RESOURCE_SOURCE
        )
        self.assertIn(
            "QueueHandle_t app_resources_button_sound_queue(void);",
            RESOURCE_HEADER,
        )
        self.assertIn("xQueueOverwrite(queue, &request)", TASK_SOURCE)
        self.assertIn("BUTTON_SOUND_SILENCE_MS", TASK_SOURCE)

    def test_selected_volume_prefers_runtime_override(self) -> None:
        function = TASK_SOURCE[
            TASK_SOURCE.index("static audio_volume_level_t selected_volume_level(") :
            TASK_SOURCE.index("static uint32_t volume_amplifier_mode(")
        ]
        override = function.index("system->volume_override_active")
        config = function.index("app_resources_copy_config(&config)")
        self.assertLess(override, config)
        self.assertIn("return system->volume_level;", function)
        self.assertIn("return config_volume_level(&config);", function)

    def test_amplifier_mapping_and_mute_contract(self) -> None:
        function = TASK_SOURCE[
            TASK_SOURCE.index("static uint32_t volume_amplifier_mode(") :
            TASK_SOURCE.index("static void request_button_sound(")
        ]
        self.assertIn(
            "return (uint32_t) volume_level + UINT32_C(1);", function
        )
        self.assertIn("case AUDIO_VOLUME_MUTE:", function)
        self.assertIn("return 0U;", function)
        self.assertIn("volume_level == AUDIO_VOLUME_MUTE || repeat_count == 0U", TASK_SOURCE)

    def test_sw1_short_release_changes_volume_but_long_hold_does_not(self) -> None:
        self.assertIn("actions->advance_volume = true;", POLICY_SOURCE)
        self.assertIn("!state->sw1_power_off_issued", POLICY_SOURCE)
        self.assertIn("SYSTEM_POLICY_POWER_OFF_HOLD_MS", POLICY_SOURCE)
        self.assertIn("if (switch_actions.advance_volume)", TASK_SOURCE)
        self.assertIn("next_volume_level(snapshot.volume_level)", TASK_SOURCE)

    def test_sw2_toggles_sink_and_requests_the_resulting_status_sound(self) -> None:
        self.assertIn("actions->toggle_sink = true;", POLICY_SOURCE)
        start = TASK_SOURCE.index("if (switch_actions.toggle_sink)")
        end = TASK_SOURCE.index("if (switch_actions.select_next_profile", start)
        block = TASK_SOURCE[start:end]
        toggle = block.index("toggle_sink_override(&snapshot);")
        request = block.index("request_sink_status_sound(")
        self.assertLess(toggle, request)
        self.assertIn("snapshot.sink_enabled_override", block)

    def test_sw3_selects_profile_and_preserves_calibration_hold(self) -> None:
        self.assertGreaterEqual(
            POLICY_SOURCE.count("actions->select_next_profile = true;"), 2
        )
        self.assertIn("SYSTEM_POLICY_IMU_SKIP_HOLD_MS", POLICY_SOURCE)
        self.assertIn("if (switch_actions.request_calibration_skip)", TASK_SOURCE)
        self.assertIn("APP_EVENT_IMU_ACCEL_CALIBRATION_SKIP_REQUEST", TASK_SOURCE)

    def test_audio_task_prioritizes_system_and_button_sounds(self) -> None:
        audio_task = TASK_SOURCE[
            TASK_SOURCE.index("void app_audio_worker_task(void *context)") :
            TASK_SOURCE.index("static uint32_t shutdown_remaining_ms")
        ]
        stop = audio_task.index("if (app_stop_requested())")
        fatal = audio_task.index("if (app_fatal_state())")
        button = audio_task.index("xQueueReceive(button_sound_queue", fatal)
        vario = audio_task.index("vario_audio_step(", button)
        self.assertLess(stop, fatal)
        self.assertLess(fatal, button)
        self.assertLess(button, vario)
        self.assertIn("play_latest_button_notification(", audio_task)
        self.assertIn("APP_EVENT_FATAL_STATE", TASK_SOURCE)
        self.assertIn("vario_audio_reset(&audio_state);", audio_task)
        self.assertNotIn("APP_EVENT_STARTUP_SOUND_", TASK_SOURCE)
        self.assertNotIn("APP_EVENT_STARTUP_SOUND_", RESOURCE_HEADER)

    def test_power_sounds_use_current_volume_without_fixed_gain(self) -> None:
        self.assertNotIn("SYSTEM_SOUND_AMPLIFIER_MODE", TASK_SOURCE)
        self.assertIn(
            "esp_err_t app_workers_play_startup_sound(", TASK_SOURCE
        )
        self.assertIn("volume_amplifier_mode(volume_level)", TASK_SOURCE)
        self.assertIn("selected_volume_level(&system)", TASK_SOURCE)
        self.assertIn("if (amplifier_mode == 0U)", TASK_SOURCE)

    def test_documentation_matches_runtime_behavior(self) -> None:
        self.assertIn("1000 Hz、80 ms、デューティ50 %", SW_SPEC)
        self.assertIn(
            "ONでは700 Hzを180 ms鳴動、80 ms無音、1200 Hzを120 ms鳴動",
            SW_SPEC,
        )
        self.assertIn(
            "OFFでは1200 Hzを120 ms鳴動、80 ms無音、700 Hzを180 ms鳴動",
            SW_SPEC,
        )
        self.assertIn("深さ1のlatest-value queueへ非ブロッキング", SW_SPEC)
        self.assertIn("変更後の音量", SW_SPEC)
        self.assertIn("消音時は鳴りません", PARAMETER_SPEC)
        self.assertIn("SW2はONを低音→高音、OFFを高音→低音で通知します", PARAMETER_SPEC)
        self.assertIn("SW1の音量", PARAMETER_SPEC)


if __name__ == "__main__":
    unittest.main()
