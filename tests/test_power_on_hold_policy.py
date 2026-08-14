import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN_SOURCE = (ROOT / "SRC/app/startup.c").read_text(encoding="utf-8")
TASK_SOURCE = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
TASK_HEADER = (ROOT / "SRC/app/app_tasks.h").read_text(encoding="utf-8")
SYSTEM_POLICY = (ROOT / "SRC/domain/system_policy.c").read_text(
    encoding="utf-8"
)
SYSTEM_POLICY_HEADER = (ROOT / "SRC/domain/system_policy.h").read_text(
    encoding="utf-8"
)
AUDIO_SOURCE = (ROOT / "SRC/platform/audio_output.c").read_text(
    encoding="utf-8"
)
CONFIG_SOURCE = "\n".join(
    (ROOT / path).read_text(encoding="utf-8")
    for path in (
        "SRC/platform/config_storage.c",
        "SRC/platform/config_json.c",
    )
)
UPDATE_SOURCE = (ROOT / "SRC/platform/firmware_update.c").read_text(
    encoding="utf-8"
)
UPDATE_HEADER = (ROOT / "SRC/platform/firmware_update.h").read_text(
    encoding="utf-8"
)
SW_SPEC = (ROOT / "DOC/SW_spec.md").read_text(encoding="utf-8")


class PowerOnHoldPolicyTests(unittest.TestCase):
    def test_power_on_hold_duration_is_a_shared_two_second_define(self) -> None:
        self.assertIn("#define POWER_ON_HOLD_MS UINT32_C(2000)", TASK_HEADER)
        self.assertIn("POWER_ON_HOLD_MS", MAIN_SOURCE)
        self.assertIn("POWER_ON_HOLD_MS", TASK_SOURCE)

    def test_power_on_wait_allows_only_parallel_startup_preparation(self) -> None:
        power_hold = MAIN_SOURCE.index("ret = board_init_power_hold();")
        safe_gpio = MAIN_SOURCE.index("ret = board_init_safe_gpio();")
        ota_check = MAIN_SOURCE.index(
            "firmware_update_running_image_pending_verify();"
        )
        preparation = MAIN_SOURCE.index(
            "startup_preparation_started = start_startup_preparation();"
        )
        power_wait = MAIN_SOURCE.index(
            "power_on_result = startup_power_on_confirmed();"
        )
        storage = MAIN_SOURCE.index("usb_device_storage_init(")

        self.assertLess(power_hold, safe_gpio)
        self.assertLess(safe_gpio, ota_check)
        self.assertLess(ota_check, preparation)
        self.assertLess(preparation, power_wait)
        self.assertLess(power_wait, storage)

    def test_ota_pending_verify_boot_bypasses_physical_hold(self) -> None:
        self.assertIn(
            "bool firmware_update_running_image_pending_verify(void);",
            UPDATE_HEADER,
        )
        helper_start = UPDATE_SOURCE.index(
            "bool firmware_update_running_image_pending_verify(void)"
        )
        helper_end = UPDATE_SOURCE.index(
            "esp_err_t firmware_update_process_boot", helper_start
        )
        helper = UPDATE_SOURCE[helper_start:helper_end]

        self.assertIn("esp_ota_get_running_partition()", helper)
        self.assertIn("esp_ota_get_state_partition", helper)
        self.assertIn("ESP_OTA_IMG_PENDING_VERIFY", helper)
        self.assertIn(
            "watchdog_service_begin_boot(ota_confirmation_boot,",
            MAIN_SOURCE,
        )
        self.assertIn(
            "watchdog_boot_action != WATCHDOG_BOOT_REQUIRE_SW1",
            MAIN_SOURCE,
        )
        self.assertIn(
            "board_set_status_leds_brightness(100U, false);",
            MAIN_SOURCE,
        )
        self.assertNotIn("esp_reset_reason", helper)

    def test_startup_wait_debounces_and_fades_green_until_confirmation(self) -> None:
        start = MAIN_SOURCE.index(
            "static startup_power_on_result_t startup_power_on_confirmed(void)"
        )
        end = MAIN_SOURCE.index("static bool startup_config_format_requested", start)
        wait_function = MAIN_SOURCE[start:end]

        self.assertIn("SYSTEM_POLICY_SAMPLE_PERIOD_MS UINT32_C(10)", SYSTEM_POLICY_HEADER)
        self.assertIn("SYSTEM_POLICY_SWITCH_DEBOUNCE_MS UINT32_C(30)", SYSTEM_POLICY_HEADER)
        self.assertIn("system_policy_debounce", wait_function)
        self.assertIn("button->stable_time_ms = 0U;", SYSTEM_POLICY)
        self.assertNotIn(
            "button->stable_time_ms = SYSTEM_POLICY_SAMPLE_PERIOD_MS;",
            SYSTEM_POLICY,
        )
        initial_sample = wait_function.index(
            "sw1.candidate_pressed = board_is_sw1_pressed();"
        )
        first_delay = wait_function.index(
            "vTaskDelay(pdMS_TO_TICKS(STARTUP_POWER_SAMPLE_PERIOD_MS));"
        )
        loop = wait_function.index("for (;;)")
        self.assertLess(initial_sample, first_delay)
        self.assertLess(first_delay, loop)
        self.assertIn("board_set_status_leds_brightness(", wait_function)
        self.assertIn("system_policy_power_on_brightness(", wait_function)
        self.assertIn("result.confirmed = true;", wait_function)
        self.assertIn("result.config_format_requested", wait_function)
        self.assertGreaterEqual(wait_function.count("return result;"), 2)

    def test_startup_preparation_runs_on_core_one_without_early_nvs_erase(self) -> None:
        preparation = MAIN_SOURCE[
            MAIN_SOURCE.index("static void run_startup_preparation(void)") :
            MAIN_SOURCE.index("static void log_hardware_configuration")
        ]
        recovery = MAIN_SOURCE[
            MAIN_SOURCE.index("static esp_err_t recover_nvs_after_power_confirmation") :
            MAIN_SOURCE.index("static void run_startup_preparation")
        ]

        self.assertIn("STARTUP_PREP_TASK_STACK_BYTES UINT32_C(4096)", MAIN_SOURCE)
        self.assertIn("STARTUP_PREP_TASK_PRIORITY ((UBaseType_t) 5U)", MAIN_SOURCE)
        self.assertIn("STARTUP_PREP_TASK_CORE ((BaseType_t) 1)", MAIN_SOURCE)
        self.assertIn('startup_preparation_task, "startup_prep"', preparation)
        self.assertLess(
            preparation.index("audio_output_init()"),
            preparation.index("nvs_flash_init()"),
        )
        self.assertNotIn("nvs_flash_erase", preparation)
        self.assertIn("nvs_flash_erase()", recovery)

    def test_confirmation_sound_precedes_slow_peripheral_startup(self) -> None:
        sound = MAIN_SOURCE.index("app_tasks_play_startup_sound(")
        storage = MAIN_SOURCE.index("usb_device_storage_init(")
        usb = MAIN_SOURCE.index("usb_device_start();")
        sensor = MAIN_SOURCE.index("sensor_bus_init();")

        self.assertLess(sound, storage)
        self.assertLess(sound, usb)
        self.assertLess(sound, sensor)
        self.assertIn(
            "switch_preferences.volume_level", MAIN_SOURCE[sound : sound + 160]
        )
        self.assertNotIn("board_set_safe_indicators();", AUDIO_SOURCE)
        self.assertIn("board_set_audio_shutdown();", AUDIO_SOURCE)

    def test_battery_adc_is_initialized_once_before_update_processing(self) -> None:
        adc_init = MAIN_SOURCE.index("system_io_result = system_io_init();")
        update = MAIN_SOURCE.index("firmware_update_process_boot(")

        self.assertLess(adc_init, update)
        self.assertEqual(MAIN_SOURCE.count("system_io_init();"), 1)
        self.assertIn("startup_read_battery(", MAIN_SOURCE)
        self.assertIn(
            "#define STARTUP_BATTERY_SAMPLE_PERIOD_MS UINT32_C(100)",
            MAIN_SOURCE,
        )
        self.assertIn(
            "#define STARTUP_BATTERY_SAMPLE_COUNT UINT32_C(5)",
            MAIN_SOURCE,
        )
        self.assertIn("else if (!external_power_present)", MAIN_SOURCE)
        startup_gate = MAIN_SOURCE.index("battery_power_startup_blocked(")
        sound = MAIN_SOURCE.index("app_tasks_play_startup_sound(")
        storage = MAIN_SOURCE.index("usb_device_storage_init(")
        self.assertLess(adc_init, startup_gate)
        self.assertLess(startup_gate, sound)
        self.assertLess(startup_gate, storage)

    def test_multi_profile_startup_buffers_are_not_on_main_stack(self) -> None:
        startup = MAIN_SOURCE[MAIN_SOURCE.index("void app_startup_run(void)") :]

        self.assertIn(
            "static app_config_profiles_t startup_runtime_profiles;",
            MAIN_SOURCE,
        )
        self.assertNotIn("app_config_profiles_t runtime_profiles", startup)
        self.assertIn("config_storage_workspace", CONFIG_SOURCE)
        self.assertNotIn(
            "app_config_profiles_t candidate = {0};", CONFIG_SOURCE
        )
        self.assertNotIn(
            "app_config_profiles_t recovered = {0};", CONFIG_SOURCE
        )

    def test_rejected_startup_releases_hold_into_safe_stop(self) -> None:
        self.assertIn("void app_tasks_run_safe_stop(void);", TASK_HEADER)
        self.assertIn("app_tasks_run_safe_stop();", MAIN_SOURCE)
        safe_stop = TASK_SOURCE[
            TASK_SOURCE.index("void app_workers_run_safe_stop(void)") :
            TASK_SOURCE.index("static void request_power_off", TASK_SOURCE.index("void app_workers_run_safe_stop(void)"))
        ]
        self.assertIn("board_set_safe_indicators();", safe_stop)
        self.assertIn("board_set_power_hold(false)", safe_stop)
        self.assertIn(
            "run_safe_stop_loop(safe_sleep_enabled, WATCHDOG_ACTOR_STARTUP)",
            safe_stop,
        )
        self.assertLess(
            safe_stop.index("usb_device_stop();"),
            safe_stop.index("board_set_power_hold(false)"),
        )
        self.assertLess(
            safe_stop.index("board_set_power_hold(false)"),
            safe_stop.index("app_power_prepare_safe_stop();"),
        )

    def test_safe_stop_uses_the_same_duration_and_reverse_fade(self) -> None:
        start = TASK_SOURCE.index("static void safe_stop_qualify_power_on")
        end = TASK_SOURCE.index("static void run_safe_stop_polling_fallback", start)
        loop = TASK_SOURCE[start:end]

        self.assertIn("system_policy_power_on_brightness(", loop)
        self.assertIn("hold_time_ms >= POWER_ON_HOLD_MS", loop)
        self.assertIn("esp_restart();", loop)
        self.assertNotIn("hold_time_ms >= POWER_OFF_HOLD_MS", loop)

    def test_specification_records_power_on_wait(self) -> None:
        self.assertIn("POWER_ON_WAIT", SW_SPEC)
        self.assertIn("POWER_ON_HOLD_MS", SW_SPEC)
        self.assertIn("0→100 %", SW_SPEC)


if __name__ == "__main__":
    unittest.main()
