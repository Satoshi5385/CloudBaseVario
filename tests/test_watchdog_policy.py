import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULTS = (ROOT / "sdkconfig.defaults").read_text(encoding="utf-8")
STARTUP = (ROOT / "SRC/app/startup.c").read_text(encoding="utf-8")
TASKS = (ROOT / "SRC/app/app_tasks.c").read_text(encoding="utf-8")
WORKERS = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
BLE_WORKER = (ROOT / "SRC/app/ble_tx_worker.c").read_text(encoding="utf-8")
UPDATE = (ROOT / "SRC/platform/firmware_update.c").read_text(encoding="utf-8")
SERVICE = (ROOT / "SRC/platform/watchdog_service.c").read_text(encoding="utf-8")
POLICY = (ROOT / "SRC/domain/watchdog_recovery_policy.c").read_text(
    encoding="utf-8"
)
SPEC = (ROOT / "DOC/SW_spec.md").read_text(encoding="utf-8")
WORKFLOW = (ROOT / ".github/workflows/build.yml").read_text(encoding="utf-8")


class WatchdogPolicyTests(unittest.TestCase):
    def test_product_watchdog_defaults_are_explicit(self) -> None:
        for setting in (
            "CONFIG_BOOTLOADER_WDT_ENABLE=y",
            "CONFIG_BOOTLOADER_WDT_TIME_MS=9000",
            "CONFIG_ESP_INT_WDT=y",
            "CONFIG_ESP_INT_WDT_TIMEOUT_MS=300",
            "CONFIG_ESP_INT_WDT_CHECK_CPU1=y",
            "CONFIG_ESP_TASK_WDT_EN=y",
            "CONFIG_ESP_TASK_WDT_INIT=y",
            "CONFIG_ESP_TASK_WDT_PANIC=y",
            "CONFIG_ESP_TASK_WDT_TIMEOUT_S=5",
            "CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y",
            "CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=y",
            "CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y",
            "CONFIG_ESP_SYSTEM_PANIC_REBOOT_DELAY_SECONDS=0",
            "CONFIG_ESP_COREDUMP_ENABLE_TO_NONE=y",
        ):
            self.assertIn(setting, DEFAULTS)
        self.assertIn(
            "# CONFIG_BOOTLOADER_WDT_DISABLE_IN_USER_CODE is not set",
            DEFAULTS,
        )
        self.assertNotIn("CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y", DEFAULTS)

    def test_only_typed_watchdog_resets_are_auto_recovery_candidates(self) -> None:
        eligible = POLICY[
            POLICY.index("bool watchdog_recovery_reset_is_eligible") :
            POLICY.index("void watchdog_recovery_state_initialize")
        ]
        self.assertIn("WATCHDOG_RESET_TASK_WDT", eligible)
        self.assertIn("WATCHDOG_RESET_INTERRUPT_WDT", eligible)
        self.assertNotIn("WATCHDOG_RESET_OTHER_WDT", eligible)
        self.assertIn("watchdog_reset_count == 1U", POLICY)
        self.assertIn("WATCHDOG_BOOT_OTA_CONFIRMATION", POLICY)
        self.assertIn("WATCHDOG_RECOVERY_STABLE_PERIOD_MS", POLICY)

    def test_rtc_record_is_lightweight_and_power_on_gate_uses_decision(self) -> None:
        self.assertIn("RTC_NOINIT_ATTR watchdog_rtc_store_t", SERVICE)
        self.assertIn("watchdog_recovery_select_state(", SERVICE)
        self.assertIn("heartbeat_inverse", SERVICE)
        self.assertIn("current_diagnostics.previous_stage = retained_stage;", SERVICE)
        self.assertIn("current_diagnostics.suspected_actor = suspected;", SERVICE)
        self.assertIn("watchdog_service_begin_boot(ota_confirmation_boot,", STARTUP)
        self.assertIn("WATCHDOG_BOOT_REQUIRE_SW1", STARTUP)
        self.assertIn("watchdog_service_mark_user_confirmed();", STARTUP)
        self.assertNotIn("nvs_", SERVICE.lower())

    def test_critical_tasks_are_registered_but_communications_are_not(self) -> None:
        for actor in (
            "WATCHDOG_ACTOR_STARTUP",
            "WATCHDOG_ACTOR_STARTUP_PREP",
            "WATCHDOG_ACTOR_SENSOR",
            "WATCHDOG_ACTOR_AUDIO",
            "WATCHDOG_ACTOR_SYSTEM",
        ):
            self.assertIn(actor, STARTUP + TASKS + WORKERS)
        console = WORKERS[
            WORKERS.index("void app_console_worker_task") :
            WORKERS.index("void app_workers_run_fatal_fallback")
        ]
        self.assertNotIn("watchdog_service_register_current", console)
        self.assertNotIn("watchdog_service_register_current", BLE_WORKER)

    def test_safe_stop_sw1_confirmation_clears_recovery_window(self) -> None:
        safe_stop = WORKERS[
            WORKERS.index("static void safe_stop_qualify_power_on") :
            WORKERS.index("static void run_safe_stop_polling_fallback")
        ]
        self.assertLess(
            safe_stop.index("watchdog_service_mark_user_confirmed();"),
            safe_stop.index("esp_restart();"),
        )

    def test_intentional_fatal_and_shutdown_paths_are_wdt_safe(self) -> None:
        self.assertIn(
            "unregister_critical_watchdog(WATCHDOG_ACTOR_SENSOR", WORKERS
        )
        self.assertIn(
            "unregister_critical_watchdog(WATCHDOG_ACTOR_AUDIO", WORKERS
        )
        shutdown_wait = WORKERS[
            WORKERS.index("static bool wait_for_shutdown_bits") :
            WORKERS.index("static void run_safe_stop_loop")
        ]
        self.assertIn("wait_ms > SHUTDOWN_WAIT_SLICE_MS", shutdown_wait)
        self.assertIn("wait_ms = SHUTDOWN_WAIT_SLICE_MS", shutdown_wait)
        self.assertIn("WATCHDOG_ACTOR_SYSTEM", shutdown_wait)
        self.assertIn("WATCHDOG_STAGE_SAFE_STOP", WORKERS)

    def test_ota_write_feeds_startup_watchdog(self) -> None:
        apply_update = UPDATE[
            UPDATE.index("static esp_err_t apply_update") :
            UPDATE.index("bool firmware_update_running_image_pending_verify")
        ]
        self.assertIn("esp_ota_write", apply_update)
        self.assertIn(
            "watchdog_service_feed(WATCHDOG_ACTOR_STARTUP)", apply_update
        )

    def test_diag_and_build_contract_cover_effective_configuration(self) -> None:
        self.assertIn('"WATCHDOG reset=%s action=%s', WORKERS)
        self.assertIn("CONFIG_ESP_TASK_WDT_PANIC 1", WORKFLOW)
        self.assertIn("CONFIG_ESP_COREDUMP_ENABLE_TO_NONE 1", WORKFLOW)
        self.assertIn("test_watchdog_recovery_policy", WORKFLOW)
        self.assertIn("Task Watchdog", SPEC)


if __name__ == "__main__":
    unittest.main()
