import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKERS = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
USB = (ROOT / "SRC/platform/usb_device_service.c").read_text(
    encoding="utf-8"
)
USB_HEADER = (ROOT / "SRC/platform/usb_device_service.h").read_text(
    encoding="utf-8"
)
TINYUSB_MSC = (ROOT / "components/esp_tinyusb/tinyusb_msc.c").read_text(
    encoding="utf-8"
)
POWER = (ROOT / "SRC/platform/app_power.c").read_text(encoding="utf-8")
WAKE = (ROOT / "SRC/platform/safe_stop_wake.c").read_text(encoding="utf-8")


class LowPowerSafeStopPolicyTests(unittest.TestCase):
    def test_console_uses_connection_dependent_periods(self) -> None:
        self.assertIn(
            "#define CONSOLE_CONNECTED_POLL_MS UINT32_C(10)", WORKERS
        )
        self.assertIn(
            "#define CONSOLE_DISCONNECTED_POLL_MS UINT32_C(250)", WORKERS
        )
        console = WORKERS[
            WORKERS.index("void app_console_worker_task") :
            WORKERS.index("void app_workers_run_fatal_fallback")
        ]
        self.assertIn("if (connected && !previously_connected)", console)
        self.assertIn("else if (!connected)", console)
        self.assertIn("if (connected && now_us >= next_monitor_us)", console)

    def test_tinyusb_stop_preserves_storage_and_rejects_writes(self) -> None:
        self.assertIn("esp_err_t usb_device_stop(void);", USB_HEADER)
        stop = USB[
            USB.index("esp_err_t usb_device_stop(void)") :
            USB.index("esp_err_t usb_device_enable_msc", USB.index("esp_err_t usb_device_stop(void)"))
        ]
        self.assertIn("usb_diagnostics.storage_mode_active", stop)
        self.assertIn("usb_diagnostics.pending_write_count != 0U", stop)
        self.assertIn("tinyusb_msc_stop_host_io", stop)
        self.assertIn("internal_pending_writes != 0U", stop)
        self.assertLess(
            stop.index("tinyusb_console_deinit"),
            stop.index("tinyusb_driver_uninstall"),
        )
        self.assertLess(
            stop.index("tinyusb_driver_uninstall"),
            stop.index("tinyusb_cdcacm_deinit"),
        )
        self.assertNotIn("tinyusb_msc_delete_storage", stop)
        self.assertNotIn("tinyusb_msc_uninstall_driver", stop)
        self.assertIn("storage->host_io_enabled = false;", TINYUSB_MSC)
        self.assertIn("storage->deffered_writes", TINYUSB_MSC)

    def test_runtime_shutdown_stops_usb_before_enabling_sleep(self) -> None:
        shutdown = WORKERS[
            WORKERS.index("static void request_power_off") :
            WORKERS.index("void app_system_worker_task")
        ]
        self.assertLess(
            shutdown.index("usb_device_stop();"),
            shutdown.index("board_set_power_hold(false)"),
        )
        self.assertLess(
            shutdown.index("board_set_power_hold(false)"),
            shutdown.index("app_power_prepare_safe_stop();"),
        )
        self.assertNotIn("usb_device_bus_active()", shutdown)

    def test_safe_stop_uses_gpio_wake_and_one_second_wdt_wait(self) -> None:
        safe_stop = WORKERS[
            WORKERS.index("static void run_safe_stop_loop") :
            WORKERS.index("void app_workers_run_safe_stop")
        ]
        self.assertIn("safe_stop_wake_arm(false)", safe_stop)
        self.assertIn("safe_stop_wake_arm(true)", safe_stop)
        self.assertIn("ulTaskNotifyTake(", safe_stop)
        self.assertIn("SAFE_STOP_WATCHDOG_PERIOD_MS", safe_stop)
        self.assertIn("run_safe_stop_polling_fallback", safe_stop)
        self.assertIn("gpio_wakeup_enable(PIN_SW_1, trigger)", WAKE)
        self.assertIn("esp_sleep_enable_gpio_wakeup()", WAKE)

    def test_pm_supports_early_safe_stop_and_interaction_lock(self) -> None:
        prepare = POWER[
            POWER.index("esp_err_t app_power_prepare_safe_stop") :
            POWER.index("void app_power_get_diagnostics")
        ]
        self.assertIn("app_power_init();", prepare)
        self.assertIn("app_power_enter_safe_stop();", prepare)
        self.assertIn("esp_pm_lock_acquire(no_light_sleep_lock)", prepare)
        self.assertIn("esp_pm_lock_release(no_light_sleep_lock)", prepare)


if __name__ == "__main__":
    unittest.main()
