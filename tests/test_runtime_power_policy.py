import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKERS = (ROOT / "SRC/app/sensor_worker.c").read_text(encoding="utf-8")
AUDIO_OUTPUT = (ROOT / "SRC/platform/audio_output.c").read_text(
    encoding="utf-8"
)
DEFAULTS = (ROOT / "sdkconfig.defaults").read_text(encoding="utf-8")
SCHEDULER = (ROOT / "SRC/domain/sensor_scheduler.c").read_text(
    encoding="utf-8"
)


def function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start + len(signature))
    return source[start:end]


class RuntimePowerPolicyTests(unittest.TestCase):
    def test_imu_processing_does_not_publish_at_400_hz(self):
        imu = function_body(
            WORKERS,
            "static bool sensor_process_imu_sample(",
            "static bool sensor_process_imu_batch(",
        )
        self.assertIn("state->publication_pending = true;", imu)
        self.assertNotIn("app_resources_publish_vario", imu)
        self.assertNotIn("app_resources_publish_imu_diagnostics", imu)
        self.assertNotIn("app_resources_copy_config", imu)
        self.assertTrue(imu.rstrip().endswith("return false;\n}"))

    def test_sensor_publication_is_capped_at_100_hz(self):
        publisher = function_body(
            WORKERS,
            "static void sensor_publish_snapshots(",
            "static TickType_t sensor_wait_ticks(",
        )
        sensor_task = WORKERS[WORKERS.index("void app_sensor_worker_task(") :]
        self.assertIn(
            "#define SENSOR_PUBLICATION_PERIOD_US BMP581_SAMPLE_PERIOD_US",
            WORKERS,
        )
        self.assertIn("app_resources_publish_vario", publisher)
        self.assertIn("app_resources_publish_imu_diagnostics", publisher)
        self.assertIn("sensor_publication_due(&state, now_us)", sensor_task)
        self.assertEqual(2, WORKERS.count("app_resources_publish_vario"))
        self.assertEqual(
            2, WORKERS.count("app_resources_publish_imu_diagnostics")
        )

    def test_fifo_watermark_drives_the_shared_measurement_burst(self):
        due = function_body(
            WORKERS, "static bool sensor_measurement_work_due(",
            "static bool sensor_execute_measurement_work(",
        )
        wait = function_body(
            WORKERS, "static TickType_t sensor_wait_ticks(",
            "static bool sensor_measurement_work_due(",
        )
        measurement = function_body(
            WORKERS, "static bool sensor_execute_measurement_work(",
            "static bool sensor_execute_work(",
        )
        sensor_task = WORKERS[WORKERS.index("void app_sensor_worker_task(") :]
        self.assertIn("state->imu_watermark_pending", due)
        self.assertIn("sensor_scheduler_trigger", due)
        self.assertIn("sensor_scheduler_next_wake_us", wait)
        self.assertNotIn("IMU_WATERMARK_FALLBACK_GRACE_US", WORKERS)
        self.assertIn("ulTaskNotifyTake", sensor_task)
        self.assertIn("state.imu_watermark_pending = true", sensor_task)
        self.assertIn("sensor_acquire_cycle(state", measurement)
        self.assertIn("sensor_apply_cycle(state", measurement)
        driver = (ROOT / "SRC/platform/icm42688_hxy.c").read_text(encoding="utf-8")
        watermark_isr = function_body(
            driver,
            "static void IRAM_ATTR fifo_watermark_isr(",
            "static esp_err_t configure_fifo_gpio(",
        )
        self.assertIn("vTaskNotifyGiveFromISR", driver)
        self.assertIn("gpio_isr_handler_add", driver)
        self.assertIn(".intr_type = GPIO_INTR_POSEDGE", driver)
        self.assertIn("ICM42688_HXY_INT1_FIFO_WATERMARK", driver)
        self.assertIn("ICM42688_HXY_FIFO_WATERMARK_THRESHOLD_WORDS", driver)
        self.assertIn("gpio_ll_intr_disable(&GPIO, PIN_INT_ICM)", watermark_isr)
        self.assertNotIn("gpio_intr_disable(", watermark_isr)

    def test_same_cycle_pipeline_acquires_then_estimates_in_order(self):
        acquire = function_body(
            WORKERS, "static void sensor_acquire_cycle(",
            "static bool sensor_record_bmp_cycle(",
        )
        apply = function_body(
            WORKERS, "static bool sensor_apply_cycle(",
            "static bool sensor_check_stale(",
        )
        self.assertLess(
            acquire.index("icm42688_hxy_read_fifo"),
            acquire.index("bmp581_read_sample"),
        )
        self.assertLess(
            acquire.index("bmp581_read_sample"),
            acquire.index("cycle->config = state->measurement_config"),
        )
        self.assertLess(
            apply.index("sensor_process_imu_batch"),
            apply.index("sensor_apply_bmp_sample"),
        )
        self.assertNotIn("pending_bmp", WORKERS)

    def test_audio_output_skips_unchanged_commands_and_repeated_shutdown(self):
        apply = function_body(
            AUDIO_OUTPUT,
            "esp_err_t audio_output_apply(",
            "void audio_output_shutdown(",
        )
        shutdown = AUDIO_OUTPUT[AUDIO_OUTPUT.index("void audio_output_shutdown(") :]
        self.assertIn("frequency_hz == active_frequency_hz", apply)
        self.assertIn("duty_percent == active_duty_percent", apply)
        self.assertIn("amplifier_mode == active_amplifier_mode", apply)
        self.assertIn("if (!timer_running)", shutdown)
        self.assertIn("return;", shutdown)

    def test_product_build_uses_performance_optimization(self):
        self.assertIn("CONFIG_COMPILER_OPTIMIZATION_PERF=y", DEFAULTS)
        self.assertNotIn("CONFIG_COMPILER_OPTIMIZATION_DEBUG=y", DEFAULTS)

    def test_sensor_cpu_lock_only_wraps_due_sample_processing(self):
        measurement = function_body(
            WORKERS,
            "static bool sensor_execute_measurement_work(",
            "static bool sensor_execute_work(",
        )
        maintenance = function_body(
            WORKERS,
            "static bool sensor_execute_work(",
            "void app_sensor_worker_task(",
        )
        calibration = function_body(
            WORKERS,
            "static bool sensor_process_factory_accel_calibration(",
            "static bool sensor_process_imu_sample(",
        )

        self.assertIn("sensor_measurement_work_due(state, now_us)", measurement)
        self.assertLess(
            measurement.index("app_power_sensor_work_begin()"),
            measurement.index("sensor_acquire_cycle("),
        )
        self.assertLess(
            measurement.index("sensor_apply_cycle("),
            measurement.index("app_power_sensor_work_end()"),
        )
        for excluded in (
            "sensor_try_initialize_devices",
            "sensor_refresh_configs",
            "sensor_try_save_accel_calibration",
            "sensor_check_stale",
            "sensor_recover_shared_bus",
        ):
            self.assertNotIn(excluded, measurement)
            self.assertIn(excluded, maintenance)
        self.assertNotIn("app_power_sensor_work_begin", maintenance)
        self.assertNotIn("app_power_sensor_work_end", maintenance)
        self.assertNotIn("sensor_try_save_accel_calibration", calibration)

    def test_bmp_timer_uses_an_absolute_deadline(self):
        state_definition = function_body(
            WORKERS,
            "typedef struct {\n    vario_result_t result;",
            "typedef struct {\n    icm42688_hxy_batch_t imu_batch;",
        )
        self.assertIn("sensor_scheduler_t scheduler;", state_definition)
        self.assertNotIn("next_bmp_deadline_us", state_definition)
        self.assertIn("scheduler->next_bmp_us +=", SCHEDULER)
        self.assertIn("periods_elapsed + 1", SCHEDULER)
if __name__ == "__main__":
    unittest.main()
