import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKERS = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
AUDIO_OUTPUT = (ROOT / "SRC/platform/audio_output.c").read_text(
    encoding="utf-8"
)
DEFAULTS = (ROOT / "sdkconfig.defaults").read_text(encoding="utf-8")
SW_SPEC = (ROOT / "DOC/SW_spec.md").read_text(encoding="utf-8")


def function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start + len(signature))
    return source[start:end]


class RuntimePowerPolicyTests(unittest.TestCase):
    def test_imu_processing_does_not_publish_at_400_hz(self):
        imu = function_body(
            WORKERS,
            "static bool sensor_process_imu(",
            "static bool sensor_process_bmp581(",
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
        sensor_task = function_body(
            WORKERS,
            "void app_sensor_worker_task(",
            "static bool system_sound_abort_requested(",
        )
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
        self.assertIn("performance optimization", SW_SPEC)
        self.assertIn("`-O2`", SW_SPEC)

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
            "static bool sensor_process_imu(",
        )

        self.assertIn("sensor_measurement_work_due(state, now_us)", measurement)
        self.assertLess(
            measurement.index("app_power_sensor_work_begin()"),
            measurement.index("sensor_process_imu("),
        )
        self.assertLess(
            measurement.index("sensor_process_bmp581("),
            measurement.index("app_power_sensor_work_end()"),
        )
        for excluded in (
            "sensor_try_initialize_devices",
            "sensor_refresh_imu_config",
            "sensor_try_save_accel_calibration",
            "sensor_check_stale",
            "sensor_recover_shared_bus",
        ):
            self.assertNotIn(excluded, measurement)
            self.assertIn(excluded, maintenance)
        self.assertNotIn("app_power_sensor_work_begin", maintenance)
        self.assertNotIn("app_power_sensor_work_end", maintenance)
        self.assertNotIn("sensor_try_save_accel_calibration", calibration)
        self.assertIn("実際にIMUまたはBMP581のサンプル処理", SW_SPEC)

    def test_bmp_overrun_tracking_starts_with_first_read_attempt(self):
        state_definition = function_body(
            WORKERS,
            "typedef struct {\n    vario_result_t result;",
            "static const char *TAG",
        )
        initialization = function_body(
            WORKERS,
            "static bool sensor_try_initialize_devices(",
            "static bool imu_configs_match(",
        )
        bmp = function_body(
            WORKERS,
            "static bool sensor_process_bmp581(",
            "static bool sensor_check_stale(",
        )

        self.assertIn("bool bmp_period_tracking_started;", state_definition)
        self.assertIn(
            "state->bmp_period_tracking_started = false;", initialization
        )
        tracking_start = bmp.index("if (!state->bmp_period_tracking_started)")
        overrun_calculation = bmp.index("periods_elapsed =", tracking_start)
        first_read = bmp.index("bmp581_read_sample(&sample)")
        self.assertLess(tracking_start, overrun_calculation)
        self.assertLess(overrun_calculation, first_read)
        self.assertIn(
            "state->next_bmp_deadline_us = now_us + BMP581_SAMPLE_PERIOD_US;",
            bmp,
        )
        self.assertIn(
            "最初のBMP581読み出しを10 ms絶対期限の基準", SW_SPEC
        )


if __name__ == "__main__":
    unittest.main()
