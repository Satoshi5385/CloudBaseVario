import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
STARTUP = (ROOT / "SRC/app/startup.c").read_text(encoding="utf-8")
TASKS = (ROOT / "SRC/app/app_tasks.c").read_text(encoding="utf-8")
WORKERS = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
BMP581 = (ROOT / "SRC/platform/bmp581.c").read_text(encoding="utf-8")
IMU = (ROOT / "SRC/platform/icm42688_hxy.c").read_text(encoding="utf-8")
SENSOR_BUS = (ROOT / "SRC/platform/sensor_bus.c").read_text(
    encoding="utf-8"
)
AUDIO_OUTPUT = (ROOT / "SRC/platform/audio_output.c").read_text(
    encoding="utf-8"
)
DEFAULTS = (ROOT / "sdkconfig.defaults").read_text(encoding="utf-8")


def function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start + len(signature))
    return source[start:end]


class RuntimePowerPolicyTests(unittest.TestCase):
    def test_imu_interrupt_read_only_buffers_sample(self):
        imu_read = function_body(
            WORKERS,
            "static bool sensor_read_imu(",
            "static bool sensor_process_buffered_imu_sample(",
        )
        self.assertIn("icm42688_hxy_read_sample(", imu_read)
        self.assertIn("imu_sample_buffer_push(", imu_read)
        self.assertIn("state->publication_pending = true;", imu_read)
        self.assertNotIn("imu_fusion_update", imu_read)
        self.assertNotIn("vario_estimator_update_imu", imu_read)
        self.assertNotIn("app_resources_copy_config", imu_read)
        self.assertNotIn("app_power_sensor_work_begin", imu_read)

    def test_bmp_read_drains_buffer_and_runs_estimators_without_imu_wait(self):
        bmp = function_body(
            WORKERS,
            "static bool sensor_process_bmp581(",
            "static bool sensor_check_stale(",
        )
        self.assertLess(
            bmp.index("bmp581_read_sample("),
            bmp.index("sensor_drain_imu_buffer("),
        )
        self.assertLess(
            bmp.index("sensor_drain_imu_buffer("),
            bmp.index("vario_estimator_update("),
        )
        self.assertNotIn("icm42688_hxy_read_sample", bmp)
        self.assertNotIn("ulTaskNotifyTake", bmp)
        self.assertNotIn("vTaskDelay", bmp)

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
            "#define SENSOR_PUBLICATION_PERIOD_US INT64_C(10000)",
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

    def test_cpu_lock_wraps_computation_but_not_i2c(self):
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
        bmp = function_body(
            WORKERS,
            "static bool sensor_process_bmp581(",
            "static bool sensor_check_stale(",
        )

        self.assertIn("sensor_read_imu(", measurement)
        self.assertIn("sensor_process_bmp581(", measurement)
        self.assertNotIn("app_power_sensor_work_begin", measurement)
        self.assertNotIn("app_power_sensor_work_end", measurement)
        self.assertLess(
            bmp.index("bmp581_read_sample("),
            bmp.index("app_power_sensor_work_begin()"),
        )
        self.assertLess(
            bmp.index("app_power_sensor_work_begin()"),
            bmp.index("sensor_drain_imu_buffer("),
        )
        self.assertLess(
            bmp.index("vario_estimator_update("),
            bmp.index("app_power_sensor_work_end()"),
        )
        for excluded in (
            "sensor_try_initialize_devices",
            "sensor_refresh_imu_config",
            "sensor_try_save_accel_calibration",
            "sensor_check_stale",
            "sensor_recover_shared_bus",
        ):
            self.assertIn(excluded, maintenance)
        self.assertNotIn("app_power_sensor_work_begin", maintenance)
        self.assertNotIn("app_power_sensor_work_end", maintenance)

    def test_sensor_bus_uses_xtal_clock_source(self):
        self.assertIn(".clk_source = I2C_CLK_SRC_XTAL", SENSOR_BUS)

    def test_sensor_bus_is_created_by_core1_sensor_task(self):
        initialization = function_body(
            WORKERS,
            "static bool sensor_try_initialize_devices(",
            "static bool imu_configs_match(",
        )
        descriptor_start = TASKS.index("{APP_TASK_WORKER_SENSOR")
        descriptor_end = TASKS.index("},", descriptor_start)
        sensor_descriptor = TASKS[descriptor_start:descriptor_end]

        self.assertNotIn("sensor_bus_init();", STARTUP)
        self.assertIn("ret = sensor_bus_init();", initialization)
        self.assertLess(
            initialization.index("sensor_bus_init();"),
            initialization.index("sensor_try_initialize_imu("),
        )
        self.assertIn(
            "#define HIGH_RATE_TASK_CORE ((BaseType_t) 1)", TASKS
        )
        self.assertIn("HIGH_RATE_TASK_CORE", sensor_descriptor)

    def test_scheduler_uses_only_data_ready_events_for_sensor_reads(self):
        due = function_body(
            WORKERS,
            "static bool sensor_measurement_work_due(",
            "static void sensor_collect_data_ready_events(",
        )
        collect = function_body(
            WORKERS,
            "static void sensor_collect_data_ready_events(",
            "static bool sensor_execute_measurement_work(",
        )
        self.assertIn("state->imu_interrupt_pending", due)
        self.assertIn("state->bmp_interrupt_pending", due)
        self.assertIn("icm42688_hxy_take_data_ready_event", collect)
        self.assertIn("bmp581_take_data_ready_event", collect)
        self.assertNotIn("next_bmp_deadline_us", WORKERS)
        self.assertNotIn("BMP581_SAMPLE_PERIOD_US", WORKERS)

    def test_initialization_cannot_immediately_trigger_stale_detection(self):
        initialization = function_body(
            WORKERS,
            "static bool sensor_try_initialize_imu(",
            "static void sensor_invalidate_estimate(",
        ) + function_body(
            WORKERS,
            "static bool sensor_try_initialize_devices(",
            "static bool imu_configs_match(",
        )
        work = function_body(
            WORKERS,
            "static bool sensor_execute_work(",
            "void app_sensor_worker_task(",
        )

        self.assertIn(
            "state->last_imu_valid_us = esp_timer_get_time();",
            initialization,
        )
        self.assertIn(
            "state->last_bmp_valid_us = esp_timer_get_time();",
            initialization,
        )
        self.assertNotIn("state->last_imu_valid_us = now_us;", initialization)
        self.assertNotIn("state->last_bmp_valid_us = now_us;", initialization)
        self.assertLess(
            work.index("sensor_try_initialize_devices("),
            work.index("sensor_collect_data_ready_events("),
        )
        self.assertLess(
            work.index("sensor_collect_data_ready_events("),
            work.index("sensor_check_stale("),
        )

    def test_bmp_data_ready_read_uses_two_i2c_transactions(self):
        read = BMP581[BMP581.index("esp_err_t bmp581_read_sample(") :]
        self.assertEqual(2, read.count("bmp581_read_registers("))
        self.assertIn("BMP581_REG_INT_STATUS", read)
        self.assertIn("BMP581_REG_DATA_START", read)
        self.assertIn("interrupt_timestamp_us", read)
        self.assertIn("GPIO_INTR_POSEDGE", BMP581)
        self.assertIn("PIN_INT_BMP", BMP581)
        self.assertIn("vTaskNotifyGiveFromISR", BMP581)
        self.assertIn("BMP581_INT_CONFIG_VALUE UINT8_C(0x3A)", BMP581)
        self.assertIn("BMP581_INT_SOURCE_DATA_READY UINT8_C(0x01)", BMP581)

    def test_imu_odr_is_200_hz_and_timestamp_comes_from_isr(self):
        self.assertIn("ICM42688_HXY_SAMPLE_RATE_HZ == UINT32_C(200)", IMU)

        initialization = function_body(
            IMU,
            "esp_err_t icm42688_hxy_init(",
            "static int16_t decode_be_int16(",
        )
        self.assertLess(
            initialization.index(
                "ret = configure_data_ready_gpio(sensor_task);"
            ),
            initialization.index("ret = configure_sensor();"),
        )
        self.assertIn(
            "ICM42688_HXY_ACC_CONF_200HZ_VALUE UINT8_C(0xA9)", IMU
        )
        self.assertIn(
            "ICM42688_HXY_GYR_CONF_200HZ_VALUE UINT8_C(0xA9)", IMU
        )
        self.assertIn("latest_data_ready_timestamp_us = esp_timer_get_time()", IMU)
        self.assertIn("sample->timestamp_us = interrupt_timestamp_us", IMU)
        self.assertIn("GPIO_INTR_POSEDGE", IMU)
        self.assertIn("vTaskNotifyGiveFromISR", IMU)


if __name__ == "__main__":
    unittest.main()
