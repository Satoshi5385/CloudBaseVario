from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
GPS_WORKER = (ROOT / "SRC/app/gps_worker.c").read_text(encoding="utf-8")
GPS_PLATFORM = (ROOT / "SRC/platform/gps_l96.c").read_text(encoding="utf-8")
APP_WORKERS = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
APP_TASKS = (ROOT / "SRC/app/app_tasks.c").read_text(encoding="utf-8")
BLE_PLATFORM = (ROOT / "SRC/platform/ble_vario.c").read_text(encoding="utf-8")
BLE_WORKER = (ROOT / "SRC/app/ble_tx_worker.c").read_text(encoding="utf-8")
BLE_NUS_TX = (ROOT / "SRC/domain/ble_nus_tx.c").read_text(encoding="utf-8")
SYSTEM_POLICY = (ROOT / "SRC/domain/system_policy.c").read_text(encoding="utf-8")
SYSTEM_POLICY_HEADER = (ROOT / "SRC/domain/system_policy.h").read_text(
    encoding="utf-8"
)


class GpsPolicyTests(unittest.TestCase):
    def test_absent_identity_exits_before_any_uart_call(self) -> None:
        absent = GPS_WORKER.index("if (!snapshot.installed)")
        connect = GPS_WORKER.index("gps_l96_connect(")
        self.assertLess(absent, connect)
        absent_block = GPS_WORKER[absent:connect]
        self.assertIn("acknowledge_and_delete();", absent_block)
        self.assertNotIn("gps_l96_", absent_block)

    def test_absent_model_does_not_create_gps_task(self) -> None:
        self.assertIn(
            "descriptor->worker != APP_TASK_WORKER_GPS || gps_is_installed()",
            APP_TASKS,
        )
        create_loop = APP_TASKS.split("esp_err_t app_tasks_start", 1)[1].split(
            "if (result != ESP_OK)", 1
        )[0]
        self.assertIn("if (!worker_is_enabled(descriptor))", create_loop)
        self.assertIn("continue;", create_loop)
        required_workers = APP_TASKS.split(
            "bool app_tasks_required_workers_started", 1
        )[1].split("TaskHandle_t app_tasks_worker_handle", 1)[0]
        self.assertIn("worker_is_enabled(descriptor)", required_workers)
        self.assertIn("worker_stack_watermark(APP_TASK_WORKER_GPS)", APP_WORKERS)

    def test_l96_configuration_and_retry_contract(self) -> None:
        for token in (
            'send_pmtk("PMTK605")',
            'send_pmtk("PMTK251,115200")',
            '"PMTK353,1,1,0,0,0"',
            '"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"',
            '"PMTK220,%" PRIu32',
        ):
            self.assertIn(token, GPS_PLATFORM)
        self.assertIn("GPS_RETRY_INTERVAL_MS UINT32_C(5000)", GPS_WORKER)
        self.assertIn("GPS_MIN_STALE_TIMEOUT_MS UINT32_C(3000)", GPS_WORKER)
        self.assertIn("gps_pmtk_ack_parse(line, &ack)", GPS_PLATFORM)
        self.assertIn("is_restart_notice(line)", GPS_PLATFORM)
        self.assertIn('strncmp(line, "$PMTK010,003*", 13U)', GPS_PLATFORM)
        restart_ready = GPS_PLATFORM.split(
            "static bool is_restart_ready", 1
        )[1].split("static esp_err_t wait_for_search_mode", 1)[0]
        self.assertNotIn("PMTK011", restart_ready)
        self.assertIn("last_sentence_us", GPS_WORKER)
        self.assertIn(
            "target_sentence && gps_nmea_checksum_valid(line)", GPS_WORKER
        )
        pmtk314 = re.search(r'"(PMTK314,[0-9,]+)"', GPS_PLATFORM)
        self.assertIsNotNone(pmtk314)
        self.assertEqual(len(pmtk314.group(1).split(",")) - 1, 19)
        self.assertIn(".source_clk = UART_SCLK_XTAL", GPS_PLATFORM)
        self.assertNotIn(".source_clk = UART_SCLK_DEFAULT", GPS_PLATFORM)
        self.assertNotIn('"$PMTK001,%u,3*"', GPS_PLATFORM)

    def test_gps_pair_is_serialized_rmc_then_gga(self) -> None:
        pair = BLE_NUS_TX.split("static void set_gps_transaction", 1)[1].split(
            "static void start_transaction", 1
        )[0]
        self.assertLess(pair.index("rmc_length"), pair.index("gga_length"))
        self.assertIn("BLE_NUS_TX_MAX_SENTENCES", pair)
        self.assertIn("ble_nus_tx_offer_lk8ex1", BLE_WORKER)
        self.assertIn("ble_nus_tx_offer_gps", BLE_WORKER)
        self.assertIn("negotiated_att_mtu -", BLE_PLATFORM)
        self.assertIn("BLE_ATT_NOTIFICATION_OVERHEAD", BLE_PLATFORM)
        self.assertNotIn("ble_vario_notify_lk8ex1", BLE_PLATFORM)
        self.assertNotIn("ble_vario_notify_gps_pair", BLE_PLATFORM)

    def test_gps_fix_only_gates_led_for_installed_model(self) -> None:
        self.assertIn("bool gps_installed;", SYSTEM_POLICY_HEADER)
        self.assertIn("bool gps_fix_valid;", SYSTEM_POLICY_HEADER)
        self.assertIn(
            "(!input->gps_installed || input->gps_fix_valid)", SYSTEM_POLICY
        )
        self.assertIn(".gps_installed = identity != NULL", APP_WORKERS)
        self.assertIn(".gps_fix_valid = (bits & APP_EVENT_GPS_FIX_VALID)", APP_WORKERS)
        self.assertIn("APP_EVENT_GPS_FIX_VALID", GPS_WORKER)

    def test_gps_monitor_record_is_independent_from_baro(self) -> None:
        self.assertIn('"GPS installed=%d identified=%d', APP_WORKERS)
        self.assertIn("GPS_MONITOR_HEARTBEAT_US INT64_C(1000000)", APP_WORKERS)
        self.assertIn("console_write_gps_monitor_line", APP_WORKERS)
        self.assertNotIn("DIAG STATUS\r\n", APP_WORKERS)


if __name__ == "__main__":
    unittest.main()
