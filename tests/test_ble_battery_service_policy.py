import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BLE_SOURCE = "\n".join(
    (ROOT / path).read_text(encoding="utf-8")
    for path in (
        "SRC/platform/ble_vario.c",
        "SRC/domain/battery_level.c",
        "SRC/domain/lk8ex1.c",
    )
)
BLE_HEADER = (ROOT / "SRC/platform/ble_vario.h").read_text(encoding="utf-8")
BATTERY_HEADER = (ROOT / "SRC/domain/battery_level.h").read_text(encoding="utf-8")
TASK_SOURCE = "\n".join(
    (ROOT / path).read_text(encoding="utf-8")
    for path in ("SRC/app/app_workers.c", "SRC/app/ble_tx_worker.c")
)
BLE_WORKER_SOURCE = (ROOT / "SRC/app/ble_tx_worker.c").read_text(
    encoding="utf-8"
)
SDKCONFIG_DEFAULTS = (ROOT / "sdkconfig.defaults").read_text(encoding="utf-8")
BLE_SPEC = (ROOT / "DOC/BLE_IF.md").read_text(encoding="utf-8")


def battery_level_from_voltage(voltage: float) -> int:
    curve = (
        (3.20, 0),
        (3.50, 10),
        (3.60, 20),
        (3.70, 40),
        (3.80, 60),
        (3.90, 80),
        (4.10, 100),
    )
    if not math.isfinite(voltage) or voltage <= curve[0][0]:
        return 0
    if voltage >= curve[-1][0]:
        return 100
    for lower, upper in zip(curve, curve[1:]):
        if voltage <= upper[0]:
            level = lower[1] + (
                (voltage - lower[0]) * (upper[1] - lower[1])
                / (upper[0] - lower[0])
            )
            return math.floor(level + 0.5)
    return 100


def battery_level_status(external_power_present: bool) -> bytes:
    battery_present = 0x0001
    wired_external = 0x0002
    charging = 0x0020
    discharging_active = 0x0040
    power_state = battery_present
    if external_power_present:
        power_state |= wired_external | charging
    else:
        power_state |= discharging_active
    return bytes((0, power_state & 0xFF, power_state >> 8))


def update_battery_level(previous: int, voltage: float, valid: bool) -> int:
    if valid and math.isfinite(voltage) and voltage >= 0.0:
        return battery_level_from_voltage(voltage)
    return previous


class BleBatteryServicePolicyTests(unittest.TestCase):
    def test_voltage_to_percent_contract(self) -> None:
        self.assertEqual(battery_level_from_voltage(2.9), 0)
        self.assertEqual(battery_level_from_voltage(3.2), 0)
        self.assertEqual(battery_level_from_voltage(3.5), 10)
        self.assertEqual(battery_level_from_voltage(3.6), 20)
        self.assertEqual(battery_level_from_voltage(3.7), 40)
        self.assertEqual(battery_level_from_voltage(3.8), 60)
        self.assertEqual(battery_level_from_voltage(3.9), 80)
        self.assertEqual(battery_level_from_voltage(4.0), 90)
        self.assertEqual(battery_level_from_voltage(4.1), 100)
        self.assertEqual(battery_level_from_voltage(4.2), 100)
        self.assertEqual(battery_level_from_voltage(4.3), 100)
        self.assertEqual(battery_level_from_voltage(float("nan")), 0)
        self.assertEqual(update_battery_level(0, 3.55, True), 15)
        self.assertEqual(update_battery_level(50, float("nan"), True), 50)
        self.assertEqual(update_battery_level(50, 4.2, False), 50)
        for point in ("{3.20f, 0U}", "{3.50f, 10U}", "{3.60f, 20U}",
                      "{3.70f, 40U}", "{3.80f, 60U}", "{3.90f, 80U}",
                      "{4.10f, 100U}"):
            self.assertIn(point, BLE_SOURCE)
        self.assertIn("lroundf(level)", BLE_SOURCE)
        self.assertIn(
            "return battery_level_percent_from_voltage(battery_voltage_v);",
            BLE_SOURCE,
        )
        self.assertRegex(
            BLE_SOURCE,
            re.compile(
                r"LK8EX1_BATTERY_PERCENT_OFFSET\s*\+\s*"
                r"battery_level_percent_from_voltage\(\s*"
                r"system->battery_display_voltage_v\)",
                re.DOTALL,
            ),
        )

    def test_display_uses_30_second_minimum_and_holds_invalid(self) -> None:
        self.assertIn(
            "BATTERY_DISPLAY_UPDATE_INTERVAL_US INT64_C(30000000)",
            BATTERY_HEADER,
        )
        self.assertIn("battery_display_update(", BLE_SOURCE)
        self.assertIn("sample_voltage_v < state->window_min_voltage_v", BLE_SOURCE)
        self.assertIn("snapshot.battery_display_valid = battery_display_update(", TASK_SOURCE)
        self.assertIn("system->battery_display_valid", BLE_SOURCE)
        self.assertIn("system->battery_display_voltage_v", BLE_SOURCE)
        self.assertRegex(
            BLE_SOURCE,
            re.compile(
                r"if \(level_valid\) \{\s*"
                r"level_changed = battery_level_percent != next_level;\s*"
                r"battery_level_percent = next_level;",
                re.DOTALL,
            ),
        )

    def test_gss_power_state_values(self) -> None:
        self.assertEqual(battery_level_status(False), bytes.fromhex("00 41 00"))
        self.assertEqual(battery_level_status(True), bytes.fromhex("00 23 00"))
        self.assertIn("UINT16_C(0x0023)", BLE_SOURCE)
        self.assertIn("UINT16_C(0x0041)", BLE_SOURCE)
        self.assertIn("status[0] = 0U;", BLE_SOURCE)

    def test_gatt_service_and_cccd_contract(self) -> None:
        self.assertIn("BATTERY_SERVICE_UUID UINT16_C(0x180F)", BLE_SOURCE)
        self.assertIn("BATTERY_LEVEL_UUID UINT16_C(0x2A19)", BLE_SOURCE)
        self.assertIn("BATTERY_LEVEL_STATUS_UUID UINT16_C(0x2BED)", BLE_SOURCE)
        self.assertRegex(
            BLE_SOURCE,
            re.compile(
                r"\.uuid = &battery_level_uuid\.u,.*?"
                r"\.flags = BLE_GATT_CHR_F_READ \| BLE_GATT_CHR_F_NOTIFY,.*?"
                r"\.val_handle = &battery_level_value_handle,",
                re.DOTALL,
            ),
        )
        self.assertRegex(
            BLE_SOURCE,
            re.compile(
                r"\.uuid = &battery_level_status_uuid\.u,.*?"
                r"\.flags = BLE_GATT_CHR_F_READ \| BLE_GATT_CHR_F_NOTIFY,",
                re.DOTALL,
            ),
        )
        self.assertIn("CONFIG_BT_NIMBLE_MAX_CCCDS=3", SDKCONFIG_DEFAULTS)
        self.assertIn("#if CONFIG_BT_NIMBLE_BAS_SERVICE", BLE_SOURCE)
        self.assertIn("ble_gatts_count_cfg(nus_services)", BLE_SOURCE)
        self.assertIn("ble_gatts_count_cfg(battery_services)", BLE_SOURCE)
        self.assertIn(
            "scan_response_fields.uuids16 = (ble_uuid16_t *) &battery_service_uuid;",
            BLE_SOURCE,
        )

    def test_battery_updates_do_not_require_nus_subscription(self) -> None:
        update = TASK_SOURCE.index("ble_vario_update_battery(&system);")
        can_notify = TASK_SOURCE.index("ble_vario_can_notify()", update)
        self.assertLess(update, can_notify)
        self.assertIn("if (app_resources_copy_system(&system))", TASK_SOURCE)
        self.assertIn("ble_gatts_chr_updated(status_handle);", BLE_SOURCE)
        self.assertIn("notify_status = status_changed && nimble_initialized", BLE_SOURCE)
        self.assertIn("notify_level = level_changed && nimble_initialized", BLE_SOURCE)
        self.assertIn("ble_gatts_chr_updated(level_handle);", BLE_SOURCE)

    def test_lk8ex1_rate_uses_independent_absolute_deadline(self) -> None:
        self.assertIn(
            "#define BATTERY_UPDATE_PERIOD_US INT64_C(1000000)",
            BLE_WORKER_SOURCE,
        )
        self.assertNotIn("BLE_TX_PERIOD_MS", BLE_WORKER_SOURCE)
        self.assertIn(
            "MICROSECONDS_PER_SECOND / (int64_t) rate_hz",
            BLE_WORKER_SOURCE,
        )
        self.assertEqual(1_000_000 // 1, 1_000_000)
        self.assertEqual(1_000_000 // 10, 100_000)
        self.assertEqual(1_000_000 // 50, 20_000)
        self.assertIn(
            "periods_elapsed = (now_us - deadline_us) / period_us;",
            BLE_WORKER_SOURCE,
        )
        self.assertIn(
            "deadline_us + (periods_elapsed + 1) * period_us",
            BLE_WORKER_SOURCE,
        )
        self.assertIn("ulTaskNotifyTake(", BLE_WORKER_SOURCE)
        self.assertIn("can_notify));", BLE_WORKER_SOURCE)
        self.assertNotIn("vTaskDelay(", BLE_WORKER_SOURCE)

    def test_gap_state_changes_wake_the_ble_tx_worker(self) -> None:
        self.assertIn("ble_vario_set_tx_wakeup_task", BLE_HEADER)
        self.assertIn("xTaskNotifyGive(tx_wakeup_task);", BLE_SOURCE)
        self.assertIn(
            "ble_vario_set_tx_wakeup_task(xTaskGetCurrentTaskHandle());",
            BLE_WORKER_SOURCE,
        )
        self.assertIn(
            "ble_vario_set_tx_wakeup_task(NULL);", BLE_WORKER_SOURCE
        )
        self.assertIn(
            "config.bluetooth_notify_rate_hz != previous_notify_rate_hz",
            BLE_WORKER_SOURCE,
        )
        battery_update = BLE_WORKER_SOURCE.index("ble_vario_update_battery(&system);")
        lk8ex1_notify = BLE_WORKER_SOURCE.index("ble_vario_notify_lk8ex1(")
        self.assertLess(battery_update, lk8ex1_notify)

    def test_tx_power_presets_cover_advertising_and_connections(self) -> None:
        for preset, level in (
            ("APP_BLUETOOTH_TX_POWER_MIN", "ESP_PWR_LVL_N24"),
            ("APP_BLUETOOTH_TX_POWER_LOW", "ESP_PWR_LVL_N12"),
            ("APP_BLUETOOTH_TX_POWER_NORMAL", "ESP_PWR_LVL_N0"),
            ("APP_BLUETOOTH_TX_POWER_HIGH", "ESP_PWR_LVL_P9"),
        ):
            self.assertIn(f"case {preset}:", BLE_SOURCE)
            self.assertIn(f"*level = {level};", BLE_SOURCE)
        self.assertNotIn("ESP_PWR_LVL_P20", BLE_SOURCE)
        self.assertIn("ESP_BLE_PWR_TYPE_DEFAULT", BLE_SOURCE)
        self.assertIn("ESP_BLE_PWR_TYPE_ADV", BLE_SOURCE)
        self.assertIn("ESP_BLE_ENHANCED_PWR_TYPE_CONN", BLE_SOURCE)
        self.assertIn(
            "config_revision != previous_config_revision", BLE_WORKER_SOURCE
        )
        self.assertIn(
            "ble_vario_apply_tx_power(config.bluetooth_tx_power)",
            BLE_WORKER_SOURCE,
        )
        connect = BLE_SOURCE.index("case BLE_GAP_EVENT_CONNECT:")
        disconnect = BLE_SOURCE.index("case BLE_GAP_EVENT_DISCONNECT:")
        self.assertIn(
            "ble_vario_apply_tx_power(tx_power)", BLE_SOURCE[connect:disconnect]
        )

    def test_lk8ex1_percent_uses_protocol_offset(self) -> None:
        self.assertIn(
            "#define LK8EX1_BATTERY_PERCENT_OFFSET UINT16_C(1000)",
            BLE_SOURCE,
        )
        self.assertIn(
            "LK8EX1_BATTERY_PERCENT_OFFSET +",
            BLE_SOURCE,
        )

    def test_lk8ex1_wire_format_is_not_extended(self) -> None:
        self.assertIn(
            '"LK8EX1,%s,%s,%s,%s,%s,"',
            BLE_SOURCE,
        )
        self.assertIn("LK8EX1には充電状態を示す標準フィールドがない", BLE_SPEC)
        self.assertIn("BLE_VARIO_BATTERY_LEVEL_STATUS_SIZE 3U", BLE_HEADER)


if __name__ == "__main__":
    unittest.main()
