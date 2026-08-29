import unittest

from tools.monitor_gui.cloudbasevario_protocol import (
    DISPLAY_ERROR,
    DISPLAY_UNAVAILABLE,
    DISPLAY_WARNING,
    build_gps_view,
    build_telemetry_view,
    gps_stale_timeout_seconds,
    parse_gps_line,
    parse_telemetry_line,
)


class MonitorGuiGpsTests(unittest.TestCase):
    def _view(self, fields: str, *, stale: bool = False):
        sample = parse_gps_line("GPS " + fields)
        self.assertIsNotNone(sample)
        return build_gps_view(sample, stale=stale)

    def test_parser_is_strict_and_retains_unknown_fields(self) -> None:
        sample = parse_gps_line("GPS installed=1 future_field=value")
        self.assertIsNotNone(sample)
        self.assertEqual(sample.text("future_field"), "value")
        utc_sample = parse_gps_line("GPS utc_valid=1 utc=010203.00")
        self.assertEqual(utc_sample.text("utc"), "010203.00")
        self.assertIsNone(parse_gps_line("GPS installed=1 installed=0"))
        self.assertIsNone(parse_gps_line("GPS missing_value="))
        self.assertIsNone(parse_gps_line("GPS installed=maybe"))
        self.assertIsNone(parse_gps_line("GPS interval_ms=1.5"))
        self.assertIsNone(parse_gps_line("BARO installed=1"))

    def test_status_states(self) -> None:
        self.assertEqual(
            self._view("installed=0").status.value, "NOT INSTALLED"
        )
        error = self._view("installed=1 communicating=0")
        self.assertEqual(error.status.value, "ERROR")
        self.assertEqual(error.status.state, DISPLAY_ERROR)
        searching = self._view("installed=1 communicating=1 fix=0")
        self.assertEqual(searching.status.value, "SEARCHING")
        self.assertEqual(searching.status.state, DISPLAY_WARNING)
        self.assertEqual(
            self._view("installed=1 communicating=1 fix=1").status.value,
            "FIX",
        )
        self.assertEqual(
            self._view("installed=1 communicating=1 fix=1", stale=True)
            .status.value,
            "STALE",
        )

    def test_fix_validity_and_counter_warnings(self) -> None:
        view = self._view(
            "installed=1 identified=1 communicating=1 fix=0 "
            "utc_valid=1 utc=010203.00 position_valid=1 "
            "latitude_deg=35.0 longitude_deg=139.0 altitude_valid=1 "
            "altitude_m=12.0 satellites_valid=1 satellites=0 "
            "hdop_valid=1 hdop=9.9 speed_valid=1 speed_kmh=0.0 "
            "course_valid=1 course_deg=0.0 "
            "baud=115200 interval_ms=1000 age_ms=10 received=8 "
            "invalid=2 updates=0 retries=1 sent=0 dropped=3 "
            "last_error=ESP_ERR_TIMEOUT last_error_code=263"
        )
        items = {item.key: item for item in view.diagnostics.items}
        self.assertEqual(items["latitude_deg"].value, "--")
        self.assertEqual(items["altitude_m"].value, "--")
        self.assertEqual(items["speed_kmh"].value, "--")
        self.assertEqual(items["course_deg"].value, "--")
        self.assertEqual(items["satellites"].value, "0")
        self.assertEqual(items["invalid"].state, DISPLAY_WARNING)
        self.assertEqual(items["retries"].state, DISPLAY_WARNING)
        self.assertEqual(items["dropped"].state, DISPLAY_WARNING)
        self.assertEqual(items["last_error"].state, DISPLAY_ERROR)

    def test_missing_gps_record_does_not_change_baro_contract(self) -> None:
        sample = parse_telemetry_line(
            "BARO seq=1 online=1 pressure_valid=1 pressure_pa=101325"
        )
        self.assertIsNotNone(sample)
        view = build_telemetry_view(sample)
        self.assertEqual(view.statuses[0].value, "READY")
        self.assertEqual(
            next(item for item in view.flight if item.key == "altitude_m").state,
            DISPLAY_UNAVAILABLE,
        )

    def test_stale_timeout_tracks_configured_interval(self) -> None:
        fast = parse_gps_line("GPS interval_ms=200")
        default = parse_gps_line("GPS interval_ms=1000")
        slow = parse_gps_line("GPS interval_ms=10000")
        self.assertEqual(gps_stale_timeout_seconds(fast), 3.0)
        self.assertEqual(gps_stale_timeout_seconds(default), 3.0)
        self.assertEqual(gps_stale_timeout_seconds(slow), 30.0)


if __name__ == "__main__":
    unittest.main()
