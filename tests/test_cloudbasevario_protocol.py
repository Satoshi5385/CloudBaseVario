import unittest

from tools.cloudbasevario_protocol import (
    DISPLAY_ERROR,
    DISPLAY_UNAVAILABLE,
    DISPLAY_WARNING,
    DisplayItem,
    TelemetryViewModel,
    build_telemetry_view,
    format_command,
    parse_parameter_line,
    parse_telemetry_line,
)


class TelemetryParserTests(unittest.TestCase):
    def test_parses_numeric_and_status_fields(self) -> None:
        sample = parse_telemetry_line(
            "BARO seq=125 timestamp_us=12345678 online=1 "
            "pressure_pa=101325.25 climb_mps=-0.125 "
            "ble_battery=3.95 filter=AUTO"
        )

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample.integer("seq"), 125)
        self.assertTrue(sample.flag("online"))
        self.assertAlmostEqual(sample.number("pressure_pa"), 101325.25)
        self.assertAlmostEqual(sample.number("climb_mps"), -0.125)
        self.assertEqual(sample.text("filter"), "AUTO")

    def test_preserves_ble_invalid_sentinels_as_numeric_values(self) -> None:
        sample = parse_telemetry_line(
            "BARO ble_pressure_pa=999999 ble_vario_cm_s=9999 "
            "ble_battery=999"
        )

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample.integer("ble_pressure_pa"), 999999)
        self.assertEqual(sample.text("ble_battery"), "999")

    def test_accepts_first_boot_imu_calibration_diagnostics(self) -> None:
        sample = parse_telemetry_line(
            "BARO imu_accel_calibrated=0 imu_accel_cal_persisted=0 "
            "imu_accel_cal_skipped=0 "
            "kalman_accel_bias_mps2=-0.1120 "
            "kalman_baro_innovation_m=0.0320 "
            "kalman_baro_innovation_valid=1 "
            "kalman_accel_innovation_mps2=-0.0180 "
            "kalman_accel_innovation_valid=1 "
            "kalman_baro_r_m2=0.04000 "
            "kalman_accel_r_m2_s4=0.48650 "
            "imu_confidence=0.875 imu_vibration_rms_g=0.0065 "
            "imu_kp_effective=4.375 imu_ki_effective=0.04375 "
            "imu_ki_active=1 imu_cal_samples=127 "
            "imu_cal_save_pending=0 imu_cal_storage=MISSING "
            "imu_cal_storage_error=0"
        )

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertFalse(sample.flag("imu_accel_calibrated"))
        self.assertFalse(sample.flag("imu_accel_cal_persisted"))
        self.assertFalse(sample.flag("imu_accel_cal_skipped"))
        self.assertAlmostEqual(sample.number("kalman_accel_bias_mps2"), -0.112)
        self.assertAlmostEqual(sample.number("kalman_baro_innovation_m"), 0.032)
        self.assertTrue(sample.flag("kalman_baro_innovation_valid"))
        self.assertAlmostEqual(
            sample.number("kalman_accel_innovation_mps2"), -0.018
        )
        self.assertTrue(sample.flag("kalman_accel_innovation_valid"))
        self.assertAlmostEqual(sample.number("kalman_baro_r_m2"), 0.04)
        self.assertAlmostEqual(sample.number("kalman_accel_r_m2_s4"), 0.4865)
        self.assertAlmostEqual(sample.number("imu_confidence"), 0.875)
        self.assertAlmostEqual(sample.number("imu_vibration_rms_g"), 0.0065)
        self.assertTrue(sample.flag("imu_ki_active"))
        self.assertEqual(sample.integer("imu_cal_samples"), 127)
        self.assertEqual(sample.text("imu_cal_storage"), "MISSING")

    def test_rejects_non_telemetry_and_ambiguous_fields(self) -> None:
        self.assertIsNone(parse_telemetry_line("I (10) boot"))
        self.assertIsNone(parse_telemetry_line("BARO seq=1 seq=2"))
        self.assertIsNone(parse_telemetry_line("BARO malformed"))


class TelemetryPresentationTests(unittest.TestCase):
    @staticmethod
    def _item(view: TelemetryViewModel, key: str) -> DisplayItem:
        for item in view.flight + view.statuses:
            if item.key == key:
                return item
        for group in view.diagnostics:
            for item in group.items:
                if item.key == key:
                    return item
        raise AssertionError(f"missing display item: {key}")

    def test_interprets_the_current_complete_baro_contract(self) -> None:
        sample = parse_telemetry_line(
            "BARO seq=125 timestamp_us=12345678 online=1 pressure_valid=1 "
            "raw_temp=-321 raw_pressure=6640432 temp_c=21.75 "
            "pressure_pa=101325.25 altitude_m=124.50 climb_mps=-0.125 "
            "climb_valid=1 estimate_valid=1 i2c_errors=0 overruns=0 "
            "ble_pressure_pa=101325 ble_altitude_m=125 ble_vario_cm_s=-12 "
            "ble_temperature_c=22 ble_battery=3.95 ble_available=1 "
            "ble_notify=1 imu_online=1 imu_calibrated=1 "
            "imu_attitude_valid=1 imu_accel_calibrated=1 "
            "imu_accel_cal_persisted=1 imu_accel_cal_skipped=0 imu_stale=0 "
            "q_w=1.00000 q_x=0.00000 q_y=0.00000 q_z=0.00000 "
            "roll_deg=1.25 pitch_deg=-2.50 yaw_deg=3.75 "
            "vertical_accel_mps2=0.010 vertical_accel_valid=1 fusion_active=1 "
            "kalman_accel_bias_mps2=-0.1120 "
            "kalman_baro_innovation_m=0.0320 "
            "kalman_baro_innovation_valid=1 "
            "kalman_accel_innovation_mps2=-0.0180 "
            "kalman_accel_innovation_valid=1 kalman_baro_r_m2=0.04000 "
            "kalman_accel_r_m2_s4=0.48650 imu_samples=4000 imu_missed=0 "
            "imu_confidence=0.875 imu_vibration_rms_g=0.0065 "
            "imu_kp_effective=4.3750 imu_ki_effective=0.0438 "
            "imu_ki_active=1 imu_cal_samples=800 imu_cal_save_pending=0 "
            "imu_cal_storage=VALID imu_cal_storage_error=0 stream_drops=0 "
            "future_field=retained"
        )

        self.assertIsNotNone(sample)
        assert sample is not None
        view = build_telemetry_view(sample)

        self.assertEqual(self._item(view, "pressure_pa").value, "101325.25 Pa")
        self.assertEqual(self._item(view, "temp_c").value, "21.75 °C")
        self.assertEqual(self._item(view, "imu_confidence").value, "87.5 %")
        self.assertEqual(
            self._item(view, "kalman_baro_innovation_m").value,
            "0.0320 m",
        )
        self.assertEqual(self._item(view, "calibration").value, "READY")
        self.assertEqual(self._item(view, "stream").value, "CLEAN")
        self.assertEqual(self._item(view, "ble_notify").value, "NOTIFY")
        self.assertEqual(sample.text("future_field"), "retained")

    def test_marks_invalid_sentinels_and_calibration_failures(self) -> None:
        sample = parse_telemetry_line(
            "BARO online=1 pressure_valid=0 estimate_valid=0 climb_valid=0 "
            "vertical_accel_valid=0 ble_pressure_pa=999999 "
            "ble_altitude_m=99999 ble_vario_cm_s=9999 "
            "ble_temperature_c=99 ble_battery=999 ble_available=0 "
            "ble_notify=0 imu_online=1 imu_accel_cal_skipped=0 "
            "imu_cal_save_pending=0 imu_cal_storage_error=-5 stream_drops=3"
        )

        self.assertIsNotNone(sample)
        assert sample is not None
        view = build_telemetry_view(sample)

        self.assertEqual(self._item(view, "pressure_pa").state, DISPLAY_UNAVAILABLE)
        self.assertEqual(self._item(view, "ble_pressure_pa").value, "--")
        self.assertEqual(self._item(view, "ble_battery").state, DISPLAY_UNAVAILABLE)
        self.assertEqual(self._item(view, "calibration").state, DISPLAY_ERROR)
        self.assertEqual(self._item(view, "stream").state, DISPLAY_WARNING)

    def test_reports_calibration_skip_without_presenting_it_as_ready(self) -> None:
        sample = parse_telemetry_line(
            "BARO imu_online=0 imu_accel_cal_skipped=1 "
            "imu_cal_save_pending=0 imu_cal_storage_error=0"
        )

        self.assertIsNotNone(sample)
        assert sample is not None
        view = build_telemetry_view(sample)

        self.assertEqual(self._item(view, "calibration").value, "SKIPPED")
        self.assertEqual(self._item(view, "calibration").state, DISPLAY_WARNING)


class ConsoleProtocolTests(unittest.TestCase):
    def test_parses_parameter_response(self) -> None:
        self.assertEqual(
            parse_parameter_line("sink_freq_start_hz=300"),
            ("sink_freq_start_hz", "300"),
        )
        self.assertEqual(
            parse_parameter_line("filter_mode=BARO_ONLY"),
            ("filter_mode", "BARO_ONLY"),
        )

    def test_rejects_logs_as_parameter_responses(self) -> None:
        self.assertIsNone(parse_parameter_line("BARO seq=10"))
        self.assertIsNone(parse_parameter_line("OK"))
        self.assertIsNone(parse_parameter_line("bad name=1"))

    def test_formats_one_ascii_command(self) -> None:
        self.assertEqual(
            format_command(" PARAM SET predictive_buzzer_enabled true "),
            b"PARAM SET predictive_buzzer_enabled true\r\n",
        )

    def test_rejects_multiline_and_non_ascii_commands(self) -> None:
        with self.assertRaises(ValueError):
            format_command("PARAM LIST\nPARAM SAVE")
        with self.assertRaises(ValueError):
            format_command("PARAM GET 音声")


if __name__ == "__main__":
    unittest.main()
