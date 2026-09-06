import unittest

from tools.monitor_gui.cloudbasevario_protocol import (
    DISPLAY_UNAVAILABLE,
    DISPLAY_WARNING,
    build_telemetry_view,
    parse_telemetry_line,
)


class MonitorGuiMotionTests(unittest.TestCase):
    def test_motion_status_and_diagnostics(self) -> None:
        sample = parse_telemetry_line(
            "BARO online=1 pressure_valid=1 estimate_valid=1 climb_valid=1 "
            "motion_state=FLYING motion_evidence=5 "
            "motion_altitude_evidence=1 motion_climb_evidence=0 "
            "motion_gps_evidence=1 motion_imu_evidence=0 "
            "motion_state_elapsed_s=12 stationary_elapsed_s=0 "
            "motion_altitude_range_m=10.5 motion_vario_used=1 "
            "motion_gps_used=1 motion_imu_used=0 imu_motion_valid=1 "
            "imu_motion_accel_rms_g=0.0312 "
            "imu_motion_gyro_rms_dps=11.25"
        )
        self.assertIsNotNone(sample)
        view = build_telemetry_view(sample)
        motion_status = next(item for item in view.statuses if item.key == "motion")
        self.assertEqual(motion_status.value, "FLYING")
        group = next(group for group in view.diagnostics if group.key == "motion")
        items = {item.key: item for item in group.items}
        self.assertEqual(items["motion_evidence"].value, "5")
        self.assertEqual(items["motion_altitude_range_m"].value, "10.50 m")
        self.assertEqual(items["imu_motion_gyro_rms_dps"].value, "11.250 deg/s")

    def test_unknown_and_missing_motion_status(self) -> None:
        unknown = parse_telemetry_line("BARO motion_state=UNKNOWN")
        self.assertIsNotNone(unknown)
        status = next(
            item for item in build_telemetry_view(unknown).statuses
            if item.key == "motion"
        )
        self.assertEqual(status.state, DISPLAY_WARNING)

        missing = parse_telemetry_line("BARO online=1 pressure_valid=1")
        self.assertIsNotNone(missing)
        status = next(
            item for item in build_telemetry_view(missing).statuses
            if item.key == "motion"
        )
        self.assertEqual(status.state, DISPLAY_UNAVAILABLE)


if __name__ == "__main__":
    unittest.main()
