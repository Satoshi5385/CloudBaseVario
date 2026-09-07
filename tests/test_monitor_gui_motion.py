import unittest
from pathlib import Path

from tools.monitor_gui.cloudbasevario_protocol import (
    DISPLAY_UNAVAILABLE,
    DISPLAY_WARNING,
    build_telemetry_view,
    parse_telemetry_line,
)


ROOT = Path(__file__).resolve().parents[1]
GUI_APP = (
    ROOT / "tools" / "monitor_gui" / "cloudbasevario_gui_app.py"
).read_text(encoding="utf-8")


class MonitorGuiMotionTests(unittest.TestCase):
    def test_gui_exposes_motion_badge_and_diagnostics(self) -> None:
        self.assertIn('(\"motion\", \"MOTION\")', GUI_APP)
        self.assertIn(
            '\"motion\": DiagnosticTable(container, '
            '\"Flight / stationary state\")',
            GUI_APP,
        )

    def test_motion_status_and_diagnostics(self) -> None:
        sample = parse_telemetry_line(
            "BARO online=1 pressure_valid=1 estimate_valid=1 climb_valid=1 "
            "motion_state=FLYING motion_evidence=3 "
            "motion_altitude_evidence=1 motion_gps_evidence=1 "
            "motion_state_elapsed_s=12 stationary_elapsed_s=0 "
            "motion_altitude_range_m=5.5 motion_vario_used=1 "
            "motion_gps_used=1 motion_gps_high_elapsed_s=2 "
            "motion_gps_high_updates=2 motion_gps_high_pending=0 "
            "auto_power_off_elapsed_s=0"
        )
        self.assertIsNotNone(sample)
        view = build_telemetry_view(sample)
        motion_status = next(item for item in view.statuses if item.key == "motion")
        self.assertEqual(motion_status.value, "FLYING")
        group = next(group for group in view.diagnostics if group.key == "motion")
        items = {item.key: item for item in group.items}
        self.assertEqual(items["motion_evidence"].value, "3")
        self.assertEqual(items["motion_altitude_range_m"].value, "5.50 m")
        self.assertEqual(items["motion_gps_high_updates"].value, "2")

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
