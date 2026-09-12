import unittest

from tools.monitor_gui.cloudbasevario_protocol import (
    DISPLAY_WARNING,
    build_telemetry_view,
    parse_telemetry_line,
)


class MonitorGuiImuBufferTests(unittest.TestCase):
    def test_buffer_usage_and_overflow_are_reported(self) -> None:
        sample = parse_telemetry_line(
            "BARO imu_online=1 imu_buffer_high_watermark=3 "
            "imu_buffer_overflows=2 imu_confidence=0.75"
        )
        self.assertIsNotNone(sample)
        view = build_telemetry_view(sample)
        group = next(group for group in view.diagnostics if group.key == "imu")
        items = {item.key: item for item in group.items}

        self.assertEqual(items["imu_buffer_high_watermark"].value, "3 samples")
        self.assertEqual(items["imu_buffer_overflows"].value, "2")
        self.assertEqual(items["imu_buffer_overflows"].state, DISPLAY_WARNING)
        self.assertEqual(items["imu_confidence"].value, "75.0 %")


if __name__ == "__main__":
    unittest.main()
