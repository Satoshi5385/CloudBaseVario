import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MAIN_SOURCE = (ROOT / "SRC/app/startup.c").read_text(encoding="utf-8")
TASK_SOURCE = (ROOT / "SRC/app/app_workers.c").read_text(encoding="utf-8")
RESOURCE_HEADER = (ROOT / "SRC/app/app_events.h").read_text(
    encoding="utf-8"
)
SYSTEM_POLICY_HEADER = (ROOT / "SRC/domain/system_policy.h").read_text(
    encoding="utf-8"
)
CALIBRATION_CONTROLLER_SOURCE = (
    ROOT / "SRC/domain/imu_calibration_controller.c"
).read_text(encoding="utf-8")


class ImuCalibrationSkipPolicyTests(unittest.TestCase):
    def test_sw3_hold_threshold_is_three_seconds(self) -> None:
        match = re.search(
            r"^#define SYSTEM_POLICY_IMU_SKIP_HOLD_MS UINT32_C\((\d+)\)$",
            SYSTEM_POLICY_HEADER,
            re.MULTILINE,
        )

        self.assertIsNotNone(match)
        assert match is not None
        self.assertEqual(int(match.group(1)), 3000)

    def test_event_bits_fit_in_freertos_user_bit_range(self) -> None:
        self.assertIn(
            "APP_EVENT_IMU_ACCEL_CALIBRATION_REQUIRED BIT21",
            RESOURCE_HEADER,
        )
        self.assertIn(
            "APP_EVENT_IMU_ACCEL_CALIBRATION_SKIP_REQUEST BIT22",
            RESOURCE_HEADER,
        )
        self.assertIn(
            "APP_EVENT_IMU_ACCEL_CALIBRATION_SKIPPED BIT23",
            RESOURCE_HEADER,
        )

    def test_skip_cancels_pending_calibration_and_disables_imu(self) -> None:
        self.assertIn(
            "controller->save_pending = false;",
            CALIBRATION_CONTROLLER_SOURCE,
        )
        self.assertIn(
            "memset(&controller->pending, 0, sizeof(controller->pending));",
            CALIBRATION_CONTROLLER_SOURCE,
        )
        self.assertIn("controller->skipped = true;", CALIBRATION_CONTROLLER_SOURCE)
        self.assertIn(
            "imu_calibration_controller_request_skip(",
            TASK_SOURCE,
        )
        self.assertIn("icm42688_hxy_deinit();", TASK_SOURCE)
        self.assertIn(
            "APP_EVENT_IMU_ACCEL_CALIBRATION_SKIPPED",
            TASK_SOURCE,
        )

    def test_msc_gate_accepts_saved_or_skipped_completion(self) -> None:
        wait_position = MAIN_SOURCE.index(
            "APP_EVENT_IMU_ACCEL_CALIBRATION_SAVED |"
        )
        skipped_position = MAIN_SOURCE.index(
            "APP_EVENT_IMU_ACCEL_CALIBRATION_SKIPPED |",
            wait_position,
        )
        msc_position = MAIN_SOURCE.index("usb_device_enable_msc();")

        self.assertLess(wait_position, skipped_position)
        self.assertLess(skipped_position, msc_position)

    def test_skip_is_reported_in_telemetry(self) -> None:
        self.assertIn("imu_accel_cal_skipped=%d", TASK_SOURCE)


if __name__ == "__main__":
    unittest.main()
