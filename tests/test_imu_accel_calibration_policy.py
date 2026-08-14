import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FUSION_SOURCE = (ROOT / "SRC/domain/imu_fusion.c").read_text(encoding="utf-8")
FUSION_HEADER = (ROOT / "SRC/domain/imu_fusion.h").read_text(encoding="utf-8")
STORAGE_SOURCE = (ROOT / "SRC/platform/imu_calibration_storage.c").read_text(
    encoding="utf-8"
)


def source_float_constant(name: str) -> float:
    match = re.search(
        rf"^#define\s+{re.escape(name)}\s+([0-9.]+)f$",
        FUSION_SOURCE,
        re.MULTILINE,
    )
    if match is None:
        raise AssertionError(f"missing float constant: {name}")
    return float(match.group(1))


def source_sample_count() -> int:
    match = re.search(
        r"^#define\s+IMU_ACCEL_CALIBRATION_SAMPLE_COUNT\s+UINT32_C\((\d+)\)$",
        FUSION_HEADER,
        re.MULTILINE,
    )
    if match is None:
        raise AssertionError("missing accelerometer calibration sample count")
    return int(match.group(1))


class ImuAccelCalibrationPolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.xy_max_g = source_float_constant("IMU_FACTORY_LEVEL_XY_MAX_G")
        self.z_min_g = source_float_constant("IMU_FACTORY_POSE_Z_MIN_G")
        self.z_max_g = source_float_constant("IMU_FACTORY_POSE_Z_MAX_G")
        self.offset_max_g = source_float_constant("IMU_ACCEL_OFFSET_MAX_G")
        self.sample_count = source_sample_count()

    def pose_accepts(self, x_g: float, y_g: float, z_g: float) -> bool:
        return (
            abs(x_g) <= self.xy_max_g
            and abs(y_g) <= self.xy_max_g
            and self.z_min_g <= z_g <= self.z_max_g
        )

    def calibration_offset_is_valid(self, mean_g: tuple[float, ...]) -> bool:
        expected_g = (0.0, 0.0, 1.0)
        return all(
            math.isfinite(value)
            and abs(value - expected) <= self.offset_max_g
            for value, expected in zip(mean_g, expected_g)
        )

    def test_acquisition_gate_is_coarser_than_final_offset_validation(self) -> None:
        self.assertEqual(self.z_min_g, 0.75)
        self.assertEqual(self.z_max_g, 1.25)
        self.assertEqual(self.offset_max_g, 0.20)

    def test_measured_z_offset_can_complete_800_consecutive_samples(self) -> None:
        samples = [1.145, 1.153] * (self.sample_count // 2)

        self.assertEqual(len(samples), 800)
        self.assertTrue(all(self.pose_accepts(-0.005, 0.011, z) for z in samples))
        mean_z_g = sum(samples) / len(samples)
        self.assertAlmostEqual(mean_z_g - 1.0, 0.149, places=6)
        self.assertTrue(self.calibration_offset_is_valid((-0.005, 0.011, mean_z_g)))

    def test_out_of_range_offset_is_rejected_after_collection(self) -> None:
        self.assertTrue(self.pose_accepts(0.0, 0.0, 1.21))
        self.assertFalse(self.calibration_offset_is_valid((0.0, 0.0, 1.21)))

    def test_inverted_board_is_rejected_by_pose_gate(self) -> None:
        self.assertFalse(self.pose_accepts(0.04, 0.0, -0.86))

    def test_mc_data_keeps_only_model_and_offsets_as_calibration_keys(self) -> None:
        match = re.search(
            r"calibration_keys\[\]\s*=\s*\{([^}]*)\}", STORAGE_SOURCE
        )
        self.assertIsNotNone(match)
        self.assertEqual(
            re.findall(r'"([a-z0-9_]+)"', match.group(1)),
            ["model", "offset_mps2"],
        )

    def test_mc_data_model_hardcodes_calibration_contract(self) -> None:
        self.assertIn("#define MC_DATA_FORMAT_VERSION 2", STORAGE_SOURCE)
        self.assertRegex(STORAGE_SOURCE, r'\.model\s*=\s*"ICM-42688P-HXY"')
        self.assertRegex(
            STORAGE_SOURCE,
            r"\.who_am_i\s*=\s*ICM42688_HXY_WHO_AM_I_VALUE",
        )
        self.assertRegex(STORAGE_SOURCE, r'\.coordinate\s*=\s*"SENSOR"')
        self.assertRegex(STORAGE_SOURCE, r'\.method\s*=\s*"LEVEL_Z_UP"')
        self.assertRegex(
            STORAGE_SOURCE,
            r"\.sample_count\s*=\s*IMU_ACCEL_CALIBRATION_SAMPLE_COUNT",
        )


if __name__ == "__main__":
    unittest.main()
