import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
APP_CONFIG_HEADER = (ROOT / "SRC/domain/app_config.h").read_text(encoding="utf-8")
APP_CONFIG_SOURCE = (ROOT / "SRC/domain/app_config.c").read_text(encoding="utf-8")
BOARD_SOURCE = (ROOT / "SRC/platform/board.c").read_text(encoding="utf-8")
CONFIG_STORAGE_SOURCE = (ROOT / "SRC/platform/config_storage.c").read_text(
    encoding="utf-8"
)
FUSION_SOURCE = (ROOT / "SRC/domain/imu_fusion.c").read_text(encoding="utf-8")

AXIS_KEYS = (
    "imu_accel_x_source",
    "imu_accel_y_source",
    "imu_accel_z_source",
    "imu_accel_x_sign",
    "imu_accel_y_sign",
    "imu_accel_z_sign",
    "imu_gyro_x_source",
    "imu_gyro_y_source",
    "imu_gyro_z_source",
    "imu_gyro_x_sign",
    "imu_gyro_y_sign",
    "imu_gyro_z_sign",
)


class BoardImuAxisMapPolicyTests(unittest.TestCase):
    def test_axis_map_is_not_a_runtime_or_json_parameter(self) -> None:
        for key in AXIS_KEYS:
            self.assertNotIn(key, APP_CONFIG_HEADER)
            self.assertNotIn(key, APP_CONFIG_SOURCE)

    def test_old_json_axis_keys_are_not_migrated_in_version_4(self) -> None:
        for key in AXIS_KEYS:
            self.assertNotIn(f'"{key}"', CONFIG_STORAGE_SOURCE)
        self.assertNotIn("legacy_board_axis_parameter_index", CONFIG_STORAGE_SOURCE)

    def test_aohazuku_rev0_uses_identity_positive_axis_map(self) -> None:
        initializer = re.search(
            r"static const imu_axis_map_t aohazuku_rev0_imu_axis_map = "
            r"\{(?P<body>.*?)\n\};",
            BOARD_SOURCE,
            re.DOTALL,
        )
        self.assertIsNotNone(initializer)
        body = initializer.group("body")
        self.assertIn(".accel_source = {0U, 1U, 2U}", body)
        self.assertIn(".gyro_source = {0U, 1U, 2U}", body)
        self.assertIn(".accel_sign = {1.0f, 1.0f, 1.0f}", body)
        self.assertIn(".gyro_sign = {1.0f, 1.0f, 1.0f}", body)
        self.assertIn(
            "imu_axis_map_validate(&aohazuku_rev0_imu_axis_map)", BOARD_SOURCE
        )

    def test_common_axis_transform_logic_remains(self) -> None:
        self.assertIn("bool imu_axis_map_validate", FUSION_SOURCE)
        self.assertIn("bool imu_fusion_apply_axis_map", FUSION_SOURCE)
        self.assertIn("axis_map->accel_source[index]", FUSION_SOURCE)
        self.assertIn("axis_map->accel_sign[index]", FUSION_SOURCE)
        self.assertIn("axis_map->gyro_source[index]", FUSION_SOURCE)
        self.assertIn("axis_map->gyro_sign[index]", FUSION_SOURCE)


if __name__ == "__main__":
    unittest.main()
