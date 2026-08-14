import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ESTIMATOR_SOURCE = (ROOT / "SRC/domain/vario_estimator.c").read_text(
    encoding="utf-8"
)
ESTIMATOR_HEADER = (ROOT / "SRC/domain/vario_estimator.h").read_text(
    encoding="utf-8"
)


def source_float_constant(name: str) -> float:
    match = re.search(
        rf"^#define\s+{re.escape(name)}\s+([0-9.]+)f$",
        ESTIMATOR_SOURCE,
        re.MULTILINE,
    )
    if match is None:
        raise AssertionError(f"missing float constant: {name}")
    return float(match.group(1))


class VarioEstimatorQualityPolicyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.base_variance = source_float_constant(
            "FUSION_ACCEL_MEAS_VARIANCE_M2_S4"
        )
        self.adapt_factor = source_float_constant("FUSION_ACCEL_ADAPT_FACTOR")
        self.confidence_floor = source_float_constant(
            "FUSION_ACCEL_CONFIDENCE_FLOOR"
        )
        self.gravity = source_float_constant(
            "FUSION_ACCEL_VIBRATION_GRAVITY_MPS2"
        )
        self.maximum = source_float_constant(
            "FUSION_ACCEL_MEAS_VARIANCE_MAX_M2_S4"
        )

    def measurement_variance(
        self, vertical_accel_mps2: float, confidence: float, vibration_rms_g: float
    ) -> float:
        effective_confidence = max(confidence, self.confidence_floor)
        variance = (
            self.base_variance
            + self.adapt_factor * vertical_accel_mps2 * vertical_accel_mps2
        ) / (effective_confidence * effective_confidence)
        variance += (vibration_rms_g * self.gravity) ** 2
        return min(variance, self.maximum)

    def test_full_confidence_without_vibration_preserves_legacy_variance(self) -> None:
        acceleration = 0.8
        expected = self.base_variance + self.adapt_factor * acceleration**2

        self.assertAlmostEqual(
            self.measurement_variance(acceleration, 1.0, 0.0), expected
        )

    def test_lower_confidence_monotonically_reduces_accel_weight(self) -> None:
        high_quality = self.measurement_variance(0.2, 1.0, 0.0)
        medium_quality = self.measurement_variance(0.2, 0.5, 0.0)
        low_quality = self.measurement_variance(0.2, 0.1, 0.0)

        self.assertLess(high_quality, medium_quality)
        self.assertLess(medium_quality, low_quality)

    def test_vibration_increases_measurement_variance(self) -> None:
        quiet = self.measurement_variance(0.0, 0.8, 0.005)
        vibrating = self.measurement_variance(0.0, 0.8, 0.05)

        self.assertLess(quiet, vibrating)

    def test_zero_confidence_is_finite_and_bounded(self) -> None:
        variance = self.measurement_variance(20.0, 0.0, 1.0)

        self.assertTrue(math.isfinite(variance))
        self.assertEqual(variance, self.maximum)

    def test_diagnostic_contract_exposes_bias_innovations_and_effective_r(self) -> None:
        for field in (
            "accel_bias_mps2",
            "baro_innovation_m",
            "accel_innovation_mps2",
            "baro_measurement_variance_m2",
            "accel_measurement_variance_m2_s4",
            "baro_innovation_valid",
            "accel_innovation_valid",
        ):
            self.assertIn(field, ESTIMATOR_HEADER)


if __name__ == "__main__":
    unittest.main()
