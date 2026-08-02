#include "domain/vario_estimator.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define ESTIMATOR_WARMUP_SAMPLE_COUNT UINT32_C(100)
#define ESTIMATOR_MIN_PRESSURE_PA 30000.0f
#define ESTIMATOR_MAX_PRESSURE_PA 125000.0f
#define ESTIMATOR_MAX_DT_SECONDS 0.5f
#define ESTIMATOR_ALTITUDE_ALPHA 0.08f
#define ESTIMATOR_VELOCITY_BETA 0.002f
#define ESTIMATOR_MIN_CLIMB_RATE_MPS -50.0f
#define ESTIMATOR_MAX_CLIMB_RATE_MPS 50.0f
#define FUSION_STATE_COUNT UINT32_C(4)
#define FUSION_ALTITUDE_MEAS_VARIANCE_M2 0.04f
#define FUSION_ACCEL_MEAS_VARIANCE_M2_S4 0.25f
#define FUSION_ACCEL_PROCESS_VARIANCE_M2_S4 4.0f
#define FUSION_BIAS_PROCESS_VARIANCE_M2_S4 0.0001f
#define FUSION_INITIAL_ALTITUDE_VARIANCE_M2 4.0f
#define FUSION_INITIAL_VELOCITY_VARIANCE_M2_S2 4.0f
#define FUSION_INITIAL_ACCEL_VARIANCE_M2_S4 100.0f
#define FUSION_INITIAL_BIAS_VARIANCE_M2_S4 4.0f
#define FUSION_ACCEL_ADAPT_FACTOR 0.25f
#define FUSION_ACCEL_CONFIDENCE_FLOOR 0.05f
#define FUSION_ACCEL_VIBRATION_GRAVITY_MPS2 9.80665f
#define FUSION_ACCEL_MEAS_VARIANCE_MAX_M2_S4 10000.0f

static bool pressure_to_altitude(float pressure_pa, float sea_level_pressure_pa,
                                 float *altitude_m) {
    float ratio = 0.0f;
    float converted = 0.0f;

    if (altitude_m == NULL || !isfinite(pressure_pa) ||
        !isfinite(sea_level_pressure_pa) ||
        pressure_pa < ESTIMATOR_MIN_PRESSURE_PA ||
        pressure_pa > ESTIMATOR_MAX_PRESSURE_PA ||
        sea_level_pressure_pa < 80000.0f ||
        sea_level_pressure_pa > 110000.0f) {
        return false;
    }

    ratio = pressure_pa / sea_level_pressure_pa;
    converted = 44330.0f * (1.0f - powf(ratio, 0.19029495f));
    if (!isfinite(converted)) {
        return false;
    }
    *altitude_m = converted;
    return true;
}

void vario_estimator_reset(vario_estimator_t *estimator) {
    if (estimator != NULL) {
        memset(estimator, 0, sizeof(*estimator));
    }
}

void vario_estimator_disable_fusion(vario_estimator_t *estimator) {
    if (estimator == NULL) {
        return;
    }
    estimator->fusion_altitude_m = 0.0f;
    estimator->fusion_climb_rate_mps = 0.0f;
    estimator->fusion_accel_mps2 = 0.0f;
    estimator->fusion_accel_bias_mps2 = 0.0f;
    estimator->latest_vertical_accel_mps2 = 0.0f;
    estimator->fusion_baro_innovation_m = 0.0f;
    estimator->fusion_accel_innovation_mps2 = 0.0f;
    estimator->fusion_baro_measurement_variance_m2 = 0.0f;
    estimator->fusion_accel_measurement_variance_m2_s4 = 0.0f;
    estimator->fusion_timestamp_us = 0;
    estimator->fusion_accel_available = false;
    estimator->fusion_initialized = false;
    estimator->fusion_baro_innovation_valid = false;
    estimator->fusion_accel_innovation_valid = false;
    memset(estimator->fusion_covariance, 0,
           sizeof(estimator->fusion_covariance));
}

static bool fusion_state_is_valid(const vario_estimator_t *estimator) {
    if (estimator == NULL ||
        !isfinite(estimator->fusion_altitude_m) ||
        !isfinite(estimator->fusion_climb_rate_mps) ||
        !isfinite(estimator->fusion_accel_mps2) ||
        !isfinite(estimator->fusion_accel_bias_mps2) ||
        estimator->fusion_climb_rate_mps < ESTIMATOR_MIN_CLIMB_RATE_MPS ||
        estimator->fusion_climb_rate_mps > ESTIMATOR_MAX_CLIMB_RATE_MPS) {
        return false;
    }
    for (size_t row = 0U; row < FUSION_STATE_COUNT; row++) {
        for (size_t column = 0U; column < FUSION_STATE_COUNT; column++) {
            if (!isfinite(estimator->fusion_covariance[row][column])) {
                return false;
            }
        }
    }
    return true;
}

static bool fusion_predict_to(vario_estimator_t *estimator,
                              int64_t timestamp_us) {
    float transition[FUSION_STATE_COUNT][FUSION_STATE_COUNT] = {0};
    float intermediate[FUSION_STATE_COUNT][FUSION_STATE_COUNT] = {0};
    float covariance[FUSION_STATE_COUNT][FUSION_STATE_COUNT] = {0};
    float dt_seconds = 0.0f;
    float acceleration_mps2 = 0.0f;
    int64_t delta_us = 0;

    if (estimator == NULL || !estimator->fusion_initialized ||
        timestamp_us <= 0 || estimator->fusion_timestamp_us <= 0) {
        return false;
    }
    delta_us = timestamp_us - estimator->fusion_timestamp_us;
    if (delta_us < 0) {
        return false;
    }
    if (delta_us == 0) {
        return true;
    }
    dt_seconds = (float) delta_us / 1000000.0f;
    if (!isfinite(dt_seconds) || dt_seconds > ESTIMATOR_MAX_DT_SECONDS) {
        return false;
    }

    transition[0][0] = 1.0f;
    transition[0][1] = dt_seconds;
    transition[0][2] = 0.5f * dt_seconds * dt_seconds;
    transition[0][3] = -0.5f * dt_seconds * dt_seconds;
    transition[1][1] = 1.0f;
    transition[1][2] = dt_seconds;
    transition[1][3] = -dt_seconds;
    transition[2][2] = 1.0f;
    transition[3][3] = 1.0f;

    acceleration_mps2 =
        estimator->fusion_accel_mps2 - estimator->fusion_accel_bias_mps2;
    estimator->fusion_altitude_m +=
        estimator->fusion_climb_rate_mps * dt_seconds +
        0.5f * acceleration_mps2 * dt_seconds * dt_seconds;
    estimator->fusion_climb_rate_mps += acceleration_mps2 * dt_seconds;

    for (size_t row = 0U; row < FUSION_STATE_COUNT; row++) {
        for (size_t column = 0U; column < FUSION_STATE_COUNT; column++) {
            for (size_t index = 0U; index < FUSION_STATE_COUNT; index++) {
                intermediate[row][column] +=
                    transition[row][index] *
                    estimator->fusion_covariance[index][column];
            }
        }
    }
    for (size_t row = 0U; row < FUSION_STATE_COUNT; row++) {
        for (size_t column = 0U; column < FUSION_STATE_COUNT; column++) {
            for (size_t index = 0U; index < FUSION_STATE_COUNT; index++) {
                covariance[row][column] +=
                    intermediate[row][index] * transition[column][index];
            }
        }
    }
    memcpy(estimator->fusion_covariance, covariance,
           sizeof(estimator->fusion_covariance));
    estimator->fusion_covariance[2][2] +=
        FUSION_ACCEL_PROCESS_VARIANCE_M2_S4 * dt_seconds;
    estimator->fusion_covariance[3][3] +=
        FUSION_BIAS_PROCESS_VARIANCE_M2_S4 * dt_seconds;
    estimator->fusion_timestamp_us = timestamp_us;
    return fusion_state_is_valid(estimator);
}

static bool fusion_update_scalar(vario_estimator_t *estimator,
                                 size_t observed_state, float measurement,
                                 float measurement_variance,
                                 float *innovation_out) {
    float state[FUSION_STATE_COUNT] = {0};
    float gain[FUSION_STATE_COUNT] = {0};
    float covariance[FUSION_STATE_COUNT][FUSION_STATE_COUNT] = {0};
    float innovation = 0.0f;
    float innovation_variance = 0.0f;

    if (estimator == NULL || observed_state >= FUSION_STATE_COUNT ||
        !isfinite(measurement) || !isfinite(measurement_variance) ||
        measurement_variance <= 0.0f) {
        return false;
    }
    state[0] = estimator->fusion_altitude_m;
    state[1] = estimator->fusion_climb_rate_mps;
    state[2] = estimator->fusion_accel_mps2;
    state[3] = estimator->fusion_accel_bias_mps2;
    innovation = measurement - state[observed_state];
    innovation_variance =
        estimator->fusion_covariance[observed_state][observed_state] +
        measurement_variance;
    if (!isfinite(innovation_variance) ||
        innovation_variance <= 1.0e-9f) {
        return false;
    }

    for (size_t row = 0U; row < FUSION_STATE_COUNT; row++) {
        gain[row] =
            estimator->fusion_covariance[row][observed_state] /
            innovation_variance;
        state[row] += gain[row] * innovation;
    }
    for (size_t row = 0U; row < FUSION_STATE_COUNT; row++) {
        for (size_t column = 0U; column < FUSION_STATE_COUNT; column++) {
            covariance[row][column] =
                estimator->fusion_covariance[row][column] -
                gain[row] *
                    estimator->fusion_covariance[observed_state][column];
        }
    }
    for (size_t row = 0U; row < FUSION_STATE_COUNT; row++) {
        for (size_t column = row + 1U; column < FUSION_STATE_COUNT;
             column++) {
            float symmetric_value =
                0.5f * (covariance[row][column] +
                        covariance[column][row]);

            covariance[row][column] = symmetric_value;
            covariance[column][row] = symmetric_value;
        }
    }

    estimator->fusion_altitude_m = state[0];
    estimator->fusion_climb_rate_mps = state[1];
    estimator->fusion_accel_mps2 = state[2];
    estimator->fusion_accel_bias_mps2 = state[3];
    memcpy(estimator->fusion_covariance, covariance,
           sizeof(estimator->fusion_covariance));
    if (!fusion_state_is_valid(estimator)) {
        return false;
    }
    if (innovation_out != NULL) {
        *innovation_out = innovation;
    }
    return true;
}

static void fusion_seed_from_baro(vario_estimator_t *estimator,
                                  int64_t timestamp_us) {
    if (estimator == NULL) {
        return;
    }
    estimator->fusion_altitude_m = estimator->altitude_m;
    estimator->fusion_climb_rate_mps = estimator->climb_rate_mps;
    estimator->fusion_accel_mps2 =
        estimator->latest_vertical_accel_mps2;
    estimator->fusion_accel_bias_mps2 = 0.0f;
    memset(estimator->fusion_covariance, 0,
           sizeof(estimator->fusion_covariance));
    estimator->fusion_covariance[0][0] =
        FUSION_INITIAL_ALTITUDE_VARIANCE_M2;
    estimator->fusion_covariance[1][1] =
        FUSION_INITIAL_VELOCITY_VARIANCE_M2_S2;
    estimator->fusion_covariance[2][2] =
        FUSION_INITIAL_ACCEL_VARIANCE_M2_S4;
    estimator->fusion_covariance[3][3] =
        FUSION_INITIAL_BIAS_VARIANCE_M2_S4;
    estimator->fusion_timestamp_us = timestamp_us;
    estimator->fusion_initialized = true;
}

static bool fusion_accel_measurement_variance(
    float vertical_accel_mps2, float imu_confidence,
    float vibration_rms_g, float *measurement_variance) {
    float confidence = 0.0f;
    float vibration_mps2 = 0.0f;
    float variance = 0.0f;

    if (measurement_variance == NULL || !isfinite(vertical_accel_mps2) ||
        !isfinite(imu_confidence) || imu_confidence < 0.0f ||
        imu_confidence > 1.0f || !isfinite(vibration_rms_g) ||
        vibration_rms_g < 0.0f) {
        return false;
    }
    confidence = fmaxf(imu_confidence, FUSION_ACCEL_CONFIDENCE_FLOOR);
    vibration_mps2 =
        vibration_rms_g * FUSION_ACCEL_VIBRATION_GRAVITY_MPS2;
    variance =
        (FUSION_ACCEL_MEAS_VARIANCE_M2_S4 +
         FUSION_ACCEL_ADAPT_FACTOR * vertical_accel_mps2 *
             vertical_accel_mps2) /
            (confidence * confidence) +
        vibration_mps2 * vibration_mps2;
    if (!isfinite(variance) || variance <= 0.0f) {
        return false;
    }
    *measurement_variance =
        fminf(variance, FUSION_ACCEL_MEAS_VARIANCE_MAX_M2_S4);
    return true;
}

bool vario_estimator_update_imu(vario_estimator_t *estimator,
                                float vertical_accel_mps2,
                                float imu_confidence,
                                float vibration_rms_g,
                                int64_t timestamp_us) {
    float accel_variance = 0.0f;
    float accel_innovation = 0.0f;

    if (estimator == NULL || timestamp_us <= 0 ||
        !fusion_accel_measurement_variance(
            vertical_accel_mps2, imu_confidence, vibration_rms_g,
            &accel_variance)) {
        if (estimator != NULL) {
            vario_estimator_disable_fusion(estimator);
        }
        return false;
    }
    if (estimator->fusion_timestamp_us != 0 &&
        timestamp_us <= estimator->fusion_timestamp_us) {
        vario_estimator_disable_fusion(estimator);
        return false;
    }

    estimator->latest_vertical_accel_mps2 = vertical_accel_mps2;
    estimator->fusion_accel_measurement_variance_m2_s4 = accel_variance;
    estimator->fusion_accel_innovation_valid = false;
    estimator->fusion_accel_available = true;
    if (!estimator->fusion_initialized) {
        estimator->fusion_timestamp_us = timestamp_us;
        return true;
    }
    if (!fusion_predict_to(estimator, timestamp_us)) {
        vario_estimator_disable_fusion(estimator);
        return false;
    }
    if (!fusion_update_scalar(estimator, 2U, vertical_accel_mps2,
                              accel_variance, &accel_innovation)) {
        vario_estimator_disable_fusion(estimator);
        return false;
    }
    estimator->fusion_accel_innovation_mps2 = accel_innovation;
    estimator->fusion_accel_innovation_valid = true;
    return true;
}

bool vario_estimator_get_diagnostics(
    const vario_estimator_t *estimator,
    vario_estimator_diagnostics_t *diagnostics) {
    if (estimator == NULL || diagnostics == NULL) {
        return false;
    }
    memset(diagnostics, 0, sizeof(*diagnostics));
    diagnostics->accel_bias_mps2 = estimator->fusion_accel_bias_mps2;
    diagnostics->baro_innovation_m = estimator->fusion_baro_innovation_m;
    diagnostics->accel_innovation_mps2 =
        estimator->fusion_accel_innovation_mps2;
    diagnostics->baro_measurement_variance_m2 =
        estimator->fusion_baro_measurement_variance_m2;
    diagnostics->accel_measurement_variance_m2_s4 =
        estimator->fusion_accel_measurement_variance_m2_s4;
    diagnostics->initialized = estimator->fusion_initialized;
    diagnostics->baro_innovation_valid =
        estimator->fusion_baro_innovation_valid;
    diagnostics->accel_innovation_valid =
        estimator->fusion_accel_innovation_valid;
    return true;
}

bool vario_estimator_update(vario_estimator_t *estimator,
                            int32_t pressure_pa_x100,
                            int64_t timestamp_us,
                            float sea_level_pressure_pa,
                            bool fusion_requested,
                            vario_estimate_t *estimate) {
    float pressure_pa = (float) pressure_pa_x100 / 100.0f;
    float measured_altitude_m = 0.0f;
    float dt_seconds = 0.0f;
    float baro_innovation_m = 0.0f;

    if (estimator == NULL || estimate == NULL) {
        return false;
    }
    memset(estimate, 0, sizeof(*estimate));

    if (!pressure_to_altitude(pressure_pa, sea_level_pressure_pa,
                              &measured_altitude_m) ||
        timestamp_us <= 0) {
        vario_estimator_reset(estimator);
        return false;
    }

    if (estimator->previous_timestamp_us != 0) {
        int64_t delta_us = timestamp_us - estimator->previous_timestamp_us;

        dt_seconds = (float) delta_us / 1000000.0f;
        if (delta_us <= 0 || !isfinite(dt_seconds) ||
            dt_seconds > ESTIMATOR_MAX_DT_SECONDS) {
            vario_estimator_reset(estimator);
            estimator->previous_timestamp_us = timestamp_us;
            estimator->warmup_altitude_sum_m = measured_altitude_m;
            estimator->warmup_samples = 1U;
            estimate->warming_up = true;
            return false;
        }
    }
    estimator->previous_timestamp_us = timestamp_us;

    if (!estimator->initialized) {
        estimator->warmup_altitude_sum_m += measured_altitude_m;
        estimator->warmup_samples++;
        estimate->warming_up = true;
        if (estimator->warmup_samples < ESTIMATOR_WARMUP_SAMPLE_COUNT) {
            return false;
        }

        estimator->altitude_m =
            (float) (estimator->warmup_altitude_sum_m /
                     (double) estimator->warmup_samples);
        estimator->climb_rate_mps = 0.0f;
        estimator->initialized = true;
        estimate->warming_up = false;
    } else {
        float predicted_altitude_m =
            estimator->altitude_m + estimator->climb_rate_mps * dt_seconds;
        float innovation_m = measured_altitude_m - predicted_altitude_m;

        estimator->altitude_m =
            predicted_altitude_m + ESTIMATOR_ALTITUDE_ALPHA * innovation_m;
        estimator->climb_rate_mps +=
            (ESTIMATOR_VELOCITY_BETA / dt_seconds) * innovation_m;
    }

    if (!isfinite(estimator->altitude_m) ||
        !isfinite(estimator->climb_rate_mps) ||
        estimator->climb_rate_mps < ESTIMATOR_MIN_CLIMB_RATE_MPS ||
        estimator->climb_rate_mps > ESTIMATOR_MAX_CLIMB_RATE_MPS) {
        vario_estimator_reset(estimator);
        return false;
    }

    if (fusion_requested && estimator->fusion_accel_available) {
        if (!estimator->fusion_initialized) {
            fusion_seed_from_baro(estimator, timestamp_us);
        } else if (!fusion_predict_to(estimator, timestamp_us)) {
            vario_estimator_disable_fusion(estimator);
        }
        if (estimator->fusion_initialized &&
            fusion_update_scalar(estimator, 0U, measured_altitude_m,
                                 FUSION_ALTITUDE_MEAS_VARIANCE_M2,
                                 &baro_innovation_m)) {
            estimator->fusion_baro_innovation_m = baro_innovation_m;
            estimator->fusion_baro_measurement_variance_m2 =
                FUSION_ALTITUDE_MEAS_VARIANCE_M2;
            estimator->fusion_baro_innovation_valid = true;
            estimate->altitude_m = estimator->fusion_altitude_m;
            estimate->climb_rate_mps =
                estimator->fusion_climb_rate_mps;
            estimate->fusion_active = true;
        }
    } else {
        vario_estimator_disable_fusion(estimator);
    }
    if (!estimate->fusion_active) {
        estimate->altitude_m = estimator->altitude_m;
        estimate->climb_rate_mps = estimator->climb_rate_mps;
    }
    estimate->altitude_valid = true;
    estimate->climb_rate_valid = true;
    estimate->warming_up = false;
    return true;
}
