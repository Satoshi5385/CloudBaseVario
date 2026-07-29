#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t warmup_samples;
    int64_t previous_timestamp_us;
    double warmup_altitude_sum_m;
    float altitude_m;
    float climb_rate_mps;
    float fusion_altitude_m;
    float fusion_climb_rate_mps;
    float fusion_accel_mps2;
    float fusion_accel_bias_mps2;
    float fusion_covariance[4][4];
    float latest_vertical_accel_mps2;
    int64_t fusion_timestamp_us;
    bool fusion_accel_available;
    bool fusion_initialized;
    bool initialized;
} vario_estimator_t;

typedef struct {
    float altitude_m;
    float climb_rate_mps;
    bool altitude_valid;
    bool climb_rate_valid;
    bool warming_up;
    bool fusion_active;
} vario_estimate_t;

/** Reset the pressure-only and fused estimators and their warm-up state. */
void vario_estimator_reset(vario_estimator_t *estimator);

/**
 * Supply one attitude-corrected vertical-acceleration observation.
 *
 * @param[in,out] estimator Persistent pressure and fusion state.
 * @param[in] vertical_accel_mps2 Earth-frame vertical acceleration, up positive.
 * @param[in] timestamp_us Monotonic sample timestamp.
 * @return true when the observation was accepted.
 */
bool vario_estimator_update_imu(vario_estimator_t *estimator,
                                float vertical_accel_mps2,
                                int64_t timestamp_us);

/**
 * @brief Drop fused state without disturbing the pressure-only estimator.
 * @param[in,out] estimator Persistent pressure and fusion state.
 */
void vario_estimator_disable_fusion(vario_estimator_t *estimator);

/**
 * Update pressure-only and, when requested, pressure/IMU fused filters.
 *
 * Invalid pressure or time steps restart warm-up and never produce a clamped
 * valid result. Fusion is aligned to the current pressure-only output before
 * it becomes active.
 */
bool vario_estimator_update(vario_estimator_t *estimator,
                            int32_t pressure_pa_x100,
                            int64_t timestamp_us,
                            float sea_level_pressure_pa,
                            bool fusion_requested,
                            vario_estimate_t *estimate);
