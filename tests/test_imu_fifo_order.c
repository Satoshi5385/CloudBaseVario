#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "domain/vario_estimator.h"

/* Replay 400 Hz acceleration + 100 Hz pressure using the real estimator.
 * The barometer completes 1.8 ms after a batch boundary: the next FIFO
 * contains an IMU sample older than that previously captured pressure. */
static int64_t imu_time(size_t index) {
    return INT64_C(1000000) + (int64_t) index * 2500;
}
static int64_t baro_time(size_t index) {
    return INT64_C(1000000) + ((int64_t) index + 1) * 10000 + 1800;
}
static bool push_imu(vario_estimator_t *estimator, size_t index) {
    return vario_estimator_update_imu(estimator, 0.1f * sinf((float)index * 0.1f),
                                      0.95f, 0.002f, imu_time(index));
}
static void push_baro(vario_estimator_t *estimator, size_t index) {
    vario_estimate_t estimate = {0};
    (void) vario_estimator_update(estimator, 10132500, baro_time(index),
                                 101325.0f, true, &estimate);
}
int main(void) {
    vario_estimator_t ordered = {0};
    vario_estimator_t batched = {0};
    vario_estimator_t naive = {0};
    const size_t batch_count = 300;
    size_t baro_index = 0;
    size_t rejected = 0;
    bool pending = false;
    size_t pending_index = 0;

    vario_estimator_reset(&ordered);
    vario_estimator_reset(&batched);
    vario_estimator_reset(&naive);
    /* Independent chronological reference: merge the two timestamp streams. */
    for (size_t imu_index = 0; imu_index < batch_count * 4; imu_index++) {
        while (baro_index < batch_count && baro_time(baro_index) < imu_time(imu_index)) {
            push_baro(&ordered, baro_index);
            baro_index++;
        }
        assert(push_imu(&ordered, imu_index));
    }
    while (baro_index < batch_count) {
        push_baro(&ordered, baro_index);
        baro_index++;
    }
    for (size_t batch = 0; batch < batch_count; batch++) {
        for (size_t i = batch * 4; i < (batch + 1) * 4; i++) {
            if (pending && baro_time(pending_index) < imu_time(i)) {
                push_baro(&batched, pending_index);
                pending = false;
            }
            assert(push_imu(&batched, i));
            if (!push_imu(&naive, i)) rejected++;
        }
        if (pending) push_baro(&batched, pending_index);
        pending_index = batch;
        pending = true;
        push_baro(&naive, batch);
    }
    push_baro(&batched, pending_index);
    assert(rejected > 0); /* A plain IMU-batch -> immediate BMP loop is unsafe. */
    assert(ordered.fusion_initialized && batched.fusion_initialized);
    assert(ordered.fusion_timestamp_us == batched.fusion_timestamp_us);
    assert(fabsf(ordered.fusion_climb_rate_mps - batched.fusion_climb_rate_mps) < 1e-6f);
    assert(fabsf(ordered.fusion_altitude_m - batched.fusion_altitude_m) < 1e-6f);
    assert(fabsf(ordered.fusion_accel_bias_mps2 - batched.fusion_accel_bias_mps2) < 1e-6f);
    puts("IMU/BMP chronological batch replay passed");
    return 0;
}
