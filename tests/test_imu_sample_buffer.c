#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "domain/imu_sample_buffer.h"

static imu_sample_t sample_at(int64_t timestamp_us) {
    imu_sample_t sample;

    memset(&sample, 0, sizeof(sample));
    sample.timestamp_us = timestamp_us;
    sample.accel_mps2[0] = (float) timestamp_us;
    sample.valid = true;
    return sample;
}

static void test_fifo_and_cutoff(void) {
    imu_sample_buffer_t buffer;
    imu_sample_t output;
    imu_sample_t sample;

    imu_sample_buffer_reset(&buffer);
    sample = sample_at(100);
    assert(imu_sample_buffer_push(&buffer, &sample));
    sample = sample_at(200);
    assert(imu_sample_buffer_push(&buffer, &sample));
    sample = sample_at(300);
    assert(imu_sample_buffer_push(&buffer, &sample));
    assert(!imu_sample_buffer_pop_through(&buffer, 99, &output));
    assert(imu_sample_buffer_pop_through(&buffer, 200, &output));
    assert(output.timestamp_us == 100);
    assert(imu_sample_buffer_pop_through(&buffer, 200, &output));
    assert(output.timestamp_us == 200);
    assert(!imu_sample_buffer_pop_through(&buffer, 200, &output));
    assert(imu_sample_buffer_count(&buffer) == 1U);
}

static void test_overflow_keeps_latest_samples(void) {
    imu_sample_buffer_t buffer;
    imu_sample_t output;
    imu_sample_t sample;

    imu_sample_buffer_reset(&buffer);
    for (size_t index = 0U; index < IMU_SAMPLE_BUFFER_CAPACITY + 2U;
         index++) {
        sample = sample_at((int64_t) index + 1);
        bool accepted_without_overflow =
            imu_sample_buffer_push(&buffer, &sample);

        assert(accepted_without_overflow ==
               (index < IMU_SAMPLE_BUFFER_CAPACITY));
    }
    assert(imu_sample_buffer_count(&buffer) == IMU_SAMPLE_BUFFER_CAPACITY);
    assert(imu_sample_buffer_high_watermark(&buffer) ==
           IMU_SAMPLE_BUFFER_CAPACITY);
    assert(imu_sample_buffer_overflow_count(&buffer) == 2U);
    assert(imu_sample_buffer_pop_through(&buffer, INT64_MAX, &output));
    assert(output.timestamp_us == 3);
}

static void test_reset_clears_samples_and_diagnostics(void) {
    imu_sample_buffer_t buffer;
    imu_sample_t sample = sample_at(1);

    imu_sample_buffer_reset(&buffer);
    assert(imu_sample_buffer_push(&buffer, &sample));
    imu_sample_buffer_reset(&buffer);
    assert(imu_sample_buffer_count(&buffer) == 0U);
    assert(imu_sample_buffer_high_watermark(&buffer) == 0U);
    assert(imu_sample_buffer_overflow_count(&buffer) == 0U);
}

static void test_clear_preserves_diagnostics(void) {
    imu_sample_buffer_t buffer;
    imu_sample_t sample;

    imu_sample_buffer_reset(&buffer);
    for (size_t index = 0U; index < IMU_SAMPLE_BUFFER_CAPACITY + 1U;
         index++) {
        sample = sample_at((int64_t) index + 1);
        (void) imu_sample_buffer_push(&buffer, &sample);
    }
    imu_sample_buffer_clear(&buffer);
    assert(imu_sample_buffer_count(&buffer) == 0U);
    assert(imu_sample_buffer_high_watermark(&buffer) ==
           IMU_SAMPLE_BUFFER_CAPACITY);
    assert(imu_sample_buffer_overflow_count(&buffer) == 1U);
}

int main(void) {
    test_fifo_and_cutoff();
    test_overflow_keeps_latest_samples();
    test_clear_preserves_diagnostics();
    test_reset_clears_samples_and_diagnostics();
    puts("imu_sample_buffer tests passed");
    return 0;
}
