#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "domain/imu_fusion.h"

#define IMU_SAMPLE_BUFFER_CAPACITY 32U

typedef struct {
    imu_sample_t samples[IMU_SAMPLE_BUFFER_CAPACITY];
    size_t head;
    size_t count;
    size_t high_watermark;
    uint32_t overflow_count;
} imu_sample_buffer_t;

/** Reset buffered samples and diagnostics. */
void imu_sample_buffer_reset(imu_sample_buffer_t *buffer);

/** Discard buffered samples while preserving lifetime diagnostics. */
void imu_sample_buffer_clear(imu_sample_buffer_t *buffer);

/**
 * Store one valid sample, discarding the oldest sample if the buffer is full.
 * @return true when no sample was discarded, false after an overflow.
 */
bool imu_sample_buffer_push(imu_sample_buffer_t *buffer,
                            const imu_sample_t *sample);

/**
 * Pop the oldest sample only when its timestamp is not newer than cutoff_us.
 * @return true when a sample was returned.
 */
bool imu_sample_buffer_pop_through(imu_sample_buffer_t *buffer,
                                   int64_t cutoff_us,
                                   imu_sample_t *sample);

/** Return the number of currently buffered samples. */
size_t imu_sample_buffer_count(const imu_sample_buffer_t *buffer);

/** Return the maximum number of simultaneously buffered samples. */
size_t imu_sample_buffer_high_watermark(const imu_sample_buffer_t *buffer);

/** Return the number of oldest samples discarded due to overflow. */
uint32_t imu_sample_buffer_overflow_count(const imu_sample_buffer_t *buffer);
