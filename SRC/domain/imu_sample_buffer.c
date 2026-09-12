#include "domain/imu_sample_buffer.h"

#include <string.h>

void imu_sample_buffer_reset(imu_sample_buffer_t *buffer) {
    if (buffer == NULL) {
        return;
    }
    memset(buffer, 0, sizeof(*buffer));
}

void imu_sample_buffer_clear(imu_sample_buffer_t *buffer) {
    if (buffer == NULL) {
        return;
    }
    buffer->head = 0U;
    buffer->count = 0U;
}

bool imu_sample_buffer_push(imu_sample_buffer_t *buffer,
                            const imu_sample_t *sample) {
    size_t tail = 0U;
    bool overflowed = false;

    if (buffer == NULL || sample == NULL || !sample->valid ||
        sample->timestamp_us <= 0) {
        return false;
    }
    if (buffer->count == IMU_SAMPLE_BUFFER_CAPACITY) {
        buffer->head = (buffer->head + 1U) % IMU_SAMPLE_BUFFER_CAPACITY;
        buffer->count--;
        if (buffer->overflow_count < UINT32_MAX) {
            buffer->overflow_count++;
        }
        overflowed = true;
    }
    tail = (buffer->head + buffer->count) % IMU_SAMPLE_BUFFER_CAPACITY;
    buffer->samples[tail] = *sample;
    buffer->count++;
    if (buffer->count > buffer->high_watermark) {
        buffer->high_watermark = buffer->count;
    }
    return !overflowed;
}

bool imu_sample_buffer_pop_through(imu_sample_buffer_t *buffer,
                                   int64_t cutoff_us,
                                   imu_sample_t *sample) {
    if (buffer == NULL || sample == NULL || buffer->count == 0U ||
        cutoff_us <= 0 ||
        buffer->samples[buffer->head].timestamp_us > cutoff_us) {
        return false;
    }
    *sample = buffer->samples[buffer->head];
    buffer->head = (buffer->head + 1U) % IMU_SAMPLE_BUFFER_CAPACITY;
    buffer->count--;
    return true;
}

size_t imu_sample_buffer_count(const imu_sample_buffer_t *buffer) {
    if (buffer == NULL) {
        return 0U;
    }
    return buffer->count;
}

size_t imu_sample_buffer_high_watermark(const imu_sample_buffer_t *buffer) {
    if (buffer == NULL) {
        return 0U;
    }
    return buffer->high_watermark;
}

uint32_t imu_sample_buffer_overflow_count(const imu_sample_buffer_t *buffer) {
    if (buffer == NULL) {
        return 0U;
    }
    return buffer->overflow_count;
}
