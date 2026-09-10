#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ICM42688_HXY_I2C_ADDRESS UINT16_C(0x18)
#define ICM42688_HXY_WHO_AM_I_REGISTER UINT8_C(0x01)
#define ICM42688_HXY_WHO_AM_I_VALUE UINT8_C(0x6A)
#define ICM42688_HXY_SAMPLE_RATE_HZ UINT32_C(400)
#define ICM42688_HXY_SAMPLE_PERIOD_US INT64_C(2500)
#define ICM42688_HXY_FIFO_MAX_SAMPLES UINT32_C(16)
#define ICM42688_HXY_ACCEL_RANGE_G 8.0f
#define ICM42688_HXY_GYRO_RANGE_DPS 2000.0f

typedef struct {
    uint8_t address;
    uint8_t who_am_i;
} icm42688_hxy_identity_t;

typedef struct {
    float accel_mps2[3];
    float gyro_radps[3];
    int16_t raw_accel[3];
    int16_t raw_gyro[3];
    int64_t timestamp_us;
    uint8_t data_status;
    bool valid;
} icm42688_hxy_sample_t;

typedef struct {
    icm42688_hxy_sample_t samples[ICM42688_HXY_FIFO_MAX_SAMPLES];
    size_t sample_count;
    uint32_t discarded_samples;
    uint32_t sensor_time;
    uint8_t data_status;
    bool overflow;
} icm42688_hxy_batch_t;

/** Begin non-blocking HXY initialization without performing bus I/O. */
esp_err_t icm42688_hxy_init_begin(i2c_master_bus_handle_t bus_handle,
                                  TaskHandle_t sensor_task);

/** Execute at most one due initialization step. */
esp_err_t icm42688_hxy_init_poll(int64_t now_us,
                                 icm42688_hxy_identity_t *identity);

/** Return the earliest time at which the next initialization step may run. */
int64_t icm42688_hxy_init_next_action_us(void);

/** Return true while non-blocking initialization is active. */
bool icm42688_hxy_init_in_progress(void);

/** Abort an in-progress initialization and release partial resources. */
esp_err_t icm42688_hxy_init_abort(void);

/**
 * @brief Read one four-sample WTM burst and rearm FIFO mode.
 * @param[out] batch Required output; diagnostics are also valid on error.
 * Timestamps are host-time estimates spaced 2500 us within the batch, anchored
 * to the completed fixed-size transfer, not per-sample hardware times.
 * Sensor_Time is retained as a raw diagnostic; its sample association is not
 * assumed. A WTM notification guarantees one four-byte Sensor_Time followed by
 * four complete 12-byte samples, so the normal path performs one fixed 52-byte
 * FIFO read and the two writes needed for BY-PASS to FIFO rearming.
 * @return ESP_OK for the four-sample batch. Other errors discard the batch; no
 * partial batch may reach the estimator.
 */
esp_err_t icm42688_hxy_read_fifo(icm42688_hxy_batch_t *batch);

/**
 * @brief Remove the GPIO14 ISR, power down, and remove the I2C handle.
 * @return ESP_OK when resources were removed successfully.
 */
esp_err_t icm42688_hxy_deinit(void);
