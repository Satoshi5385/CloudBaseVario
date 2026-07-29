#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ICM42688_HXY_I2C_ADDRESS UINT16_C(0x18)
#define ICM42688_HXY_WHO_AM_I_REGISTER UINT8_C(0x01)
#define ICM42688_HXY_WHO_AM_I_VALUE UINT8_C(0x6A)
#define ICM42688_HXY_SAMPLE_RATE_HZ UINT32_C(400)
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

/**
 * @brief Detect and configure the C46550687 HXY IMU for fused-vario use.
 *
 * The device is configured at 400 Hz, +/-8 g, and +/-2000 dps. GPIO14 is
 * routed from the HXY gyro Data Ready signal and only notifies sensor_task;
 * I2C is never accessed from the ISR.
 *
 * @param[in] bus_handle Shared I2C master bus.
 * @param[in] sensor_task Task to notify from the GPIO14 ISR.
 * @param[out] identity Observed fixed address and WHO_AM_I value.
 * @return ESP_OK only after all HXY configuration read-backs match.
 */
esp_err_t icm42688_hxy_init(i2c_master_bus_handle_t bus_handle,
                            TaskHandle_t sensor_task,
                            icm42688_hxy_identity_t *identity);

/**
 * @brief Read one coherent acceleration and angular-rate sample.
 * @param[out] sample Raw and physical HXY sample in sensor coordinates.
 * @return ESP_OK for a complete Data Ready frame.
 */
esp_err_t icm42688_hxy_read_sample(icm42688_hxy_sample_t *sample);

/**
 * @brief Disable GPIO14 notification, power down, and remove the I2C handle.
 * @return ESP_OK when resources were removed successfully.
 */
esp_err_t icm42688_hxy_deinit(void);
