#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

/**
 * @brief Create the shared ESP-IDF I2C master bus used by both sensors.
 * @return ESP_OK on success, otherwise an ESP-IDF I2C driver error.
 */
esp_err_t sensor_bus_init(void);

/**
 * @brief Return the shared bus handle for sensor-driver initialization.
 * @return I2C bus handle, or NULL when initialization failed.
 */
i2c_master_bus_handle_t sensor_bus_get_handle(void);
