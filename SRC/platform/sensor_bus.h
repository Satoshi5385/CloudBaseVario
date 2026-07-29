#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

/**
 * @brief Create the shared ESP-IDF I2C master bus used by both sensors.
 * @return ESP_OK on success, otherwise an ESP-IDF I2C driver error.
 */
esp_err_t sensor_bus_init(void);

/**
 * @brief Delete the shared bus after all device handles have been removed.
 * @return ESP_OK when the bus is absent or was deleted successfully.
 */
esp_err_t sensor_bus_deinit(void);

/**
 * @brief Recreate the bus after a stuck-bus failure.
 * @return ESP_OK when a fresh bus handle was created.
 */
esp_err_t sensor_bus_recover(void);

/**
 * @brief Return the shared bus handle for sensor-driver initialization.
 * @return I2C bus handle, or NULL when initialization failed.
 */
i2c_master_bus_handle_t sensor_bus_get_handle(void);
