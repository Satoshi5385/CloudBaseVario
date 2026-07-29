#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    int64_t timestamp_us;
    int32_t raw_temperature;
    uint32_t raw_pressure;
    int32_t temperature_c_x100;
    int32_t pressure_pa_x100;
    bool valid;
} bmp581_sample_t;

/** Initialize and configure the BMP581 at its fixed 0x46 address. */
esp_err_t bmp581_init(i2c_master_bus_handle_t bus_handle);

/** Put the BMP581 in standby and remove its I2C device handle. */
esp_err_t bmp581_deinit(void);

/** Read and convert one temperature/pressure frame. */
esp_err_t bmp581_read_sample(bmp581_sample_t *sample);

/** Decode a six-byte BMP581 frame. Exposed for deterministic conversion tests. */
bool bmp581_decode_sample(const uint8_t data[6], int64_t timestamp_us, bmp581_sample_t *sample);
