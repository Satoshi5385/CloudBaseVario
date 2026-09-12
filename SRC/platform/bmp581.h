#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct {
    int64_t timestamp_us;
    int32_t raw_temperature;
    uint32_t raw_pressure;
    int32_t temperature_c_x100;
    int32_t pressure_pa_x100;
    bool valid;
} bmp581_sample_t;

/** Initialize the BMP581 and route its Data Ready interrupt to sensor_task. */
esp_err_t bmp581_init(i2c_master_bus_handle_t bus_handle,
                      TaskHandle_t sensor_task);

/** Put the BMP581 in standby and remove its I2C device handle. */
esp_err_t bmp581_deinit(void);

/** Atomically consume pending Data Ready edges and their latest ISR time. */
bool bmp581_take_data_ready_event(int64_t *timestamp_us,
                                  uint32_t *interrupt_count);

/** Read and convert one Data Ready temperature/pressure frame. */
esp_err_t bmp581_read_sample(int64_t interrupt_timestamp_us,
                             bmp581_sample_t *sample);

/** Decode a six-byte BMP581 frame. Exposed for deterministic conversion tests. */
bool bmp581_decode_sample(const uint8_t data[6], int64_t timestamp_us, bmp581_sample_t *sample);
