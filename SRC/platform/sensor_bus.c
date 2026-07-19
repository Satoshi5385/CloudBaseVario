#include "platform/sensor_bus.h"

#include <stddef.h>

#include "platform/board.h"

#define SENSOR_BUS_GLITCH_IGNORE_COUNT UINT8_C(7)

/* Shared I2C bus handle. Device transactions will be owned by sensor_task. */
static i2c_master_bus_handle_t sensor_bus_handle = NULL;

esp_err_t sensor_bus_init(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = BOARD_I2C_PORT,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = SENSOR_BUS_GLITCH_IGNORE_COUNT,
        .flags.enable_internal_pullup = false,
    };

    if (sensor_bus_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_new_master_bus(&bus_config, &sensor_bus_handle);
}

i2c_master_bus_handle_t sensor_bus_get_handle(void) {
    return sensor_bus_handle;
}
