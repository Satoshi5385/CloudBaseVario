#include "platform/sensor_bus.h"

#include <stddef.h>

#include "platform/board.h"

#define SENSOR_BUS_GLITCH_IGNORE_COUNT UINT8_C(7)

/* Shared sensor I2C bus handle. All transactions are owned by sensor_task. */
static i2c_master_bus_handle_t sensor_bus_handle = NULL;

static esp_err_t sensor_bus_create(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = BOARD_I2C_PORT,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = SENSOR_BUS_GLITCH_IGNORE_COUNT,
        .flags.enable_internal_pullup = false,
    };

    return i2c_new_master_bus(&bus_config, &sensor_bus_handle);
}

esp_err_t sensor_bus_init(void) {
    if (sensor_bus_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return sensor_bus_create();
}

esp_err_t sensor_bus_deinit(void) {
    esp_err_t ret = ESP_OK;

    if (sensor_bus_handle == NULL) {
        return ESP_OK;
    }

    ret = i2c_del_master_bus(sensor_bus_handle);
    if (ret == ESP_OK) {
        sensor_bus_handle = NULL;
    }
    return ret;
}

esp_err_t sensor_bus_recover(void) {
    esp_err_t ret = sensor_bus_deinit();

    if (ret != ESP_OK) {
        return ret;
    }
    return sensor_bus_create();
}

i2c_master_bus_handle_t sensor_bus_get_handle(void) {
    return sensor_bus_handle;
}
