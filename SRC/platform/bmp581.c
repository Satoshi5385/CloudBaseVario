#include "platform/bmp581.h"

#include <stddef.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "platform/board.h"

#define BMP581_I2C_ADDRESS UINT16_C(0x46)
#define BMP581_I2C_TIMEOUT_MS 5
#define BMP581_RESET_SETTLE_MS UINT32_C(2)

#define BMP581_REG_CHIP_ID UINT8_C(0x01)
#define BMP581_REG_INT_STATUS UINT8_C(0x27)
#define BMP581_REG_STATUS UINT8_C(0x28)
#define BMP581_REG_DSP_CONFIG UINT8_C(0x30)
#define BMP581_REG_DSP_IIR UINT8_C(0x31)
#define BMP581_REG_OSR_CONFIG UINT8_C(0x36)
#define BMP581_REG_ODR_CONFIG UINT8_C(0x37)
#define BMP581_REG_OSR_EFF UINT8_C(0x38)
#define BMP581_REG_COMMAND UINT8_C(0x7E)
#define BMP581_REG_DATA_START UINT8_C(0x1D)

#define BMP581_CHIP_ID UINT8_C(0x50)
#define BMP581_SOFT_RESET_COMMAND UINT8_C(0xB6)
#define BMP581_POR_COMPLETE_MASK UINT8_C(0x10)
#define BMP581_NVM_READY_MASK UINT8_C(0x02)
#define BMP581_NVM_ERROR_MASK UINT8_C(0x0C)
#define BMP581_ODR_VALID_MASK UINT8_C(0x80)

#define BMP581_DSP_CONFIG_VALUE UINT8_C(0x03)
#define BMP581_DSP_IIR_VALUE UINT8_C(0x00)
#define BMP581_OSR_CONFIG_VALUE UINT8_C(0x58)
#define BMP581_ODR_CONFIG_VALUE UINT8_C(0xA9)
#define BMP581_ODR_STANDBY_VALUE UINT8_C(0xA8)

#define BMP581_TEMPERATURE_MIN_C_X100 INT32_C(-4000)
#define BMP581_TEMPERATURE_MAX_C_X100 INT32_C(8500)
#define BMP581_PRESSURE_MIN_PA_X100 INT32_C(3000000)
#define BMP581_PRESSURE_MAX_PA_X100 INT32_C(12500000)

static const char *TAG = "bmp581";
static i2c_master_dev_handle_t bmp581_device = NULL;

static esp_err_t bmp581_read_registers(uint8_t register_address, uint8_t *data, size_t length) {
    if (bmp581_device == NULL || data == NULL || length == 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_master_transmit_receive(bmp581_device, &register_address, sizeof(register_address),
                                       data, length, BMP581_I2C_TIMEOUT_MS);
}

static esp_err_t bmp581_write_register(uint8_t register_address, uint8_t value) {
    uint8_t transaction[2] = {register_address, value};

    if (bmp581_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_master_transmit(bmp581_device, transaction, sizeof(transaction),
                               BMP581_I2C_TIMEOUT_MS);
}

static esp_err_t bmp581_remove_device(void) {
    esp_err_t ret = ESP_OK;

    if (bmp581_device == NULL) {
        return ESP_OK;
    }

    ret = i2c_master_bus_rm_device(bmp581_device);
    if (ret == ESP_OK) {
        bmp581_device = NULL;
    }
    return ret;
}

static int32_t divide_round_nearest(int64_t numerator, int64_t denominator) {
    if (numerator >= 0) {
        return (int32_t) ((numerator + denominator / 2) / denominator);
    }
    return (int32_t) -((-numerator + denominator / 2) / denominator);
}

static int32_t sign_extend_24(uint32_t value) {
    if ((value & UINT32_C(0x00800000)) != 0U) {
        value |= UINT32_C(0xFF000000);
    }
    return (int32_t) value;
}

bool bmp581_decode_sample(const uint8_t data[6], int64_t timestamp_us, bmp581_sample_t *sample) {
    uint32_t raw_temperature = 0U;
    uint32_t raw_pressure = 0U;

    if (data == NULL || sample == NULL || timestamp_us <= 0) {
        return false;
    }

    memset(sample, 0, sizeof(*sample));
    raw_temperature = (uint32_t) data[0] | ((uint32_t) data[1] << 8U) | ((uint32_t) data[2] << 16U);
    raw_pressure = (uint32_t) data[3] | ((uint32_t) data[4] << 8U) | ((uint32_t) data[5] << 16U);

    sample->timestamp_us = timestamp_us;
    sample->raw_temperature = sign_extend_24(raw_temperature);
    sample->raw_pressure = raw_pressure;
    sample->temperature_c_x100 =
        divide_round_nearest((int64_t) sample->raw_temperature * INT64_C(100), INT64_C(65536));
    sample->pressure_pa_x100 =
        divide_round_nearest((int64_t) sample->raw_pressure * INT64_C(100), INT64_C(64));
    sample->valid = sample->temperature_c_x100 >= BMP581_TEMPERATURE_MIN_C_X100 &&
                    sample->temperature_c_x100 <= BMP581_TEMPERATURE_MAX_C_X100 &&
                    sample->pressure_pa_x100 >= BMP581_PRESSURE_MIN_PA_X100 &&
                    sample->pressure_pa_x100 <= BMP581_PRESSURE_MAX_PA_X100;
    return sample->valid;
}

esp_err_t bmp581_init(i2c_master_bus_handle_t bus_handle) {
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP581_I2C_ADDRESS,
        .scl_speed_hz = BOARD_BMP581_I2C_SPEED_HZ,
    };
    uint8_t chip_id = 0U;
    uint8_t interrupt_status = 0U;
    uint8_t status = 0U;
    uint8_t dsp_readback[2] = {0U};
    uint8_t measurement_readback[2] = {0U};
    uint8_t osr_effective = 0U;
    esp_err_t ret = ESP_OK;

    if (bus_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (bmp581_device != NULL) {
        ret = bmp581_deinit();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    ret = i2c_master_bus_add_device(bus_handle, &device_config, &bmp581_device);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = bmp581_read_registers(BMP581_REG_CHIP_ID, &chip_id, sizeof(chip_id));
    if (ret == ESP_OK && chip_id != BMP581_CHIP_ID) {
        ret = ESP_ERR_NOT_FOUND;
    }
    if (ret == ESP_OK) {
        ret = bmp581_write_register(BMP581_REG_COMMAND, BMP581_SOFT_RESET_COMMAND);
    }
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(BMP581_RESET_SETTLE_MS));
        ret = bmp581_read_registers(BMP581_REG_INT_STATUS, &interrupt_status,
                                    sizeof(interrupt_status));
    }
    if (ret == ESP_OK) {
        ret = bmp581_read_registers(BMP581_REG_STATUS, &status, sizeof(status));
    }
    if (ret == ESP_OK &&
        ((interrupt_status & BMP581_POR_COMPLETE_MASK) == 0U ||
         (status & BMP581_NVM_READY_MASK) == 0U || (status & BMP581_NVM_ERROR_MASK) != 0U)) {
        ret = ESP_ERR_INVALID_STATE;
    }
    if (ret == ESP_OK) {
        ret = bmp581_write_register(BMP581_REG_DSP_CONFIG, BMP581_DSP_CONFIG_VALUE);
    }
    if (ret == ESP_OK) {
        ret = bmp581_write_register(BMP581_REG_DSP_IIR, BMP581_DSP_IIR_VALUE);
    }
    if (ret == ESP_OK) {
        ret = bmp581_write_register(BMP581_REG_OSR_CONFIG, BMP581_OSR_CONFIG_VALUE);
    }
    if (ret == ESP_OK) {
        ret = bmp581_write_register(BMP581_REG_ODR_CONFIG, BMP581_ODR_CONFIG_VALUE);
    }
    if (ret == ESP_OK) {
        ret = bmp581_read_registers(BMP581_REG_DSP_CONFIG, dsp_readback, sizeof(dsp_readback));
    }
    if (ret == ESP_OK) {
        ret = bmp581_read_registers(BMP581_REG_OSR_CONFIG, measurement_readback,
                                    sizeof(measurement_readback));
    }
    if (ret == ESP_OK) {
        ret = bmp581_read_registers(BMP581_REG_OSR_EFF, &osr_effective, sizeof(osr_effective));
    }
    if (ret == ESP_OK &&
        (dsp_readback[0] != BMP581_DSP_CONFIG_VALUE || dsp_readback[1] != BMP581_DSP_IIR_VALUE ||
         measurement_readback[0] != BMP581_OSR_CONFIG_VALUE ||
         measurement_readback[1] != BMP581_ODR_CONFIG_VALUE ||
         (osr_effective & BMP581_ODR_VALID_MASK) == 0U)) {
        ret = ESP_ERR_INVALID_STATE;
    }

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "initialization failed: %s", esp_err_to_name(ret));
        (void) bmp581_remove_device();
        return ret;
    }

    ESP_LOGI(TAG, "online address=0x%02x chip_id=0x%02x", BMP581_I2C_ADDRESS, chip_id);
    return ESP_OK;
}

esp_err_t bmp581_deinit(void) {
    esp_err_t first_error = ESP_OK;
    esp_err_t ret = ESP_OK;

    if (bmp581_device == NULL) {
        return ESP_OK;
    }

    first_error = bmp581_write_register(BMP581_REG_ODR_CONFIG, BMP581_ODR_STANDBY_VALUE);
    ret = bmp581_remove_device();
    if (first_error == ESP_OK) {
        first_error = ret;
    }
    return first_error;
}

esp_err_t bmp581_read_sample(bmp581_sample_t *sample) {
    uint8_t data[6] = {0U};
    esp_err_t ret = ESP_OK;

    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(sample, 0, sizeof(*sample));

    ret = bmp581_read_registers(BMP581_REG_DATA_START, data, sizeof(data));
    if (ret != ESP_OK) {
        return ret;
    }

    (void) bmp581_decode_sample(data, esp_timer_get_time(), sample);
    return ESP_OK;
}
