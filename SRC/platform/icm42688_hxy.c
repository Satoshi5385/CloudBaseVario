#include "platform/icm42688_hxy.h"

#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "platform/board.h"

#define ICM42688_HXY_I2C_TIMEOUT_MS 5
#define ICM42688_HXY_REGISTER_WRITE_DELAY_MS UINT32_C(1)
#define ICM42688_HXY_POWER_START_DELAY_MS UINT32_C(10)
#define ICM42688_HXY_RESET_DELAY_MS UINT32_C(10)
#define ICM42688_HXY_SENSOR_START_DELAY_MS UINT32_C(50)

#define ICM42688_HXY_REG_COM_CFG UINT8_C(0x05)
#define ICM42688_HXY_REG_INT_CFG1 UINT8_C(0x06)
#define ICM42688_HXY_REG_DATA_STAT UINT8_C(0x0B)
#define ICM42688_HXY_REG_ACC_CONF UINT8_C(0x40)
#define ICM42688_HXY_REG_ACC_RANGE UINT8_C(0x41)
#define ICM42688_HXY_REG_GYR_CONF UINT8_C(0x42)
#define ICM42688_HXY_REG_GYR_RANGE UINT8_C(0x43)
#define ICM42688_HXY_REG_SOFT_RST UINT8_C(0x4A)
#define ICM42688_HXY_REG_PWR_CTRL UINT8_C(0x7D)

#define ICM42688_HXY_SOFT_RESET_VALUE UINT8_C(0xA5)
#define ICM42688_HXY_COM_CFG_VALUE UINT8_C(0x50)
#define ICM42688_HXY_INT1_GYRO_DATA_READY UINT8_C(0x03)
#define ICM42688_HXY_ACC_CONF_400HZ_VALUE UINT8_C(0xAA)
#define ICM42688_HXY_ACC_RANGE_8G_VALUE UINT8_C(0x02)
#define ICM42688_HXY_GYR_CONF_400HZ_VALUE UINT8_C(0xAA)
#define ICM42688_HXY_GYR_RANGE_2000DPS_VALUE UINT8_C(0x00)
#define ICM42688_HXY_PWR_ACCEL_GYRO_TEMP_VALUE UINT8_C(0x0E)
#define ICM42688_HXY_PWR_OFF_VALUE UINT8_C(0x00)

#define ICM42688_HXY_DATA_STATUS_CONFIG_ERROR_MASK UINT8_C(0x30)
#define ICM42688_HXY_DATA_STATUS_READY_MASK UINT8_C(0x03)
#define ICM42688_HXY_FRAME_LENGTH UINT32_C(13)
#define ICM42688_HXY_ACCEL_LSB_PER_G 4096.0f
#define ICM42688_HXY_GYRO_DPS_PER_LSB 0.061f
#define ICM42688_HXY_STANDARD_GRAVITY_MPS2 9.80665f
#define ICM42688_HXY_DEGREES_TO_RADIANS 0.01745329252f

static const char *TAG = "icm42688_hxy";
static i2c_master_dev_handle_t icm42688_hxy_device = NULL;
static bool icm42688_hxy_isr_registered = false;

_Static_assert(ICM42688_HXY_I2C_ADDRESS == UINT16_C(0x18),
               "Aohazuku Rev.0 fixes the HXY IMU SDO pin Low");
_Static_assert(BOARD_ICM42688_HXY_I2C_SPEED_HZ == UINT32_C(400000),
               "ICM-42688P-HXY I2C must not exceed 400 kHz");
_Static_assert(ICM42688_HXY_SAMPLE_RATE_HZ == UINT32_C(400),
               "HXY ODR table has no 500 Hz setting");

static void delay_ms(uint32_t delay_time_ms) {
    vTaskDelay(pdMS_TO_TICKS(delay_time_ms));
}

static esp_err_t read_registers(uint8_t register_address, uint8_t *data,
                                size_t data_length) {
    if (icm42688_hxy_device == NULL || data == NULL || data_length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_master_transmit_receive(
        icm42688_hxy_device, &register_address, sizeof(register_address),
        data, data_length, ICM42688_HXY_I2C_TIMEOUT_MS);
}

static esp_err_t read_register(uint8_t register_address, uint8_t *value) {
    return read_registers(register_address, value, sizeof(*value));
}

static esp_err_t write_register(uint8_t register_address, uint8_t value) {
    uint8_t transaction[2] = {register_address, value};

    if (icm42688_hxy_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit(icm42688_hxy_device, transaction,
                               sizeof(transaction),
                               ICM42688_HXY_I2C_TIMEOUT_MS);
}

static esp_err_t write_and_verify_register(uint8_t register_address,
                                           uint8_t value) {
    uint8_t read_back = 0U;
    esp_err_t ret = write_register(register_address, value);

    if (ret != ESP_OK) {
        return ret;
    }
    delay_ms(ICM42688_HXY_REGISTER_WRITE_DELAY_MS);
    ret = read_register(register_address, &read_back);
    if (ret != ESP_OK) {
        return ret;
    }
    if (read_back != value) {
        ESP_LOGE(TAG,
                 "register 0x%02x read-back mismatch expected=0x%02x actual=0x%02x",
                 register_address, value, read_back);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t remove_device_handle(void) {
    esp_err_t ret = ESP_OK;

    if (icm42688_hxy_device == NULL) {
        return ESP_OK;
    }
    ret = i2c_master_bus_rm_device(icm42688_hxy_device);
    if (ret == ESP_OK) {
        icm42688_hxy_device = NULL;
    }
    return ret;
}

static void IRAM_ATTR data_ready_isr(void *context) {
    BaseType_t high_priority_task_woken = pdFALSE;
    TaskHandle_t task = (TaskHandle_t) context;

    if (task != NULL) {
        vTaskNotifyGiveFromISR(task, &high_priority_task_woken);
    }
    if (high_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t configure_data_ready_gpio(TaskHandle_t sensor_task) {
    gpio_config_t data_ready_gpio_config = {
        .pin_bit_mask = UINT64_C(1) << PIN_INT_ICM,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    esp_err_t ret = ESP_OK;

    if (sensor_task == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ret = gpio_config(&data_ready_gpio_config);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ret = gpio_isr_handler_add(PIN_INT_ICM, data_ready_isr, sensor_task);
    if (ret != ESP_OK) {
        return ret;
    }
    icm42688_hxy_isr_registered = true;
    return ESP_OK;
}

static esp_err_t disable_data_ready_gpio(void) {
    esp_err_t first_error = ESP_OK;
    esp_err_t ret = gpio_intr_disable(PIN_INT_ICM);

    if (ret != ESP_OK) {
        first_error = ret;
    }
    if (icm42688_hxy_isr_registered) {
        ret = gpio_isr_handler_remove(PIN_INT_ICM);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        if (ret == ESP_OK) {
            icm42688_hxy_isr_registered = false;
        }
    }
    ret = gpio_reset_pin(PIN_INT_ICM);
    if (ret != ESP_OK && first_error == ESP_OK) {
        first_error = ret;
    }
    return first_error;
}

static esp_err_t verify_identity(icm42688_hxy_identity_t *identity) {
    uint8_t who_am_i = 0U;
    esp_err_t ret = read_register(ICM42688_HXY_WHO_AM_I_REGISTER, &who_am_i);

    identity->who_am_i = who_am_i;
    if (ret != ESP_OK) {
        return ret;
    }
    if (who_am_i != ICM42688_HXY_WHO_AM_I_VALUE) {
        ESP_LOGE(TAG,
                 "identity mismatch expected=0x%02x actual=0x%02x",
                 ICM42688_HXY_WHO_AM_I_VALUE, who_am_i);
        return ESP_ERR_INVALID_RESPONSE;
    }
    identity->address = (uint8_t) ICM42688_HXY_I2C_ADDRESS;
    return ESP_OK;
}

static esp_err_t configure_sensor(void) {
    const uint8_t registers[][2] = {
        {ICM42688_HXY_REG_COM_CFG, ICM42688_HXY_COM_CFG_VALUE},
        {ICM42688_HXY_REG_ACC_CONF,
         ICM42688_HXY_ACC_CONF_400HZ_VALUE},
        {ICM42688_HXY_REG_ACC_RANGE,
         ICM42688_HXY_ACC_RANGE_8G_VALUE},
        {ICM42688_HXY_REG_GYR_CONF,
         ICM42688_HXY_GYR_CONF_400HZ_VALUE},
        {ICM42688_HXY_REG_GYR_RANGE,
         ICM42688_HXY_GYR_RANGE_2000DPS_VALUE},
        {ICM42688_HXY_REG_INT_CFG1,
         ICM42688_HXY_INT1_GYRO_DATA_READY},
    };
    uint8_t data_status = 0U;
    esp_err_t ret = ESP_OK;

    ret = write_register(ICM42688_HXY_REG_SOFT_RST,
                         ICM42688_HXY_SOFT_RESET_VALUE);
    if (ret != ESP_OK) {
        return ret;
    }
    delay_ms(ICM42688_HXY_RESET_DELAY_MS);

    ret = write_and_verify_register(
        ICM42688_HXY_REG_PWR_CTRL,
        ICM42688_HXY_PWR_ACCEL_GYRO_TEMP_VALUE);
    if (ret != ESP_OK) {
        return ret;
    }
    delay_ms(ICM42688_HXY_POWER_START_DELAY_MS);

    for (size_t index = 0U;
         index < sizeof(registers) / sizeof(registers[0]); index++) {
        ret = write_and_verify_register(registers[index][0],
                                        registers[index][1]);
        if (ret != ESP_OK) {
            return ret;
        }
    }
    delay_ms(ICM42688_HXY_SENSOR_START_DELAY_MS);
    ret = read_register(ICM42688_HXY_REG_DATA_STAT, &data_status);
    if (ret != ESP_OK) {
        return ret;
    }
    if ((data_status & ICM42688_HXY_DATA_STATUS_CONFIG_ERROR_MASK) != 0U) {
        ESP_LOGE(TAG, "HXY configuration error status=0x%02x",
                 data_status);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t icm42688_hxy_init(i2c_master_bus_handle_t bus_handle,
                            TaskHandle_t sensor_task,
                            icm42688_hxy_identity_t *identity) {
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM42688_HXY_I2C_ADDRESS,
        .scl_speed_hz = BOARD_ICM42688_HXY_I2C_SPEED_HZ,
    };
    esp_err_t ret = ESP_OK;

    if (bus_handle == NULL || sensor_task == NULL || identity == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(identity, 0, sizeof(*identity));
    if (icm42688_hxy_device != NULL || icm42688_hxy_isr_registered) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = i2c_master_probe(bus_handle, ICM42688_HXY_I2C_ADDRESS,
                           ICM42688_HXY_I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "no ACK at fixed address 0x%02x: %s",
                 ICM42688_HXY_I2C_ADDRESS, esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_master_bus_add_device(bus_handle, &device_config,
                                    &icm42688_hxy_device);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = verify_identity(identity);
    if (ret != ESP_OK) {
        (void) remove_device_handle();
        return ret;
    }
    ret = configure_sensor();
    if (ret == ESP_OK) {
        ret = verify_identity(identity);
    }
    if (ret == ESP_OK) {
        ret = configure_data_ready_gpio(sensor_task);
    }
    if (ret != ESP_OK) {
        (void) icm42688_hxy_deinit();
        return ret;
    }

    ESP_LOGI(TAG,
             "online address=0x%02x who_am_i=0x%02x odr=%" PRIu32
             "Hz accel=+/-%.0fg gyro=+/-%.0fdps",
             identity->address, identity->who_am_i,
             ICM42688_HXY_SAMPLE_RATE_HZ,
             (double) ICM42688_HXY_ACCEL_RANGE_G,
             (double) ICM42688_HXY_GYRO_RANGE_DPS);
    return ESP_OK;
}

static int16_t decode_be_int16(uint8_t high_byte, uint8_t low_byte) {
    uint16_t combined = ((uint16_t) high_byte << 8U) | low_byte;

    return (int16_t) combined;
}

esp_err_t icm42688_hxy_read_sample(icm42688_hxy_sample_t *sample) {
    uint8_t frame[ICM42688_HXY_FRAME_LENGTH] = {0};
    esp_err_t ret = ESP_OK;

    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(sample, 0, sizeof(*sample));
    ret = read_registers(ICM42688_HXY_REG_DATA_STAT, frame,
                         sizeof(frame));
    if (ret != ESP_OK) {
        return ret;
    }
    sample->data_status = frame[0];
    if ((frame[0] & ICM42688_HXY_DATA_STATUS_CONFIG_ERROR_MASK) != 0U) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((frame[0] & ICM42688_HXY_DATA_STATUS_READY_MASK) !=
        ICM42688_HXY_DATA_STATUS_READY_MASK) {
        return ESP_ERR_NOT_FINISHED;
    }

    for (size_t axis = 0U; axis < 3U; axis++) {
        size_t accel_offset = 1U + axis * 2U;
        size_t gyro_offset = 7U + axis * 2U;

        sample->raw_accel[axis] =
            decode_be_int16(frame[accel_offset],
                            frame[accel_offset + 1U]);
        sample->raw_gyro[axis] =
            decode_be_int16(frame[gyro_offset],
                            frame[gyro_offset + 1U]);
        sample->accel_mps2[axis] =
            ((float) sample->raw_accel[axis] /
             ICM42688_HXY_ACCEL_LSB_PER_G) *
            ICM42688_HXY_STANDARD_GRAVITY_MPS2;
        sample->gyro_radps[axis] =
            (float) sample->raw_gyro[axis] *
            ICM42688_HXY_GYRO_DPS_PER_LSB *
            ICM42688_HXY_DEGREES_TO_RADIANS;
        if (!isfinite(sample->accel_mps2[axis]) ||
            !isfinite(sample->gyro_radps[axis])) {
            return ESP_ERR_INVALID_RESPONSE;
        }
    }
    sample->timestamp_us = esp_timer_get_time();
    sample->valid = true;
    return ESP_OK;
}

esp_err_t icm42688_hxy_deinit(void) {
    esp_err_t first_error = ESP_OK;
    esp_err_t ret = disable_data_ready_gpio();

    if (ret != ESP_OK) {
        first_error = ret;
    }
    if (icm42688_hxy_device != NULL) {
        ret = write_register(ICM42688_HXY_REG_PWR_CTRL,
                             ICM42688_HXY_PWR_OFF_VALUE);
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
        ret = remove_device_handle();
        if (ret != ESP_OK && first_error == ESP_OK) {
            first_error = ret;
        }
    }
    return first_error;
}
