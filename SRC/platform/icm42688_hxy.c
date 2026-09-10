#include "platform/icm42688_hxy.h"

#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"
#include "platform/board.h"
#include "soc/gpio_struct.h"

#define ICM42688_HXY_I2C_TIMEOUT_MS 5
#define ICM42688_HXY_REGISTER_WRITE_DELAY_MS UINT32_C(1)
#define ICM42688_HXY_POWER_START_DELAY_MS UINT32_C(10)
#define ICM42688_HXY_RESET_DELAY_MS UINT32_C(10)
#define ICM42688_HXY_SENSOR_START_DELAY_MS UINT32_C(50)

#define ICM42688_HXY_REG_COM_CFG UINT8_C(0x05)
#define ICM42688_HXY_REG_INT_CFG1 UINT8_C(0x06)
#define ICM42688_HXY_REG_DATA_STAT UINT8_C(0x0B)
#define ICM42688_HXY_REG_FIFO_CFG0 UINT8_C(0x1C)
#define ICM42688_HXY_REG_FIFO_CFG1 UINT8_C(0x1D)
#define ICM42688_HXY_REG_FIFO_CFG2 UINT8_C(0x1E)
#define ICM42688_HXY_REG_FIFO_DATA UINT8_C(0x21)
#define ICM42688_HXY_REG_FIFO_DOWNS UINT8_C(0x45)
#define ICM42688_HXY_REG_ACC_CONF UINT8_C(0x40)
#define ICM42688_HXY_REG_ACC_RANGE UINT8_C(0x41)
#define ICM42688_HXY_REG_GYR_CONF UINT8_C(0x42)
#define ICM42688_HXY_REG_GYR_RANGE UINT8_C(0x43)
#define ICM42688_HXY_REG_SOFT_RST UINT8_C(0x4A)
#define ICM42688_HXY_REG_PWR_CTRL UINT8_C(0x7D)

#define ICM42688_HXY_SOFT_RESET_VALUE UINT8_C(0xA5)
#define ICM42688_HXY_COM_CFG_VALUE UINT8_C(0x40)
/* HXY datasheet sections 5.3, 5.16-5.21, 5.41: fixed-address FIFO reads,
 * full-width gyro XYZ then accel XYZ, no headers, no downsampling. */
#define ICM42688_HXY_INT1_FIFO_WATERMARK UINT8_C(0x09)
#define ICM42688_HXY_FIFO_ACCEL_GYRO UINT8_C(0x06)
#define ICM42688_HXY_FIFO_BYPASS UINT8_C(0x00)
#define ICM42688_HXY_FIFO_MODE UINT8_C(0x10)
#define ICM42688_HXY_FIFO_FILTERED_400HZ UINT8_C(0x88)
#define ICM42688_HXY_FIFO_TIME_BYTES UINT32_C(4)
#define ICM42688_HXY_FIFO_SAMPLE_BYTES UINT32_C(12)
#define ICM42688_HXY_FIFO_WATERMARK_SAMPLES UINT32_C(4)
#define ICM42688_HXY_FIFO_BUFFER_BYTES \
    (ICM42688_HXY_FIFO_TIME_BYTES + \
     ICM42688_HXY_FIFO_WATERMARK_SAMPLES * \
         ICM42688_HXY_FIFO_SAMPLE_BYTES)
#define ICM42688_HXY_FIFO_WORD_BYTES UINT32_C(2)
/* WTM asserts when the word count exceeds FTH. Sensor Time is two words. */
#define ICM42688_HXY_FIFO_WATERMARK_THRESHOLD_WORDS \
    ((ICM42688_HXY_FIFO_TIME_BYTES + \
      ICM42688_HXY_FIFO_WATERMARK_SAMPLES * ICM42688_HXY_FIFO_SAMPLE_BYTES) / \
     ICM42688_HXY_FIFO_WORD_BYTES - UINT32_C(1))
#define ICM42688_HXY_FIFO_THRESHOLD_HIGH \
    ((uint8_t) ((ICM42688_HXY_FIFO_WATERMARK_THRESHOLD_WORDS >> 8U) & 0x07U))
#define ICM42688_HXY_FIFO_THRESHOLD_LOW \
    ((uint8_t) (ICM42688_HXY_FIFO_WATERMARK_THRESHOLD_WORDS & 0xFFU))
#define ICM42688_HXY_FIFO_MODE_CONFIG \
    (ICM42688_HXY_FIFO_MODE | ICM42688_HXY_FIFO_THRESHOLD_HIGH)
#define ICM42688_HXY_ACC_CONF_400HZ_VALUE UINT8_C(0xAA)
#define ICM42688_HXY_ACC_RANGE_8G_VALUE UINT8_C(0x02)
#define ICM42688_HXY_GYR_CONF_400HZ_VALUE UINT8_C(0xAA)
#define ICM42688_HXY_GYR_RANGE_2000DPS_VALUE UINT8_C(0x00)
#define ICM42688_HXY_PWR_ACCEL_GYRO_TEMP_VALUE UINT8_C(0x0E)
#define ICM42688_HXY_PWR_OFF_VALUE UINT8_C(0x00)

#define ICM42688_HXY_DATA_STATUS_CONFIG_ERROR_MASK UINT8_C(0x30)
#define ICM42688_HXY_ACCEL_LSB_PER_G 4096.0f
#define ICM42688_HXY_GYRO_DPS_PER_LSB 0.061f
#define ICM42688_HXY_STANDARD_GRAVITY_MPS2 9.80665f
#define ICM42688_HXY_DEGREES_TO_RADIANS 0.01745329252f

static const char *TAG = "icm42688_hxy";
static i2c_master_dev_handle_t icm42688_hxy_device = NULL;
static bool icm42688_hxy_isr_registered = false;
static bool icm42688_hxy_fifo_needs_restart = false;
static int64_t icm42688_hxy_last_sample_us = 0;
static int64_t icm42688_hxy_fifo_read_after_us = 0;
static uint8_t icm42688_hxy_last_data_status = 0U;

typedef enum {
    ICM42688_HXY_INIT_IDLE = 0,
    ICM42688_HXY_INIT_PROBE,
    ICM42688_HXY_INIT_ADD_DEVICE,
    ICM42688_HXY_INIT_VERIFY_FIRST,
    ICM42688_HXY_INIT_RESET,
    ICM42688_HXY_INIT_WAIT_RESET,
    ICM42688_HXY_INIT_POWER,
    ICM42688_HXY_INIT_VERIFY_POWER,
    ICM42688_HXY_INIT_WAIT_POWER,
    ICM42688_HXY_INIT_CONFIGURE,
    ICM42688_HXY_INIT_VERIFY_CONFIG,
    ICM42688_HXY_INIT_WAIT_SENSOR,
    ICM42688_HXY_INIT_CHECK_STATUS,
    ICM42688_HXY_INIT_VERIFY_SECOND,
    ICM42688_HXY_INIT_CONFIGURE_GPIO,
    ICM42688_HXY_INIT_FIFO_BYPASS,
    ICM42688_HXY_INIT_FIFO_ENABLE,
    ICM42688_HXY_INIT_WAIT_FIFO,
    ICM42688_HXY_INIT_VERIFY_FIFO,
} icm42688_hxy_init_phase_t;

static const uint8_t icm42688_hxy_configuration[][2] = {
    {ICM42688_HXY_REG_COM_CFG, ICM42688_HXY_COM_CFG_VALUE},
    {ICM42688_HXY_REG_ACC_CONF, ICM42688_HXY_ACC_CONF_400HZ_VALUE},
    {ICM42688_HXY_REG_ACC_RANGE, ICM42688_HXY_ACC_RANGE_8G_VALUE},
    {ICM42688_HXY_REG_GYR_CONF, ICM42688_HXY_GYR_CONF_400HZ_VALUE},
    {ICM42688_HXY_REG_GYR_RANGE, ICM42688_HXY_GYR_RANGE_2000DPS_VALUE},
    {ICM42688_HXY_REG_FIFO_DOWNS, ICM42688_HXY_FIFO_FILTERED_400HZ},
    {ICM42688_HXY_REG_FIFO_CFG0, ICM42688_HXY_FIFO_ACCEL_GYRO},
    {ICM42688_HXY_REG_FIFO_CFG2, ICM42688_HXY_FIFO_THRESHOLD_LOW},
    {ICM42688_HXY_REG_INT_CFG1, ICM42688_HXY_INT1_FIFO_WATERMARK},
};
static icm42688_hxy_init_phase_t icm42688_hxy_init_phase =
    ICM42688_HXY_INIT_IDLE;
static i2c_master_bus_handle_t icm42688_hxy_init_bus = NULL;
static TaskHandle_t icm42688_hxy_init_task = NULL;
static icm42688_hxy_identity_t icm42688_hxy_init_identity;
static size_t icm42688_hxy_init_config_index = 0U;
static int64_t icm42688_hxy_init_next_us = 0;

_Static_assert(ICM42688_HXY_I2C_ADDRESS == UINT16_C(0x18),
               "Aohazuku Rev.0 fixes the HXY IMU SDO pin Low");
_Static_assert(BOARD_ICM42688_HXY_I2C_SPEED_HZ == UINT32_C(400000),
               "ICM-42688P-HXY I2C must not exceed 400 kHz");
_Static_assert(ICM42688_HXY_SAMPLE_RATE_HZ == UINT32_C(400),
               "HXY ODR table has no 500 Hz setting");

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

static esp_err_t verify_register(uint8_t register_address, uint8_t value) {
    uint8_t read_back = 0U;
    esp_err_t ret = read_register(register_address, &read_back);
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

static void IRAM_ATTR fifo_watermark_isr(void *context) {
    BaseType_t high_priority_task_woken = pdFALSE;
    TaskHandle_t task = (TaskHandle_t) context;

    /* HXY WTM may pulse again while the count remains above FTH. Mask the
     * source until BY-PASS -> FIFO has cleared this batch. */
    gpio_ll_intr_disable(&GPIO, PIN_INT_ICM);
    if (task != NULL) {
        vTaskNotifyGiveFromISR(task, &high_priority_task_woken);
    }
    if (high_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t configure_fifo_gpio(TaskHandle_t sensor_task) {
    const gpio_config_t input_config = {
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
    ret = gpio_config(&input_config);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    ret = gpio_isr_handler_add(PIN_INT_ICM, fifo_watermark_isr, sensor_task);
    if (ret == ESP_OK) {
        icm42688_hxy_isr_registered = true;
    }
    return ret;
}

static esp_err_t disable_fifo_gpio(void) {
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

static esp_err_t restart_fifo(void) {
    esp_err_t ret = write_register(ICM42688_HXY_REG_FIFO_CFG1,
                                   ICM42688_HXY_FIFO_BYPASS);

    icm42688_hxy_fifo_needs_restart = true;
    if (ret == ESP_OK) {
        ret = write_register(ICM42688_HXY_REG_FIFO_CFG1,
                             ICM42688_HXY_FIFO_MODE_CONFIG);
    }
    if (ret == ESP_OK) {
        icm42688_hxy_fifo_needs_restart = false;
        icm42688_hxy_fifo_read_after_us = esp_timer_get_time() +
            (int64_t) ICM42688_HXY_REGISTER_WRITE_DELAY_MS * INT64_C(1000);
        ret = gpio_intr_enable(PIN_INT_ICM);
        if (ret != ESP_OK) {
            icm42688_hxy_fifo_needs_restart = true;
        }
    }
    /* The reader enforces the HXY 1 ms configuration-to-read delay, even
     * when a late burst ends close to the next absolute 10 ms deadline. */
    return ret;
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

static void clear_init_state(void) {
    icm42688_hxy_init_phase = ICM42688_HXY_INIT_IDLE;
    icm42688_hxy_init_bus = NULL;
    icm42688_hxy_init_task = NULL;
    memset(&icm42688_hxy_init_identity, 0,
           sizeof(icm42688_hxy_init_identity));
    icm42688_hxy_init_config_index = 0U;
    icm42688_hxy_init_next_us = 0;
}

static esp_err_t fail_initialization(esp_err_t error) {
    if (icm42688_hxy_isr_registered) {
        (void) disable_fifo_gpio();
    }
    (void) remove_device_handle();
    icm42688_hxy_fifo_needs_restart = false;
    clear_init_state();
    return error;
}

esp_err_t icm42688_hxy_init_begin(i2c_master_bus_handle_t bus_handle,
                                  TaskHandle_t sensor_task) {
    if (bus_handle == NULL || sensor_task == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (icm42688_hxy_init_phase != ICM42688_HXY_INIT_IDLE ||
        icm42688_hxy_device != NULL || icm42688_hxy_isr_registered) {
        return ESP_ERR_INVALID_STATE;
    }
    icm42688_hxy_init_bus = bus_handle;
    icm42688_hxy_init_task = sensor_task;
    memset(&icm42688_hxy_init_identity, 0,
           sizeof(icm42688_hxy_init_identity));
    icm42688_hxy_init_config_index = 0U;
    icm42688_hxy_init_next_us = esp_timer_get_time();
    icm42688_hxy_init_phase = ICM42688_HXY_INIT_PROBE;
    return ESP_OK;
}

int64_t icm42688_hxy_init_next_action_us(void) {
    return icm42688_hxy_init_next_us;
}

bool icm42688_hxy_init_in_progress(void) {
    return icm42688_hxy_init_phase != ICM42688_HXY_INIT_IDLE;
}

esp_err_t icm42688_hxy_init_abort(void) {
    return icm42688_hxy_deinit();
}

esp_err_t icm42688_hxy_init_poll(int64_t now_us,
                                 icm42688_hxy_identity_t *identity) {
    const i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM42688_HXY_I2C_ADDRESS,
        .scl_speed_hz = BOARD_ICM42688_HXY_I2C_SPEED_HZ,
    };
    uint8_t data_status = 0U;
    esp_err_t ret = ESP_OK;

    if (identity == NULL ||
        icm42688_hxy_init_phase == ICM42688_HXY_INIT_IDLE) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(identity, 0, sizeof(*identity));
    if (now_us < icm42688_hxy_init_next_us) {
        return ESP_ERR_NOT_FINISHED;
    }
    switch (icm42688_hxy_init_phase) {
        case ICM42688_HXY_INIT_PROBE:
            ret = i2c_master_probe(icm42688_hxy_init_bus,
                                   ICM42688_HXY_I2C_ADDRESS,
                                   ICM42688_HXY_I2C_TIMEOUT_MS);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "no ACK at fixed address 0x%02x: %s",
                         ICM42688_HXY_I2C_ADDRESS, esp_err_to_name(ret));
                return fail_initialization(ret);
            }
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_ADD_DEVICE;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_ADD_DEVICE:
            ret = i2c_master_bus_add_device(icm42688_hxy_init_bus,
                                            &device_config,
                                            &icm42688_hxy_device);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_VERIFY_FIRST;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_VERIFY_FIRST:
            ret = verify_identity(&icm42688_hxy_init_identity);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_RESET;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_RESET:
            ret = write_register(ICM42688_HXY_REG_SOFT_RST,
                                 ICM42688_HXY_SOFT_RESET_VALUE);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            icm42688_hxy_init_next_us = now_us +
                (int64_t) ICM42688_HXY_RESET_DELAY_MS * INT64_C(1000);
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_WAIT_RESET;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_WAIT_RESET:
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_POWER;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_POWER:
            ret = write_register(ICM42688_HXY_REG_PWR_CTRL,
                                 ICM42688_HXY_PWR_ACCEL_GYRO_TEMP_VALUE);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            icm42688_hxy_init_next_us = now_us +
                (int64_t) ICM42688_HXY_REGISTER_WRITE_DELAY_MS * INT64_C(1000);
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_VERIFY_POWER;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_VERIFY_POWER:
            ret = verify_register(ICM42688_HXY_REG_PWR_CTRL,
                                  ICM42688_HXY_PWR_ACCEL_GYRO_TEMP_VALUE);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            icm42688_hxy_init_next_us = now_us +
                (int64_t) ICM42688_HXY_POWER_START_DELAY_MS * INT64_C(1000);
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_WAIT_POWER;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_WAIT_POWER:
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_CONFIGURE;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_CONFIGURE:
            ret = write_register(
                icm42688_hxy_configuration[icm42688_hxy_init_config_index][0],
                icm42688_hxy_configuration[icm42688_hxy_init_config_index][1]);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            icm42688_hxy_init_next_us = now_us +
                (int64_t) ICM42688_HXY_REGISTER_WRITE_DELAY_MS * INT64_C(1000);
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_VERIFY_CONFIG;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_VERIFY_CONFIG:
            ret = verify_register(
                icm42688_hxy_configuration[icm42688_hxy_init_config_index][0],
                icm42688_hxy_configuration[icm42688_hxy_init_config_index][1]);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            icm42688_hxy_init_config_index++;
            if (icm42688_hxy_init_config_index ==
                sizeof(icm42688_hxy_configuration) /
                    sizeof(icm42688_hxy_configuration[0])) {
                icm42688_hxy_init_next_us = now_us +
                    (int64_t) ICM42688_HXY_SENSOR_START_DELAY_MS * INT64_C(1000);
                icm42688_hxy_init_phase = ICM42688_HXY_INIT_WAIT_SENSOR;
            } else {
                icm42688_hxy_init_phase = ICM42688_HXY_INIT_CONFIGURE;
            }
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_WAIT_SENSOR:
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_CHECK_STATUS;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_CHECK_STATUS:
            ret = read_register(ICM42688_HXY_REG_DATA_STAT, &data_status);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            if ((data_status & ICM42688_HXY_DATA_STATUS_CONFIG_ERROR_MASK) != 0U) {
                ESP_LOGE(TAG, "HXY configuration error status=0x%02x",
                         data_status);
                return fail_initialization(ESP_ERR_INVALID_STATE);
            }
            icm42688_hxy_last_data_status = data_status;
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_VERIFY_SECOND;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_VERIFY_SECOND:
            ret = verify_identity(&icm42688_hxy_init_identity);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_CONFIGURE_GPIO;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_CONFIGURE_GPIO:
            ret = configure_fifo_gpio(icm42688_hxy_init_task);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_FIFO_BYPASS;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_FIFO_BYPASS:
            icm42688_hxy_fifo_needs_restart = true;
            ret = write_register(ICM42688_HXY_REG_FIFO_CFG1,
                                 ICM42688_HXY_FIFO_BYPASS);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_FIFO_ENABLE;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_FIFO_ENABLE:
            ret = write_register(ICM42688_HXY_REG_FIFO_CFG1,
                                 ICM42688_HXY_FIFO_MODE_CONFIG);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            icm42688_hxy_fifo_needs_restart = false;
            icm42688_hxy_fifo_read_after_us = now_us +
                (int64_t) ICM42688_HXY_REGISTER_WRITE_DELAY_MS * INT64_C(1000);
            ret = gpio_intr_enable(PIN_INT_ICM);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            icm42688_hxy_init_next_us = now_us +
                (int64_t) ICM42688_HXY_REGISTER_WRITE_DELAY_MS * INT64_C(1000);
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_WAIT_FIFO;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_WAIT_FIFO:
            icm42688_hxy_init_phase = ICM42688_HXY_INIT_VERIFY_FIFO;
            return ESP_ERR_NOT_FINISHED;
        case ICM42688_HXY_INIT_VERIFY_FIFO:
            ret = verify_register(ICM42688_HXY_REG_FIFO_CFG1,
                                  ICM42688_HXY_FIFO_MODE_CONFIG);
            if (ret != ESP_OK) {
                return fail_initialization(ret);
            }
            *identity = icm42688_hxy_init_identity;
            icm42688_hxy_last_sample_us = 0;
            ESP_LOGI(TAG,
                     "online address=0x%02x who_am_i=0x%02x odr=%" PRIu32
                     "Hz fifo_wtm=100Hz accel=+/-%.0fg gyro=+/-%.0fdps",
                     identity->address, identity->who_am_i,
                     ICM42688_HXY_SAMPLE_RATE_HZ,
                     (double) ICM42688_HXY_ACCEL_RANGE_G,
                     (double) ICM42688_HXY_GYRO_RANGE_DPS);
            clear_init_state();
            return ESP_OK;
        case ICM42688_HXY_INIT_IDLE:
        default:
            return ESP_ERR_INVALID_STATE;
    }
}

static int16_t decode_be_int16(uint8_t high_byte, uint8_t low_byte) {
    uint16_t combined = ((uint16_t) high_byte << 8U) | low_byte;

    return (int16_t) combined;
}

/* Required pointers refer to a complete 12-byte six-axis frame and output. */
static void decode_fifo_sample(const uint8_t *frame, int64_t timestamp_us,
                               uint8_t data_status, icm42688_hxy_sample_t *sample) {
    memset(sample, 0, sizeof(*sample));
    for (size_t axis = 0U; axis < 3U; axis++) {
        size_t gyro_offset = axis * 2U;
        size_t accel_offset = 6U + axis * 2U;

        sample->raw_accel[axis] =
            decode_be_int16(frame[accel_offset], frame[accel_offset + 1U]);
        sample->raw_gyro[axis] =
            decode_be_int16(frame[gyro_offset], frame[gyro_offset + 1U]);
        sample->accel_mps2[axis] =
            ((float) sample->raw_accel[axis] / ICM42688_HXY_ACCEL_LSB_PER_G) *
            ICM42688_HXY_STANDARD_GRAVITY_MPS2;
        sample->gyro_radps[axis] = (float) sample->raw_gyro[axis] *
            ICM42688_HXY_GYRO_DPS_PER_LSB * ICM42688_HXY_DEGREES_TO_RADIANS;
    }
    sample->timestamp_us = timestamp_us;
    sample->data_status = data_status;
    sample->valid = true;
}

esp_err_t icm42688_hxy_read_fifo(icm42688_hxy_batch_t *batch) {
    uint8_t frame[ICM42688_HXY_FIFO_BUFFER_BYTES] = {0};
    int64_t newest_timestamp_us = 0;
    int64_t first_timestamp_us = 0;
    esp_err_t ret = ESP_OK;
    esp_err_t reset_ret = ESP_OK;

    if (batch == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(batch, 0, sizeof(*batch));
    if (icm42688_hxy_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (icm42688_hxy_fifo_needs_restart) {
        ret = restart_fifo();
        if (ret != ESP_OK) {
            return ret;
        }
        return ESP_ERR_NOT_FINISHED;
    }
    if (esp_timer_get_time() < icm42688_hxy_fifo_read_after_us) {
        return ESP_ERR_NOT_FINISHED;
    }
    batch->data_status = icm42688_hxy_last_data_status;
    ret = read_registers(ICM42688_HXY_REG_FIFO_DATA, frame,
                         sizeof(frame));
    /* WTM guarantees one Sensor_Time and four complete six-axis frames. Read
     * exactly that burst, then rearm without FIFO count transactions. */
    reset_ret = restart_fifo();
    if (reset_ret != ESP_OK) {
        ret = reset_ret;
    }
    newest_timestamp_us = esp_timer_get_time();
    first_timestamp_us = newest_timestamp_us;
    first_timestamp_us -=
        ((int64_t) ICM42688_HXY_FIFO_WATERMARK_SAMPLES - 1) *
        ICM42688_HXY_SAMPLE_PERIOD_US;
    if (ret == ESP_OK && first_timestamp_us <= icm42688_hxy_last_sample_us) {
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    if (ret != ESP_OK) {
        batch->discarded_samples = ICM42688_HXY_FIFO_WATERMARK_SAMPLES;
        return ret;
    }
    batch->sensor_time = ((uint32_t) frame[1] << 16U) |
                         ((uint32_t) frame[2] << 8U) | frame[3];
    for (size_t index = 0U;
         index < ICM42688_HXY_FIFO_WATERMARK_SAMPLES; index++) {
        decode_fifo_sample(&frame[ICM42688_HXY_FIFO_TIME_BYTES +
                                  index * ICM42688_HXY_FIFO_SAMPLE_BYTES],
                           first_timestamp_us + (int64_t) index *
                               ICM42688_HXY_SAMPLE_PERIOD_US,
                           batch->data_status, &batch->samples[index]);
    }
    batch->sample_count = ICM42688_HXY_FIFO_WATERMARK_SAMPLES;
    icm42688_hxy_last_sample_us = newest_timestamp_us;
    return ESP_OK;
}

esp_err_t icm42688_hxy_deinit(void) {
    esp_err_t first_error = ESP_OK;
    esp_err_t ret = disable_fifo_gpio();

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
    icm42688_hxy_last_sample_us = 0;
    icm42688_hxy_fifo_needs_restart = false;
    icm42688_hxy_last_data_status = 0U;
    clear_init_state();
    return first_error;
}
