#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_types.h"
#include "driver/ledc.h"
#include "domain/imu_fusion.h"
#include "esp_err.h"

#if !CONFIG_CBV_BOARD_AOHAZUKU_REV0
#error "A supported CloudBaseVario board revision must be selected"
#endif

/* Aohazuku Rev.0 GPIO assignment. GPIO19/20 and GPIO35/36/37 are not touched. */
#define PIN_BOOTMODE GPIO_NUM_0
#define PIN_BAT_ADC GPIO_NUM_1
#define PIN_SW_2 GPIO_NUM_2
#define PIN_I2C_SDA GPIO_NUM_4
#define PIN_I2C_SCL GPIO_NUM_5
#define PIN_LCD_SCLK GPIO_NUM_6
#define PIN_LCD_SI GPIO_NUM_7
#define PIN_LCD_SCS GPIO_NUM_8
#define PIN_SW_3 GPIO_NUM_9
#define PIN_SD_CS GPIO_NUM_10
#define PIN_SD_MOSI GPIO_NUM_11
#define PIN_SD_CLK GPIO_NUM_12
#define PIN_SD_MISO GPIO_NUM_13
#define PIN_INT_ICM GPIO_NUM_14
#define PIN_BUZZER_MODE1 GPIO_NUM_15
#define PIN_LED_1 GPIO_NUM_16
#define PIN_GPS_UART_TX GPIO_NUM_17
#define PIN_GPS_UART_RX GPIO_NUM_18
#define PIN_USB_DN GPIO_NUM_19
#define PIN_USB_DP GPIO_NUM_20
#define PIN_INT_BMP GPIO_NUM_21
#define PIN_LCD_DISP GPIO_NUM_38
#define PIN_GPS_PPS GPIO_NUM_39
#define PIN_BUZZER_PWM GPIO_NUM_40
#define PIN_SD_DET GPIO_NUM_41
#define PIN_PWR_EXT GPIO_NUM_42
#define PIN_LED_2 GPIO_NUM_43
#define PIN_BUZZER_MODE2 GPIO_NUM_44
#define PIN_PWR_HOLD GPIO_NUM_47
#define PIN_SW_1 GPIO_NUM_48

#define BOARD_I2C_PORT I2C_NUM_0
#define BOARD_BMP581_I2C_SPEED_HZ UINT32_C(1000000)
#define BOARD_ICM42688_HXY_I2C_SPEED_HZ UINT32_C(400000)
#define BOARD_LEDC_MODE LEDC_LOW_SPEED_MODE
#define BOARD_AUDIO_LEDC_TIMER LEDC_TIMER_0
#define BOARD_AUDIO_LEDC_CHANNEL LEDC_CHANNEL_0
#define BOARD_GREEN_LED_LEDC_TIMER LEDC_TIMER_1
#define BOARD_GREEN_LED_LEDC_CHANNEL LEDC_CHANNEL_1

#define BAT_ADC_R_HIGH_OHM UINT32_C(1000000)
#define BAT_ADC_R_LOW_OHM UINT32_C(330000)
#define BAT_ADC_SCALE (133.0f / 33.0f)
#define BAT_ADC_GAIN_CORRECTION 1.0f
#define BAT_ADC_OFFSET_V 0.0f

/**
 * @brief Assert the power-hold output before any other application setup.
 * @return ESP_OK on success, otherwise the first GPIO driver error.
 */
esp_err_t board_init_power_hold(void);

/**
 * @brief Configure safe startup levels, digital inputs, and green LED PWM.
 * @return ESP_OK on success, otherwise the first GPIO/LEDC driver error.
 */
esp_err_t board_init_safe_gpio(void);

/**
 * @brief Validate compile-time board constants required by the initial build.
 * @return true when the selected board definition is internally consistent.
 */
bool board_config_is_valid(void);

/** Return the fixed sensor-to-board IMU axis map for this board revision. */
const imu_axis_map_t *board_imu_axis_map(void);

/**
 * @brief Force both LEDs and the PAM8904E controls to their safe off levels.
 */
void board_set_safe_indicators(void);

/**
 * @brief Force only the buzzer PWM and PAM8904E controls to their shutdown levels.
 */
void board_set_audio_shutdown(void);

/**
 * @brief Set the active-low status LEDs without changing the audio outputs.
 * @param[in] green_enabled true to illuminate the green LED.
 * @param[in] yellow_enabled true to illuminate the yellow LED.
 */
void board_set_status_leds(bool green_enabled, bool yellow_enabled);

/**
 * @brief Set green LED brightness and the yellow LED state.
 * @param[in] green_brightness_percent green LED brightness from 0 to 100 percent.
 * @param[in] yellow_enabled true to illuminate the yellow LED.
 */
void board_set_status_leds_brightness(uint32_t green_brightness_percent,
                                      bool yellow_enabled);

/**
 * @brief Enable or release the external power hold circuit.
 * @param[in] enabled true to keep the board powered.
 * @return ESP_OK on success, otherwise a GPIO driver error.
 */
esp_err_t board_set_power_hold(bool enabled);

/**
 * @brief Read the active-high SW1 signal from the power-latch circuit.
 * @return true while SW1 is pressed.
 */
bool board_is_sw1_pressed(void);
