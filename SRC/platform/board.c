#include "platform/board.h"

#include <math.h>
#include <stddef.h>

#include "driver/ledc.h"
#include "esp_log.h"

#define BOARD_OUTPUT_MASK                                                                          \
    ((UINT64_C(1) << PIN_PWR_HOLD) | (UINT64_C(1) << PIN_LED_1) | (UINT64_C(1) << PIN_LED_2) |     \
     (UINT64_C(1) << PIN_BUZZER_PWM) | (UINT64_C(1) << PIN_BUZZER_MODE1) |                         \
     (UINT64_C(1) << PIN_BUZZER_MODE2) | (UINT64_C(1) << PIN_SD_CS))

#define BOARD_INPUT_MASK                                                                           \
    ((UINT64_C(1) << PIN_SW_1) | (UINT64_C(1) << PIN_SW_2) | (UINT64_C(1) << PIN_SW_3) |           \
     (UINT64_C(1) << PIN_PWR_EXT))

#define BOARD_EXPECTED_BAT_SCALE (133.0f / 33.0f)
#define BOARD_FLOAT_TOLERANCE 0.000001f
#define GREEN_LED_LEDC_DUTY_RESOLUTION LEDC_TIMER_10_BIT
#define GREEN_LED_PWM_FREQUENCY_HZ UINT32_C(5000)
#define GREEN_LED_DUTY_MAX UINT32_C(1023)

static const char *TAG = "board";
static bool green_led_pwm_initialized = false;

_Static_assert(PIN_PWR_HOLD == GPIO_NUM_47, "Unexpected power-hold GPIO");
_Static_assert(PIN_LED_1 == GPIO_NUM_16, "Unexpected green LED GPIO");
_Static_assert(PIN_LED_2 == GPIO_NUM_43, "Unexpected yellow LED GPIO");
_Static_assert(PIN_I2C_SDA == GPIO_NUM_4, "Unexpected I2C SDA GPIO");
_Static_assert(PIN_I2C_SCL == GPIO_NUM_5, "Unexpected I2C SCL GPIO");
_Static_assert(PIN_USB_DN == GPIO_NUM_19, "Unexpected USB D- GPIO");
_Static_assert(PIN_USB_DP == GPIO_NUM_20, "Unexpected USB D+ GPIO");
_Static_assert(BOARD_AUDIO_LEDC_TIMER != BOARD_GREEN_LED_LEDC_TIMER,
               "Audio and green LED require independent LEDC timers");
_Static_assert(BOARD_AUDIO_LEDC_CHANNEL != BOARD_GREEN_LED_LEDC_CHANNEL,
               "Audio and green LED require independent LEDC channels");

static esp_err_t keep_safe_outputs_during_light_sleep(void) {
    const gpio_num_t safety_outputs[] = {
        PIN_PWR_HOLD,     PIN_LED_1,        PIN_LED_2, PIN_BUZZER_PWM,
        PIN_BUZZER_MODE1, PIN_BUZZER_MODE2, PIN_SD_CS,
    };

    for (size_t index = 0U; index < sizeof(safety_outputs) / sizeof(safety_outputs[0]); index++) {
        esp_err_t ret = gpio_sleep_sel_dis(safety_outputs[index]);

        if (ret != ESP_OK) {
            return ret;
        }
    }
    return ESP_OK;
}

static esp_err_t init_green_led_pwm(void) {
    ledc_timer_config_t timer_config = {
        .speed_mode = BOARD_LEDC_MODE,
        .duty_resolution = GREEN_LED_LEDC_DUTY_RESOLUTION,
        .timer_num = BOARD_GREEN_LED_LEDC_TIMER,
        .freq_hz = GREEN_LED_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_USE_XTAL_CLK,
    };
    ledc_channel_config_t channel_config = {
        .gpio_num = PIN_LED_1,
        .speed_mode = BOARD_LEDC_MODE,
        .channel = BOARD_GREEN_LED_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BOARD_GREEN_LED_LEDC_TIMER,
        .duty = 0U,
        .hpoint = 0U,
        .flags.output_invert = 1U,
    };
    esp_err_t ret = ledc_timer_config(&timer_config);

    if (ret != ESP_OK) {
        return ret;
    }
    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        return ret;
    }
    green_led_pwm_initialized = true;
    return ESP_OK;
}

esp_err_t board_init_power_hold(void) {
    esp_err_t ret = ESP_OK;

    /* Program the output latch before enabling the driver to avoid a low pulse. */
    ret = gpio_set_level(PIN_PWR_HOLD, 1U);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_set_direction(PIN_PWR_HOLD, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        return ret;
    }

    return gpio_set_level(PIN_PWR_HOLD, 1U);
}

esp_err_t board_init_safe_gpio(void) {
    gpio_config_t output_config = {
        .pin_bit_mask = BOARD_OUTPUT_MASK,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t input_config = {
        .pin_bit_mask = BOARD_INPUT_MASK,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = ESP_OK;

    board_set_safe_indicators();
    ret = gpio_set_level(PIN_SD_CS, 1U);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_config(&output_config);
    if (ret != ESP_OK) {
        return ret;
    }

    board_set_safe_indicators();
    ret = gpio_set_level(PIN_PWR_HOLD, 1U);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = gpio_set_level(PIN_SD_CS, 1U);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_config(&input_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = init_green_led_pwm();
    if (ret != ESP_OK) {
        return ret;
    }
    board_set_status_leds(false, false);

    return keep_safe_outputs_during_light_sleep();
}

bool board_config_is_valid(void) {
    float scale_difference = fabsf(BAT_ADC_SCALE - BOARD_EXPECTED_BAT_SCALE);

    if (BAT_ADC_R_HIGH_OHM != UINT32_C(1000000)) {
        ESP_LOGE(TAG, "invalid battery high-side resistor configuration");
        return false;
    }
    if (BAT_ADC_R_LOW_OHM != UINT32_C(330000)) {
        ESP_LOGE(TAG, "invalid battery low-side resistor configuration");
        return false;
    }
    if (scale_difference > BOARD_FLOAT_TOLERANCE) {
        ESP_LOGE(TAG, "invalid battery divider scale");
        return false;
    }

    return true;
}

void board_set_safe_indicators(void) {
    board_set_status_leds(false, false);
    board_set_audio_shutdown();
}

void board_set_audio_shutdown(void) {
    (void) gpio_set_level(PIN_BUZZER_PWM, 0U);
    (void) gpio_set_level(PIN_BUZZER_MODE1, 0U);
    (void) gpio_set_level(PIN_BUZZER_MODE2, 0U);
}

void board_set_status_leds(bool green_enabled, bool yellow_enabled) {
    board_set_status_leds_brightness(green_enabled ? 100U : 0U,
                                     yellow_enabled);
}

void board_set_status_leds_brightness(uint32_t green_brightness_percent,
                                      bool yellow_enabled) {
    uint32_t yellow_level = yellow_enabled ? 0U : 1U;

    if (green_brightness_percent > 100U) {
        green_brightness_percent = 100U;
    }
    if (green_led_pwm_initialized) {
        uint32_t green_duty =
            (GREEN_LED_DUTY_MAX * green_brightness_percent + 50U) / 100U;

        (void) ledc_set_duty(BOARD_LEDC_MODE,
                             BOARD_GREEN_LED_LEDC_CHANNEL,
                             green_duty);
        (void) ledc_update_duty(BOARD_LEDC_MODE,
                                BOARD_GREEN_LED_LEDC_CHANNEL);
    } else {
        (void) gpio_set_level(PIN_LED_1,
                              green_brightness_percent > 0U ? 0U : 1U);
    }
    (void) gpio_set_level(PIN_LED_2, yellow_level);
}

esp_err_t board_set_power_hold(bool enabled) {
    uint32_t output_level = 0U;

    if (enabled) {
        output_level = 1U;
    }
    return gpio_set_level(PIN_PWR_HOLD, output_level);
}

bool board_is_sw1_pressed(void) {
    int input_level = gpio_get_level(PIN_SW_1);

    return input_level != 0;
}
