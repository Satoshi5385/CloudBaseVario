#include "platform/audio_output.h"

#include "driver/ledc.h"
#include "platform/board.h"

#define AUDIO_LEDC_MODE BOARD_LEDC_MODE
#define AUDIO_LEDC_TIMER BOARD_AUDIO_LEDC_TIMER
#define AUDIO_LEDC_CHANNEL BOARD_AUDIO_LEDC_CHANNEL
#define AUDIO_LEDC_DUTY_RESOLUTION LEDC_TIMER_10_BIT
#define AUDIO_SAFE_FREQUENCY_HZ UINT32_C(1300)
#define AUDIO_MIN_FREQUENCY_HZ UINT32_C(130)
#define AUDIO_MAX_FREQUENCY_HZ UINT32_C(5000)
#define AUDIO_MIN_DUTY_PERCENT UINT32_C(1)
#define AUDIO_MAX_DUTY_PERCENT UINT32_C(99)
#define AUDIO_PERCENT_SCALE UINT32_C(100)
#define AUDIO_PERCENT_ROUNDING UINT32_C(50)
#define AUDIO_MIN_AMPLIFIER_MODE UINT32_C(1)
#define AUDIO_MAX_AMPLIFIER_MODE UINT32_C(3)
#define AUDIO_MODE1_HIGH_FROM_MODE UINT32_C(2)
#define AUDIO_SILENT_DUTY UINT32_C(0)
#define AUDIO_DUTY_MAX UINT32_C(1023)

static bool timer_initialized = false;
static bool timer_running = false;

esp_err_t audio_output_init(void) {
    ledc_timer_config_t timer_config = {
        .speed_mode = AUDIO_LEDC_MODE,
        .duty_resolution = AUDIO_LEDC_DUTY_RESOLUTION,
        .timer_num = AUDIO_LEDC_TIMER,
        .freq_hz = AUDIO_SAFE_FREQUENCY_HZ,
        .clk_cfg = LEDC_USE_XTAL_CLK,
    };
    ledc_channel_config_t channel_config = {
        .gpio_num = PIN_BUZZER_PWM,
        .speed_mode = AUDIO_LEDC_MODE,
        .channel = AUDIO_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = AUDIO_LEDC_TIMER,
        .duty = AUDIO_SILENT_DUTY,
        .hpoint = 0U,
        .flags.output_invert = 0U,
    };
    esp_err_t ret = ESP_OK;

    board_set_audio_shutdown();
    ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        return ret;
    }

    timer_initialized = true;
    timer_running = true;
    audio_output_shutdown();
    return ESP_OK;
}

esp_err_t audio_output_apply(uint32_t frequency_hz, uint32_t duty_percent,
                             uint32_t amplifier_mode) {
    uint32_t duty = 0U;
    uint32_t mode1_level = 0U;
    uint32_t mode2_level = 0U;
    esp_err_t ret = ESP_OK;

    if (!timer_initialized || frequency_hz < AUDIO_MIN_FREQUENCY_HZ ||
        frequency_hz > AUDIO_MAX_FREQUENCY_HZ ||
        duty_percent < AUDIO_MIN_DUTY_PERCENT ||
        duty_percent > AUDIO_MAX_DUTY_PERCENT ||
        amplifier_mode < AUDIO_MIN_AMPLIFIER_MODE ||
        amplifier_mode > AUDIO_MAX_AMPLIFIER_MODE) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!timer_running) {
        ret = ledc_timer_resume(AUDIO_LEDC_MODE, AUDIO_LEDC_TIMER);
        if (ret != ESP_OK) {
            return ret;
        }
        timer_running = true;
    }
    ret = ledc_set_freq(AUDIO_LEDC_MODE, AUDIO_LEDC_TIMER, frequency_hz);
    if (ret != ESP_OK) {
        audio_output_shutdown();
        return ret;
    }

    duty = (AUDIO_DUTY_MAX * duty_percent + AUDIO_PERCENT_ROUNDING) /
           AUDIO_PERCENT_SCALE;
    ret = ledc_set_duty(AUDIO_LEDC_MODE, AUDIO_LEDC_CHANNEL, duty);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(AUDIO_LEDC_MODE, AUDIO_LEDC_CHANNEL);
    }
    if (ret != ESP_OK) {
        audio_output_shutdown();
        return ret;
    }

    /* PAM8904E truth table: 1x=01, 2x=10, 3x=11 (EN1, EN2). */
    if (amplifier_mode >= AUDIO_MODE1_HIGH_FROM_MODE) {
        mode1_level = 1U;
    }
    if (amplifier_mode == AUDIO_MIN_AMPLIFIER_MODE ||
        amplifier_mode == AUDIO_MAX_AMPLIFIER_MODE) {
        mode2_level = 1U;
    }
    ret = gpio_set_level(PIN_BUZZER_MODE1, mode1_level);
    if (ret == ESP_OK) {
        ret = gpio_set_level(PIN_BUZZER_MODE2, mode2_level);
    }
    if (ret != ESP_OK) {
        audio_output_shutdown();
    }
    return ret;
}

void audio_output_shutdown(void) {
    if (timer_initialized) {
        (void) ledc_set_duty(AUDIO_LEDC_MODE, AUDIO_LEDC_CHANNEL, AUDIO_SILENT_DUTY);
        (void) ledc_update_duty(AUDIO_LEDC_MODE, AUDIO_LEDC_CHANNEL);
        if (timer_running) {
            (void) ledc_timer_pause(AUDIO_LEDC_MODE, AUDIO_LEDC_TIMER);
            timer_running = false;
        }
    }
    board_set_audio_shutdown();
}
