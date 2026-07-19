#include "platform/audio_output.h"

#include "driver/ledc.h"
#include "platform/board.h"

#define AUDIO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define AUDIO_LEDC_TIMER LEDC_TIMER_0
#define AUDIO_LEDC_CHANNEL LEDC_CHANNEL_0
#define AUDIO_LEDC_DUTY_RESOLUTION LEDC_TIMER_10_BIT
#define AUDIO_SAFE_FREQUENCY_HZ UINT32_C(1300)
#define AUDIO_SILENT_DUTY UINT32_C(0)

static bool timer_initialized = false;
static bool timer_running = false;

esp_err_t audio_output_init(void) {
    ledc_timer_config_t timer_config = {
        .speed_mode = AUDIO_LEDC_MODE,
        .duty_resolution = AUDIO_LEDC_DUTY_RESOLUTION,
        .timer_num = AUDIO_LEDC_TIMER,
        .freq_hz = AUDIO_SAFE_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
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

    board_set_safe_indicators();
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
