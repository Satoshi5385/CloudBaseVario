#pragma once

#include <stdint.h>

#include "esp_err.h"

/**
 * @brief Initialize LEDC with zero duty, pause its timer, and keep the amplifier shut down.
 * @return ESP_OK on success, otherwise the first LEDC driver error.
 */
esp_err_t audio_output_init(void);

/**
 * @brief Generate one tone and enable the selected PAM8904E gain.
 * @param frequency_hz PWM frequency in hertz.
 * @param duty_percent PWM duty in percent.
 * @param amplifier_mode PAM8904E 1x, 2x, or 3x mode.
 * @return ESP_OK on success, otherwise the first LEDC/GPIO error.
 */
esp_err_t audio_output_apply(uint32_t frequency_hz, uint32_t duty_percent,
                             uint32_t amplifier_mode);

/**
 * @brief Immediately force zero duty, pause the LEDC timer, and shut down the PAM8904E.
 */
void audio_output_shutdown(void);
