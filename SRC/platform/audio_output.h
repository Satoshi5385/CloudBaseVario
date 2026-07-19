#pragma once

#include "esp_err.h"

/**
 * @brief Initialize LEDC with zero duty, pause its timer, and keep the amplifier shut down.
 * @return ESP_OK on success, otherwise the first LEDC driver error.
 */
esp_err_t audio_output_init(void);

/**
 * @brief Immediately force zero duty, pause the LEDC timer, and shut down the PAM8904E.
 */
void audio_output_shutdown(void);
