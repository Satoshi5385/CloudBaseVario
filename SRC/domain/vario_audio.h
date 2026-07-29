#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "domain/app_config.h"
#include "domain/app_types.h"

typedef enum {
    VARIO_AUDIO_SILENT = 0,
    VARIO_AUDIO_LIFT,
    VARIO_AUDIO_SINK,
    VARIO_AUDIO_PREDICTIVE,
} vario_audio_mode_t;

typedef struct {
    vario_audio_mode_t mode;
    int64_t mode_started_us;
    int64_t phase_started_us;
    bool phase_on;
} vario_audio_state_t;

typedef struct {
    vario_audio_mode_t mode;
    uint32_t frequency_hz;
    uint32_t duty_percent;
    uint32_t amplifier_mode;
    bool sounding;
} vario_audio_command_t;

void vario_audio_reset(vario_audio_state_t *state);

/**
 * Evaluate one pressure-only vario audio step.
 *
 * Invalid/stale data and disabled audio force immediate silence. Hysteresis
 * and the configured hold time apply to ordinary state transitions.
 */
void vario_audio_step(vario_audio_state_t *state,
                      const app_config_t *config,
                      const vario_result_t *result,
                      int64_t now_us,
                      vario_audio_command_t *command);

