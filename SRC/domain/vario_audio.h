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

#define VARIO_AUDIO_MAX_HISTORY_SAMPLES 1001U

typedef struct {
    vario_audio_mode_t mode;
    int64_t mode_started_us;
    int64_t phase_started_us;
    int64_t history_timestamps_us[VARIO_AUDIO_MAX_HISTORY_SAMPLES];
    float history_rates_mps[VARIO_AUDIO_MAX_HISTORY_SAMPLES];
    double history_sum_mps;
    float averaged_climb_rate_mps;
    uint16_t history_head;
    uint16_t history_count;
    bool phase_on;
    bool input_source_valid;
    bool last_debug_input_active;
    bool averaged_climb_rate_valid;
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
 * Invalid/stale data and disabled audio force immediate silence. The sound
 * model uses a configurable simple moving average without changing the
 * published estimator result. Hysteresis and the configured hold time apply
 * to ordinary state transitions; predictive/lift transitions are immediate.
 */
void vario_audio_step(vario_audio_state_t *state,
                      const app_config_t *config,
                      const vario_result_t *result,
                      int64_t now_us,
                      vario_audio_command_t *command);
