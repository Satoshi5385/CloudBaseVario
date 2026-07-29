#include "domain/vario_audio.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static uint32_t interpolate_u32(float x, float x0, float x1,
                                uint32_t y0, uint32_t y1) {
    float fraction = 0.0f;
    float value = 0.0f;

    if (x <= x0 || x1 <= x0) {
        return y0;
    }
    if (x >= x1) {
        return y1;
    }
    fraction = (x - x0) / (x1 - x0);
    value = (float) y0 + fraction * ((float) y1 - (float) y0);
    return (uint32_t) lroundf(value);
}

static uint32_t lift_phase_time_ms(const app_config_t *config,
                                   float climb_rate_mps) {
    if (climb_rate_mps <= 0.2f) {
        return config->lift_time_ms_at_0p2;
    }
    if (climb_rate_mps <= 1.0f) {
        return interpolate_u32(climb_rate_mps, 0.2f, 1.0f,
                               config->lift_time_ms_at_0p2,
                               config->lift_time_ms_at_1p0);
    }
    if (climb_rate_mps <= 2.5f) {
        return interpolate_u32(climb_rate_mps, 1.0f, 2.5f,
                               config->lift_time_ms_at_1p0,
                               config->lift_time_ms_at_2p5);
    }
    return interpolate_u32(climb_rate_mps, 2.5f, 5.0f,
                           config->lift_time_ms_at_2p5,
                           config->lift_time_ms_at_5p0);
}

static uint32_t lift_frequency_hz(const app_config_t *config,
                                  float climb_rate_mps) {
    float frequency =
        (float) config->lift_freq_base_hz +
        config->lift_freq_rate_hz_per_mps * fmaxf(climb_rate_mps, 0.0f);

    frequency = fminf(frequency, (float) config->lift_freq_max_hz);
    return (uint32_t) lroundf(frequency);
}

static uint32_t sink_frequency_hz(const app_config_t *config,
                                  float climb_rate_mps) {
    float stronger_sink_mps = fmaxf((-climb_rate_mps) - 1.0f, 0.0f);
    float frequency =
        (float) config->sink_freq_start_hz -
        config->sink_freq_rate_hz_per_mps * stronger_sink_mps;

    frequency = fmaxf(frequency, (float) config->sink_freq_min_hz);
    return (uint32_t) lroundf(frequency);
}

void vario_audio_reset(vario_audio_state_t *state) {
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
        state->mode = VARIO_AUDIO_SILENT;
    }
}

static vario_audio_mode_t requested_mode(vario_audio_mode_t current,
                                         const app_config_t *config,
                                         float climb_rate_mps) {
    switch (current) {
    case VARIO_AUDIO_LIFT:
        if (climb_rate_mps < config->lift_end_mps) {
            return VARIO_AUDIO_SILENT;
        }
        return VARIO_AUDIO_LIFT;
    case VARIO_AUDIO_SINK:
        if (!config->sink_enabled || climb_rate_mps > config->sink_end_mps) {
            return VARIO_AUDIO_SILENT;
        }
        return VARIO_AUDIO_SINK;
    case VARIO_AUDIO_PREDICTIVE:
        if (!config->predictive_buzzer_enabled ||
            climb_rate_mps < config->predictive_min_mps ||
            climb_rate_mps > config->predictive_max_mps) {
            return VARIO_AUDIO_SILENT;
        }
        return VARIO_AUDIO_PREDICTIVE;
    case VARIO_AUDIO_SILENT:
    default:
        if (climb_rate_mps > config->lift_start_mps) {
            return VARIO_AUDIO_LIFT;
        }
        if (config->sink_enabled &&
            climb_rate_mps < config->sink_start_mps) {
            return VARIO_AUDIO_SINK;
        }
        if (config->predictive_buzzer_enabled &&
            climb_rate_mps >= config->predictive_min_mps &&
            climb_rate_mps <= config->predictive_max_mps) {
            return VARIO_AUDIO_PREDICTIVE;
        }
        return VARIO_AUDIO_SILENT;
    }
}

void vario_audio_step(vario_audio_state_t *state,
                      const app_config_t *config,
                      const vario_result_t *result,
                      int64_t now_us,
                      vario_audio_command_t *command) {
    vario_audio_mode_t requested = VARIO_AUDIO_SILENT;
    int64_t sample_age_us = 0;
    bool force_silent = true;

    if (state == NULL || config == NULL || result == NULL || command == NULL) {
        return;
    }
    memset(command, 0, sizeof(*command));
    command->mode = VARIO_AUDIO_SILENT;

    sample_age_us = now_us - result->timestamp_us;
    force_silent =
        !config->audio_enabled || !result->climb_rate_valid ||
        !isfinite(result->climb_rate_mps) || sample_age_us < 0 ||
        sample_age_us > (int64_t) config->audio_stale_ms * INT64_C(1000);
    if (force_silent) {
        vario_audio_reset(state);
        return;
    }

    requested = requested_mode(state->mode, config, result->climb_rate_mps);
    if (state->mode == VARIO_AUDIO_LIFT &&
        requested != VARIO_AUDIO_LIFT && state->phase_on) {
        int64_t elapsed_us = now_us - state->phase_started_us;
        int64_t phase_us =
            (int64_t) lift_phase_time_ms(config, result->climb_rate_mps) *
            INT64_C(1000);

        if (elapsed_us >= 0 && elapsed_us < phase_us) {
            requested = VARIO_AUDIO_LIFT;
        }
    }
    if (requested != state->mode) {
        int64_t held_us = now_us - state->mode_started_us;

        if (state->mode_started_us == 0 ||
            held_us >= (int64_t) config->audio_state_hold_ms * INT64_C(1000)) {
            state->mode = requested;
            state->mode_started_us = now_us;
            state->phase_started_us = now_us;
            state->phase_on = requested != VARIO_AUDIO_SILENT;
        }
    }

    command->mode = state->mode;
    command->duty_percent = config->audio_duty_percent;
    command->amplifier_mode = config->audio_amp_mode;

    switch (state->mode) {
    case VARIO_AUDIO_LIFT: {
        uint32_t phase_ms = lift_phase_time_ms(config, result->climb_rate_mps);
        int64_t elapsed_us = now_us - state->phase_started_us;
        int64_t phase_us = (int64_t) phase_ms * INT64_C(1000);

        if (elapsed_us >= phase_us) {
            int64_t phases_elapsed = elapsed_us / phase_us;

            if ((phases_elapsed & INT64_C(1)) != 0) {
                state->phase_on = !state->phase_on;
            }
            state->phase_started_us += phases_elapsed * phase_us;
        }
        command->frequency_hz =
            lift_frequency_hz(config, result->climb_rate_mps);
        command->sounding = state->phase_on;
        break;
    }
    case VARIO_AUDIO_SINK:
        command->frequency_hz =
            sink_frequency_hz(config, result->climb_rate_mps);
        command->sounding = true;
        break;
    case VARIO_AUDIO_PREDICTIVE: {
        uint32_t phase_ms =
            state->phase_on ? config->predictive_on_ms
                            : config->predictive_off_ms;
        int64_t phase_us = (int64_t) phase_ms * INT64_C(1000);

        if (now_us - state->phase_started_us >= phase_us) {
            state->phase_on = !state->phase_on;
            state->phase_started_us = now_us;
        }
        command->frequency_hz = config->predictive_freq_hz;
        command->sounding = state->phase_on;
        break;
    }
    case VARIO_AUDIO_SILENT:
    default:
        command->sounding = false;
        break;
    }
}
