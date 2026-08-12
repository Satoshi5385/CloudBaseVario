#include "domain/app_config.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

typedef struct {
    const char *name;
    app_parameter_type_t type;
    app_parameter_scope_t scope;
    size_t offset;
    app_parameter_value_t default_value;
    double minimum;
    double maximum;
} app_parameter_descriptor_t;

#define PARAM_BOOL(member, default_value_, scope_)                                              \
    {                                                                                            \
        #member, APP_PARAMETER_BOOL, (scope_), offsetof(app_config_t, member),                   \
            {.boolean = (default_value_)}, 0.0, 1.0                                             \
    }
#define PARAM_UINT(member, default_value_, minimum_, maximum_, scope_)                           \
    {                                                                                            \
        #member, APP_PARAMETER_UINT32, (scope_), offsetof(app_config_t, member),                 \
            {.uint32 = UINT32_C(default_value_)}, (minimum_), (maximum_)                         \
    }
#define PARAM_FLOAT(member, default_value_, minimum_, maximum_, scope_)                          \
    {                                                                                            \
        #member, APP_PARAMETER_FLOAT, (scope_), offsetof(app_config_t, member),                  \
            {.real = (default_value_)}, (minimum_), (maximum_)                                  \
    }
#define PARAM_ENUM(member, default_value_, scope_)                                               \
    {                                                                                            \
        #member, APP_PARAMETER_ENUM, (scope_), offsetof(app_config_t, member),                   \
            {.filter_mode = (default_value_)}, APP_FILTER_MODE_AUTO, APP_FILTER_MODE_BARO_ONLY   \
    }

/*
 * This is the single source for public names, types, defaults, and scalar
 * ranges.
 */
static const app_parameter_descriptor_t parameter_table[] = {
    PARAM_FLOAT(sea_level_pressure_pa, 101325.0f, 80000.0, 110000.0, APP_PARAMETER_SCOPE_SHARED),
    PARAM_UINT(auto_power_off_minutes, 60, 0.0, 1440.0, APP_PARAMETER_SCOPE_SHARED),
    PARAM_ENUM(filter_mode, APP_FILTER_MODE_AUTO, APP_PARAMETER_SCOPE_SHARED),
    PARAM_UINT(i2c_reinit_error_count, 10, 1.0, 100.0, APP_PARAMETER_SCOPE_SHARED),
    PARAM_UINT(imu_gyro_calibration_samples, 200, 50.0, 2000.0, APP_PARAMETER_SCOPE_SHARED),
    PARAM_FLOAT(imu_mahony_kp, 5.0f, 0.0, 20.0, APP_PARAMETER_SCOPE_SHARED),
    PARAM_FLOAT(imu_mahony_ki, 0.05f, 0.0, 5.0, APP_PARAMETER_SCOPE_SHARED),
    PARAM_BOOL(predictive_buzzer_enabled, false, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_FLOAT(audio_climb_rate_average_s, 1.0f, 0.0, 10.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_FLOAT(lift_start_mps, 0.10f, -1.0, 5.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_FLOAT(lift_end_mps, 0.08f, -1.0, 5.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_FLOAT(sink_start_mps, -1.80f, -10.0, 0.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_FLOAT(sink_end_mps, -1.70f, -10.0, 0.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(audio_state_hold_ms, 200, 0.0, 1000.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(audio_stale_ms, 500, 100.0, 500.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(lift_freq_base_hz, 1047, 200.0, 5000.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_FLOAT(lift_freq_rate_hz_per_mps, 100.0f, 0.0, 1000.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(lift_freq_max_hz, 2600, 200.0, 5000.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(lift_time_ms_at_0p2, 400, 20.0, 2000.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(lift_time_ms_at_1p0, 400, 20.0, 2000.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(lift_time_ms_at_2p5, 300, 20.0, 2000.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(lift_time_ms_at_5p0, 100, 70.0, 2000.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(sink_freq_start_hz, 523, 130.0, 2000.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_FLOAT(sink_freq_rate_hz_per_mps, 40.0f, 0.0, 500.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(sink_freq_min_hz, 240, 130.0, 2000.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(audio_duty_percent, 50, 10.0, 90.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(predictive_interval_ms, 1000, 20.0, 2000.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_UINT(predictive_duration_ms, 150, 10.0, 1000.0, APP_PARAMETER_SCOPE_PROFILE),
    PARAM_FLOAT(predictive_min_mps, 0.01f, -2.0, 1.0, APP_PARAMETER_SCOPE_PROFILE),
};

static const app_parameter_descriptor_t *find_parameter(const char *name,
                                                         size_t *index_out) {
    if (name == NULL) {
        return NULL;
    }

    for (size_t index = 0U;
         index < sizeof(parameter_table) / sizeof(parameter_table[0]); index++) {
        if (strcasecmp(name, parameter_table[index].name) == 0) {
            if (index_out != NULL) {
                *index_out = index;
            }
            return &parameter_table[index];
        }
    }
    return NULL;
}

static void write_value(app_config_t *config,
                        const app_parameter_descriptor_t *descriptor,
                        app_parameter_value_t value) {
    uint8_t *destination = (uint8_t *) config + descriptor->offset;

    switch (descriptor->type) {
    case APP_PARAMETER_BOOL:
        memcpy(destination, &value.boolean, sizeof(value.boolean));
        break;
    case APP_PARAMETER_UINT32:
        memcpy(destination, &value.uint32, sizeof(value.uint32));
        break;
    case APP_PARAMETER_FLOAT:
        memcpy(destination, &value.real, sizeof(value.real));
        break;
    case APP_PARAMETER_ENUM:
        memcpy(destination, &value.filter_mode, sizeof(value.filter_mode));
        break;
    default:
        break;
    }
}

static bool value_in_range(const app_parameter_descriptor_t *descriptor,
                           app_parameter_value_t value) {
    double number = 0.0;

    if (descriptor == NULL) {
        return false;
    }
    switch (descriptor->type) {
    case APP_PARAMETER_BOOL:
        return true;
    case APP_PARAMETER_UINT32:
        number = (double) value.uint32;
        break;
    case APP_PARAMETER_FLOAT:
        if (!isfinite(value.real)) {
            return false;
        }
        number = value.real;
        break;
    case APP_PARAMETER_ENUM:
        number = value.filter_mode;
        break;
    default:
        return false;
    }
    return number >= descriptor->minimum && number <= descriptor->maximum;
}

void app_config_set_defaults(app_config_t *config) {
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(*config));
    /* SW1/SW2 own these runtime-only values; they are not public parameters. */
    config->audio_enabled = true;
    config->sink_enabled = true;
    config->audio_amp_mode = 1U;
    for (size_t index = 0U;
         index < sizeof(parameter_table) / sizeof(parameter_table[0]); index++) {
        write_value(config, &parameter_table[index],
                    parameter_table[index].default_value);
    }
}

void app_config_set_profile_defaults(app_config_t *config,
                                     uint8_t parameter_number) {
    app_config_set_defaults(config);
    if (config == NULL) {
        return;
    }
    if (parameter_number == 2U) {
        config->lift_start_mps = 0.20f;
        config->lift_end_mps = 0.18f;
        config->sink_start_mps = -2.00f;
        config->sink_end_mps = -1.90f;
    } else if (parameter_number == 3U) {
        config->lift_start_mps = 0.30f;
        config->lift_end_mps = 0.29f;
        config->sink_start_mps = -2.20f;
        config->sink_end_mps = -2.10f;
    }
}

void app_config_profiles_set_defaults(app_config_profiles_t *profiles) {
    if (profiles == NULL) {
        return;
    }
    memset(profiles, 0, sizeof(*profiles));
    app_config_set_defaults(&profiles->shared_config);
    profiles->count = 3U;
    for (size_t index = 0U; index < profiles->count; index++) {
        profiles->profiles[index].parameter_number =
            (uint8_t) (APP_CONFIG_PROFILE_MIN_NUMBER + index);
        app_config_set_profile_defaults(&profiles->profiles[index].config,
                                        profiles->profiles[index].parameter_number);
    }
}

bool app_config_profiles_validate(const app_config_profiles_t *profiles) {
    bool seen[APP_CONFIG_PROFILE_MAX_NUMBER + 1U] = {false};
    app_config_t effective = {0};

    if (profiles == NULL || profiles->count == 0U ||
        profiles->count > APP_CONFIG_PROFILE_MAX_COUNT) {
        return false;
    }
    for (size_t index = 0U; index < profiles->count; index++) {
        uint8_t number = profiles->profiles[index].parameter_number;

        if (number < APP_CONFIG_PROFILE_MIN_NUMBER ||
            number > APP_CONFIG_PROFILE_MAX_NUMBER || seen[number] ||
            !app_config_profiles_get_config(profiles, index, &effective) ||
            !app_config_validate(&effective)) {
            return false;
        }
        seen[number] = true;
    }
    return true;
}

static void copy_parameter_scope(app_config_t *destination,
                                 const app_config_t *source,
                                 app_parameter_scope_t scope) {
    if (destination == NULL || source == NULL) {
        return;
    }
    for (size_t index = 0U; index < app_config_parameter_count(); index++) {
        const app_parameter_descriptor_t *descriptor = &parameter_table[index];
        const uint8_t *source_value =
            (const uint8_t *) source + descriptor->offset;
        uint8_t *destination_value =
            (uint8_t *) destination + descriptor->offset;
        size_t value_size = 0U;

        if (descriptor->scope != scope) {
            continue;
        }
        switch (descriptor->type) {
        case APP_PARAMETER_BOOL:
            value_size = sizeof(bool);
            break;
        case APP_PARAMETER_UINT32:
            value_size = sizeof(uint32_t);
            break;
        case APP_PARAMETER_FLOAT:
            value_size = sizeof(float);
            break;
        case APP_PARAMETER_ENUM:
            value_size = sizeof(app_filter_mode_t);
            break;
        default:
            continue;
        }
        memcpy(destination_value, source_value, value_size);
    }
}

bool app_config_profiles_get_config(const app_config_profiles_t *profiles,
                                    size_t profile_index,
                                    app_config_t *config) {
    if (profiles == NULL || config == NULL || profile_index >= profiles->count) {
        return false;
    }
    app_config_set_defaults(config);
    copy_parameter_scope(config, &profiles->shared_config,
                         APP_PARAMETER_SCOPE_SHARED);
    copy_parameter_scope(config, &profiles->profiles[profile_index].config,
                         APP_PARAMETER_SCOPE_PROFILE);
    return true;
}

bool app_config_profiles_set_config(app_config_profiles_t *profiles,
                                    size_t profile_index,
                                    const app_config_t *config) {
    if (profiles == NULL || config == NULL || profile_index >= profiles->count ||
        !app_config_validate(config)) {
        return false;
    }
    copy_parameter_scope(&profiles->shared_config, config,
                         APP_PARAMETER_SCOPE_SHARED);
    copy_parameter_scope(&profiles->profiles[profile_index].config, config,
                         APP_PARAMETER_SCOPE_PROFILE);
    return true;
}

void app_config_profiles_sort(app_config_profiles_t *profiles) {
    if (profiles == NULL) {
        return;
    }
    for (size_t index = 1U; index < profiles->count; index++) {
        app_config_profile_t value = profiles->profiles[index];
        size_t destination = index;

        while (destination > 0U &&
               profiles->profiles[destination - 1U].parameter_number >
                   value.parameter_number) {
            profiles->profiles[destination] =
                profiles->profiles[destination - 1U];
            destination--;
        }
        profiles->profiles[destination] = value;
    }
}

bool app_config_profiles_find(const app_config_profiles_t *profiles,
                              uint8_t parameter_number, size_t *index_out) {
    if (profiles == NULL) {
        return false;
    }
    for (size_t index = 0U; index < profiles->count; index++) {
        if (profiles->profiles[index].parameter_number == parameter_number) {
            if (index_out != NULL) {
                *index_out = index;
            }
            return true;
        }
    }
    return false;
}

size_t app_config_profiles_next_index(const app_config_profiles_t *profiles,
                                      size_t current_index) {
    if (profiles == NULL || profiles->count == 0U) {
        return 0U;
    }
    return (current_index + 1U) % profiles->count;
}

bool app_config_validate(const app_config_t *config) {
    if (config == NULL) {
        return false;
    }

    if (config->audio_amp_mode < 1U || config->audio_amp_mode > 3U) {
        return false;
    }

    for (size_t index = 0U;
         index < sizeof(parameter_table) / sizeof(parameter_table[0]); index++) {
        app_parameter_value_t value = {0};

        if (!app_config_get_value(config, index, &value) ||
            !value_in_range(&parameter_table[index], value)) {
            return false;
        }
    }

    if (!(config->sink_start_mps <= config->sink_end_mps &&
          config->sink_end_mps < config->lift_end_mps &&
          config->lift_end_mps <= config->lift_start_mps)) {
        return false;
    }
    if (config->lift_freq_base_hz > config->lift_freq_max_hz ||
        config->sink_freq_min_hz > config->sink_freq_start_hz) {
        return false;
    }
    if (!(config->lift_time_ms_at_0p2 >= config->lift_time_ms_at_1p0 &&
          config->lift_time_ms_at_1p0 >= config->lift_time_ms_at_2p5 &&
          config->lift_time_ms_at_2p5 >= config->lift_time_ms_at_5p0)) {
        return false;
    }
    if (config->predictive_min_mps > config->lift_start_mps ||
        config->predictive_duration_ms > config->predictive_interval_ms) {
        return false;
    }
    return true;
}

size_t app_config_parameter_count(void) {
    return sizeof(parameter_table) / sizeof(parameter_table[0]);
}

bool app_config_parameter_info(size_t index, app_parameter_info_t *info) {
    if (info == NULL || index >= app_config_parameter_count()) {
        return false;
    }
    info->name = parameter_table[index].name;
    info->type = parameter_table[index].type;
    info->scope = parameter_table[index].scope;
    return true;
}

bool app_config_get_value(const app_config_t *config, size_t index,
                          app_parameter_value_t *value) {
    const app_parameter_descriptor_t *descriptor = NULL;
    const uint8_t *source = NULL;

    if (config == NULL || value == NULL || index >= app_config_parameter_count()) {
        return false;
    }
    descriptor = &parameter_table[index];
    source = (const uint8_t *) config + descriptor->offset;
    memset(value, 0, sizeof(*value));

    switch (descriptor->type) {
    case APP_PARAMETER_BOOL:
        memcpy(&value->boolean, source, sizeof(value->boolean));
        break;
    case APP_PARAMETER_UINT32:
        memcpy(&value->uint32, source, sizeof(value->uint32));
        break;
    case APP_PARAMETER_FLOAT:
        memcpy(&value->real, source, sizeof(value->real));
        break;
    case APP_PARAMETER_ENUM:
        memcpy(&value->filter_mode, source, sizeof(value->filter_mode));
        break;
    default:
        return false;
    }
    return true;
}

bool app_config_assign_value(app_config_t *config, const char *name,
                             app_parameter_type_t type,
                             app_parameter_value_t value) {
    const app_parameter_descriptor_t *descriptor = find_parameter(name, NULL);

    if (config == NULL || descriptor == NULL || descriptor->type != type ||
        !value_in_range(descriptor, value)) {
        return false;
    }
    write_value(config, descriptor, value);
    return true;
}

static bool parse_bool(const char *text, bool *value) {
    if (strcasecmp(text, "true") == 0) {
        *value = true;
        return true;
    }
    if (strcasecmp(text, "false") == 0) {
        *value = false;
        return true;
    }
    return false;
}

static bool parse_uint32(const char *text, uint32_t *value) {
    char *end = NULL;
    unsigned long parsed = 0UL;

    if (text == NULL || text[0] == '\0' || text[0] == '-') {
        return false;
    }
    errno = 0;
    parsed = strtoul(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' || parsed > UINT32_MAX) {
        return false;
    }
    *value = (uint32_t) parsed;
    return true;
}

static bool parse_float(const char *text, float *value) {
    char *end = NULL;
    float parsed = 0.0f;

    if (text == NULL || text[0] == '\0') {
        return false;
    }
    errno = 0;
    parsed = strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed)) {
        return false;
    }
    *value = parsed;
    return true;
}

static bool parse_filter_mode(const char *text, app_filter_mode_t *mode) {
    if (strcasecmp(text, "AUTO") == 0) {
        *mode = APP_FILTER_MODE_AUTO;
        return true;
    }
    if (strcasecmp(text, "BARO_ONLY") == 0) {
        *mode = APP_FILTER_MODE_BARO_ONLY;
        return true;
    }
    return false;
}

bool app_config_set_text(app_config_t *config, const char *name,
                         const char *text) {
    const app_parameter_descriptor_t *descriptor = find_parameter(name, NULL);
    app_parameter_value_t value = {0};
    app_config_t candidate = {0};
    bool parsed = false;

    if (config == NULL || descriptor == NULL || text == NULL) {
        return false;
    }

    switch (descriptor->type) {
    case APP_PARAMETER_BOOL:
        parsed = parse_bool(text, &value.boolean);
        break;
    case APP_PARAMETER_UINT32:
        parsed = parse_uint32(text, &value.uint32);
        break;
    case APP_PARAMETER_FLOAT:
        parsed = parse_float(text, &value.real);
        break;
    case APP_PARAMETER_ENUM:
        parsed = parse_filter_mode(text, &value.filter_mode);
        break;
    default:
        break;
    }
    if (!parsed || !value_in_range(descriptor, value)) {
        return false;
    }

    candidate = *config;
    write_value(&candidate, descriptor, value);
    if (!app_config_validate(&candidate)) {
        return false;
    }
    *config = candidate;
    return true;
}

bool app_config_reset(app_config_t *config, uint8_t parameter_number,
                      const char *name) {
    const app_parameter_descriptor_t *descriptor = NULL;
    app_config_t candidate = {0};
    app_config_t defaults = {0};

    if (config == NULL || name == NULL) {
        return false;
    }
    app_config_set_profile_defaults(&defaults, parameter_number);
    if (strcasecmp(name, "ALL") == 0) {
        *config = defaults;
        return true;
    }

    descriptor = find_parameter(name, NULL);
    if (descriptor == NULL) {
        return false;
    }
    candidate = *config;
    memcpy((uint8_t *) &candidate + descriptor->offset,
           (const uint8_t *) &defaults + descriptor->offset,
           descriptor->type == APP_PARAMETER_BOOL ? sizeof(bool) :
           descriptor->type == APP_PARAMETER_UINT32 ? sizeof(uint32_t) :
           descriptor->type == APP_PARAMETER_FLOAT ? sizeof(float) :
                                                     sizeof(app_filter_mode_t));
    if (!app_config_validate(&candidate)) {
        return false;
    }
    *config = candidate;
    return true;
}

const char *app_config_filter_mode_name(app_filter_mode_t mode) {
    if (mode == APP_FILTER_MODE_AUTO) {
        return "AUTO";
    }
    if (mode == APP_FILTER_MODE_BARO_ONLY) {
        return "BARO_ONLY";
    }
    return "INVALID";
}

bool app_config_format_value(const app_config_t *config, size_t index,
                             char *buffer, size_t buffer_size) {
    app_parameter_value_t value = {0};
    app_parameter_info_t info = {0};
    int written = -1;

    if (buffer == NULL || buffer_size == 0U ||
        !app_config_parameter_info(index, &info) ||
        !app_config_get_value(config, index, &value)) {
        return false;
    }

    switch (info.type) {
    case APP_PARAMETER_BOOL:
        written = snprintf(buffer, buffer_size, "%s",
                           value.boolean ? "true" : "false");
        break;
    case APP_PARAMETER_UINT32:
        written = snprintf(buffer, buffer_size, "%lu",
                           (unsigned long) value.uint32);
        break;
    case APP_PARAMETER_FLOAT:
        written = snprintf(buffer, buffer_size, "%.6g", (double) value.real);
        break;
    case APP_PARAMETER_ENUM:
        written = snprintf(buffer, buffer_size, "%s",
                           app_config_filter_mode_name(value.filter_mode));
        break;
    default:
        break;
    }
    return written >= 0 && (size_t) written < buffer_size;
}
