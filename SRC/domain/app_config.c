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
    size_t offset;
    app_parameter_value_t default_value;
    double minimum;
    double maximum;
} app_parameter_descriptor_t;

#define PARAM_BOOL(member, default_value_)                                                      \
    {                                                                                            \
        #member, APP_PARAMETER_BOOL, offsetof(app_config_t, member),                             \
            {.boolean = (default_value_)}, 0.0, 1.0                                             \
    }
#define PARAM_UINT(member, default_value_, minimum_, maximum_)                                   \
    {                                                                                            \
        #member, APP_PARAMETER_UINT32, offsetof(app_config_t, member),                           \
            {.uint32 = UINT32_C(default_value_)}, (minimum_), (maximum_)                         \
    }
#define PARAM_FLOAT(member, default_value_, minimum_, maximum_)                                  \
    {                                                                                            \
        #member, APP_PARAMETER_FLOAT, offsetof(app_config_t, member),                            \
            {.real = (default_value_)}, (minimum_), (maximum_)                                  \
    }
#define PARAM_ENUM(member, default_value_)                                                       \
    {                                                                                            \
        #member, APP_PARAMETER_ENUM, offsetof(app_config_t, member),                             \
            {.filter_mode = (default_value_)}, APP_FILTER_MODE_AUTO, APP_FILTER_MODE_BARO_ONLY   \
    }

/*
 * This is the single source for public names, types, defaults, and scalar
 * ranges.
 */
static const app_parameter_descriptor_t parameter_table[] = {
    PARAM_FLOAT(sea_level_pressure_pa, 101325.0f, 80000.0, 110000.0),
    PARAM_ENUM(filter_mode, APP_FILTER_MODE_AUTO),
    PARAM_UINT(i2c_reinit_error_count, 10, 1.0, 100.0),
    PARAM_UINT(imu_gyro_calibration_samples, 200, 50.0, 2000.0),
    PARAM_FLOAT(imu_accel_correction_min_g, 0.75f, 0.5, 1.0),
    PARAM_FLOAT(imu_accel_correction_max_g, 1.25f, 1.0, 1.5),
    PARAM_FLOAT(imu_mahony_kp, 5.0f, 0.0, 20.0),
    PARAM_FLOAT(imu_mahony_ki, 0.0f, 0.0, 5.0),
    PARAM_UINT(imu_accel_x_source, 0, 0.0, 2.0),
    PARAM_UINT(imu_accel_y_source, 1, 0.0, 2.0),
    PARAM_UINT(imu_accel_z_source, 2, 0.0, 2.0),
    PARAM_FLOAT(imu_accel_x_sign, 1.0f, -1.0, 1.0),
    PARAM_FLOAT(imu_accel_y_sign, 1.0f, -1.0, 1.0),
    PARAM_FLOAT(imu_accel_z_sign, 1.0f, -1.0, 1.0),
    PARAM_UINT(imu_gyro_x_source, 0, 0.0, 2.0),
    PARAM_UINT(imu_gyro_y_source, 1, 0.0, 2.0),
    PARAM_UINT(imu_gyro_z_source, 2, 0.0, 2.0),
    PARAM_FLOAT(imu_gyro_x_sign, 1.0f, -1.0, 1.0),
    PARAM_FLOAT(imu_gyro_y_sign, 1.0f, -1.0, 1.0),
    PARAM_FLOAT(imu_gyro_z_sign, 1.0f, -1.0, 1.0),
    PARAM_BOOL(audio_enabled, true),
    PARAM_BOOL(sink_enabled, true),
    PARAM_BOOL(predictive_buzzer_enabled, false),
    PARAM_FLOAT(lift_start_mps, 0.10f, -1.0, 5.0),
    PARAM_FLOAT(lift_end_mps, 0.05f, -1.0, 5.0),
    PARAM_FLOAT(sink_start_mps, -1.00f, -10.0, 0.0),
    PARAM_FLOAT(sink_end_mps, -0.80f, -10.0, 0.0),
    PARAM_UINT(audio_state_hold_ms, 60, 0.0, 1000.0),
    PARAM_UINT(audio_stale_ms, 500, 100.0, 500.0),
    PARAM_UINT(lift_freq_base_hz, 600, 200.0, 5000.0),
    PARAM_FLOAT(lift_freq_rate_hz_per_mps, 100.0f, 0.0, 1000.0),
    PARAM_UINT(lift_freq_max_hz, 1800, 200.0, 5000.0),
    PARAM_UINT(lift_time_ms_at_0p2, 480, 20.0, 2000.0),
    PARAM_UINT(lift_time_ms_at_1p0, 220, 20.0, 2000.0),
    PARAM_UINT(lift_time_ms_at_2p5, 100, 20.0, 2000.0),
    PARAM_UINT(lift_time_ms_at_5p0, 70, 70.0, 2000.0),
    PARAM_UINT(sink_freq_start_hz, 400, 130.0, 2000.0),
    PARAM_FLOAT(sink_freq_rate_hz_per_mps, 70.0f, 0.0, 500.0),
    PARAM_UINT(sink_freq_min_hz, 130, 130.0, 2000.0),
    PARAM_UINT(audio_duty_percent, 50, 10.0, 90.0),
    PARAM_UINT(audio_amp_mode, 1, 1.0, 3.0),
    PARAM_UINT(predictive_freq_hz, 800, 200.0, 5000.0),
    PARAM_UINT(predictive_on_ms, 20, 10.0, 1000.0),
    PARAM_UINT(predictive_off_ms, 20, 10.0, 1000.0),
    PARAM_FLOAT(predictive_min_mps, -0.10f, -2.0, 1.0),
    PARAM_FLOAT(predictive_max_mps, 0.10f, -1.0, 2.0),
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
    for (size_t index = 0U;
         index < sizeof(parameter_table) / sizeof(parameter_table[0]); index++) {
        write_value(config, &parameter_table[index],
                    parameter_table[index].default_value);
    }
}

static bool axis_map_is_permutation(uint32_t x_source, uint32_t y_source,
                                    uint32_t z_source) {
    return x_source != y_source && x_source != z_source &&
           y_source != z_source;
}

static bool axis_sign_is_valid(float sign) {
    return sign == -1.0f || sign == 1.0f;
}

bool app_config_validate(const app_config_t *config) {
    if (config == NULL) {
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
    if (config->predictive_min_mps > config->predictive_max_mps) {
        return false;
    }
    if (config->imu_accel_correction_min_g >=
        config->imu_accel_correction_max_g) {
        return false;
    }
    if (!axis_map_is_permutation(config->imu_accel_x_source,
                                 config->imu_accel_y_source,
                                 config->imu_accel_z_source) ||
        !axis_map_is_permutation(config->imu_gyro_x_source,
                                 config->imu_gyro_y_source,
                                 config->imu_gyro_z_source)) {
        return false;
    }
    if (!axis_sign_is_valid(config->imu_accel_x_sign) ||
        !axis_sign_is_valid(config->imu_accel_y_sign) ||
        !axis_sign_is_valid(config->imu_accel_z_sign) ||
        !axis_sign_is_valid(config->imu_gyro_x_sign) ||
        !axis_sign_is_valid(config->imu_gyro_y_sign) ||
        !axis_sign_is_valid(config->imu_gyro_z_sign)) {
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

bool app_config_reset(app_config_t *config, const char *name) {
    const app_parameter_descriptor_t *descriptor = NULL;
    app_config_t candidate = {0};

    if (config == NULL || name == NULL) {
        return false;
    }
    if (strcasecmp(name, "ALL") == 0) {
        app_config_set_defaults(config);
        return true;
    }

    descriptor = find_parameter(name, NULL);
    if (descriptor == NULL) {
        return false;
    }
    candidate = *config;
    write_value(&candidate, descriptor, descriptor->default_value);
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
