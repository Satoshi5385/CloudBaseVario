#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    APP_FILTER_MODE_AUTO = 0,
    APP_FILTER_MODE_BARO_ONLY,
} app_filter_mode_t;

typedef struct {
    float sea_level_pressure_pa;
    app_filter_mode_t filter_mode;
    uint32_t i2c_reinit_error_count;
    uint32_t imu_gyro_calibration_samples;
    float imu_accel_correction_min_g;
    float imu_accel_correction_max_g;
    float imu_mahony_kp;
    float imu_mahony_ki;
    uint32_t imu_accel_x_source;
    uint32_t imu_accel_y_source;
    uint32_t imu_accel_z_source;
    float imu_accel_x_sign;
    float imu_accel_y_sign;
    float imu_accel_z_sign;
    uint32_t imu_gyro_x_source;
    uint32_t imu_gyro_y_source;
    uint32_t imu_gyro_z_source;
    float imu_gyro_x_sign;
    float imu_gyro_y_sign;
    float imu_gyro_z_sign;
    bool audio_enabled;
    bool sink_enabled;
    bool predictive_buzzer_enabled;
    float lift_start_mps;
    float lift_end_mps;
    float sink_start_mps;
    float sink_end_mps;
    uint32_t audio_state_hold_ms;
    uint32_t audio_stale_ms;
    uint32_t lift_freq_base_hz;
    float lift_freq_rate_hz_per_mps;
    uint32_t lift_freq_max_hz;
    uint32_t lift_time_ms_at_0p2;
    uint32_t lift_time_ms_at_1p0;
    uint32_t lift_time_ms_at_2p5;
    uint32_t lift_time_ms_at_5p0;
    uint32_t sink_freq_start_hz;
    float sink_freq_rate_hz_per_mps;
    uint32_t sink_freq_min_hz;
    uint32_t audio_duty_percent;
    uint32_t audio_amp_mode;
    uint32_t predictive_freq_hz;
    uint32_t predictive_on_ms;
    uint32_t predictive_off_ms;
    float predictive_min_mps;
    float predictive_max_mps;
} app_config_t;

typedef enum {
    APP_PARAMETER_BOOL = 0,
    APP_PARAMETER_UINT32,
    APP_PARAMETER_FLOAT,
    APP_PARAMETER_ENUM,
} app_parameter_type_t;

typedef union {
    bool boolean;
    uint32_t uint32;
    float real;
    app_filter_mode_t filter_mode;
} app_parameter_value_t;

typedef struct {
    const char *name;
    app_parameter_type_t type;
} app_parameter_info_t;

/** Fill a complete configuration from the single built-in parameter table. */
void app_config_set_defaults(app_config_t *config);

/** Validate all individual ranges and cross-parameter relationships. */
bool app_config_validate(const app_config_t *config);

/** Return the number of public runtime parameters. */
size_t app_config_parameter_count(void);

/** Obtain public metadata for a parameter table entry. */
bool app_config_parameter_info(size_t index, app_parameter_info_t *info);

/** Read one typed value by table index. */
bool app_config_get_value(const app_config_t *config, size_t index,
                          app_parameter_value_t *value);

/**
 * Assign one typed value without publishing a partial configuration.
 *
 * The caller must validate the complete candidate with app_config_validate()
 * before replacing the runtime configuration.
 */
bool app_config_assign_value(app_config_t *config, const char *name,
                             app_parameter_type_t type,
                             app_parameter_value_t value);

/** Parse and apply one console value, including complete relationship checks. */
bool app_config_set_text(app_config_t *config, const char *name,
                         const char *text);

/** Reset one parameter, or every parameter when name is "ALL". */
bool app_config_reset(app_config_t *config, const char *name);

/** Format one value for console output. */
bool app_config_format_value(const app_config_t *config, size_t index,
                             char *buffer, size_t buffer_size);

/** Convert the filter enum to its stable JSON/console name. */
const char *app_config_filter_mode_name(app_filter_mode_t mode);
