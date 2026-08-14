#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    APP_FILTER_MODE_AUTO = 0,
    APP_FILTER_MODE_BARO_ONLY,
} app_filter_mode_t;

typedef enum {
    APP_BLUETOOTH_BATTERY_MODE_VOLTAGE = 0,
    APP_BLUETOOTH_BATTERY_MODE_PERCENT,
} app_bluetooth_battery_mode_t;

typedef struct {
    float sea_level_pressure_pa;
    uint32_t auto_power_off_minutes;
    app_filter_mode_t filter_mode;
    app_bluetooth_battery_mode_t bluetooth_battery_mode;
    uint32_t bluetooth_notify_rate_hz;
    uint32_t i2c_reinit_error_count;
    uint32_t imu_gyro_calibration_samples;
    float imu_mahony_kp;
    float imu_mahony_ki;
    bool audio_enabled;
    bool sink_enabled;
    bool predictive_buzzer_enabled;
    float audio_climb_rate_average_s;
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
    uint32_t predictive_interval_ms;
    uint32_t predictive_duration_ms;
    float predictive_min_mps;
} app_config_t;

#define APP_CONFIG_PROFILE_MIN_NUMBER UINT8_C(1)
#define APP_CONFIG_PROFILE_MAX_NUMBER UINT8_C(5)
#define APP_CONFIG_PROFILE_MAX_COUNT 5U

typedef struct {
    uint8_t parameter_number;
    app_config_t config;
} app_config_profile_t;

typedef struct {
    app_config_t shared_config;
    size_t count;
    app_config_profile_t profiles[APP_CONFIG_PROFILE_MAX_COUNT];
} app_config_profiles_t;

typedef enum {
    APP_PARAMETER_SCOPE_SHARED = 0,
    APP_PARAMETER_SCOPE_PROFILE,
} app_parameter_scope_t;

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
    int32_t enumeration;
} app_parameter_value_t;

typedef struct {
    const char *name;
    app_parameter_type_t type;
    app_parameter_scope_t scope;
} app_parameter_info_t;

/** Fill public parameters plus SW1/SW2-owned runtime audio defaults. */
void app_config_set_defaults(app_config_t *config);

/** Fill one effective configuration with defaults for a profile number. */
void app_config_set_profile_defaults(app_config_t *config,
                                     uint8_t parameter_number);

/** Fill a profile collection with built-in parameter sets 1, 2, and 3. */
void app_config_profiles_set_defaults(app_config_profiles_t *profiles);

/** Validate profile count, unique numbers, and every complete configuration. */
bool app_config_profiles_validate(const app_config_profiles_t *profiles);

/** Sort an already valid profile collection by parameter number. */
void app_config_profiles_sort(app_config_profiles_t *profiles);

/** Find a profile by its stable parameter number. */
bool app_config_profiles_find(const app_config_profiles_t *profiles,
                              uint8_t parameter_number, size_t *index_out);

/** Compose one complete runtime configuration from shared and profile values. */
bool app_config_profiles_get_config(const app_config_profiles_t *profiles,
                                    size_t profile_index,
                                    app_config_t *config);

/** Store shared values once and profile values in the selected profile. */
bool app_config_profiles_set_config(app_config_profiles_t *profiles,
                                    size_t profile_index,
                                    const app_config_t *config);

/** Return the next profile index in ascending-number cyclic order. */
size_t app_config_profiles_next_index(const app_config_profiles_t *profiles,
                                      size_t current_index);

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
bool app_config_reset(app_config_t *config, uint8_t parameter_number,
                      const char *name);

/** Format one value for console output. */
bool app_config_format_value(const app_config_t *config, size_t index,
                             char *buffer, size_t buffer_size);

/** Convert the filter enum to its stable JSON/console name. */
const char *app_config_filter_mode_name(app_filter_mode_t mode);

/** Convert an LK8EX1 battery mode to its stable JSON/console name. */
const char *app_config_bluetooth_battery_mode_name(
    app_bluetooth_battery_mode_t mode);

/** Parse a stable enum value for the named public parameter. */
bool app_config_parse_enum_value(const char *parameter_name,
                                 const char *text, int32_t *value);

/** Format a stable enum value for the named public parameter. */
const char *app_config_enum_value_name(const char *parameter_name,
                                       int32_t value);
