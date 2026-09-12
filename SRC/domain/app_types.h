#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "domain/flight_state.h"

#define GPS_NMEA_SENTENCE_CAPACITY 96U

typedef struct {
    uint32_t sequence;
    int64_t timestamp_us;
    int32_t raw_temperature;
    uint32_t raw_pressure;
    int32_t temperature_c_x100;
    int32_t pressure_pa_x100;
    float altitude_m;
    float climb_rate_mps;
    float vertical_accel_mps2;
    float kalman_accel_bias_mps2;
    float kalman_baro_innovation_m;
    float kalman_accel_innovation_mps2;
    float kalman_baro_r_m2;
    float kalman_accel_r_m2_s4;
    bool pressure_valid;
    bool climb_rate_valid;
    bool estimate_valid;
    bool estimator_warming_up;
    bool bmp581_online;
    bool imu_online;
    bool imu_calibrated;
    bool imu_stale;
    bool imu_fusion_active;
    bool vertical_accel_valid;
    bool kalman_baro_innovation_valid;
    bool kalman_accel_innovation_valid;
    bool debug_input_active;
    uint32_t i2c_error_count;
    uint32_t bmp_period_overrun_count;
    uint32_t missed_imu_sample_count;
} vario_result_t;

typedef enum {
    AUDIO_VOLUME_SMALL = 0,
    AUDIO_VOLUME_MEDIUM,
    AUDIO_VOLUME_LARGE,
    AUDIO_VOLUME_MUTE,
} audio_volume_level_t;

typedef struct {
    audio_volume_level_t volume_level;
    bool sink_enabled;
    uint8_t parameter_number;
} switch_preferences_t;

typedef enum {
    AUDIO_NOTIFICATION_BUTTON = 0,
    AUDIO_NOTIFICATION_SINK_ENABLED,
    AUDIO_NOTIFICATION_SINK_DISABLED,
} audio_notification_kind_t;

typedef struct {
    audio_notification_kind_t kind;
    audio_volume_level_t volume_level;
    uint8_t repeat_count;
} audio_notification_request_t;

typedef struct {
    int64_t timestamp_us;
    float battery_voltage_v;
    bool battery_valid;
    float battery_display_voltage_v;
    bool battery_display_valid;
    int32_t battery_raw;
    int32_t battery_adc_mv;
    uint32_t battery_sample_count;
    uint32_t battery_error_count;
    uint32_t battery_saturation_count;
    bool external_power_present;
    bool sw1_pressed;
    bool sw2_pressed;
    bool sw3_pressed;
    uint32_t sw1_hold_ms;
    uint32_t sw3_hold_ms;
    bool volume_override_active;
    audio_volume_level_t volume_level;
    bool sink_override_active;
    bool sink_enabled_override;
    uint8_t parameter_number;
    uint8_t parameter_set_count;
    bool switch_preferences_dirty;
    bool power_off_requested;
    flight_state_t motion_state;
    uint32_t motion_evidence;
    uint32_t motion_state_elapsed_s;
    uint32_t stationary_elapsed_s;
    uint32_t motion_gps_high_speed_elapsed_s;
    uint32_t auto_power_off_elapsed_s;
    uint8_t motion_gps_high_speed_updates;
    float motion_altitude_range_m;
    bool motion_vario_used;
    bool motion_gps_used;
    bool motion_gps_high_speed_pending;
} system_snapshot_t;

typedef struct {
    int64_t timestamp_us;
    int64_t last_receive_us;
    uint32_t sequence;
    uint32_t received_sentence_count;
    uint32_t invalid_sentence_count;
    uint32_t paired_update_count;
    uint32_t retry_count;
    uint32_t baud_rate;
    int32_t last_error;
    uint16_t rmc_length;
    uint16_t gga_length;
    uint8_t satellites;
    bool installed;
    bool identified;
    bool communicating;
    bool fix_valid;
    bool utc_valid;
    bool position_valid;
    bool altitude_valid;
    bool satellites_valid;
    bool hdop_valid;
    bool speed_valid;
    bool course_valid;
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    double hdop;
    double speed_kmh;
    double course_deg;
    char utc[16];
    char rmc[GPS_NMEA_SENTENCE_CAPACITY];
    char gga[GPS_NMEA_SENTENCE_CAPACITY];
} gps_snapshot_t;

typedef struct {
    bool enabled;
    bool online;
    bool configured;
    bool accel_calibrated;
    bool accel_calibration_persisted;
    bool accel_calibration_save_pending;
    bool accel_calibration_skipped;
    bool calibrated;
    bool attitude_valid;
    bool fusion_active;
    bool stale;
    uint8_t address;
    uint8_t who_am_i;
    uint8_t data_status;
    int32_t last_error;
    uint32_t retry_count;
    uint32_t sample_count;
    uint32_t consecutive_error_count;
    uint32_t calibration_sample_count;
    uint32_t accel_calibration_sample_count;
    uint32_t missed_interrupt_count;
    uint32_t buffer_high_watermark;
    uint32_t buffer_overflow_count;
    float accel_norm_g;
    float accel_offset_mps2[3];
    float gyro_bias_radps[3];
    float confidence;
    float vibration_rms_g;
    float kp_effective;
    float ki_effective;
    float quaternion[4];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    int64_t motion_timestamp_us;
    float motion_acceleration_rms_g;
    float motion_gyro_rms_dps;
    int32_t accel_calibration_storage_result;
    int32_t accel_calibration_storage_error;
    bool ki_active;
    bool motion_valid;
} imu_diagnostics_t;

typedef enum {
    DIAGNOSTIC_EVENT_STARTUP = 0,
    DIAGNOSTIC_EVENT_RESOURCE_FAILURE,
    DIAGNOSTIC_EVENT_PERIPHERAL_FAILURE,
    DIAGNOSTIC_EVENT_TASK_FAILURE,
    DIAGNOSTIC_EVENT_BLE_FAILURE,
    DIAGNOSTIC_EVENT_POWER_OFF,
} diagnostic_event_type_t;

typedef struct {
    diagnostic_event_type_t type;
    int64_t timestamp_us;
    int32_t detail;
} diagnostic_event_t;
