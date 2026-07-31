#pragma once

#include <stdbool.h>
#include <stdint.h>

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
    int64_t timestamp_us;
    float battery_voltage_v;
    bool battery_valid;
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
    bool volume_override_active;
    audio_volume_level_t volume_level;
    bool sink_override_active;
    bool sink_enabled_override;
    bool power_off_requested;
} system_snapshot_t;

typedef struct {
    bool enabled;
    bool online;
    bool configured;
    bool accel_calibrated;
    bool accel_calibration_persisted;
    bool accel_calibration_save_pending;
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
    int32_t accel_calibration_storage_result;
    int32_t accel_calibration_storage_error;
    bool ki_active;
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
