#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t sequence;
    int64_t timestamp_us;
    int32_t temperature_c_x100;
    int32_t pressure_pa_x100;
    float altitude_m;
    float climb_rate_mps;
    float vertical_accel_mps2;
    bool estimate_valid;
    bool bmp581_online;
    bool imu_online;
    bool imu_fusion_active;
    bool debug_input_active;
    uint32_t i2c_error_count;
    uint32_t missed_imu_sample_count;
} vario_result_t;

typedef struct {
    int64_t timestamp_us;
    float battery_voltage_v;
    bool battery_valid;
    bool external_power_present;
    bool sw1_pressed;
    bool sw2_pressed;
    bool sw3_pressed;
    bool power_off_requested;
} system_snapshot_t;

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
