#pragma once

#include <stdbool.h>
#include <stdint.h>

#define FLIGHT_STATE_ALTITUDE_RANGE_M 10.0f
#define FLIGHT_STATE_VARIO_CONFIRM_US INT64_C(3000000)
#define FLIGHT_STATE_IMU_CONFIRM_US INT64_C(1000000)
#define FLIGHT_STATE_LANDING_CONFIRM_US INT64_C(120000000)
#define FLIGHT_STATE_VARIO_FRESH_US INT64_C(500000)
#define FLIGHT_STATE_IMU_FRESH_US INT64_C(250000)
#define FLIGHT_STATE_IMU_ACCEL_ACTIVE_RMS_G 0.03f
#define FLIGHT_STATE_IMU_GYRO_ACTIVE_RMS_DPS 10.0f
#define FLIGHT_STATE_IMU_ACCEL_QUIET_RMS_G 0.01f
#define FLIGHT_STATE_IMU_GYRO_QUIET_RMS_DPS 3.0f

typedef enum {
    FLIGHT_STATE_UNKNOWN = 0,
    FLIGHT_STATE_STATIONARY,
    FLIGHT_STATE_FLYING,
} flight_state_t;

typedef enum {
    FLIGHT_EVIDENCE_NONE = 0,
    FLIGHT_EVIDENCE_ALTITUDE_RANGE = UINT32_C(1) << 0,
    FLIGHT_EVIDENCE_CLIMB_RATE = UINT32_C(1) << 1,
    FLIGHT_EVIDENCE_GPS_SPEED = UINT32_C(1) << 2,
    FLIGHT_EVIDENCE_IMU_ACTIVITY = UINT32_C(1) << 3,
} flight_evidence_t;

typedef struct {
    int64_t now_us;
    float climb_rate_threshold_mps;
    float gps_speed_threshold_kmh;
    uint32_t stationary_confirm_seconds;
    bool vario_available;
    int64_t vario_timestamp_us;
    float altitude_m;
    float climb_rate_mps;
    bool gps_available;
    uint32_t gps_sequence;
    float gps_speed_kmh;
    bool imu_available;
    int64_t imu_timestamp_us;
    float imu_acceleration_rms_g;
    float imu_gyro_rms_dps;
} flight_state_input_t;

typedef struct {
    flight_state_t state;
    uint32_t evidence;
    int64_t state_since_us;
    int64_t stationary_since_us;
    uint32_t state_elapsed_seconds;
    uint32_t stationary_elapsed_seconds;
    float altitude_range_m;
    bool vario_used;
    bool gps_used;
    bool imu_used;
} flight_state_output_t;

typedef struct {
    flight_state_t state;
    uint32_t evidence;
    int64_t last_update_us;
    int64_t state_since_us;
    int64_t stationary_candidate_since_us;
    int64_t vario_active_since_us;
    int64_t imu_active_since_us;
    float altitude_minimum_m;
    float altitude_maximum_m;
    float stationary_minimum_m;
    float stationary_maximum_m;
    uint32_t last_gps_sequence;
    uint8_t gps_high_speed_updates;
    bool altitude_tracking;
    bool stationary_tracking;
    bool gps_sequence_valid;
} flight_state_detector_t;

/** Reset all classification and debounce state to UNKNOWN. */
void flight_state_reset(flight_state_detector_t *detector);

/** Update the conservative motion classifier and return its current result. */
void flight_state_update(flight_state_detector_t *detector,
                         const flight_state_input_t *input,
                         flight_state_output_t *output);

/** Stable diagnostic name for a classifier state. */
const char *flight_state_name(flight_state_t state);
