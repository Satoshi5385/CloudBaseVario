#pragma once

#include <stdbool.h>
#include <stdint.h>

#define FLIGHT_STATE_ALTITUDE_RANGE_M 5.0f
#define FLIGHT_STATE_GPS_CONFIRM_US INT64_C(1000000)
#define FLIGHT_STATE_VARIO_FRESH_US INT64_C(500000)

typedef enum {
    FLIGHT_STATE_UNKNOWN = 0,
    FLIGHT_STATE_STATIONARY,
    FLIGHT_STATE_FLYING,
} flight_state_t;

typedef enum {
    FLIGHT_EVIDENCE_NONE = 0,
    FLIGHT_EVIDENCE_ALTITUDE_RANGE = UINT32_C(1) << 0,
    FLIGHT_EVIDENCE_GPS_SPEED = UINT32_C(1) << 1,
} flight_evidence_t;

typedef struct {
    int64_t now_us;
    float gps_speed_threshold_kmh;
    uint32_t stationary_confirm_seconds;
    bool vario_available;
    int64_t vario_timestamp_us;
    float altitude_m;
    bool gps_available;
    uint32_t gps_sequence;
    float gps_speed_kmh;
} flight_state_input_t;

typedef struct {
    flight_state_t state;
    uint32_t evidence;
    int64_t state_since_us;
    int64_t stationary_since_us;
    uint32_t state_elapsed_seconds;
    uint32_t stationary_candidate_elapsed_seconds;
    float altitude_range_m;
    uint32_t gps_high_speed_elapsed_seconds;
    uint8_t gps_high_speed_updates;
    bool vario_used;
    bool gps_used;
    bool gps_high_speed_pending;
} flight_state_output_t;

typedef struct {
    flight_state_t state;
    uint32_t evidence;
    int64_t last_update_us;
    int64_t state_since_us;
    int64_t stationary_candidate_since_us;
    int64_t gps_high_speed_since_us;
    float altitude_minimum_m;
    float altitude_maximum_m;
    uint32_t last_gps_sequence;
    uint8_t gps_high_speed_updates;
    bool stationary_tracking;
    bool gps_sequence_valid;
    bool flight_condition_active;
} flight_state_detector_t;

/** Reset all classification and debounce state to UNKNOWN. */
void flight_state_reset(flight_state_detector_t *detector);

/** Update the altitude/GPS motion classifier and return its current result. */
void flight_state_update(flight_state_detector_t *detector,
                         const flight_state_input_t *input,
                         flight_state_output_t *output);

/** Stable diagnostic name for a classifier state. */
const char *flight_state_name(flight_state_t state);
