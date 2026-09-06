#include "domain/flight_state.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

static uint32_t elapsed_seconds(int64_t since_us, int64_t now_us) {
    int64_t elapsed_us = 0;

    if (since_us <= 0 || now_us < since_us) {
        return 0U;
    }
    elapsed_us = now_us - since_us;
    if (elapsed_us / INT64_C(1000000) > (int64_t) UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t) (elapsed_us / INT64_C(1000000));
}

static bool valid_input(const flight_state_input_t *input) {
    return input != NULL && input->now_us > 0 &&
           isfinite(input->climb_rate_threshold_mps) &&
           input->climb_rate_threshold_mps > 0.0f &&
           isfinite(input->gps_speed_threshold_kmh) &&
           input->gps_speed_threshold_kmh > 0.0f &&
           input->stationary_confirm_seconds > 0U;
}

static void clear_stationary_candidate(flight_state_detector_t *detector) {
    detector->stationary_candidate_since_us = 0;
    detector->stationary_minimum_m = 0.0f;
    detector->stationary_maximum_m = 0.0f;
    detector->stationary_tracking = false;
}

static void set_state(flight_state_detector_t *detector,
                      flight_state_t state, int64_t now_us) {
    if (detector->state != state) {
        detector->state = state;
        detector->state_since_us = now_us;
    }
}

static bool update_sustained(bool active, int64_t now_us,
                             int64_t required_us, int64_t *since_us) {
    if (!active) {
        *since_us = 0;
        return false;
    }
    if (*since_us == 0) {
        *since_us = now_us;
    }
    return now_us - *since_us >= required_us;
}

static uint32_t update_altitude_evidence(flight_state_detector_t *detector,
                                         bool vario_used,
                                         float altitude_m) {
    if (!vario_used) {
        detector->altitude_tracking = false;
        return FLIGHT_EVIDENCE_NONE;
    }
    if (!detector->altitude_tracking) {
        detector->altitude_minimum_m = altitude_m;
        detector->altitude_maximum_m = altitude_m;
        detector->altitude_tracking = true;
        return FLIGHT_EVIDENCE_NONE;
    }
    if (altitude_m < detector->altitude_minimum_m) {
        detector->altitude_minimum_m = altitude_m;
    }
    if (altitude_m > detector->altitude_maximum_m) {
        detector->altitude_maximum_m = altitude_m;
    }
    if (detector->altitude_maximum_m - detector->altitude_minimum_m >
        FLIGHT_STATE_ALTITUDE_RANGE_M) {
        detector->altitude_minimum_m = altitude_m;
        detector->altitude_maximum_m = altitude_m;
        return FLIGHT_EVIDENCE_ALTITUDE_RANGE;
    }
    return FLIGHT_EVIDENCE_NONE;
}

static bool update_gps_evidence(flight_state_detector_t *detector,
                                const flight_state_input_t *input) {
    if (!input->gps_available || !isfinite(input->gps_speed_kmh)) {
        detector->gps_high_speed_updates = 0U;
        detector->gps_sequence_valid = false;
        return false;
    }
    if (!detector->gps_sequence_valid ||
        input->gps_sequence != detector->last_gps_sequence) {
        detector->last_gps_sequence = input->gps_sequence;
        detector->gps_sequence_valid = true;
        if (input->gps_speed_kmh >= input->gps_speed_threshold_kmh) {
            if (detector->gps_high_speed_updates < UINT8_MAX) {
                detector->gps_high_speed_updates++;
            }
        } else {
            detector->gps_high_speed_updates = 0U;
        }
    }
    return detector->gps_high_speed_updates >= 2U &&
           input->gps_speed_kmh >= input->gps_speed_threshold_kmh;
}

static bool update_stationary_candidate(
    flight_state_detector_t *detector, const flight_state_input_t *input,
    bool calm) {
    if (!calm) {
        clear_stationary_candidate(detector);
        return false;
    }
    if (!detector->stationary_tracking) {
        detector->stationary_candidate_since_us = input->now_us;
        detector->stationary_minimum_m = input->altitude_m;
        detector->stationary_maximum_m = input->altitude_m;
        detector->stationary_tracking = true;
        return true;
    }
    if (input->altitude_m < detector->stationary_minimum_m) {
        detector->stationary_minimum_m = input->altitude_m;
    }
    if (input->altitude_m > detector->stationary_maximum_m) {
        detector->stationary_maximum_m = input->altitude_m;
    }
    if (detector->stationary_maximum_m - detector->stationary_minimum_m >
        FLIGHT_STATE_ALTITUDE_RANGE_M) {
        clear_stationary_candidate(detector);
        return false;
    }
    return true;
}

void flight_state_reset(flight_state_detector_t *detector) {
    if (detector != NULL) {
        memset(detector, 0, sizeof(*detector));
        detector->state = FLIGHT_STATE_UNKNOWN;
    }
}

void flight_state_update(flight_state_detector_t *detector,
                         const flight_state_input_t *input,
                         flight_state_output_t *output) {
    uint32_t evidence = FLIGHT_EVIDENCE_NONE;
    int64_t stationary_required_us = 0;
    bool vario_used = false;
    bool gps_used = false;
    bool imu_used = false;
    bool vario_active = false;
    bool imu_active = false;
    bool calm = false;
    bool ambiguous = false;

    if (output != NULL) {
        memset(output, 0, sizeof(*output));
    }
    if (detector == NULL || output == NULL || !valid_input(input)) {
        if (detector != NULL) {
            flight_state_reset(detector);
        }
        return;
    }
    if (detector->last_update_us > 0 &&
        input->now_us < detector->last_update_us) {
        flight_state_reset(detector);
    }
    detector->last_update_us = input->now_us;

    vario_used = input->vario_available &&
                  input->vario_timestamp_us > 0 &&
                  input->now_us >= input->vario_timestamp_us &&
                  input->now_us - input->vario_timestamp_us <=
                      FLIGHT_STATE_VARIO_FRESH_US &&
                  isfinite(input->altitude_m) &&
                  isfinite(input->climb_rate_mps);
    gps_used = input->gps_available && isfinite(input->gps_speed_kmh) &&
               input->gps_speed_kmh >= 0.0f;
    imu_used = input->imu_available && input->imu_timestamp_us > 0 &&
               input->now_us >= input->imu_timestamp_us &&
               input->now_us - input->imu_timestamp_us <=
                   FLIGHT_STATE_IMU_FRESH_US &&
               isfinite(input->imu_acceleration_rms_g) &&
               isfinite(input->imu_gyro_rms_dps) &&
               input->imu_acceleration_rms_g >= 0.0f &&
               input->imu_gyro_rms_dps >= 0.0f;

    calm = vario_used &&
           fabsf(input->climb_rate_mps) <=
               input->climb_rate_threshold_mps * 0.5f &&
           (!gps_used ||
            input->gps_speed_kmh <= input->gps_speed_threshold_kmh * 0.5f) &&
           (!imu_used ||
            (input->imu_acceleration_rms_g <=
                 FLIGHT_STATE_IMU_ACCEL_QUIET_RMS_G &&
             input->imu_gyro_rms_dps <=
                 FLIGHT_STATE_IMU_GYRO_QUIET_RMS_DPS));

    evidence |= update_altitude_evidence(detector, calm,
                                         input->altitude_m);
    vario_active = vario_used &&
                   fabsf(input->climb_rate_mps) >=
                       input->climb_rate_threshold_mps;
    if (update_sustained(vario_active, input->now_us,
                         FLIGHT_STATE_VARIO_CONFIRM_US,
                         &detector->vario_active_since_us)) {
        evidence |= FLIGHT_EVIDENCE_CLIMB_RATE;
    }
    if (gps_used && update_gps_evidence(detector, input)) {
        evidence |= FLIGHT_EVIDENCE_GPS_SPEED;
    } else if (!gps_used) {
        (void) update_gps_evidence(detector, input);
    }
    imu_active = imu_used &&
                 (input->imu_acceleration_rms_g >=
                      FLIGHT_STATE_IMU_ACCEL_ACTIVE_RMS_G ||
                  input->imu_gyro_rms_dps >=
                      FLIGHT_STATE_IMU_GYRO_ACTIVE_RMS_DPS);
    if (update_sustained(imu_active, input->now_us,
                         FLIGHT_STATE_IMU_CONFIRM_US,
                         &detector->imu_active_since_us)) {
        evidence |= FLIGHT_EVIDENCE_IMU_ACTIVITY;
    }
    ambiguous =
        (vario_used &&
         fabsf(input->climb_rate_mps) >
             input->climb_rate_threshold_mps * 0.5f &&
         fabsf(input->climb_rate_mps) <
             input->climb_rate_threshold_mps) ||
        (gps_used &&
         input->gps_speed_kmh > input->gps_speed_threshold_kmh * 0.5f &&
         input->gps_speed_kmh < input->gps_speed_threshold_kmh) ||
        (imu_used && !imu_active &&
         (input->imu_acceleration_rms_g >
              FLIGHT_STATE_IMU_ACCEL_QUIET_RMS_G ||
          input->imu_gyro_rms_dps >
              FLIGHT_STATE_IMU_GYRO_QUIET_RMS_DPS));

    if (evidence != FLIGHT_EVIDENCE_NONE) {
        if (detector->state == FLIGHT_STATE_FLYING) {
            detector->evidence |= evidence;
        } else {
            detector->evidence = evidence;
        }
        clear_stationary_candidate(detector);
        set_state(detector, FLIGHT_STATE_FLYING, input->now_us);
    } else if (detector->state == FLIGHT_STATE_FLYING) {
        if (calm) {
            if (update_stationary_candidate(detector, input, true) &&
                input->now_us - detector->stationary_candidate_since_us >=
                    FLIGHT_STATE_LANDING_CONFIRM_US) {
                detector->evidence = FLIGHT_EVIDENCE_NONE;
                set_state(detector, FLIGHT_STATE_STATIONARY,
                          input->now_us);
            }
        } else {
            clear_stationary_candidate(detector);
            if (!vario_used || ambiguous) {
                detector->evidence = FLIGHT_EVIDENCE_NONE;
                set_state(detector, FLIGHT_STATE_UNKNOWN,
                          input->now_us);
            }
        }
    } else if (update_stationary_candidate(detector, input, calm)) {
        stationary_required_us =
            (int64_t) input->stationary_confirm_seconds * INT64_C(1000000);
        if (input->now_us - detector->stationary_candidate_since_us >=
            stationary_required_us) {
            detector->evidence = FLIGHT_EVIDENCE_NONE;
            set_state(detector, FLIGHT_STATE_STATIONARY, input->now_us);
        } else {
            detector->evidence = FLIGHT_EVIDENCE_NONE;
            set_state(detector, FLIGHT_STATE_UNKNOWN, input->now_us);
        }
    } else {
        detector->evidence = FLIGHT_EVIDENCE_NONE;
        set_state(detector, FLIGHT_STATE_UNKNOWN, input->now_us);
    }

    output->state = detector->state;
    output->evidence = detector->evidence;
    output->state_since_us = detector->state_since_us;
    output->stationary_since_us = detector->stationary_candidate_since_us;
    output->state_elapsed_seconds =
        elapsed_seconds(detector->state_since_us, input->now_us);
    output->stationary_elapsed_seconds = elapsed_seconds(
        detector->stationary_candidate_since_us, input->now_us);
    if (detector->stationary_tracking) {
        output->altitude_range_m =
            detector->stationary_maximum_m -
            detector->stationary_minimum_m;
    } else if (detector->altitude_tracking) {
        output->altitude_range_m =
            detector->altitude_maximum_m - detector->altitude_minimum_m;
    }
    output->vario_used = vario_used;
    output->gps_used = gps_used;
    output->imu_used = imu_used;
}

const char *flight_state_name(flight_state_t state) {
    switch (state) {
    case FLIGHT_STATE_STATIONARY:
        return "STATIONARY";
    case FLIGHT_STATE_FLYING:
        return "FLYING";
    case FLIGHT_STATE_UNKNOWN:
    default:
        return "UNKNOWN";
    }
}
