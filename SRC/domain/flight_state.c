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
           isfinite(input->gps_speed_threshold_kmh) &&
           input->gps_speed_threshold_kmh > 0.0f &&
           input->stationary_confirm_seconds > 0U;
}

static void clear_stationary_candidate(flight_state_detector_t *detector) {
    detector->stationary_candidate_since_us = 0;
    detector->altitude_minimum_m = 0.0f;
    detector->altitude_maximum_m = 0.0f;
    detector->stationary_tracking = false;
}

static void start_stationary_candidate(
    flight_state_detector_t *detector, const flight_state_input_t *input) {
    detector->stationary_candidate_since_us = input->now_us;
    detector->altitude_minimum_m = input->altitude_m;
    detector->altitude_maximum_m = input->altitude_m;
    detector->stationary_tracking = true;
}

static void set_state(flight_state_detector_t *detector,
                      flight_state_t state, int64_t now_us) {
    if (detector->state != state) {
        detector->state = state;
        detector->state_since_us = now_us;
    }
}

static void clear_gps_evidence(flight_state_detector_t *detector) {
    detector->gps_high_speed_since_us = 0;
    detector->gps_high_speed_updates = 0U;
    detector->gps_sequence_valid = false;
}

static bool update_gps_evidence(flight_state_detector_t *detector,
                                const flight_state_input_t *input,
                                bool gps_used, bool *pending) {
    bool new_update = false;

    *pending = false;
    if (!gps_used || input->gps_speed_kmh <
                         input->gps_speed_threshold_kmh) {
        clear_gps_evidence(detector);
        return false;
    }

    new_update = !detector->gps_sequence_valid ||
                 input->gps_sequence != detector->last_gps_sequence;
    if (new_update) {
        detector->last_gps_sequence = input->gps_sequence;
        detector->gps_sequence_valid = true;
        if (detector->gps_high_speed_updates < UINT8_MAX) {
            detector->gps_high_speed_updates++;
        }
    }
    if (detector->gps_high_speed_since_us == 0) {
        detector->gps_high_speed_since_us = input->now_us;
    }

    if (detector->gps_high_speed_updates >= 2U ||
        input->now_us - detector->gps_high_speed_since_us >=
            FLIGHT_STATE_GPS_CONFIRM_US) {
        return true;
    }
    *pending = true;
    return false;
}

static bool update_altitude_range(flight_state_detector_t *detector,
                                  const flight_state_input_t *input) {
    if (!detector->stationary_tracking) {
        start_stationary_candidate(detector, input);
        return false;
    }
    if (input->altitude_m < detector->altitude_minimum_m) {
        detector->altitude_minimum_m = input->altitude_m;
    }
    if (input->altitude_m > detector->altitude_maximum_m) {
        detector->altitude_maximum_m = input->altitude_m;
    }
    return detector->altitude_maximum_m -
               detector->altitude_minimum_m >
           FLIGHT_STATE_ALTITUDE_RANGE_M;
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
    bool gps_pending = false;

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
                 isfinite(input->altitude_m);
    gps_used = input->gps_available && isfinite(input->gps_speed_kmh) &&
               input->gps_speed_kmh >= 0.0f;

    if (update_gps_evidence(detector, input, gps_used, &gps_pending)) {
        evidence |= FLIGHT_EVIDENCE_GPS_SPEED;
    }
    if (vario_used && update_altitude_range(detector, input)) {
        evidence |= FLIGHT_EVIDENCE_ALTITUDE_RANGE;
    }

    if (evidence != FLIGHT_EVIDENCE_NONE) {
        if (detector->state == FLIGHT_STATE_FLYING) {
            detector->evidence |= evidence;
        } else {
            detector->evidence = evidence;
        }
        if (vario_used) {
            start_stationary_candidate(detector, input);
        } else {
            clear_stationary_candidate(detector);
        }
        detector->flight_condition_active = true;
        set_state(detector, FLIGHT_STATE_FLYING, input->now_us);
    } else if (!vario_used || gps_pending) {
        detector->evidence = FLIGHT_EVIDENCE_NONE;
        clear_stationary_candidate(detector);
        detector->flight_condition_active = false;
        set_state(detector, FLIGHT_STATE_UNKNOWN, input->now_us);
    } else {
        if (detector->flight_condition_active) {
            start_stationary_candidate(detector, input);
        } else if (!detector->stationary_tracking) {
            start_stationary_candidate(detector, input);
        }
        detector->flight_condition_active = false;
        stationary_required_us =
            (int64_t) input->stationary_confirm_seconds * INT64_C(1000000);
        if (input->now_us - detector->stationary_candidate_since_us >=
            stationary_required_us) {
            detector->evidence = FLIGHT_EVIDENCE_NONE;
            set_state(detector, FLIGHT_STATE_STATIONARY, input->now_us);
        } else if (detector->state != FLIGHT_STATE_FLYING) {
            detector->evidence = FLIGHT_EVIDENCE_NONE;
            set_state(detector, FLIGHT_STATE_UNKNOWN, input->now_us);
        }
    }

    output->state = detector->state;
    output->evidence = detector->evidence;
    output->state_since_us = detector->state_since_us;
    if (detector->state == FLIGHT_STATE_STATIONARY) {
        output->stationary_since_us = detector->state_since_us;
    }
    output->state_elapsed_seconds =
        elapsed_seconds(detector->state_since_us, input->now_us);
    output->stationary_candidate_elapsed_seconds = elapsed_seconds(
        detector->stationary_candidate_since_us, input->now_us);
    if (detector->stationary_tracking) {
        output->altitude_range_m = detector->altitude_maximum_m -
                                   detector->altitude_minimum_m;
    }
    output->gps_high_speed_elapsed_seconds = elapsed_seconds(
        detector->gps_high_speed_since_us, input->now_us);
    output->gps_high_speed_updates = detector->gps_high_speed_updates;
    output->vario_used = vario_used;
    output->gps_used = gps_used;
    output->gps_high_speed_pending = gps_pending;
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
