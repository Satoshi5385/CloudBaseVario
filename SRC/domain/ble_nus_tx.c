#include "domain/ble_nus_tx.h"

#include <limits.h>
#include <string.h>

static const uint8_t stream_resync_marker[] = {'\r', '\n'};

static void increment_counter(uint32_t *counter) {
    if (counter != NULL && *counter < UINT32_MAX) {
        (*counter)++;
    }
}

static void clear_transaction(ble_nus_tx_transaction_t *transaction) {
    if (transaction != NULL) {
        memset(transaction, 0, sizeof(*transaction));
    }
}

static bool valid_sentence(const char *sentence, size_t length) {
    return sentence != NULL && length > 0U &&
           length < BLE_NUS_TX_SENTENCE_CAPACITY &&
           sentence[length - 1U] == '\n';
}

static void copy_sentence(char destination[BLE_NUS_TX_SENTENCE_CAPACITY],
                          const char *source, size_t length) {
    memcpy(destination, source, length);
    destination[length] = '\0';
}

static void set_lk8ex1_transaction(ble_nus_tx_transaction_t *transaction,
                                  const char *sentence, size_t length) {
    clear_transaction(transaction);
    transaction->valid = true;
    transaction->source = BLE_NUS_TX_SOURCE_LK8EX1;
    transaction->sentence_count = 1U;
    transaction->lengths[0] = length;
    copy_sentence(transaction->sentences[0], sentence, length);
}

static void set_gps_transaction(ble_nus_tx_transaction_t *transaction,
                                uint32_t sequence,
                                const char *rmc, size_t rmc_length,
                                const char *gga, size_t gga_length) {
    clear_transaction(transaction);
    transaction->valid = true;
    transaction->source = BLE_NUS_TX_SOURCE_GPS;
    transaction->sequence = sequence;
    transaction->sentence_count = BLE_NUS_TX_MAX_SENTENCES;
    transaction->lengths[0] = rmc_length;
    transaction->lengths[1] = gga_length;
    copy_sentence(transaction->sentences[0], rmc, rmc_length);
    copy_sentence(transaction->sentences[1], gga, gga_length);
}

static void start_transaction(ble_nus_tx_state_t *state,
                              ble_nus_tx_transaction_t *pending,
                              size_t payload_capacity) {
    state->active = *pending;
    clear_transaction(pending);
    state->last_selected_source = state->active.source;
    state->active_sentence_index = 0U;
    state->active_offset = 0U;
    state->active_fragment_capacity = payload_capacity;
}

static bool start_next_transaction(ble_nus_tx_state_t *state,
                                   size_t payload_capacity) {
    bool lk8ex1_ready = state->pending_lk8ex1.valid;
    bool gps_ready = state->pending_gps.valid;

    if (!lk8ex1_ready && !gps_ready) {
        return false;
    }
    if (lk8ex1_ready && gps_ready) {
        if (state->last_selected_source == BLE_NUS_TX_SOURCE_LK8EX1) {
            start_transaction(state, &state->pending_gps,
                              payload_capacity);
        } else {
            start_transaction(state, &state->pending_lk8ex1,
                              payload_capacity);
        }
    } else if (lk8ex1_ready) {
        start_transaction(state, &state->pending_lk8ex1,
                          payload_capacity);
    } else {
        start_transaction(state, &state->pending_gps,
                          payload_capacity);
    }
    return true;
}

static void clear_active(ble_nus_tx_state_t *state) {
    clear_transaction(&state->active);
    state->active_sentence_index = 0U;
    state->active_offset = 0U;
    state->active_fragment_capacity = 0U;
    state->offered_fragment_length = 0U;
    state->fragment_offered = false;
}

void ble_nus_tx_init(ble_nus_tx_state_t *state) {
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

void ble_nus_tx_reset_link(ble_nus_tx_state_t *state) {
    if (state == NULL) {
        return;
    }
    clear_transaction(&state->pending_lk8ex1);
    clear_transaction(&state->pending_gps);
    clear_active(state);
    state->last_selected_source = BLE_NUS_TX_SOURCE_NONE;
    state->last_completed_gps_sequence = 0U;
    state->available_tokens = 0U;
    state->next_token_refill_us = 0;
    state->resync_pending = false;
}

void ble_nus_tx_suspend_stream(ble_nus_tx_state_t *state) {
    bool needs_resync = false;

    if (state == NULL) {
        return;
    }
    needs_resync = state->resync_pending ||
                   (state->active.valid && state->active_offset > 0U);
    if (!state->resync_pending && needs_resync) {
        increment_counter(&state->statistics.partial_abort_count);
    }
    clear_transaction(&state->pending_lk8ex1);
    clear_transaction(&state->pending_gps);
    clear_active(state);
    state->last_completed_gps_sequence = 0U;
    state->available_tokens = 0U;
    state->next_token_refill_us = 0;
    state->resync_pending = needs_resync;
}

ble_nus_tx_offer_result_t ble_nus_tx_offer_lk8ex1(
    ble_nus_tx_state_t *state, const char *sentence, size_t length) {
    ble_nus_tx_offer_result_t result = BLE_NUS_TX_OFFER_QUEUED;

    if (state == NULL || !valid_sentence(sentence, length)) {
        return BLE_NUS_TX_OFFER_INVALID;
    }
    if (state->pending_lk8ex1.valid) {
        result = BLE_NUS_TX_OFFER_REPLACED;
        increment_counter(&state->statistics.lk8ex1_coalesced_count);
    }
    set_lk8ex1_transaction(&state->pending_lk8ex1, sentence, length);
    return result;
}

ble_nus_tx_offer_result_t ble_nus_tx_offer_gps(
    ble_nus_tx_state_t *state, uint32_t sequence,
    const char *rmc, size_t rmc_length,
    const char *gga, size_t gga_length) {
    ble_nus_tx_offer_result_t result = BLE_NUS_TX_OFFER_QUEUED;

    if (state == NULL || sequence == 0U ||
        !valid_sentence(rmc, rmc_length) ||
        !valid_sentence(gga, gga_length)) {
        return BLE_NUS_TX_OFFER_INVALID;
    }
    if (sequence == state->last_completed_gps_sequence ||
        (state->active.valid &&
         state->active.source == BLE_NUS_TX_SOURCE_GPS &&
         state->active.sequence == sequence) ||
        (state->pending_gps.valid &&
         state->pending_gps.sequence == sequence)) {
        return BLE_NUS_TX_OFFER_UNCHANGED;
    }
    if (state->pending_gps.valid) {
        result = BLE_NUS_TX_OFFER_REPLACED;
        increment_counter(&state->statistics.gps_coalesced_count);
    }
    set_gps_transaction(&state->pending_gps, sequence, rmc, rmc_length,
                        gga, gga_length);
    return result;
}

void ble_nus_tx_reset_budget(ble_nus_tx_state_t *state, int64_t now_us,
                             uint32_t connection_interval_us) {
    if (state == NULL || connection_interval_us == 0U) {
        return;
    }
    state->available_tokens = BLE_NUS_TX_TOKENS_PER_INTERVAL;
    state->next_token_refill_us =
        now_us + (int64_t) connection_interval_us;
}

void ble_nus_tx_refill_budget(ble_nus_tx_state_t *state, int64_t now_us,
                              uint32_t connection_interval_us) {
    int64_t intervals_elapsed = 0;

    if (state == NULL || connection_interval_us == 0U) {
        return;
    }
    if (state->next_token_refill_us == 0) {
        ble_nus_tx_reset_budget(state, now_us, connection_interval_us);
        return;
    }
    if (now_us < state->next_token_refill_us) {
        return;
    }
    intervals_elapsed =
        (now_us - state->next_token_refill_us) /
        (int64_t) connection_interval_us;
    state->next_token_refill_us +=
        (intervals_elapsed + 1) * (int64_t) connection_interval_us;
    state->available_tokens = BLE_NUS_TX_TOKENS_PER_INTERVAL;
}

bool ble_nus_tx_has_token(const ble_nus_tx_state_t *state) {
    return state != NULL && state->available_tokens > 0U;
}

bool ble_nus_tx_take_token(ble_nus_tx_state_t *state) {
    if (!ble_nus_tx_has_token(state)) {
        return false;
    }
    state->available_tokens--;
    return true;
}

void ble_nus_tx_defer_until_refill(ble_nus_tx_state_t *state) {
    if (state != NULL) {
        state->available_tokens = 0U;
    }
}

int64_t ble_nus_tx_next_refill_us(const ble_nus_tx_state_t *state) {
    if (state == NULL) {
        return 0;
    }
    return state->next_token_refill_us;
}

bool ble_nus_tx_has_work(const ble_nus_tx_state_t *state) {
    return state != NULL &&
           (state->active.valid || state->pending_lk8ex1.valid ||
            state->pending_gps.valid || state->resync_pending);
}

bool ble_nus_tx_next_fragment(ble_nus_tx_state_t *state,
                              size_t payload_capacity,
                              ble_nus_tx_fragment_t *fragment) {
    size_t remaining = 0U;
    size_t length = 0U;

    if (state == NULL || fragment == NULL || payload_capacity == 0U ||
        state->fragment_offered) {
        return false;
    }
    memset(fragment, 0, sizeof(*fragment));
    if (state->resync_pending) {
        fragment->data = stream_resync_marker;
        fragment->length = sizeof(stream_resync_marker);
        fragment->source = BLE_NUS_TX_SOURCE_NONE;
        fragment->resync = true;
        state->offered_fragment_length = fragment->length;
        state->fragment_offered = true;
        return true;
    }
    if (!state->active.valid &&
        !start_next_transaction(state, payload_capacity)) {
        return false;
    }
    remaining = state->active.lengths[state->active_sentence_index] -
                state->active_offset;
    length = state->active_fragment_capacity;
    if (remaining < length) {
        length = remaining;
    }
    fragment->data = (const uint8_t *)
        &state->active.sentences[state->active_sentence_index]
                                [state->active_offset];
    fragment->length = length;
    fragment->source = state->active.source;
    fragment->resync = false;
    state->offered_fragment_length = length;
    state->fragment_offered = true;
    return true;
}

ble_nus_tx_progress_t ble_nus_tx_accept_fragment(
    ble_nus_tx_state_t *state) {
    ble_nus_tx_source_t completed_source = BLE_NUS_TX_SOURCE_NONE;

    if (state == NULL || !state->fragment_offered) {
        return BLE_NUS_TX_PROGRESS_NONE;
    }
    increment_counter(&state->statistics.fragment_attempt_count);
    increment_counter(&state->statistics.fragment_accepted_count);
    state->fragment_offered = false;
    if (state->resync_pending) {
        state->resync_pending = false;
        state->offered_fragment_length = 0U;
        increment_counter(&state->statistics.stream_resync_count);
        return BLE_NUS_TX_PROGRESS_RESYNC_COMPLETE;
    }
    state->active_offset += state->offered_fragment_length;
    state->offered_fragment_length = 0U;
    if (state->active_offset <
        state->active.lengths[state->active_sentence_index]) {
        return BLE_NUS_TX_PROGRESS_FRAGMENT_ACCEPTED;
    }
    state->active_sentence_index++;
    state->active_offset = 0U;
    if (state->active_sentence_index < state->active.sentence_count) {
        return BLE_NUS_TX_PROGRESS_FRAGMENT_ACCEPTED;
    }
    completed_source = state->active.source;
    if (completed_source == BLE_NUS_TX_SOURCE_LK8EX1) {
        increment_counter(&state->statistics.sentence_count);
    } else if (completed_source == BLE_NUS_TX_SOURCE_GPS) {
        increment_counter(&state->statistics.gps_pair_count);
        state->last_completed_gps_sequence = state->active.sequence;
    }
    state->statistics.last_notify_error = 0;
    clear_active(state);
    if (completed_source == BLE_NUS_TX_SOURCE_LK8EX1) {
        return BLE_NUS_TX_PROGRESS_LK8EX1_COMPLETE;
    }
    return BLE_NUS_TX_PROGRESS_GPS_COMPLETE;
}

ble_nus_tx_rejection_t ble_nus_tx_reject_fragment(
    ble_nus_tx_state_t *state, int32_t error) {
    ble_nus_tx_rejection_t rejection = {0};

    if (state == NULL || !state->fragment_offered) {
        return rejection;
    }
    increment_counter(&state->statistics.fragment_attempt_count);
    increment_counter(&state->statistics.fragment_error_count);
    state->statistics.last_notify_error = error;
    state->fragment_offered = false;
    state->offered_fragment_length = 0U;
    if (state->resync_pending) {
        return rejection;
    }
    rejection.source = state->active.source;
    rejection.partial_sentence = state->active_offset > 0U;
    if (rejection.source == BLE_NUS_TX_SOURCE_LK8EX1) {
        increment_counter(&state->statistics.dropped_sentence_count);
    } else if (rejection.source == BLE_NUS_TX_SOURCE_GPS) {
        increment_counter(&state->statistics.gps_dropped_pair_count);
    }
    if (rejection.partial_sentence) {
        state->resync_pending = true;
        increment_counter(&state->statistics.partial_abort_count);
    }
    clear_active(state);
    return rejection;
}

void ble_nus_tx_get_statistics(const ble_nus_tx_state_t *state,
                               ble_nus_tx_statistics_t *statistics) {
    if (state != NULL && statistics != NULL) {
        *statistics = state->statistics;
    }
}
