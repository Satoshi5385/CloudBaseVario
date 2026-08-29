#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "domain/lk8ex1.h"

#define BLE_NUS_TX_MAX_SENTENCES 2U
#define BLE_NUS_TX_TOKENS_PER_INTERVAL 4U
#define BLE_NUS_TX_SENTENCE_CAPACITY LK8EX1_SENTENCE_MAX_LENGTH

typedef enum {
    BLE_NUS_TX_SOURCE_NONE = 0,
    BLE_NUS_TX_SOURCE_LK8EX1,
    BLE_NUS_TX_SOURCE_GPS,
} ble_nus_tx_source_t;

typedef enum {
    BLE_NUS_TX_OFFER_INVALID = 0,
    BLE_NUS_TX_OFFER_UNCHANGED,
    BLE_NUS_TX_OFFER_QUEUED,
    BLE_NUS_TX_OFFER_REPLACED,
} ble_nus_tx_offer_result_t;

typedef enum {
    BLE_NUS_TX_PROGRESS_NONE = 0,
    BLE_NUS_TX_PROGRESS_FRAGMENT_ACCEPTED,
    BLE_NUS_TX_PROGRESS_LK8EX1_COMPLETE,
    BLE_NUS_TX_PROGRESS_GPS_COMPLETE,
    BLE_NUS_TX_PROGRESS_RESYNC_COMPLETE,
} ble_nus_tx_progress_t;

typedef struct {
    uint32_t sentence_count;
    uint32_t dropped_sentence_count;
    uint32_t gps_pair_count;
    uint32_t gps_dropped_pair_count;
    uint32_t fragment_attempt_count;
    uint32_t fragment_accepted_count;
    uint32_t fragment_error_count;
    uint32_t lk8ex1_coalesced_count;
    uint32_t gps_coalesced_count;
    uint32_t partial_abort_count;
    uint32_t stream_resync_count;
    int32_t last_notify_error;
} ble_nus_tx_statistics_t;

typedef struct {
    const uint8_t *data;
    size_t length;
    ble_nus_tx_source_t source;
    bool resync;
} ble_nus_tx_fragment_t;

typedef struct {
    ble_nus_tx_source_t source;
    bool partial_sentence;
} ble_nus_tx_rejection_t;

typedef struct {
    bool valid;
    ble_nus_tx_source_t source;
    uint32_t sequence;
    size_t sentence_count;
    size_t lengths[BLE_NUS_TX_MAX_SENTENCES];
    char sentences[BLE_NUS_TX_MAX_SENTENCES]
                  [BLE_NUS_TX_SENTENCE_CAPACITY];
} ble_nus_tx_transaction_t;

typedef struct {
    ble_nus_tx_transaction_t pending_lk8ex1;
    ble_nus_tx_transaction_t pending_gps;
    ble_nus_tx_transaction_t active;
    ble_nus_tx_source_t last_selected_source;
    size_t active_sentence_index;
    size_t active_offset;
    size_t active_fragment_capacity;
    size_t offered_fragment_length;
    uint32_t last_completed_gps_sequence;
    uint8_t available_tokens;
    int64_t next_token_refill_us;
    bool fragment_offered;
    bool resync_pending;
    ble_nus_tx_statistics_t statistics;
} ble_nus_tx_state_t;

/** Initialize an empty NUS TX scheduler and clear all statistics. */
void ble_nus_tx_init(ble_nus_tx_state_t *state);

/** Discard link-local work while preserving cumulative statistics. */
void ble_nus_tx_reset_link(ble_nus_tx_state_t *state);

/** Pause a still-connected stream and retain CRLF resync when needed. */
void ble_nus_tx_suspend_stream(ble_nus_tx_state_t *state);

/** Queue or replace the latest complete LK8EX1 sentence. */
ble_nus_tx_offer_result_t ble_nus_tx_offer_lk8ex1(
    ble_nus_tx_state_t *state, const char *sentence, size_t length);

/** Queue or replace the latest coherent RMC/GGA pair. */
ble_nus_tx_offer_result_t ble_nus_tx_offer_gps(
    ble_nus_tx_state_t *state, uint32_t sequence,
    const char *rmc, size_t rmc_length,
    const char *gga, size_t gga_length);

/** Reset the four-token burst budget for a new usable link. */
void ble_nus_tx_reset_budget(ble_nus_tx_state_t *state, int64_t now_us,
                             uint32_t connection_interval_us);

/** Refill four tokens per elapsed connection interval, capped at four. */
void ble_nus_tx_refill_budget(ble_nus_tx_state_t *state, int64_t now_us,
                              uint32_t connection_interval_us);

/** Return true when a notification attempt may consume one token. */
bool ble_nus_tx_has_token(const ble_nus_tx_state_t *state);

/** Consume one token for a notification attempt. */
bool ble_nus_tx_take_token(ble_nus_tx_state_t *state);

/** Suppress unused tokens after congestion until the next refill. */
void ble_nus_tx_defer_until_refill(ble_nus_tx_state_t *state);

/** Return the next token refill deadline. */
int64_t ble_nus_tx_next_refill_us(const ble_nus_tx_state_t *state);

/** Return true when active, pending, or stream-resync work exists. */
bool ble_nus_tx_has_work(const ble_nus_tx_state_t *state);

/** Select and expose the next ordered fragment without advancing it. */
bool ble_nus_tx_next_fragment(ble_nus_tx_state_t *state,
                              size_t payload_capacity,
                              ble_nus_tx_fragment_t *fragment);

/** Commit the currently exposed fragment after NimBLE accepts it. */
ble_nus_tx_progress_t ble_nus_tx_accept_fragment(
    ble_nus_tx_state_t *state);

/** Abort the active transaction after a rejected fragment. */
ble_nus_tx_rejection_t ble_nus_tx_reject_fragment(
    ble_nus_tx_state_t *state, int32_t error);

/** Copy cumulative scheduler statistics. */
void ble_nus_tx_get_statistics(const ble_nus_tx_state_t *state,
                               ble_nus_tx_statistics_t *statistics);
