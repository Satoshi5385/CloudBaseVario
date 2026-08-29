#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "domain/ble_nus_tx.h"

static ble_nus_tx_fragment_t next_fragment(ble_nus_tx_state_t *state,
                                           size_t capacity) {
    ble_nus_tx_fragment_t fragment = {0};

    assert(ble_nus_tx_next_fragment(state, capacity, &fragment));
    assert(fragment.data != NULL);
    assert(fragment.length > 0U);
    return fragment;
}

static void accept_transaction(ble_nus_tx_state_t *state,
                               size_t capacity,
                               ble_nus_tx_progress_t completion) {
    ble_nus_tx_progress_t progress = BLE_NUS_TX_PROGRESS_NONE;

    while (progress != completion) {
        (void) next_fragment(state, capacity);
        progress = ble_nus_tx_accept_fragment(state);
        assert(progress != BLE_NUS_TX_PROGRESS_NONE);
    }
}

static void test_token_budget(void) {
    ble_nus_tx_state_t state = {0};

    ble_nus_tx_init(&state);
    ble_nus_tx_reset_budget(&state, INT64_C(1000), UINT32_C(50000));
    assert(ble_nus_tx_has_token(&state));
    assert(ble_nus_tx_take_token(&state));
    assert(ble_nus_tx_take_token(&state));
    assert(ble_nus_tx_take_token(&state));
    assert(ble_nus_tx_take_token(&state));
    assert(!ble_nus_tx_take_token(&state));
    assert(ble_nus_tx_next_refill_us(&state) == INT64_C(51000));

    ble_nus_tx_refill_budget(&state, INT64_C(50999), UINT32_C(50000));
    assert(!ble_nus_tx_has_token(&state));
    ble_nus_tx_refill_budget(&state, INT64_C(51000), UINT32_C(50000));
    assert(ble_nus_tx_take_token(&state));
    ble_nus_tx_defer_until_refill(&state);
    assert(!ble_nus_tx_has_token(&state));
    ble_nus_tx_refill_budget(&state, INT64_C(101000), UINT32_C(50000));
    assert(ble_nus_tx_take_token(&state));
    assert(ble_nus_tx_take_token(&state));
    assert(ble_nus_tx_take_token(&state));
    assert(ble_nus_tx_take_token(&state));
    assert(!ble_nus_tx_take_token(&state));

    ble_nus_tx_refill_budget(&state, INT64_C(501000), UINT32_C(50000));
    assert(ble_nus_tx_take_token(&state));
    assert(ble_nus_tx_take_token(&state));
    assert(ble_nus_tx_take_token(&state));
    assert(ble_nus_tx_take_token(&state));
    assert(!ble_nus_tx_take_token(&state));
}

static void test_mtu_fragmentation(void) {
    static const char sentence[] = "$LK8EX1,1234567890123456789012345*00\r\n";
    ble_nus_tx_state_t state = {0};
    char reconstructed[sizeof(sentence)] = {0};
    size_t reconstructed_length = 0U;
    ble_nus_tx_progress_t progress = BLE_NUS_TX_PROGRESS_NONE;

    ble_nus_tx_init(&state);
    assert(ble_nus_tx_offer_lk8ex1(
               &state, sentence, strlen(sentence)) ==
           BLE_NUS_TX_OFFER_QUEUED);
    while (progress != BLE_NUS_TX_PROGRESS_LK8EX1_COMPLETE) {
        ble_nus_tx_fragment_t fragment = next_fragment(&state, 20U);

        assert(fragment.length <= 20U);
        memcpy(&reconstructed[reconstructed_length], fragment.data,
               fragment.length);
        reconstructed_length += fragment.length;
        progress = ble_nus_tx_accept_fragment(&state);
    }
    assert(reconstructed_length == strlen(sentence));
    assert(memcmp(reconstructed, sentence, reconstructed_length) == 0);

    assert(ble_nus_tx_offer_lk8ex1(
               &state, sentence, strlen(sentence)) ==
           BLE_NUS_TX_OFFER_QUEUED);
    ble_nus_tx_fragment_t extended = next_fragment(&state, 100U);
    assert(extended.length == strlen(sentence));
    assert(ble_nus_tx_accept_fragment(&state) ==
           BLE_NUS_TX_PROGRESS_LK8EX1_COMPLETE);
}

static void test_round_robin_and_gps_atomicity(void) {
    static const char lk8_first[] = "$LK8EX1,1*00\r\n";
    static const char lk8_second[] = "$LK8EX1,2*00\r\n";
    static const char rmc[] = "$GNRMC,1*00\r\n";
    static const char gga[] = "$GNGGA,1*00\r\n";
    ble_nus_tx_state_t state = {0};
    ble_nus_tx_fragment_t fragment = {0};

    ble_nus_tx_init(&state);
    assert(ble_nus_tx_offer_lk8ex1(
               &state, lk8_first, strlen(lk8_first)) ==
           BLE_NUS_TX_OFFER_QUEUED);
    assert(ble_nus_tx_offer_gps(
               &state, 1U, rmc, strlen(rmc), gga, strlen(gga)) ==
           BLE_NUS_TX_OFFER_QUEUED);
    fragment = next_fragment(&state, 8U);
    assert(fragment.source == BLE_NUS_TX_SOURCE_LK8EX1);
    assert(ble_nus_tx_accept_fragment(&state) ==
           BLE_NUS_TX_PROGRESS_FRAGMENT_ACCEPTED);
    accept_transaction(&state, 8U,
                       BLE_NUS_TX_PROGRESS_LK8EX1_COMPLETE);

    assert(ble_nus_tx_offer_lk8ex1(
               &state, lk8_second, strlen(lk8_second)) ==
           BLE_NUS_TX_OFFER_QUEUED);
    fragment = next_fragment(&state, 8U);
    assert(fragment.source == BLE_NUS_TX_SOURCE_GPS);
    while (ble_nus_tx_accept_fragment(&state) !=
           BLE_NUS_TX_PROGRESS_GPS_COMPLETE) {
        fragment = next_fragment(&state, 8U);
        assert(fragment.source == BLE_NUS_TX_SOURCE_GPS);
    }
    fragment = next_fragment(&state, 8U);
    assert(fragment.source == BLE_NUS_TX_SOURCE_LK8EX1);
}

static void test_latest_only_and_gps_sequence(void) {
    static const char lk8_first[] = "$LK8EX1,1*00\r\n";
    static const char lk8_second[] = "$LK8EX1,2*00\r\n";
    static const char rmc[] = "$GPRMC,1*00\r\n";
    static const char gga[] = "$GPGGA,1*00\r\n";
    ble_nus_tx_state_t state = {0};
    ble_nus_tx_statistics_t statistics = {0};

    ble_nus_tx_init(&state);
    assert(ble_nus_tx_offer_lk8ex1(
               &state, lk8_first, strlen(lk8_first)) ==
           BLE_NUS_TX_OFFER_QUEUED);
    assert(ble_nus_tx_offer_lk8ex1(
               &state, lk8_second, strlen(lk8_second)) ==
           BLE_NUS_TX_OFFER_REPLACED);
    assert(ble_nus_tx_offer_gps(
               &state, 4U, rmc, strlen(rmc), gga, strlen(gga)) ==
           BLE_NUS_TX_OFFER_QUEUED);
    assert(ble_nus_tx_offer_gps(
               &state, 4U, rmc, strlen(rmc), gga, strlen(gga)) ==
           BLE_NUS_TX_OFFER_UNCHANGED);
    assert(ble_nus_tx_offer_gps(
               &state, 5U, rmc, strlen(rmc), gga, strlen(gga)) ==
           BLE_NUS_TX_OFFER_REPLACED);
    ble_nus_tx_get_statistics(&state, &statistics);
    assert(statistics.lk8ex1_coalesced_count == 1U);
    assert(statistics.gps_coalesced_count == 1U);

    accept_transaction(&state, 100U,
                       BLE_NUS_TX_PROGRESS_LK8EX1_COMPLETE);
    accept_transaction(&state, 100U,
                       BLE_NUS_TX_PROGRESS_GPS_COMPLETE);
    assert(ble_nus_tx_offer_gps(
               &state, 5U, rmc, strlen(rmc), gga, strlen(gga)) ==
           BLE_NUS_TX_OFFER_UNCHANGED);
}

static void test_partial_failure_resync(void) {
    static const char sentence[] = "$LK8EX1,12345678901234567890*00\r\n";
    ble_nus_tx_state_t state = {0};
    ble_nus_tx_fragment_t fragment = {0};
    ble_nus_tx_rejection_t rejection = {0};
    ble_nus_tx_statistics_t statistics = {0};

    ble_nus_tx_init(&state);
    assert(ble_nus_tx_offer_lk8ex1(
               &state, sentence, strlen(sentence)) ==
           BLE_NUS_TX_OFFER_QUEUED);
    fragment = next_fragment(&state, 10U);
    assert(fragment.length == 10U);
    assert(ble_nus_tx_accept_fragment(&state) ==
           BLE_NUS_TX_PROGRESS_FRAGMENT_ACCEPTED);
    (void) next_fragment(&state, 10U);
    rejection = ble_nus_tx_reject_fragment(&state, 6);
    assert(rejection.source == BLE_NUS_TX_SOURCE_LK8EX1);
    assert(rejection.partial_sentence);

    fragment = next_fragment(&state, 20U);
    assert(fragment.resync);
    assert(fragment.length == 2U);
    assert(memcmp(fragment.data, "\r\n", 2U) == 0);
    rejection = ble_nus_tx_reject_fragment(&state, 6);
    assert(rejection.source == BLE_NUS_TX_SOURCE_NONE);
    assert(ble_nus_tx_has_work(&state));
    fragment = next_fragment(&state, 20U);
    assert(fragment.resync);
    assert(ble_nus_tx_accept_fragment(&state) ==
           BLE_NUS_TX_PROGRESS_RESYNC_COMPLETE);
    assert(!ble_nus_tx_has_work(&state));

    ble_nus_tx_get_statistics(&state, &statistics);
    assert(statistics.dropped_sentence_count == 1U);
    assert(statistics.fragment_attempt_count == 4U);
    assert(statistics.fragment_accepted_count == 2U);
    assert(statistics.fragment_error_count == 2U);
    assert(statistics.partial_abort_count == 1U);
    assert(statistics.stream_resync_count == 1U);
    assert(statistics.last_notify_error == 6);
}

static void test_first_fragment_failure_and_link_reset(void) {
    static const char sentence[] = "$LK8EX1,1*00\r\n";
    ble_nus_tx_state_t state = {0};
    ble_nus_tx_rejection_t rejection = {0};

    ble_nus_tx_init(&state);
    assert(ble_nus_tx_offer_lk8ex1(
               &state, sentence, strlen(sentence)) ==
           BLE_NUS_TX_OFFER_QUEUED);
    (void) next_fragment(&state, 20U);
    rejection = ble_nus_tx_reject_fragment(&state, 7);
    assert(!rejection.partial_sentence);
    assert(!ble_nus_tx_has_work(&state));

    assert(ble_nus_tx_offer_lk8ex1(
               &state, sentence, strlen(sentence)) ==
           BLE_NUS_TX_OFFER_QUEUED);
    (void) next_fragment(&state, 5U);
    assert(ble_nus_tx_accept_fragment(&state) ==
           BLE_NUS_TX_PROGRESS_FRAGMENT_ACCEPTED);
    ble_nus_tx_reset_link(&state);
    assert(!ble_nus_tx_has_work(&state));
    assert(!ble_nus_tx_has_token(&state));
}

static void test_still_connected_suspend_preserves_resync(void) {
    static const char sentence[] = "$LK8EX1,1234567890*00\r\n";
    ble_nus_tx_state_t state = {0};
    ble_nus_tx_fragment_t fragment = {0};

    ble_nus_tx_init(&state);
    assert(ble_nus_tx_offer_lk8ex1(
               &state, sentence, strlen(sentence)) ==
           BLE_NUS_TX_OFFER_QUEUED);
    (void) next_fragment(&state, 8U);
    assert(ble_nus_tx_accept_fragment(&state) ==
           BLE_NUS_TX_PROGRESS_FRAGMENT_ACCEPTED);
    ble_nus_tx_suspend_stream(&state);
    fragment = next_fragment(&state, 20U);
    assert(fragment.resync);
    assert(ble_nus_tx_accept_fragment(&state) ==
           BLE_NUS_TX_PROGRESS_RESYNC_COMPLETE);
    assert(!ble_nus_tx_has_work(&state));
}

int main(void) {
    test_token_budget();
    test_mtu_fragmentation();
    test_round_robin_and_gps_atomicity();
    test_latest_only_and_gps_sequence();
    test_partial_failure_resync();
    test_first_fragment_failure_and_link_reset();
    test_still_connected_suspend_preserves_resync();
    puts("ble_nus_tx tests passed");
    return 0;
}
