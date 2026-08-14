#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "domain/watchdog_recovery_policy.h"

static watchdog_recovery_state_t initial_state(void) {
    watchdog_recovery_state_t state;
    watchdog_recovery_state_initialize(&state);
    return state;
}

static void test_only_task_and_interrupt_wdt_are_eligible(void) {
    assert(watchdog_recovery_reset_is_eligible(WATCHDOG_RESET_TASK_WDT));
    assert(watchdog_recovery_reset_is_eligible(
        WATCHDOG_RESET_INTERRUPT_WDT));
    assert(!watchdog_recovery_reset_is_eligible(WATCHDOG_RESET_OTHER_WDT));
    assert(!watchdog_recovery_reset_is_eligible(WATCHDOG_RESET_PANIC));
    assert(!watchdog_recovery_reset_is_eligible(WATCHDOG_RESET_SOFTWARE));
}

static void test_first_wdt_recovers_and_second_requires_sw1(void) {
    watchdog_recovery_state_t state = initial_state();
    watchdog_recovery_decision_t first = watchdog_recovery_decide(
        WATCHDOG_RESET_TASK_WDT, false, true, &state,
        WATCHDOG_STAGE_ACTIVE, WATCHDOG_ACTOR_SENSOR);
    watchdog_recovery_decision_t second;

    assert(first.action == WATCHDOG_BOOT_AUTO_RECOVER);
    assert(first.next_state.watchdog_reset_count == 1U);
    assert(first.next_state.last_watchdog_reset == WATCHDOG_RESET_TASK_WDT);
    assert(first.next_state.last_watchdog_stage == WATCHDOG_STAGE_ACTIVE);
    assert(first.next_state.suspected_actor == WATCHDOG_ACTOR_SENSOR);

    second = watchdog_recovery_decide(
        WATCHDOG_RESET_INTERRUPT_WDT, false, true, &first.next_state,
        WATCHDOG_STAGE_INITIALIZING, WATCHDOG_ACTOR_UNKNOWN);
    assert(second.action == WATCHDOG_BOOT_REQUIRE_SW1);
    assert(second.next_state.watchdog_reset_count == 2U);
}

static void test_ota_pending_verify_has_priority(void) {
    watchdog_recovery_state_t state = initial_state();
    watchdog_recovery_decision_t decision = watchdog_recovery_decide(
        WATCHDOG_RESET_TASK_WDT, true, true, &state,
        WATCHDOG_STAGE_INITIALIZING, WATCHDOG_ACTOR_STARTUP);

    assert(decision.action == WATCHDOG_BOOT_OTA_CONFIRMATION);
    assert(decision.next_state.watchdog_reset_count == 1U);
}

static void test_noneligible_resets_require_sw1(void) {
    const watchdog_reset_kind_t reset_kinds[] = {
        WATCHDOG_RESET_POWER_ON,
        WATCHDOG_RESET_SOFTWARE,
        WATCHDOG_RESET_PANIC,
        WATCHDOG_RESET_OTHER_WDT,
        WATCHDOG_RESET_BROWNOUT,
        WATCHDOG_RESET_OTHER,
    };
    watchdog_recovery_state_t state = initial_state();

    for (size_t index = 0U;
         index < sizeof(reset_kinds) / sizeof(reset_kinds[0]); index++) {
        watchdog_recovery_decision_t decision = watchdog_recovery_decide(
            reset_kinds[index], false, true, &state,
            WATCHDOG_STAGE_ACTIVE, WATCHDOG_ACTOR_UNKNOWN);
        assert(decision.action == WATCHDOG_BOOT_REQUIRE_SW1);
    }
}

static void test_corrupt_state_fails_safe(void) {
    watchdog_recovery_state_t corrupt = initial_state();
    watchdog_recovery_decision_t decision;

    corrupt.checksum ^= UINT32_C(1);
    assert(!watchdog_recovery_state_valid(&corrupt));
    decision = watchdog_recovery_decide(
        WATCHDOG_RESET_TASK_WDT, false, false, &corrupt,
        WATCHDOG_STAGE_ACTIVE, WATCHDOG_ACTOR_AUDIO);
    assert(decision.action == WATCHDOG_BOOT_REQUIRE_SW1);
    assert(decision.next_state.watchdog_reset_count == 2U);
}

static void test_double_slot_selects_latest_valid_state(void) {
    watchdog_recovery_state_t first = initial_state();
    watchdog_recovery_decision_t next = watchdog_recovery_decide(
        WATCHDOG_RESET_TASK_WDT, false, true, &first,
        WATCHDOG_STAGE_ACTIVE, WATCHDOG_ACTOR_SYSTEM);
    watchdog_recovery_state_t selected;

    assert(watchdog_recovery_select_state(&first, &next.next_state,
                                          &selected));
    assert(selected.sequence == next.next_state.sequence);
    next.next_state.checksum ^= UINT32_C(1);
    assert(watchdog_recovery_select_state(&first, &next.next_state,
                                          &selected));
    assert(selected.sequence == first.sequence);
}

static void test_manual_or_stable_operation_clears_attempts(void) {
    watchdog_recovery_state_t state = initial_state();
    watchdog_recovery_decision_t first = watchdog_recovery_decide(
        WATCHDOG_RESET_TASK_WDT, false, true, &state,
        WATCHDOG_STAGE_ACTIVE, WATCHDOG_ACTOR_SENSOR);

    assert(!watchdog_recovery_stable_elapsed(
        UINT32_C(1000), UINT32_C(300999)));
    assert(watchdog_recovery_stable_elapsed(
        UINT32_C(1000), UINT32_C(301000)));
    watchdog_recovery_clear_attempts(&first.next_state);
    assert(watchdog_recovery_state_valid(&first.next_state));
    assert(first.next_state.watchdog_reset_count == 0U);
    assert(watchdog_recovery_decide(
               WATCHDOG_RESET_TASK_WDT, false, true, &first.next_state,
               WATCHDOG_STAGE_ACTIVE, WATCHDOG_ACTOR_AUDIO)
               .action == WATCHDOG_BOOT_AUTO_RECOVER);
}

int main(void) {
    test_only_task_and_interrupt_wdt_are_eligible();
    test_first_wdt_recovers_and_second_requires_sw1();
    test_ota_pending_verify_has_priority();
    test_noneligible_resets_require_sw1();
    test_corrupt_state_fails_safe();
    test_double_slot_selects_latest_valid_state();
    test_manual_or_stable_operation_clears_attempts();
    puts("watchdog recovery policy tests passed");
    return 0;
}
