#include "domain/watchdog_recovery_policy.h"

#include <stddef.h>
#include <string.h>

static uint32_t checksum_mix(uint32_t checksum, uint32_t value) {
    checksum ^= value;
    checksum *= UINT32_C(16777619);
    return checksum;
}

static uint32_t state_checksum(const watchdog_recovery_state_t *state) {
    uint32_t checksum = UINT32_C(2166136261);

    checksum = checksum_mix(checksum, state->magic);
    checksum = checksum_mix(checksum, state->version);
    checksum = checksum_mix(checksum, state->sequence);
    checksum = checksum_mix(checksum, state->watchdog_reset_count);
    checksum = checksum_mix(checksum, state->last_watchdog_reset);
    checksum = checksum_mix(checksum, state->last_watchdog_stage);
    checksum = checksum_mix(checksum, state->suspected_actor);
    return checksum;
}

static bool reset_kind_valid(uint32_t value) {
    return value <= (uint32_t) WATCHDOG_RESET_OTHER;
}

static bool stage_valid(uint32_t value) {
    return value <= (uint32_t) WATCHDOG_STAGE_FATAL;
}

static bool actor_valid(uint32_t value) {
    return value < (uint32_t) WATCHDOG_ACTOR_COUNT ||
           value == (uint32_t) WATCHDOG_ACTOR_UNKNOWN;
}

static bool sequence_after(uint32_t left, uint32_t right) {
    return (int32_t) (left - right) > 0;
}

bool watchdog_recovery_reset_is_eligible(watchdog_reset_kind_t reset_kind) {
    return reset_kind == WATCHDOG_RESET_TASK_WDT ||
           reset_kind == WATCHDOG_RESET_INTERRUPT_WDT;
}

void watchdog_recovery_state_initialize(watchdog_recovery_state_t *state) {
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->magic = WATCHDOG_RECOVERY_STATE_MAGIC;
    state->version = WATCHDOG_RECOVERY_STATE_VERSION;
    state->last_watchdog_reset = (uint32_t) WATCHDOG_RESET_OTHER;
    state->last_watchdog_stage = (uint32_t) WATCHDOG_STAGE_UNKNOWN;
    state->suspected_actor = (uint32_t) WATCHDOG_ACTOR_UNKNOWN;
    state->checksum = state_checksum(state);
}

bool watchdog_recovery_state_valid(const watchdog_recovery_state_t *state) {
    return state != NULL && state->magic == WATCHDOG_RECOVERY_STATE_MAGIC &&
           state->version == WATCHDOG_RECOVERY_STATE_VERSION &&
           reset_kind_valid(state->last_watchdog_reset) &&
           stage_valid(state->last_watchdog_stage) &&
           actor_valid(state->suspected_actor) &&
           state->checksum == state_checksum(state);
}

bool watchdog_recovery_select_state(const watchdog_recovery_state_t *first,
                                    const watchdog_recovery_state_t *second,
                                    watchdog_recovery_state_t *selected) {
    bool first_valid = watchdog_recovery_state_valid(first);
    bool second_valid = watchdog_recovery_state_valid(second);

    if (selected == NULL || (!first_valid && !second_valid)) {
        return false;
    }
    if (first_valid &&
        (!second_valid || !sequence_after(second->sequence, first->sequence))) {
        *selected = *first;
    } else {
        *selected = *second;
    }
    return true;
}

watchdog_recovery_decision_t watchdog_recovery_decide(
    watchdog_reset_kind_t reset_kind, bool ota_pending_verify,
    bool prior_state_valid, const watchdog_recovery_state_t *prior_state,
    watchdog_stage_t previous_stage, watchdog_actor_t suspected_actor) {
    watchdog_recovery_decision_t decision = {
        .action = WATCHDOG_BOOT_REQUIRE_SW1,
    };
    bool eligible = watchdog_recovery_reset_is_eligible(reset_kind);

    if (prior_state_valid &&
        watchdog_recovery_state_valid(prior_state)) {
        decision.next_state = *prior_state;
    } else {
        watchdog_recovery_state_initialize(&decision.next_state);
    }

    if (reset_kind == WATCHDOG_RESET_POWER_ON ||
        reset_kind == WATCHDOG_RESET_BROWNOUT) {
        watchdog_recovery_state_initialize(&decision.next_state);
        prior_state_valid = true;
    }

    if (eligible) {
        if (!prior_state_valid ||
            !watchdog_recovery_state_valid(prior_state)) {
            /* A damaged retry record must not grant an automatic restart. */
            decision.next_state.watchdog_reset_count = 2U;
        } else if (decision.next_state.watchdog_reset_count < UINT32_MAX) {
            decision.next_state.watchdog_reset_count++;
        }
        decision.next_state.last_watchdog_reset = (uint32_t) reset_kind;
        decision.next_state.last_watchdog_stage = (uint32_t) previous_stage;
        decision.next_state.suspected_actor = (uint32_t) suspected_actor;
    }

    if (ota_pending_verify) {
        decision.action = WATCHDOG_BOOT_OTA_CONFIRMATION;
    } else if (eligible && decision.next_state.watchdog_reset_count == 1U) {
        decision.action = WATCHDOG_BOOT_AUTO_RECOVER;
    }

    decision.next_state.sequence++;
    decision.next_state.checksum = state_checksum(&decision.next_state);
    return decision;
}

void watchdog_recovery_clear_attempts(watchdog_recovery_state_t *state) {
    if (state == NULL || !watchdog_recovery_state_valid(state)) {
        return;
    }
    state->watchdog_reset_count = 0U;
    state->sequence++;
    state->checksum = state_checksum(state);
}

bool watchdog_recovery_stable_elapsed(uint32_t active_since_ms,
                                      uint32_t now_ms) {
    return (uint32_t) (now_ms - active_since_ms) >=
           WATCHDOG_RECOVERY_STABLE_PERIOD_MS;
}

const char *watchdog_reset_kind_name(watchdog_reset_kind_t reset_kind) {
    switch (reset_kind) {
        case WATCHDOG_RESET_POWER_ON:
            return "POWER_ON";
        case WATCHDOG_RESET_SOFTWARE:
            return "SOFTWARE";
        case WATCHDOG_RESET_PANIC:
            return "PANIC";
        case WATCHDOG_RESET_TASK_WDT:
            return "TASK_WDT";
        case WATCHDOG_RESET_INTERRUPT_WDT:
            return "INT_WDT";
        case WATCHDOG_RESET_OTHER_WDT:
            return "OTHER_WDT";
        case WATCHDOG_RESET_BROWNOUT:
            return "BROWNOUT";
        case WATCHDOG_RESET_OTHER:
        default:
            return "OTHER";
    }
}

const char *watchdog_boot_action_name(watchdog_boot_action_t action) {
    switch (action) {
        case WATCHDOG_BOOT_OTA_CONFIRMATION:
            return "OTA_CONFIRMATION";
        case WATCHDOG_BOOT_AUTO_RECOVER:
            return "AUTO_RECOVER";
        case WATCHDOG_BOOT_REQUIRE_SW1:
        default:
            return "REQUIRE_SW1";
    }
}

const char *watchdog_stage_name(watchdog_stage_t stage) {
    switch (stage) {
        case WATCHDOG_STAGE_BOOT:
            return "BOOT";
        case WATCHDOG_STAGE_POWER_ON_WAIT:
            return "POWER_ON_WAIT";
        case WATCHDOG_STAGE_INITIALIZING:
            return "INITIALIZING";
        case WATCHDOG_STAGE_ACTIVE:
            return "ACTIVE";
        case WATCHDOG_STAGE_SHUTTING_DOWN:
            return "SHUTTING_DOWN";
        case WATCHDOG_STAGE_SAFE_STOP:
            return "SAFE_STOP";
        case WATCHDOG_STAGE_FATAL:
            return "FATAL";
        case WATCHDOG_STAGE_UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

const char *watchdog_actor_name(watchdog_actor_t actor) {
    switch (actor) {
        case WATCHDOG_ACTOR_STARTUP:
            return "startup";
        case WATCHDOG_ACTOR_STARTUP_PREP:
            return "startup_prep";
        case WATCHDOG_ACTOR_SENSOR:
            return "sensor";
        case WATCHDOG_ACTOR_AUDIO:
            return "audio";
        case WATCHDOG_ACTOR_SYSTEM:
            return "system";
        case WATCHDOG_ACTOR_UNKNOWN:
        default:
            return "unknown";
    }
}
