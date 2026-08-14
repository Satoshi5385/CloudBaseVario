#include "platform/watchdog_service.h"

#include <stddef.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define WATCHDOG_RUNTIME_MAGIC UINT32_C(0x57445254)
#define WATCHDOG_RUNTIME_VERSION UINT32_C(1)
#define WATCHDOG_ACTOR_BIT(actor) (UINT32_C(1) << (uint32_t) (actor))

typedef struct {
    uint32_t magic;
    uint32_t magic_inverse;
    uint32_t version;
    uint32_t version_inverse;
    uint32_t stage;
    uint32_t stage_inverse;
    uint32_t registered_mask;
    uint32_t registered_mask_inverse;
    uint32_t heartbeat_ms[WATCHDOG_ACTOR_COUNT];
    uint32_t heartbeat_inverse[WATCHDOG_ACTOR_COUNT];
} watchdog_runtime_record_t;

typedef struct {
    watchdog_recovery_state_t recovery[2];
    watchdog_runtime_record_t runtime;
} watchdog_rtc_store_t;

static RTC_NOINIT_ATTR watchdog_rtc_store_t rtc_store;
static portMUX_TYPE service_lock = portMUX_INITIALIZER_UNLOCKED;
static watchdog_recovery_state_t current_recovery_state;
static watchdog_diagnostics_t current_diagnostics;
static bool service_initialized;
static bool active_started;
static uint32_t active_since_ms;

static bool pair_valid(uint32_t value, uint32_t inverse) {
    return value == ~inverse;
}

static uint32_t now_ms(void) {
    return (uint32_t) ((uint64_t) esp_timer_get_time() / UINT64_C(1000));
}

static bool actor_valid(watchdog_actor_t actor) {
    return (uint32_t) actor < (uint32_t) WATCHDOG_ACTOR_COUNT;
}

static bool runtime_valid(const watchdog_runtime_record_t *runtime) {
    return runtime != NULL &&
           pair_valid(runtime->magic, runtime->magic_inverse) &&
           runtime->magic == WATCHDOG_RUNTIME_MAGIC &&
           pair_valid(runtime->version, runtime->version_inverse) &&
           runtime->version == WATCHDOG_RUNTIME_VERSION;
}

static watchdog_stage_t previous_stage(void) {
    uint32_t stage = rtc_store.runtime.stage;

    if (!runtime_valid(&rtc_store.runtime) ||
        !pair_valid(stage, rtc_store.runtime.stage_inverse) ||
        stage > (uint32_t) WATCHDOG_STAGE_FATAL) {
        return WATCHDOG_STAGE_UNKNOWN;
    }
    return (watchdog_stage_t) stage;
}

static watchdog_actor_t previous_suspected_actor(
    watchdog_reset_kind_t reset_kind) {
    uint32_t mask = rtc_store.runtime.registered_mask;
    uint32_t latest_ms = 0U;
    uint32_t timeout_ms = CONFIG_ESP_TASK_WDT_TIMEOUT_S * UINT32_C(1000);
    watchdog_actor_t suspected = WATCHDOG_ACTOR_UNKNOWN;
    uint32_t suspected_count = 0U;

    if (reset_kind != WATCHDOG_RESET_TASK_WDT ||
        !runtime_valid(&rtc_store.runtime) ||
        !pair_valid(mask, rtc_store.runtime.registered_mask_inverse)) {
        return WATCHDOG_ACTOR_UNKNOWN;
    }
    mask &= (UINT32_C(1) << WATCHDOG_ACTOR_COUNT) - 1U;
    for (uint32_t actor = 0U; actor < WATCHDOG_ACTOR_COUNT; actor++) {
        uint32_t heartbeat = rtc_store.runtime.heartbeat_ms[actor];

        if ((mask & WATCHDOG_ACTOR_BIT(actor)) != 0U &&
            pair_valid(heartbeat,
                       rtc_store.runtime.heartbeat_inverse[actor]) &&
            (latest_ms == 0U || (int32_t) (heartbeat - latest_ms) > 0)) {
            latest_ms = heartbeat;
        }
    }
    if (latest_ms == 0U) {
        return WATCHDOG_ACTOR_UNKNOWN;
    }
    for (uint32_t actor = 0U; actor < WATCHDOG_ACTOR_COUNT; actor++) {
        uint32_t heartbeat = rtc_store.runtime.heartbeat_ms[actor];

        if ((mask & WATCHDOG_ACTOR_BIT(actor)) == 0U ||
            !pair_valid(heartbeat,
                        rtc_store.runtime.heartbeat_inverse[actor])) {
            continue;
        }
        if ((uint32_t) (latest_ms - heartbeat) >= timeout_ms) {
            suspected = (watchdog_actor_t) actor;
            suspected_count++;
        }
    }
    if (suspected_count == 1U) {
        return suspected;
    }
    return WATCHDOG_ACTOR_UNKNOWN;
}

static watchdog_reset_kind_t classify_reset_reason(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:
            return WATCHDOG_RESET_POWER_ON;
        case ESP_RST_SW:
            return WATCHDOG_RESET_SOFTWARE;
        case ESP_RST_PANIC:
            return WATCHDOG_RESET_PANIC;
        case ESP_RST_TASK_WDT:
            return WATCHDOG_RESET_TASK_WDT;
        case ESP_RST_INT_WDT:
            return WATCHDOG_RESET_INTERRUPT_WDT;
        case ESP_RST_WDT:
            return WATCHDOG_RESET_OTHER_WDT;
        case ESP_RST_BROWNOUT:
            return WATCHDOG_RESET_BROWNOUT;
        default:
            return WATCHDOG_RESET_OTHER;
    }
}

static void initialize_runtime_record(void) {
    memset(&rtc_store.runtime, 0, sizeof(rtc_store.runtime));
    rtc_store.runtime.magic = WATCHDOG_RUNTIME_MAGIC;
    rtc_store.runtime.magic_inverse = ~WATCHDOG_RUNTIME_MAGIC;
    rtc_store.runtime.version = WATCHDOG_RUNTIME_VERSION;
    rtc_store.runtime.version_inverse = ~WATCHDOG_RUNTIME_VERSION;
    rtc_store.runtime.stage = (uint32_t) WATCHDOG_STAGE_BOOT;
    rtc_store.runtime.stage_inverse = ~(uint32_t) WATCHDOG_STAGE_BOOT;
    rtc_store.runtime.registered_mask = 0U;
    rtc_store.runtime.registered_mask_inverse = UINT32_MAX;
    for (uint32_t actor = 0U; actor < WATCHDOG_ACTOR_COUNT; actor++) {
        rtc_store.runtime.heartbeat_ms[actor] = 0U;
        rtc_store.runtime.heartbeat_inverse[actor] = UINT32_MAX;
    }
}

static void store_recovery_state(const watchdog_recovery_state_t *state) {
    bool first_valid = watchdog_recovery_state_valid(&rtc_store.recovery[0]);
    bool second_valid = watchdog_recovery_state_valid(&rtc_store.recovery[1]);
    size_t target = 0U;

    if (!first_valid) {
        target = 0U;
    } else if (!second_valid) {
        target = 1U;
    } else {
        if ((int32_t) (rtc_store.recovery[0].sequence -
                       rtc_store.recovery[1].sequence) > 0) {
            target = 1U;
        }
    }
    rtc_store.recovery[target] = *state;
}

static void clear_recovery_attempts(void) {
    if (current_recovery_state.watchdog_reset_count == 0U) {
        return;
    }
    watchdog_recovery_clear_attempts(&current_recovery_state);
    store_recovery_state(&current_recovery_state);
    current_diagnostics.watchdog_reset_count = 0U;
}

esp_err_t watchdog_service_begin_boot(bool ota_pending_verify,
                                      watchdog_boot_action_t *action) {
    watchdog_recovery_state_t prior_state;
    watchdog_recovery_decision_t decision;
    watchdog_reset_kind_t reset_kind =
        classify_reset_reason(esp_reset_reason());
    watchdog_stage_t retained_stage = previous_stage();
    watchdog_actor_t suspected = previous_suspected_actor(reset_kind);
    const watchdog_recovery_state_t *prior_state_pointer = NULL;
    bool prior_valid = watchdog_recovery_select_state(
        &rtc_store.recovery[0], &rtc_store.recovery[1], &prior_state);

    if (action == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (prior_valid) {
        prior_state_pointer = &prior_state;
    }
    decision = watchdog_recovery_decide(
        reset_kind, ota_pending_verify, prior_valid, prior_state_pointer,
        retained_stage, suspected);
    current_recovery_state = decision.next_state;
    store_recovery_state(&current_recovery_state);
    initialize_runtime_record();

    memset(&current_diagnostics, 0, sizeof(current_diagnostics));
    current_diagnostics.reset_kind = reset_kind;
    current_diagnostics.boot_action = decision.action;
    current_diagnostics.previous_stage = retained_stage;
    current_diagnostics.suspected_actor = suspected;
    current_diagnostics.watchdog_reset_count =
        current_recovery_state.watchdog_reset_count;
    current_diagnostics.recovery_record_valid = prior_valid;
    active_started = false;
    active_since_ms = 0U;
    service_initialized = true;
    *action = decision.action;
    return ESP_OK;
}

esp_err_t watchdog_service_register_current(watchdog_actor_t actor) {
    esp_err_t result;
    uint32_t mask;
    uint32_t heartbeat;

    if (!service_initialized || !actor_valid(actor)) {
        return ESP_ERR_INVALID_STATE;
    }
    result = esp_task_wdt_add(NULL);
    portENTER_CRITICAL(&service_lock);
    if (result == ESP_OK) {
        mask = rtc_store.runtime.registered_mask |
               WATCHDOG_ACTOR_BIT(actor);
        rtc_store.runtime.registered_mask = mask;
        rtc_store.runtime.registered_mask_inverse = ~mask;
        heartbeat = now_ms();
        rtc_store.runtime.heartbeat_ms[actor] = heartbeat;
        rtc_store.runtime.heartbeat_inverse[actor] = ~heartbeat;
        current_diagnostics.registered_actor_mask = mask;
    } else {
        current_diagnostics.registration_failure_count++;
    }
    portEXIT_CRITICAL(&service_lock);
    return result;
}

esp_err_t watchdog_service_feed(watchdog_actor_t actor) {
    esp_err_t result;
    uint32_t heartbeat;
    uint32_t mask;

    if (!service_initialized || !actor_valid(actor)) {
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&service_lock);
    mask = current_diagnostics.registered_actor_mask;
    portEXIT_CRITICAL(&service_lock);
    if ((mask & WATCHDOG_ACTOR_BIT(actor)) == 0U) {
        return ESP_ERR_NOT_FOUND;
    }
    result = esp_task_wdt_reset();
    portENTER_CRITICAL(&service_lock);
    if (result == ESP_OK) {
        heartbeat = now_ms();
        rtc_store.runtime.heartbeat_ms[actor] = heartbeat;
        rtc_store.runtime.heartbeat_inverse[actor] = ~heartbeat;
    } else {
        current_diagnostics.feed_failure_count++;
    }
    portEXIT_CRITICAL(&service_lock);
    return result;
}

esp_err_t watchdog_service_unregister_current(watchdog_actor_t actor) {
    esp_err_t result;
    uint32_t mask;

    if (!service_initialized || !actor_valid(actor)) {
        return ESP_ERR_INVALID_STATE;
    }
    result = esp_task_wdt_delete(NULL);
    portENTER_CRITICAL(&service_lock);
    if (result == ESP_OK) {
        mask = rtc_store.runtime.registered_mask &
               ~WATCHDOG_ACTOR_BIT(actor);
        rtc_store.runtime.registered_mask = mask;
        rtc_store.runtime.registered_mask_inverse = ~mask;
        current_diagnostics.registered_actor_mask = mask;
    } else {
        current_diagnostics.feed_failure_count++;
    }
    portEXIT_CRITICAL(&service_lock);
    return result;
}

void watchdog_service_mark_stage(watchdog_stage_t stage) {
    if (!service_initialized || stage > WATCHDOG_STAGE_FATAL) {
        return;
    }
    portENTER_CRITICAL(&service_lock);
    rtc_store.runtime.stage = (uint32_t) stage;
    rtc_store.runtime.stage_inverse = ~(uint32_t) stage;
    if (stage != WATCHDOG_STAGE_ACTIVE) {
        active_started = false;
        active_since_ms = 0U;
        current_diagnostics.active_stable_ms = 0U;
    }
    portEXIT_CRITICAL(&service_lock);
}

void watchdog_service_mark_user_confirmed(void) {
    if (!service_initialized) {
        return;
    }
    portENTER_CRITICAL(&service_lock);
    clear_recovery_attempts();
    portEXIT_CRITICAL(&service_lock);
}

void watchdog_service_update_active(void) {
    uint32_t current_ms;

    if (!service_initialized) {
        return;
    }
    current_ms = now_ms();
    portENTER_CRITICAL(&service_lock);
    rtc_store.runtime.stage = (uint32_t) WATCHDOG_STAGE_ACTIVE;
    rtc_store.runtime.stage_inverse = ~(uint32_t) WATCHDOG_STAGE_ACTIVE;
    if (!active_started) {
        active_started = true;
        active_since_ms = current_ms;
    }
    current_diagnostics.active_stable_ms = current_ms - active_since_ms;
    if (watchdog_recovery_stable_elapsed(active_since_ms, current_ms)) {
        clear_recovery_attempts();
    }
    portEXIT_CRITICAL(&service_lock);
}

void watchdog_service_get_diagnostics(watchdog_diagnostics_t *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }
    portENTER_CRITICAL(&service_lock);
    *diagnostics = current_diagnostics;
    portEXIT_CRITICAL(&service_lock);
}
