#ifndef CLOUDBASEVARIO_DOMAIN_WATCHDOG_RECOVERY_POLICY_H
#define CLOUDBASEVARIO_DOMAIN_WATCHDOG_RECOVERY_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#define WATCHDOG_RECOVERY_STABLE_PERIOD_MS UINT32_C(300000)
#define WATCHDOG_RECOVERY_STATE_MAGIC UINT32_C(0x57445250)
#define WATCHDOG_RECOVERY_STATE_VERSION UINT32_C(1)

typedef enum {
    WATCHDOG_RESET_POWER_ON = 0,
    WATCHDOG_RESET_SOFTWARE,
    WATCHDOG_RESET_PANIC,
    WATCHDOG_RESET_TASK_WDT,
    WATCHDOG_RESET_INTERRUPT_WDT,
    WATCHDOG_RESET_OTHER_WDT,
    WATCHDOG_RESET_BROWNOUT,
    WATCHDOG_RESET_OTHER,
} watchdog_reset_kind_t;

typedef enum {
    WATCHDOG_BOOT_REQUIRE_SW1 = 0,
    WATCHDOG_BOOT_OTA_CONFIRMATION,
    WATCHDOG_BOOT_AUTO_RECOVER,
} watchdog_boot_action_t;

typedef enum {
    WATCHDOG_STAGE_UNKNOWN = 0,
    WATCHDOG_STAGE_BOOT,
    WATCHDOG_STAGE_POWER_ON_WAIT,
    WATCHDOG_STAGE_INITIALIZING,
    WATCHDOG_STAGE_ACTIVE,
    WATCHDOG_STAGE_SHUTTING_DOWN,
    WATCHDOG_STAGE_SAFE_STOP,
    WATCHDOG_STAGE_FATAL,
} watchdog_stage_t;

typedef enum {
    WATCHDOG_ACTOR_STARTUP = 0,
    WATCHDOG_ACTOR_STARTUP_PREP,
    WATCHDOG_ACTOR_SENSOR,
    WATCHDOG_ACTOR_AUDIO,
    WATCHDOG_ACTOR_SYSTEM,
    WATCHDOG_ACTOR_COUNT,
    WATCHDOG_ACTOR_UNKNOWN = -1,
} watchdog_actor_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    uint32_t watchdog_reset_count;
    uint32_t last_watchdog_reset;
    uint32_t last_watchdog_stage;
    uint32_t suspected_actor;
    uint32_t checksum;
} watchdog_recovery_state_t;

typedef struct {
    watchdog_boot_action_t action;
    watchdog_recovery_state_t next_state;
} watchdog_recovery_decision_t;

/** Report whether a reset kind is eligible for bounded automatic recovery. */
bool watchdog_recovery_reset_is_eligible(watchdog_reset_kind_t reset_kind);
/** Initialize an empty retained recovery journal entry. */
void watchdog_recovery_state_initialize(watchdog_recovery_state_t *state);
/** Validate a retained recovery journal entry and checksum. */
bool watchdog_recovery_state_valid(const watchdog_recovery_state_t *state);
/** Select the newest valid entry from the two-slot retained journal. */
bool watchdog_recovery_select_state(const watchdog_recovery_state_t *first,
                                    const watchdog_recovery_state_t *second,
                                    watchdog_recovery_state_t *selected);
/** Decide the boot action and next retained state for one reset. */
watchdog_recovery_decision_t watchdog_recovery_decide(
    watchdog_reset_kind_t reset_kind, bool ota_pending_verify,
    bool prior_state_valid, const watchdog_recovery_state_t *prior_state,
    watchdog_stage_t previous_stage, watchdog_actor_t suspected_actor);
/** Clear recovery-attempt history after physical confirmation or stability. */
void watchdog_recovery_clear_attempts(watchdog_recovery_state_t *state);
/** Report whether the active period reached the stable recovery interval. */
bool watchdog_recovery_stable_elapsed(uint32_t active_since_ms,
                                      uint32_t now_ms);
/** Return the diagnostic name of a reset kind. */
const char *watchdog_reset_kind_name(watchdog_reset_kind_t reset_kind);
/** Return the diagnostic name of a boot action. */
const char *watchdog_boot_action_name(watchdog_boot_action_t action);
/** Return the diagnostic name of a lifecycle stage. */
const char *watchdog_stage_name(watchdog_stage_t stage);
/** Return the diagnostic name of a watchdog actor. */
const char *watchdog_actor_name(watchdog_actor_t actor);

#endif
