#ifndef CLOUDBASEVARIO_PLATFORM_WATCHDOG_SERVICE_H
#define CLOUDBASEVARIO_PLATFORM_WATCHDOG_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "domain/watchdog_recovery_policy.h"
#include "esp_err.h"

typedef struct {
    watchdog_reset_kind_t reset_kind;
    watchdog_boot_action_t boot_action;
    watchdog_stage_t previous_stage;
    watchdog_actor_t suspected_actor;
    uint32_t watchdog_reset_count;
    uint32_t active_stable_ms;
    uint32_t registered_actor_mask;
    uint32_t registration_failure_count;
    uint32_t feed_failure_count;
    bool recovery_record_valid;
} watchdog_diagnostics_t;

/** Capture reset state, update the RTC retry journal, and choose this boot path. */
esp_err_t watchdog_service_begin_boot(bool ota_pending_verify,
                                      watchdog_boot_action_t *action);

/** Register the calling task with the ESP-IDF Task WDT. */
esp_err_t watchdog_service_register_current(watchdog_actor_t actor);
/** Feed the calling task's ESP-IDF Task WDT subscription. */
esp_err_t watchdog_service_feed(watchdog_actor_t actor);
/** Remove the calling task from the ESP-IDF Task WDT. */
esp_err_t watchdog_service_unregister_current(watchdog_actor_t actor);

/** Record the current lifecycle stage in RTC memory for the next reset report. */
void watchdog_service_mark_stage(watchdog_stage_t stage);

/** A physical SW1 confirmation begins a new bounded recovery window. */
void watchdog_service_mark_user_confirmed(void);

/** Track ACTIVE time and restore one auto-recovery allowance after five minutes. */
void watchdog_service_update_active(void);

/** Copy the current boot and retained lightweight diagnostics. */
void watchdog_service_get_diagnostics(watchdog_diagnostics_t *diagnostics);

#endif
