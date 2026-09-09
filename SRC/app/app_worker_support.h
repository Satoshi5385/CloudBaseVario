#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "domain/app_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "platform/watchdog_service.h"

/** Return true after the shared stop request has been raised. */
bool app_worker_stop_requested(void);
/** Return true while the application is in the fatal state. */
bool app_worker_fatal_state(void);
/** Publish a runtime diagnostic event. */
void app_worker_post_runtime_diagnostic(diagnostic_event_type_t type,
                                        esp_err_t detail);
/** Register the current task with the critical watchdog. */
bool app_worker_register_watchdog(watchdog_actor_t actor,
                                  const char *task_name);
/** Feed a previously registered critical watchdog actor. */
void app_worker_feed_watchdog(watchdog_actor_t actor,
                              bool watchdog_registered);
/** Unregister the current task from its critical watchdog actor. */
void app_worker_unregister_watchdog(watchdog_actor_t actor,
                                    bool *watchdog_registered);
/** Publish a stop acknowledgement and delete the current task. */
void app_worker_acknowledge_and_delete(EventBits_t acknowledgement_bit);
/** Prevent SAFE_STOP light sleep after a PM-lock lifecycle failure. */
void app_worker_block_safe_stop_light_sleep(void);
/** Add to a diagnostic counter without wrapping. */
void app_worker_add_saturating_u32(uint32_t *counter, uint32_t increment);
