#include "app/app_worker_support.h"

#include <limits.h>

#include "app/app_resources.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

static const char *TAG = "worker_support";

bool app_worker_stop_requested(void) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (event_group == NULL) {
        return false;
    }
    return (xEventGroupGetBits(event_group) & APP_EVENT_STOP_REQUEST) != 0U;
}

bool app_worker_fatal_state(void) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (event_group == NULL) {
        return true;
    }
    return (xEventGroupGetBits(event_group) & APP_EVENT_FATAL_STATE) != 0U;
}

void app_worker_post_runtime_diagnostic(diagnostic_event_type_t type,
                                        esp_err_t detail) {
    diagnostic_event_t event = {
        .type = type,
        .timestamp_us = esp_timer_get_time(),
        .detail = (int32_t) detail,
    };

    (void) app_resources_post_diagnostic(&event);
}

bool app_worker_register_watchdog(watchdog_actor_t actor,
                                  const char *task_name) {
    EventGroupHandle_t event_group = app_resources_event_group();
    esp_err_t result = watchdog_service_register_current(actor);

    if (result == ESP_OK) {
        return true;
    }
    ESP_LOGE(TAG, "%s watchdog registration failed: %s", task_name,
             esp_err_to_name(result));
    app_worker_post_runtime_diagnostic(DIAGNOSTIC_EVENT_TASK_FAILURE,
                                       result);
    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group, APP_EVENT_FATAL_STATE);
    }
    watchdog_service_mark_stage(WATCHDOG_STAGE_FATAL);
    return false;
}

void app_worker_feed_watchdog(watchdog_actor_t actor,
                              bool watchdog_registered) {
    if (watchdog_registered) {
        (void) watchdog_service_feed(actor);
    }
}

void app_worker_unregister_watchdog(watchdog_actor_t actor,
                                    bool *watchdog_registered) {
    if (watchdog_registered != NULL && *watchdog_registered) {
        (void) watchdog_service_unregister_current(actor);
        *watchdog_registered = false;
    }
}

void app_worker_acknowledge_and_delete(EventBits_t acknowledgement_bit) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group, acknowledgement_bit);
    }
    vTaskDelete(NULL);
}

void app_worker_block_safe_stop_light_sleep(void) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group, APP_EVENT_SAFE_SLEEP_BLOCKED);
    }
}

void app_worker_add_saturating_u32(uint32_t *counter, uint32_t increment) {
    if (counter == NULL) {
        return;
    }
    if (UINT32_MAX - *counter < increment) {
        *counter = UINT32_MAX;
    } else {
        *counter += increment;
    }
}
