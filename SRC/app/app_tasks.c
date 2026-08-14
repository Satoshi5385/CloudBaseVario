#include "app/app_tasks.h"

#include <stddef.h>
#include <string.h>

#include "app/app_resources.h"
#include "app/app_workers.h"
#include "app/ble_tx_worker.h"
#include "esp_log.h"
#include "platform/board.h"
#include "platform/firmware_update.h"
#include "platform/watchdog_service.h"

#define SENSOR_TASK_PRIORITY ((UBaseType_t) 20U)
#define AUDIO_TASK_PRIORITY ((UBaseType_t) 18U)
#define SYSTEM_TASK_PRIORITY ((UBaseType_t) 12U)
#define BLE_TX_TASK_PRIORITY ((UBaseType_t) 8U)
#define CONSOLE_TASK_PRIORITY ((UBaseType_t) 5U)

#define SENSOR_TASK_STACK_BYTES UINT32_C(8192)
#define AUDIO_TASK_STACK_BYTES UINT32_C(4096)
#define SYSTEM_TASK_STACK_BYTES UINT32_C(4096)
#define BLE_TX_TASK_STACK_BYTES UINT32_C(6144)
#define CONSOLE_TASK_STACK_BYTES UINT32_C(6144)

#define HIGH_RATE_TASK_CORE ((BaseType_t) 1)
#define COMMUNICATION_TASK_CORE ((BaseType_t) 0)
#define STARTUP_EVENT_WAIT_MS UINT32_C(250)

typedef struct {
    app_task_worker_t worker;
    TaskFunction_t entry;
    const char *name;
    uint32_t stack_size_bytes;
    UBaseType_t priority;
    BaseType_t core;
    EventBits_t acknowledgement_bit;
} app_task_descriptor_t;

static const app_task_descriptor_t task_descriptors[] = {
    {APP_TASK_WORKER_AUDIO, app_audio_worker_task, "audio_task",
     AUDIO_TASK_STACK_BYTES, AUDIO_TASK_PRIORITY, HIGH_RATE_TASK_CORE,
     APP_EVENT_AUDIO_ACK},
    {APP_TASK_WORKER_SYSTEM, app_system_worker_task, "system_task",
     SYSTEM_TASK_STACK_BYTES, SYSTEM_TASK_PRIORITY,
     COMMUNICATION_TASK_CORE, APP_EVENT_SYSTEM_ACK},
    {APP_TASK_WORKER_SENSOR, app_sensor_worker_task, "sensor_task",
     SENSOR_TASK_STACK_BYTES, SENSOR_TASK_PRIORITY, HIGH_RATE_TASK_CORE,
     APP_EVENT_SENSOR_ACK},
    {APP_TASK_WORKER_CONSOLE, app_console_worker_task, "console_task",
     CONSOLE_TASK_STACK_BYTES, CONSOLE_TASK_PRIORITY,
     COMMUNICATION_TASK_CORE, APP_EVENT_CONSOLE_ACK},
    {APP_TASK_WORKER_BLE_TX, ble_tx_worker_task, "ble_tx_task",
     BLE_TX_TASK_STACK_BYTES, BLE_TX_TASK_PRIORITY,
     COMMUNICATION_TASK_CORE, APP_EVENT_BLE_TX_ACK},
};

static const char *TAG = "app_tasks";
static TaskHandle_t worker_handles[APP_TASK_WORKER_COUNT];
static EventBits_t active_ack_mask;

_Static_assert(sizeof(task_descriptors) / sizeof(task_descriptors[0]) ==
                   APP_TASK_WORKER_COUNT,
               "Every required worker must have one task descriptor");
_Static_assert(configMAX_PRIORITIES >= 25,
               "SW_spec.md requires at least 25 priorities");

static esp_err_t create_worker(const app_task_descriptor_t *descriptor) {
    TaskHandle_t *handle = NULL;

    if (descriptor == NULL || descriptor->worker >= APP_TASK_WORKER_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    handle = &worker_handles[descriptor->worker];
    if (xTaskCreatePinnedToCore(
            descriptor->entry, descriptor->name,
            descriptor->stack_size_bytes, NULL, descriptor->priority,
            handle, descriptor->core) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    active_ack_mask |= descriptor->acknowledgement_bit;
    return ESP_OK;
}

void app_tasks_set_imu_accel_calibration(
    const imu_accel_calibration_t *calibration,
    const imu_calibration_storage_diagnostics_t *diagnostics) {
    app_workers_set_imu_accel_calibration(calibration, diagnostics);
}

void app_tasks_set_switch_preferences(
    const switch_preferences_t *preferences, bool dirty) {
    app_workers_set_switch_preferences(preferences, dirty);
}

esp_err_t app_tasks_play_startup_sound(
    audio_volume_level_t volume_level) {
    return app_workers_play_startup_sound(volume_level);
}

esp_err_t app_tasks_start(void) {
    EventGroupHandle_t event_group = app_resources_event_group();
    esp_err_t result = ESP_OK;

    memset(worker_handles, 0, sizeof(worker_handles));
    active_ack_mask = 0U;
    for (size_t index = 0U;
         index < sizeof(task_descriptors) / sizeof(task_descriptors[0]);
         index++) {
        result = create_worker(&task_descriptors[index]);
        if (result != ESP_OK) {
            break;
        }
    }

    if (result != ESP_OK) {
        board_set_safe_indicators();
        if (event_group != NULL) {
            (void) xEventGroupClearBits(event_group,
                                        APP_EVENT_FATAL_BMP581);
            (void) xEventGroupSetBits(event_group,
                                      APP_EVENT_FATAL_STATE);
        }
        ESP_LOGE(TAG, "task creation failed; entering fatal state");
        return result;
    }
    if (event_group == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    firmware_update_mark_workers_started();
    for (;;) {
        EventBits_t startup_bits = xEventGroupWaitBits(
            event_group,
            APP_EVENT_BMP581_STARTUP_COMPLETE | APP_EVENT_FATAL_STATE,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(STARTUP_EVENT_WAIT_MS));

        (void) watchdog_service_feed(WATCHDOG_ACTOR_STARTUP);
        if ((startup_bits & APP_EVENT_FATAL_STATE) != 0U) {
            if ((startup_bits & APP_EVENT_FATAL_BMP581) != 0U) {
                return ESP_ERR_NOT_FOUND;
            }
            return ESP_ERR_INVALID_STATE;
        }
        if ((startup_bits & APP_EVENT_BMP581_STARTUP_COMPLETE) != 0U) {
            return ESP_OK;
        }
    }
}

EventBits_t app_tasks_active_ack_mask(void) {
    return active_ack_mask;
}

bool app_tasks_system_started(void) {
    return worker_handles[APP_TASK_WORKER_SYSTEM] != NULL;
}

bool app_tasks_required_workers_started(void) {
    for (size_t index = 0U; index < APP_TASK_WORKER_COUNT; index++) {
        if (worker_handles[index] == NULL) {
            return false;
        }
    }
    return true;
}

TaskHandle_t app_tasks_worker_handle(app_task_worker_t worker) {
    if (worker >= APP_TASK_WORKER_COUNT) {
        return NULL;
    }
    return worker_handles[worker];
}

bool app_tasks_begin_storage_mode(uint32_t timeout_ms, void *arg) {
    EventGroupHandle_t event_group = app_resources_event_group();
    const EventBits_t wait_mask =
        APP_EVENT_SENSOR_QUIESCED | APP_EVENT_AUDIO_QUIESCED;
    EventBits_t bits = 0U;

    (void) arg;
    if (event_group == NULL) {
        return false;
    }
    (void) xEventGroupClearBits(event_group, wait_mask);
    (void) xEventGroupSetBits(event_group, APP_EVENT_STORAGE_MODE_REQUEST);
    if (worker_handles[APP_TASK_WORKER_SENSOR] != NULL) {
        (void) xTaskAbortDelay(worker_handles[APP_TASK_WORKER_SENSOR]);
    }
    bits = xEventGroupWaitBits(event_group, wait_mask, pdFALSE, pdTRUE,
                               pdMS_TO_TICKS(timeout_ms));
    return (bits & wait_mask) == wait_mask;
}

void app_tasks_end_storage_mode(void *arg) {
    EventGroupHandle_t event_group = app_resources_event_group();

    (void) arg;
    if (event_group == NULL) {
        return;
    }
    (void) xEventGroupClearBits(
        event_group, APP_EVENT_STORAGE_MODE_REQUEST |
                         APP_EVENT_SENSOR_QUIESCED |
                         APP_EVENT_AUDIO_QUIESCED);
    if (worker_handles[APP_TASK_WORKER_SENSOR] != NULL) {
        (void) xTaskAbortDelay(worker_handles[APP_TASK_WORKER_SENSOR]);
    }
}

void app_tasks_run_safe_stop(void) {
    app_workers_run_safe_stop();
}

void app_tasks_run_fatal_fallback(void) {
    app_workers_run_fatal_fallback();
}
