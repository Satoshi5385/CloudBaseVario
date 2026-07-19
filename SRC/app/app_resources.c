#include "app/app_resources.h"

#include <stddef.h>
#include <string.h>

#include "freertos/semphr.h"

#define AUDIO_QUEUE_LENGTH ((UBaseType_t) 1U)
#define DIAGNOSTIC_QUEUE_LENGTH ((UBaseType_t) 16U)
#define SNAPSHOT_MUTEX_WAIT_MS UINT32_C(5)

/* RTOS handles are allocated once during application startup. */
static QueueHandle_t audio_queue = NULL;
static QueueHandle_t diagnostic_queue = NULL;
static SemaphoreHandle_t vario_mutex = NULL;
static SemaphoreHandle_t system_mutex = NULL;
static EventGroupHandle_t app_event_group = NULL;

/* Complete latest-value snapshots protected by their respective mutexes. */
static vario_result_t latest_vario;
static system_snapshot_t latest_system;

static void app_resources_release_partial(void) {
    if (audio_queue != NULL) {
        vQueueDelete(audio_queue);
        audio_queue = NULL;
    }
    if (diagnostic_queue != NULL) {
        vQueueDelete(diagnostic_queue);
        diagnostic_queue = NULL;
    }
    if (vario_mutex != NULL) {
        vSemaphoreDelete(vario_mutex);
        vario_mutex = NULL;
    }
    if (system_mutex != NULL) {
        vSemaphoreDelete(system_mutex);
        system_mutex = NULL;
    }
    if (app_event_group != NULL) {
        vEventGroupDelete(app_event_group);
        app_event_group = NULL;
    }
}

esp_err_t app_resources_init(void) {
    memset(&latest_vario, 0, sizeof(latest_vario));
    memset(&latest_system, 0, sizeof(latest_system));

    audio_queue = xQueueCreate(AUDIO_QUEUE_LENGTH, sizeof(vario_result_t));
    diagnostic_queue = xQueueCreate(DIAGNOSTIC_QUEUE_LENGTH, sizeof(diagnostic_event_t));
    vario_mutex = xSemaphoreCreateMutex();
    system_mutex = xSemaphoreCreateMutex();
    app_event_group = xEventGroupCreate();

    if (audio_queue == NULL || diagnostic_queue == NULL || vario_mutex == NULL ||
        system_mutex == NULL || app_event_group == NULL) {
        app_resources_release_partial();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

QueueHandle_t app_resources_audio_queue(void) {
    return audio_queue;
}

QueueHandle_t app_resources_diagnostic_queue(void) {
    return diagnostic_queue;
}

EventGroupHandle_t app_resources_event_group(void) {
    return app_event_group;
}

bool app_resources_publish_vario(const vario_result_t *result) {
    BaseType_t lock_result = pdFALSE;

    if (result == NULL || vario_mutex == NULL || audio_queue == NULL) {
        return false;
    }

    lock_result = xSemaphoreTake(vario_mutex, pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS));
    if (lock_result != pdTRUE) {
        return false;
    }
    latest_vario = *result;
    (void) xSemaphoreGive(vario_mutex);

    (void) xQueueOverwrite(audio_queue, result);
    return true;
}

bool app_resources_copy_vario(vario_result_t *result) {
    BaseType_t lock_result = pdFALSE;

    if (result == NULL || vario_mutex == NULL) {
        return false;
    }

    lock_result = xSemaphoreTake(vario_mutex, pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS));
    if (lock_result != pdTRUE) {
        return false;
    }
    *result = latest_vario;
    (void) xSemaphoreGive(vario_mutex);
    return true;
}

bool app_resources_publish_system(const system_snapshot_t *snapshot) {
    BaseType_t lock_result = pdFALSE;

    if (snapshot == NULL || system_mutex == NULL) {
        return false;
    }

    lock_result = xSemaphoreTake(system_mutex, pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS));
    if (lock_result != pdTRUE) {
        return false;
    }
    latest_system = *snapshot;
    (void) xSemaphoreGive(system_mutex);
    return true;
}

bool app_resources_copy_system(system_snapshot_t *snapshot) {
    BaseType_t lock_result = pdFALSE;

    if (snapshot == NULL || system_mutex == NULL) {
        return false;
    }

    lock_result = xSemaphoreTake(system_mutex, pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS));
    if (lock_result != pdTRUE) {
        return false;
    }
    *snapshot = latest_system;
    (void) xSemaphoreGive(system_mutex);
    return true;
}

bool app_resources_post_diagnostic(const diagnostic_event_t *event) {
    BaseType_t queue_result = pdFALSE;

    if (event == NULL || diagnostic_queue == NULL) {
        return false;
    }

    queue_result = xQueueSend(diagnostic_queue, event, 0U);
    return queue_result == pdTRUE;
}
