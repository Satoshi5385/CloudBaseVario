#include "app/app_resources.h"

#include <float.h>
#include <math.h>
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
static SemaphoreHandle_t config_mutex = NULL;
static SemaphoreHandle_t debug_mutex = NULL;
static EventGroupHandle_t app_event_group = NULL;

/* Complete latest-value snapshots protected by their respective mutexes. */
static vario_result_t latest_vario;
static imu_diagnostics_t latest_imu_diagnostics;
static system_snapshot_t latest_system;
static app_config_t latest_config;
static struct {
    float climb_rate_mps;
    float altitude_m;
    int64_t altitude_timestamp_us;
    int32_t pressure_pa_x100;
    bool pressure_override_valid;
    bool altitude_valid;
    bool active;
} debug_vario;

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
    if (config_mutex != NULL) {
        vSemaphoreDelete(config_mutex);
        config_mutex = NULL;
    }
    if (debug_mutex != NULL) {
        vSemaphoreDelete(debug_mutex);
        debug_mutex = NULL;
    }
    if (app_event_group != NULL) {
        vEventGroupDelete(app_event_group);
        app_event_group = NULL;
    }
}

esp_err_t app_resources_init(void) {
    memset(&latest_vario, 0, sizeof(latest_vario));
    memset(&latest_imu_diagnostics, 0, sizeof(latest_imu_diagnostics));
    memset(&latest_system, 0, sizeof(latest_system));
    app_config_set_defaults(&latest_config);
    memset(&debug_vario, 0, sizeof(debug_vario));

    audio_queue = xQueueCreate(AUDIO_QUEUE_LENGTH, sizeof(vario_result_t));
    diagnostic_queue = xQueueCreate(DIAGNOSTIC_QUEUE_LENGTH, sizeof(diagnostic_event_t));
    vario_mutex = xSemaphoreCreateMutex();
    system_mutex = xSemaphoreCreateMutex();
    config_mutex = xSemaphoreCreateMutex();
    debug_mutex = xSemaphoreCreateMutex();
    app_event_group = xEventGroupCreate();

    if (audio_queue == NULL || diagnostic_queue == NULL || vario_mutex == NULL ||
        system_mutex == NULL || config_mutex == NULL || debug_mutex == NULL ||
        app_event_group == NULL) {
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

bool app_resources_publish_imu_diagnostics(const imu_diagnostics_t *diagnostics) {
    BaseType_t lock_result = pdFALSE;

    if (diagnostics == NULL || vario_mutex == NULL) {
        return false;
    }

    lock_result = xSemaphoreTake(vario_mutex, pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS));
    if (lock_result != pdTRUE) {
        return false;
    }
    latest_imu_diagnostics = *diagnostics;
    (void) xSemaphoreGive(vario_mutex);
    return true;
}

bool app_resources_copy_imu_diagnostics(imu_diagnostics_t *diagnostics) {
    BaseType_t lock_result = pdFALSE;

    if (diagnostics == NULL || vario_mutex == NULL) {
        return false;
    }

    lock_result = xSemaphoreTake(vario_mutex, pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS));
    if (lock_result != pdTRUE) {
        return false;
    }
    *diagnostics = latest_imu_diagnostics;
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

bool app_resources_publish_config(const app_config_t *config) {
    BaseType_t lock_result = pdFALSE;

    if (config == NULL || config_mutex == NULL || !app_config_validate(config)) {
        return false;
    }

    lock_result = xSemaphoreTake(config_mutex, pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS));
    if (lock_result != pdTRUE) {
        return false;
    }
    latest_config = *config;
    (void) xSemaphoreGive(config_mutex);
    return true;
}

bool app_resources_copy_config(app_config_t *config) {
    BaseType_t lock_result = pdFALSE;

    if (config == NULL || config_mutex == NULL) {
        return false;
    }

    lock_result = xSemaphoreTake(config_mutex, pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS));
    if (lock_result != pdTRUE) {
        return false;
    }
    *config = latest_config;
    (void) xSemaphoreGive(config_mutex);
    return true;
}

bool app_resources_set_debug_vario(float climb_rate_mps,
                                   bool pressure_override_valid,
                                   int32_t pressure_pa_x100) {
    if (debug_mutex == NULL || !isfinite(climb_rate_mps) ||
        climb_rate_mps < -50.0f || climb_rate_mps > 50.0f ||
        (pressure_override_valid &&
         (pressure_pa_x100 < INT32_C(3000000) ||
          pressure_pa_x100 > INT32_C(12500000)))) {
        return false;
    }
    if (xSemaphoreTake(debug_mutex,
                       pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS)) != pdTRUE) {
        return false;
    }
    if (!debug_vario.active) {
        debug_vario.altitude_m = 0.0f;
        debug_vario.altitude_timestamp_us = 0;
        debug_vario.altitude_valid = false;
    }
    debug_vario.climb_rate_mps = climb_rate_mps;
    debug_vario.pressure_pa_x100 = pressure_pa_x100;
    debug_vario.pressure_override_valid = pressure_override_valid;
    debug_vario.active = true;
    (void) xSemaphoreGive(debug_mutex);
    return true;
}

void app_resources_clear_debug_vario(void) {
    if (debug_mutex == NULL ||
        xSemaphoreTake(debug_mutex,
                       pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS)) != pdTRUE) {
        return;
    }
    memset(&debug_vario, 0, sizeof(debug_vario));
    (void) xSemaphoreGive(debug_mutex);
}

bool app_resources_apply_debug_vario(vario_result_t *result,
                                     int64_t current_time_us) {
    bool active = false;

    if (result == NULL || current_time_us <= 0 || debug_mutex == NULL ||
        xSemaphoreTake(debug_mutex,
                       pdMS_TO_TICKS(SNAPSHOT_MUTEX_WAIT_MS)) != pdTRUE) {
        return false;
    }
    active = debug_vario.active;
    if (active) {
        if (!debug_vario.altitude_valid) {
            debug_vario.altitude_m =
                result->estimate_valid && isfinite(result->altitude_m)
                    ? result->altitude_m
                    : 0.0f;
            debug_vario.altitude_timestamp_us = current_time_us;
            debug_vario.altitude_valid = true;
        } else if (current_time_us > debug_vario.altitude_timestamp_us) {
            double elapsed_seconds =
                (double) (current_time_us -
                          debug_vario.altitude_timestamp_us) /
                1000000.0;
            double altitude_m =
                (double) debug_vario.altitude_m +
                (double) debug_vario.climb_rate_mps * elapsed_seconds;

            if (isfinite(altitude_m) &&
                altitude_m >= -(double) FLT_MAX &&
                altitude_m <= (double) FLT_MAX) {
                debug_vario.altitude_m = (float) altitude_m;
            }
            debug_vario.altitude_timestamp_us = current_time_us;
        }
        result->timestamp_us = current_time_us;
        result->altitude_m = debug_vario.altitude_m;
        result->climb_rate_mps = debug_vario.climb_rate_mps;
        result->climb_rate_valid = true;
        result->estimate_valid = true;
        result->debug_input_active = true;
        if (debug_vario.pressure_override_valid) {
            result->pressure_pa_x100 = debug_vario.pressure_pa_x100;
            result->pressure_valid = true;
        }
    } else {
        result->debug_input_active = false;
    }
    (void) xSemaphoreGive(debug_mutex);
    return active;
}

bool app_resources_post_diagnostic(const diagnostic_event_t *event) {
    BaseType_t queue_result = pdFALSE;

    if (event == NULL || diagnostic_queue == NULL) {
        return false;
    }

    queue_result = xQueueSend(diagnostic_queue, event, 0U);
    return queue_result == pdTRUE;
}
