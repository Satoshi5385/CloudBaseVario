#include "app/app_tasks.h"

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include "app/app_resources.h"
#include "domain/app_types.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "platform/app_power.h"
#include "platform/audio_output.h"
#include "platform/ble_vario.h"
#include "platform/board.h"
#include "platform/system_io.h"

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

#define SENSOR_SKELETON_PERIOD_MS UINT32_C(10)
#define AUDIO_EVALUATION_PERIOD_MS UINT32_C(10)
#define SYSTEM_SAMPLE_PERIOD_MS UINT32_C(10)
#define BLE_TX_PERIOD_MS UINT32_C(100)
#define CONSOLE_PERIOD_MS UINT32_C(100)
#define BATTERY_SAMPLE_PERIOD_MS UINT32_C(100)
#define SWITCH_DEBOUNCE_MS UINT32_C(30)
#define POWER_OFF_HOLD_MS UINT32_C(2000)
#define POWER_OFF_TIMEOUT_MS UINT32_C(1000)
#define SAFE_STOP_PERIOD_MS UINT32_C(1000)
#define LED_PULSE_MS UINT32_C(50)
#define NORMAL_LED_PERIOD_MS UINT32_C(1000)
#define FATAL_LED_PERIOD_MS UINT32_C(500)

typedef struct {
    bool candidate_pressed;
    bool stable_pressed;
    bool stable_valid;
    uint32_t stable_time_ms;
} button_debounce_t;

static const char *TAG = "app_tasks";

/* Application task handles are retained for diagnostics and future shutdown work. */
static TaskHandle_t sensor_task_handle = NULL;
static TaskHandle_t audio_task_handle = NULL;
static TaskHandle_t system_task_handle = NULL;
static TaskHandle_t ble_tx_task_handle = NULL;
static TaskHandle_t console_task_handle = NULL;
static EventBits_t active_ack_mask = 0U;

_Static_assert(configMAX_PRIORITIES >= 25, "SW_spec.md requires at least 25 priorities");

static bool app_stop_requested(void) {
    EventGroupHandle_t event_group = app_resources_event_group();
    EventBits_t bits = 0U;

    if (event_group == NULL) {
        return false;
    }
    bits = xEventGroupGetBits(event_group);
    return (bits & APP_EVENT_STOP_REQUEST) != 0U;
}

static bool app_fatal_state(void) {
    EventGroupHandle_t event_group = app_resources_event_group();
    EventBits_t bits = 0U;

    if (event_group == NULL) {
        return true;
    }
    bits = xEventGroupGetBits(event_group);
    return (bits & APP_EVENT_FATAL_STATE) != 0U;
}

static bool debounce_button(button_debounce_t *state, bool pressed) {
    if (state == NULL) {
        return false;
    }

    if (pressed != state->candidate_pressed) {
        state->candidate_pressed = pressed;
        state->stable_time_ms = SYSTEM_SAMPLE_PERIOD_MS;
        return state->stable_pressed;
    }

    if (state->stable_time_ms < SWITCH_DEBOUNCE_MS) {
        state->stable_time_ms += SYSTEM_SAMPLE_PERIOD_MS;
    }
    if (state->stable_time_ms >= SWITCH_DEBOUNCE_MS) {
        state->stable_pressed = state->candidate_pressed;
        state->stable_valid = true;
    }
    return state->stable_pressed;
}

static void acknowledge_and_delete(EventBits_t acknowledgement_bit) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group, acknowledgement_bit);
    }
    vTaskDelete(NULL);
}

static void block_safe_stop_light_sleep(void) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group, APP_EVENT_SAFE_SLEEP_BLOCKED);
    }
}

static void sensor_task(void *context) {
    bool watchdog_registered = false;

    (void) context;
    ESP_LOGI(TAG, "sensor_task started on core %d", xPortGetCoreID());
    if (esp_task_wdt_add(NULL) == ESP_OK) {
        watchdog_registered = true;
    } else {
        ESP_LOGW(TAG, "sensor_task watchdog registration failed");
    }

    for (;;) {
        if (app_stop_requested()) {
            if (watchdog_registered) {
                (void) esp_task_wdt_delete(NULL);
            }
            acknowledge_and_delete(APP_EVENT_SENSOR_ACK);
        }
        if (app_fatal_state() && watchdog_registered) {
            (void) esp_task_wdt_delete(NULL);
            watchdog_registered = false;
        }

        esp_err_t power_ret = app_power_sensor_work_begin();

        /* Sensor device registration, acquisition, and estimation are added in the next layer. */
        if (power_ret == ESP_OK) {
            power_ret = app_power_sensor_work_end();
            if (power_ret != ESP_OK) {
                block_safe_stop_light_sleep();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_SKELETON_PERIOD_MS));
        if (watchdog_registered) {
            (void) esp_task_wdt_reset();
        }
    }
}

static void audio_task(void *context) {
    QueueHandle_t queue = app_resources_audio_queue();
    vario_result_t result = {0};
    bool watchdog_registered = false;

    (void) context;
    ESP_LOGI(TAG, "audio_task started on core %d", xPortGetCoreID());
    audio_output_shutdown();
    if (esp_task_wdt_add(NULL) == ESP_OK) {
        watchdog_registered = true;
    } else {
        ESP_LOGW(TAG, "audio_task watchdog registration failed");
    }

    for (;;) {
        if (app_stop_requested()) {
            audio_output_shutdown();
            if (watchdog_registered) {
                (void) esp_task_wdt_delete(NULL);
            }
            acknowledge_and_delete(APP_EVENT_AUDIO_ACK);
        }
        if (app_fatal_state()) {
            audio_output_shutdown();
            if (watchdog_registered) {
                (void) esp_task_wdt_delete(NULL);
                watchdog_registered = false;
            }
        }

        if (queue != NULL) {
            (void) xQueueReceive(queue, &result, pdMS_TO_TICKS(AUDIO_EVALUATION_PERIOD_MS));
        } else {
            vTaskDelay(pdMS_TO_TICKS(AUDIO_EVALUATION_PERIOD_MS));
        }
        if (watchdog_registered) {
            (void) esp_task_wdt_reset();
        }
    }
}

static void request_power_off(system_snapshot_t *snapshot) {
    EventGroupHandle_t event_group = app_resources_event_group();
    EventBits_t wait_mask = active_ack_mask & ~APP_EVENT_SYSTEM_ACK;
    int64_t shutdown_started_us = esp_timer_get_time();
    int64_t elapsed_us = 0;
    uint32_t remaining_ms = POWER_OFF_TIMEOUT_MS;
    EventBits_t acknowledged_bits = 0U;
    bool all_workers_stopped = wait_mask == 0U;
    bool safe_sleep_enabled = false;

    if (snapshot != NULL) {
        snapshot->power_off_requested = true;
        snapshot->timestamp_us = shutdown_started_us;
        (void) app_resources_publish_system(snapshot);
    }

    board_set_safe_indicators();
    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group, APP_EVENT_STOP_REQUEST);
    }

    ble_vario_begin_shutdown();

    elapsed_us = esp_timer_get_time() - shutdown_started_us;
    if (elapsed_us > 0) {
        uint32_t elapsed_ms = (uint32_t) (elapsed_us / INT64_C(1000));
        if (elapsed_ms < POWER_OFF_TIMEOUT_MS) {
            remaining_ms = POWER_OFF_TIMEOUT_MS - elapsed_ms;
        } else {
            remaining_ms = 0U;
        }
    }

    if (event_group != NULL && wait_mask != 0U && remaining_ms > 0U) {
        acknowledged_bits = xEventGroupWaitBits(event_group, wait_mask, pdFALSE, pdTRUE,
                                                pdMS_TO_TICKS(remaining_ms));
        all_workers_stopped = (acknowledged_bits & wait_mask) == wait_mask;
    }
    if (event_group != NULL &&
        (xEventGroupGetBits(event_group) & APP_EVENT_SAFE_SLEEP_BLOCKED) != 0U) {
        all_workers_stopped = false;
    }

    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group, APP_EVENT_SYSTEM_ACK);
    }
    board_set_safe_indicators();
    (void) board_set_power_hold(false);
    if (all_workers_stopped) {
        esp_err_t power_ret = app_power_enter_safe_stop();

        if (power_ret != ESP_OK) {
            ESP_LOGW(TAG, "safe-stop light sleep unavailable: %s", esp_err_to_name(power_ret));
        } else {
            safe_sleep_enabled = true;
        }
    } else {
        ESP_LOGW(TAG, "safe-stop light sleep blocked because worker shutdown timed out");
    }

    for (;;) {
        if (!safe_sleep_enabled && event_group != NULL && wait_mask != 0U) {
            EventBits_t bits = xEventGroupGetBits(event_group);

            if ((bits & wait_mask) == wait_mask && (bits & APP_EVENT_SAFE_SLEEP_BLOCKED) == 0U) {
                safe_sleep_enabled = app_power_enter_safe_stop() == ESP_OK;
            }
        }
        board_set_safe_indicators();
        vTaskDelay(pdMS_TO_TICKS(SAFE_STOP_PERIOD_MS));
    }
}

static void system_task(void *context) {
    button_debounce_t sw1 = {0};
    button_debounce_t sw2 = {0};
    button_debounce_t sw3 = {0};
    system_snapshot_t snapshot = {0};
    uint32_t sw1_hold_time_ms = 0U;
    uint32_t battery_elapsed_ms = BATTERY_SAMPLE_PERIOD_MS;
    uint32_t led_elapsed_ms = 0U;
    bool sw1_was_released = false;

    (void) context;
    ESP_LOGI(TAG, "system_task started on core %d", xPortGetCoreID());

    sw1.candidate_pressed = board_is_sw1_pressed();
    sw1.stable_pressed = sw1.candidate_pressed;
    sw2.candidate_pressed = system_io_sw2_pressed();
    sw2.stable_pressed = sw2.candidate_pressed;
    sw3.candidate_pressed = system_io_sw3_pressed();
    sw3.stable_pressed = sw3.candidate_pressed;

    for (;;) {
        uint32_t led_period_ms = NORMAL_LED_PERIOD_MS;

        snapshot.timestamp_us = esp_timer_get_time();
        snapshot.sw1_pressed = debounce_button(&sw1, board_is_sw1_pressed());
        snapshot.sw2_pressed = debounce_button(&sw2, system_io_sw2_pressed());
        snapshot.sw3_pressed = debounce_button(&sw3, system_io_sw3_pressed());
        snapshot.external_power_present = system_io_external_power_present();

        if (sw1.stable_valid && !snapshot.sw1_pressed) {
            sw1_was_released = true;
            sw1_hold_time_ms = 0U;
        } else if (sw1.stable_valid && sw1_was_released) {
            sw1_hold_time_ms += SYSTEM_SAMPLE_PERIOD_MS;
        } else {
            /* The switch used to start the board is not a power-off request. */
        }

        battery_elapsed_ms += SYSTEM_SAMPLE_PERIOD_MS;
        if (battery_elapsed_ms >= BATTERY_SAMPLE_PERIOD_MS) {
            snapshot.battery_valid = system_io_read_battery_voltage(&snapshot.battery_voltage_v);
            battery_elapsed_ms = 0U;
        }

        if (app_fatal_state()) {
            led_period_ms = FATAL_LED_PERIOD_MS;
        }
        led_elapsed_ms += SYSTEM_SAMPLE_PERIOD_MS;
        if (led_elapsed_ms >= led_period_ms) {
            led_elapsed_ms -= led_period_ms;
        }
        board_set_status_leds(false, led_elapsed_ms < LED_PULSE_MS);

        (void) app_resources_publish_system(&snapshot);

        if (sw1_was_released && sw1_hold_time_ms >= POWER_OFF_HOLD_MS) {
            request_power_off(&snapshot);
        }

        vTaskDelay(pdMS_TO_TICKS(SYSTEM_SAMPLE_PERIOD_MS));
    }
}

static void console_task(void *context) {
    QueueHandle_t queue = app_resources_diagnostic_queue();
    diagnostic_event_t event = {0};
    app_power_diagnostics_t power_diagnostics = {0};

    (void) context;
    ESP_LOGI(TAG, "console_task started on core %d", xPortGetCoreID());
    app_power_get_diagnostics(&power_diagnostics);
    ESP_LOGI(TAG,
             "power: cpu=%" PRIu32 "MHz sensor_lock=%d sleep_lock=%d sleep_count=%" PRIu32
             " freq_changes=%" PRIu32 " lock_errors=%" PRIu32,
             power_diagnostics.current_cpu_frequency_mhz, power_diagnostics.sensor_cpu_lock_held,
             power_diagnostics.light_sleep_lock_held, power_diagnostics.light_sleep_entry_count,
             power_diagnostics.observed_frequency_switch_count, power_diagnostics.lock_error_count);

    for (;;) {
        if (app_stop_requested()) {
            acknowledge_and_delete(APP_EVENT_CONSOLE_ACK);
        }

        if (queue != NULL &&
            xQueueReceive(queue, &event, pdMS_TO_TICKS(CONSOLE_PERIOD_MS)) == pdTRUE) {
            ESP_LOGI(TAG, "diagnostic event type=%d detail=%" PRId32, (int) event.type,
                     event.detail);
        } else if (queue == NULL) {
            vTaskDelay(pdMS_TO_TICKS(CONSOLE_PERIOD_MS));
        } else {
            /* Queue timeout is the normal console-task pacing mechanism. */
        }
    }
}

static void ble_tx_task(void *context) {
    (void) context;
    ESP_LOGI(TAG, "ble_tx_task started on core %d", xPortGetCoreID());

    for (;;) {
        if (app_stop_requested()) {
            if (ble_vario_stop() != ESP_OK) {
                block_safe_stop_light_sleep();
            }
            acknowledge_and_delete(APP_EVENT_BLE_TX_ACK);
        }

        /* LK8EX1 formatting and fragment scheduling are added above this layer. */
        vTaskDelay(pdMS_TO_TICKS(BLE_TX_PERIOD_MS));
    }
}

static esp_err_t create_task(TaskFunction_t function, const char *name, uint32_t stack_size_bytes,
                             UBaseType_t priority, BaseType_t core_id, TaskHandle_t *task_handle,
                             EventBits_t acknowledgement_bit) {
    BaseType_t result = pdFAIL;

    result = xTaskCreatePinnedToCore(function, name, stack_size_bytes, NULL, priority, task_handle,
                                     core_id);
    if (result != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    active_ack_mask |= acknowledgement_bit;
    return ESP_OK;
}

esp_err_t app_tasks_start(void) {
    EventGroupHandle_t event_group = app_resources_event_group();
    esp_err_t ret = ESP_OK;

    active_ack_mask = 0U;

    ret = create_task(audio_task, "audio_task", AUDIO_TASK_STACK_BYTES, AUDIO_TASK_PRIORITY,
                      HIGH_RATE_TASK_CORE, &audio_task_handle, APP_EVENT_AUDIO_ACK);
    if (ret == ESP_OK) {
        ret = create_task(system_task, "system_task", SYSTEM_TASK_STACK_BYTES, SYSTEM_TASK_PRIORITY,
                          COMMUNICATION_TASK_CORE, &system_task_handle, APP_EVENT_SYSTEM_ACK);
    }
    if (ret == ESP_OK) {
        ret = create_task(sensor_task, "sensor_task", SENSOR_TASK_STACK_BYTES, SENSOR_TASK_PRIORITY,
                          HIGH_RATE_TASK_CORE, &sensor_task_handle, APP_EVENT_SENSOR_ACK);
    }
    if (ret == ESP_OK) {
        ret = create_task(console_task, "console_task", CONSOLE_TASK_STACK_BYTES,
                          CONSOLE_TASK_PRIORITY, COMMUNICATION_TASK_CORE, &console_task_handle,
                          APP_EVENT_CONSOLE_ACK);
    }
    if (ret == ESP_OK) {
        ret = create_task(ble_tx_task, "ble_tx_task", BLE_TX_TASK_STACK_BYTES, BLE_TX_TASK_PRIORITY,
                          COMMUNICATION_TASK_CORE, &ble_tx_task_handle, APP_EVENT_BLE_TX_ACK);
    }

    if (ret != ESP_OK) {
        board_set_safe_indicators();
        if (event_group != NULL) {
            (void) xEventGroupSetBits(event_group, APP_EVENT_FATAL_STATE);
        }
        ESP_LOGE(TAG, "task creation failed; entering fatal state");
        return ret;
    }

    return ESP_OK;
}

EventBits_t app_tasks_active_ack_mask(void) {
    return active_ack_mask;
}

bool app_tasks_system_started(void) {
    return system_task_handle != NULL;
}

void app_tasks_run_fatal_fallback(void) {
    button_debounce_t sw1 = {0};
    uint32_t hold_time_ms = 0U;
    uint32_t led_elapsed_ms = 0U;
    bool was_released = false;

    sw1.candidate_pressed = board_is_sw1_pressed();
    sw1.stable_pressed = sw1.candidate_pressed;

    for (;;) {
        bool pressed = debounce_button(&sw1, board_is_sw1_pressed());

        if (sw1.stable_valid && !pressed) {
            was_released = true;
            hold_time_ms = 0U;
        } else if (sw1.stable_valid && was_released) {
            hold_time_ms += SYSTEM_SAMPLE_PERIOD_MS;
        } else {
            /* Ignore the switch that was already held during startup. */
        }

        led_elapsed_ms += SYSTEM_SAMPLE_PERIOD_MS;
        if (led_elapsed_ms >= FATAL_LED_PERIOD_MS) {
            led_elapsed_ms -= FATAL_LED_PERIOD_MS;
        }
        board_set_status_leds(false, led_elapsed_ms < LED_PULSE_MS);

        if (was_released && hold_time_ms >= POWER_OFF_HOLD_MS) {
            board_set_safe_indicators();
            (void) board_set_power_hold(false);
            for (;;) {
                board_set_safe_indicators();
                vTaskDelay(pdMS_TO_TICKS(SAFE_STOP_PERIOD_MS));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SYSTEM_SAMPLE_PERIOD_MS));
    }
}
