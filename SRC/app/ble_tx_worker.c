#include "app/ble_tx_worker.h"

#include "app/app_events.h"
#include "app/app_resources.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "platform/ble_vario.h"

#define BATTERY_UPDATE_PERIOD_US INT64_C(1000000)
#define MICROSECONDS_PER_SECOND INT64_C(1000000)

static const char *TAG = "ble_tx_worker";

static int64_t lk8ex1_notify_period_us(uint32_t rate_hz) {
    return MICROSECONDS_PER_SECOND / (int64_t) rate_hz;
}

static int64_t advance_periodic_deadline(int64_t deadline_us,
                                         int64_t now_us,
                                         int64_t period_us) {
    int64_t periods_elapsed = 0;

    if (deadline_us > now_us) {
        return deadline_us;
    }
    periods_elapsed = (now_us - deadline_us) / period_us;
    return deadline_us + (periods_elapsed + 1) * period_us;
}

static TickType_t deadline_wait_ticks(int64_t now_us,
                                      int64_t first_deadline_us,
                                      int64_t second_deadline_us,
                                      bool include_second_deadline) {
    int64_t deadline_us = first_deadline_us;

    if (include_second_deadline && second_deadline_us < first_deadline_us) {
        deadline_us = second_deadline_us;
    }
    int64_t remaining_us = deadline_us - now_us;
    uint32_t wait_ms = 1U;

    if (remaining_us > 0) {
        wait_ms = (uint32_t) ((remaining_us + INT64_C(999)) / INT64_C(1000));
    }
    return pdMS_TO_TICKS(wait_ms);
}

static bool stop_requested(void) {
    EventGroupHandle_t event_group = app_resources_event_group();

    return event_group != NULL &&
           (xEventGroupGetBits(event_group) & APP_EVENT_STOP_REQUEST) != 0U;
}

static void acknowledge_and_delete(void) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group, APP_EVENT_BLE_TX_ACK);
    }
    ble_vario_set_tx_wakeup_task(NULL);
    vTaskDelete(NULL);
}

static void block_safe_stop_light_sleep(void) {
    EventGroupHandle_t event_group = app_resources_event_group();

    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group,
                                  APP_EVENT_SAFE_SLEEP_BLOCKED);
    }
}

void ble_tx_worker_task(void *context) {
    EventGroupHandle_t event_group = app_resources_event_group();
    app_config_t config = {0};
    vario_result_t vario = {0};
    system_snapshot_t system = {0};
    int64_t next_battery_update_us = esp_timer_get_time();
    int64_t next_lk8ex1_notify_us = next_battery_update_us;
    uint32_t config_revision = 0U;
    uint32_t previous_config_revision = 0U;
    uint32_t previous_notify_rate_hz = 0U;
    bool config_valid = false;

    (void) context;
    ESP_LOGI(TAG, "started on core %d", xPortGetCoreID());
    ble_vario_set_tx_wakeup_task(xTaskGetCurrentTaskHandle());

    for (;;) {
        int64_t now_us = esp_timer_get_time();
        bool storage_mode_active =
            event_group != NULL &&
            (xEventGroupGetBits(event_group) &
             APP_EVENT_STORAGE_MODE_REQUEST) != 0U;

        if (stop_requested()) {
            esp_err_t ret = ble_vario_stop();

            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "BLE shutdown incomplete: %s",
                         esp_err_to_name(ret));
                block_safe_stop_light_sleep();
            }
            acknowledge_and_delete();
        }

        if (app_resources_copy_config_with_revision(&config,
                                                    &config_revision)) {
            if (config_valid &&
                config_revision != previous_config_revision) {
                esp_err_t tx_power_ret =
                    ble_vario_apply_tx_power(config.bluetooth_tx_power);

                if (tx_power_ret != ESP_OK &&
                    tx_power_ret != ESP_ERR_INVALID_STATE) {
                    ESP_LOGW(TAG, "BLE TX power update failed: %s",
                             esp_err_to_name(tx_power_ret));
                }
            }
            if (config_valid &&
                config.bluetooth_notify_rate_hz != previous_notify_rate_hz) {
                next_lk8ex1_notify_us =
                    now_us + lk8ex1_notify_period_us(
                                 config.bluetooth_notify_rate_hz);
            }
            previous_config_revision = config_revision;
            previous_notify_rate_hz = config.bluetooth_notify_rate_hz;
            config_valid = true;
        }

        if (now_us >= next_battery_update_us) {
            if (!storage_mode_active && app_resources_copy_system(&system)) {
                ble_vario_update_battery(&system);
            }
            next_battery_update_us = advance_periodic_deadline(
                next_battery_update_us, now_us, BATTERY_UPDATE_PERIOD_US);
        }

        bool can_notify = config_valid && ble_vario_can_notify();

        if (can_notify && now_us >= next_lk8ex1_notify_us) {
            if (!storage_mode_active && app_resources_copy_vario(&vario) &&
                ble_vario_can_notify()) {
                esp_err_t ret = ESP_OK;

                (void) app_resources_apply_debug_vario(&vario, now_us);
                ret = ble_vario_notify_lk8ex1(
                    &vario, &system, config.bluetooth_battery_mode);
                if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND &&
                    ret != ESP_ERR_INVALID_STATE) {
                    ESP_LOGW(TAG, "LK8EX1 sentence dropped: %s",
                             esp_err_to_name(ret));
                }
            }
            next_lk8ex1_notify_us = advance_periodic_deadline(
                next_lk8ex1_notify_us, now_us,
                lk8ex1_notify_period_us(config.bluetooth_notify_rate_hz));
        }
        (void) ulTaskNotifyTake(
            pdTRUE,
            deadline_wait_ticks(esp_timer_get_time(),
                                next_battery_update_us,
                                next_lk8ex1_notify_us,
                                can_notify));
    }
}
