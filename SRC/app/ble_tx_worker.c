#include "app/ble_tx_worker.h"

#include <inttypes.h>

#include "app/app_events.h"
#include "app/app_resources.h"
#include "domain/ble_nus_tx.h"
#include "domain/lk8ex1.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "platform/ble_vario.h"

#define BATTERY_UPDATE_PERIOD_US INT64_C(1000000)
#define MICROSECONDS_PER_SECOND INT64_C(1000000)
#define DEFAULT_CONNECTION_INTERVAL_US UINT32_C(50000)

static const char *TAG = "ble_tx_worker";

static int64_t lk8ex1_notify_period_us(uint32_t rate_hz) {
    return MICROSECONDS_PER_SECOND / (int64_t) rate_hz;
}

static int64_t gps_notify_period_us(uint32_t interval_ms) {
    return (int64_t) interval_ms * INT64_C(1000);
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

static int64_t earlier_deadline(int64_t first_us, int64_t second_us) {
    if (second_us < first_us) {
        return second_us;
    }
    return first_us;
}

static TickType_t deadline_wait_ticks(int64_t now_us,
                                      int64_t deadline_us) {
    int64_t remaining_us = deadline_us - now_us;
    uint32_t wait_ms = 1U;

    if (remaining_us > 0) {
        wait_ms = (uint32_t) ((remaining_us + INT64_C(999)) /
                              INT64_C(1000));
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

static void publish_diagnostics(const ble_nus_tx_state_t *state,
                                int64_t last_success_us) {
    ble_nus_tx_statistics_t statistics = {0};

    ble_nus_tx_get_statistics(state, &statistics);
    ble_vario_publish_nus_tx_diagnostics(&statistics, last_success_us);
}

static bool tx_progress_completed(ble_nus_tx_progress_t progress) {
    return progress == BLE_NUS_TX_PROGRESS_LK8EX1_COMPLETE ||
           progress == BLE_NUS_TX_PROGRESS_GPS_COMPLETE;
}

static void offer_lk8ex1(ble_nus_tx_state_t *tx_state,
                         vario_result_t *vario,
                         const system_snapshot_t *system,
                         const app_config_t *config,
                         int64_t now_us) {
    char sentence[LK8EX1_SENTENCE_MAX_LENGTH] = {0};
    size_t length = 0U;

    if (!app_resources_copy_vario(vario)) {
        return;
    }
    (void) app_resources_apply_debug_vario(vario, now_us);
    if (lk8ex1_format_sentence(
            vario, system, config->bluetooth_battery_mode,
            sentence, sizeof(sentence), &length)) {
        (void) ble_nus_tx_offer_lk8ex1(tx_state, sentence, length);
    }
}

static void offer_gps(ble_nus_tx_state_t *tx_state) {
    gps_snapshot_t gps = {0};

    if (!app_resources_copy_gps(&gps) || !gps.communicating ||
        gps.sequence == 0U) {
        return;
    }
    (void) ble_nus_tx_offer_gps(
        tx_state, gps.sequence, gps.rmc, gps.rmc_length,
        gps.gga, gps.gga_length);
}

static void send_available_fragments(ble_nus_tx_state_t *tx_state,
                                     const ble_vario_nus_link_t *link,
                                     int64_t *last_success_us) {
    while (ble_nus_tx_has_token(tx_state) &&
           ble_nus_tx_has_work(tx_state)) {
        ble_nus_tx_fragment_t fragment = {0};
        ble_nus_tx_rejection_t rejection = {0};
        ble_nus_tx_progress_t progress = BLE_NUS_TX_PROGRESS_NONE;
        esp_err_t ret = ESP_OK;
        int32_t nimble_error = 0;
        int32_t diagnostic_error = 0;

        if (!ble_nus_tx_next_fragment(
                tx_state, link->payload_capacity, &fragment)) {
            break;
        }
        if (!ble_nus_tx_take_token(tx_state)) {
            break;
        }
        ret = ble_vario_notify_nus_fragment(
            link->generation, fragment.data, fragment.length,
            &nimble_error);
        if (ret == ESP_OK) {
            progress = ble_nus_tx_accept_fragment(tx_state);
            if (tx_progress_completed(progress)) {
                *last_success_us = esp_timer_get_time();
            }
            continue;
        }

        diagnostic_error = nimble_error;
        if (diagnostic_error == 0) {
            diagnostic_error = (int32_t) ret;
        }
        rejection = ble_nus_tx_reject_fragment(
            tx_state, diagnostic_error);
        if (ret == ESP_ERR_NO_MEM) {
            ble_nus_tx_defer_until_refill(tx_state);
        } else if (ret == ESP_ERR_INVALID_STATE) {
            ble_nus_tx_reset_link(tx_state);
            *last_success_us = 0;
        } else if (fragment.resync || rejection.partial_sentence) {
            esp_err_t reset_ret =
                ble_vario_reset_nus_connection(link->generation);

            if (reset_ret != ESP_OK &&
                reset_ret != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "BLE stream reset failed: %s",
                         esp_err_to_name(reset_ret));
            }
            ble_nus_tx_reset_link(tx_state);
            *last_success_us = 0;
        }
        ESP_LOGW(TAG,
                 "NUS fragment rejected: source=%d length=%u error=%" PRId32,
                 (int) fragment.source, (unsigned int) fragment.length,
                 diagnostic_error);
        break;
    }
}

void ble_tx_worker_task(void *context) {
    EventGroupHandle_t event_group = app_resources_event_group();
    app_config_t config = {0};
    vario_result_t vario = {0};
    system_snapshot_t system = {0};
    ble_nus_tx_state_t tx_state = {0};
    ble_vario_nus_link_t link = {0};
    int64_t next_battery_update_us = esp_timer_get_time();
    int64_t next_lk8ex1_notify_us = next_battery_update_us;
    int64_t next_gps_notify_us = next_battery_update_us;
    int64_t last_notify_success_us = 0;
    uint32_t config_revision = 0U;
    uint32_t previous_config_revision = 0U;
    uint32_t previous_notify_rate_hz = 0U;
    uint32_t previous_gps_interval_ms = 0U;
    uint32_t previous_link_generation = 0U;
    uint32_t previous_connection_interval_us = 0U;
    bool config_valid = false;
    bool previous_link_usable = false;

    (void) context;
    ble_nus_tx_init(&tx_state);
    ESP_LOGI(TAG, "started on core %d", xPortGetCoreID());
    ble_vario_set_tx_wakeup_task(xTaskGetCurrentTaskHandle());

    for (;;) {
        int64_t now_us = esp_timer_get_time();
        bool storage_mode_active =
            event_group != NULL &&
            (xEventGroupGetBits(event_group) &
             APP_EVENT_STORAGE_MODE_REQUEST) != 0U;
        bool link_usable = false;
        int64_t next_wakeup_us = next_battery_update_us;

        if (stop_requested()) {
            esp_err_t ret = ESP_OK;

            publish_diagnostics(&tx_state, last_notify_success_us);
            ret = ble_vario_stop();
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
                config.bluetooth_notify_rate_hz !=
                    previous_notify_rate_hz) {
                next_lk8ex1_notify_us =
                    now_us + lk8ex1_notify_period_us(
                                 config.bluetooth_notify_rate_hz);
            }
            if (config_valid &&
                config.gps_send_interval_ms != previous_gps_interval_ms) {
                next_gps_notify_us =
                    now_us + gps_notify_period_us(
                                 config.gps_send_interval_ms);
            }
            previous_config_revision = config_revision;
            previous_notify_rate_hz = config.bluetooth_notify_rate_hz;
            previous_gps_interval_ms = config.gps_send_interval_ms;
            config_valid = true;
        }

        if (now_us >= next_battery_update_us) {
            if (!storage_mode_active &&
                app_resources_copy_system(&system)) {
                ble_vario_update_battery(&system);
            }
            next_battery_update_us = advance_periodic_deadline(
                next_battery_update_us, now_us,
                BATTERY_UPDATE_PERIOD_US);
        }

        ble_vario_get_nus_link(&link);
        if (link.connection_interval_us == 0U) {
            link.connection_interval_us =
                DEFAULT_CONNECTION_INTERVAL_US;
        }
        link_usable = config_valid && link.connected && link.subscribed &&
                      !storage_mode_active;
        if (link.generation != previous_link_generation) {
            ble_nus_tx_reset_link(&tx_state);
            last_notify_success_us = 0;
        } else if (previous_link_usable && !link_usable) {
            ble_nus_tx_suspend_stream(&tx_state);
            last_notify_success_us = 0;
        }
        if (link_usable &&
            (!previous_link_usable ||
             link.connection_interval_us !=
                 previous_connection_interval_us)) {
            ble_nus_tx_reset_budget(
                &tx_state, now_us, link.connection_interval_us);
        }
        previous_link_generation = link.generation;
        previous_connection_interval_us =
            link.connection_interval_us;
        previous_link_usable = link_usable;

        if (link_usable && now_us >= next_lk8ex1_notify_us) {
            offer_lk8ex1(&tx_state, &vario, &system, &config, now_us);
            next_lk8ex1_notify_us = advance_periodic_deadline(
                next_lk8ex1_notify_us, now_us,
                lk8ex1_notify_period_us(
                    config.bluetooth_notify_rate_hz));
        }
        if (link_usable && now_us >= next_gps_notify_us) {
            offer_gps(&tx_state);
            next_gps_notify_us = advance_periodic_deadline(
                next_gps_notify_us, now_us,
                gps_notify_period_us(config.gps_send_interval_ms));
        }
        if (link_usable) {
            ble_nus_tx_refill_budget(
                &tx_state, now_us, link.connection_interval_us);
            send_available_fragments(
                &tx_state, &link, &last_notify_success_us);
        }
        publish_diagnostics(&tx_state, last_notify_success_us);

        if (link_usable) {
            next_wakeup_us = earlier_deadline(
                next_wakeup_us, next_lk8ex1_notify_us);
            next_wakeup_us = earlier_deadline(
                next_wakeup_us, next_gps_notify_us);
            if (ble_nus_tx_has_work(&tx_state) &&
                !ble_nus_tx_has_token(&tx_state)) {
                next_wakeup_us = earlier_deadline(
                    next_wakeup_us,
                    ble_nus_tx_next_refill_us(&tx_state));
            }
        }
        (void) ulTaskNotifyTake(
            pdTRUE,
            deadline_wait_ticks(esp_timer_get_time(), next_wakeup_us));
    }
}
