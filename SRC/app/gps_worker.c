#include "app/gps_worker.h"

#include <inttypes.h>
#include <string.h>

#include "app/app_events.h"
#include "app/app_resources.h"
#include "domain/gps_nmea.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "platform/board.h"
#include "platform/gps_l96.h"

#define GPS_RETRY_INTERVAL_MS UINT32_C(5000)
#define GPS_READ_TIMEOUT_MS UINT32_C(100)
#define GPS_MIN_STALE_TIMEOUT_MS UINT32_C(3000)

static const char *TAG = "gps_worker";

static bool stop_requested(void) {
    EventGroupHandle_t event_group = app_resources_event_group();

    return event_group != NULL &&
           (xEventGroupGetBits(event_group) & APP_EVENT_STOP_REQUEST) != 0U;
}

static void increment_counter(uint32_t *counter) {
    if (*counter < UINT32_MAX) {
        (*counter)++;
    }
}

static void publish(const gps_snapshot_t *snapshot) {
    EventGroupHandle_t event_group = app_resources_event_group();

    (void) app_resources_publish_gps(snapshot);
    if (event_group == NULL) {
        return;
    }
    if (snapshot->installed && snapshot->fix_valid) {
        (void) xEventGroupSetBits(event_group, APP_EVENT_GPS_FIX_VALID);
    } else {
        (void) xEventGroupClearBits(event_group, APP_EVENT_GPS_FIX_VALID);
    }
}

static bool is_rmc_sentence(const char *line) {
    return strncmp(line, "$GPRMC,", 7U) == 0 ||
           strncmp(line, "$GNRMC,", 7U) == 0;
}

static bool is_gga_sentence(const char *line) {
    return strncmp(line, "$GPGGA,", 7U) == 0 ||
           strncmp(line, "$GNGGA,", 7U) == 0;
}

static void apply_fix(gps_snapshot_t *snapshot,
                      const gps_nmea_fix_t *fix) {
    snapshot->fix_valid = fix->fix_valid;
    snapshot->utc_valid = fix->utc_valid;
    snapshot->position_valid = fix->position_valid;
    snapshot->altitude_valid = fix->altitude_valid;
    snapshot->satellites_valid = fix->satellites_valid;
    snapshot->hdop_valid = fix->hdop_valid;
    snapshot->speed_valid = fix->speed_valid;
    snapshot->course_valid = fix->course_valid;
    snapshot->satellites = fix->satellites;
    snapshot->latitude_deg = fix->latitude_deg;
    snapshot->longitude_deg = fix->longitude_deg;
    snapshot->altitude_m = fix->altitude_m;
    snapshot->hdop = fix->hdop;
    snapshot->speed_kmh = fix->speed_kmh;
    snapshot->course_deg = fix->course_deg;
    memcpy(snapshot->utc, fix->utc, sizeof(snapshot->utc));
}

static void acknowledge_and_delete(void) {
    EventGroupHandle_t event_group = app_resources_event_group();

    gps_l96_deinit();
    if (event_group != NULL) {
        (void) xEventGroupSetBits(event_group, APP_EVENT_GPS_ACK);
    }
    vTaskDelete(NULL);
}

static bool wait_retry_or_stop(void) {
    for (uint32_t elapsed_ms = 0U; elapsed_ms < GPS_RETRY_INTERVAL_MS;
         elapsed_ms += GPS_READ_TIMEOUT_MS) {
        if (stop_requested()) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(GPS_READ_TIMEOUT_MS));
    }
    return true;
}

static uint32_t stale_timeout_ms(uint32_t interval_ms) {
    uint32_t timeout_ms = interval_ms * 3U;

    if (timeout_ms < GPS_MIN_STALE_TIMEOUT_MS) {
        timeout_ms = GPS_MIN_STALE_TIMEOUT_MS;
    }
    return timeout_ms;
}

void gps_worker_task(void *context) {
    const board_identity_t *identity = board_active_identity();
    gps_snapshot_t snapshot = {0};
    app_config_t config = {0};
    uint32_t config_revision = 0U;

    (void) context;
    snapshot.installed = identity != NULL && identity->gps_installed == 1U;
    publish(&snapshot);
    ESP_LOGI(TAG, "started on core %d installed=%d", xPortGetCoreID(),
             snapshot.installed);

    if (!snapshot.installed) {
        while (!stop_requested()) {
            vTaskDelay(pdMS_TO_TICKS(GPS_READ_TIMEOUT_MS));
        }
        acknowledge_and_delete();
    }

    while (!stop_requested()) {
        gps_nmea_pairer_t pairer = {0};
        uint32_t active_interval_ms = 1000U;
        uint32_t baud_rate = 0U;
        int64_t last_sentence_us = 0;
        esp_err_t ret = ESP_OK;

        if (app_resources_copy_config_with_revision(&config,
                                                    &config_revision)) {
            active_interval_ms = config.gps_send_interval_ms;
        }
        ret = gps_l96_connect(active_interval_ms, &baud_rate);
        if (ret != ESP_OK) {
            snapshot.identified = false;
            snapshot.communicating = false;
            snapshot.fix_valid = false;
            snapshot.baud_rate = 0U;
            snapshot.last_error = (int32_t) ret;
            increment_counter(&snapshot.retry_count);
            publish(&snapshot);
            ESP_LOGW(TAG, "L96 identify/configure failed: %s; retrying",
                     esp_err_to_name(ret));
            if (!wait_retry_or_stop()) {
                break;
            }
            continue;
        }

        gps_nmea_pairer_init(&pairer);
        snapshot.identified = true;
        snapshot.communicating = true;
        snapshot.baud_rate = baud_rate;
        snapshot.last_error = ESP_OK;
        publish(&snapshot);
        last_sentence_us = esp_timer_get_time();
        ESP_LOGI(TAG, "L96 configured at %" PRIu32 " bps, period=%" PRIu32
                      " ms", baud_rate, active_interval_ms);

        while (!stop_requested()) {
            char line[GPS_NMEA_SENTENCE_CAPACITY] = {0};
            uint32_t next_revision = config_revision;
            app_config_t next_config = config;
            int64_t now_us = 0;

            if (app_resources_copy_config_with_revision(&next_config,
                                                        &next_revision) &&
                next_revision != config_revision) {
                config = next_config;
                config_revision = next_revision;
                if (config.gps_send_interval_ms != active_interval_ms) {
                    ret = gps_l96_set_interval(config.gps_send_interval_ms);
                    if (ret != ESP_OK) {
                        snapshot.last_error = (int32_t) ret;
                        break;
                    }
                    active_interval_ms = config.gps_send_interval_ms;
                    last_sentence_us = esp_timer_get_time();
                }
            }

            ret = gps_l96_read_line(line, sizeof(line), GPS_READ_TIMEOUT_MS);
            now_us = esp_timer_get_time();
            if (ret == ESP_OK) {
                gps_nmea_result_t nmea_result = GPS_NMEA_IGNORED;
                bool fix_valid = false;
                bool target_sentence = is_rmc_sentence(line) ||
                                       is_gga_sentence(line);

                increment_counter(&snapshot.received_sentence_count);
                if (target_sentence && gps_nmea_checksum_valid(line)) {
                    last_sentence_us = now_us;
                }
                nmea_result = gps_nmea_pairer_consume(
                    &pairer, line, snapshot.rmc, snapshot.gga, &fix_valid);
                if (nmea_result == GPS_NMEA_INVALID) {
                    increment_counter(&snapshot.invalid_sentence_count);
                } else if (nmea_result == GPS_NMEA_PAIR_READY) {
                    gps_nmea_fix_t fix = {0};

                    snapshot.timestamp_us = now_us;
                    snapshot.last_receive_us = now_us;
                    if (gps_nmea_parse_fix_pair(snapshot.rmc, snapshot.gga,
                                                &fix)) {
                        apply_fix(&snapshot, &fix);
                    } else {
                        memset(&fix, 0, sizeof(fix));
                        apply_fix(&snapshot, &fix);
                    }
                    snapshot.rmc_length = (uint16_t) strlen(snapshot.rmc);
                    snapshot.gga_length = (uint16_t) strlen(snapshot.gga);
                    increment_counter(&snapshot.sequence);
                    increment_counter(&snapshot.paired_update_count);
                    snapshot.last_error = ESP_OK;
                    publish(&snapshot);
                }
            } else if (ret == ESP_ERR_INVALID_SIZE) {
                increment_counter(&snapshot.invalid_sentence_count);
            } else if (ret != ESP_ERR_TIMEOUT) {
                snapshot.last_error = (int32_t) ret;
                break;
            }

            if (now_us - last_sentence_us >
                (int64_t) stale_timeout_ms(active_interval_ms) * 1000) {
                snapshot.last_error = ESP_ERR_TIMEOUT;
                break;
            }
        }

        gps_l96_deinit();
        snapshot.communicating = false;
        snapshot.fix_valid = false;
        if (!stop_requested()) {
            increment_counter(&snapshot.retry_count);
        }
        publish(&snapshot);
        if (!stop_requested()) {
            ESP_LOGW(TAG, "L96 communication lost: %s; retrying",
                     esp_err_to_name((esp_err_t) snapshot.last_error));
            if (!wait_retry_or_stop()) {
                break;
            }
        }
    }
    acknowledge_and_delete();
}
