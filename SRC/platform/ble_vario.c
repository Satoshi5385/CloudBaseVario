#include "platform/ble_vario.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#if !CONFIG_BT_NIMBLE_ENABLED
#error "CloudBaseVario requires the ESP-IDF NimBLE Host"
#endif

#if !CONFIG_BT_CTRL_MODEM_SLEEP
#error "CloudBaseVario requires Bluetooth controller modem sleep"
#endif

#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE
#error "CloudBaseVario is BLE-only and does not use software coexistence"
#endif

#define BLE_DEVICE_NAME "CloudBaseVario"
#define BLE_ADVERTISING_INTERVAL_MS UINT32_C(250)
#define BLE_CONNECTION_INTERVAL_MIN_MS UINT32_C(30)
#define BLE_CONNECTION_INTERVAL_MAX_MS UINT32_C(50)
#define BLE_CONNECTION_LATENCY UINT16_C(1)
#define BLE_SUPERVISION_TIMEOUT_MS UINT32_C(4000)

#define NUS_SERVICE_UUID_BYTES                                                                     \
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e
#define NUS_RX_UUID_BYTES                                                                          \
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e
#define NUS_TX_UUID_BYTES                                                                          \
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e

static const char *TAG = "ble_vario";
static const ble_uuid128_t nus_service_uuid = BLE_UUID128_INIT(NUS_SERVICE_UUID_BYTES);
static const ble_uuid128_t nus_rx_uuid = BLE_UUID128_INIT(NUS_RX_UUID_BYTES);
static const ble_uuid128_t nus_tx_uuid = BLE_UUID128_INIT(NUS_TX_UUID_BYTES);

/* BLE connection state is shared between the NimBLE host and system tasks. */
static portMUX_TYPE ble_state_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t connection_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t nus_tx_value_handle = 0U;
static uint8_t own_address_type = 0U;
static bool notification_subscribed = false;
static bool nimble_initialized = false;
static bool stop_requested = false;
static uint32_t sentence_count = 0U;
static uint32_t dropped_sentence_count = 0U;
static int32_t last_notify_error = 0;
static int64_t last_notify_success_us = 0;

static int ble_gap_event_handler(struct ble_gap_event *event, void *context);
static int ble_start_advertising(void);

static void ble_set_connection_state(uint16_t handle, bool subscribed) {
    portENTER_CRITICAL(&ble_state_lock);
    connection_handle = handle;
    notification_subscribed = subscribed;
    if (handle == BLE_HS_CONN_HANDLE_NONE || !subscribed) {
        last_notify_success_us = 0;
    }
    portEXIT_CRITICAL(&ble_state_lock);
}

static bool ble_is_stopping(void) {
    bool stopping = false;

    portENTER_CRITICAL(&ble_state_lock);
    stopping = stop_requested;
    portEXIT_CRITICAL(&ble_state_lock);
    return stopping;
}

static int nus_access_callback(uint16_t connection, uint16_t attribute,
                               struct ble_gatt_access_ctxt *context, void *callback_context) {
    (void) connection;
    (void) attribute;
    (void) callback_context;

    if (context == NULL) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (context->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        /* RX is accepted but intentionally not interpreted as a command. */
        return 0;
    }
    if (context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_chr_def nus_characteristics[] = {
    {
        .uuid = &nus_rx_uuid.u,
        .access_cb = nus_access_callback,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid = &nus_tx_uuid.u,
        .access_cb = nus_access_callback,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &nus_tx_value_handle,
    },
    {0},
};

static const struct ble_gatt_svc_def nus_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_service_uuid.u,
        .characteristics = nus_characteristics,
    },
    {0},
};

static int ble_start_advertising(void) {
    struct ble_hs_adv_fields advertising_fields = {0};
    struct ble_hs_adv_fields scan_response_fields = {0};
    struct ble_gap_adv_params advertising_parameters = {0};
    int rc = 0;

    if (ble_is_stopping()) {
        return BLE_HS_EBUSY;
    }

    advertising_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    advertising_fields.tx_pwr_lvl_is_present = 1U;
    advertising_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    advertising_fields.name = (uint8_t *) BLE_DEVICE_NAME;
    advertising_fields.name_len = (uint8_t) strlen(BLE_DEVICE_NAME);
    advertising_fields.name_is_complete = 1U;

    rc = ble_gap_adv_set_fields(&advertising_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set advertising fields: %d", rc);
        return rc;
    }

    scan_response_fields.uuids128 = (ble_uuid128_t *) &nus_service_uuid;
    scan_response_fields.num_uuids128 = 1U;
    scan_response_fields.uuids128_is_complete = 1U;
    rc = ble_gap_adv_rsp_set_fields(&scan_response_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set scan response: %d", rc);
        return rc;
    }

    advertising_parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
    advertising_parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;
    advertising_parameters.itvl_min = BLE_GAP_ADV_ITVL_MS(BLE_ADVERTISING_INTERVAL_MS);
    advertising_parameters.itvl_max = BLE_GAP_ADV_ITVL_MS(BLE_ADVERTISING_INTERVAL_MS);

    rc = ble_gap_adv_start(own_address_type, NULL, BLE_HS_FOREVER, &advertising_parameters,
                           ble_gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to start advertising: %d", rc);
        return rc;
    }

    ESP_LOGI(TAG, "advertising as %s", BLE_DEVICE_NAME);
    return 0;
}

static void ble_request_connection_parameters(uint16_t handle) {
    struct ble_gap_upd_params parameters = {
        .itvl_min = BLE_GAP_CONN_ITVL_MS(BLE_CONNECTION_INTERVAL_MIN_MS),
        .itvl_max = BLE_GAP_CONN_ITVL_MS(BLE_CONNECTION_INTERVAL_MAX_MS),
        .latency = BLE_CONNECTION_LATENCY,
        .supervision_timeout = BLE_GAP_SUPERVISION_TIMEOUT_MS(BLE_SUPERVISION_TIMEOUT_MS),
        .min_ce_len = 0U,
        .max_ce_len = 0U,
    };
    int rc = ble_gap_update_params(handle, &parameters);

    if (rc != 0) {
        ESP_LOGW(TAG, "connection parameter request failed: %d", rc);
    }
}

static void ble_log_connection_parameters(uint16_t handle) {
    struct ble_gap_conn_desc description = {0};
    int rc = ble_gap_conn_find(handle, &description);

    if (rc == 0) {
        ESP_LOGI(TAG, "connection interval=%u latency=%u timeout=%u", description.conn_itvl,
                 description.conn_latency, description.supervision_timeout);
    }
}

static int ble_gap_event_handler(struct ble_gap_event *event, void *context) {
    (void) context;

    if (event == NULL) {
        return 0;
    }

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ble_set_connection_state(event->connect.conn_handle, false);
            ble_request_connection_parameters(event->connect.conn_handle);
            ESP_LOGI(TAG, "peer connected");
        } else if (!ble_is_stopping()) {
            (void) ble_start_advertising();
        } else {
            /* Shutdown intentionally suppresses re-advertising. */
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ble_set_connection_state(BLE_HS_CONN_HANDLE_NONE, false);
        ESP_LOGI(TAG, "peer disconnected: reason=%d", event->disconnect.reason);
        if (!ble_is_stopping()) {
            (void) ble_start_advertising();
        }
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == nus_tx_value_handle) {
            bool subscribed = event->subscribe.cur_notify != 0U;
            ble_set_connection_state(event->subscribe.conn_handle, subscribed);
            ESP_LOGI(TAG, "NUS notifications enabled=%d", subscribed);
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "ATT MTU=%u", event->mtu.value);
        break;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ble_log_connection_parameters(event->conn_update.conn_handle);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (!ble_is_stopping()) {
            (void) ble_start_advertising();
        }
        break;

    default:
        /* Other GAP events do not affect the initial unencrypted profile. */
        break;
    }

    return 0;
}

static void ble_host_reset(int reason) {
    ESP_LOGE(TAG, "NimBLE host reset: reason=%d", reason);
}

static void ble_host_sync(void) {
    int rc = ble_hs_util_ensure_addr(0);

    if (rc != 0) {
        ESP_LOGE(TAG, "failed to ensure BLE address: %d", rc);
        return;
    }

    rc = ble_hs_id_infer_auto(0, &own_address_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to infer BLE address type: %d", rc);
        return;
    }

    (void) ble_start_advertising();
}

static void ble_host_task(void *context) {
    (void) context;
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_vario_init(void) {
    esp_err_t ret = ESP_OK;
    int rc = 0;

    if (nimble_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to set default BLE TX power to 0 dBm: %s", esp_err_to_name(ret));
    }
    ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to set advertising TX power to 0 dBm: %s", esp_err_to_name(ret));
    }

    ble_hs_cfg.reset_cb = ble_host_reset;
    ble_hs_cfg.sync_cb = ble_host_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    if (rc != 0) {
        (void) nimble_port_deinit();
        return ESP_FAIL;
    }

    rc = ble_gatts_count_cfg(nus_services);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(nus_services);
    }
    if (rc != 0) {
        (void) nimble_port_deinit();
        return ESP_FAIL;
    }

    portENTER_CRITICAL(&ble_state_lock);
    stop_requested = false;
    nimble_initialized = true;
    portEXIT_CRITICAL(&ble_state_lock);

    nimble_port_freertos_init(ble_host_task);
    return ESP_OK;
}

void ble_vario_begin_shutdown(void) {
    uint16_t handle = BLE_HS_CONN_HANDLE_NONE;
    int rc = 0;

    portENTER_CRITICAL(&ble_state_lock);
    if (!nimble_initialized) {
        portEXIT_CRITICAL(&ble_state_lock);
        return;
    }
    stop_requested = true;
    handle = connection_handle;
    notification_subscribed = false;
    portEXIT_CRITICAL(&ble_state_lock);

    rc = ble_gap_adv_stop();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "advertising stop failed: %d", rc);
    }

    if (handle != BLE_HS_CONN_HANDLE_NONE) {
        rc = ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
        if (rc != 0 && rc != BLE_HS_EALREADY) {
            ESP_LOGW(TAG, "disconnect request failed: %d", rc);
        }
    }
}

esp_err_t ble_vario_stop(void) {
    bool initialized = false;
    int rc = 0;

    portENTER_CRITICAL(&ble_state_lock);
    initialized = nimble_initialized;
    portEXIT_CRITICAL(&ble_state_lock);
    if (!initialized) {
        return ESP_OK;
    }

    ble_vario_begin_shutdown();

    rc = nimble_port_stop();
    if (rc != 0) {
        ESP_LOGW(TAG, "NimBLE host stop failed: %d", rc);
        return ESP_FAIL;
    }

    if (nimble_port_deinit() != ESP_OK) {
        ESP_LOGW(TAG, "NimBLE port deinitialization failed");
        return ESP_FAIL;
    }

    ble_set_connection_state(BLE_HS_CONN_HANDLE_NONE, false);
    portENTER_CRITICAL(&ble_state_lock);
    nimble_initialized = false;
    portEXIT_CRITICAL(&ble_state_lock);
    return ESP_OK;
}

bool ble_vario_can_notify(void) {
    bool can_notify = false;

    portENTER_CRITICAL(&ble_state_lock);
    can_notify = nimble_initialized && !stop_requested && notification_subscribed &&
                 connection_handle != BLE_HS_CONN_HANDLE_NONE;
    portEXIT_CRITICAL(&ble_state_lock);
    return can_notify;
}

bool ble_vario_notify_active(void) {
    bool active = false;
    int64_t success_us = 0;
    int64_t now_us = esp_timer_get_time();

    portENTER_CRITICAL(&ble_state_lock);
    success_us = last_notify_success_us;
    active = nimble_initialized && !stop_requested &&
             notification_subscribed &&
             connection_handle != BLE_HS_CONN_HANDLE_NONE &&
             last_notify_error == 0 && success_us > 0;
    portEXIT_CRITICAL(&ble_state_lock);
    return active && now_us >= success_us &&
           now_us - success_us <= INT64_C(500000);
}

static uint8_t lk8ex1_checksum(const char *body) {
    uint8_t checksum = 0U;

    if (body != NULL) {
        for (const unsigned char *cursor = (const unsigned char *) body;
             *cursor != '\0'; cursor++) {
            checksum ^= *cursor;
        }
    }
    return checksum;
}

static void record_notify_result(bool success, int error) {
    portENTER_CRITICAL(&ble_state_lock);
    if (success) {
        if (sentence_count < UINT32_MAX) {
            sentence_count++;
        }
        last_notify_error = 0;
        last_notify_success_us = esp_timer_get_time();
    } else {
        if (dropped_sentence_count < UINT32_MAX) {
            dropped_sentence_count++;
        }
        last_notify_error = error;
    }
    portEXIT_CRITICAL(&ble_state_lock);
}

bool ble_vario_format_lk8ex1_fields(
    const vario_result_t *vario, const system_snapshot_t *system,
    ble_vario_lk8ex1_fields_t *fields) {
    int written = 0;

    if (vario == NULL || system == NULL || fields == NULL) {
        return false;
    }
    memset(fields, 0, sizeof(*fields));
    (void) strcpy(fields->raw_pressure, "999999");
    (void) strcpy(fields->altitude, "99999");
    (void) strcpy(fields->vario, "9999");
    (void) strcpy(fields->temperature, "99");
    (void) strcpy(fields->battery, "999");

    if (vario->pressure_valid) {
        written = snprintf(
            fields->raw_pressure, sizeof(fields->raw_pressure), "%ld",
            (long) lroundf((float) vario->pressure_pa_x100 / 100.0f));
        if (written <= 0 ||
            (size_t) written >= sizeof(fields->raw_pressure)) {
            return false;
        }
    }
    if (vario->climb_rate_valid && isfinite(vario->climb_rate_mps)) {
        written = snprintf(fields->vario, sizeof(fields->vario), "%ld",
                           (long) lroundf(vario->climb_rate_mps * 100.0f));
        if (written <= 0 || (size_t) written >= sizeof(fields->vario)) {
            return false;
        }
    }
    if (system->battery_valid && isfinite(system->battery_voltage_v) &&
        system->battery_voltage_v >= 0.0f) {
        written = snprintf(fields->battery, sizeof(fields->battery), "%.2f",
                           (double) system->battery_voltage_v);
        if (written <= 0 || (size_t) written >= sizeof(fields->battery)) {
            return false;
        }
    }
    fields->sentence_available =
        vario->pressure_valid || vario->climb_rate_valid;
    return true;
}

esp_err_t ble_vario_notify_lk8ex1(const vario_result_t *vario,
                                  const system_snapshot_t *system) {
    char body[96] = {0};
    char sentence[104] = {0};
    ble_vario_lk8ex1_fields_t fields = {0};
    uint16_t handle = BLE_HS_CONN_HANDLE_NONE;
    uint16_t mtu = 23U;
    size_t chunk_size = 20U;
    size_t sentence_length = 0U;
    int written = 0;

    if (!ble_vario_format_lk8ex1_fields(vario, system, &fields)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!fields.sentence_available) {
        return ESP_ERR_NOT_FOUND;
    }

    written = snprintf(body, sizeof(body), "LK8EX1,%s,%s,%s,%s,%s,",
                       fields.raw_pressure, fields.altitude, fields.vario,
                       fields.temperature, fields.battery);
    if (written <= 0 || (size_t) written >= sizeof(body)) {
        return ESP_ERR_INVALID_SIZE;
    }
    written = snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", body,
                       lk8ex1_checksum(body));
    if (written <= 0 || (size_t) written >= sizeof(sentence)) {
        return ESP_ERR_INVALID_SIZE;
    }
    sentence_length = (size_t) written;

    portENTER_CRITICAL(&ble_state_lock);
    if (nimble_initialized && !stop_requested && notification_subscribed) {
        handle = connection_handle;
    }
    portEXIT_CRITICAL(&ble_state_lock);
    if (handle == BLE_HS_CONN_HANDLE_NONE) {
        return ESP_ERR_INVALID_STATE;
    }

    mtu = ble_att_mtu(handle);
    if (mtu > 3U) {
        chunk_size = (size_t) mtu - 3U;
    }
    for (size_t offset = 0U; offset < sentence_length; offset += chunk_size) {
        size_t remaining = sentence_length - offset;
        size_t chunk_length = remaining < chunk_size ? remaining : chunk_size;
        struct os_mbuf *packet =
            ble_hs_mbuf_from_flat(&sentence[offset], (uint16_t) chunk_length);
        int rc = 0;

        if (packet == NULL) {
            record_notify_result(false, BLE_HS_ENOMEM);
            return ESP_ERR_NO_MEM;
        }
        rc = ble_gatts_notify_custom(handle, nus_tx_value_handle, packet);
        if (rc != 0) {
            record_notify_result(false, rc);
            return ESP_FAIL;
        }
    }

    record_notify_result(true, 0);
    return ESP_OK;
}

void ble_vario_get_diagnostics(ble_vario_diagnostics_t *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    portENTER_CRITICAL(&ble_state_lock);
    diagnostics->sentence_count = sentence_count;
    diagnostics->dropped_sentence_count = dropped_sentence_count;
    diagnostics->last_notify_error = last_notify_error;
    diagnostics->last_notify_success_us = last_notify_success_us;
    diagnostics->connected =
        connection_handle != BLE_HS_CONN_HANDLE_NONE;
    diagnostics->subscribed = notification_subscribed;
    portEXIT_CRITICAL(&ble_state_lock);
}
