#include "platform/ble_vario.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "domain/battery_level.h"
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

#if CONFIG_BT_NIMBLE_BAS_SERVICE
#error "CloudBaseVario registers its own Battery Service 1.1"
#endif

#if CONFIG_BT_NIMBLE_MAX_CCCDS < 3
#error "CloudBaseVario requires CCCDs for NUS TX and Battery Service"
#endif

#define BLE_DEVICE_NAME "CloudBaseVario"
#define BLE_ADVERTISING_INTERVAL_MS UINT32_C(250)
#define BLE_CONNECTION_INTERVAL_MIN_MS UINT32_C(30)
#define BLE_CONNECTION_INTERVAL_MAX_MS UINT32_C(50)
#define BLE_CONNECTION_LATENCY UINT16_C(1)
#define BLE_SUPERVISION_TIMEOUT_MS UINT32_C(4000)

#define BATTERY_SERVICE_UUID UINT16_C(0x180F)
#define BATTERY_LEVEL_UUID UINT16_C(0x2A19)
#define BATTERY_LEVEL_STATUS_UUID UINT16_C(0x2BED)
#define BATTERY_POWER_STATE_PRESENT UINT16_C(0x0001)
#define BATTERY_POWER_STATE_WIRED_EXTERNAL UINT16_C(0x0002)
#define BATTERY_POWER_STATE_CHARGING UINT16_C(0x0020)
#define BATTERY_POWER_STATE_DISCHARGING_ACTIVE UINT16_C(0x0040)

_Static_assert((BATTERY_POWER_STATE_PRESENT |
                BATTERY_POWER_STATE_WIRED_EXTERNAL |
                BATTERY_POWER_STATE_CHARGING) == UINT16_C(0x0023),
               "Unexpected charging power-state encoding");
_Static_assert((BATTERY_POWER_STATE_PRESENT |
                BATTERY_POWER_STATE_DISCHARGING_ACTIVE) == UINT16_C(0x0041),
               "Unexpected discharging power-state encoding");

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
static const ble_uuid16_t battery_service_uuid = BLE_UUID16_INIT(BATTERY_SERVICE_UUID);
static const ble_uuid16_t battery_level_uuid = BLE_UUID16_INIT(BATTERY_LEVEL_UUID);
static const ble_uuid16_t battery_level_status_uuid =
    BLE_UUID16_INIT(BATTERY_LEVEL_STATUS_UUID);

/* BLE connection state is shared between the NimBLE host and system tasks. */
static portMUX_TYPE ble_state_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t connection_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t nus_tx_value_handle = 0U;
static uint16_t battery_level_value_handle = 0U;
static uint16_t battery_level_status_value_handle = 0U;
static uint8_t own_address_type = 0U;
static uint8_t battery_level_percent = 0U;
static uint8_t battery_level_status[BLE_VARIO_BATTERY_LEVEL_STATUS_SIZE] = {
    0U,
    (uint8_t) BATTERY_POWER_STATE_DISCHARGING_ACTIVE |
        (uint8_t) BATTERY_POWER_STATE_PRESENT,
    0U,
};
static bool notification_subscribed = false;
static bool nimble_initialized = false;
static bool stop_requested = false;
static app_bluetooth_tx_power_t configured_tx_power =
    APP_BLUETOOTH_TX_POWER_LOW;
static uint32_t sentence_count = 0U;
static uint32_t dropped_sentence_count = 0U;
static int32_t last_notify_error = 0;
static int64_t last_notify_success_us = 0;
static TaskHandle_t tx_wakeup_task;

static int ble_gap_event_handler(struct ble_gap_event *event, void *context);
static int ble_start_advertising(void);

static bool ble_tx_power_level(app_bluetooth_tx_power_t tx_power,
                               esp_power_level_t *level) {
    switch (tx_power) {
    case APP_BLUETOOTH_TX_POWER_MIN:
        if (level != NULL) {
            *level = ESP_PWR_LVL_N24;
        }
        return true;
    case APP_BLUETOOTH_TX_POWER_LOW:
        if (level != NULL) {
            *level = ESP_PWR_LVL_N12;
        }
        return true;
    case APP_BLUETOOTH_TX_POWER_NORMAL:
        if (level != NULL) {
            *level = ESP_PWR_LVL_N0;
        }
        return true;
    case APP_BLUETOOTH_TX_POWER_HIGH:
        if (level != NULL) {
            *level = ESP_PWR_LVL_P9;
        }
        return true;
    default:
        return false;
    }
}

static esp_err_t ble_apply_base_tx_power(
    app_bluetooth_tx_power_t tx_power) {
    esp_power_level_t level = ESP_PWR_LVL_INVALID;
    esp_err_t first_error = ESP_OK;
    esp_err_t ret = ESP_OK;
    int32_t dbm = app_config_bluetooth_tx_power_dbm(tx_power);

    if (!ble_tx_power_level(tx_power, &level)) {
        return ESP_ERR_INVALID_ARG;
    }
    ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, level);
    if (ret != ESP_OK) {
        first_error = ret;
        ESP_LOGW(TAG, "failed to set default BLE TX power to %ld dBm: %s",
                 (long) dbm, esp_err_to_name(ret));
    }
    ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, level);
    if (ret != ESP_OK) {
        if (first_error == ESP_OK) {
            first_error = ret;
        }
        ESP_LOGW(TAG,
                 "failed to set advertising BLE TX power to %ld dBm: %s",
                 (long) dbm, esp_err_to_name(ret));
    }
    return first_error;
}

static esp_err_t ble_apply_connection_tx_power(
    uint16_t handle, app_bluetooth_tx_power_t tx_power) {
    esp_power_level_t level = ESP_PWR_LVL_INVALID;
    esp_err_t ret = ESP_OK;

    if (handle == BLE_HS_CONN_HANDLE_NONE ||
        !ble_tx_power_level(tx_power, &level)) {
        return ESP_ERR_INVALID_ARG;
    }
    ret = esp_ble_tx_power_set_enhanced(
        ESP_BLE_ENHANCED_PWR_TYPE_CONN, handle, level);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG,
                 "failed to set connection BLE TX power to %ld dBm: %s",
                 (long) app_config_bluetooth_tx_power_dbm(tx_power),
                 esp_err_to_name(ret));
    }
    return ret;
}

static void ble_notify_tx_worker(void) {
    portENTER_CRITICAL(&ble_state_lock);
    if (tx_wakeup_task != NULL) {
        xTaskNotifyGive(tx_wakeup_task);
    }
    portEXIT_CRITICAL(&ble_state_lock);
}

static void ble_set_connection_state(uint16_t handle, bool subscribed) {
    portENTER_CRITICAL(&ble_state_lock);
    connection_handle = handle;
    notification_subscribed = subscribed;
    if (handle == BLE_HS_CONN_HANDLE_NONE || !subscribed) {
        last_notify_success_us = 0;
    }
    if (tx_wakeup_task != NULL) {
        xTaskNotifyGive(tx_wakeup_task);
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

static int battery_access_callback(
    uint16_t connection, uint16_t attribute,
    struct ble_gatt_access_ctxt *context, void *callback_context) {
    uint8_t value[BLE_VARIO_BATTERY_LEVEL_STATUS_SIZE] = {0};
    uint16_t uuid = 0U;
    size_t value_size = 0U;

    (void) connection;
    (void) attribute;
    (void) callback_context;

    if (context == NULL || context->chr == NULL) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (context->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    uuid = ble_uuid_u16(context->chr->uuid);
    portENTER_CRITICAL(&ble_state_lock);
    if (uuid == BATTERY_LEVEL_UUID) {
        value[0] = battery_level_percent;
        value_size = 1U;
    } else if (uuid == BATTERY_LEVEL_STATUS_UUID) {
        memcpy(value, battery_level_status, sizeof(battery_level_status));
        value_size = sizeof(battery_level_status);
    }
    portEXIT_CRITICAL(&ble_state_lock);

    if (value_size == 0U) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (os_mbuf_append(context->om, value, (uint16_t) value_size) != 0) {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return 0;
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

static const struct ble_gatt_chr_def battery_characteristics[] = {
    {
        .uuid = &battery_level_uuid.u,
        .access_cb = battery_access_callback,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &battery_level_value_handle,
    },
    {
        .uuid = &battery_level_status_uuid.u,
        .access_cb = battery_access_callback,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &battery_level_status_value_handle,
    },
    {0},
};

static const struct ble_gatt_svc_def battery_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &battery_service_uuid.u,
        .characteristics = battery_characteristics,
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
    scan_response_fields.uuids16 = (ble_uuid16_t *) &battery_service_uuid;
    scan_response_fields.num_uuids16 = 1U;
    scan_response_fields.uuids16_is_complete = 1U;
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
            app_bluetooth_tx_power_t tx_power = APP_BLUETOOTH_TX_POWER_LOW;

            ble_set_connection_state(event->connect.conn_handle, false);
            portENTER_CRITICAL(&ble_state_lock);
            tx_power = configured_tx_power;
            portEXIT_CRITICAL(&ble_state_lock);
            (void) ble_vario_apply_tx_power(tx_power);
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
        } else if (event->subscribe.attr_handle ==
                   battery_level_value_handle) {
            ble_notify_tx_worker();
            ESP_LOGI(TAG, "Battery Level notifications enabled=%d",
                     event->subscribe.cur_notify != 0U);
        } else if (event->subscribe.attr_handle ==
                   battery_level_status_value_handle) {
            ble_notify_tx_worker();
            ESP_LOGI(TAG, "Battery Level Status notifications enabled=%d",
                     event->subscribe.cur_notify != 0U);
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

esp_err_t ble_vario_init(app_bluetooth_tx_power_t tx_power) {
    esp_err_t ret = ESP_OK;
    int rc = 0;

    if (nimble_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        return ret;
    }

    if (!ble_tx_power_level(tx_power, NULL)) {
        (void) nimble_port_deinit();
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&ble_state_lock);
    configured_tx_power = tx_power;
    portEXIT_CRITICAL(&ble_state_lock);
    (void) ble_apply_base_tx_power(tx_power);

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
        rc = ble_gatts_count_cfg(battery_services);
    }
    if (rc == 0) {
        rc = ble_gatts_add_svcs(nus_services);
    }
    if (rc == 0) {
        rc = ble_gatts_add_svcs(battery_services);
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

esp_err_t ble_vario_apply_tx_power(app_bluetooth_tx_power_t tx_power) {
    uint16_t handle = BLE_HS_CONN_HANDLE_NONE;
    bool initialized = false;
    esp_err_t first_error = ESP_OK;
    esp_err_t ret = ESP_OK;

    if (!ble_tx_power_level(tx_power, NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&ble_state_lock);
    configured_tx_power = tx_power;
    initialized = nimble_initialized;
    handle = connection_handle;
    portEXIT_CRITICAL(&ble_state_lock);
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    first_error = ble_apply_base_tx_power(tx_power);
    if (handle != BLE_HS_CONN_HANDLE_NONE) {
        ret = ble_apply_connection_tx_power(handle, tx_power);
        if (first_error == ESP_OK) {
            first_error = ret;
        }
    }
    return first_error;
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
    if (tx_wakeup_task != NULL) {
        xTaskNotifyGive(tx_wakeup_task);
    }
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

void ble_vario_set_tx_wakeup_task(TaskHandle_t task) {
    portENTER_CRITICAL(&ble_state_lock);
    tx_wakeup_task = task;
    portEXIT_CRITICAL(&ble_state_lock);
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

uint8_t ble_vario_battery_level_from_voltage(float battery_voltage_v) {
    return battery_level_percent_from_voltage(battery_voltage_v);
}

void ble_vario_format_battery_level_status(
    bool external_power_present,
    uint8_t status[BLE_VARIO_BATTERY_LEVEL_STATUS_SIZE]) {
    uint16_t power_state = BATTERY_POWER_STATE_PRESENT;

    if (status == NULL) {
        return;
    }
    if (external_power_present) {
        power_state |= BATTERY_POWER_STATE_WIRED_EXTERNAL |
                       BATTERY_POWER_STATE_CHARGING;
    } else {
        power_state |= BATTERY_POWER_STATE_DISCHARGING_ACTIVE;
    }

    status[0] = 0U;
    status[1] = (uint8_t) (power_state & UINT16_C(0x00FF));
    status[2] = (uint8_t) (power_state >> 8U);
}

void ble_vario_update_battery(const system_snapshot_t *system) {
    uint8_t next_status[BLE_VARIO_BATTERY_LEVEL_STATUS_SIZE] = {0};
    uint8_t next_level = 0U;
    uint16_t level_handle = 0U;
    uint16_t status_handle = 0U;
    bool level_changed = false;
    bool level_valid = false;
    bool notify_level = false;
    bool status_changed = false;
    bool notify_status = false;

    if (system == NULL) {
        return;
    }
    ble_vario_format_battery_level_status(
        system->external_power_present, next_status);
    level_valid = system->battery_display_valid &&
                  isfinite(system->battery_display_voltage_v) &&
                  system->battery_display_voltage_v >= 0.0f;
    if (level_valid) {
        next_level =
            ble_vario_battery_level_from_voltage(
                system->battery_display_voltage_v);
    }

    portENTER_CRITICAL(&ble_state_lock);
    if (level_valid) {
        level_changed = battery_level_percent != next_level;
        battery_level_percent = next_level;
    }
    status_changed =
        memcmp(battery_level_status, next_status,
               sizeof(battery_level_status)) != 0;
    if (status_changed) {
        memcpy(battery_level_status, next_status,
               sizeof(battery_level_status));
    }
    notify_level = level_changed && nimble_initialized &&
                   !stop_requested &&
                   battery_level_value_handle != 0U;
    notify_status = status_changed && nimble_initialized &&
                    !stop_requested &&
                    battery_level_status_value_handle != 0U;
    level_handle = battery_level_value_handle;
    status_handle = battery_level_status_value_handle;
    portEXIT_CRITICAL(&ble_state_lock);

    if (notify_level) {
        ble_gatts_chr_updated(level_handle);
    }
    if (notify_status) {
        ble_gatts_chr_updated(status_handle);
    }
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
    app_bluetooth_battery_mode_t battery_mode,
    ble_vario_lk8ex1_fields_t *fields) {
    return lk8ex1_format_fields(vario, system, battery_mode, fields);
}

esp_err_t ble_vario_notify_lk8ex1(const vario_result_t *vario,
                                  const system_snapshot_t *system,
                                  app_bluetooth_battery_mode_t battery_mode) {
    char sentence[LK8EX1_SENTENCE_MAX_LENGTH] = {0};
    ble_vario_lk8ex1_fields_t fields = {0};
    uint16_t handle = BLE_HS_CONN_HANDLE_NONE;
    uint16_t mtu = 23U;
    size_t chunk_size = 20U;
    size_t sentence_length = 0U;

    if (!ble_vario_format_lk8ex1_fields(vario, system, battery_mode,
                                        &fields)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!fields.sentence_available) {
        return ESP_ERR_NOT_FOUND;
    }

    if (!lk8ex1_format_sentence(vario, system, battery_mode, sentence,
                                sizeof(sentence), &sentence_length)) {
        return ESP_ERR_INVALID_SIZE;
    }

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
        size_t chunk_length = chunk_size;
        struct os_mbuf *packet =
            NULL;
        int rc = 0;

        if (remaining < chunk_length) {
            chunk_length = remaining;
        }
        packet = ble_hs_mbuf_from_flat(&sentence[offset],
                                       (uint16_t) chunk_length);

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
