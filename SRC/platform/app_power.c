#include "platform/app_power.h"

#include <stddef.h>

#include "esp_clk_tree.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "soc/clk_tree_defs.h"

#if !CONFIG_PM_ENABLE
#error "CloudBaseVario requires ESP-IDF power management"
#endif

#if CONFIG_PM_DFS_INIT_AUTO
#error "CloudBaseVario configures DFS explicitly in app_power_init"
#endif

#if !CONFIG_FREERTOS_USE_TICKLESS_IDLE
#error "CloudBaseVario safe-stop light sleep requires tickless idle"
#endif

#define APP_POWER_MAX_FREQUENCY_MHZ 80
#define APP_POWER_MIN_FREQUENCY_MHZ 40
#define HERTZ_PER_MHZ UINT32_C(1000000)

static const char *TAG = "app_power";
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
static esp_pm_lock_handle_t sensor_cpu_lock = NULL;
static esp_pm_lock_handle_t no_light_sleep_lock = NULL;
static uint32_t last_observed_frequency_mhz = 0U;
static uint32_t observed_frequency_switch_count = 0U;
static uint32_t sensor_lock_acquire_count = 0U;
static uint32_t lock_error_count = 0U;
static uint32_t light_sleep_entry_count = 0U;
static bool configured = false;
static bool fixed_frequency_fallback = false;
static bool sensor_cpu_lock_held = false;
static bool light_sleep_lock_held = false;

static uint32_t read_cpu_frequency_mhz(void) {
    uint32_t frequency_hz = 0U;

    if (esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_CPU, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                                     &frequency_hz) != ESP_OK) {
        return 0U;
    }
    return frequency_hz / HERTZ_PER_MHZ;
}

static void observe_cpu_frequency(void) {
    uint32_t frequency_mhz = read_cpu_frequency_mhz();

    if (frequency_mhz == 0U) {
        return;
    }

    portENTER_CRITICAL(&state_lock);
    if (last_observed_frequency_mhz != 0U && last_observed_frequency_mhz != frequency_mhz) {
        observed_frequency_switch_count++;
    }
    last_observed_frequency_mhz = frequency_mhz;
    portEXIT_CRITICAL(&state_lock);
}

static void record_lock_error(void) {
    portENTER_CRITICAL(&state_lock);
    lock_error_count++;
    portEXIT_CRITICAL(&state_lock);
}

static void release_created_locks(void) {
    if (no_light_sleep_lock != NULL) {
        if (light_sleep_lock_held) {
            (void) esp_pm_lock_release(no_light_sleep_lock);
            light_sleep_lock_held = false;
        }
        (void) esp_pm_lock_delete(no_light_sleep_lock);
        no_light_sleep_lock = NULL;
    }
    if (sensor_cpu_lock != NULL) {
        (void) esp_pm_lock_delete(sensor_cpu_lock);
        sensor_cpu_lock = NULL;
    }
}

static void enter_fixed_frequency_fallback(void) {
    const esp_pm_config_t fallback_config = {
        .max_freq_mhz = APP_POWER_MAX_FREQUENCY_MHZ,
        .min_freq_mhz = APP_POWER_MAX_FREQUENCY_MHZ,
        .light_sleep_enable = false,
    };

    release_created_locks();
    if (esp_pm_configure(&fallback_config) != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure the 80 MHz PM fallback");
    }

    portENTER_CRITICAL(&state_lock);
    configured = true;
    fixed_frequency_fallback = true;
    sensor_cpu_lock_held = false;
    light_sleep_lock_held = false;
    portEXIT_CRITICAL(&state_lock);
    observe_cpu_frequency();
}

#if CONFIG_PM_LIGHT_SLEEP_CALLBACKS
static esp_err_t light_sleep_exit_callback(int64_t sleep_time_us, void *context) {
    (void) sleep_time_us;
    (void) context;
    __atomic_add_fetch(&light_sleep_entry_count, 1U, __ATOMIC_RELAXED);
    return ESP_OK;
}

static esp_pm_sleep_cbs_register_config_t sleep_callbacks = {
    .enter_cb = NULL,
    .exit_cb = light_sleep_exit_callback,
    .enter_cb_user_arg = NULL,
    .exit_cb_user_arg = NULL,
    .enter_cb_prior = 0U,
    .exit_cb_prior = 0U,
};
#endif

esp_err_t app_power_init(void) {
    const esp_pm_config_t power_config = {
        .max_freq_mhz = APP_POWER_MAX_FREQUENCY_MHZ,
        .min_freq_mhz = APP_POWER_MIN_FREQUENCY_MHZ,
        .light_sleep_enable = true,
    };
    esp_err_t ret = ESP_OK;

    portENTER_CRITICAL(&state_lock);
    if (configured) {
        portEXIT_CRITICAL(&state_lock);
        return ESP_ERR_INVALID_STATE;
    }
    portEXIT_CRITICAL(&state_lock);

    last_observed_frequency_mhz = read_cpu_frequency_mhz();
    ret = esp_pm_configure(&power_config);
    if (ret != ESP_OK) {
        record_lock_error();
        enter_fixed_frequency_fallback();
        return ret;
    }

    ret = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "sensor", &sensor_cpu_lock);
    if (ret != ESP_OK) {
        record_lock_error();
        enter_fixed_frequency_fallback();
        return ret;
    }

    ret = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "normal_run", &no_light_sleep_lock);
    if (ret != ESP_OK) {
        record_lock_error();
        enter_fixed_frequency_fallback();
        return ret;
    }

    ret = esp_pm_lock_acquire(no_light_sleep_lock);
    if (ret != ESP_OK) {
        record_lock_error();
        enter_fixed_frequency_fallback();
        return ret;
    }

    portENTER_CRITICAL(&state_lock);
    configured = true;
    light_sleep_lock_held = true;
    portEXIT_CRITICAL(&state_lock);

#if CONFIG_PM_LIGHT_SLEEP_CALLBACKS
    ret = esp_pm_light_sleep_register_cbs(&sleep_callbacks);
    if (ret != ESP_OK) {
        record_lock_error();
        ESP_LOGW(TAG, "light-sleep diagnostics unavailable: %s", esp_err_to_name(ret));
    }
#endif

    observe_cpu_frequency();
    ESP_LOGI(TAG, "DFS configured: max=%d MHz min=%d MHz, normal light sleep blocked",
             APP_POWER_MAX_FREQUENCY_MHZ, APP_POWER_MIN_FREQUENCY_MHZ);
    return ESP_OK;
}

esp_err_t app_power_sensor_work_begin(void) {
    esp_err_t ret = ESP_OK;

    portENTER_CRITICAL(&state_lock);
    if (!configured) {
        portEXIT_CRITICAL(&state_lock);
        record_lock_error();
        return ESP_ERR_INVALID_STATE;
    }
    if (fixed_frequency_fallback) {
        portEXIT_CRITICAL(&state_lock);
        return ESP_OK;
    }
    if (sensor_cpu_lock_held) {
        portEXIT_CRITICAL(&state_lock);
        record_lock_error();
        return ESP_ERR_INVALID_STATE;
    }
    portEXIT_CRITICAL(&state_lock);

    observe_cpu_frequency();
    ret = esp_pm_lock_acquire(sensor_cpu_lock);
    if (ret != ESP_OK) {
        record_lock_error();
        return ret;
    }

    portENTER_CRITICAL(&state_lock);
    sensor_cpu_lock_held = true;
    sensor_lock_acquire_count++;
    portEXIT_CRITICAL(&state_lock);
    observe_cpu_frequency();
    return ESP_OK;
}

esp_err_t app_power_sensor_work_end(void) {
    esp_err_t ret = ESP_OK;

    portENTER_CRITICAL(&state_lock);
    if (!configured) {
        portEXIT_CRITICAL(&state_lock);
        record_lock_error();
        return ESP_ERR_INVALID_STATE;
    }
    if (fixed_frequency_fallback) {
        portEXIT_CRITICAL(&state_lock);
        return ESP_OK;
    }
    if (!sensor_cpu_lock_held) {
        portEXIT_CRITICAL(&state_lock);
        record_lock_error();
        return ESP_ERR_INVALID_STATE;
    }
    portEXIT_CRITICAL(&state_lock);

    observe_cpu_frequency();
    ret = esp_pm_lock_release(sensor_cpu_lock);
    if (ret != ESP_OK) {
        record_lock_error();
        return ret;
    }

    portENTER_CRITICAL(&state_lock);
    sensor_cpu_lock_held = false;
    portEXIT_CRITICAL(&state_lock);
    observe_cpu_frequency();
    return ESP_OK;
}

esp_err_t app_power_enter_safe_stop(void) {
    esp_err_t ret = ESP_OK;

    portENTER_CRITICAL(&state_lock);
    if (!configured) {
        portEXIT_CRITICAL(&state_lock);
        record_lock_error();
        return ESP_ERR_INVALID_STATE;
    }
    if (fixed_frequency_fallback) {
        portEXIT_CRITICAL(&state_lock);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (sensor_cpu_lock_held || !light_sleep_lock_held) {
        portEXIT_CRITICAL(&state_lock);
        record_lock_error();
        return ESP_ERR_INVALID_STATE;
    }
    portEXIT_CRITICAL(&state_lock);

    ret = esp_pm_lock_release(no_light_sleep_lock);
    if (ret != ESP_OK) {
        record_lock_error();
        return ret;
    }

    portENTER_CRITICAL(&state_lock);
    light_sleep_lock_held = false;
    portEXIT_CRITICAL(&state_lock);
    observe_cpu_frequency();
    ESP_LOGI(TAG, "safe-stop automatic light sleep permitted");
    return ESP_OK;
}

esp_err_t app_power_prepare_safe_stop(void) {
    bool initialized;
    bool fallback;
    bool sleep_blocked;
    esp_err_t ret;

    portENTER_CRITICAL(&state_lock);
    initialized = configured;
    fallback = fixed_frequency_fallback;
    sleep_blocked = light_sleep_lock_held;
    portEXIT_CRITICAL(&state_lock);
    if (!initialized) {
        ret = app_power_init();
        if (ret != ESP_OK) {
            return ret;
        }
        sleep_blocked = true;
        fallback = false;
    }
    if (fallback) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!sleep_blocked) {
        return ESP_OK;
    }
    return app_power_enter_safe_stop();
}

esp_err_t app_power_safe_stop_interaction_begin(void) {
    bool initialized;
    bool fallback;
    bool held;
    esp_err_t ret;

    portENTER_CRITICAL(&state_lock);
    initialized = configured;
    fallback = fixed_frequency_fallback;
    held = light_sleep_lock_held;
    portEXIT_CRITICAL(&state_lock);
    if (!initialized || fallback) {
        return ESP_ERR_INVALID_STATE;
    }
    if (held) {
        return ESP_OK;
    }
    ret = esp_pm_lock_acquire(no_light_sleep_lock);
    if (ret != ESP_OK) {
        record_lock_error();
        return ret;
    }
    portENTER_CRITICAL(&state_lock);
    light_sleep_lock_held = true;
    portEXIT_CRITICAL(&state_lock);
    return ESP_OK;
}

esp_err_t app_power_safe_stop_interaction_end(void) {
    bool initialized;
    bool fallback;
    bool held;
    esp_err_t ret;

    portENTER_CRITICAL(&state_lock);
    initialized = configured;
    fallback = fixed_frequency_fallback;
    held = light_sleep_lock_held;
    portEXIT_CRITICAL(&state_lock);
    if (!initialized || fallback) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!held) {
        return ESP_OK;
    }
    ret = esp_pm_lock_release(no_light_sleep_lock);
    if (ret != ESP_OK) {
        record_lock_error();
        return ret;
    }
    portENTER_CRITICAL(&state_lock);
    light_sleep_lock_held = false;
    portEXIT_CRITICAL(&state_lock);
    return ESP_OK;
}

void app_power_get_diagnostics(app_power_diagnostics_t *diagnostics) {
    if (diagnostics == NULL) {
        return;
    }

    observe_cpu_frequency();
    portENTER_CRITICAL(&state_lock);
    diagnostics->current_cpu_frequency_mhz = last_observed_frequency_mhz;
    diagnostics->light_sleep_entry_count =
        __atomic_load_n(&light_sleep_entry_count, __ATOMIC_RELAXED);
    diagnostics->observed_frequency_switch_count = observed_frequency_switch_count;
    diagnostics->sensor_lock_acquire_count = sensor_lock_acquire_count;
    diagnostics->lock_error_count = lock_error_count;
    diagnostics->configured = configured;
    diagnostics->fixed_frequency_fallback = fixed_frequency_fallback;
    diagnostics->sensor_cpu_lock_held = sensor_cpu_lock_held;
    diagnostics->light_sleep_lock_held = light_sleep_lock_held;
    portEXIT_CRITICAL(&state_lock);
}
