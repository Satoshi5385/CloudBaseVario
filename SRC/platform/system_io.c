#include "platform/system_io.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "platform/board.h"

#define BATTERY_SAMPLE_COUNT UINT32_C(5)
#define BATTERY_ADC_MAX_RAW INT32_C(4095)
#define MILLIVOLTS_PER_VOLT 1000.0f

static const char *TAG = "system_io";

/* ADC handles are created once and subsequently owned by system_task. */
static adc_oneshot_unit_handle_t battery_adc_handle = NULL;
static adc_cali_handle_t battery_calibration_handle = NULL;
static adc_unit_t battery_adc_unit = ADC_UNIT_1;
static adc_channel_t battery_adc_channel = ADC_CHANNEL_0;
static bool battery_calibration_ready = false;
static int32_t battery_mv_history[BATTERY_SAMPLE_COUNT] = {0};
static uint32_t battery_history_count = 0U;
static uint32_t battery_history_next = 0U;
static system_io_battery_diagnostics_t battery_diagnostics = {0};

static void sort_samples(int32_t samples[BATTERY_SAMPLE_COUNT]) {
    for (uint32_t index = 1U; index < BATTERY_SAMPLE_COUNT; index++) {
        int32_t value = samples[index];
        uint32_t insert_at = index;

        while (insert_at > 0U && samples[insert_at - 1U] > value) {
            samples[insert_at] = samples[insert_at - 1U];
            insert_at--;
        }
        samples[insert_at] = value;
    }
}

static void clear_battery_history(void) {
    memset(battery_mv_history, 0, sizeof(battery_mv_history));
    battery_history_count = 0U;
    battery_history_next = 0U;
    battery_diagnostics.valid_sample_count = 0U;
}

static void increment_counter(uint32_t *counter) {
    if (counter != NULL && *counter < UINT32_MAX) {
        (*counter)++;
    }
}

esp_err_t system_io_init(void) {
    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t ret = ESP_OK;
    int discarded_raw = 0;

    clear_battery_history();
    memset(&battery_diagnostics, 0, sizeof(battery_diagnostics));

    ret = adc_oneshot_io_to_channel(PIN_BAT_ADC, &battery_adc_unit, &battery_adc_channel);
    if (ret != ESP_OK || battery_adc_unit != ADC_UNIT_1) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = adc_oneshot_new_unit(&unit_config, &battery_adc_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = adc_oneshot_config_channel(battery_adc_handle, battery_adc_channel, &channel_config);
    if (ret != ESP_OK) {
        return ret;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t calibration_config = {
        .unit_id = battery_adc_unit,
        .chan = battery_adc_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_curve_fitting(&calibration_config, &battery_calibration_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC calibration unavailable: %s", esp_err_to_name(ret));
        battery_calibration_ready = false;
        return ret;
    }
    battery_calibration_ready = true;
#else
    ESP_LOGW(TAG, "ADC curve-fitting calibration is not supported");
    battery_calibration_ready = false;
    return ESP_ERR_NOT_SUPPORTED;
#endif

    /* The high-impedance divider requires discarding the first configured read. */
    ret = adc_oneshot_read(battery_adc_handle, battery_adc_channel, &discarded_raw);
    if (ret != ESP_OK) {
        battery_calibration_ready = false;
        return ret;
    }

    return ESP_OK;
}

bool system_io_read_battery_voltage(float *battery_voltage_v) {
    int32_t samples[BATTERY_SAMPLE_COUNT] = {0};
    int raw = 0;
    int calibrated_mv = 0;
    float converted_voltage_v = 0.0f;
    esp_err_t ret = ESP_OK;

    if (battery_voltage_v == NULL || battery_adc_handle == NULL ||
        battery_calibration_handle == NULL || !battery_calibration_ready) {
        increment_counter(&battery_diagnostics.error_count);
        clear_battery_history();
        return false;
    }

    ret = adc_oneshot_read(battery_adc_handle, battery_adc_channel, &raw);
    battery_diagnostics.last_raw = raw;
    if (ret != ESP_OK || raw < 0) {
        increment_counter(&battery_diagnostics.error_count);
        clear_battery_history();
        return false;
    }
    if (raw >= BATTERY_ADC_MAX_RAW) {
        increment_counter(&battery_diagnostics.saturation_count);
        clear_battery_history();
        return false;
    }
    ret = adc_cali_raw_to_voltage(
        battery_calibration_handle, raw, &calibrated_mv);
    if (ret != ESP_OK || calibrated_mv < 0) {
        increment_counter(&battery_diagnostics.error_count);
        clear_battery_history();
        return false;
    }

    battery_diagnostics.last_calibrated_mv = calibrated_mv;
    battery_mv_history[battery_history_next] = calibrated_mv;
    battery_history_next = (battery_history_next + 1U) % BATTERY_SAMPLE_COUNT;
    if (battery_history_count < BATTERY_SAMPLE_COUNT) {
        battery_history_count++;
    }
    battery_diagnostics.valid_sample_count = battery_history_count;
    if (battery_history_count < BATTERY_SAMPLE_COUNT) {
        return false;
    }

    memcpy(samples, battery_mv_history, sizeof(samples));
    sort_samples(samples);
    converted_voltage_v =
        ((float) samples[2] / MILLIVOLTS_PER_VOLT) * BAT_ADC_SCALE * BAT_ADC_GAIN_CORRECTION +
        BAT_ADC_OFFSET_V;
    if (!isfinite(converted_voltage_v) || converted_voltage_v < 0.0f) {
        increment_counter(&battery_diagnostics.error_count);
        clear_battery_history();
        return false;
    }

    *battery_voltage_v = converted_voltage_v;
    return true;
}

void system_io_get_battery_diagnostics(
    system_io_battery_diagnostics_t *diagnostics) {
    if (diagnostics != NULL) {
        *diagnostics = battery_diagnostics;
    }
}

bool system_io_external_power_present(void) {
    return gpio_get_level(PIN_PWR_EXT) != 0;
}

bool system_io_sw2_pressed(void) {
    return gpio_get_level(PIN_SW_2) == 0;
}

bool system_io_sw3_pressed(void) {
    return gpio_get_level(PIN_SW_3) == 0;
}
