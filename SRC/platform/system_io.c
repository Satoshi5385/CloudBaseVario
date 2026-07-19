#include "platform/system_io.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

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

static void sort_samples(int samples[BATTERY_SAMPLE_COUNT]) {
    for (uint32_t index = 1U; index < BATTERY_SAMPLE_COUNT; index++) {
        int value = samples[index];
        uint32_t insert_at = index;

        while (insert_at > 0U && samples[insert_at - 1U] > value) {
            samples[insert_at] = samples[insert_at - 1U];
            insert_at--;
        }
        samples[insert_at] = value;
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
    int samples[BATTERY_SAMPLE_COUNT] = {0};
    int calibrated_mv = 0;
    float converted_voltage_v = 0.0f;

    if (battery_voltage_v == NULL || battery_adc_handle == NULL ||
        battery_calibration_handle == NULL || !battery_calibration_ready) {
        return false;
    }

    for (uint32_t index = 0U; index < BATTERY_SAMPLE_COUNT; index++) {
        esp_err_t ret = adc_oneshot_read(battery_adc_handle, battery_adc_channel, &samples[index]);
        if (ret != ESP_OK || samples[index] < 0 || samples[index] >= BATTERY_ADC_MAX_RAW) {
            return false;
        }
    }

    sort_samples(samples);
    if (adc_cali_raw_to_voltage(battery_calibration_handle, samples[2], &calibrated_mv) != ESP_OK) {
        return false;
    }

    converted_voltage_v =
        ((float) calibrated_mv / MILLIVOLTS_PER_VOLT) * BAT_ADC_SCALE * BAT_ADC_GAIN_CORRECTION +
        BAT_ADC_OFFSET_V;
    if (!isfinite(converted_voltage_v) || converted_voltage_v < 0.0f) {
        return false;
    }

    *battery_voltage_v = converted_voltage_v;
    return true;
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
