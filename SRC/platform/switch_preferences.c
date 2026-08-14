#include "platform/switch_preferences.h"

#include <stddef.h>
#include <string.h>

#include "domain/app_config.h"
#include "nvs.h"

#define SWITCH_PREFERENCES_NAMESPACE "switch_pref"
#define SWITCH_PREFERENCES_KEY "state"
#define SWITCH_PREFERENCES_FORMAT_VERSION UINT8_C(2)
#define SWITCH_PREFERENCES_PAYLOAD_SIZE 5U
#define SWITCH_PREFERENCES_LEGACY_FORMAT_VERSION UINT8_C(1)
#define SWITCH_PREFERENCES_LEGACY_PAYLOAD_SIZE 4U

static switch_preferences_diagnostics_t preference_diagnostics = {
    .source = SWITCH_PREFERENCES_SOURCE_DEFAULT,
    .load_result = SWITCH_PREFERENCES_LOAD_NOT_FOUND,
    .last_load_error = ESP_ERR_NVS_NOT_FOUND,
    .last_save_result = ESP_OK,
    .last_clear_result = ESP_OK,
};

static bool preferences_valid(const switch_preferences_t *preferences) {
    return preferences != NULL &&
           preferences->volume_level >= AUDIO_VOLUME_SMALL &&
           preferences->volume_level <= AUDIO_VOLUME_MUTE &&
           preferences->parameter_number >= APP_CONFIG_PROFILE_MIN_NUMBER &&
           preferences->parameter_number <= APP_CONFIG_PROFILE_MAX_NUMBER;
}

void switch_preferences_set_defaults(switch_preferences_t *preferences) {
    if (preferences == NULL) {
        return;
    }
    preferences->volume_level = AUDIO_VOLUME_SMALL;
    preferences->sink_enabled = true;
    preferences->parameter_number = APP_CONFIG_PROFILE_MIN_NUMBER;
}

switch_preferences_load_result_t switch_preferences_load(
    switch_preferences_t *preferences) {
    nvs_handle_t handle = 0;
    uint8_t payload[SWITCH_PREFERENCES_PAYLOAD_SIZE] = {0};
    size_t payload_size = 0U;
    esp_err_t ret = ESP_OK;
    switch_preferences_load_result_t result =
        SWITCH_PREFERENCES_LOAD_IO_ERROR;

    if (preferences == NULL) {
        return SWITCH_PREFERENCES_LOAD_IO_ERROR;
    }
    switch_preferences_set_defaults(preferences);
    preference_diagnostics.source = SWITCH_PREFERENCES_SOURCE_DEFAULT;
    ret = nvs_open(SWITCH_PREFERENCES_NAMESPACE, NVS_READONLY, &handle);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        result = SWITCH_PREFERENCES_LOAD_NOT_FOUND;
        goto done;
    }
    if (ret != ESP_OK) {
        result = SWITCH_PREFERENCES_LOAD_IO_ERROR;
        goto done;
    }
    ret = nvs_get_blob(handle, SWITCH_PREFERENCES_KEY, NULL,
                       &payload_size);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        result = SWITCH_PREFERENCES_LOAD_NOT_FOUND;
        goto done;
    }
    if (ret != ESP_OK) {
        result = SWITCH_PREFERENCES_LOAD_IO_ERROR;
        goto done;
    }
    if (payload_size != sizeof(payload) &&
        payload_size != SWITCH_PREFERENCES_LEGACY_PAYLOAD_SIZE) {
        result = SWITCH_PREFERENCES_LOAD_INVALID_SIZE;
        goto done;
    }
    ret = nvs_get_blob(handle, SWITCH_PREFERENCES_KEY, payload,
                       &payload_size);
    if (ret != ESP_OK) {
        result = SWITCH_PREFERENCES_LOAD_IO_ERROR;
        goto done;
    }
    if (payload_size == SWITCH_PREFERENCES_LEGACY_PAYLOAD_SIZE &&
        payload[0] == SWITCH_PREFERENCES_LEGACY_FORMAT_VERSION) {
        if (payload[1] > (uint8_t) AUDIO_VOLUME_MUTE ||
            payload[2] > 1U || payload[3] != 0U) {
            result = SWITCH_PREFERENCES_LOAD_INVALID_VALUE;
            goto done;
        }
        preferences->volume_level = (audio_volume_level_t) payload[1];
        preferences->sink_enabled = payload[2] != 0U;
        preference_diagnostics.source = SWITCH_PREFERENCES_SOURCE_NVS;
        result = SWITCH_PREFERENCES_LOAD_LEGACY;
        goto done;
    }
    if (payload[0] != SWITCH_PREFERENCES_FORMAT_VERSION) {
        result = SWITCH_PREFERENCES_LOAD_UNSUPPORTED_VERSION;
        goto done;
    }
    if (payload[1] > (uint8_t) AUDIO_VOLUME_MUTE || payload[2] > 1U ||
        payload[3] < APP_CONFIG_PROFILE_MIN_NUMBER ||
        payload[3] > APP_CONFIG_PROFILE_MAX_NUMBER || payload[4] != 0U) {
        result = SWITCH_PREFERENCES_LOAD_INVALID_VALUE;
        goto done;
    }
    preferences->volume_level = (audio_volume_level_t) payload[1];
    preferences->sink_enabled = payload[2] != 0U;
    preferences->parameter_number = payload[3];
    preference_diagnostics.source = SWITCH_PREFERENCES_SOURCE_NVS;
    result = SWITCH_PREFERENCES_LOAD_OK;

done:
    if (handle != 0) {
        nvs_close(handle);
    }
    preference_diagnostics.load_result = result;
    preference_diagnostics.last_load_error = ret;
    if (result != SWITCH_PREFERENCES_LOAD_OK &&
        result != SWITCH_PREFERENCES_LOAD_LEGACY &&
        result != SWITCH_PREFERENCES_LOAD_NOT_FOUND) {
        preference_diagnostics.load_error_count++;
    }
    return result;
}

esp_err_t switch_preferences_save(const switch_preferences_t *preferences) {
    nvs_handle_t handle = 0;
    uint8_t payload[SWITCH_PREFERENCES_PAYLOAD_SIZE] = {
        SWITCH_PREFERENCES_FORMAT_VERSION, 0U, 0U, 0U, 0U};
    esp_err_t ret = ESP_OK;

    if (!preferences_valid(preferences)) {
        ret = ESP_ERR_INVALID_ARG;
        goto done;
    }
    payload[1] = (uint8_t) preferences->volume_level;
    payload[2] = 0U;
    if (preferences->sink_enabled) {
        payload[2] = 1U;
    }
    payload[3] = preferences->parameter_number;
    ret = nvs_open(SWITCH_PREFERENCES_NAMESPACE, NVS_READWRITE, &handle);
    if (ret == ESP_OK) {
        ret = nvs_set_blob(handle, SWITCH_PREFERENCES_KEY, payload,
                           sizeof(payload));
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }

done:
    if (handle != 0) {
        nvs_close(handle);
    }
    preference_diagnostics.last_save_result = ret;
    if (ret == ESP_OK) {
        preference_diagnostics.save_count++;
    } else {
        preference_diagnostics.save_error_count++;
    }
    return ret;
}

esp_err_t switch_preferences_clear(void) {
    nvs_handle_t handle = 0;
    esp_err_t ret = nvs_open(SWITCH_PREFERENCES_NAMESPACE, NVS_READWRITE,
                             &handle);

    if (ret == ESP_OK) {
        ret = nvs_erase_key(handle, SWITCH_PREFERENCES_KEY);
        if (ret == ESP_ERR_NVS_NOT_FOUND) {
            ret = ESP_OK;
        } else if (ret == ESP_OK) {
            ret = nvs_commit(handle);
        }
    }
    if (handle != 0) {
        nvs_close(handle);
    }
    preference_diagnostics.last_clear_result = ret;
    if (ret != ESP_OK) {
        preference_diagnostics.clear_error_count++;
    } else {
        preference_diagnostics.source = SWITCH_PREFERENCES_SOURCE_DEFAULT;
        preference_diagnostics.load_result =
            SWITCH_PREFERENCES_LOAD_NOT_FOUND;
        preference_diagnostics.last_load_error = ESP_ERR_NVS_NOT_FOUND;
    }
    return ret;
}

void switch_preferences_get_diagnostics(
    switch_preferences_diagnostics_t *diagnostics) {
    if (diagnostics != NULL) {
        *diagnostics = preference_diagnostics;
    }
}

const char *switch_preferences_source_name(
    switch_preferences_source_t source) {
    if (source == SWITCH_PREFERENCES_SOURCE_NVS) {
        return "NVS";
    }
    return "DEFAULT";
}

const char *switch_preferences_load_result_name(
    switch_preferences_load_result_t result) {
    switch (result) {
    case SWITCH_PREFERENCES_LOAD_OK:
        return "OK";
    case SWITCH_PREFERENCES_LOAD_LEGACY:
        return "LEGACY";
    case SWITCH_PREFERENCES_LOAD_NOT_FOUND:
        return "NOT_FOUND";
    case SWITCH_PREFERENCES_LOAD_INVALID_SIZE:
        return "INVALID_SIZE";
    case SWITCH_PREFERENCES_LOAD_UNSUPPORTED_VERSION:
        return "UNSUPPORTED_VERSION";
    case SWITCH_PREFERENCES_LOAD_INVALID_VALUE:
        return "INVALID_VALUE";
    case SWITCH_PREFERENCES_LOAD_IO_ERROR:
    default:
        return "IO_ERROR";
    }
}
