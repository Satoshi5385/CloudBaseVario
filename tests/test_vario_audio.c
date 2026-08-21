#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "domain/vario_audio.h"

static app_config_t test_config(void) {
    app_config_t config = {0};

    app_config_set_defaults(&config);
    config.audio_enabled = true;
    config.sink_enabled = true;
    config.audio_climb_rate_average_s = 0.0f;
    config.lift_start_mps = 0.20f;
    config.lift_end_mps = 0.01f;
    config.sink_start_mps = -1.0f;
    config.sink_end_mps = -0.8f;
    config.audio_state_hold_ms = 0U;
    config.lift_freq_base_hz = 600U;
    config.lift_freq_rate_hz_per_mps = 100.0f;
    config.lift_freq_max_hz = 1800U;
    config.predictive_interval_ms = 500U;
    config.predictive_duration_ms = 50U;
    config.predictive_min_mps = 0.0f;
    return config;
}

static bool has_parameter(const char *name) {
    for (size_t index = 0U; index < app_config_parameter_count(); index++) {
        app_parameter_info_t info = {0};

        if (app_config_parameter_info(index, &info) &&
            strcmp(info.name, name) == 0) {
            return true;
        }
    }
    return false;
}

static vario_audio_command_t step_at(vario_audio_state_t *state,
                                     const app_config_t *config,
                                     int64_t timestamp_us,
                                     float climb_rate_mps,
                                     bool valid,
                                     bool debug_input_active) {
    vario_result_t result = {0};
    vario_audio_command_t command = {0};

    result.timestamp_us = timestamp_us;
    result.climb_rate_mps = climb_rate_mps;
    result.climb_rate_valid = valid;
    result.debug_input_active = debug_input_active;
    vario_audio_step(state, config, &result, timestamp_us, &command);
    return command;
}

static void test_parameter_contract(void) {
    app_config_t config = {0};
    size_t shared_count = 0U;
    size_t profile_count = 0U;

    app_config_set_defaults(&config);
    assert(app_config_parameter_count() == 32U);
    assert(config.auto_power_off_minutes == 60U);
    assert(config.bluetooth_battery_mode ==
           APP_BLUETOOTH_BATTERY_MODE_VOLTAGE);
    assert(config.bluetooth_tx_power == APP_BLUETOOTH_TX_POWER_LOW);
    assert(config.bluetooth_notify_rate_hz == 10U);
    assert(config.audio_climb_rate_average_s == 1.0f);
    assert(config.predictive_interval_ms == 1000U);
    assert(config.predictive_duration_ms == 150U);
    assert(config.predictive_min_mps == 0.01f);
    assert(has_parameter("audio_climb_rate_average_s"));
    assert(has_parameter("predictive_interval_ms"));
    assert(has_parameter("predictive_duration_ms"));
    assert(has_parameter("bluetooth_battery_mode"));
    assert(has_parameter("bluetooth_tx_power"));
    assert(has_parameter("bluetooth_notify_rate_hz"));
    assert(!has_parameter("lift_confirm_distance_m"));
    assert(!has_parameter("sink_confirm_distance_m"));
    assert(!has_parameter("predictive_freq_hz"));
    assert(!has_parameter("predictive_max_mps"));
    for (size_t index = 0U; index < app_config_parameter_count(); index++) {
        app_parameter_info_t info = {0};

        assert(app_config_parameter_info(index, &info));
        if (info.scope == APP_PARAMETER_SCOPE_SHARED) {
            shared_count++;
        } else if (info.scope == APP_PARAMETER_SCOPE_PROFILE) {
            profile_count++;
        }
    }
    assert(shared_count == 10U);
    assert(profile_count == 22U);
    assert(app_config_validate(&config));
    assert(app_config_set_text(&config, "bluetooth_battery_mode", "PERCENT"));
    assert(config.bluetooth_battery_mode ==
           APP_BLUETOOTH_BATTERY_MODE_PERCENT);
    assert(!app_config_set_text(&config, "bluetooth_battery_mode", "INVALID"));
    assert(app_config_set_text(&config, "bluetooth_tx_power", "MIN"));
    assert(config.bluetooth_tx_power == APP_BLUETOOTH_TX_POWER_MIN);
    assert(app_config_bluetooth_tx_power_dbm(config.bluetooth_tx_power) == -24);
    assert(app_config_set_text(&config, "bluetooth_tx_power", "LOW"));
    assert(app_config_bluetooth_tx_power_dbm(config.bluetooth_tx_power) == -12);
    assert(app_config_set_text(&config, "bluetooth_tx_power", "NORMAL"));
    assert(app_config_bluetooth_tx_power_dbm(config.bluetooth_tx_power) == 0);
    assert(app_config_set_text(&config, "bluetooth_tx_power", "HIGH"));
    assert(app_config_bluetooth_tx_power_dbm(config.bluetooth_tx_power) == 9);
    assert(strcmp(app_config_bluetooth_tx_power_name(
                      config.bluetooth_tx_power),
                  "HIGH") == 0);
    assert(!app_config_set_text(&config, "bluetooth_tx_power", "MAX"));
    assert(!app_config_set_text(&config, "bluetooth_tx_power", "20"));
    assert(app_config_reset(&config, 1U, "bluetooth_tx_power"));
    assert(config.bluetooth_tx_power == APP_BLUETOOTH_TX_POWER_LOW);
    assert(app_config_set_text(&config, "bluetooth_notify_rate_hz", "1"));
    assert(config.bluetooth_notify_rate_hz == 1U);
    assert(app_config_set_text(&config, "bluetooth_notify_rate_hz", "50"));
    assert(config.bluetooth_notify_rate_hz == 50U);
    assert(!app_config_set_text(&config, "bluetooth_notify_rate_hz", "0"));
    assert(!app_config_set_text(&config, "bluetooth_notify_rate_hz", "51"));

    config.predictive_duration_ms = config.predictive_interval_ms + 1U;
    assert(!app_config_validate(&config));
    app_config_set_defaults(&config);
    config.predictive_min_mps = config.lift_start_mps + 0.01f;
    assert(!app_config_validate(&config));
}

static void test_parameter_profile_contract(void) {
    app_config_profiles_t profiles = {0};
    app_config_t candidate = {0};
    app_config_t selected = {0};
    app_config_t other = {0};
    size_t index = 0U;

    app_config_profiles_set_defaults(&profiles);
    assert(app_config_profiles_validate(&profiles));
    assert(profiles.count == 3U);
    assert(profiles.profiles[0].parameter_number == 1U);
    assert(profiles.profiles[0].config.lift_start_mps == 0.10f);
    assert(profiles.profiles[1].parameter_number == 2U);
    assert(profiles.profiles[1].config.lift_start_mps == 0.20f);
    assert(profiles.profiles[1].config.sink_start_mps == -2.00f);
    assert(profiles.profiles[2].parameter_number == 3U);
    assert(profiles.profiles[2].config.lift_start_mps == 0.30f);
    assert(profiles.profiles[2].config.sink_start_mps == -2.20f);
    candidate = profiles.profiles[1].config;
    candidate.lift_start_mps = 0.50f;
    assert(app_config_reset(&candidate, 2U, "lift_start_mps"));
    assert(candidate.lift_start_mps == 0.20f);
    profiles.count = 3U;
    profiles.profiles[0].parameter_number = 5U;
    profiles.profiles[1].parameter_number = 1U;
    profiles.profiles[2].parameter_number = 3U;
    app_config_set_defaults(&profiles.profiles[1].config);
    app_config_set_defaults(&profiles.profiles[2].config);
    assert(app_config_profiles_validate(&profiles));
    app_config_profiles_sort(&profiles);
    assert(app_config_profiles_find(&profiles, 3U, &index));
    assert(index == 1U);
    assert(app_config_profiles_next_index(&profiles, 2U) == 0U);

    assert(app_config_profiles_get_config(&profiles, index, &candidate));
    candidate.sea_level_pressure_pa = 100123.0f;
    candidate.lift_freq_base_hz = 1200U;
    assert(app_config_profiles_set_config(&profiles, index, &candidate));
    assert(app_config_profiles_get_config(&profiles, index, &selected));
    assert(app_config_profiles_get_config(&profiles, 0U, &other));
    assert(selected.sea_level_pressure_pa == 100123.0f);
    assert(other.sea_level_pressure_pa == 100123.0f);
    assert(selected.lift_freq_base_hz == 1200U);
    assert(other.lift_freq_base_hz == 1047U);
}

static void test_average_bypass_and_partial_window(void) {
    app_config_t config = test_config();
    static vario_audio_state_t state;

    vario_audio_reset(&state);
    (void) step_at(&state, &config, INT64_C(1000000), 2.0f, true, false);
    assert(state.averaged_climb_rate_mps == 2.0f);
    assert(state.history_count == 0U);

    config.audio_climb_rate_average_s = 5.0f;
    vario_audio_reset(&state);
    (void) step_at(&state, &config, INT64_C(1000000), 0.0f, true, false);
    (void) step_at(&state, &config, INT64_C(2000000), 1.0f, true, false);
    (void) step_at(&state, &config, INT64_C(3000000), 2.0f, true, false);
    assert(fabsf(state.averaged_climb_rate_mps - 1.0f) < 0.0001f);
    assert(state.history_count == 3U);

    (void) step_at(&state, &config, INT64_C(3000000), 20.0f, true, false);
    assert(fabsf(state.averaged_climb_rate_mps - 1.0f) < 0.0001f);
    assert(state.history_count == 3U);
}

static void test_average_window_expiry_and_resets(void) {
    app_config_t config = test_config();
    static vario_audio_state_t state;
    vario_audio_command_t command = {0};

    config.audio_climb_rate_average_s = 5.0f;
    vario_audio_reset(&state);
    (void) step_at(&state, &config, INT64_C(1000000), 0.0f, true, false);
    (void) step_at(&state, &config, INT64_C(6000000), 2.0f, true, false);
    assert(fabsf(state.averaged_climb_rate_mps - 1.0f) < 0.0001f);
    (void) step_at(&state, &config, INT64_C(6000001), 4.0f, true, false);
    assert(fabsf(state.averaged_climb_rate_mps - 3.0f) < 0.0001f);
    assert(state.history_count == 2U);

    command = step_at(&state, &config, INT64_C(7000000), NAN, false, false);
    assert(command.mode == VARIO_AUDIO_SILENT);
    assert(state.history_count == 0U);
    (void) step_at(&state, &config, INT64_C(8000000), 1.0f, true, false);
    (void) step_at(&state, &config, INT64_C(9000000), 3.0f, true, false);
    assert(fabsf(state.averaged_climb_rate_mps - 2.0f) < 0.0001f);
    (void) step_at(&state, &config, INT64_C(10000000), 4.0f, true, true);
    assert(fabsf(state.averaged_climb_rate_mps - 4.0f) < 0.0001f);
    assert(state.history_count == 1U);

    vario_audio_reset(&state);
    assert(state.history_count == 0U);
    assert(!state.averaged_climb_rate_valid);
}

static void test_predictive_lift_direct_transitions(void) {
    app_config_t config = test_config();
    static vario_audio_state_t state;
    vario_audio_command_t command = {0};

    config.predictive_buzzer_enabled = true;
    config.audio_state_hold_ms = 1000U;
    vario_audio_reset(&state);
    command = step_at(&state, &config, INT64_C(1000000), 0.10f, true, false);
    assert(command.mode == VARIO_AUDIO_PREDICTIVE);
    assert(command.sounding);
    assert(command.frequency_hz == 610U);

    command = step_at(&state, &config, INT64_C(1010000), 0.21f, true, false);
    assert(command.mode == VARIO_AUDIO_LIFT);
    assert(command.sounding);
    assert(command.frequency_hz == 621U);

    command = step_at(&state, &config, INT64_C(1020000), 0.0f, true, false);
    assert(command.mode == VARIO_AUDIO_PREDICTIVE);
    assert(command.sounding);
    assert(command.frequency_hz == 600U);

    vario_audio_reset(&state);
    command = step_at(&state, &config, INT64_C(2000000),
                      config.lift_start_mps, true, false);
    assert(command.mode == VARIO_AUDIO_PREDICTIVE);
}

static void test_predictive_fixed_timing(void) {
    app_config_t config = test_config();
    static vario_audio_state_t state;
    vario_audio_command_t command = {0};

    config.predictive_buzzer_enabled = true;
    vario_audio_reset(&state);
    command = step_at(&state, &config, INT64_C(1000000), 0.1f, true, false);
    assert(command.sounding);
    command = step_at(&state, &config, INT64_C(1049999), 0.1f, true, false);
    assert(command.sounding);
    command = step_at(&state, &config, INT64_C(1050000), 0.1f, true, false);
    assert(!command.sounding);
    command = step_at(&state, &config, INT64_C(1499999), 0.1f, true, false);
    assert(!command.sounding);
    command = step_at(&state, &config, INT64_C(1500000), 0.1f, true, false);
    assert(command.sounding);
    command = step_at(&state, &config, INT64_C(6500000), 0.1f, true, false);
    assert(command.sounding);
}

int main(void) {
    test_parameter_contract();
    test_parameter_profile_contract();
    test_average_bypass_and_partial_window();
    test_average_window_expiry_and_resets();
    test_predictive_lift_direct_transitions();
    test_predictive_fixed_timing();
    puts("vario_audio tests passed");
    return 0;
}
