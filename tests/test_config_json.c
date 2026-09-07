#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "domain/app_config.h"
#include "platform/config_json.h"

#define STRINGIFY_VALUE_INNER(value) #value
#define STRINGIFY_VALUE(value) STRINGIFY_VALUE_INNER(value)

#define SHARED_FIELDS                                                        \
    "\"sea_level_pressure_pa\":100000,"                                   \
    "\"auto_power_off_minutes\":60,"                                      \
    "\"flight_gps_speed_threshold_kmh\":10.0,"                          \
    "\"stationary_confirm_seconds\":60,"                                 \
    "\"filter_mode\":\"AUTO\","                                        \
    "\"bluetooth_battery_mode\":\"PERCENT\","                          \
    "\"bluetooth_tx_power\":\"LOW\","                                  \
    "\"bluetooth_notify_rate_hz\":10,"                                    \
    "\"gps_send_interval_ms\":1000,"                                      \
    "\"imu_gyro_calibration_samples\":200"

#define PROFILE_FIELDS(lift_start, lift_end, sink_start, sink_end)            \
    "\"audio_mute_when_stationary\":false,"                               \
    "\"predictive_buzzer_enabled\":false,"                                \
    "\"audio_climb_rate_average_s\":1.0,"                                 \
    "\"lift_start_mps\":" STRINGIFY_VALUE(lift_start) ","                \
    "\"lift_end_mps\":" STRINGIFY_VALUE(lift_end) ","                    \
    "\"sink_start_mps\":" STRINGIFY_VALUE(sink_start) ","                \
    "\"sink_end_mps\":" STRINGIFY_VALUE(sink_end) ","                    \
    "\"audio_state_hold_ms\":200,"                                        \
    "\"audio_stale_ms\":500,"                                             \
    "\"lift_freq_base_hz\":1047,"                                         \
    "\"lift_freq_rate_hz_per_mps\":100.0,"                                \
    "\"lift_freq_max_hz\":2600,"                                          \
    "\"lift_time_ms_at_0p2\":400,"                                        \
    "\"lift_time_ms_at_1p0\":400,"                                        \
    "\"lift_time_ms_at_2p5\":300,"                                        \
    "\"lift_time_ms_at_5p0\":100,"                                        \
    "\"sink_freq_start_hz\":523,"                                         \
    "\"sink_freq_rate_hz_per_mps\":40.0,"                                 \
    "\"sink_freq_min_hz\":240,"                                           \
    "\"audio_duty_percent\":50,"                                          \
    "\"predictive_interval_ms\":1000,"                                    \
    "\"predictive_duration_ms\":150,"                                     \
    "\"predictive_min_mps\":0.01"

#define PROFILE(number, fields)                                               \
    "{\"parameter_number\":" STRINGIFY_VALUE(number)                      \
    ",\"parameters\":{" fields "}}"

#define DOCUMENT(shared, sets)                                                \
    "{\"format_version\":1,\"mc_parameters\":{" shared                 \
    "},\"vario_parameter_sets\":[" sets "]}"

static bool parse(const char *json, app_config_profiles_t *profiles) {
    config_storage_diagnostics_t diagnostics = {0};

    app_config_profiles_set_defaults(profiles);
    return config_json_parse(json, strlen(json), profiles, &diagnostics);
}

static void assert_default_profiles(const app_config_profiles_t *profiles) {
    assert(profiles->count == 3U);
    assert(profiles->profiles[0].parameter_number == 1U);
    assert(profiles->profiles[1].parameter_number == 2U);
    assert(profiles->profiles[2].parameter_number == 3U);
    assert(fabsf(profiles->shared_config.sea_level_pressure_pa - 101325.0f) <
           0.01f);
}

static void test_unknown_items_are_ignored(void) {
    static const char document[] =
        "{\"format_version\":1,\"future_top\":true,"
        "\"mc_parameters\":{" SHARED_FIELDS ",\"future_shared\":7},"
        "\"vario_parameter_sets\":[{\"parameter_number\":1,"
        "\"future_set\":false,\"parameters\":{"
        PROFILE_FIELDS(0.15, 0.08, -1.8, -1.7)
        ",\"future_profile\":9}}]}";
    app_config_profiles_t profiles = {0};

    assert(parse(document, &profiles));
    assert(profiles.count == 1U);
    assert(fabsf(profiles.shared_config.sea_level_pressure_pa - 100000.0f) <
           0.01f);
    assert(fabsf(profiles.profiles[0].config.lift_start_mps - 0.15f) < 0.001f);
}

static void test_missing_shared_item_uses_defaults(void) {
    static const char document[] = DOCUMENT(
        "\"auto_power_off_minutes\":60,"
        "\"flight_gps_speed_threshold_kmh\":10.0,"
        "\"stationary_confirm_seconds\":60,\"filter_mode\":\"AUTO\","
        "\"bluetooth_battery_mode\":\"PERCENT\","
        "\"bluetooth_tx_power\":\"LOW\","
        "\"bluetooth_notify_rate_hz\":10,\"gps_send_interval_ms\":1000,"
        "\"imu_gyro_calibration_samples\":200",
        PROFILE(1, PROFILE_FIELDS(0.1, 0.08, -1.8, -1.7)));
    app_config_profiles_t profiles = {0};

    assert(!parse(document, &profiles));
    assert_default_profiles(&profiles);
}

static void test_invalid_sets_are_skipped(void) {
    static const char document[] = DOCUMENT(
        SHARED_FIELDS,
        PROFILE(1, PROFILE_FIELDS(0.1, 0.08, -1.8, -1.7)) ","
        "{\"parameter_number\":2,\"parameters\":{"
        "\"predictive_buzzer_enabled\":false}},"
        PROFILE(3, PROFILE_FIELDS(0.3, 1.0, -2.2, -2.1)) ","
        "null,{\"parameter_number\":9,\"parameters\":{}},"
        PROFILE(4, PROFILE_FIELDS(0.4, 0.35, -1.8, -1.7)));
    app_config_profiles_t profiles = {0};

    assert(parse(document, &profiles));
    assert(profiles.count == 2U);
    assert(profiles.profiles[0].parameter_number == 1U);
    assert(profiles.profiles[1].parameter_number == 4U);
}

static void test_duplicate_numbers_are_all_skipped(void) {
    static const char document[] = DOCUMENT(
        SHARED_FIELDS,
        PROFILE(1, PROFILE_FIELDS(0.1, 0.08, -1.8, -1.7)) ","
        PROFILE(1, PROFILE_FIELDS(0.2, 0.18, -2.0, -1.9)) ","
        PROFILE(2, PROFILE_FIELDS(0.2, 0.18, -2.0, -1.9)));
    app_config_profiles_t profiles = {0};

    assert(parse(document, &profiles));
    assert(profiles.count == 1U);
    assert(profiles.profiles[0].parameter_number == 2U);
}

static void test_no_valid_set_uses_defaults(void) {
    static const char document[] = DOCUMENT(
        SHARED_FIELDS,
        "{\"parameter_number\":1,\"parameters\":{}},"
        PROFILE(2, PROFILE_FIELDS(0.2, 1.0, -2.0, -1.9)));
    app_config_profiles_t profiles = {0};

    assert(!parse(document, &profiles));
    assert_default_profiles(&profiles);
}

static void test_duplicate_known_keys_use_matching_scope(void) {
    static const char duplicate_shared[] = DOCUMENT(
        SHARED_FIELDS ",\"sea_level_pressure_pa\":101325",
        PROFILE(1, PROFILE_FIELDS(0.1, 0.08, -1.8, -1.7)));
    static const char duplicate_profile_number[] = DOCUMENT(
        SHARED_FIELDS,
        "{\"parameter_number\":1,\"parameter_number\":1,"
        "\"parameters\":{" PROFILE_FIELDS(0.1, 0.08, -1.8, -1.7) "}},"
        PROFILE(2, PROFILE_FIELDS(0.2, 0.18, -2.0, -1.9)));
    app_config_profiles_t profiles = {0};

    assert(!parse(duplicate_shared, &profiles));
    assert_default_profiles(&profiles);
    assert(parse(duplicate_profile_number, &profiles));
    assert(profiles.count == 1U);
    assert(profiles.profiles[0].parameter_number == 2U);
}

int main(void) {
    test_unknown_items_are_ignored();
    test_missing_shared_item_uses_defaults();
    test_invalid_sets_are_skipped();
    test_duplicate_numbers_are_all_skipped();
    test_no_valid_set_uses_defaults();
    test_duplicate_known_keys_use_matching_scope();
    return 0;
}
