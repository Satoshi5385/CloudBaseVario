#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "domain/lk8ex1.h"
#include "domain/battery_level.h"

static void test_battery_level_conversion(void) {
    assert(battery_level_percent_from_voltage(2.9f) == 0U);
    assert(battery_level_percent_from_voltage(3.2f) == 0U);
    assert(battery_level_percent_from_voltage(3.35f) == 5U);
    assert(battery_level_percent_from_voltage(3.5f) == 10U);
    assert(battery_level_percent_from_voltage(3.55f) == 15U);
    assert(battery_level_percent_from_voltage(3.6f) == 20U);
    assert(battery_level_percent_from_voltage(3.65f) == 30U);
    assert(battery_level_percent_from_voltage(3.7f) == 40U);
    assert(battery_level_percent_from_voltage(3.8f) == 60U);
    assert(battery_level_percent_from_voltage(3.9f) == 80U);
    assert(battery_level_percent_from_voltage(4.0f) == 90U);
    assert(battery_level_percent_from_voltage(4.1f) == 100U);
    assert(battery_level_percent_from_voltage(4.2f) == 100U);
    assert(battery_level_percent_from_voltage(4.3f) == 100U);
    assert(battery_level_percent_from_voltage(NAN) == 0U);
}

static void test_battery_power_policy(void) {
    assert(battery_power_startup_blocked(false, true, 3.1999f));
    assert(battery_power_startup_blocked(false, true, 3.2f));
    assert(!battery_power_startup_blocked(false, true, 3.2001f));
    assert(!battery_power_startup_blocked(true, true, 3.0f));
    assert(!battery_power_startup_blocked(false, false, 3.0f));
    assert(!battery_power_startup_blocked(false, true, NAN));

    assert(battery_power_shutdown_required(false, true, 3.0999f));
    assert(battery_power_shutdown_required(false, true, 3.1f));
    assert(!battery_power_shutdown_required(false, true, 3.1001f));
    assert(!battery_power_shutdown_required(true, true, 3.0f));
    assert(!battery_power_shutdown_required(false, false, 3.0f));
    assert(!battery_power_shutdown_required(false, true, NAN));
}

static void test_battery_display_window(void) {
    battery_display_state_t state = {0};
    float displayed_voltage_v = 0.0f;

    assert(!battery_display_update(
        &state, false, 0.0f, INT64_C(0), &displayed_voltage_v));
    assert(battery_display_update(
        &state, true, 3.9f, INT64_C(0), &displayed_voltage_v));
    assert(fabsf(displayed_voltage_v - 3.9f) < 0.0001f);

    assert(battery_display_update(
        &state, true, 3.7f, INT64_C(10000000), &displayed_voltage_v));
    assert(fabsf(displayed_voltage_v - 3.9f) < 0.0001f);
    assert(battery_display_update(
        &state, false, NAN, INT64_C(20000000), &displayed_voltage_v));
    assert(fabsf(displayed_voltage_v - 3.9f) < 0.0001f);

    assert(battery_display_update(
        &state, true, 3.8f, BATTERY_DISPLAY_UPDATE_INTERVAL_US,
        &displayed_voltage_v));
    assert(fabsf(displayed_voltage_v - 3.7f) < 0.0001f);
    assert(battery_display_update(
        &state, false, NAN,
        BATTERY_DISPLAY_UPDATE_INTERVAL_US * INT64_C(2),
        &displayed_voltage_v));
    assert(fabsf(displayed_voltage_v - 3.8f) < 0.0001f);
}

static void assert_sentence_checksum(const char *sentence) {
    const char *separator = strchr(sentence, '*');
    unsigned int expected = 0U;

    assert(sentence[0] == '$');
    assert(separator != NULL);
    for (const unsigned char *cursor =
             (const unsigned char *) sentence + 1;
         cursor < (const unsigned char *) separator; cursor++) {
        expected ^= *cursor;
    }
    assert((unsigned long) expected == strtoul(separator + 1, NULL, 16));
}

static void test_invalid_sentinels(void) {
    vario_result_t vario = {0};
    system_snapshot_t system = {0};
    lk8ex1_fields_t fields = {0};

    assert(lk8ex1_format_fields(
        &vario, &system, APP_BLUETOOTH_BATTERY_MODE_VOLTAGE, &fields));
    assert(strcmp(fields.raw_pressure, "999999") == 0);
    assert(strcmp(fields.altitude, "99999") == 0);
    assert(strcmp(fields.vario, "9999") == 0);
    assert(strcmp(fields.temperature, "99") == 0);
    assert(strcmp(fields.battery, "999") == 0);
    assert(!fields.sentence_available);

    assert(lk8ex1_format_fields(
        &vario, &system, APP_BLUETOOTH_BATTERY_MODE_PERCENT, &fields));
    assert(strcmp(fields.battery, "999") == 0);
}

static void test_percent_wire_encoding(void) {
    vario_result_t vario = {0};
    system_snapshot_t system = {
        .battery_display_valid = true,
    };
    lk8ex1_fields_t fields = {0};

    system.battery_display_voltage_v = 3.0f;
    assert(lk8ex1_format_fields(
        &vario, &system, APP_BLUETOOTH_BATTERY_MODE_PERCENT, &fields));
    assert(strcmp(fields.battery, "1000") == 0);

    system.battery_display_voltage_v = 4.06f;
    assert(lk8ex1_format_fields(
        &vario, &system, APP_BLUETOOTH_BATTERY_MODE_PERCENT, &fields));
    assert(strcmp(fields.battery, "1096") == 0);

    system.battery_display_voltage_v = 4.1f;
    assert(lk8ex1_format_fields(
        &vario, &system, APP_BLUETOOTH_BATTERY_MODE_PERCENT, &fields));
    assert(strcmp(fields.battery, "1100") == 0);
}

static void test_sentence_contract(app_bluetooth_battery_mode_t battery_mode,
                                   const char *expected_battery) {
    vario_result_t vario = {
        .pressure_pa_x100 = 10132500,
        .climb_rate_mps = 1.23f,
        .pressure_valid = true,
        .climb_rate_valid = true,
    };
    system_snapshot_t system = {
        .battery_display_voltage_v = 3.8f,
        .battery_display_valid = true,
    };
    char sentence[LK8EX1_SENTENCE_MAX_LENGTH] = {0};
    size_t length = 0U;

    assert(lk8ex1_format_sentence(&vario, &system, battery_mode, sentence,
                                  sizeof(sentence), &length));
    assert(length == strlen(sentence));
    char expected[80] = {0};
    int written = snprintf(expected, sizeof(expected),
                           "$LK8EX1,101325,99999,123,99,%s,*",
                           expected_battery);
    assert(written > 0 && (size_t) written < sizeof(expected));
    assert(strncmp(sentence, expected, strlen(expected)) == 0);
    assert_sentence_checksum(sentence);
    assert(sentence[length - 2U] == '\r');
    assert(sentence[length - 1U] == '\n');
}

int main(void) {
    test_battery_level_conversion();
    test_battery_power_policy();
    test_battery_display_window();
    test_invalid_sentinels();
    test_percent_wire_encoding();
    test_sentence_contract(APP_BLUETOOTH_BATTERY_MODE_VOLTAGE, "3.80");
    test_sentence_contract(APP_BLUETOOTH_BATTERY_MODE_PERCENT, "1060");
    puts("lk8ex1 tests passed");
    return 0;
}
