#include "domain/lk8ex1.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "domain/battery_level.h"

#define LK8EX1_BATTERY_PERCENT_OFFSET UINT16_C(1000)

static uint8_t lk8ex1_checksum(const char *body) {
    uint8_t checksum = 0U;

    if (body == NULL) {
        return 0U;
    }
    for (const unsigned char *cursor = (const unsigned char *) body;
         *cursor != '\0'; cursor++) {
        checksum ^= *cursor;
    }
    return checksum;
}

bool lk8ex1_format_fields(const vario_result_t *vario,
                          const system_snapshot_t *system,
                          app_bluetooth_battery_mode_t battery_mode,
                          lk8ex1_fields_t *fields) {
    int written = 0;

    if (vario == NULL || system == NULL || fields == NULL) {
        return false;
    }
    if (battery_mode != APP_BLUETOOTH_BATTERY_MODE_VOLTAGE &&
        battery_mode != APP_BLUETOOTH_BATTERY_MODE_PERCENT) {
        return false;
    }
    memset(fields, 0, sizeof(*fields));
    (void) memcpy(fields->raw_pressure, "999999", sizeof("999999"));
    (void) memcpy(fields->altitude, "99999", sizeof("99999"));
    (void) memcpy(fields->vario, "9999", sizeof("9999"));
    (void) memcpy(fields->temperature, "99", sizeof("99"));
    (void) memcpy(fields->battery, "999", sizeof("999"));

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
    if (system->battery_display_valid &&
        isfinite(system->battery_display_voltage_v) &&
        system->battery_display_voltage_v >= 0.0f) {
    if (battery_mode == APP_BLUETOOTH_BATTERY_MODE_PERCENT) {
            written = snprintf(
                fields->battery, sizeof(fields->battery), "%u",
                                (unsigned int) (LK8EX1_BATTERY_PERCENT_OFFSET +
                                battery_level_percent_from_voltage(
                                    system->battery_display_voltage_v)));
        } else {
            written = snprintf(fields->battery, sizeof(fields->battery),
                               "%.2f",
                               (double) system->battery_display_voltage_v);
        }
        if (written <= 0 || (size_t) written >= sizeof(fields->battery)) {
            return false;
        }
    }
    fields->sentence_available =
        vario->pressure_valid || vario->climb_rate_valid;
    return true;
}

bool lk8ex1_format_sentence(const vario_result_t *vario,
                            const system_snapshot_t *system,
                            app_bluetooth_battery_mode_t battery_mode,
                            char *sentence, size_t capacity,
                            size_t *length) {
    char body[96] = {0};
    lk8ex1_fields_t fields = {0};
    int written = 0;

    if (sentence == NULL || length == NULL || capacity == 0U ||
        !lk8ex1_format_fields(vario, system, battery_mode, &fields) ||
        !fields.sentence_available) {
        return false;
    }
    written = snprintf(body, sizeof(body), "LK8EX1,%s,%s,%s,%s,%s,",
                       fields.raw_pressure, fields.altitude, fields.vario,
                       fields.temperature, fields.battery);
    if (written <= 0 || (size_t) written >= sizeof(body)) {
        return false;
    }
    written = snprintf(sentence, capacity, "$%s*%02X\r\n", body,
                       lk8ex1_checksum(body));
    if (written <= 0 || (size_t) written >= capacity) {
        return false;
    }
    *length = (size_t) written;
    return true;
}
