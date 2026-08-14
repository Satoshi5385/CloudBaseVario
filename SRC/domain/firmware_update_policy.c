#include "domain/firmware_update_policy.h"

#include <math.h>
#include <string.h>

bool firmware_update_policy_power_allowed(bool external_power_present,
                                          bool battery_valid,
                                          float battery_voltage_v) {
    return external_power_present ||
           (battery_valid && isfinite(battery_voltage_v) &&
            battery_voltage_v > FIRMWARE_UPDATE_MIN_BATTERY_V);
}

bool firmware_update_policy_project_name_matches(
    const char *candidate,
    size_t candidate_capacity,
    const char *expected) {
    const char *terminator;
    size_t candidate_length;
    size_t expected_length;

    if (candidate == NULL || candidate_capacity == 0U || expected == NULL) {
        return false;
    }
    terminator = memchr(candidate, '\0', candidate_capacity);
    if (terminator == NULL) {
        return false;
    }
    candidate_length = (size_t) (terminator - candidate);
    expected_length = strlen(expected);
    return candidate_length == expected_length &&
           memcmp(candidate, expected, expected_length) == 0;
}
