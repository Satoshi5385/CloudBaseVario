#pragma once

#include <stdbool.h>
#include <stddef.h>

#define FIRMWARE_UPDATE_MIN_BATTERY_V 3.4f

/**
 * @brief Decide whether the available power is safe for an OTA flash write.
 *
 * USB external power always permits the update. Without USB power, the
 * battery reading must be valid, finite, and strictly above 3.4 V.
 */
bool firmware_update_policy_power_allowed(bool external_power_present,
                                          bool battery_valid,
                                          float battery_voltage_v);

/**
 * @brief Match a fixed-width app descriptor project name exactly.
 *
 * The candidate must contain a NUL terminator within candidate_capacity.
 * Prefix, suffix, and unterminated values are rejected.
 */
bool firmware_update_policy_project_name_matches(
    const char *candidate,
    size_t candidate_capacity,
    const char *expected);
