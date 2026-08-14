#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "domain/firmware_update_policy.h"

int main(void) {
    static const char expected_project[] = "CloudBaseVario-Aohazuku";
    char unterminated_project[32];

    assert(firmware_update_policy_power_allowed(true, false, NAN));
    assert(firmware_update_policy_power_allowed(true, true, 3.0f));
    assert(firmware_update_policy_power_allowed(false, true, 3.4001f));
    assert(!firmware_update_policy_power_allowed(false, true, 3.4f));
    assert(!firmware_update_policy_power_allowed(false, true, 3.3999f));
    assert(!firmware_update_policy_power_allowed(false, false, 4.2f));
    assert(!firmware_update_policy_power_allowed(false, true, NAN));
    assert(!firmware_update_policy_power_allowed(false, true, INFINITY));

    assert(firmware_update_policy_project_name_matches(
        expected_project, sizeof(expected_project), expected_project));
    assert(!firmware_update_policy_project_name_matches(
        "CloudBaseVario", sizeof("CloudBaseVario"), expected_project));
    assert(!firmware_update_policy_project_name_matches(
        "CloudBaseVario-Aohazuku-extra",
        sizeof("CloudBaseVario-Aohazuku-extra"), expected_project));
    assert(!firmware_update_policy_project_name_matches(
        "CloudBaseVario-Aohazuk", sizeof("CloudBaseVario-Aohazuk"),
        expected_project));
    memset(unterminated_project, 'A', sizeof(unterminated_project));
    assert(!firmware_update_policy_project_name_matches(
        unterminated_project, sizeof(unterminated_project), expected_project));
    assert(!firmware_update_policy_project_name_matches(
        NULL, sizeof(expected_project), expected_project));

    puts("firmware update policy tests passed");
    return 0;
}
