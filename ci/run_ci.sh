#!/usr/bin/env bash

set -euo pipefail

script_dir="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
repo_root="$(CDPATH= cd -- "${script_dir}/.." && pwd)"
build_dir="${CI_BUILD_DIR:-build}"
test_bin_dir=""

cd "${repo_root}"
git config --global --add safe.directory "${GITHUB_WORKSPACE:-${repo_root}}"

section() {
    printf '\n==> %s\n' "$1"
}

run_idf() {
    if [[ -n "${CI_BUILD_DIR:-}" ]]; then
        /opt/esp/entrypoint.sh idf.py -B "${build_dir}" \
            -D "SDKCONFIG=${repo_root}/sdkconfig.ci" "$@"
    else
        /opt/esp/entrypoint.sh idf.py "$@"
    fi
}

run_python_tests() {
    section "Run Python tests"
    python -m unittest discover -s tests -p "test_*.py"
}

check_python_tools() {
    section "Check Python tools"
    python -m compileall -q tools tests
}

configure_target() {
    section "Configure ESP32-S3 target"
    run_idf set-target esp32s3
}

compile_and_run() {
    local name="$1"
    shift
    cc -std=c11 -Wall -Wextra -Werror -pedantic "$@" \
        -o "${test_bin_dir}/${name}"
    "${test_bin_dir}/${name}"
}

run_host_c_tests() (
    section "Build and run host C tests"
    test_bin_dir="$(mktemp -d)"
    trap 'rm -rf -- "${test_bin_dir}"' EXIT

    compile_and_run test_auto_power_off \
        -I SRC tests/test_auto_power_off.c SRC/domain/auto_power_off.c -lm
    compile_and_run test_firmware_update_policy \
        -I SRC tests/test_firmware_update_policy.c \
        SRC/domain/firmware_update_policy.c -lm
    compile_and_run test_firmware_pending_record \
        -I SRC tests/test_firmware_pending_record.c \
        SRC/domain/firmware_pending_record.c
    compile_and_run test_firmware_metadata \
        -I SRC tests/test_firmware_metadata.c SRC/domain/firmware_metadata.c
    compile_and_run test_board_info \
        -I SRC tests/test_board_info.c SRC/domain/board_identity.c \
        SRC/domain/board_info.c SRC/domain/firmware_metadata.c \
        SRC/domain/firmware_authentication.c
    compile_and_run test_vario_audio \
        -I SRC tests/test_vario_audio.c SRC/domain/app_config.c \
        SRC/domain/vario_audio.c -lm
    compile_and_run test_system_policy \
        -I SRC tests/test_system_policy.c SRC/domain/system_policy.c
    compile_and_run test_imu_calibration_controller \
        -I SRC tests/test_imu_calibration_controller.c \
        SRC/domain/imu_calibration_controller.c SRC/domain/imu_fusion.c \
        SRC/domain/app_config.c -lm
    compile_and_run test_lk8ex1 \
        -I SRC tests/test_lk8ex1.c SRC/domain/lk8ex1.c \
        SRC/domain/battery_level.c -lm
    compile_and_run test_ble_nus_tx \
        -I SRC tests/test_ble_nus_tx.c SRC/domain/ble_nus_tx.c
    compile_and_run test_usb_storage_policy \
        -I SRC tests/test_usb_storage_policy.c \
        SRC/domain/usb_storage_policy.c
    compile_and_run test_watchdog_recovery_policy \
        -I SRC tests/test_watchdog_recovery_policy.c \
        SRC/domain/watchdog_recovery_policy.c
    compile_and_run test_board_identity \
        -I SRC tests/test_board_identity.c SRC/domain/board_identity.c
    compile_and_run test_storage_psram_buffer \
        -I components/esp_tinyusb/include_private \
        tests/test_storage_psram_buffer.c \
        components/esp_tinyusb/storage_psram_buffer.c
    compile_and_run test_config_json \
        -I tests/host_stubs -I SRC \
        -I managed_components/espressif__cjson/cJSON \
        tests/test_config_json.c SRC/platform/config_json.c \
        SRC/domain/app_config.c \
        managed_components/espressif__cjson/cJSON/cJSON.c -lm

)

build_firmware() {
    section "Build firmware"
    run_idf build
}

verify_configuration() {
    section "Verify effective watchdog and Bluetooth configuration"
    grep -Fx '#define CONFIG_BOOTLOADER_WDT_TIME_MS 9000' "${build_dir}/config/sdkconfig.h"
    grep -Fx '#define CONFIG_ESP_INT_WDT_TIMEOUT_MS 300' "${build_dir}/config/sdkconfig.h"
    grep -Fx '#define CONFIG_ESP_INT_WDT_CHECK_CPU1 1' "${build_dir}/config/sdkconfig.h"
    grep -Fx '#define CONFIG_ESP_TASK_WDT_PANIC 1' "${build_dir}/config/sdkconfig.h"
    grep -Fx '#define CONFIG_ESP_TASK_WDT_TIMEOUT_S 5' "${build_dir}/config/sdkconfig.h"
    grep -Fx '#define CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0 1' "${build_dir}/config/sdkconfig.h"
    grep -Fx '#define CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 1' "${build_dir}/config/sdkconfig.h"
    grep -Fx '#define CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT 1' "${build_dir}/config/sdkconfig.h"
    grep -Fx '#define CONFIG_ESP_COREDUMP_ENABLE_TO_NONE 1' "${build_dir}/config/sdkconfig.h"
    grep -Fx '#define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 1' "${build_dir}/config/sdkconfig.h"
    grep -Fx '#define CONFIG_BT_NIMBLE_MAX_CCCDS 3' "${build_dir}/config/sdkconfig.h"
    grep -Fx '#define CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU 247' "${build_dir}/config/sdkconfig.h"
}

run_all() {
    run_python_tests
    check_python_tools
    configure_target
    run_host_c_tests
    build_firmware
    verify_configuration
}

case "${1:-all}" in
    python-tests) run_python_tests ;;
    python-compile) check_python_tools ;;
    configure) configure_target ;;
    host-c) run_host_c_tests ;;
    firmware) build_firmware ;;
    configuration) verify_configuration ;;
    all) run_all ;;
    *)
        printf 'Usage: %s {python-tests|python-compile|configure|host-c|firmware|configuration|all}\n' "$0" >&2
        exit 2
        ;;
esac
