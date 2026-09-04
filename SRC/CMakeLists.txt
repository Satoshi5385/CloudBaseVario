idf_component_register(
    SRCS
        "app/main.c"
        "app/startup.c"
        "app/app_resources.c"
        "app/app_tasks.c"
        "app/app_workers.c"
        "app/ble_tx_worker.c"
        "app/gps_worker.c"
        "domain/app_config.c"
        "domain/battery_level.c"
        "domain/ble_nus_tx.c"
        "domain/board_info.c"
        "domain/board_identity.c"
        "domain/auto_power_off.c"
        "domain/firmware_update_policy.c"
        "domain/firmware_authentication.c"
        "domain/firmware_metadata.c"
        "domain/gps_nmea.c"
        "domain/imu_calibration_controller.c"
        "domain/imu_fusion.c"
        "domain/lk8ex1.c"
        "domain/system_policy.c"
        "domain/usb_storage_policy.c"
        "domain/watchdog_recovery_policy.c"
        "domain/vario_audio.c"
        "domain/vario_estimator.c"
        "platform/app_power.c"
        "platform/audio_output.c"
        "platform/ble_vario.c"
        "platform/bmp581.c"
        "platform/board.c"
        "platform/board_identity_storage.c"
        "platform/config_storage.c"
        "platform/config_json.c"
        "platform/firmware_update.c"
        "platform/firmware_auth.c"
        "platform/gps_l96.c"
        "platform/icm42688_hxy.c"
        "platform/imu_calibration_storage.c"
        "platform/sensor_bus.c"
        "platform/safe_stop_wake.c"
        "platform/system_io.c"
        "platform/switch_preferences.c"
        "platform/usb_device_service.c"
        "platform/watchdog_service.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        app_update
        bt
        bootloader_support
        esp_adc
        esp_app_format
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_ledc
        esp_driver_uart
        esp_hw_support
        esp_pm
        esp_psram
        esp_tinyusb
        esp_system
        fatfs
        mbedtls
        heap
        cjson
        nvs_flash
        spi_flash
        wear_levelling
)

# Ship the offline setting editor inside the application image. Normal startup
# restores changed or missing bytes to the read-only MSC root.
target_add_binary_data(
    ${COMPONENT_LIB}
    "${CMAKE_SOURCE_DIR}/DOC/setting_editor.html"
    BINARY
    RENAME_TO "setting_editor_html"
)

# Keep the USB-MSC update identity identical to the project name embedded by
# ESP-IDF in esp_app_desc_t.
target_compile_definitions(
    ${COMPONENT_LIB}
    PRIVATE CBV_FIRMWARE_PROJECT_NAME="${CMAKE_PROJECT_NAME}"
)

# Generate an initially empty, wear-levelling FAT image without adding it to
# the normal "flash" target. Provision it once with "idf.py config-flash";
# subsequent application-only flashes then preserve setting.json.
set(config_fat_seed_dir "${CMAKE_CURRENT_BINARY_DIR}/config_fat_seed")
file(MAKE_DIRECTORY "${config_fat_seed_dir}")

if(NOT CONFIG_FATFS_SECTOR_512 OR
   NOT CONFIG_WL_SECTOR_SIZE_512 OR
   NOT CONFIG_WL_SECTOR_MODE_SAFE)
    message(FATAL_ERROR
        "config FAT generation requires 512-byte FAT/WL sectors and Safety mode")
endif()

idf_build_get_property(config_fat_idf_path IDF_PATH)
idf_build_get_property(config_fat_python PYTHON)
partition_table_get_partition_info(
    config_fat_partition_size "--partition-name config" "size")
partition_table_get_partition_info(
    config_fat_partition_offset "--partition-name config" "offset")
if(NOT config_fat_partition_size OR NOT config_fat_partition_offset)
    message(FATAL_ERROR "config FAT partition size and offset are required")
endif()

set(config_fat_image "${CMAKE_BINARY_DIR}/config.bin")
set(config_fat_generator
    "${config_fat_idf_path}/components/fatfs/wl_fatfsgen.py")
set(config_fat_validator
    "${CMAKE_SOURCE_DIR}/tools/verify_config_fat.py")
set(config_fat_long_names_option)
if(CONFIG_FATFS_LFN_HEAP OR CONFIG_FATFS_LFN_STACK)
    list(APPEND config_fat_long_names_option --long_name_support)
endif()

add_custom_target(
    fatfs_config_bin ALL
    COMMAND "${config_fat_python}" "${config_fat_generator}"
        "${config_fat_seed_dir}"
        ${config_fat_long_names_option}
        --use_default_datetime
        --partition_size "${config_fat_partition_size}"
        --output_file "${config_fat_image}"
        --sector_size 512
        --wl_mode safe
    COMMAND "${config_fat_python}" "${config_fat_validator}"
        --image "${config_fat_image}"
        --partition-size "${config_fat_partition_size}"
    COMMENT
        "Generating and verifying Safety WL config FAT at ${config_fat_partition_offset}"
    VERBATIM
)
set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND PROPERTY
    ADDITIONAL_CLEAN_FILES "${config_fat_image}")

idf_component_get_property(config_fat_flash_args esptool_py FLASH_ARGS)
idf_component_get_property(config_fat_flash_sub_args esptool_py FLASH_SUB_ARGS)
esptool_py_flash_target(
    config-flash "${config_fat_flash_args}"
    "${config_fat_flash_sub_args}" ALWAYS_PLAINTEXT)
esptool_py_flash_to_partition(
    config-flash config "${config_fat_image}")
add_dependencies(config-flash fatfs_config_bin)
