idf_component_register(
    SRCS
        "app/main.c"
        "app/startup.c"
        "app/app_resources.c"
        "app/app_tasks.c"
        "app/app_workers.c"
        "app/ble_tx_worker.c"
        "domain/app_config.c"
        "domain/battery_level.c"
        "domain/board_info.c"
        "domain/board_identity.c"
        "domain/auto_power_off.c"
        "domain/firmware_update_policy.c"
        "domain/firmware_authentication.c"
        "domain/firmware_metadata.c"
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
fatfs_create_spiflash_image(config "${config_fat_seed_dir}")
