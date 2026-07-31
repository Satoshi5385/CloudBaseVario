idf_component_register(
    SRCS
        "app/main.c"
        "app/app_resources.c"
        "app/app_tasks.c"
        "domain/app_config.c"
        "domain/imu_fusion.c"
        "domain/vario_audio.c"
        "domain/vario_estimator.c"
        "platform/app_power.c"
        "platform/audio_output.c"
        "platform/ble_vario.c"
        "platform/bmp581.c"
        "platform/board.c"
        "platform/config_storage.c"
        "platform/firmware_update.c"
        "platform/icm42688_hxy.c"
        "platform/imu_calibration_storage.c"
        "platform/sensor_bus.c"
        "platform/system_io.c"
        "platform/usb_device_service.c"
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
        heap
        cjson
        nvs_flash
        spi_flash
        wear_levelling
)

# Generate an initially empty, wear-levelling FAT image without adding it to
# the normal "flash" target. Provision it once with "idf.py config-flash";
# subsequent application-only flashes then preserve parameters.json.
set(config_fat_seed_dir "${CMAKE_CURRENT_BINARY_DIR}/config_fat_seed")
file(MAKE_DIRECTORY "${config_fat_seed_dir}")
fatfs_create_spiflash_image(config "${config_fat_seed_dir}")
