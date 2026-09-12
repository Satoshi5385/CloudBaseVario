idf_build_get_property(target IDF_TARGET)
set(IDF_VERSION "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}")

set(srcs
    "descriptors_control.c"
    "tinyusb.c"
    "usb_descriptors.c"
    "tinyusb_task.c"
    )

set(priv_req "")
set(req fatfs vfs)

# USB-PHY driver
# For IDF 6.x and later, usb_phy driver is moved from usb to esp_hw_support
if(IDF_VERSION VERSION_LESS "6.0.0")
    list(APPEND priv_req usb)
else()
    list(APPEND priv_req esp_hw_support)
endif()

if(CONFIG_TINYUSB_CDC_ENABLED)
    list(APPEND srcs
        "cdc.c"
        "tinyusb_cdc_acm.c"
        )
    if(CONFIG_VFS_SUPPORT_IO)
        list(APPEND srcs
            "tinyusb_console.c"
            "vfs_tinyusb.c"
            )
    endif() # CONFIG_VFS_SUPPORT_IO
endif() # CONFIG_TINYUSB_CDC_ENABLED

if(CONFIG_TINYUSB_MSC_ENABLED)
    list(APPEND srcs
        "tinyusb_msc.c"
        "storage_spiflash.c"
        "storage_psram.c"
        "storage_psram_buffer.c"
        )
    if(CONFIG_SOC_SDMMC_HOST_SUPPORTED)
        list(APPEND srcs
            "storage_sdmmc.c"
            )
    endif() # CONFIG_SOC_SDMMC_HOST_SUPPORTED
endif() # CONFIG_TINYUSB_MSC_ENABLED


if(CONFIG_TINYUSB_NET_MODE_NCM)
    list(APPEND srcs
         "tinyusb_net.c"
         )
endif() # CONFIG_TINYUSB_NET_MODE_NCM

# Software VBUS monitoring is needed for chips with HS USB-OTG peripheral
if(${target} STREQUAL "esp32p4" OR ${target} STREQUAL "esp32s31")
    list(APPEND srcs "tinyusb_vbus_monitor.c")

    # VBUS monitoring requires GPIO driver
    if(IDF_VERSION VERSION_GREATER_EQUAL "5.3")
        list(APPEND priv_req esp_driver_gpio)
    else()
        list(APPEND priv_req driver)
    endif()
endif() # esp32p4/esp32s31

# idf_component.yml is ignored when the component manager is disabled, so the
# dependency on tinyusb must be declared here instead. It belongs in REQUIRES
# (not PRIV_REQUIRES) because the public header tinyusb.h includes tusb.h.
idf_build_get_property(idf_component_manager IDF_COMPONENT_MANAGER)
if(NOT idf_component_manager)
    list(APPEND req tinyusb)
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "include_private"
                       PRIV_REQUIRES ${priv_req}
                       REQUIRES ${req}
                       )

# Determine whether tinyusb is fetched from component registry or from local path
idf_build_get_property(build_components BUILD_COMPONENTS)
if(tinyusb IN_LIST build_components)
    set(tinyusb_name tinyusb) # Local component
else()
    set(tinyusb_name espressif__tinyusb) # Managed component
endif()

# Pass tusb_config.h from this component to TinyUSB
idf_component_get_property(tusb_lib ${tinyusb_name} COMPONENT_LIB)
target_include_directories(${tusb_lib} PRIVATE "include")

if(CONFIG_TINYUSB_MSC_ENABLED)
    # Replace only the MSC translation unit. The bridge includes the original
    # pinned source, so component-manager downloads are never edited in place.
    get_target_property(tinyusb_sources ${tusb_lib} SOURCES)
    get_target_property(tinyusb_source_dir ${tusb_lib} SOURCE_DIR)
    set(tinyusb_msc_source "")
    foreach(source IN LISTS tinyusb_sources)
        if(source MATCHES "(^|/)msc_device\\.c$")
            if(tinyusb_msc_source)
                message(FATAL_ERROR "Multiple TinyUSB MSC drivers found")
            endif()
            get_filename_component(tinyusb_msc_source "${source}" ABSOLUTE
                                   BASE_DIR "${tinyusb_source_dir}")
            list(REMOVE_ITEM tinyusb_sources "${source}")
        endif()
    endforeach()
    if(NOT EXISTS "${tinyusb_msc_source}")
        message(FATAL_ERROR "Pinned TinyUSB MSC driver not found for reset bridge")
    endif()
    configure_file(msc_device_bridge.c.in
                   "${CMAKE_CURRENT_BINARY_DIR}/msc_device_bridge.c" @ONLY)
    set_property(TARGET ${tusb_lib} PROPERTY SOURCES "${tinyusb_sources}")
    target_sources(${tusb_lib} PRIVATE
                   "${CMAKE_CURRENT_BINARY_DIR}/msc_device_bridge.c")
    target_include_directories(${tusb_lib} PRIVATE
                               "${CMAKE_CURRENT_SOURCE_DIR}/include_private")
endif()
