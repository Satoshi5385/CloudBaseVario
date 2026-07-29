# Local esp_tinyusb patch

This directory vendors Espressif `esp_tinyusb` 2.2.1 so the MSC durability fix
is reproducible and is not overwritten when managed components are resolved.

The local change in `tinyusb_msc.c` keeps each WRITE(10) command asynchronous
until the SPI flash Wear Levelling write has completed, then reports completion
with `tud_msc_async_io_done()`. It also accepts SYNCHRONIZE CACHE(10/16), for
which no further flush is needed because no successful WRITE(10) remains
pending. The TinyUSB core dependency is pinned to 0.21.0~1 because this patch
depends on its asynchronous MSC completion API.
