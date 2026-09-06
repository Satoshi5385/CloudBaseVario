# Local esp_tinyusb patch

This directory vendors Espressif `esp_tinyusb` 2.2.1 so the MSC durability fix
is reproducible and is not overwritten when managed components are resolved.

The local change in `tinyusb_msc.c` keeps each WRITE(10) command asynchronous
until the SPI flash Wear Levelling write has completed, then reports completion
with `tud_msc_async_io_done()`. A dedicated `tinyusb_msc_io` worker performs the
blocking storage-mode preparation and medium write so the TinyUSB event task
never waits for sensor/audio quiescence or flash I/O. SYNCHRONIZE CACHE(10/16)
succeeds only after the accepted-write count reaches zero and the medium mutex
can be acquired. Eject and detach close new host I/O first and defer application
mounting until any accepted write and its TinyUSB asynchronous completion have
drained. The TinyUSB core dependency is pinned to 0.21.0~1 because this patch
depends on its asynchronous MSC completion API.

The local component also adds `tinyusb_msc_new_storage_psram()`. The backend
registers a caller-owned byte-addressable PSRAM buffer with FatFs, validates
the complete LBA/offset/length range for every request, and keeps TinyUSB's
existing 4 KiB internal DMA-capable transfer buffer between USB and PSRAM.
CloudBaseVario uses this backend only for the volatile recovery LUN.
Formatting always passes the allocated FatFs drive name to `f_mkfs()` and
force-mounts that same drive afterward. This is required because recovery keeps
the private config Flash on drive 0 while the PSRAM disk uses drive 1; an empty
format path would otherwise select and erase drive 0.
