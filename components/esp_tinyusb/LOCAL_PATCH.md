# Local esp_tinyusb patch

This directory vendors Espressif `esp_tinyusb` 2.2.1 so the MSC durability fix
is reproducible and is not overwritten when managed components are resolved.

The local change in `tinyusb_msc.c` reserves the single WRITE(10) buffer
before copying its payload or address metadata. The reservation prevents another
submission, ownership transition, or deletion until both the medium write and
its queued USB completion have drained. A separate ready flag publishes the
filled buffer to `tinyusb_msc_io`; the worker never repeats a write just because
its completion is still queued. Flash I/O and sensor/audio quiescence stay on
that worker, outside the TinyUSB event task.

`msc_device_bridge.c.in` is compiled in place of the pinned TinyUSB core MSC
translation unit and includes the original source without modifying the managed
component. It preserves normal protocol processing while invalidating write
identities on bus reset, valid BOT reset, and driver init/deinit. BOT reset also
clears `pending_io`, which the pinned upstream implementation leaves set. The
worker queues one completion into the TinyUSB task; the bridge checks the request
identity and completes inline in that same event. Calling the upstream
`tud_msc_async_io_done()` here would queue a second unguarded event and reopen the
reset race. A reset discards the old USB acknowledgement, but the already accepted
medium write is allowed to finish with its original buffer.

SYNCHRONIZE CACHE and eject still drain accepted writes, including queued USB
completion. A callback lifetime flag also prevents stopping/deleting storage
while its completion notification or ownership transition is executing.
The TinyUSB core remains pinned to 0.21.0~1: the bridge deliberately uses its
internal MSC state and must be reviewed when that dependency changes. CMake
fails if it cannot find exactly one MSC translation unit to replace.

`tests/test_usb_msc_write_runtime.py` compiles the production submit/worker/
completion functions and reset bridge against deterministic RTOS/medium/protocol
fakes. It exercises rejection without buffer modification, stop during copy,
bus/BOT reset before completion, queued-completion drain, worker non-repetition,
write errors, and restart. Firmware CI separately compiles the bridge against
the actual pinned dependency. Physical USB/Flash corruption still requires
hardware validation with repeated large-file copies and hash comparison.

The local component also adds `tinyusb_msc_new_storage_psram()`. The backend
registers a caller-owned byte-addressable PSRAM buffer with FatFs, validates
the complete LBA/offset/length range for every request, and keeps TinyUSB's
existing 4 KiB internal DMA-capable transfer buffer between USB and PSRAM.
CloudBaseVario uses this backend only for the volatile recovery LUN.
Formatting always passes the allocated FatFs drive name to `f_mkfs()` and
force-mounts that same drive afterward. This is required because recovery keeps
the private config Flash on drive 0 while the PSRAM disk uses drive 1; an empty
format path would otherwise select and erase drive 0.
