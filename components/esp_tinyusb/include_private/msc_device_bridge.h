#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Begin an accepted WRITE callback. TinyUSB task only; returns its identity. */
uint64_t msc_device_begin_write(void);

/** Complete inline in the TinyUSB task, only if reset has not invalidated it. */
bool msc_device_complete_write(uint64_t token, uint8_t lun, int32_t bytes);
