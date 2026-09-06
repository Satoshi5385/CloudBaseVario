/*
 * SPDX-FileCopyrightText: 2026 CloudBaseVario contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "storage_psram_buffer.h"

#include <stdint.h>
#include <string.h>

bool storage_psram_buffer_range_valid(
    const storage_psram_buffer_t *buffer, uint32_t lba, uint32_t offset,
    size_t size, size_t *address) {
    size_t sector_offset;
    size_t end;

    if (buffer == NULL || buffer->data == NULL ||
        buffer->sector_size == 0U || address == NULL ||
        ((size_t) lba != 0U &&
         (size_t) buffer->sector_size > SIZE_MAX / (size_t) lba)) {
        return false;
    }
    sector_offset = (size_t) lba * (size_t) buffer->sector_size;
    if ((size_t) offset > SIZE_MAX - sector_offset) {
        return false;
    }
    *address = sector_offset + (size_t) offset;
    if (size > SIZE_MAX - *address) {
        return false;
    }
    end = *address + size;
    return end <= buffer->size_bytes;
}

bool storage_psram_buffer_read(
    const storage_psram_buffer_t *buffer, uint32_t lba, uint32_t offset,
    void *destination, size_t size) {
    size_t address;

    if (destination == NULL ||
        !storage_psram_buffer_range_valid(buffer, lba, offset, size,
                                          &address)) {
        return false;
    }
    memcpy(destination, buffer->data + address, size);
    return true;
}

bool storage_psram_buffer_write(
    const storage_psram_buffer_t *buffer, uint32_t lba, uint32_t offset,
    const void *source, size_t size) {
    size_t address;

    if (source == NULL ||
        !storage_psram_buffer_range_valid(buffer, lba, offset, size,
                                          &address)) {
        return false;
    }
    memcpy(buffer->data + address, source, size);
    return true;
}
