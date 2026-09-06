/*
 * SPDX-FileCopyrightText: 2026 CloudBaseVario contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t *data;
    size_t size_bytes;
    uint32_t sector_size;
} storage_psram_buffer_t;

bool storage_psram_buffer_range_valid(
    const storage_psram_buffer_t *buffer, uint32_t lba, uint32_t offset,
    size_t size, size_t *address);

bool storage_psram_buffer_read(
    const storage_psram_buffer_t *buffer, uint32_t lba, uint32_t offset,
    void *destination, size_t size);

bool storage_psram_buffer_write(
    const storage_psram_buffer_t *buffer, uint32_t lba, uint32_t offset,
    const void *source, size_t size);
