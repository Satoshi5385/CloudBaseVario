#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "storage_psram_buffer.h"

#define DISK_BYTES (4U * 1024U * 1024U)
#define SECTOR_BYTES 512U
#define SECTOR_COUNT 8192U

int main(void) {
    uint8_t *memory = calloc(1U, DISK_BYTES);
    storage_psram_buffer_t disk = {
        .data = memory,
        .size_bytes = DISK_BYTES,
        .sector_size = SECTOR_BYTES,
    };
    uint8_t source[19];
    uint8_t destination[19];
    size_t address = 0U;

    assert(memory != NULL);
    for (size_t index = 0U; index < DISK_BYTES; index++) {
        assert(memory[index] == 0U);
    }
    for (size_t index = 0U; index < sizeof(source); index++) {
        source[index] = (uint8_t) (index + 1U);
    }

    assert(storage_psram_buffer_range_valid(
        &disk, 0U, 0U, SECTOR_BYTES, &address));
    assert(address == 0U);
    assert(storage_psram_buffer_write(
        &disk, 3U, 17U, source, sizeof(source)));
    memset(destination, 0, sizeof(destination));
    assert(storage_psram_buffer_read(
        &disk, 3U, 17U, destination, sizeof(destination)));
    assert(memcmp(source, destination, sizeof(source)) == 0);

    assert(storage_psram_buffer_write(
        &disk, SECTOR_COUNT - 1U, SECTOR_BYTES - 1U, source, 1U));
    assert(memory[DISK_BYTES - 1U] == source[0]);
    assert(!storage_psram_buffer_write(
        &disk, SECTOR_COUNT - 1U, SECTOR_BYTES - 1U, source, 2U));
    assert(!storage_psram_buffer_read(
        &disk, SECTOR_COUNT, 0U, destination, 1U));
    assert(!storage_psram_buffer_range_valid(
        &disk, UINT32_MAX, UINT32_MAX, SIZE_MAX, &address));
    assert(!storage_psram_buffer_range_valid(
        NULL, 0U, 0U, 1U, &address));

    free(memory);
    puts("PSRAM storage buffer tests passed");
    return 0;
}
