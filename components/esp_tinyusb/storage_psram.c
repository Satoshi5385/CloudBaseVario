/*
 * SPDX-FileCopyrightText: 2026 CloudBaseVario contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "storage_psram.h"
#include "storage_psram_buffer.h"

#include <assert.h>
#include <string.h>

#include "diskio_impl.h"
#include "esp_check.h"
#include "esp_log.h"
#include "ff.h"

static const char *TAG = "storage_psram";
static uint8_t *psram_buffer;
static size_t psram_size;
static uint32_t psram_sector_size;
static BYTE psram_pdrv = TINYUSB_MSC_PDRV_INVALID;
static tinyusb_msc_psram_diagnostics_t psram_diagnostics;

static storage_psram_buffer_t psram_view(void) {
    const storage_psram_buffer_t view = {
        .data = psram_buffer,
        .size_bytes = psram_size,
        .sector_size = psram_sector_size,
    };

    return view;
}

static DSTATUS psram_disk_initialize(BYTE pdrv) {
    return pdrv == psram_pdrv && psram_buffer != NULL ? 0U : STA_NOINIT;
}

static DSTATUS psram_disk_status(BYTE pdrv) {
    return psram_disk_initialize(pdrv);
}

static DRESULT psram_disk_read(BYTE pdrv, BYTE *buffer, DWORD sector,
                               UINT count) {
    const storage_psram_buffer_t view = psram_view();
    size_t length;

    if (pdrv != psram_pdrv || buffer == NULL || count == 0U ||
        __builtin_umul_overflow((size_t) count,
                               (size_t) psram_sector_size, &length) ||
        !storage_psram_buffer_read(&view, sector, 0U, buffer, length)) {
        psram_diagnostics.read_error_count += 1U;
        return RES_PARERR;
    }
    psram_diagnostics.read_count += 1U;
    psram_diagnostics.read_bytes += length;
    return RES_OK;
}

static DRESULT psram_disk_write(BYTE pdrv, const BYTE *buffer, DWORD sector,
                                UINT count) {
    const storage_psram_buffer_t view = psram_view();
    size_t length;

    if (pdrv != psram_pdrv || buffer == NULL || count == 0U ||
        __builtin_umul_overflow((size_t) count,
                               (size_t) psram_sector_size, &length) ||
        !storage_psram_buffer_write(&view, sector, 0U, buffer, length)) {
        psram_diagnostics.write_error_count += 1U;
        return RES_PARERR;
    }
    psram_diagnostics.write_count += 1U;
    psram_diagnostics.written_bytes += length;
    return RES_OK;
}

static DRESULT psram_disk_ioctl(BYTE pdrv, BYTE command, void *buffer) {
    if (pdrv != psram_pdrv) {
        return RES_PARERR;
    }
    if (command == CTRL_SYNC) {
        return RES_OK;
    }
    if (buffer == NULL) {
        return RES_PARERR;
    }
    switch (command) {
    case GET_SECTOR_COUNT:
        *((DWORD *) buffer) = (DWORD) (psram_size / psram_sector_size);
        return RES_OK;
    case GET_SECTOR_SIZE:
        *((WORD *) buffer) = (WORD) psram_sector_size;
        return RES_OK;
    case GET_BLOCK_SIZE:
        *((DWORD *) buffer) = 1U;
        return RES_OK;
    default:
        return RES_PARERR;
    }
}

static const ff_diskio_impl_t psram_diskio = {
    .init = psram_disk_initialize,
    .status = psram_disk_status,
    .read = psram_disk_read,
    .write = psram_disk_write,
    .ioctl = psram_disk_ioctl,
};

static esp_err_t psram_mount(BYTE pdrv) {
    ESP_RETURN_ON_FALSE(psram_buffer != NULL &&
                            psram_pdrv == TINYUSB_MSC_PDRV_INVALID,
                        ESP_ERR_INVALID_STATE, TAG,
                        "PSRAM medium is unavailable or already mounted");
    psram_pdrv = pdrv;
    psram_diagnostics.drive_number = pdrv;
    ff_diskio_register(pdrv, &psram_diskio);
    return ESP_OK;
}

static esp_err_t psram_unmount(void) {
    char drive[3];

    ESP_RETURN_ON_FALSE(psram_pdrv != TINYUSB_MSC_PDRV_INVALID,
                        ESP_ERR_INVALID_STATE, TAG,
                        "PSRAM medium is not mounted");
    drive[0] = (char) ('0' + psram_pdrv);
    drive[1] = ':';
    drive[2] = '\0';
    (void) f_mount(NULL, drive, 0U);
    ff_diskio_unregister(psram_pdrv);
    psram_pdrv = TINYUSB_MSC_PDRV_INVALID;
    psram_diagnostics.drive_number = TINYUSB_MSC_PDRV_INVALID;
    return ESP_OK;
}

static esp_err_t psram_read(uint32_t lba, uint32_t offset, size_t size,
                            void *destination) {
    const storage_psram_buffer_t view = psram_view();

    if (!storage_psram_buffer_read(&view, lba, offset, destination, size)) {
        psram_diagnostics.read_error_count += 1U;
        return ESP_ERR_INVALID_SIZE;
    }
    psram_diagnostics.read_count += 1U;
    psram_diagnostics.read_bytes += size;
    return ESP_OK;
}

static esp_err_t psram_write(uint32_t lba, uint32_t offset, size_t size,
                             const void *source) {
    const storage_psram_buffer_t view = psram_view();

    if (!storage_psram_buffer_write(&view, lba, offset, source, size)) {
        psram_diagnostics.write_error_count += 1U;
        return ESP_ERR_INVALID_SIZE;
    }
    psram_diagnostics.write_count += 1U;
    psram_diagnostics.written_bytes += size;
    return ESP_OK;
}

static esp_err_t psram_get_info(storage_info_t *info) {
    ESP_RETURN_ON_FALSE(info != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Storage info pointer is NULL");
    ESP_RETURN_ON_FALSE(psram_buffer != NULL && psram_sector_size != 0U,
                        ESP_ERR_INVALID_STATE, TAG,
                        "PSRAM medium is not open");
    info->total_sectors = (uint32_t) (psram_size / psram_sector_size);
    info->sector_size = psram_sector_size;
    return ESP_OK;
}

static void psram_close(void) {
    psram_buffer = NULL;
    psram_size = 0U;
    psram_sector_size = 0U;
    psram_pdrv = TINYUSB_MSC_PDRV_INVALID;
}

static const storage_medium_t psram_medium = {
    .type = STORAGE_MEDIUM_TYPE_PSRAM,
    .mount = psram_mount,
    .unmount = psram_unmount,
    .read = psram_read,
    .write = psram_write,
    .get_info = psram_get_info,
    .close = psram_close,
};

esp_err_t storage_psram_open_medium(
    const tinyusb_msc_psram_config_t *config,
    const storage_medium_t **medium) {
    ESP_RETURN_ON_FALSE(config != NULL && medium != NULL,
                        ESP_ERR_INVALID_ARG, TAG, "Invalid PSRAM config");
    ESP_RETURN_ON_FALSE(config->buffer != NULL && config->size_bytes > 0U &&
                            config->sector_size > 0U &&
                            config->size_bytes % config->sector_size == 0U &&
                            config->sector_size <= UINT16_MAX,
                        ESP_ERR_INVALID_ARG, TAG,
                        "Invalid PSRAM geometry");
    ESP_RETURN_ON_FALSE(psram_buffer == NULL, ESP_ERR_INVALID_STATE, TAG,
                        "Only one PSRAM medium is supported");

    psram_buffer = config->buffer;
    psram_size = config->size_bytes;
    psram_sector_size = config->sector_size;
    memset(&psram_diagnostics, 0, sizeof(psram_diagnostics));
    psram_diagnostics.drive_number = TINYUSB_MSC_PDRV_INVALID;
    *medium = &psram_medium;
    return ESP_OK;
}

void storage_psram_get_diagnostics(
    tinyusb_msc_psram_diagnostics_t *diagnostics) {
    if (diagnostics != NULL) {
        *diagnostics = psram_diagnostics;
    }
}
