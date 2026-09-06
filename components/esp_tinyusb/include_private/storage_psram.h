/*
 * SPDX-FileCopyrightText: 2026 CloudBaseVario contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "msc_storage.h"
#include "tinyusb_msc.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t storage_psram_open_medium(
    const tinyusb_msc_psram_config_t *config,
    const storage_medium_t **medium);

void storage_psram_get_diagnostics(
    tinyusb_msc_psram_diagnostics_t *diagnostics);

#ifdef __cplusplus
}
#endif
