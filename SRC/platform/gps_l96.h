#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/** Detect and configure an installed Quectel L96-M33 on UART1. */
esp_err_t gps_l96_connect(uint32_t interval_ms, uint32_t *baud_rate);

/** Apply a validated 200-10000 ms NMEA update period and require PMTK ACK. */
esp_err_t gps_l96_set_interval(uint32_t interval_ms);

/** Read one CR/LF-delimited UART line into a NUL-terminated buffer. */
esp_err_t gps_l96_read_line(char *line, size_t capacity,
                            uint32_t timeout_ms);

/** Release UART1 and the GPS pins. Safe to call when already stopped. */
void gps_l96_deinit(void);
