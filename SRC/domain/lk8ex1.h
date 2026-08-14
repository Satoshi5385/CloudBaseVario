#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "domain/app_config.h"
#include "domain/app_types.h"

#define LK8EX1_SENTENCE_MAX_LENGTH 104U

typedef struct {
    char raw_pressure[16];
    char altitude[16];
    char vario[16];
    char temperature[16];
    char battery[16];
    bool sentence_available;
} lk8ex1_fields_t;

/** Select and format the five LK8EX1 data fields. */
bool lk8ex1_format_fields(const vario_result_t *vario,
                          const system_snapshot_t *system,
                          app_bluetooth_battery_mode_t battery_mode,
                          lk8ex1_fields_t *fields);

/** Format one complete checksum-protected LK8EX1 sentence. */
bool lk8ex1_format_sentence(const vario_result_t *vario,
                            const system_snapshot_t *system,
                            app_bluetooth_battery_mode_t battery_mode,
                            char *sentence, size_t capacity,
                            size_t *length);
