#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "domain/app_types.h"

typedef enum {
    GPS_NMEA_IGNORED = 0,
    GPS_NMEA_INVALID,
    GPS_NMEA_PENDING,
    GPS_NMEA_PAIR_READY,
} gps_nmea_result_t;

#define GPS_PMTK_MAX_ACK_VALUES 8U

typedef struct {
    unsigned command;
    unsigned flag;
    unsigned values[GPS_PMTK_MAX_ACK_VALUES];
    size_t value_count;
} gps_pmtk_ack_t;

typedef struct {
    bool fix_valid;
    bool utc_valid;
    bool position_valid;
    bool altitude_valid;
    bool satellites_valid;
    bool hdop_valid;
    bool speed_valid;
    bool course_valid;
    uint8_t satellites;
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    double hdop;
    double speed_kmh;
    double course_deg;
    char utc[16];
} gps_nmea_fix_t;

typedef struct {
    char rmc[GPS_NMEA_SENTENCE_CAPACITY];
    char gga[GPS_NMEA_SENTENCE_CAPACITY];
    char rmc_utc[16];
    char gga_utc[16];
    bool rmc_fix_valid;
    bool gga_fix_valid;
    bool have_rmc;
    bool have_gga;
} gps_nmea_pairer_t;

/** Initialize an empty latest-value RMC/GGA pairer. */
void gps_nmea_pairer_init(gps_nmea_pairer_t *pairer);

/** Validate the NMEA XOR checksum and strict $...*HH framing. */
bool gps_nmea_checksum_valid(const char *sentence);

/** Parse a checksum-valid standard or extended $PMTK001 acknowledgement. */
bool gps_pmtk_ack_parse(const char *sentence, gps_pmtk_ack_t *ack);

/** Decode positioning values from a checksum-valid, equal-UTC RMC/GGA pair. */
bool gps_nmea_parse_fix_pair(const char *rmc, const char *gga,
                             gps_nmea_fix_t *fix);

/**
 * Consume one line without CR/LF and return a coherent GP/GN RMC/GGA pair.
 * Output sentences are normalized to CRLF and retain their original checksum.
 */
gps_nmea_result_t gps_nmea_pairer_consume(
    gps_nmea_pairer_t *pairer, const char *sentence,
    char rmc[GPS_NMEA_SENTENCE_CAPACITY],
    char gga[GPS_NMEA_SENTENCE_CAPACITY], bool *fix_valid);
