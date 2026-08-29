#include <assert.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "domain/gps_nmea.h"

static void sentence(const char *body,
                     char output[GPS_NMEA_SENTENCE_CAPACITY]) {
    unsigned checksum = 0U;

    for (const char *cursor = body; *cursor != '\0'; cursor++) {
        checksum ^= (unsigned char) *cursor;
    }
    assert(snprintf(output, GPS_NMEA_SENTENCE_CAPACITY, "$%s*%02X",
                    body, checksum) > 0);
}

static void test_gp_pair_and_fix(void) {
    gps_nmea_pairer_t pairer = {0};
    char input[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    char rmc[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    char gga[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    bool fix = false;
    gps_nmea_fix_t decoded = {0};

    gps_nmea_pairer_init(&pairer);
    sentence("GPRMC,123519,A,4807.038,N,01131.000,E,0.0,0.0,010100,,,A",
             input);
    assert(gps_nmea_pairer_consume(&pairer, input, rmc, gga, &fix) ==
           GPS_NMEA_PENDING);
    sentence("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,",
             input);
    assert(gps_nmea_pairer_consume(&pairer, input, rmc, gga, &fix) ==
           GPS_NMEA_PAIR_READY);
    assert(strncmp(rmc, "$GPRMC,", 7U) == 0);
    assert(strncmp(gga, "$GPGGA,", 7U) == 0);
    assert(strstr(rmc, "\r\n") != NULL);
    assert(fix);
    assert(gps_nmea_parse_fix_pair(rmc, gga, &decoded));
    assert(decoded.fix_valid);
    assert(decoded.utc_valid);
    assert(strcmp(decoded.utc, "123519") == 0);
    assert(fabs(decoded.latitude_deg - 48.1173) < 0.000001);
    assert(fabs(decoded.longitude_deg - 11.5166667) < 0.000001);
    assert(fabs(decoded.altitude_m - 545.4) < 0.001);
    assert(decoded.satellites == 8U);
    assert(fabs(decoded.hdop - 0.9) < 0.001);
}

static void test_fix_values_south_west_and_speed(void) {
    char rmc[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    char gga[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    gps_nmea_fix_t fix = {0};

    sentence("GNRMC,010203.50,A,3456.789,S,13830.000,W,10.0,270.0,010100,,,A",
             rmc);
    sentence("GNGGA,010203.50,3456.789,S,13830.000,W,1,12,0.8,25.5,M,0,M,,",
             gga);
    assert(gps_nmea_parse_fix_pair(rmc, gga, &fix));
    assert(fix.fix_valid);
    assert(fix.position_valid);
    assert(fabs(fix.latitude_deg + 34.9464833) < 0.000001);
    assert(fabs(fix.longitude_deg + 138.5) < 0.000001);
    assert(fabs(fix.speed_kmh - 18.52) < 0.001);
    assert(fabs(fix.course_deg - 270.0) < 0.001);
}

static void test_no_fix_and_optional_fields(void) {
    char rmc[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    char gga[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    gps_nmea_fix_t fix = {0};

    sentence("GPRMC,010203,V,0,N,0,E,0,0,010100,,,N", rmc);
    sentence("GPGGA,010203,0,N,0,E,0,03,9.9,0,M,0,M,,", gga);
    assert(gps_nmea_parse_fix_pair(rmc, gga, &fix));
    assert(!fix.fix_valid);
    assert(fix.utc_valid);
    assert(fix.satellites_valid && fix.satellites == 3U);
    assert(fix.hdop_valid);
    assert(!fix.position_valid);
    assert(!fix.altitude_valid);
    assert(!fix.speed_valid);
    assert(!fix.course_valid);

    sentence("GPRMC,010203,A,3500.000,N,13900.000,E,0.0,0.0,010100,,,A",
             rmc);
    sentence("GPGGA,010203,3500.000,N,13900.000,E,1,08,0.9,,M,0,M,,",
             gga);
    assert(gps_nmea_parse_fix_pair(rmc, gga, &fix));
    assert(fix.fix_valid);
    assert(!fix.altitude_valid);
}

static void test_fix_rejects_bad_pair_and_range(void) {
    char rmc[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    char gga[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    gps_nmea_fix_t fix = {0};

    sentence("GPRMC,010203,A,3500.000,N,13900.000,E,0.0,0.0,010100,,,A",
             rmc);
    sentence("GPGGA,010204,3500.000,N,13900.000,E,1,08,0.9,1.0,M,0,M,,",
             gga);
    assert(!gps_nmea_parse_fix_pair(rmc, gga, &fix));
    sentence("GPGGA,010203,3500.000,N,13900.000,E,1,08,0.9,1.0,M,0,M,,",
             gga);
    if (rmc[strlen(rmc) - 1U] == '0') {
        rmc[strlen(rmc) - 1U] = '1';
    } else {
        rmc[strlen(rmc) - 1U] = '0';
    }
    assert(!gps_nmea_parse_fix_pair(rmc, gga, &fix));

    sentence("GPRMC,010203,A,9100.000,N,13900.000,E,0.0,0.0,010100,,,A",
             rmc);
    assert(gps_nmea_parse_fix_pair(rmc, gga, &fix));
    assert(!fix.fix_valid);
    assert(!fix.position_valid);
}

static void test_gn_mismatch_invalid_and_no_fix(void) {
    gps_nmea_pairer_t pairer = {0};
    char input[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    char rmc[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    char gga[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    bool fix = true;

    gps_nmea_pairer_init(&pairer);
    sentence("GNRMC,010203,V,0,N,0,E,0,0,010100,,,N", input);
    assert(gps_nmea_pairer_consume(&pairer, input, rmc, gga, &fix) ==
           GPS_NMEA_PENDING);
    sentence("GNGGA,010204,0,N,0,E,0,00,9.9,0,M,0,M,,", input);
    assert(gps_nmea_pairer_consume(&pairer, input, rmc, gga, &fix) ==
           GPS_NMEA_PENDING);
    sentence("GNGGA,010203,0,N,0,E,0,00,9.9,0,M,0,M,,", input);
    assert(gps_nmea_pairer_consume(&pairer, input, rmc, gga, &fix) ==
           GPS_NMEA_PAIR_READY);
    assert(!fix);
    input[strlen(input) - 1U] = input[strlen(input) - 1U] == '0' ? '1' : '0';
    assert(!gps_nmea_checksum_valid(input));
    assert(gps_nmea_pairer_consume(&pairer, input, rmc, gga, &fix) ==
           GPS_NMEA_INVALID);
    sentence("PMTK001,220,3", input);
    assert(gps_nmea_pairer_consume(&pairer, input, rmc, gga, &fix) ==
           GPS_NMEA_IGNORED);
}

static void test_pmtk_standard_and_extended_ack(void) {
    char input[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    gps_pmtk_ack_t ack = {0};

    sentence("PMTK001,220,3", input);
    assert(gps_pmtk_ack_parse(input, &ack));
    assert(ack.command == 220U);
    assert(ack.flag == 3U);
    assert(ack.value_count == 0U);

    sentence("PMTK001,353,3,1,1,0,0,0,3", input);
    assert(gps_pmtk_ack_parse(input, &ack));
    assert(ack.command == 353U);
    assert(ack.flag == 3U);
    assert(ack.value_count == 6U);
    assert(ack.values[0] == 1U);
    assert(ack.values[1] == 1U);
}

static void test_pmtk_ack_rejects_bad_frames(void) {
    char input[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    gps_pmtk_ack_t ack = {0};

    sentence("PMTK001,353,4,1,1,0,0,0", input);
    assert(!gps_pmtk_ack_parse(input, &ack));
    sentence("PMTK001,353,3,1,1,0,0,0", input);
    input[strlen(input) - 1U] = '0';
    assert(!gps_pmtk_ack_parse(input, &ack));
    sentence("PMTK705,L96", input);
    assert(!gps_pmtk_ack_parse(input, &ack));
}

int main(void) {
    test_gp_pair_and_fix();
    test_fix_values_south_west_and_speed();
    test_no_fix_and_optional_fields();
    test_fix_rejects_bad_pair_and_range();
    test_gn_mismatch_invalid_and_no_fix();
    test_pmtk_standard_and_extended_ack();
    test_pmtk_ack_rejects_bad_frames();
    return 0;
}
