#include "domain/gps_nmea.h"

#include <errno.h>
#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
    GPS_SENTENCE_OTHER = 0,
    GPS_SENTENCE_RMC,
    GPS_SENTENCE_GGA,
} gps_sentence_kind_t;

static int hex_value(char value) {
    if (value >= '0' && value <= '9') {
        return value - '0';
    }
    value = (char) toupper((unsigned char) value);
    if (value >= 'A' && value <= 'F') {
        return value - 'A' + 10;
    }
    return -1;
}

bool gps_nmea_checksum_valid(const char *sentence) {
    const char *checksum = NULL;
    unsigned char calculated = 0U;
    int high = 0;
    int low = 0;

    if (sentence == NULL || sentence[0] != '$') {
        return false;
    }
    checksum = strrchr(sentence, '*');
    if (checksum == NULL || checksum <= sentence + 1 ||
        checksum[1] == '\0' || checksum[2] == '\0' ||
        checksum[3] != '\0') {
        return false;
    }
    high = hex_value(checksum[1]);
    low = hex_value(checksum[2]);
    if (high < 0 || low < 0) {
        return false;
    }
    for (const char *cursor = sentence + 1; cursor < checksum; cursor++) {
        calculated ^= (unsigned char) *cursor;
    }
    return calculated == (unsigned char) ((high << 4) | low);
}

static bool parse_unsigned_field(const char **cursor, unsigned *value) {
    const char *current = NULL;
    unsigned parsed = 0U;
    bool have_digit = false;

    if (cursor == NULL || *cursor == NULL || value == NULL) {
        return false;
    }
    current = *cursor;
    while (*current >= '0' && *current <= '9') {
        unsigned digit = (unsigned) (*current - '0');

        if (parsed > (UINT_MAX - digit) / 10U) {
            return false;
        }
        parsed = parsed * 10U + digit;
        have_digit = true;
        current++;
    }
    if (!have_digit) {
        return false;
    }
    *cursor = current;
    *value = parsed;
    return true;
}

bool gps_pmtk_ack_parse(const char *sentence, gps_pmtk_ack_t *ack) {
    const char *cursor = NULL;
    gps_pmtk_ack_t parsed = {0};

    if (sentence == NULL || ack == NULL ||
        strncmp(sentence, "$PMTK001,", 9U) != 0 ||
        !gps_nmea_checksum_valid(sentence)) {
        return false;
    }
    cursor = sentence + 9U;
    if (!parse_unsigned_field(&cursor, &parsed.command) || *cursor != ',') {
        return false;
    }
    cursor++;
    if (!parse_unsigned_field(&cursor, &parsed.flag) || parsed.flag > 3U) {
        return false;
    }
    while (*cursor == ',') {
        if (parsed.value_count >= GPS_PMTK_MAX_ACK_VALUES) {
            return false;
        }
        cursor++;
        if (!parse_unsigned_field(&cursor,
                                  &parsed.values[parsed.value_count])) {
            return false;
        }
        parsed.value_count++;
    }
    if (*cursor != '*') {
        return false;
    }
    *ack = parsed;
    return true;
}

static gps_sentence_kind_t sentence_kind(const char *sentence) {
    if (sentence == NULL || strlen(sentence) < 7U) {
        return GPS_SENTENCE_OTHER;
    }
    if ((memcmp(sentence, "$GPRMC,", 7U) == 0) ||
        (memcmp(sentence, "$GNRMC,", 7U) == 0)) {
        return GPS_SENTENCE_RMC;
    }
    if ((memcmp(sentence, "$GPGGA,", 7U) == 0) ||
        (memcmp(sentence, "$GNGGA,", 7U) == 0)) {
        return GPS_SENTENCE_GGA;
    }
    return GPS_SENTENCE_OTHER;
}

static bool copy_field(const char *sentence, size_t field_number,
                       char *output, size_t output_size) {
    const char *start = sentence;
    const char *end = NULL;

    if (sentence == NULL || output == NULL || output_size == 0U) {
        return false;
    }
    for (size_t field = 0U; field < field_number; field++) {
        start = strchr(start, ',');
        if (start == NULL) {
            return false;
        }
        start++;
    }
    end = strpbrk(start, ",*");
    if (end == NULL || end == start || (size_t) (end - start) >= output_size) {
        return false;
    }
    memcpy(output, start, (size_t) (end - start));
    output[end - start] = '\0';
    return true;
}

static bool decimal_text_valid(const char *text) {
    const char *cursor = text;
    bool have_digit = false;
    bool have_decimal_point = false;

    if (*cursor == '+' || *cursor == '-') {
        cursor++;
    }
    while (*cursor != '\0') {
        if (*cursor >= '0' && *cursor <= '9') {
            have_digit = true;
        } else if (*cursor == '.' && !have_decimal_point) {
            have_decimal_point = true;
        } else {
            return false;
        }
        cursor++;
    }
    return have_digit;
}

static bool parse_double_field(const char *sentence, size_t field_number,
                               double *value) {
    char field[32] = {0};
    char *end = NULL;
    double parsed = 0.0;

    if (value == NULL ||
        !copy_field(sentence, field_number, field, sizeof(field)) ||
        !decimal_text_valid(field)) {
        return false;
    }
    errno = 0;
    parsed = strtod(field, &end);
    if (errno != 0 || end == field || *end != '\0' || !isfinite(parsed)) {
        return false;
    }
    *value = parsed;
    return true;
}

static bool parse_uint8_field(const char *sentence, size_t field_number,
                              uint8_t *value) {
    char field[8] = {0};
    char *end = NULL;
    unsigned long parsed = 0UL;

    if (value == NULL ||
        !copy_field(sentence, field_number, field, sizeof(field))) {
        return false;
    }
    for (const char *cursor = field; *cursor != '\0'; cursor++) {
        if (*cursor < '0' || *cursor > '9') {
            return false;
        }
    }
    errno = 0;
    parsed = strtoul(field, &end, 10);
    if (errno != 0 || end == field || *end != '\0' || parsed > UINT8_MAX) {
        return false;
    }
    *value = (uint8_t) parsed;
    return true;
}

static bool parse_utc(const char *sentence, char utc[16]) {
    char field[16] = {0};
    char *end = NULL;
    double parsed = 0.0;
    unsigned hours = 0U;
    unsigned minutes = 0U;
    double seconds = 0.0;

    if (!copy_field(sentence, 1U, field, sizeof(field)) ||
        strlen(field) < 6U || !decimal_text_valid(field)) {
        return false;
    }
    errno = 0;
    parsed = strtod(field, &end);
    if (errno != 0 || end == field || *end != '\0' || !isfinite(parsed) ||
        parsed < 0.0 || parsed >= 240000.0) {
        return false;
    }
    hours = (unsigned) (parsed / 10000.0);
    minutes = (unsigned) ((parsed - (double) hours * 10000.0) / 100.0);
    seconds = parsed - (double) hours * 10000.0 -
              (double) minutes * 100.0;
    if (hours > 23U || minutes > 59U || seconds >= 60.0) {
        return false;
    }
    memcpy(utc, field, strlen(field) + 1U);
    return true;
}

static bool copy_without_line_ending(
    const char *source, char destination[GPS_NMEA_SENTENCE_CAPACITY]) {
    size_t length = 0U;

    if (source == NULL) {
        return false;
    }
    length = strlen(source);
    while (length > 0U &&
           (source[length - 1U] == '\r' || source[length - 1U] == '\n')) {
        length--;
    }
    if (length == 0U || length >= GPS_NMEA_SENTENCE_CAPACITY) {
        return false;
    }
    memcpy(destination, source, length);
    destination[length] = '\0';
    return true;
}

static bool parse_coordinate(const char *sentence, size_t value_field,
                             size_t hemisphere_field, unsigned degree_limit,
                             char positive_hemisphere,
                             char negative_hemisphere,
                             double *coordinate) {
    char hemisphere[4] = {0};
    double raw = 0.0;
    unsigned degrees = 0U;
    double minutes = 0.0;
    double parsed = 0.0;

    if (coordinate == NULL ||
        !parse_double_field(sentence, value_field, &raw) || raw < 0.0 ||
        !copy_field(sentence, hemisphere_field, hemisphere,
                    sizeof(hemisphere)) || hemisphere[1] != '\0') {
        return false;
    }
    degrees = (unsigned) (raw / 100.0);
    minutes = raw - (double) degrees * 100.0;
    if (degrees > degree_limit || minutes < 0.0 || minutes >= 60.0 ||
        (degrees == degree_limit && minutes > 0.0)) {
        return false;
    }
    parsed = (double) degrees + minutes / 60.0;
    if (hemisphere[0] == negative_hemisphere) {
        parsed = -parsed;
    } else if (hemisphere[0] != positive_hemisphere) {
        return false;
    }
    *coordinate = parsed;
    return true;
}

bool gps_nmea_parse_fix_pair(const char *rmc, const char *gga,
                             gps_nmea_fix_t *fix) {
    gps_nmea_fix_t parsed = {0};
    char rmc_line[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    char gga_line[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    char rmc_utc[16] = {0};
    char gga_utc[16] = {0};
    char status[4] = {0};
    char altitude_unit[4] = {0};
    uint8_t quality = 0U;
    double speed_knots = 0.0;

    if (fix == NULL || !copy_without_line_ending(rmc, rmc_line) ||
        !copy_without_line_ending(gga, gga_line) ||
        sentence_kind(rmc_line) != GPS_SENTENCE_RMC ||
        sentence_kind(gga_line) != GPS_SENTENCE_GGA ||
        !gps_nmea_checksum_valid(rmc_line) ||
        !gps_nmea_checksum_valid(gga_line) ||
        !parse_utc(rmc_line, rmc_utc) ||
        !parse_utc(gga_line, gga_utc) ||
        strcmp(rmc_utc, gga_utc) != 0 ||
        !copy_field(rmc_line, 2U, status, sizeof(status)) ||
        (strcmp(status, "A") != 0 && strcmp(status, "V") != 0) ||
        !parse_uint8_field(gga_line, 6U, &quality) || quality > 8U) {
        return false;
    }
    parsed.utc_valid = true;
    memcpy(parsed.utc, rmc_utc, sizeof(parsed.utc));
    parsed.satellites_valid = parse_uint8_field(gga_line, 7U,
                                                 &parsed.satellites);
    parsed.hdop_valid = parse_double_field(gga_line, 8U, &parsed.hdop) &&
                        parsed.hdop >= 0.0;
    parsed.altitude_valid = parse_double_field(gga_line, 9U,
                                                &parsed.altitude_m) &&
                            parsed.altitude_m >= -1000.0 &&
                            parsed.altitude_m <= 60000.0 &&
                            copy_field(gga_line, 10U, altitude_unit,
                                       sizeof(altitude_unit)) &&
                            strcmp(altitude_unit, "M") == 0;
    parsed.position_valid =
        parse_coordinate(rmc_line, 3U, 4U, 90U, 'N', 'S',
                         &parsed.latitude_deg) &&
        parse_coordinate(rmc_line, 5U, 6U, 180U, 'E', 'W',
                         &parsed.longitude_deg);
    parsed.speed_valid = parse_double_field(rmc_line, 7U, &speed_knots) &&
                         speed_knots >= 0.0 && speed_knots <= 2000.0;
    if (parsed.speed_valid) {
        parsed.speed_kmh = speed_knots * 1.852;
    }
    parsed.course_valid = parse_double_field(rmc_line, 8U,
                                              &parsed.course_deg) &&
                          parsed.course_deg >= 0.0 &&
                          parsed.course_deg <= 360.0;
    parsed.fix_valid = strcmp(status, "A") == 0 && quality != 0U &&
                       parsed.position_valid;
    if (!parsed.fix_valid) {
        parsed.position_valid = false;
        parsed.altitude_valid = false;
        parsed.speed_valid = false;
        parsed.course_valid = false;
    }
    *fix = parsed;
    return true;
}

static bool normalize_sentence(const char *input,
                               char output[GPS_NMEA_SENTENCE_CAPACITY]) {
    size_t length = strlen(input);

    if (length + 3U > GPS_NMEA_SENTENCE_CAPACITY) {
        return false;
    }
    memcpy(output, input, length);
    output[length] = '\r';
    output[length + 1U] = '\n';
    output[length + 2U] = '\0';
    return true;
}

void gps_nmea_pairer_init(gps_nmea_pairer_t *pairer) {
    if (pairer != NULL) {
        memset(pairer, 0, sizeof(*pairer));
    }
}

gps_nmea_result_t gps_nmea_pairer_consume(
    gps_nmea_pairer_t *pairer, const char *sentence,
    char rmc[GPS_NMEA_SENTENCE_CAPACITY],
    char gga[GPS_NMEA_SENTENCE_CAPACITY], bool *fix_valid) {
    gps_sentence_kind_t kind = sentence_kind(sentence);
    char utc[16] = {0};
    char status[4] = {0};

    if (pairer == NULL || sentence == NULL || rmc == NULL || gga == NULL ||
        fix_valid == NULL) {
        return GPS_NMEA_INVALID;
    }
    if (kind == GPS_SENTENCE_OTHER) {
        return GPS_NMEA_IGNORED;
    }
    if (strlen(sentence) + 3U > GPS_NMEA_SENTENCE_CAPACITY ||
        !gps_nmea_checksum_valid(sentence) ||
        !copy_field(sentence, 1U, utc, sizeof(utc))) {
        return GPS_NMEA_INVALID;
    }

    if (kind == GPS_SENTENCE_RMC) {
        if (!copy_field(sentence, 2U, status, sizeof(status)) ||
            (strcmp(status, "A") != 0 && strcmp(status, "V") != 0) ||
            !normalize_sentence(sentence, pairer->rmc)) {
            return GPS_NMEA_INVALID;
        }
        memcpy(pairer->rmc_utc, utc, sizeof(pairer->rmc_utc));
        pairer->rmc_utc[sizeof(pairer->rmc_utc) - 1U] = '\0';
        pairer->rmc_fix_valid = strcmp(status, "A") == 0;
        pairer->have_rmc = true;
    } else {
        if (!copy_field(sentence, 6U, status, sizeof(status)) ||
            status[1] != '\0' || status[0] < '0' || status[0] > '8' ||
            !normalize_sentence(sentence, pairer->gga)) {
            return GPS_NMEA_INVALID;
        }
        memcpy(pairer->gga_utc, utc, sizeof(pairer->gga_utc));
        pairer->gga_utc[sizeof(pairer->gga_utc) - 1U] = '\0';
        pairer->gga_fix_valid = status[0] != '0';
        pairer->have_gga = true;
    }

    if (!pairer->have_rmc || !pairer->have_gga ||
        strcmp(pairer->rmc_utc, pairer->gga_utc) != 0) {
        return GPS_NMEA_PENDING;
    }
    memcpy(rmc, pairer->rmc, sizeof(pairer->rmc));
    memcpy(gga, pairer->gga, sizeof(pairer->gga));
    *fix_valid = pairer->rmc_fix_valid && pairer->gga_fix_valid;
    pairer->have_rmc = false;
    pairer->have_gga = false;
    return GPS_NMEA_PAIR_READY;
}
