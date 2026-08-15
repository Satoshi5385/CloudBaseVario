#include "domain/firmware_metadata.h"

#include <string.h>

#define FIRMWARE_METADATA_GIT_HASH_LENGTH 7U

static bool is_decimal_digit(char value) {
    return value >= '0' && value <= '9';
}

static bool is_lower_hex_digit(char value) {
    return is_decimal_digit(value) || (value >= 'a' && value <= 'f');
}

static bool version_is_valid(const char *value, size_t length) {
    size_t component_digits = 0U;
    size_t separator_count = 0U;

    if (length == 0U) {
        return false;
    }
    for (size_t index = 0U; index < length; index++) {
        if (is_decimal_digit(value[index])) {
            component_digits++;
        } else if (value[index] == '.' && component_digits > 0U &&
                   separator_count < 2U) {
            separator_count++;
            component_digits = 0U;
        } else {
            return false;
        }
    }
    return separator_count == 2U && component_digits > 0U;
}

static bool hash_is_valid(const char *value, size_t length) {
    if (length != FIRMWARE_METADATA_GIT_HASH_LENGTH) {
        return false;
    }
    for (size_t index = 0U; index < length; index++) {
        if (!is_lower_hex_digit(value[index])) {
            return false;
        }
    }
    return true;
}

static void set_unknown(firmware_metadata_t *metadata) {
    (void) strcpy(metadata->version, "-");
    (void) strcpy(metadata->git_hash, "-");
}

bool firmware_metadata_parse(const char *embedded, size_t capacity,
                             firmware_metadata_t *metadata) {
    const char *separator = NULL;
    size_t length;
    size_t version_length;
    size_t hash_length;

    if (metadata == NULL) {
        return false;
    }
    set_unknown(metadata);
    if (embedded == NULL || capacity == 0U) {
        return false;
    }
    length = 0U;
    while (length < capacity && embedded[length] != '\0') {
        length++;
    }
    if (length == capacity) {
        return false;
    }
    if (hash_is_valid(embedded, length)) {
        (void) memcpy(metadata->git_hash, embedded, length);
        metadata->git_hash[length] = '\0';
        return true;
    }

    separator = memchr(embedded, '+', length);
    if (separator == NULL) {
        return false;
    }
    version_length = (size_t) (separator - embedded);
    hash_length = length - version_length - 1U;
    if (!version_is_valid(embedded, version_length) ||
        version_length >= sizeof(metadata->version) ||
        !hash_is_valid(separator + 1, hash_length)) {
        return false;
    }
    (void) memcpy(metadata->version, embedded, version_length);
    metadata->version[version_length] = '\0';
    (void) memcpy(metadata->git_hash, separator + 1, hash_length);
    metadata->git_hash[hash_length] = '\0';
    return true;
}
