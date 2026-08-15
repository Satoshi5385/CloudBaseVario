#include <assert.h>
#include <stdbool.h>
#include <string.h>

#include "domain/firmware_metadata.h"

static void assert_metadata(const char *embedded, const char *version,
                            const char *git_hash) {
    firmware_metadata_t metadata;

    assert(firmware_metadata_parse(embedded, strlen(embedded) + 1U,
                                   &metadata));
    assert(strcmp(metadata.version, version) == 0);
    assert(strcmp(metadata.git_hash, git_hash) == 0);
}

static void assert_invalid(const char *embedded) {
    firmware_metadata_t metadata;

    assert(!firmware_metadata_parse(embedded, strlen(embedded) + 1U,
                                    &metadata));
    assert(strcmp(metadata.version, "-") == 0);
    assert(strcmp(metadata.git_hash, "-") == 0);
}

int main(void) {
    firmware_metadata_t metadata;
    char unterminated[32];

    assert_metadata("0.1.0+48a4472", "0.1.0", "48a4472");
    assert_metadata("48a4472", "-", "48a4472");
    assert_invalid("0.1+48a4472");
    assert_invalid("0.1.0+48A4472");
    assert_invalid("0.1.0+48a4472-dirty");
    assert_invalid("0.1.0+48a4472+abcdef0");

    (void) memset(unterminated, '1', sizeof(unterminated));
    assert(!firmware_metadata_parse(unterminated, sizeof(unterminated),
                                    &metadata));
    assert(strcmp(metadata.version, "-") == 0);
    assert(strcmp(metadata.git_hash, "-") == 0);
    return 0;
}
