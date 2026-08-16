#include "platform/firmware_auth.h"

#include <string.h>

#include "esp_app_format.h"
#include "psa/crypto.h"
#include "domain/firmware_auth_key.h"
#include "domain/firmware_authentication.h"

#define FIRMWARE_AUTH_FORMAT_VERSION UINT16_C(1)
#define FIRMWARE_AUTH_PUBLIC_KEY_LENGTH 65U
#define FIRMWARE_AUTH_SIGNATURE_PREFIX_SIZE 100U
#define FIRMWARE_AUTH_IO_BUFFER_SIZE 1024U

static const uint8_t k_magic[8] = {'C', 'B', 'V', 'O', 'T', 'A', '0', '1'};
static const uint8_t k_signature_domain[] =
    "CloudBaseVario OTA authentication v1";
static const uint8_t k_public_key[FIRMWARE_AUTH_PUBLIC_KEY_LENGTH] =
    CBV_FIRMWARE_AUTH_PUBLIC_KEY;

_Static_assert(sizeof(firmware_auth_header_t) == FIRMWARE_AUTH_HEADER_SIZE,
               "firmware auth header format drift");
_Static_assert(sizeof(CBV_FIRMWARE_AUTH_KEY_ID) ==
                   FIRMWARE_AUTH_KEY_ID_LENGTH + 1U,
               "firmware auth key id must be exactly 16 ASCII bytes");

static bool key_is_provisioned(void) {
    return k_public_key[0] == 0x04U &&
           memcmp(CBV_FIRMWARE_AUTH_KEY_ID, "UNPROVISIONED000",
                  FIRMWARE_AUTH_KEY_ID_LENGTH) != 0;
}

static bool exact_text_matches(const char *actual, size_t capacity,
                               const char *expected) {
    size_t expected_length;

    if (actual == NULL || expected == NULL ||
        memchr(actual, '\0', capacity) == NULL) {
        return false;
    }
    expected_length = strlen(expected);
    return expected_length < capacity &&
           memcmp(actual, expected, expected_length + 1U) == 0;
}

static bool header_is_structurally_valid(const firmware_auth_header_t *header,
                                         size_t file_size,
                                         const char *expected_project) {
    size_t expected_size;

    if (header == NULL || memcmp(header->magic, k_magic, sizeof(k_magic)) != 0 ||
        header->format_version != FIRMWARE_AUTH_FORMAT_VERSION ||
        header->header_size != FIRMWARE_AUTH_HEADER_SIZE ||
        header->reserved != 0U ||
        header->chip_id != ESP_CHIP_ID_ESP32S3 ||
        !exact_text_matches(header->project_name,
                            sizeof(header->project_name), expected_project) ||
        header->payload_size == 0U ||
        header->payload_size > FIRMWARE_AUTH_MAX_PAYLOAD_SIZE ||
        memcmp(header->key_id, CBV_FIRMWARE_AUTH_KEY_ID,
               FIRMWARE_AUTH_KEY_ID_LENGTH) != 0) {
        return false;
    }
    expected_size = (size_t) header->header_size + header->payload_size;
    return expected_size == file_size;
}

static esp_err_t sha256_begin(psa_hash_operation_t *operation) {
    *operation = (psa_hash_operation_t) PSA_HASH_OPERATION_INIT;
    if (psa_crypto_init() != PSA_SUCCESS ||
        psa_hash_setup(operation, PSA_ALG_SHA_256) != PSA_SUCCESS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t sha256_finish(psa_hash_operation_t *operation,
                               uint8_t output[FIRMWARE_AUTH_SHA256_LENGTH]) {
    size_t output_length = 0U;
    if (psa_hash_finish(operation, output, FIRMWARE_AUTH_SHA256_LENGTH,
                        &output_length) == PSA_SUCCESS &&
        output_length == FIRMWARE_AUTH_SHA256_LENGTH) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

static esp_err_t signature_digest(const firmware_auth_header_t *header,
                                  uint8_t digest[FIRMWARE_AUTH_SHA256_LENGTH]) {
    psa_hash_operation_t context;
    esp_err_t ret;

    ret = sha256_begin(&context);
    if (ret == ESP_OK &&
        (psa_hash_update(&context, k_signature_domain,
                         sizeof(k_signature_domain) - 1U) != PSA_SUCCESS ||
         psa_hash_update(&context, (const uint8_t *) header,
                         FIRMWARE_AUTH_SIGNATURE_PREFIX_SIZE) != PSA_SUCCESS)) {
        ret = ESP_FAIL;
    }
    if (ret != ESP_OK) {
        (void) psa_hash_abort(&context);
        return ret;
    }
    return sha256_finish(&context, digest);
}

static esp_err_t verify_signature(const firmware_auth_header_t *header) {
    uint8_t digest[FIRMWARE_AUTH_SHA256_LENGTH];
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t public_key = MBEDTLS_SVC_KEY_ID_INIT;
    esp_err_t ret;

    if (!key_is_provisioned()) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    ret = signature_digest(header, digest);
    if (ret != ESP_OK) {
        return ret;
    }
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_VERIFY_HASH);
    psa_set_key_algorithm(&attributes, PSA_ALG_ECDSA(PSA_ALG_SHA_256));
    psa_set_key_type(&attributes,
                     PSA_KEY_TYPE_ECC_PUBLIC_KEY(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&attributes, 256U);
    if (psa_import_key(&attributes, k_public_key, sizeof(k_public_key),
                       &public_key) != PSA_SUCCESS ||
        psa_verify_hash(public_key, PSA_ALG_ECDSA(PSA_ALG_SHA_256), digest,
                        sizeof(digest), header->signature,
                        sizeof(header->signature)) != PSA_SUCCESS) {
        ret = ESP_ERR_INVALID_CRC;
    }
    if (public_key != MBEDTLS_SVC_KEY_ID_INIT) {
        (void) psa_destroy_key(public_key);
    }
    psa_reset_key_attributes(&attributes);
    return ret;
}

static esp_err_t hash_file_payload(FILE *file, const firmware_auth_header_t *header,
                                   uint8_t output[FIRMWARE_AUTH_SHA256_LENGTH]) {
    uint8_t buffer[FIRMWARE_AUTH_IO_BUFFER_SIZE];
    psa_hash_operation_t context;
    size_t remaining = header->payload_size;
    esp_err_t ret = sha256_begin(&context);

    if (ret != ESP_OK || fseek(file, header->header_size, SEEK_SET) != 0) {
        if (ret == ESP_OK) {
            (void) psa_hash_abort(&context);
        }
        return ESP_FAIL;
    }
    while (remaining > 0U) {
        size_t wanted = remaining;

        if (wanted > sizeof(buffer)) {
            wanted = sizeof(buffer);
        }
        if (fread(buffer, 1, wanted, file) != wanted ||
            psa_hash_update(&context, buffer, wanted) != PSA_SUCCESS) {
            (void) psa_hash_abort(&context);
            return ESP_FAIL;
        }
        remaining -= wanted;
    }
    return sha256_finish(&context, output);
}

esp_err_t firmware_auth_verify_package(FILE *file, size_t file_size,
                                       const char *expected_project,
                                       firmware_auth_header_t *header) {
    uint8_t payload_hash[FIRMWARE_AUTH_SHA256_LENGTH];
    esp_err_t ret;

    if (file == NULL || header == NULL || expected_project == NULL ||
        file_size < FIRMWARE_AUTH_HEADER_SIZE ||
        fread(header, 1, sizeof(*header), file) != sizeof(*header) ||
        !header_is_structurally_valid(header, file_size, expected_project)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    ret = hash_file_payload(file, header, payload_hash);
    if (ret != ESP_OK ||
        memcmp(payload_hash, header->payload_sha256, sizeof(payload_hash)) != 0) {
        return ESP_ERR_INVALID_CRC;
    }
    return verify_signature(header);
}

static esp_err_t hash_partition_payload(
    const esp_partition_t *partition, uint32_t payload_size,
    uint8_t output[FIRMWARE_AUTH_SHA256_LENGTH]) {
    uint8_t buffer[FIRMWARE_AUTH_IO_BUFFER_SIZE];
    psa_hash_operation_t context;
    uint32_t offset = 0U;
    esp_err_t ret = sha256_begin(&context);

    if (ret != ESP_OK) {
        return ret;
    }
    while (offset < payload_size) {
        size_t length = payload_size - offset;
        if (length > sizeof(buffer)) {
            length = sizeof(buffer);
        }
        if (esp_partition_read(partition, offset, buffer, length) != ESP_OK ||
            psa_hash_update(&context, buffer, length) != PSA_SUCCESS) {
            (void) psa_hash_abort(&context);
            return ESP_FAIL;
        }
        offset += (uint32_t) length;
    }
    return sha256_finish(&context, output);
}

esp_err_t firmware_authenticate_partition(
    const esp_partition_t *partition, const char *expected_project,
    firmware_authentication_t *authentication) {
    firmware_auth_header_t header;
    uint8_t payload_hash[FIRMWARE_AUTH_SHA256_LENGTH];
    size_t record_offset;
    esp_err_t ret;

    if (authentication == NULL || partition == NULL || expected_project == NULL ||
        partition->size < FIRMWARE_AUTH_RECORD_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(authentication, 0, sizeof(*authentication));
    authentication->authenticity = FIRMWARE_AUTH_UNKNOWN;
    record_offset = partition->size - FIRMWARE_AUTH_RECORD_SIZE;
    ret = esp_partition_read(partition, record_offset, &header, sizeof(header));
    if (ret != ESP_OK) {
        return ret;
    }
    if (!header_is_structurally_valid(&header,
                                      (size_t) header.header_size +
                                          header.payload_size,
                                      expected_project)) {
        authentication->authenticity = FIRMWARE_AUTH_NON_OFFICIAL;
        return ESP_OK;
    }
    ret = hash_partition_payload(partition, header.payload_size, payload_hash);
    if (ret != ESP_OK) {
        return ret;
    }
    if (memcmp(payload_hash, header.payload_sha256, sizeof(payload_hash)) != 0 ||
        verify_signature(&header) != ESP_OK) {
        authentication->authenticity = FIRMWARE_AUTH_NON_OFFICIAL;
        return ESP_OK;
    }
    authentication->authenticity = FIRMWARE_AUTH_OFFICIAL;
    memcpy(authentication->key_id, header.key_id, sizeof(header.key_id));
    authentication->key_id[sizeof(header.key_id)] = '\0';
    for (size_t index = 0U; index < sizeof(payload_hash); index++) {
        static const char hex[] = "0123456789abcdef";
        authentication->payload_sha256[index * 2U] = hex[payload_hash[index] >> 4U];
        authentication->payload_sha256[index * 2U + 1U] =
            hex[payload_hash[index] & 0x0fU];
    }
    return ESP_OK;
}
