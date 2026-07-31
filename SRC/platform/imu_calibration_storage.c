#include "platform/imu_calibration_storage.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "cJSON.h"
#include "platform/icm42688_hxy.h"

#define MC_DATA_FORMAT_VERSION 1
#define MC_DATA_PATH_BUFFER_SIZE 96U
#define MC_DATA_MAX_FILE_BYTES 2048U
#define MC_DATA_MODEL "ICM-42688P-HXY"
#define MC_DATA_COORDINATE "SENSOR"
#define MC_DATA_METHOD "LEVEL_Z_UP"

static bool make_path(const char *base_path, const char *filename,
                      char path[MC_DATA_PATH_BUFFER_SIZE]) {
    int written = 0;

    if (base_path == NULL || filename == NULL || path == NULL) {
        return false;
    }
    written = snprintf(path, MC_DATA_PATH_BUFFER_SIZE, "%s/%s", base_path,
                       filename);
    return written > 0 && (size_t) written < MC_DATA_PATH_BUFFER_SIZE;
}

static void set_diagnostics(
    imu_calibration_storage_diagnostics_t *diagnostics,
    imu_calibration_storage_result_t result, int error) {
    if (diagnostics != NULL) {
        diagnostics->result = result;
        diagnostics->io_error = error;
    }
}

static bool object_has_exact_keys(const cJSON *object,
                                  const char *const keys[],
                                  size_t key_count) {
    size_t child_count = 0U;

    if (!cJSON_IsObject(object)) {
        return false;
    }
    for (const cJSON *child = object->child; child != NULL;
         child = child->next) {
        bool known = false;

        if (child->string == NULL) {
            return false;
        }
        for (size_t index = 0U; index < key_count; index++) {
            if (strcmp(child->string, keys[index]) == 0) {
                known = true;
                break;
            }
        }
        if (!known) {
            return false;
        }
        child_count++;
    }
    if (child_count != key_count) {
        return false;
    }
    for (size_t index = 0U; index < key_count; index++) {
        unsigned int occurrences = 0U;

        for (const cJSON *child = object->child; child != NULL;
             child = child->next) {
            if (strcmp(child->string, keys[index]) == 0) {
                occurrences++;
            }
        }
        if (occurrences != 1U) {
            return false;
        }
    }
    return true;
}

static bool parse_document(const char *json, size_t length,
                           imu_accel_calibration_t *calibration) {
    static const char *const root_keys[] = {
        "format_version", "imu_accel_calibration"};
    static const char *const calibration_keys[] = {
        "model", "who_am_i", "coordinate", "method", "sample_count",
        "offset_mps2"};
    const char *document = json;
    size_t document_length = length;
    const char *parse_end = NULL;
    cJSON *root = NULL;
    cJSON *section = NULL;
    cJSON *offsets = NULL;
    imu_accel_calibration_t candidate = {0};
    bool valid = false;

    if (json == NULL || calibration == NULL || length == 0U) {
        return false;
    }
    if (length >= 3U && (uint8_t) json[0] == UINT8_C(0xEF) &&
        (uint8_t) json[1] == UINT8_C(0xBB) &&
        (uint8_t) json[2] == UINT8_C(0xBF)) {
        document += 3;
        document_length -= 3U;
    }
    root = cJSON_ParseWithLengthOpts(document, document_length + 1U,
                                     &parse_end, true);
    if (root == NULL || parse_end != document + document_length ||
        !object_has_exact_keys(root, root_keys,
                               sizeof(root_keys) / sizeof(root_keys[0]))) {
        goto cleanup;
    }
    cJSON *version = cJSON_GetObjectItemCaseSensitive(root, "format_version");
    section = cJSON_GetObjectItemCaseSensitive(root,
                                                "imu_accel_calibration");
    if (!cJSON_IsNumber(version) || version->valuedouble != 1.0 ||
        !object_has_exact_keys(
            section, calibration_keys,
            sizeof(calibration_keys) / sizeof(calibration_keys[0]))) {
        goto cleanup;
    }

    cJSON *model = cJSON_GetObjectItemCaseSensitive(section, "model");
    cJSON *who_am_i = cJSON_GetObjectItemCaseSensitive(section, "who_am_i");
    cJSON *coordinate =
        cJSON_GetObjectItemCaseSensitive(section, "coordinate");
    cJSON *method = cJSON_GetObjectItemCaseSensitive(section, "method");
    cJSON *sample_count =
        cJSON_GetObjectItemCaseSensitive(section, "sample_count");
    offsets = cJSON_GetObjectItemCaseSensitive(section, "offset_mps2");
    if (!cJSON_IsString(model) || model->valuestring == NULL ||
        strcmp(model->valuestring, MC_DATA_MODEL) != 0 ||
        !cJSON_IsNumber(who_am_i) ||
        who_am_i->valuedouble != ICM42688_HXY_WHO_AM_I_VALUE ||
        !cJSON_IsString(coordinate) || coordinate->valuestring == NULL ||
        strcmp(coordinate->valuestring, MC_DATA_COORDINATE) != 0 ||
        !cJSON_IsString(method) || method->valuestring == NULL ||
        strcmp(method->valuestring, MC_DATA_METHOD) != 0 ||
        !cJSON_IsNumber(sample_count) ||
        sample_count->valuedouble != IMU_ACCEL_CALIBRATION_SAMPLE_COUNT ||
        !cJSON_IsArray(offsets) || cJSON_GetArraySize(offsets) != 3) {
        goto cleanup;
    }
    for (size_t axis = 0U; axis < IMU_AXIS_COUNT; axis++) {
        cJSON *item = cJSON_GetArrayItem(offsets, (int) axis);

        if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) {
            goto cleanup;
        }
        candidate.offset_mps2[axis] = (float) item->valuedouble;
    }
    candidate.sample_count = IMU_ACCEL_CALIBRATION_SAMPLE_COUNT;
    candidate.valid = true;
    if (!imu_accel_calibration_validate(&candidate)) {
        goto cleanup;
    }
    *calibration = candidate;
    valid = true;

cleanup:
    cJSON_Delete(root);
    return valid;
}

static imu_calibration_storage_result_t load_path(
    const char *path, imu_accel_calibration_t *calibration, int *io_error) {
    struct stat status = {0};
    FILE *file = NULL;
    char *contents = NULL;
    size_t bytes_read = 0U;
    imu_calibration_storage_result_t result =
        IMU_CALIBRATION_STORAGE_IO_ERROR;

    if (stat(path, &status) != 0) {
        if (errno == ENOENT) {
            return IMU_CALIBRATION_STORAGE_MISSING;
        }
        if (io_error != NULL) {
            *io_error = errno;
        }
        return IMU_CALIBRATION_STORAGE_IO_ERROR;
    }
    if (status.st_size <= 0 ||
        (uint64_t) status.st_size > MC_DATA_MAX_FILE_BYTES) {
        return IMU_CALIBRATION_STORAGE_INVALID;
    }
    contents = malloc((size_t) status.st_size + 1U);
    if (contents == NULL) {
        if (io_error != NULL) {
            *io_error = ENOMEM;
        }
        return IMU_CALIBRATION_STORAGE_IO_ERROR;
    }
    file = fopen(path, "rb");
    if (file == NULL) {
        if (io_error != NULL) {
            *io_error = errno;
        }
        free(contents);
        return IMU_CALIBRATION_STORAGE_IO_ERROR;
    }
    bytes_read = fread(contents, 1U, (size_t) status.st_size, file);
    if (ferror(file) != 0 || bytes_read != (size_t) status.st_size) {
        if (io_error != NULL) {
            *io_error = ferror(file) != 0 ? errno : EIO;
        }
        result = IMU_CALIBRATION_STORAGE_IO_ERROR;
    } else {
        contents[bytes_read] = '\0';
        result = parse_document(contents, bytes_read, calibration)
                     ? IMU_CALIBRATION_STORAGE_VALID
                     : IMU_CALIBRATION_STORAGE_INVALID;
    }
    (void) fclose(file);
    free(contents);
    return result;
}

imu_calibration_storage_result_t imu_calibration_storage_load(
    const char *base_path, imu_accel_calibration_t *calibration,
    imu_calibration_storage_diagnostics_t *diagnostics) {
    char target_path[MC_DATA_PATH_BUFFER_SIZE] = {0};
    char backup_path[MC_DATA_PATH_BUFFER_SIZE] = {0};
    imu_accel_calibration_t recovered = {0};
    imu_calibration_storage_result_t result;
    int io_error = 0;

    if (calibration == NULL ||
        !make_path(base_path, "mc_data.json", target_path) ||
        !make_path(base_path, "mc_data.bak", backup_path)) {
        set_diagnostics(diagnostics, IMU_CALIBRATION_STORAGE_IO_ERROR,
                        EINVAL);
        return IMU_CALIBRATION_STORAGE_IO_ERROR;
    }
    memset(calibration, 0, sizeof(*calibration));
    result = load_path(target_path, calibration, &io_error);
    if (result == IMU_CALIBRATION_STORAGE_VALID) {
        (void) unlink(backup_path);
        set_diagnostics(diagnostics, result, 0);
        return result;
    }
    if (result != IMU_CALIBRATION_STORAGE_MISSING) {
        set_diagnostics(diagnostics, result, io_error);
        return result;
    }

    result = load_path(backup_path, &recovered, &io_error);
    if (result == IMU_CALIBRATION_STORAGE_VALID &&
        rename(backup_path, target_path) == 0) {
        *calibration = recovered;
        set_diagnostics(diagnostics, IMU_CALIBRATION_STORAGE_RECOVERED, 0);
        return IMU_CALIBRATION_STORAGE_RECOVERED;
    }
    if (result == IMU_CALIBRATION_STORAGE_MISSING) {
        set_diagnostics(diagnostics, IMU_CALIBRATION_STORAGE_MISSING, 0);
        return IMU_CALIBRATION_STORAGE_MISSING;
    }
    if (result == IMU_CALIBRATION_STORAGE_VALID) {
        io_error = errno;
        result = IMU_CALIBRATION_STORAGE_IO_ERROR;
    }
    set_diagnostics(diagnostics, result, io_error);
    return result;
}

static bool write_json(FILE *file,
                       const imu_accel_calibration_t *calibration) {
    return fprintf(
               file,
               "{\n"
               "  \"format_version\": %d,\n"
               "  \"imu_accel_calibration\": {\n"
               "    \"model\": \"%s\",\n"
               "    \"who_am_i\": %u,\n"
               "    \"coordinate\": \"%s\",\n"
               "    \"method\": \"%s\",\n"
               "    \"sample_count\": %" PRIu32 ",\n"
               "    \"offset_mps2\": [%.9g, %.9g, %.9g]\n"
               "  }\n"
               "}\n",
               MC_DATA_FORMAT_VERSION, MC_DATA_MODEL,
               (unsigned int) ICM42688_HXY_WHO_AM_I_VALUE,
               MC_DATA_COORDINATE, MC_DATA_METHOD,
               calibration->sample_count,
               (double) calibration->offset_mps2[0],
               (double) calibration->offset_mps2[1],
               (double) calibration->offset_mps2[2]) > 0;
}

esp_err_t imu_calibration_storage_save(
    const char *base_path, const imu_accel_calibration_t *calibration) {
    char target_path[MC_DATA_PATH_BUFFER_SIZE] = {0};
    char temporary_path[MC_DATA_PATH_BUFFER_SIZE] = {0};
    char backup_path[MC_DATA_PATH_BUFFER_SIZE] = {0};
    FILE *file = NULL;
    int file_descriptor = -1;
    struct stat status = {0};
    bool target_exists = false;
    imu_accel_calibration_t verified = {0};
    int io_error = 0;

    if (!imu_accel_calibration_validate(calibration) ||
        !make_path(base_path, "mc_data.json", target_path) ||
        !make_path(base_path, "mc_data.tmp", temporary_path) ||
        !make_path(base_path, "mc_data.bak", backup_path)) {
        return ESP_ERR_INVALID_ARG;
    }
    file = fopen(temporary_path, "wb");
    if (file == NULL) {
        return ESP_FAIL;
    }
    if (!write_json(file, calibration) || fflush(file) != 0) {
        (void) fclose(file);
        (void) unlink(temporary_path);
        return ESP_FAIL;
    }
    file_descriptor = fileno(file);
    if (file_descriptor < 0 || fsync(file_descriptor) != 0 ||
        fclose(file) != 0) {
        (void) unlink(temporary_path);
        return ESP_FAIL;
    }
    if (load_path(temporary_path, &verified, &io_error) !=
            IMU_CALIBRATION_STORAGE_VALID ||
        memcmp(verified.offset_mps2, calibration->offset_mps2,
               sizeof(verified.offset_mps2)) != 0) {
        (void) unlink(temporary_path);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (stat(target_path, &status) == 0) {
        target_exists = true;
    } else if (errno != ENOENT) {
        (void) unlink(temporary_path);
        return ESP_FAIL;
    }
    if (target_exists) {
        if (unlink(backup_path) != 0 && errno != ENOENT) {
            (void) unlink(temporary_path);
            return ESP_FAIL;
        }
        if (rename(target_path, backup_path) != 0) {
            (void) unlink(temporary_path);
            return ESP_FAIL;
        }
    }
    if (rename(temporary_path, target_path) != 0) {
        if (target_exists) {
            (void) rename(backup_path, target_path);
        }
        (void) unlink(temporary_path);
        return ESP_FAIL;
    }
    if (target_exists) {
        (void) unlink(backup_path);
    }
    return ESP_OK;
}

const char *imu_calibration_storage_result_name(
    imu_calibration_storage_result_t result) {
    switch (result) {
    case IMU_CALIBRATION_STORAGE_VALID:
        return "VALID";
    case IMU_CALIBRATION_STORAGE_MISSING:
        return "MISSING";
    case IMU_CALIBRATION_STORAGE_RECOVERED:
        return "RECOVERED";
    case IMU_CALIBRATION_STORAGE_INVALID:
        return "INVALID";
    case IMU_CALIBRATION_STORAGE_IO_ERROR:
        return "IO_ERROR";
    default:
        return "UNKNOWN";
    }
}
