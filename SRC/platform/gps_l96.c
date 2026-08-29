#include "platform/gps_l96.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "domain/gps_nmea.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "platform/board.h"

#define GPS_UART_PORT UART_NUM_1
#define GPS_UART_BUFFER_BYTES 1024
#define GPS_IDENTIFY_TIMEOUT_MS UINT32_C(500)
#define GPS_ACK_TIMEOUT_MS UINT32_C(700)
#define GPS_SEARCH_MODE_TIMEOUT_MS UINT32_C(3000)
#define GPS_BAUD_SETTLE_MS UINT32_C(100)

static const char *TAG = "gps_l96";

static bool uart_installed;
static char receive_line[GPS_NMEA_SENTENCE_CAPACITY];
static size_t receive_length;
static bool receive_overflow;

static uint8_t pmtk_checksum(const char *body) {
    uint8_t checksum = 0U;

    for (const char *cursor = body; *cursor != '\0'; cursor++) {
        checksum ^= (uint8_t) *cursor;
    }
    return checksum;
}

static esp_err_t send_pmtk(const char *body) {
    char command[128] = {0};
    int length = 0;
    int written = 0;

    if (!uart_installed || body == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    length = snprintf(command, sizeof(command), "$%s*%02X\r\n", body,
                      pmtk_checksum(body));
    if (length <= 0 || (size_t) length >= sizeof(command)) {
        return ESP_ERR_INVALID_SIZE;
    }
    written = uart_write_bytes(GPS_UART_PORT, command, (size_t) length);
    if (written != length) {
        return ESP_FAIL;
    }
    return uart_wait_tx_done(GPS_UART_PORT, pdMS_TO_TICKS(100U));
}

esp_err_t gps_l96_read_line(char *line, size_t capacity,
                            uint32_t timeout_ms) {
    int64_t deadline_us = esp_timer_get_time() +
                          (int64_t) timeout_ms * INT64_C(1000);

    if (!uart_installed || line == NULL || capacity == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    while (esp_timer_get_time() < deadline_us) {
        uint8_t byte = 0U;
        int64_t remaining_us = deadline_us - esp_timer_get_time();
        uint32_t wait_ms = 0U;
        if (remaining_us > 0) {
            wait_ms = (uint32_t) ((remaining_us + 999) / 1000);
        }
        int count = uart_read_bytes(GPS_UART_PORT, &byte, 1U,
                                    pdMS_TO_TICKS(wait_ms));

        if (count <= 0) {
            continue;
        }
        if (byte == '\r' || byte == '\n') {
            if (receive_overflow) {
                receive_length = 0U;
                receive_overflow = false;
                return ESP_ERR_INVALID_SIZE;
            }
            if (receive_length == 0U) {
                continue;
            }
            receive_line[receive_length] = '\0';
            if (receive_length + 1U > capacity) {
                receive_length = 0U;
                return ESP_ERR_INVALID_SIZE;
            }
            memcpy(line, receive_line, receive_length + 1U);
            receive_length = 0U;
            return ESP_OK;
        }
        if (receive_length + 1U < sizeof(receive_line)) {
            receive_line[receive_length] = (char) byte;
            receive_length++;
        } else {
            receive_overflow = true;
        }
    }
    return ESP_ERR_TIMEOUT;
}

static bool contains_l96(const char *line) {
    char upper[GPS_NMEA_SENTENCE_CAPACITY] = {0};
    size_t length = strlen(line);

    if (length >= sizeof(upper)) {
        return false;
    }
    for (size_t index = 0U; index < length; index++) {
        upper[index] = (char) toupper((unsigned char) line[index]);
    }
    return strstr(upper, "L96") != NULL;
}

static esp_err_t identify_l96(void) {
    int64_t deadline_us = esp_timer_get_time() +
                          (int64_t) GPS_IDENTIFY_TIMEOUT_MS * 1000;
    esp_err_t ret = send_pmtk("PMTK605");

    if (ret != ESP_OK) {
        return ret;
    }
    while (esp_timer_get_time() < deadline_us) {
        char line[GPS_NMEA_SENTENCE_CAPACITY] = {0};

        ret = gps_l96_read_line(line, sizeof(line), 100U);
        if (ret == ESP_OK && strncmp(line, "$PMTK705,", 9U) == 0 &&
            gps_nmea_checksum_valid(line)) {
            if (contains_l96(line)) {
                return ESP_OK;
            }
            return ESP_ERR_NOT_SUPPORTED;
        }
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t ack_flag_result(unsigned flag) {
    if (flag == 3U) {
        return ESP_OK;
    }
    if (flag == 1U) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (flag == 2U) {
        return ESP_FAIL;
    }
    return ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t wait_for_ack(unsigned command_number) {
    int64_t deadline_us = esp_timer_get_time() +
                          (int64_t) GPS_ACK_TIMEOUT_MS * 1000;

    while (esp_timer_get_time() < deadline_us) {
        char line[GPS_NMEA_SENTENCE_CAPACITY] = {0};
        gps_pmtk_ack_t ack = {0};
        esp_err_t ret = gps_l96_read_line(line, sizeof(line), 100U);

        if (ret == ESP_OK && gps_pmtk_ack_parse(line, &ack) &&
            ack.command == command_number) {
            return ack_flag_result(ack.flag);
        }
    }
    return ESP_ERR_TIMEOUT;
}

static bool is_restart_notice(const char *line) {
    if (!gps_nmea_checksum_valid(line)) {
        return false;
    }
    return strncmp(line, "$PMTK010,", 9U) == 0 ||
           strncmp(line, "$PMTK011,", 9U) == 0;
}

static bool is_restart_ready(const char *line) {
    return strncmp(line, "$PMTK011,", 9U) == 0 ||
           strncmp(line, "$PMTK010,002*", 13U) == 0;
}

static esp_err_t wait_for_search_mode(void) {
    int64_t deadline_us = esp_timer_get_time() +
                          (int64_t) GPS_SEARCH_MODE_TIMEOUT_MS * 1000;
    bool restart_seen = false;

    while (esp_timer_get_time() < deadline_us) {
        char line[GPS_NMEA_SENTENCE_CAPACITY] = {0};
        gps_pmtk_ack_t ack = {0};
        esp_err_t ret = gps_l96_read_line(line, sizeof(line), 100U);

        if (ret == ESP_OK && gps_pmtk_ack_parse(line, &ack) &&
            ack.command == 353U) {
            return ack_flag_result(ack.flag);
        }
        if (ret == ESP_OK && is_restart_notice(line)) {
            restart_seen = true;
            if (is_restart_ready(line)) {
                vTaskDelay(pdMS_TO_TICKS(GPS_BAUD_SETTLE_MS));
                return identify_l96();
            }
        }
        if (ret == ESP_ERR_TIMEOUT && restart_seen) {
            vTaskDelay(pdMS_TO_TICKS(GPS_BAUD_SETTLE_MS));
            return identify_l96();
        }
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t configure_search_mode(void) {
    esp_err_t ret = send_pmtk("PMTK353,1,1,0,0,0");

    if (ret != ESP_OK) {
        return ret;
    }
    return wait_for_search_mode();
}

static esp_err_t send_with_ack(const char *body, unsigned command_number) {
    esp_err_t ret = send_pmtk(body);

    if (ret != ESP_OK) {
        return ret;
    }
    return wait_for_ack(command_number);
}

esp_err_t gps_l96_set_interval(uint32_t interval_ms) {
    char body[32] = {0};

    if (interval_ms < 200U || interval_ms > 10000U) {
        return ESP_ERR_INVALID_ARG;
    }
    (void) snprintf(body, sizeof(body), "PMTK220,%" PRIu32,
                    interval_ms);
    return send_with_ack(body, 220U);
}

static esp_err_t configure_l96(uint32_t interval_ms) {
    esp_err_t ret = configure_search_mode();

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "PMTK353 GPS+GLONASS configuration failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }
    ret = send_with_ack(
        "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", 314U);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "PMTK314 sentence configuration failed: %s",
                 esp_err_to_name(ret));
        return ret;
    }
    ret = gps_l96_set_interval(interval_ms);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "PMTK220 update interval configuration failed: %s",
                 esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t gps_l96_connect(uint32_t interval_ms, uint32_t *baud_rate) {
    static const uint32_t supported_bauds[] = {
        9600U, 115200U, 4800U, 14400U, 19200U, 38400U, 57600U,
    };
    uart_config_t config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uint32_t detected_baud = 0U;
    esp_err_t ret = ESP_OK;

    if (baud_rate == NULL || interval_ms < 200U || interval_ms > 10000U) {
        return ESP_ERR_INVALID_ARG;
    }
    gps_l96_deinit();
    ret = uart_driver_install(GPS_UART_PORT, GPS_UART_BUFFER_BYTES, 0, 0,
                              NULL, 0);
    if (ret != ESP_OK) {
        return ret;
    }
    uart_installed = true;
    receive_length = 0U;
    receive_overflow = false;
    ret = uart_param_config(GPS_UART_PORT, &config);
    if (ret == ESP_OK) {
        ret = uart_set_pin(GPS_UART_PORT, PIN_GPS_UART_TX, PIN_GPS_UART_RX,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (ret != ESP_OK) {
        gps_l96_deinit();
        return ret;
    }

    ret = ESP_ERR_NOT_FOUND;
    for (size_t index = 0U;
         index < sizeof(supported_bauds) / sizeof(supported_bauds[0]);
         index++) {
        if (uart_set_baudrate(GPS_UART_PORT, supported_bauds[index]) != ESP_OK) {
            continue;
        }
        (void) uart_flush_input(GPS_UART_PORT);
        receive_length = 0U;
        receive_overflow = false;
        ret = identify_l96();
        if (ret == ESP_OK) {
            detected_baud = supported_bauds[index];
            ESP_LOGI(TAG, "L96 identified at %" PRIu32 " bps",
                     detected_baud);
            break;
        }
        if (ret == ESP_ERR_NOT_SUPPORTED) {
            gps_l96_deinit();
            return ret;
        }
    }
    if (detected_baud == 0U) {
        gps_l96_deinit();
        return ret;
    }

    if (detected_baud != 115200U) {
        ret = send_pmtk("PMTK251,115200");
        if (ret == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(GPS_BAUD_SETTLE_MS));
            ret = uart_set_baudrate(GPS_UART_PORT, 115200U);
        }
        if (ret == ESP_OK) {
            (void) uart_flush_input(GPS_UART_PORT);
            receive_length = 0U;
            receive_overflow = false;
            ret = identify_l96();
        }
    }
    if (ret == ESP_OK) {
        ret = configure_l96(interval_ms);
    }
    if (ret != ESP_OK) {
        gps_l96_deinit();
        return ret;
    }
    *baud_rate = 115200U;
    return ESP_OK;
}

void gps_l96_deinit(void) {
    if (uart_installed) {
        (void) uart_driver_delete(GPS_UART_PORT);
        uart_installed = false;
    }
    receive_length = 0U;
    receive_overflow = false;
}
