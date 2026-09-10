#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "platform/icm42688_hxy.h"
#include "soc/gpio_struct.h"

/* Behavioral I2C model: a fixed-address FIFO byte stream and register bank.
 * This validates the real driver, not the physical HXY rearm behavior. */
static uint8_t registers[256];
static uint8_t fifo[4096];
static size_t fifo_size;
static size_t fifo_offset;
static size_t data_reads;
static size_t largest_read;
static size_t rearm_count;
static size_t arrivals;
static size_t read_transactions;
static size_t write_transactions;
static size_t count_reads;
static bool fail_data;
static bool fail_rearm;
static int64_t time_us;
static size_t delay_calls;
static gpio_isr_t watermark_handler;
static void *watermark_context;
static size_t watermark_notifications;
static bool watermark_interrupt_enabled;

gpio_dev_t GPIO;

void test_imu_log(const char *tag, const char *format, ...) {
    (void)tag;
    (void)format;
}
const char *esp_err_to_name(esp_err_t error) {
    (void)error;
    return "test";
}
int64_t esp_timer_get_time(void) { return time_us; }
void vTaskDelay(uint32_t ticks) {
    delay_calls++;
    time_us += (int64_t)ticks * 1000;
}
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *task_woken) {
    assert(task == watermark_context && task_woken != NULL);
    watermark_notifications++;
    *task_woken = pdTRUE;
}
esp_err_t gpio_config(const gpio_config_t *config) {
    assert(config->pin_bit_mask == (UINT64_C(1) << 14));
    assert(config->intr_type == GPIO_INTR_POSEDGE);
    return ESP_OK;
}
esp_err_t gpio_install_isr_service(int flags) {
    assert(flags == ESP_INTR_FLAG_IRAM);
    return ESP_OK;
}
esp_err_t gpio_isr_handler_add(int pin, gpio_isr_t handler, void *context) {
    assert(pin == 14 && handler != NULL && context != NULL);
    watermark_handler = handler;
    watermark_context = context;
    return ESP_OK;
}
esp_err_t gpio_isr_handler_remove(int pin) {
    assert(pin == 14 && watermark_handler != NULL);
    watermark_handler = NULL;
    watermark_context = NULL;
    return ESP_OK;
}
esp_err_t gpio_intr_disable(int pin) {
    assert(pin == 14);
    watermark_interrupt_enabled = false;
    return ESP_OK;
}
void gpio_ll_intr_disable(gpio_dev_t *hardware, int pin) {
    assert(hardware == &GPIO && pin == 14);
    watermark_interrupt_enabled = false;
}
esp_err_t gpio_intr_enable(int pin) {
    assert(pin == 14 && watermark_handler != NULL);
    watermark_interrupt_enabled = true;
    return ESP_OK;
}
esp_err_t gpio_reset_pin(int pin) { assert(pin == 14); return ESP_OK; }
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, uint16_t address, int timeout_ms) {
    assert(bus != NULL && address == 0x18 && timeout_ms == 5);
    return ESP_OK;
}
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
    const i2c_device_config_t *config, i2c_master_dev_handle_t *device) {
    assert(bus != NULL && config->device_address == 0x18);
    assert(config->scl_speed_hz == 400000);
    *device = bus;
    return ESP_OK;
}
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device) {
    assert(device != NULL);
    return ESP_OK;
}
static void append_sample(void) {
    /* Gyro XYZ = -32768, 32767, -1; accel XYZ = 4096, -4096, 2048. */
    const uint8_t sample[12] = {0x80,0, 0x7F,0xFF, 0xFF,0xFF,
                              0x10,0, 0xF0,0, 0x08,0};
    assert(fifo_size + sizeof(sample) <= sizeof(fifo));
    memcpy(fifo + fifo_size, sample, sizeof(sample));
    fifo_size += sizeof(sample);
}
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device,
    const uint8_t *tx, size_t size, int timeout_ms) {
    assert(device != NULL && size == 2 && timeout_ms == 5);
    write_transactions++;
    time_us += 75;
    if (tx[0] == 0x1D && fail_rearm) return ESP_ERR_TIMEOUT;
    registers[tx[0]] = tx[1];
    if (tx[0] == 0x1D && tx[1] == 0) {
        fifo_size = 0;
        fifo_offset = 0;
        rearm_count++;
    }
    return ESP_OK;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
    const uint8_t *tx, size_t tx_size, uint8_t *rx, size_t size, int timeout_ms) {
    assert(device != NULL && tx_size == 1 && timeout_ms == 5);
    read_transactions++;
    time_us += (int64_t)(size + 3) * 23;
    if (*tx == 0x21) {
        assert(registers[0x05] == 0x40); /* Addr_Auto MUST be off. */
        data_reads++;
        if (size > largest_read) largest_read = size;
        if (fail_data) return ESP_ERR_TIMEOUT;
        assert(fifo_offset + size <= fifo_size);
        memcpy(rx, fifo + fifo_offset, size);
        fifo_offset += size;
        if (arrivals > 0) {
            append_sample();
            arrivals--;
        }
    } else {
        assert(size == 1);
        if (*tx == 0x1F || *tx == 0x20) count_reads++;
        rx[0] = registers[*tx];
    }
    return ESP_OK;
}
static void init_device(void) {
    icm42688_hxy_identity_t identity = {0};
    esp_err_t ret = ESP_ERR_NOT_FINISHED;

    (void)icm42688_hxy_deinit();
    memset(registers, 0, sizeof(registers));
    fifo_size = fifo_offset = data_reads = largest_read = rearm_count = arrivals = 0;
    read_transactions = write_transactions = count_reads = 0;
    delay_calls = 0;
    watermark_notifications = 0;
    watermark_interrupt_enabled = false;
    fail_data = fail_rearm = false;
    time_us = 1000000;
    registers[1] = 0x6A;
    registers[0x0B] = 0x07;
    assert(icm42688_hxy_init_begin((void *)registers,
                                   (void *)registers) == ESP_OK);
    while (ret == ESP_ERR_NOT_FINISHED) {
        int64_t next_us = icm42688_hxy_init_next_action_us();

        if (time_us < next_us) {
            time_us = next_us;
        }
        ret = icm42688_hxy_init_poll(time_us, &identity);
    }
    assert(ret == ESP_OK && delay_calls == 0);
    assert(identity.address == 0x18 && identity.who_am_i == 0x6A);
    assert(registers[0x40] == 0xAA && registers[0x42] == 0xAA);
    assert(registers[0x41] == 2 && registers[0x43] == 0);
    assert(registers[0x06] == 0x09 && registers[0x1C] == 6);
    assert(registers[0x1E] == 25);
    assert(registers[0x45] == 0x88 && registers[0x1D] == 0x10);
    assert(watermark_handler != NULL && watermark_context == registers);
    assert(watermark_interrupt_enabled);
    watermark_handler(watermark_context);
    assert(watermark_notifications == 1 && !watermark_interrupt_enabled);
}
static void load_samples(size_t count) {
    fifo_offset = 0;
    fifo_size = 4;
    fifo[0] = 0; fifo[1] = 0xAB; fifo[2] = 0xCD; fifo[3] = 0xEF;
    for (size_t i = 0; i < count; i++) append_sample();
    time_us += 10000;
}
static void test_decode_and_cadence(void) {
    icm42688_hxy_batch_t batch;
    int64_t last_us = 0;
    init_device();
    for (size_t j = 0; j < 5; j++) {
        size_t reads_before = read_transactions;
        size_t writes_before = write_transactions;

        load_samples(4);
        assert(icm42688_hxy_read_fifo(&batch) == ESP_OK);
        assert(watermark_interrupt_enabled);
        assert(batch.sample_count == 4 && batch.discarded_samples == 0);
        assert(batch.sensor_time == 0xABCDEF);
        assert(batch.data_status == 0x07 && !batch.overflow);
        assert(read_transactions - reads_before == 1);
        assert(write_transactions - writes_before == 2);
        assert(count_reads == 0);
        for (size_t i = 0; i < batch.sample_count; i++) {
            const icm42688_hxy_sample_t *s = &batch.samples[i];
            assert(s->valid && s->timestamp_us > last_us && s->timestamp_us <= time_us);
            if (i > 0) assert(s->timestamp_us - last_us == 2500);
            assert(s->raw_gyro[0] == -32768 && s->raw_gyro[1] == 32767);
            assert(s->raw_gyro[2] == -1 && s->raw_accel[0] == 4096);
            assert(s->raw_accel[1] == -4096 && s->raw_accel[2] == 2048);
            assert(fabsf(s->accel_mps2[0] - 9.80665f) < 0.0001f);
            assert(s->gyro_radps[0] < -34.0f && s->gyro_radps[1] > 34.0f);
            last_us = s->timestamp_us;
        }
    }
    assert(largest_read == 52);
}
static void test_late_arrivals_are_cleared_by_rearm(void) {
    icm42688_hxy_batch_t batch;
    init_device(); load_samples(4); arrivals = 2;
    assert(icm42688_hxy_read_fifo(&batch) == ESP_OK);
    assert(batch.sample_count == 4 && data_reads == 1);
    assert(fifo_size == 0 && registers[0x1D] == 0x10);
    assert(icm42688_hxy_read_fifo(&batch) == ESP_ERR_NOT_FINISHED);
    assert(data_reads == 1); /* The 1 ms rearm guard prevents an early read. */
}

static void test_initialization_can_be_aborted(void) {
    icm42688_hxy_identity_t identity = {0};

    init_device();
    assert(icm42688_hxy_deinit() == ESP_OK);
    registers[1] = 0x6A;
    delay_calls = 0;
    assert(icm42688_hxy_init_begin((void *)registers,
                                   (void *)registers) == ESP_OK);
    assert(icm42688_hxy_init_poll(time_us, &identity) ==
           ESP_ERR_NOT_FINISHED);
    assert(icm42688_hxy_init_poll(time_us, &identity) ==
           ESP_ERR_NOT_FINISHED);
    assert(icm42688_hxy_init_in_progress());
    assert(icm42688_hxy_init_abort() == ESP_OK);
    assert(!icm42688_hxy_init_in_progress());
    assert(delay_calls == 0);
}
static void test_failed_transfer_and_rearm(void) {
    icm42688_hxy_batch_t batch;
    init_device(); load_samples(4); fail_data = true; fail_rearm = true;
    assert(icm42688_hxy_read_fifo(&batch) == ESP_ERR_TIMEOUT);
    assert(batch.sample_count == 0 && batch.discarded_samples == 4);
    size_t reads = data_reads;
    fail_data = false;
    assert(icm42688_hxy_read_fifo(&batch) == ESP_ERR_TIMEOUT);
    assert(data_reads == reads);
    fail_rearm = false;
    assert(icm42688_hxy_read_fifo(&batch) == ESP_ERR_NOT_FINISHED);
    assert(data_reads == reads);
    load_samples(4);
    assert(icm42688_hxy_read_fifo(&batch) == ESP_OK);
}
static void test_nonmonotonic_batch_is_not_published(void) {
    icm42688_hxy_batch_t batch;
    init_device(); load_samples(4);
    assert(icm42688_hxy_read_fifo(&batch) == ESP_OK);
    load_samples(4);
    time_us -= 8900;
    assert(icm42688_hxy_read_fifo(&batch) == ESP_ERR_INVALID_RESPONSE);
    assert(batch.sample_count == 0 && batch.discarded_samples == 4);
}
int main(void) {
    assert(icm42688_hxy_read_fifo(NULL) == ESP_ERR_INVALID_ARG);
    assert(icm42688_hxy_init_begin((void *)registers, NULL) ==
           ESP_ERR_INVALID_ARG);
    test_decode_and_cadence();
    test_late_arrivals_are_cleared_by_rearm();
    test_failed_transfer_and_rearm();
    test_nonmonotonic_batch_is_not_published();
    test_initialization_can_be_aborted();
    assert(icm42688_hxy_deinit() == ESP_OK);
    icm42688_hxy_batch_t batch;
    assert(icm42688_hxy_read_fifo(&batch) == ESP_ERR_INVALID_STATE);
    puts("IMU FIFO driver tests passed");
    return 0;
}
