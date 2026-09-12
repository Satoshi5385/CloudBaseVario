#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <setjmp.h>

typedef int esp_err_t;
typedef void *TaskHandle_t;
typedef void *SemaphoreHandle_t;
typedef uint8_t BYTE;
typedef int tinyusb_msc_mount_point_t;
typedef void *tinyusb_msc_storage_handle_t;
typedef struct { int unused; } storage_medium_t;
typedef void (*tusb_msc_callback_t)(void);
typedef struct {
    int id;
    uint8_t lun;
    uint32_t lba, offset;
    size_t size;
    esp_err_t result;
    uint32_t pending_count;
} tinyusb_msc_write_event_t;
typedef void (*tusb_msc_write_callback_t)(tinyusb_msc_storage_handle_t,
    const tinyusb_msc_write_event_t *, void *);
typedef struct {
    struct { unsigned type; } bmRequestType_bit;
    uint8_t bRequest;
    uint16_t wValue, wLength;
} tusb_control_request_t;

#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 1
#define ESP_ERR_NOT_FOUND 2
#define ESP_ERR_INVALID_SIZE 3
#define ESP_ERR_INVALID_ARG 4
#define TINYUSB_MSC_STORAGE_MOUNT_USB 1
#define MSC_STORAGE_BUFFER_SIZE 4096
#define TINYUSB_MSC_STORAGE_MAX_LUNS 2
#define TINYUSB_MSC_WRITE_EVENT_BEGIN 1
#define TINYUSB_MSC_WRITE_EVENT_COMPLETE 2
#define TUD_MSC_RET_ERROR (-1)
#define SCSI_CMD_WRITE_10 0x2a
#define MSC_REQ_RESET 0xff
#define CONTROL_STAGE_SETUP 0
#define TUSB_REQ_TYPE_CLASS 1
#define pdTRUE 1
#define portMAX_DELAY 0
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_RETURN_ON_FALSE(condition, error, ...) \
    do { if (!(condition)) return (error); } while (0)
static unsigned lock_depth;
#define MSC_ENTER_CRITICAL() do { assert(lock_depth == 0); lock_depth++; } while (0)
#define MSC_EXIT_CRITICAL() do { assert(lock_depth == 1); lock_depth--; } while (0)
#define MSC_CHECK_ON_CRITICAL(condition, error) \
    do { if (!(condition)) { MSC_EXIT_CRITICAL(); return (error); } } while (0)

#include "production_types.inc"
#include "msc_device_bridge.inc"

static msc_storage_obj_t disk;
static tinyusb_msc_driver_t driver;
static void (*queued_completion)(void *);
static void *queued_argument;
static uint8_t medium_data[4096];
static uint32_t medium_lba, medium_offset;
static size_t medium_size;
static unsigned writes, notifications, completions;
static esp_err_t medium_result;
static bool stop_during_copy;
static unsigned copy_observations;
static jmp_buf worker_wait;

static bool _msc_storage_get_by_lun(uint8_t lun, msc_storage_obj_t **storage)
{
    *storage = &disk;
    return lun == 0;
}
static bool msc_storage_range_valid(msc_storage_obj_t *storage, uint32_t lba,
                                   uint32_t offset, size_t size)
{
    (void)storage;
    return (uint64_t)lba * 512 + offset + size <= 4 * 1024 * 1024;
}
static void xTaskNotifyGive(TaskHandle_t worker)
{
    assert(worker == &disk);
    assert(disk.deffered_writes == 1 && disk.write_worker_ready);
    notifications++;
}
static esp_err_t msc_storage_write_sector(uint8_t lun, uint32_t lba,
                                         uint32_t offset, size_t size,
                                         const void *source)
{
    assert(lun == 0 && disk.deffered_writes == 1);
    memcpy(medium_data, source, size);
    medium_lba = lba;
    medium_offset = offset;
    medium_size = size;
    writes++;
    return medium_result;
}
static void usbd_defer_func(void (*callback)(void *), void *argument, bool isr)
{
    assert(!isr && !queued_completion && disk.deffered_writes == 1);
    queued_completion = callback;
    queued_argument = argument;
}
static void tusb_apply_requested_mount(void *argument)
{
    assert(argument == &disk && disk.deffered_writes == 0);
    assert(disk.completion_callback_active);
    disk.mount_transition_pending = false;
}
static unsigned ulTaskNotifyTake(int clear, unsigned timeout)
{
    (void)clear; (void)timeout;
    /* Return once to run the real worker; jump out when it next sleeps. */
    static bool running;
    if (running) { running = false; longjmp(worker_wait, 1); }
    running = true;
    return 1;
}
static int xSemaphoreGive(SemaphoreHandle_t semaphore)
{
    (void)semaphore;
    return 1;
}
static void vTaskDelete(void *task) { (void)task; assert(false); }

static void *observed_memcpy(void *dest, const void *src, size_t size)
{
    /* A stop/ownership request can run after reservation but before publication. */
    assert(disk.deffered_writes == 1 && !disk.write_worker_ready);
    assert(lock_depth == 0);
    copy_observations++;
    if (stop_during_copy) {
        disk.host_io_enabled = false;
        disk.mount_transition_pending = true;
    }
    return memcpy(dest, src, size);
}
#define memcpy observed_memcpy
#include "production_functions.inc"
#undef memcpy

static void event_callback(tinyusb_msc_storage_handle_t handle,
                           const tinyusb_msc_write_event_t *event, void *arg)
{
    (void)arg;
    assert(handle == &disk);
    if (event->id == TINYUSB_MSC_WRITE_EVENT_COMPLETE) {
        uint32_t pending;
        assert(disk.completion_callback_active && event->pending_count == 0);
        assert(tinyusb_msc_stop_host_io(handle, &pending) == ESP_OK);
        assert(pending == 1); /* storage/USB task cannot be destroyed mid-callback */
        assert(event->result == medium_result);
        completions++;
    }
}
static void run_worker(void)
{
    if (setjmp(worker_wait) == 0) msc_storage_write_worker_task(&disk);
}
static void drain(void)
{
    void (*callback)(void *) = queued_completion;
    assert(callback);
    queued_completion = NULL;
    callback(queued_argument);
    assert(!disk.deffered_writes && !disk.completion_callback_active);
}
static void prepare_host_write(void)
{
    disk.host_io_enabled = true;
    _mscd_itf.pending_io = true;
    _mscd_itf.cbw.command[0] = SCSI_CMD_WRITE_10;
    _mscd_itf.cbw.lun = 0;
}

int main(void)
{
    uint8_t first[4096], second[4096];
    memset(first, 0x35, sizeof(first));
    memset(second, 0xa6, sizeof(second));
    p_msc_driver = &driver;
    driver.dynamic.write_cb = event_callback;
    disk.mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB;
    disk.write_worker_handle = &disk;
    mscd_init();

    /* Rejection cannot alter an accepted payload or its address. */
    prepare_host_write();
    assert(msc_storage_write_sector_deferred(0, 17, 0, 4096, first) == ESP_OK);
    assert(msc_storage_write_sector_deferred(0, 39, 128, 512, second) == ESP_ERR_INVALID_STATE);
    assert(memcmp(disk.storage_buffer.data_buffer, first, 4096) == 0);
    assert(disk.storage_buffer.lba == 17 && disk.storage_buffer.offset == 0);
    assert(disk.storage_buffer.bufsize == 4096 && notifications == 1);
    run_worker();
    assert(writes == 1 && medium_lba == 17 && medium_offset == 0 && medium_size == 4096);
    assert(memcmp(medium_data, first, 4096) == 0);
    assert(disk.deffered_writes == 1 && usb_completions == 0);
    run_worker(); /* queued completion is not another write job */
    assert(writes == 1);
    assert(msc_storage_write_sector_deferred(0, 40, 0, 4096, second) == ESP_ERR_INVALID_STATE);
    drain();
    assert(usb_completions == 1 && usb_result == 4096 && completions == 1);

    /* Bus reset before medium completion cannot complete a new command. */
    prepare_host_write();
    assert(msc_storage_write_sector_deferred(0, 41, 0, 4096, second) == ESP_OK);
    mscd_reset(0);
    prepare_host_write();
    assert(msc_storage_write_sector_deferred(0, 42, 0, 4096, first) == ESP_ERR_INVALID_STATE);
    run_worker();
    drain();
    assert(usb_completions == 1 && _mscd_itf.pending_io);
    assert(memcmp(medium_data, second, 4096) == 0);

    /* BOT reset after medium completion but before the queued callback. */
    prepare_host_write();
    assert(msc_storage_write_sector_deferred(0, 43, 0, 512, first) == ESP_OK);
    run_worker();
    tusb_control_request_t reset = {.bmRequestType_bit = {TUSB_REQ_TYPE_CLASS},
                                   .bRequest = MSC_REQ_RESET};
    assert(mscd_control_xfer_cb(0, CONTROL_STAGE_SETUP, &reset));
    assert(!_mscd_itf.pending_io);
    prepare_host_write();
    drain();
    assert(usb_completions == 1 && _mscd_itf.pending_io);

    /* Stop during memcpy retains the accepted write through completion. */
    prepare_host_write();
    stop_during_copy = true;
    assert(msc_storage_write_sector_deferred(0, 44, 0, 512, second) == ESP_OK);
    assert(!disk.host_io_enabled && disk.deffered_writes == 1);
    run_worker();
    drain();
    assert(usb_completions == 2 && !disk.mount_transition_pending);
    stop_during_copy = false;

    /* Errors propagate; invalid BOT requests do not invalidate valid writes. */
    prepare_host_write();
    medium_result = ESP_ERR_INVALID_SIZE;
    assert(msc_storage_write_sector_deferred(0, 45, 0, 512, first) == ESP_OK);
    reset.wLength = 1;
    assert(!mscd_control_xfer_cb(0, CONTROL_STAGE_SETUP, &reset));
    run_worker();
    drain();
    assert(usb_completions == 3 && usb_result == TUD_MSC_RET_ERROR);

    /* Driver restart also invalidates old queued work. */
    prepare_host_write();
    assert(msc_storage_write_sector_deferred(0, 46, 0, 512, second) == ESP_OK);
    run_worker();
    assert(mscd_deinit());
    mscd_init();
    prepare_host_write();
    drain();
    assert(usb_completions == 3 && _mscd_itf.pending_io);
    assert(writes == 6 && copy_observations == 6 && completions == 6);
    return 0;
}
