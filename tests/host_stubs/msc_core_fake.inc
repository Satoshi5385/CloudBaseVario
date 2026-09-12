/* Minimal protocol fake for the real bridge; BOT deliberately leaves pending_io
 * set, as TinyUSB 0.21.0 does. Firmware CI validates actual upstream integration.
 */
static struct {
    bool pending_io;
    struct { uint8_t command[16]; uint8_t lun; } cbw;
} _mscd_itf;
static unsigned usb_completions;
static int32_t usb_result;

static void proc_async_io_done(void *value)
{
    assert(_mscd_itf.pending_io);
    _mscd_itf.pending_io = false;
    usb_completions++;
    usb_result = (int32_t)(intptr_t)value;
}

void mscd_init(void) { memset(&_mscd_itf, 0, sizeof(_mscd_itf)); }
bool mscd_deinit(void) { return true; }
void mscd_reset(uint8_t rhport) { (void)rhport; mscd_init(); }
bool mscd_control_xfer_cb(uint8_t rhport, uint8_t stage,
                        tusb_control_request_t const *request)
{
    (void)rhport;
    return stage != CONTROL_STAGE_SETUP ||
           (request->wValue == 0 && request->wLength == 0);
}
