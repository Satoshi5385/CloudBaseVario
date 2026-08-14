#include "platform/safe_stop_wake.h"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "hal/gpio_ll.h"
#include "platform/board.h"
#include "soc/gpio_struct.h"

static portMUX_TYPE wake_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t wake_task;
static bool handler_registered;

static void IRAM_ATTR sw1_wake_isr(void *arg) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    TaskHandle_t task = NULL;

    (void) arg;
    gpio_ll_intr_disable(&GPIO, PIN_SW_1);
    portENTER_CRITICAL_ISR(&wake_lock);
    task = wake_task;
    portEXIT_CRITICAL_ISR(&wake_lock);
    if (task != NULL) {
        vTaskNotifyGiveFromISR(task, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t safe_stop_wake_init(TaskHandle_t task) {
    esp_err_t ret;

    if (task == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&wake_lock);
    if (handler_registered) {
        portEXIT_CRITICAL(&wake_lock);
        return ESP_ERR_INVALID_STATE;
    }
    wake_task = task;
    portEXIT_CRITICAL(&wake_lock);

    ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        goto fail;
    }
    ret = gpio_isr_handler_add(PIN_SW_1, sw1_wake_isr, NULL);
    if (ret != ESP_OK) {
        goto fail;
    }
    portENTER_CRITICAL(&wake_lock);
    handler_registered = true;
    portEXIT_CRITICAL(&wake_lock);
    ret = esp_sleep_enable_gpio_wakeup();
    if (ret != ESP_OK) {
        safe_stop_wake_deinit();
        return ret;
    }
    return ESP_OK;

fail:
    portENTER_CRITICAL(&wake_lock);
    wake_task = NULL;
    portEXIT_CRITICAL(&wake_lock);
    return ret;
}

esp_err_t safe_stop_wake_arm(bool wake_when_pressed) {
    gpio_int_type_t trigger = GPIO_INTR_LOW_LEVEL;
    esp_err_t ret;

    if (wake_when_pressed) {
        trigger = GPIO_INTR_HIGH_LEVEL;
    }
    safe_stop_wake_disarm();
    ret = gpio_set_intr_type(PIN_SW_1, trigger);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = gpio_wakeup_enable(PIN_SW_1, trigger);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = gpio_intr_enable(PIN_SW_1);
    if (ret != ESP_OK) {
        (void) gpio_wakeup_disable(PIN_SW_1);
    }
    return ret;
}

void safe_stop_wake_disarm(void) {
    (void) gpio_intr_disable(PIN_SW_1);
    (void) gpio_wakeup_disable(PIN_SW_1);
}

void safe_stop_wake_deinit(void) {
    bool registered;

    safe_stop_wake_disarm();
    portENTER_CRITICAL(&wake_lock);
    registered = handler_registered;
    handler_registered = false;
    wake_task = NULL;
    portEXIT_CRITICAL(&wake_lock);
    if (registered) {
        (void) gpio_isr_handler_remove(PIN_SW_1);
    }
}
