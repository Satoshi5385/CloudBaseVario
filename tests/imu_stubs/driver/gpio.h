#pragma once
#include <stdint.h>
#include "esp_err.h"
#define GPIO_MODE_INPUT 1
#define GPIO_PULLUP_DISABLE 0
#define GPIO_PULLDOWN_DISABLE 0
#define GPIO_INTR_DISABLE 0
#define GPIO_INTR_POSEDGE 1
#define ESP_INTR_FLAG_IRAM 1
typedef void (*gpio_isr_t)(void *context);
typedef struct {
    uint64_t pin_bit_mask;
    int mode;
    int pull_up_en;
    int pull_down_en;
    int intr_type;
} gpio_config_t;
esp_err_t gpio_config(const gpio_config_t *config);
esp_err_t gpio_install_isr_service(int flags);
esp_err_t gpio_isr_handler_add(int pin, gpio_isr_t handler, void *context);
esp_err_t gpio_isr_handler_remove(int pin);
esp_err_t gpio_intr_disable(int pin);
esp_err_t gpio_intr_enable(int pin);
esp_err_t gpio_reset_pin(int pin);
