/**
 * @file hal/gpio.h
 * @brief GPIO HAL interface  [ISR-safe for get/set]
 */
#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include "types.h"

typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALT_FUNC,
} gpio_mode_t;

typedef enum {
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
} gpio_pull_t;

int hal_gpio_init(uint32_t pin, gpio_mode_t mode, gpio_pull_t pull);
int hal_gpio_set(uint32_t pin, uint8_t value);
int hal_gpio_get(uint32_t pin);
int hal_gpio_set_alt_func(uint32_t pin, uint32_t func);

#endif /* HAL_GPIO_H */
