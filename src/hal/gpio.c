/**
 * @file gpio.c
 * @brief GPIO HAL implementation
 */
#include "hal/gpio.h"
#include "errno.h"

int hal_gpio_init(uint32_t pin, gpio_mode_t mode, gpio_pull_t pull)
{
    (void)pin; (void)mode; (void)pull;
    return E_OK;
}

int hal_gpio_set(uint32_t pin, uint8_t value)
{
    (void)pin; (void)value;
    return E_OK;
}

int hal_gpio_get(uint32_t pin)
{
    (void)pin;
    return 0;
}

int hal_gpio_set_alt_func(uint32_t pin, uint32_t func)
{
    (void)pin; (void)func;
    return E_OK;
}
