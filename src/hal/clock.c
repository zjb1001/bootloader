/**
 * @file clock.c
 * @brief Clock management HAL implementation
 */
#include "hal/clock.h"
#include "errno.h"

int hal_clock_init(const clock_config_t *cfg)
{
    /* TODO: configure PLL, set dividers, verify frequencies */
    (void)cfg;
    return E_OK;
}

int hal_clock_get_freq(clock_source_t src)
{
    /* TODO: read back actual frequency from hardware */
    (void)src;
    return E_OK;
}

int hal_clock_set_divisor(uint32_t domain, uint32_t divisor)
{
    (void)domain;
    (void)divisor;
    return E_OK;
}

int hal_clock_enable_periph(uint32_t periph_id, uint8_t enable)
{
    (void)periph_id;
    (void)enable;
    return E_OK;
}
