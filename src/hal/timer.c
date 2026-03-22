/**
 * @file timer.c
 * @brief Hardware timer HAL implementation
 */
#include "hal/timer.h"
#include "errno.h"

int hal_timer_init(void)
{
    /* TODO: configure free-running counter */
    return E_OK;
}

uint32_t hal_timer_get_ms(void)
{
    /* TODO: read hardware counter, convert to ms */
    return 0;
}

uint64_t hal_timer_get_us(void)
{
    return 0;
}

void hal_timer_delay_ms(uint32_t ms)
{
    /* TODO: busy-wait using counter */
    (void)ms;
}

void hal_timer_delay_us(uint32_t us)
{
    (void)us;
}
