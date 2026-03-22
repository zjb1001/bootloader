/**
 * @file mock_hal.c
 * @brief Mock HAL layer for host-side testing
 */
#include "types.h"
#include "errno.h"

/* Mock state that tests can inspect / configure */
static uint32_t mock_timer_ms = 0;

void mock_hal_set_timer_ms(uint32_t ms)
{
    mock_timer_ms = ms;
}

uint32_t mock_hal_get_timer_ms(void)
{
    return mock_timer_ms;
}
