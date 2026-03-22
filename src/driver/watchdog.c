/**
 * @file watchdog.c
 * @brief Watchdog timer driver
 */
#include "driver/watchdog.h"
#include "errno.h"

int driver_watchdog_init(uint32_t timeout_ms)
{
    /* TODO: configure WDT registers */
    (void)timeout_ms;
    return E_OK;
}

int driver_watchdog_feed(void)
{
    /* TODO: write feed sequence to WDT */
    return E_OK;
}

int driver_watchdog_disable(void)
{
    return E_OK;
}

void driver_watchdog_force_reset(void)
{
    /* TODO: trigger immediate reset */
    for (;;) { }
}

uint32_t driver_watchdog_status(void)
{
    return 0;
}
