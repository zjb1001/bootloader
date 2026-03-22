/**
 * @file driver/watchdog.h
 * @brief Watchdog timer driver interface
 */
#ifndef DRIVER_WATCHDOG_H
#define DRIVER_WATCHDOG_H

#include "types.h"

int      driver_watchdog_init(uint32_t timeout_ms);
int      driver_watchdog_feed(void);
int      driver_watchdog_disable(void);
void     driver_watchdog_force_reset(void);
uint32_t driver_watchdog_status(void);

#endif /* DRIVER_WATCHDOG_H */
