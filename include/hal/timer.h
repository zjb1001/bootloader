/**
 * @file hal/timer.h
 * @brief Hardware timer HAL interface
 */
#ifndef HAL_TIMER_H
#define HAL_TIMER_H

#include "types.h"

/** Initialize timer subsystem.  [INIT] */
int hal_timer_init(void);

/** Get current time in milliseconds since boot.  [ISR-safe] */
uint32_t hal_timer_get_ms(void);

/** Get current time in microseconds since boot.  [ISR-safe] */
uint64_t hal_timer_get_us(void);

/** Blocking delay in milliseconds.  [THREAD] */
void hal_timer_delay_ms(uint32_t ms);

/** Blocking delay in microseconds.  [THREAD] */
void hal_timer_delay_us(uint32_t us);

#endif /* HAL_TIMER_H */
