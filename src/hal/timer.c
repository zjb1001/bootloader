/**
 * @file timer.c
 * @brief Hardware timer HAL implementation
 *
 * Provides timekeeping services using ARM generic timer.
 */
#include "hal/timer.h"
#include "platform/platform.h"
#include "errno.h"
#include "types.h"

/* ARM Generic Timer register access */
#define cntfrq() ({ uint32_t val; __asm__ volatile("mrc p15, 0, %0, c14, c0, 0" : "=r"(val)); val; })
#define cntvct() ({ uint64_t val; __asm__ volatile("mrrc p15, 0, %0, %H0, c14" : "=r"(val)); val; })
#define cntkctl() ({ uint32_t val; __asm__ volatile("mrc p15, 0, %0, c14, c1, 0" : "=r"(val)); val; })
#define cntkctl_write(val) __asm__ volatile("mcr p15, 0, %0, c14, c1, 0" :: "r"(val))

/* Cached timer frequency */
static uint32_t s_timer_freq_hz = 0;

/**
 * Initialize timer subsystem
 */
int hal_timer_init(void)
{
    /* Read timer frequency from CNTFRQ register */
    s_timer_freq_hz = cntfrq();

    /* Enable PL1 physical timer */
    uint32_t cntkctl_val = cntkctl();
    cntkctl_val |= (1u << 0);  /* Enable PL1 physical timer */
    cntkctl_write(cntkctl_val);

    return E_OK;
}

/**
 * Get current time in milliseconds since boot
 */
uint32_t hal_timer_get_ms(void)
{
    if (s_timer_freq_hz == 0) {
        s_timer_freq_hz = cntfrq();
    }

    uint64_t ticks = cntvct();
    return (uint32_t)(ticks / (s_timer_freq_hz / 1000));
}

/**
 * Get current time in microseconds since boot
 */
uint64_t hal_timer_get_us(void)
{
    if (s_timer_freq_hz == 0) {
        s_timer_freq_hz = cntfrq();
    }

    uint64_t ticks = cntvct();
    return (ticks * 1000000) / s_timer_freq_hz;
}

/**
 * Blocking delay in milliseconds
 */
void hal_timer_delay_ms(uint32_t ms)
{
    if (s_timer_freq_hz == 0) {
        s_timer_freq_hz = cntfrq();
    }

    uint64_t start = cntvct();
    uint64_t ticks = ((uint64_t)ms * s_timer_freq_hz) / 1000;

    while ((cntvct() - start) < ticks) {
        __asm__ volatile("nop");
    }
}

/**
 * Blocking delay in microseconds
 */
void hal_timer_delay_us(uint32_t us)
{
    if (s_timer_freq_hz == 0) {
        s_timer_freq_hz = cntfrq();
    }

    uint64_t start = cntvct();
    uint64_t ticks = ((uint64_t)us * s_timer_freq_hz) / 1000000;

    while ((cntvct() - start) < ticks) {
        __asm__ volatile("nop");
    }
}
