/**
 * @file hal/clock.h
 * @brief Clock management HAL interface  [INIT]
 */
#ifndef HAL_CLOCK_H
#define HAL_CLOCK_H

#include "types.h"

typedef enum {
    CLOCK_OSC_24M = 0,      /**< External oscillator 24 MHz */
    CLOCK_OSC_32K,           /**< Low-speed oscillator 32 kHz */
    CLOCK_PLL_CORE,          /**< Core PLL */
    CLOCK_PLL_DDR,           /**< DDR PLL */
} clock_source_t;

typedef struct {
    uint32_t core_freq;      /**< Core frequency (MHz) */
    uint32_t ahb_freq;       /**< AHB bus frequency (MHz) */
    uint32_t apb_freq;       /**< APB bus frequency (MHz) */
    uint32_t dram_freq;      /**< DRAM frequency (MHz) */
} clock_config_t;

/**
 * Initialize system clocks.
 * @param cfg  Clock configuration.
 * @return E_OK on success, < 0 on error.
 */
int hal_clock_init(const clock_config_t *cfg);

/**
 * Query current frequency of a clock source.
 * @return Frequency in MHz, or < 0 on error.
 */
int hal_clock_get_freq(clock_source_t src);

/**
 * Set divider for a clock domain.
 * @param domain  Clock domain identifier (CORE/AHB/APB/DRAM).
 * @param divisor Divider value.
 */
int hal_clock_set_divisor(uint32_t domain, uint32_t divisor);

/**
 * Enable or disable a peripheral clock gate.
 * @param periph_id Peripheral identifier.
 * @param enable    1 = enable, 0 = disable.
 */
int hal_clock_enable_periph(uint32_t periph_id, uint8_t enable);

#endif /* HAL_CLOCK_H */
