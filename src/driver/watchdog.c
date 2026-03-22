/**
 * @file watchdog.c
 * @brief Watchdog timer driver
 *
 * Platform-specific watchdog implementation for ARMv7-A.
 */
#include "driver/watchdog.h"
#include "hal/timer.h"
#include "errno.h"
#include "types.h"

/* Watchdog register base */
#define WDT_BASE        0x01C20C00u

/* Watchdog register offsets */
#define WDT_CTRL        0x0000  /* Control register */
#define WDT_CFG         0x0004  /* Configuration register */
#define WDT_MODE        0x0008  /* Mode register */
#define WDT_VALUE       0x0010  /* Current value counter */
#define WDT_RELOAD      0x0014  /* Reload value */
#define WDT_IRQ_EN      0x0018  /* IRQ enable */
#define WDT_IRQ_STA     0x001C  /* IRQ status */
#define WDT_SOFT_RST    0x0020  /* Software reset */

/* Control register bits */
#define WDT_CTRL_RESET  (1u << 0)   /* Reset watchdog */
#define WDT_CTRL_START  (1u << 1)   /* Start watchdog */

/* Configuration bits */
#define WDT_CFG_SRC_24M (0x0 << 0)  /* 24MHz oscillator */
#define WDT_CFG_SRC_32K (0x1 << 0)  /* 32kHz oscillator */
#define WDT_CFG_PRES    0x3         /* Prescaler value mask */

/* Mode bits */
#define WDT_MODE_EN     (1u << 0)   /* Enable watchdog */
#define WDT_MODE_RST    (1u << 1)   /* Enable reset on timeout */

#define REG32(addr) (*(volatile uint32_t *)(addr))

/* Cached timeout value */
static uint32_t s_timeout_ms = 0;

/**
 * Initialize watchdog with timeout
 */
int driver_watchdog_init(uint32_t timeout_ms)
{
    /* Calculate reload value (assuming 24MHz clock) */
    /* prescale by 1 (divide by 1) for maximum resolution */
    uint32_t reload_val = (timeout_ms * 24000) - 1;

    /* Clamp to maximum value */
    if (reload_val > 0xFFFFF) {
        reload_val = 0xFFFFF;
    }

    /* Disable watchdog first */
    REG32(WDT_BASE + WDT_CTRL) &= ~WDT_CTRL_START;
    REG32(WDT_BASE + WDT_MODE) = 0;

    /* Configure clock source (24MHz) */
    REG32(WDT_BASE + WDT_CFG) = WDT_CFG_SRC_24M;

    /* Set reload value */
    REG32(WDT_BASE + WDT_RELOAD) = reload_val;

    /* Clear interrupts */
    REG32(WDT_BASE + WDT_IRQ_STA) = 0xFF;

    /* Enable watchdog in reset mode */
    REG32(WDT_BASE + WDT_MODE) = WDT_MODE_EN | WDT_MODE_RST;

    /* Reset counter */
    REG32(WDT_BASE + WDT_CTRL) |= WDT_CTRL_RESET;

    /* Start watchdog */
    REG32(WDT_BASE + WDT_CTRL) |= WDT_CTRL_START;

    /* Cache timeout value for reference */
    s_timeout_ms = timeout_ms;

    return E_OK;
}

/**
 * Feed watchdog (reset counter)
 */
int driver_watchdog_feed(void)
{
    /* Write reload value to reset counter */
    uint32_t reload_val = REG32(WDT_BASE + WDT_RELOAD);
    REG32(WDT_BASE + WDT_VALUE) = reload_val;

    return E_OK;
}

/**
 * Disable watchdog
 */
int driver_watchdog_disable(void)
{
    /* Stop watchdog */
    REG32(WDT_BASE + WDT_CTRL) &= ~WDT_CTRL_START;
    REG32(WDT_BASE + WDT_MODE) = 0;

    return E_OK;
}

/**
 * Force immediate system reset via watchdog
 */
void driver_watchdog_force_reset(void)
{
    /* Set minimum timeout and start */
    REG32(WDT_BASE + WDT_RELOAD) = 1;
    REG32(WDT_BASE + WDT_CTRL) |= WDT_CTRL_RESET | WDT_CTRL_START;
    REG32(WDT_BASE + WDT_MODE) = WDT_MODE_EN | WDT_MODE_RST;

    /* Wait for reset */
    while (1) {
        __asm__ volatile("nop");
    }
}

/**
 * Get watchdog status
 * Returns: bit0=enabled, bit1=reset_pending, bits[15:2]=countdown_value
 */
uint32_t driver_watchdog_status(void)
{
    uint32_t status = 0;
    uint32_t mode = REG32(WDT_BASE + WDT_MODE);
    uint32_t value = REG32(WDT_BASE + WDT_VALUE);

    if (mode & WDT_MODE_EN) {
        status |= 0x01;
    }
    if (mode & WDT_MODE_RST) {
        status |= 0x02;
    }

    /* Pack countdown value */
    status |= ((value & 0x3FFF) << 2);

    return status;
}
