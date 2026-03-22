/**
 * @file clock.c
 * @brief Clock management HAL implementation
 *
 * Provides PLL configuration, clock divider setup, and peripheral clock gating.
 * This is a platform-specific implementation for ARMv7-A platforms.
 */
#include "hal/clock.h"
#include "platform/platform.h"
#include "platform/chip.h"
#include "errno.h"
#include "types.h"

/* Register base addresses - platform specific */
#define PLL_CTRL_BASE    0x01C20000u  /* PLL control registers */
#define CLOCK_CTRL_BASE  0x01C20000u  /* Clock gating registers */

/* PLL control register offsets */
#define PLL_CPU_CTRL     0x0000
#define PLL_AXI_CTRL     0x0004
#define PLL_AHB_CTRL     0x0008
#define PLL_APB_CTRL     0x000C
#define PLL_DDR_CTRL     0x0010

/* Clock gating register offsets */
#define CLOCK_GATING_REG 0x0100

/* PLL control bits */
#define PLL_ENABLE       (1u << 31)
#define PLL_LOCK         (1u << 28)
#define PLL_FACTOR_SHIFT 8
#define PLL_FACTOR_MASK  0xFF

/* Peripheral clock gate bits */
#define PERIPH_UART0     (1u << 16)
#define PERIPH_UART1     (1u << 17)
#define PERIPH_SPI0      (1u << 20)
#define PERIPH_SPI1      (1u << 21)

/* Current clock frequencies (cached after init) */
static uint32_t s_core_freq = 1200;   /* MHz */
static uint32_t s_ahb_freq = 400;     /* MHz */
static uint32_t s_apb_freq = 200;     /* MHz */
static uint32_t s_dram_freq = 400;    /* MHz */

/* Volatile register access macros */
#define REG32(addr)      (*(volatile uint32_t *)(addr))

/**
 * Wait for PLL to lock with timeout
 */
static int pll_wait_lock(uint32_t pll_ctrl_reg, uint32_t timeout_ms)
{
    uint32_t timeout = timeout_ms * 1000;

    while (timeout--) {
        if (REG32(PLL_CTRL_BASE + pll_ctrl_reg) & PLL_LOCK) {
            return E_OK;
        }
        /* Small delay */
        for (volatile int i = 0; i < 100; i++);
    }

    return E_TIMEOUT;
}

/**
 * Configure a PLL to output target frequency
 * Assumes input oscillator is 24 MHz
 */
static int pll_configure(uint32_t ctrl_reg, uint32_t target_freq_mhz)
{
    const uint32_t osc_freq = 24;  /* 24 MHz input */
    uint32_t factor = (target_freq_mhz + osc_freq / 2) / osc_freq;

    /* Clamp factor to valid range */
    if (factor < 1) factor = 1;
    if (factor > PLL_FACTOR_MASK) factor = PLL_FACTOR_MASK;

    /* Disable PLL before reconfiguration */
    REG32(PLL_CTRL_BASE + ctrl_reg) &= ~PLL_ENABLE;

    /* Small delay to ensure PLL is off */
    for (volatile int i = 0; i < 1000; i++);

    /* Set new factor and enable */
    uint32_t reg_val = PLL_ENABLE | (factor << PLL_FACTOR_SHIFT);
    REG32(PLL_CTRL_BASE + ctrl_reg) = reg_val;

    /* Wait for lock */
    return pll_wait_lock(ctrl_reg, 100);
}

/**
 * Initialize system clocks based on configuration
 */
int hal_clock_init(const clock_config_t *cfg)
{
    int ret;

    if (!cfg) return E_INVAL;

    /* Configure PLLs */
    ret = pll_configure(PLL_CPU_CTRL, cfg->core_freq);
    if (ret < 0) return ret;

    ret = pll_configure(PLL_AXI_CTRL, cfg->ahb_freq);
    if (ret < 0) return ret;

    ret = pll_configure(PLL_AHB_CTRL, cfg->ahb_freq);
    if (ret < 0) return ret;

    ret = pll_configure(PLL_APB_CTRL, cfg->apb_freq);
    if (ret < 0) return ret;

    ret = pll_configure(PLL_DDR_CTRL, cfg->dram_freq);
    if (ret < 0) return ret;

    /* Cache configured frequencies */
    s_core_freq = cfg->core_freq;
    s_ahb_freq = cfg->ahb_freq;
    s_apb_freq = cfg->apb_freq;
    s_dram_freq = cfg->dram_freq;

    /* Enable default peripheral clocks */
    REG32(CLOCK_CTRL_BASE + CLOCK_GATING_REG) |= PERIPH_UART0;

    return E_OK;
}

/**
 * Query current frequency of a clock source
 */
int hal_clock_get_freq(clock_source_t src)
{
    switch (src) {
        case CLOCK_OSC_24M:
            return 24;
        case CLOCK_OSC_32K:
            return 0;  /* 32 kHz not implemented */
        case CLOCK_PLL_CORE:
            return s_core_freq;
        case CLOCK_PLL_DDR:
            return s_dram_freq;
        default:
            return E_INVAL;
    }
}

/**
 * Set divider for a clock domain
 */
int hal_clock_set_divisor(uint32_t domain, uint32_t divisor)
{
    if (divisor == 0 || divisor > 16) {
        return E_INVAL;
    }

    /* Dividers are typically (n+1) so adjust */
    uint32_t div_val = divisor - 1;

    switch (domain) {
        case 0:  /* CORE */
            /* Core clock divider */
            REG32(PLL_CTRL_BASE + 0x0020) = (REG32(PLL_CTRL_BASE + 0x0020) & ~0xFF) | div_val;
            break;
        case 1:  /* AHB */
            REG32(PLL_CTRL_BASE + 0x0024) = (REG32(PLL_CTRL_BASE + 0x0024) & ~0xFF) | div_val;
            s_ahb_freq = s_core_freq / divisor;
            break;
        case 2:  /* APB */
            REG32(PLL_CTRL_BASE + 0x0028) = (REG32(PLL_CTRL_BASE + 0x0028) & ~0xFF) | div_val;
            s_apb_freq = s_ahb_freq / divisor;
            break;
        case 3:  /* DRAM */
            REG32(PLL_CTRL_BASE + 0x002C) = (REG32(PLL_CTRL_BASE + 0x002C) & ~0xFF) | div_val;
            s_dram_freq = s_core_freq / divisor;
            break;
        default:
            return E_INVAL;
    }

    return E_OK;
}

/**
 * Enable or disable a peripheral clock gate
 */
int hal_clock_enable_periph(uint32_t periph_id, uint8_t enable)
{
    volatile uint32_t *gate_reg = (volatile uint32_t *)(CLOCK_CTRL_BASE + CLOCK_GATING_REG);
    uint32_t bit;

    /* Map peripheral ID to gate bit */
    switch (periph_id) {
        case 0:  bit = PERIPH_UART0; break;
        case 1:  bit = PERIPH_UART1; break;
        case 2:  bit = PERIPH_SPI0;  break;
        case 3:  bit = PERIPH_SPI1;  break;
        default: return E_INVAL;
    }

    if (enable) {
        *gate_reg |= bit;
    } else {
        *gate_reg &= ~bit;
    }

    return E_OK;
}
