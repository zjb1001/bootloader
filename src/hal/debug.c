/**
 * @file debug.c
 * @brief Debug HAL implementation
 *
 * Provides debugging facilities like breakpoint, register dump, etc.
 */
#include "types.h"
#include "hal/uart.h"
#include "config.h"

/**
 * Software breakpoint - drops into debugger if attached
 */
void hal_debug_breakpoint(void)
{
    __asm__ volatile("bkpt #0");
}

/**
 * Dump CPU registers to console
 */
void hal_debug_dump_regs(void)
{
    uint32_t regs[16];
    uint32_t cpsr, pc, sp;

    __asm__ volatile("stmia %0, {r0-r15}" : : "r"(regs) : "memory");

    cpsr = regs[15];  /* Actually PC in stmia context */
    __asm__ volatile("mrs %0, cpsr" : "=r"(cpsr));

    extern int hal_uart_write(uint32_t, const uint8_t *, uint32_t);

    const char *header = "\n=== Register Dump ===\n";
    hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)header, 22);

    char buf[64];
    for (int i = 0; i < 8; i++) {
        int len = 0;
        const char hex[] = "0123456789ABCDEF";

        /* "rNN: 0xXXXXXXXX" */
        buf[len++] = 'r';
        buf[len++] = '0' + i;
        buf[len++] = ':';
        buf[len++] = ' ';
        buf[len++] = '0';
        buf[len++] = 'x';

        uint32_t val = regs[i];
        for (int j = 28; j >= 0; j -= 4) {
            buf[len++] = hex[(val >> j) & 0xF];
        }
        buf[len++] = '\r';
        buf[len++] = '\n';

        hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)buf, len);
    }

    for (int i = 8; i < 16; i++) {
        int len = 0;
        const char hex[] = "0123456789ABCDEF";

        /* "rNN: 0xXXXXXXXX" */
        buf[len++] = 'r';
        if (i < 10) {
            buf[len++] = '0' + i;
        } else {
            buf[len++] = '1';
            buf[len++] = '0' + (i - 10);
        }
        buf[len++] = ':';
        buf[len++] = ' ';
        buf[len++] = '0';
        buf[len++] = 'x';

        uint32_t val = regs[i];
        for (int j = 28; j >= 0; j -= 4) {
            buf[len++] = hex[(val >> j) & 0xF];
        }
        buf[len++] = '\r';
        buf[len++] = '\n';

        hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)buf, len);
    }
}

/**
 * Enable/disable D-cache
 */
void hal_debug_set_dcache(int enable)
{
    uint32_t sctlr;
    __asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(sctlr));

    if (enable) {
        sctlr |= (1u << 2);  /* C bit */
    } else {
        sctlr &= ~(1u << 2);
    }

    __asm__ volatile("mcr p15, 0, %0, c1, c0, 0" :: "r"(sctlr));
    __asm__ volatile("isb");
}

/**
 * Enable/disable I-cache
 */
void hal_debug_set_icache(int enable)
{
    uint32_t sctlr;
    __asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(sctlr));

    if (enable) {
        sctlr |= (1u << 12);  /* I bit */
    } else {
        sctlr &= ~(1u << 12);
    }

    __asm__ volatile("mcr p15, 0, %0, c1, c0, 0" :: "r"(sctlr));
    __asm__ volatile("isb");
}

/**
 * Read CPU ID
 */
uint32_t hal_debug_get_cpuid(void)
{
    uint32_t cpuid;
    __asm__ volatile("mrc p15, 0, %0, c0, c0, 0" : "=r"(cpuid));
    return cpuid;
}

/**
 * Read cache line size
 */
uint32_t hal_debug_get_cache_line_size(void)
{
    uint32_t ccsidr;
    __asm__ volatile("mrc p15, 1, %0, c0, c0, 0" : "=r"(ccsidr));
    return 4 << ((ccsidr >> 0) & 0x7);
}
