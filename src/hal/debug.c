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
#if defined(__arm__) || defined(__aarch64__)
    __asm__ volatile("bkpt #0");
#else
    __builtin_trap();
#endif
}

/**
 * Dump CPU registers to console
 */
void hal_debug_dump_regs(void)
{
    uint32_t regs[16];

#if defined(__arm__)
    uint32_t cpsr;
    __asm__ volatile(
        "str r0,  [%0, #0]\n\t"
        "str r1,  [%0, #4]\n\t"
        "str r2,  [%0, #8]\n\t"
        "str r3,  [%0, #12]\n\t"
        "str r4,  [%0, #16]\n\t"
        "str r5,  [%0, #20]\n\t"
        "str r6,  [%0, #24]\n\t"
        "str r7,  [%0, #28]\n\t"
        "str r8,  [%0, #32]\n\t"
        "str r9,  [%0, #36]\n\t"
        "str r10, [%0, #40]\n\t"
        "str r11, [%0, #44]\n\t"
        "str r12, [%0, #48]\n\t"
        "str sp,  [%0, #52]\n\t"
        "str lr,  [%0, #56]\n\t"
        "mov r0, pc\n\t"
        "str r0,  [%0, #60]"
        : : "r"(regs) : "r0", "memory"
    );
    __asm__ volatile("mrs %0, cpsr" : "=r"(cpsr));
    (void)cpsr;
#else
    for (int i = 0; i < 16; i++) regs[i] = 0;
#endif

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
#if defined(__arm__) || defined(__aarch64__)
    uint32_t sctlr;
    __asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(sctlr));

    if (enable) {
        sctlr |= (1u << 2);  /* C bit */
    } else {
        sctlr &= ~(1u << 2);
    }

    __asm__ volatile("mcr p15, 0, %0, c1, c0, 0" :: "r"(sctlr));
    __asm__ volatile("isb");
#else
    (void)enable;
#endif
}

/**
 * Enable/disable I-cache
 */
void hal_debug_set_icache(int enable)
{
#if defined(__arm__) || defined(__aarch64__)
    uint32_t sctlr;
    __asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(sctlr));

    if (enable) {
        sctlr |= (1u << 12);  /* I bit */
    } else {
        sctlr &= ~(1u << 12);
    }

    __asm__ volatile("mcr p15, 0, %0, c1, c0, 0" :: "r"(sctlr));
    __asm__ volatile("isb");
#else
    (void)enable;
#endif
}

/**
 * Read CPU ID
 */
uint32_t hal_debug_get_cpuid(void)
{
    uint32_t cpuid;
#if defined(__arm__) || defined(__aarch64__)
    __asm__ volatile("mrc p15, 0, %0, c0, c0, 0" : "=r"(cpuid));
#else
    cpuid = 0;
#endif
    return cpuid;
}

/**
 * Read cache line size
 */
uint32_t hal_debug_get_cache_line_size(void)
{
    uint32_t ccsidr;
#if defined(__arm__) || defined(__aarch64__)
    __asm__ volatile("mrc p15, 1, %0, c0, c0, 0" : "=r"(ccsidr));
#else
    ccsidr = 0;
#endif
    return 4 << ((ccsidr >> 0) & 0x7);
}
