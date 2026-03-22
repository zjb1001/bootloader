/**
 * @file cache.S
 * @brief ARMv7 cache maintenance operations
 */
    .syntax unified
    .arch   armv7-a

    .global arm_invalidate_icache
    .global arm_invalidate_dcache
    .global arm_clean_dcache
    .global arm_flush_dcache

    .text

/* Invalidate entire instruction cache */
arm_invalidate_icache:
    mov     r0, #0
    mcr     p15, 0, r0, c7, c5, 0   @ ICIALLU
    dsb
    isb
    bx      lr

/* Invalidate entire data cache */
arm_invalidate_dcache:
    mov     r0, #0
    mcr     p15, 0, r0, c15, c5, 0  @ DCCIMVAC - invalidate by set/way
    dsb
    isb
    bx      lr

/* Clean entire data cache */
arm_clean_dcache:
    mov     r0, #0
    mcr     p15, 0, r0, c7, c10, 0  @ DCCSW - clean by set/way
    dsb
    isb
    bx      lr

/* Clean and invalidate entire data cache */
arm_flush_dcache:
    mov     r0, #0
    mcr     p15, 0, r0, c15, c6, 0  @ DCCISW - clean&invalidate by set/way
    dsb
    isb
    bx      lr

/* Enable data cache - r0 = SCTLR value */
arm_enable_dcache:
    mrc     p15, 0, r1, c1, c0, 0
    orr     r1, r1, #0x0004          @ Set C bit
    mcr     p15, 0, r1, c1, c0, 0
    dsb
    isb
    bx      lr

/* Disable data cache */
arm_disable_dcache:
    mrc     p15, 0, r1, c1, c0, 0
    bic     r1, r1, #0x0004          @ Clear C bit
    mcr     p15, 0, r1, c1, c0, 0
    dsb
    isb
    bx      lr
