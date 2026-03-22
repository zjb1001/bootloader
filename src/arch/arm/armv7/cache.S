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

arm_invalidate_icache:
    /* TODO: ICIALLU */
    bx      lr

arm_invalidate_dcache:
    /* TODO: iterate sets/ways */
    bx      lr

arm_clean_dcache:
    /* TODO: DCCSW iterate */
    bx      lr

arm_flush_dcache:
    /* TODO: DCCISW iterate */
    bx      lr
