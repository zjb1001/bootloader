/**
 * @file sys.c
 * @brief ARMv7 system-level C utilities
 */
#include "types.h"

void arm_dsb(void)
{
    __asm__ volatile("dsb" ::: "memory");
}

void arm_dmb(void)
{
    __asm__ volatile("dmb" ::: "memory");
}

void arm_isb(void)
{
    __asm__ volatile("isb" ::: "memory");
}

void arm_wfi(void)
{
    __asm__ volatile("wfi");
}
