/**
 * @file mmu.S
 * @brief ARMv7 MMU setup assembly helpers
 */
    .syntax unified
    .arch   armv7-a

    .global arm_mmu_enable
    .global arm_mmu_disable
    .global arm_tlb_invalidate
    .global arm_set_ttbr0
    .global arm_set_dacr

    .text

/* Enable MMU by setting SCTLR.M bit */
arm_mmu_enable:
    mrc     p15, 0, r0, c1, c0, 0   @ Read SCTLR
    orr     r0, r0, #0x0001          @ Set M bit
    mcr     p15, 0, r0, c1, c0, 0   @ Write SCTLR
    dsb
    isb
    bx      lr

/* Disable MMU by clearing SCTLR.M bit */
arm_mmu_disable:
    mrc     p15, 0, r0, c1, c0, 0   @ Read SCTLR
    bic     r0, r0, #0x0001          @ Clear M bit
    bic     r0, r0, #0x0004          @ Clear C bit (data cache)
    bic     r0, r0, #0x1000          @ Clear I bit (instruction cache)
    mcr     p15, 0, r0, c1, c0, 0   @ Write SCTLR
    dsb
    isb
    bx      lr

/* Invalidate entire TLB */
arm_tlb_invalidate:
    mov     r0, #0
    mcr     p15, 0, r0, c8, c7, 0   @ TLBIALL
    dsb
    isb
    bx      lr

/* Set TTBR0 (page table base) - r0 = base address */
arm_set_ttbr0:
    mcr     p15, 0, r0, c2, c0, 0   @ Write TTBR0
    dsb
    isb
    bx      lr

/* Set DACR (domain access control) - r0 = DACR value */
arm_set_dacr:
    mcr     p15, 0, r0, c3, c0, 0   @ Write DACR
    isb
    bx      lr
