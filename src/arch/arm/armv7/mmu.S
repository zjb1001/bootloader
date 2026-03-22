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

    .text

arm_mmu_enable:
    /* TODO: set SCTLR.M bit */
    bx      lr

arm_mmu_disable:
    /* TODO: clear SCTLR.M bit */
    bx      lr

arm_tlb_invalidate:
    /* TODO: TLBIALL */
    bx      lr

arm_set_ttbr0:
    /* r0 = page table base address */
    /* TODO: MCR p15, 0, r0, c2, c0, 0 */
    bx      lr
