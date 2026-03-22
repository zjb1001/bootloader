/**
 * @file boot.S
 * @brief ARMv7 Stage-1 startup assembly
 *
 * Responsibilities:
 *   1. Disable interrupts
 *   2. Invalidate caches
 *   3. Set up stack pointer
 *   4. Clear BSS
 *   5. Copy .data from Flash to SRAM
 *   6. Jump to _crt0_init (C runtime)
 */

    .syntax unified
    .arch   armv7-a
    .section .text.start, "ax"
    .global _start
    .type   _start, %function

_start:
    /* ── Disable IRQ & FIQ ── */
    cpsid   if

    /* ── Set SVC mode ── */
    mrs     r0, cpsr
    bic     r0, r0, #0x1F
    orr     r0, r0, #0x13       @ SVC mode
    msr     cpsr_c, r0

    /* ── Invalidate I-cache & TLB ── */
    mov     r0, #0
    mcr     p15, 0, r0, c7, c5, 0   @ ICIALLU - invalidate I-cache
    mcr     p15, 0, r0, c8, c7, 0   @ TLBIALL - invalidate TLB
    dsb
    isb

    /* ── Disable MMU and caches ── */
    mrc     p15, 0, r0, c1, c0, 0   @ Read SCTLR
    bic     r0, r0, #0x0005          @ Clear M and C bits
    mcr     p15, 0, r0, c1, c0, 0   @ Write SCTLR
    isb

    /* ── Set up stack ── */
    ldr     sp, =_stack_top

    /* ── Clear BSS ── */
    ldr     r0, =_bss_start
    ldr     r1, =_bss_end
    mov     r2, #0
    cmp     r0, r1
1:  strlt   r2, [r0], #4
    blt     1b

    /* ── Jump to C runtime init ── */
    bl      _crt0_init

    /* ── Should not return ── */
2:  wfi
    b       2b

    .size   _start, . - _start
