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

    /* ── Invalidate I-cache & D-cache ── */
    /* TODO: implement cache invalidation sequence */

    /* ── Set up stack ── */
    ldr     sp, =_stack_top

    /* ── Clear BSS ── */
    ldr     r0, =_bss_start
    ldr     r1, =_bss_end
    mov     r2, #0
.bss_loop:
    cmp     r0, r1
    strlt   r2, [r0], #4
    blt     .bss_loop

    /* ── Jump to C runtime init ── */
    bl      _crt0_init

    /* ── Should not return ── */
.hang:
    wfi
    b       .hang

    .size   _start, . - _start
