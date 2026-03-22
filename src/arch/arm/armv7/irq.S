/**
 * @file irq.S
 * @brief ARMv7 interrupt vector table and dispatcher
 */
    .syntax unified
    .arch   armv7-a
    .section .text.vectors, "ax"

    .global _vector_table
    .type _vector_table, %object

_vector_table:
    ldr     pc, =_start             @ Reset
    ldr     pc, =_undef_handler     @ Undefined instruction
    ldr     pc, =_svc_handler       @ SVC
    ldr     pc, =_prefetch_handler  @ Prefetch abort
    ldr     pc, =_data_handler      @ Data abort
    nop                             @ Reserved
    ldr     pc, =_irq_handler       @ IRQ
    ldr     pc, =_fiq_handler       @ FIQ

    .size   _vector_table, . - _vector_table

    .weak   _undef_handler
    .weak   _svc_handler
    .weak   _prefetch_handler
    .weak   _data_handler
    .weak   _irq_handler
    .weak   _fiq_handler

    .text

/* Default exception handlers - infinite loop */
_undef_handler:
_svc_handler:
_prefetch_handler:
_data_handler:
_fiq_handler:
    b       .                       @ Default: infinite loop

/* IRQ handler entry point */
_irq_handler:
    /* Save registers */
    sub     sp, sp, #4 * 17
    stmia   sp, {r0-r12, lr}

    /* Save CPSR and SPSR */
    mrs     r0, cpsr
    mrs     r1, spsr
    stmia   sp, {r0, r1}

    /* Call C IRQ handler */
    bl      hal_irq_handler_entry

    /* Restore CPSR and SPSR */
    ldmia   sp, {r0, r1}
    msr     cpsr_c, r0
    msr     spsr_cxsf, r1

    /* Restore registers */
    ldmia   sp, {r0-r12, pc}^       @ Return from interrupt

/* External reference to C handler */
    .global hal_irq_handler_entry
    .weak   hal_irq_handler_entry
    .type   hal_irq_handler_entry, %function
hal_irq_handler_entry:
    bx      lr
