/**
 * @file irq.S
 * @brief ARMv7 interrupt vector table and dispatcher
 */
    .syntax unified
    .arch   armv7-a
    .section .text.vectors, "ax"

    .global _vector_table
_vector_table:
    ldr     pc, =_start             @ Reset
    ldr     pc, =_undef_handler     @ Undefined instruction
    ldr     pc, =_svc_handler       @ SVC
    ldr     pc, =_prefetch_handler  @ Prefetch abort
    ldr     pc, =_data_handler      @ Data abort
    nop                             @ Reserved
    ldr     pc, =_irq_handler       @ IRQ
    ldr     pc, =_fiq_handler       @ FIQ

    .weak   _undef_handler
    .weak   _svc_handler
    .weak   _prefetch_handler
    .weak   _data_handler
    .weak   _irq_handler
    .weak   _fiq_handler

_undef_handler:
_svc_handler:
_prefetch_handler:
_data_handler:
_fiq_handler:
    b       .                       @ Default: infinite loop

_irq_handler:
    /* TODO: save context, call C IRQ dispatcher, restore */
    b       .
