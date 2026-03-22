/**
 * bootloader.ld – Linker script for ARM bootloader
 *
 * Memory layout aligned with design/01_SYSTEM_PROPERTIES.md
 */
ENTRY(_start)

MEMORY
{
    SRAM  (rwx) : ORIGIN = 0x00000000, LENGTH = 64K
    FLASH (rx)  : ORIGIN = 0x10000000, LENGTH = 256K
    DRAM  (rwx) : ORIGIN = 0x80000000, LENGTH = 2048M
    OTP   (r)   : ORIGIN = 0x40000000, LENGTH = 8K
}

/* Stack at top of SRAM */
_stack_top = ORIGIN(SRAM) + LENGTH(SRAM);
_stack_size = 32K;

SECTIONS
{
    /* ─── Code (in Flash/XIP) ─── */
    .text : ALIGN(4)
    {
        KEEP(*(.text.vectors))      /* Exception vector table first */
        KEEP(*(.text.start))        /* _start entry point */
        *(.text .text.*)
    } > FLASH

    /* ─── Read-only data ─── */
    .rodata : ALIGN(4)
    {
        *(.rodata .rodata.*)
    } > FLASH

    /* ─── Initialised data (load from Flash, run in SRAM) ─── */
    _data_load = LOADADDR(.data);
    .data : ALIGN(4)
    {
        _data_start = .;
        *(.data .data.*)
        _data_end = .;
    } > SRAM AT > FLASH

    /* ─── Zero-initialised data ─── */
    .bss (NOLOAD) : ALIGN(4)
    {
        _bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        _bss_end = .;
    } > SRAM

    /* ─── Heap (after BSS, grows up) ─── */
    .heap (NOLOAD) : ALIGN(4)
    {
        _heap_start = .;
        . += 16K;
        _heap_end = .;
    } > SRAM

    /* ─── Stack guard ─── */
    ASSERT(_stack_top - _bss_end >= _stack_size,
           "ERROR: not enough SRAM for stack + BSS + heap")

    /* ─── Discard ─── */
    /DISCARD/ :
    {
        *(.comment)
        *(.ARM.attributes)
    }
}
