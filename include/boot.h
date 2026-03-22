/**
 * @file boot.h
 * @brief Top-level boot sequence API
 *
 * Defines the stage functions called from startup.c / main.c.
 */
#ifndef BOOTLOADER_BOOT_H
#define BOOTLOADER_BOOT_H

#include "types.h"

/** Full boot sequence entry point. */
int boot_main(void);

/** Stage 1 – early assembly→C bridge (stack, BSS). */
int boot_stage1_early_init(void);

/** Stage 2 – HAL initialization (clock, DRAM, IRQ, UART). */
int boot_stage2_hw_init(void);

/** Stage 3 – Power-On Self Test. */
int boot_stage3_selftest(void);

/** Stage 4 – Load & verify kernel image. */
int boot_stage4_load_verify(void);

/** Stage 5 – Prepare parameters and jump to kernel. */
int boot_stage5_start_kernel(void);

#endif /* BOOTLOADER_BOOT_H */
