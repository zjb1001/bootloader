/**
 * @file config.h
 * @brief System configuration constants
 *
 * All hardware addresses, size limits, and timing budgets come from
 * design/01_SYSTEM_PROPERTIES.md.  Platform overrides live in
 * src/platform/<board>/board.h.
 */
#ifndef BOOTLOADER_CONFIG_H
#define BOOTLOADER_CONFIG_H

#include "types.h"

/* ═══════════════════════════════════════════════
 *  Memory Map
 * ═══════════════════════════════════════════════ */
#define SRAM_BASE               0x00000000u
#define SRAM_SIZE               (64u * 1024u)           /* 64 KB */

#define DRAM_BASE               0x80000000u
#define DRAM_SIZE               (2048u * 1024u * 1024u) /* 2 GB */

#define FLASH_XIP_BASE          0x10000000u
#define FLASH_XIP_SIZE          (256u * 1024u * 1024u)  /* 256 MB */

#define OTP_BASE                0x40000000u
#define OTP_SIZE                (8u * 1024u)             /* 8 KB */

/* ═══════════════════════════════════════════════
 *  Partition Layout  (in Flash block-access space)
 * ═══════════════════════════════════════════════ */
#define BOOTLOADER_PART_OFFSET  0x00000000u
#define BOOTLOADER_PART_SIZE    (512u * 1024u)           /* 512 KB */
#define BOOTLOADER_BK_OFFSET    0x00080000u

#define CONFIG_PART_OFFSET      0x00100000u
#define CONFIG_PART_SIZE        (256u * 1024u)           /* 256 KB */

#define KERNEL_PART_OFFSET      0x00140000u
#define KERNEL_PART_MAX_SIZE    (128u * 1024u * 1024u)   /* 128 MB */

#define RECOVERY_PART_OFFSET    0x08140000u
#define RECOVERY_PART_MAX_SIZE  (64u * 1024u * 1024u)    /* 64 MB */

#define DTB_MAX_SIZE            (1u * 1024u * 1024u)     /* 1 MB */

/* ═══════════════════════════════════════════════
 *  Size Constraints
 * ═══════════════════════════════════════════════ */
#define BOOTLOADER_MAX_IMAGE_SIZE   (256u * 1024u)       /* 256 KB */
#define BOOTLOADER_STACK_SIZE       (32u * 1024u)        /* 32 KB */
#define BOOTLOADER_BSS_SIZE         (64u * 1024u)        /* 64 KB */
#define BOOTLOADER_HEAP_SIZE        (16u * 1024u)        /* 16 KB */

/* ═══════════════════════════════════════════════
 *  Timing Budgets (ms)
 * ═══════════════════════════════════════════════ */
#define BOOT_STAGE1_MAX_TIME    100u
#define BOOT_STAGE2_MAX_TIME    300u
#define BOOT_STAGE3_MAX_TIME    500u
#define BOOT_STAGE4_MAX_TIME    600u
#define BOOT_STAGE5_MAX_TIME    500u
#define BOOT_TOTAL_MAX_TIME     2000u

/* ═══════════════════════════════════════════════
 *  Performance Thresholds
 * ═══════════════════════════════════════════════ */
#define MIN_DRAM_SPEED_MHZ      400u
#define MIN_FLASH_READ_MB_S     25u
#define MAX_SHA256_TIME_MS      100u   /* for 128 MB payload */

/* ═══════════════════════════════════════════════
 *  Hardware Defaults
 * ═══════════════════════════════════════════════ */
#define DEFAULT_UART_BAUDRATE   115200u
#define WATCHDOG_TIMEOUT_MS     30000u
#define DEFAULT_UART_ID         0u

/* ═══════════════════════════════════════════════
 *  Boot Flags
 * ═══════════════════════════════════════════════ */
#define BOOT_FLAG_SECURE_BOOT   0x01u
#define BOOT_FLAG_VERBOSE       0x02u
#define BOOT_FLAG_RECOVERY      0x04u
#define BOOT_FLAG_FASTBOOT      0x08u

/* ═══════════════════════════════════════════════
 *  Misc Limits
 * ═══════════════════════════════════════════════ */
#define MAX_BOOTARGS_LEN        1024u
#define PARTITION_TABLE_MAGIC   0x50415254u  /* "PART" */
#define MAX_PARTITIONS          16u

#endif /* BOOTLOADER_CONFIG_H */
