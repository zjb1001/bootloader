/**
 * @file core/boot.h
 * @brief Boot control interface  [THREAD]
 *
 * Orchestrates the final hand-off from bootloader to kernel.
 */
#ifndef CORE_BOOT_H
#define CORE_BOOT_H

#include "types.h"

typedef enum {
    BOOT_MODE_NORMAL = 0,
    BOOT_MODE_RECOVERY,
    BOOT_MODE_FASTBOOT,
} boot_mode_t;

typedef struct {
    uint32_t     kernel_addr;   /**< Kernel load address */
    uint32_t     dtb_addr;      /**< DTB load address */
    uint32_t     ramdisk_addr;  /**< Ramdisk address (0 = unused) */
    const char  *bootargs;      /**< Kernel command line */
    boot_mode_t  mode;
} boot_params_t;

/** Initialize boot subsystem. */
int core_boot_init(void);

/** Pre-boot sanity checks. @return 0=ok, 1=degraded, <0=fail. */
int core_boot_check(void);

/** Jump to kernel. Does NOT return. */
void core_boot_kernel(const boot_params_t *params);

/** Enter recovery mode. Does NOT return. */
void core_boot_recovery(void);

int         core_set_boot_mode(boot_mode_t mode);
boot_mode_t core_get_boot_mode(void);

#endif /* CORE_BOOT_H */
