/**
 * @file core/recovery.h
 * @brief Failure recovery interface  [THREAD]
 */
#ifndef CORE_RECOVERY_H
#define CORE_RECOVERY_H

#include "types.h"

typedef enum {
    RECOVERY_FALLBACK = 0,          /**< Boot from backup partition */
    RECOVERY_FACTORY_RESET,         /**< Restore factory image */
    RECOVERY_BOOTLOADER_MODE,       /**< Stay in bootloader shell */
    RECOVERY_HALT,                  /**< Stop and wait */
} recovery_action_t;

/** Determine recovery action based on failure reason. */
recovery_action_t core_handle_boot_failure(uint32_t reason);

/** Attempt to boot from the backup partition. */
int core_recovery_fallback_boot(void);

/** Enter interactive bootloader mode. Does NOT return. */
void core_recovery_bootloader_mode(void);

/** Log recovery event (audit trail). */
void core_recovery_log(const char *fmt, ...);

#endif /* CORE_RECOVERY_H */
