/**
 * @file recovery.c
 * @brief Boot failure detection and recovery actions
 */
#include "core/recovery.h"
#include "errno.h"

#include <stdarg.h>

recovery_action_t core_handle_boot_failure(uint32_t reason)
{
    /* TODO: decide recovery strategy based on failure reason */
    (void)reason;
    return RECOVERY_FALLBACK;
}

int core_recovery_fallback_boot(void)
{
    /* TODO: locate backup partition and attempt boot */
    return E_OK;
}

void core_recovery_bootloader_mode(void)
{
    /* TODO: enter interactive loop (UART shell, etc.) */
    for (;;) { }
}

void core_recovery_log(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    /* TODO: write to audit log over UART / flash */
    (void)fmt;
    va_end(ap);
}
