/**
 * @file log.c
 * @brief Logging subsystem
 *
 * Compile-time switchable via CONFIG_LOG_LEVEL.
 * Production builds default to errors/warnings only.
 */
#include "types.h"
#include "hal/uart.h"
#include "config.h"

#include <stdarg.h>

typedef enum {
    LOG_ERROR = 0,
    LOG_WARN,
    LOG_INFO,
    LOG_DEBUG,
} log_level_t;

#ifndef CONFIG_LOG_LEVEL
#ifdef DEBUG
#define CONFIG_LOG_LEVEL LOG_DEBUG
#else
#define CONFIG_LOG_LEVEL LOG_WARN
#endif
#endif

void bl_log(log_level_t level, const char *fmt, ...)
{
    if (level > CONFIG_LOG_LEVEL) return;

    va_list ap;
    va_start(ap, fmt);
    /* TODO: minimal vsnprintf → hal_uart_write */
    (void)fmt;
    va_end(ap);
}
