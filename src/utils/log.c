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
#include "errno.h"

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

static const char *level_prefix[] = {
    [LOG_ERROR] = "ERR",
    [LOG_WARN]  = "WRN",
    [LOG_INFO]  = "INF",
    [LOG_DEBUG] = "DBG",
};

/* Simple buffer for formatted output */
static char log_buf[256];

/* Minimal integer to string conversion */
static int itoa_str(uint32_t value, char *str, int base, int is_signed)
{
    char tmp[16];
    int i = 0, j = 0;

    if (base < 2 || base > 16) {
        str[0] = '\0';
        return 0;
    }

    int32_t signed_value = (int32_t)value;

    if (is_signed && signed_value < 0) {
        str[j++] = '-';
        value = (uint32_t)(-signed_value);
    }

    if (value == 0) {
        str[j++] = '0';
        str[j] = '\0';
        return j;
    }

    while (value > 0) {
        int digit = value % base;
        tmp[i++] = (digit < 10) ? ('0' + digit) : ('a' + digit - 10);
        value /= base;
    }

    while (i > 0) {
        str[j++] = tmp[--i];
    }
    str[j] = '\0';
    return j;
}

/* Minimal vsnprintf implementation */
static int bl_vsnprintf(char *buf, size_t size, const char *fmt, va_list ap)
{
    size_t i = 0;

    while (*fmt && i < size - 1) {
        if (*fmt == '%') {
            fmt++;
            if (*fmt == '\0') break;

            /* Width modifier (simplified) */
            int width = 0;
            int pad_zero = 0;
            while (*fmt >= '0' && *fmt <= '9') {
                if (*fmt == '0' && width == 0) pad_zero = 1;
                width = width * 10 + (*fmt - '0');
                fmt++;
            }

            /* Length modifiers */
            int is_long = 0;
            if (*fmt == 'l') {
                is_long = 1;
                fmt++;
            }

            switch (*fmt) {
                case 'd':
                case 'i': {
                    long val = is_long ? va_arg(ap, long) : va_arg(ap, int);
                    char tmp[16];
                    int len = itoa_str((uint32_t)val, tmp, 10, 1);
                    for (int j = 0; j < len && i < size - 1; j++) {
                        buf[i++] = tmp[j];
                    }
                    break;
                }
                case 'u': {
                    unsigned long val = is_long ? va_arg(ap, unsigned long) : va_arg(ap, unsigned int);
                    char tmp[16];
                    int len = itoa_str((uint32_t)val, tmp, 10, 0);
                    for (int j = 0; j < len && i < size - 1; j++) {
                        buf[i++] = tmp[j];
                    }
                    break;
                }
                case 'x':
                case 'X': {
                    unsigned long val = is_long ? va_arg(ap, unsigned long) : va_arg(ap, unsigned int);
                    char tmp[16];
                    int len = itoa_str((uint32_t)val, tmp, 16, 0);
                    if (width > len) {
                        while (width > len && i < size - 1) {
                            buf[i++] = pad_zero ? '0' : ' ';
                            width--;
                        }
                    }
                    for (int j = 0; j < len && i < size - 1; j++) {
                        buf[i++] = tmp[j];
                    }
                    break;
                }
                case 'p': {
                    void *ptr = va_arg(ap, void *);
                    char tmp[16];
                    int len = itoa_str((uint32_t)(uintptr)ptr, tmp, 16, 0);
                    /* Pad to 8 hex digits */
                    while (len < 8) {
                        if (i < size - 1) buf[i++] = '0';
                        len++;
                    }
                    for (int j = 0; j < len && i < size - 1; j++) {
                        buf[i++] = tmp[j];
                    }
                    break;
                }
                case 's': {
                    const char *str = va_arg(ap, const char *);
                    if (str) {
                        while (*str && i < size - 1) {
                            buf[i++] = *str++;
                        }
                    } else {
                        const char *null_str = "(null)";
                        while (*null_str && i < size - 1) {
                            buf[i++] = *null_str++;
                        }
                    }
                    break;
                }
                case 'c': {
                    char c = (char)va_arg(ap, int);
                    if (i < size - 1) buf[i++] = c;
                    break;
                }
                case '%':
                    if (i < size - 1) buf[i++] = '%';
                    break;
                default:
                    if (i < size - 1) buf[i++] = '%';
                    if (i < size - 1) buf[i++] = *fmt;
                    break;
            }
            fmt++;
        } else {
            buf[i++] = *fmt++;
        }
    }
    buf[i] = '\0';
    return (int)i;
}

void bl_log(log_level_t level, const char *fmt, ...)
{
    if (level > CONFIG_LOG_LEVEL) return;

    va_list ap;
    int len = 0;

    /* Add level prefix */
    if (level <= LOG_DEBUG) {
        const char *prefix = level_prefix[level];
        while (*prefix && len < (int)sizeof(log_buf) - 1) {
            log_buf[len++] = *prefix++;
        }
        if (len < (int)sizeof(log_buf) - 1) log_buf[len++] = ':';
        if (len < (int)sizeof(log_buf) - 1) log_buf[len++] = ' ';
    }

    va_start(ap, fmt);
    int msg_len = bl_vsnprintf(log_buf + len, sizeof(log_buf) - len, fmt, ap);
    va_end(ap);

    len += msg_len;
    if (len < (int)sizeof(log_buf) - 2) {
        log_buf[len++] = '\r';
        log_buf[len++] = '\n';
    }
    log_buf[len] = '\0';

    /* Output to UART if initialized */
    extern int hal_uart_write(uint32_t, const uint8_t *, uint32_t);
    hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)log_buf, len);
}

/* Public API wrappers */
void log_error(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    bl_log(LOG_ERROR, fmt, ap);
    va_end(ap);
}

void log_warn(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    bl_log(LOG_WARN, fmt, ap);
    va_end(ap);
}

void log_info(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    bl_log(LOG_INFO, fmt, ap);
    va_end(ap);
}

void log_debug(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    bl_log(LOG_DEBUG, fmt, ap);
    va_end(ap);
}
