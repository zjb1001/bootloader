/**
 * @file assert.c
 * @brief Runtime assertion handler
 */
#include "types.h"
#include "hal/uart.h"
#include "config.h"
#include "errno.h"

/* Assertion failure handler - called when bl_assert() fails */
void bl_assert_fail(const char *expr, const char *file, int line)
{
    const char *msg = "\n*** ASSERTION FAILED ***\n";
    const char *expr_msg = "Expression: ";
    const char *file_msg = "\nFile: ";
    const char *line_msg = "\nLine: ";

    /* Output error message to UART */
    extern int hal_uart_write(uint32_t, const uint8_t *, uint32_t);
    hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)msg, (uint32_t)__builtin_strlen(msg));
    hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)expr_msg, (uint32_t)__builtin_strlen(expr_msg));
    if (expr) {
        hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)expr, (uint32_t)__builtin_strlen(expr));
    }
    if (file) {
        hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)file_msg, (uint32_t)__builtin_strlen(file_msg));
        hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)file, (uint32_t)__builtin_strlen(file));
    }
    if (line > 0) {
        char line_buf[16];
        int i = 0;
        int tmp = line;
        /* Convert line number to string */
        do {
            line_buf[i++] = '0' + (tmp % 10);
            tmp /= 10;
        } while (tmp > 0);
        line_buf[i] = '\n';
        /* Reverse digits */
        for (int j = 0; j < i / 2; j++) {
            char t = line_buf[j];
            line_buf[j] = line_buf[i - 1 - j];
            line_buf[i - 1 - j] = t;
        }
        hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)line_msg, (uint32_t)__builtin_strlen(line_msg));
        hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)line_buf, i + 1);
    }
    hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)"\n", 1);

    /* Trigger watchdog reset or halt */
#ifdef CONFIG_WATCHDOG
    extern void driver_watchdog_force_reset(void);
    driver_watchdog_force_reset();
#endif

    /* Halt if watchdog not available */
    for (;;) {
#if defined(__arm__) || defined(__aarch64__)
        __asm__ volatile ("wfi");
#endif
    }
}
