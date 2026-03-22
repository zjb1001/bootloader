/**
 * @file console.c
 * @brief Debug console output (wraps UART)
 *
 * Provides simple console I/O functions for debugging.
 */
#include "driver/console.h"
#include "hal/uart.h"
#include "config.h"
#include "types.h"

/* Console is bound to DEFAULT_UART_ID */
#define CONSOLE_UART_ID  DEFAULT_UART_ID

/* RX buffer for simple input handling */
#define RX_BUFFER_SIZE  128
static uint8_t s_rx_buffer[RX_BUFFER_SIZE];
static uint32_t s_rx_head = 0;
static uint32_t s_rx_tail = 0;

/**
 * Initialize console
 */
int console_init(uint32_t baudrate)
{
    uart_config_t cfg = {
        .baudrate = baudrate,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = 0
    };

    return hal_uart_init(CONSOLE_UART_ID, &cfg);
}

/**
 * Put character to console
 */
int console_putchar(int c)
{
    uint8_t ch = (uint8_t)c;
    hal_uart_write(CONSOLE_UART_ID, &ch, 1);
    return c;
}

/**
 * Get character from console (non-blocking)
 */
int console_getchar(void)
{
    if (s_rx_head != s_rx_tail) {
        uint8_t c = s_rx_buffer[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) % RX_BUFFER_SIZE;
        return c;
    }

    /* Check for new data */
    if (hal_uart_available(CONSOLE_UART_ID)) {
        uint8_t c;
        hal_uart_read(CONSOLE_UART_ID, &c, 1);
        return c;
    }

    return -1;  /* No data available */
}

/**
 * Get character from console (blocking)
 */
int console_getchar_wait(void)
{
    int c;
    while ((c = console_getchar()) < 0) {
        __asm__ volatile("nop");
    }
    return c;
}

/**
 * Put string to console
 */
int console_puts(const char *s)
{
    if (!s) return E_INVAL;

    hal_uart_write(CONSOLE_UART_ID, (const uint8_t *)s, (uint32_t)__builtin_strlen(s));
    return E_OK;
}

/**
 * Read line from console
 */
int console_gets(char *buf, uint32_t max_len)
{
    if (!buf || max_len == 0) return E_INVAL;

    uint32_t i = 0;

    while (i < max_len - 1) {
        int c = console_getchar_wait();

        if (c == '\r' || c == '\n') {
            /* Echo newline */
            console_putchar('\r');
            console_putchar('\n');
            break;
        } else if (c == '\b' || c == 127) {
            /* Backspace */
            if (i > 0) {
                i--;
                console_putchar('\b');
                console_putchar(' ');
                console_putchar('\b');
            }
        } else if (c >= 32 && c < 127) {
            /* Printable character */
            buf[i++] = (char)c;
            console_putchar(c);
        }
    }

    buf[i] = '\0';
    return i;
}
