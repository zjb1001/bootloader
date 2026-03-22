/**
 * @file uart.c
 * @brief UART HAL implementation
 *
 * Provides UART driver for console output and debugging.
 * Platform specific for ARMv7-A with PL011 compatible UART.
 */
#include "hal/uart.h"
#include "hal/clock.h"
#include "errno.h"
#include "types.h"

/* UART register base addresses */
#define UART0_BASE  0x01C28000u
#define UART1_BASE  0x01C28400u
#define UART2_BASE  0x01C28800u
#define UART3_BASE  0x01C28C00u

/* PL011 UART register offsets */
#define UART_DR     0x0000  /* Data register */
#define UART_FR     0x0018  /* Flag register */
#define UART_IBRD   0x0024  /* Integer baud rate divisor */
#define UART_FBRD   0x0028  /* Fractional baud rate divisor */
#define UART_LCRH   0x002C  /* Line control register */
#define UART_CR     0x0030  /* Control register */
#define UART_IFLS   0x0034  /* Interrupt FIFO level select */
#define UART_IMSC   0x0038  /* Interrupt mask set/clear */
#define UART_ICR    0x0044  /* Interrupt clear register */

/* Flag register bits */
#define FR_TXFE     (1u << 7)  /* TX FIFO empty */
#define FR_RXFF     (1u << 6)  /* RX FIFO full */
#define FR_TXFF     (1u << 5)  /* TX FIFO full */
#define FR_RXFE     (1u << 4)  /* RX FIFO empty */

/* Control register bits */
#define CR_UARTEN   (1u << 0)  /* UART enable */
#define CR_TXE      (1u << 8)  /* TX enable */
#define CR_RXE      (1u << 9)  /* RX enable */

/* Line control bits */
#define LCRH_WLEN_8 (3u << 5)  /* 8-bit word length */
#define LCRH_WLEN_7 (2u << 5)
#define LCRH_WLEN_6 (1u << 5)
#define LCRH_WLEN_5 (0u << 5)
#define LCRH_FEN    (1u << 4)  /* Enable FIFOs */

/* Interrupt bits */
#define INT_RX      (1u << 4)  /* RX interrupt */
#define INT_TX      (1u << 5)  /* TX interrupt */

#define REG32(addr) (*(volatile uint32_t *)(addr))

/* Convert physical UART ID to base address */
static inline uint32_t uart_get_base(uint32_t uart_id)
{
    switch (uart_id) {
        case 0: return UART0_BASE;
        case 1: return UART1_BASE;
        case 2: return UART2_BASE;
        case 3: return UART3_BASE;
        default: return 0;
    }
}

/**
 * Calculate UART baud rate divisor
 * Formula: baud = UARTCLK / (16 * div)
 */
static void uart_set_baudrate(uint32_t base, uint32_t uart_clk, uint32_t baudrate)
{
    uint32_t div = uart_clk / (16 * baudrate);
    uint32_t rem = uart_clk % (16 * baudrate);
    uint32_t frac = ((rem * 64) + (16 * baudrate) / 2) / (16 * baudrate);

    REG32(base + UART_IBRD) = div;
    REG32(base + UART_FBRD) = frac & 0x3F;
}

/**
 * Initialize UART with specified configuration
 */
int hal_uart_init(uint32_t uart_id, const uart_config_t *cfg)
{
    uint32_t base = uart_get_base(uart_id);

    if (!cfg || !base) return E_INVAL;

    /* Get UART clock frequency */
    int uart_clk = hal_clock_get_freq(CLOCK_PLL_CORE);
    if (uart_clk <= 0) uart_clk = 400;  /* Default to 400 MHz */

    /* Disable UART before configuration */
    REG32(base + UART_CR) = 0;

    /* Wait for TX to complete */
    while (!(REG32(base + UART_FR) & FR_TXFE));

    /* Disable interrupts */
    REG32(base + UART_IMSC) = 0;

    /* Clear pending interrupts */
    REG32(base + UART_ICR) = 0x7FF;

    /* Set baud rate */
    uart_set_baudrate(base, uart_clk * 1000000u, cfg->baudrate);

    /* Configure line control */
    uint32_t lcrh = LCRH_FEN;  /* Enable FIFO */

    switch (cfg->data_bits) {
        case 5: lcrh |= LCRH_WLEN_5; break;
        case 6: lcrh |= LCRH_WLEN_6; break;
        case 7: lcrh |= LCRH_WLEN_7; break;
        case 8: lcrh |= LCRH_WLEN_8; break;
        default: return E_INVAL;
    }

    /* Parity (not fully implemented - just basic support) */
    if (cfg->parity != 0) {
        lcrh |= (1u << 1);  /* Enable parity */
        if (cfg->parity == 2) {
            lcrh |= (1u << 2);  /* Even parity */
        }
    }

    /* Stop bits (2 stop bits requires setting bit 3) */
    if (cfg->stop_bits == 2) {
        lcrh |= (1u << 3);
    }

    REG32(base + UART_LCRH) = lcrh;

    /* Set FIFO interrupt threshold */
    REG32(base + UART_IFLS) = 0;  /* RX 1/8, TX 1/8 */

    /* Enable UART */
    REG32(base + UART_CR) = CR_UARTEN | CR_TXE | CR_RXE;

    return E_OK;
}

/**
 * Write data to UART (blocking)
 */
int hal_uart_write(uint32_t uart_id, const uint8_t *data, uint32_t len)
{
    uint32_t base = uart_get_base(uart_id);

    if (!base || !data) return E_INVAL;

    for (uint32_t i = 0; i < len; i++) {
        /* Wait for TX FIFO not full */
        while (REG32(base + UART_FR) & FR_TXFF);

        REG32(base + UART_DR) = data[i];
    }

    return (int)len;
}

/**
 * Read data from UART (blocking)
 */
int hal_uart_read(uint32_t uart_id, uint8_t *data, uint32_t len)
{
    uint32_t base = uart_get_base(uart_id);

    if (!base || !data) return E_INVAL;

    for (uint32_t i = 0; i < len; i++) {
        /* Wait for RX FIFO not empty */
        while (REG32(base + UART_FR) & FR_RXFE);

        data[i] = (uint8_t)REG32(base + UART_DR);
    }

    return (int)len;
}

/**
 * Check if data is available for reading
 */
int hal_uart_available(uint32_t uart_id)
{
    uint32_t base = uart_get_base(uart_id);

    if (!base) return E_INVAL;

    if (REG32(base + UART_FR) & FR_RXFE) {
        return 0;  /* No data available */
    }

    return 1;  /* Data available */
}

/**
 * Enable or disable UART interrupt
 */
int hal_uart_set_interrupt(uint32_t uart_id, uint8_t enable)
{
    uint32_t base = uart_get_base(uart_id);

    if (!base) return E_INVAL;

    if (enable) {
        REG32(base + UART_IMSC) |= INT_RX;
    } else {
        REG32(base + UART_IMSC) &= ~INT_RX;
    }

    return E_OK;
}
