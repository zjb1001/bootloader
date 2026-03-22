/**
 * @file hal/uart.h
 * @brief UART HAL interface  [THREAD]
 */
#ifndef HAL_UART_H
#define HAL_UART_H

#include "types.h"

typedef struct {
    uint32_t baudrate;       /**< Baud rate */
    uint8_t  data_bits;      /**< Data bits (5-8) */
    uint8_t  stop_bits;      /**< Stop bits (1-2) */
    uint8_t  parity;         /**< 0=none, 1=odd, 2=even */
} uart_config_t;

int hal_uart_init(uint32_t uart_id, const uart_config_t *cfg);
int hal_uart_write(uint32_t uart_id, const uint8_t *data, uint32_t len);
int hal_uart_read(uint32_t uart_id, uint8_t *data, uint32_t len);
int hal_uart_available(uint32_t uart_id);
int hal_uart_set_interrupt(uint32_t uart_id, uint8_t enable);

#endif /* HAL_UART_H */
