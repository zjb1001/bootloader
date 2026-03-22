/**
 * @file uart.c
 * @brief UART HAL implementation
 */
#include "hal/uart.h"
#include "errno.h"

int hal_uart_init(uint32_t uart_id, const uart_config_t *cfg)
{
    /* TODO: configure baud rate generator, pin mux, FIFO */
    (void)uart_id; (void)cfg;
    return E_OK;
}

int hal_uart_write(uint32_t uart_id, const uint8_t *data, uint32_t len)
{
    /* TODO: poll TX-ready, write bytes */
    (void)uart_id; (void)data; (void)len;
    return (int)len;
}

int hal_uart_read(uint32_t uart_id, uint8_t *data, uint32_t len)
{
    /* TODO: poll RX-ready, read bytes */
    (void)uart_id; (void)data; (void)len;
    return 0;
}

int hal_uart_available(uint32_t uart_id)
{
    (void)uart_id;
    return 0;
}

int hal_uart_set_interrupt(uint32_t uart_id, uint8_t enable)
{
    (void)uart_id; (void)enable;
    return E_OK;
}
