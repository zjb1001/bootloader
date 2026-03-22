/**
 * @file driver/console.h
 * @brief Console driver interface  [THREAD]
 */
#ifndef DRIVER_CONSOLE_H
#define DRIVER_CONSOLE_H

#include "types.h"

int console_init(uint32_t baudrate);
int console_putchar(int c);
int console_getchar(void);
int console_getchar_wait(void);
int console_puts(const char *s);
int console_gets(char *buf, uint32_t max_len);

#endif /* DRIVER_CONSOLE_H */
