/**
 * @file debug.c
 * @brief Debug output helpers (conditional on build config)
 */
#include "hal/uart.h"
#include "config.h"
#include "errno.h"

/* Debug output is routed through DEFAULT_UART_ID */
