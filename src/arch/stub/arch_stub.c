/**
 * @file arch_stub.c
 * @brief Architecture stub for host (test) builds
 *
 * Provides empty implementations of arch-specific symbols
 * so that unit tests can link without cross-compiler.
 */
#include "types.h"

/* Stub linker symbols */
uint32_t _data_load  = 0;
uint32_t _data_start = 0;
uint32_t _data_end   = 0;
uint32_t _bss_start  = 0;
uint32_t _bss_end    = 0;
uint32_t _stack_top  = 0;
