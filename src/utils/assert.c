/**
 * @file assert.c
 * @brief Runtime assertion handler
 */
#include "types.h"

void bl_assert_fail(const char *expr, const char *file, int line)
{
    /* TODO: log assertion, trigger watchdog reset */
    (void)expr; (void)file; (void)line;
    for (;;) { }
}
