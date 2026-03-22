/**
 * @file crt0.c
 * @brief Minimal C runtime initialization
 *
 * Called from assembly _start after stack/BSS are ready.
 * Initialises static data, then calls main().
 */
#include "types.h"

extern int main(void);

/* Linker-provided symbols */
extern uint32_t _data_load;
extern uint32_t _data_start;
extern uint32_t _data_end;

void _crt0_init(void)
{
    /* Copy .data from Flash (LMA) to RAM (VMA) */
    uint32_t *src = &_data_load;
    uint32_t *dst = &_data_start;
    while (dst < &_data_end) {
        *dst++ = *src++;
    }

    /* BSS already zeroed by assembly startup */

    main();

    /* If main returns, halt */
    for (;;) {
        /* TODO: low-power wait */
    }
}
