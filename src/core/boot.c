/**
 * @file boot.c
 * @brief Boot control – mode management and kernel hand-off
 */
#include "core/boot.h"
#include "errno.h"

static boot_mode_t s_boot_mode = BOOT_MODE_NORMAL;

int core_boot_init(void)
{
    /* TODO: read boot mode from persistent flags / GPIO */
    return E_OK;
}

int core_boot_check(void)
{
    /* TODO: pre-boot sanity checks */
    return E_OK;
}

void core_boot_kernel(const boot_params_t *params)
{
    /* TODO: disable interrupts, flush caches, jump to entry */
    (void)params;
    for (;;) { /* should not reach */ }
}

void core_boot_recovery(void)
{
    /* TODO: enter recovery mode loop */
    for (;;) { }
}

int core_set_boot_mode(boot_mode_t mode)
{
    s_boot_mode = mode;
    return E_OK;
}

boot_mode_t core_get_boot_mode(void)
{
    return s_boot_mode;
}
