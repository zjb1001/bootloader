/**
 * @file startup.c
 * @brief 5-stage boot sequence orchestrator
 *
 * Stage 1: Early init (assembly → C bridge)
 * Stage 2: HAL init  (clock, DRAM, IRQ, UART, WDT)
 * Stage 3: POST      (memory, flash, clock, CRC self-test)
 * Stage 4: Load+Verify (partition → kernel → CRC → SHA → sig → version)
 * Stage 5: Hand-off  (prepare params, cleanup, jump to kernel)
 */
#include "boot.h"
#include "config.h"
#include "errno.h"
#include "hal/clock.h"
#include "hal/uart.h"
#include "hal/irq.h"
#include "hal/memory.h"
#include "hal/timer.h"
#include "driver/watchdog.h"
#include "driver/flash.h"
#include "core/verify.h"
#include "core/loader.h"
#include "core/boot.h"
#include "core/partition.h"
#include "core/recovery.h"

/* ─── Stage 1 ─── */
int boot_stage1_early_init(void)
{
    /* TODO: any post-assembly early C init */
    return E_OK;
}

/* ─── Stage 2 ─── */
int boot_stage2_hw_init(void)
{
    /* TODO: hal_clock_init, hal_dram_init, hal_irq_init, hal_uart_init,
     *       driver_watchdog_init */
    return E_OK;
}

/* ─── Stage 3 ─── */
int boot_stage3_selftest(void)
{
    /* TODO: DRAM test, Flash probe, clock verify, CRC self-check */
    return E_OK;
}

/* ─── Stage 4 ─── */
int boot_stage4_load_verify(void)
{
    /* TODO: partition_read, load_image, verify_image pipeline */
    return E_OK;
}

/* ─── Stage 5 ─── */
int boot_stage5_start_kernel(void)
{
    /* TODO: prepare boot params, cleanup, core_boot_kernel */
    return E_OK;
}

/* ─── Master sequence ─── */
int boot_main(void)
{
    int rc;

    rc = boot_stage1_early_init();
    if (rc != E_OK) return rc;

    rc = boot_stage2_hw_init();
    if (rc != E_OK) return rc;

    rc = boot_stage3_selftest();
    if (rc != E_OK) return rc;

    rc = boot_stage4_load_verify();
    if (rc != E_OK) {
        /* Attempt recovery */
        core_handle_boot_failure((uint32_t)rc);
        return rc;
    }

    rc = boot_stage5_start_kernel();
    /* Should not reach here */
    return rc;
}
