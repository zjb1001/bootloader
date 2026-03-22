/**
 * @file boot.c
 * @brief Boot control – mode management and kernel hand-off
 */
#include "core/boot.h"
#include "hal/uart.h"
#include "hal/irq.h"
#include "hal/memory.h"
#include "hal/gpio.h"
#include "config.h"
#include "errno.h"
#include "types.h"

static boot_mode_t s_boot_mode = BOOT_MODE_NORMAL;

/* Check for recovery mode GPIO */
static int check_recovery_gpio(void)
{
    /* Check GPIO for recovery request */
    /* For now, return 0 (no recovery) */
    return 0;
}

int core_boot_init(void)
{
    /* Check boot mode sources */
    if (check_recovery_gpio()) {
        s_boot_mode = BOOT_MODE_RECOVERY;
    }

    return E_OK;
}

int core_boot_check(void)
{
    /* Pre-boot sanity checks */

    /* Verify kernel entry point is in valid range */
    extern uint32_t s_kernel_entry;
    if (s_kernel_entry < DRAM_BASE || s_kernel_entry > DRAM_BASE + DRAM_SIZE) {
        return E_INVAL;
    }

    /* Verify DTB address is valid */
    /* (Implementation specific) */

    return E_OK;
}

void core_boot_kernel(const boot_params_t *params)
{
    if (!params) {
        /* Halt if invalid params */
        for (;;) {
#if defined(__arm__) || defined(__aarch64__)
            __asm__ volatile("wfi");
#endif
        }
    }

    /* Flush all caches */
    hal_cache_flush(0);
    hal_cache_flush(1);

    /* Disable MMU */
    hal_mmu_disable();

    /* Disable interrupts */
    hal_irq_disable_all();

    /* Disable instruction and data cache */
    hal_cache_disable(0);

    /* Setup kernel arguments */
    uint32_t r0 = 0;              /* Normally machine type */
    uint32_t r1 = params->dtb_addr;  /* DTB address */
    uint32_t r2 = params->bootargs ? (uintptr_t)params->bootargs : 0;  /* ATAGS/DTB */

    /* Jump to kernel */
    uint32_t kernel_entry = params->kernel_addr;

#if defined(__arm__)
    __asm__ volatile (
        "mov r0, %0\n"
        "mov r1, %1\n"
        "mov r2, %2\n"
        "bx %3\n"
        :
        : "r"(r0), "r"(r1), "r"(r2), "r"(kernel_entry)
        : "r0", "r1", "r2"
    );
#else
    /* Host build stub - cannot execute ARM assembly */
    (void)r0; (void)r1; (void)r2; (void)kernel_entry;
#endif

    /* Should never return */
    for (;;) {
#if defined(__arm__) || defined(__aarch64__)
        __asm__ volatile("wfi");
#endif
    }
}

void core_boot_recovery(void)
{
    /* Enter recovery mode */
    extern void core_recovery_bootloader_mode(void);
    core_recovery_bootloader_mode();
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
