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
#include "driver/console.h"
#include "core/verify.h"
#include "core/loader.h"
#include "core/boot.h"
#include "core/partition.h"
#include "core/recovery.h"

/* Boot state tracking */
static int s_boot_stage = 0;
static uint32_t s_boot_flags = 0;

/* ─── Stage 1: Early C init ─── */
int boot_stage1_early_init(void)
{
    s_boot_stage = 1;

    /* Minimal early initialization */
    /* At this point, only SRAM is available */

    return E_OK;
}

/* ─── Stage 2: Hardware init ─── */
int boot_stage2_hw_init(void)
{
    int ret;
    s_boot_stage = 2;

    /* Configure system clocks */
    clock_config_t clk_cfg = {
        .core_freq = 1200,
        .ahb_freq = 400,
        .apb_freq = 200,
        .dram_freq = 400
    };
    ret = hal_clock_init(&clk_cfg);
    if (ret < 0) return BOOT_ERR_CLOCK_INIT;

    /* Initialize timer */
    ret = hal_timer_init();
    if (ret < 0) return ret;

    /* Initialize UART for console */
    uart_config_t uart_cfg = {
        .baudrate = DEFAULT_UART_BAUDRATE,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = 0
    };
    ret = hal_uart_init(DEFAULT_UART_ID, &uart_cfg);
    if (ret < 0) return ret;

    /* Initialize console */
    console_init(DEFAULT_UART_BAUDRATE);
    console_puts("\n=== Bootloader Stage 2 ===\n");

    /* Initialize interrupt controller */
    ret = hal_irq_init();
    if (ret < 0) return ret;

    /* Initialize MMU (identity mapping) */
    ret = hal_mmu_init();
    if (ret < 0) return ret;

    ret = hal_mmu_enable();
    if (ret < 0) return ret;

    /* Enable caches */
    hal_cache_enable(0);  /* L1 */
    hal_cache_enable(1);  /* L2 if present */

    /* Initialize DRAM */
    ret = hal_dram_init(DRAM_SIZE);
    if (ret < 0) return BOOT_ERR_DRAM_INIT;

    /* Initialize watchdog */
    ret = driver_watchdog_init(WATCHDOG_TIMEOUT_MS);
    if (ret < 0) return ret;

    /* Initialize Flash driver */
    ret = driver_flash_init();
    if (ret < 0) return BOOT_ERR_FLASH_INIT;

    /* Feed watchdog after stage 2 */
    driver_watchdog_feed();

    console_puts("Stage 2 complete\n");

    return E_OK;
}

/* ─── Stage 3: Power-On Self Test ─── */
int boot_stage3_selftest(void)
{
    int ret;
    s_boot_stage = 3;

    console_puts("=== Stage 3: POST ===\n");

    /* DRAM test (test first and last 1MB) */
    console_puts("Testing DRAM...\n");
    ret = hal_dram_test(DRAM_BASE, 1024 * 1024);
    if (ret < 0) return BOOT_ERR_DRAM_TEST;

    ret = hal_dram_test(DRAM_BASE + DRAM_SIZE - 1024 * 1024, 1024 * 1024);
    if (ret < 0) return BOOT_ERR_DRAM_TEST;

    console_puts("DRAM test passed\n");

    /* Verify Flash is accessible */
    flash_info_t flash_info;
    ret = driver_flash_get_info(&flash_info);
    if (ret < 0) return ret;

    /* Clock verification */
    int core_clk = hal_clock_get_freq(CLOCK_PLL_CORE);
    if (core_clk < MIN_DRAM_SPEED_MHZ) {
        console_puts("Warning: Core clock below minimum\n");
    }

    /* CRC self-test */
    uint8_t test_data[] = "123456789";
    uint32_t crc = driver_crc32(test_data, sizeof(test_data) - 1, 0);
    if (crc != 0xCBF43926) {
        return BOOT_ERR_CRC_MISMATCH;
    }

    /* Feed watchdog */
    driver_watchdog_feed();

    console_puts("Stage 3 complete\n");

    return E_OK;
}

/* ─── Stage 4: Load and Verify ─── */
int boot_stage4_load_verify(void)
{
    int ret;
    s_boot_stage = 4;

    console_puts("=== Stage 4: Load & Verify ===\n");

    /* Read partition table */
    partition_table_t ptable;
    ret = core_partition_read(&ptable);
    if (ret < 0) return BOOT_ERR_PARTITION_TABLE;

    /* Find kernel partition */
    partition_entry_t *kernel_part = core_partition_find("kernel");
    if (!kernel_part) {
        /* Try backup */
        kernel_part = core_partition_find("kernel_b");
        if (!kernel_part) {
            return BOOT_ERR_NO_VALID_BOOT;
        }
    }

    console_puts("Loading kernel from partition\n");

    /* Load kernel image */
    image_header_t img_header;
    uint32_t kernel_flash_addr = core_partition_get_addr("kernel");
    ret = core_load_image(kernel_flash_addr, &img_header);
    if (ret < 0) return BOOT_ERR_IMAGE_LOAD;

    /* Verify kernel */
    image_t kernel_img = {
        .addr = img_header.load_addr,
        .size = img_header.size,
        .methods = VERIFY_ALL
    };

    ret = core_verify_image(&kernel_img);
    if (ret < 0) return BOOT_ERR_SIGNATURE_INVALID;

    /* Load DTB if present */
    uint32_t dtb_addr = core_partition_get_addr("dtb");
    if (dtb_addr) {
        console_puts("Loading DTB\n");
        ret = core_load_dtb(dtb_addr, 0x81000000);
        if (ret < 0) return BOOT_ERR_DTB_INVALID;
    }

    /* Set default bootargs */
    core_set_bootargs("console", "ttyS0,115200");
    core_set_bootargs("root", "/dev/mmcblk0p2");

    /* Feed watchdog */
    driver_watchdog_feed();

    console_puts("Stage 4 complete\n");

    return E_OK;
}

/* ─── Stage 5: Start Kernel ─── */
int boot_stage5_start_kernel(void)
{
    s_boot_stage = 5;

    console_puts("=== Stage 5: Boot Kernel ===\n");

    /* Prepare boot parameters */
    boot_params_t params = {
        .kernel_addr = 0x80008000,
        .dtb_addr = 0x81000000,
        .ramdisk_addr = 0,
        .bootargs = core_get_bootargs(),
        .mode = BOOT_MODE_NORMAL
    };

    /* Pre-boot checks */
    int ret = core_boot_check();
    if (ret < 0) {
        console_puts("Boot check failed\n");
        return ret;
    }

    /* Disable watchdog before jumping to kernel */
    driver_watchdog_disable();

    /* Disable interrupts */
    uint32_t irq_state = hal_irq_disable_all();

    /* Flush caches */
    hal_cache_flush(0);
    hal_cache_flush(1);

    /* Disable MMU */
    hal_mmu_disable();

    console_puts("Jumping to kernel\n");

    /* Jump to kernel - does not return */
    core_boot_kernel(&params);

    /* Should never reach here */
    return E_OK;
}

/* ─── Master sequence ─── */
int boot_main(void)
{
    int rc;

    /* Stage 1: Early init */
    rc = boot_stage1_early_init();
    if (rc != E_OK) return rc;

    /* Stage 2: Hardware init */
    rc = boot_stage2_hw_init();
    if (rc != E_OK) {
        recovery_action_t action = core_handle_boot_failure((uint32_t)rc);
        if (action == RECOVERY_HALT) {
            console_puts("Halted in Stage 2\n");
            for (;;);
        }
        return rc;
    }

    /* Stage 3: POST */
    rc = boot_stage3_selftest();
    if (rc != E_OK) {
        recovery_action_t action = core_handle_boot_failure((uint32_t)rc);
        if (action == RECOVERY_HALT) {
            console_puts("Halted in Stage 3\n");
            for (;;);
        }
        return rc;
    }

    /* Stage 4: Load and verify */
    rc = boot_stage4_load_verify();
    if (rc != E_OK) {
        /* Attempt recovery */
        recovery_action_t action = core_handle_boot_failure((uint32_t)rc);
        if (action == RECOVERY_FALLBACK) {
            rc = core_recovery_fallback_boot();
            if (rc < 0) {
                core_recovery_bootloader_mode();
            }
        } else if (action == RECOVERY_BOOTLOADER_MODE) {
            core_recovery_bootloader_mode();
        }
        return rc;
    }

    /* Stage 5: Start kernel */
    rc = boot_stage5_start_kernel();

    /* Should not reach here */
    console_puts("ERROR: Returned from kernel jump\n");
    for (;;) {
        __asm__ volatile("wfi");
    }

    return rc;
}

/* Get boot flags */
uint32_t boot_get_flags(void)
{
    return s_boot_flags;
}

/* Set boot flags */
void boot_set_flags(uint32_t flags)
{
    s_boot_flags = flags;
}
