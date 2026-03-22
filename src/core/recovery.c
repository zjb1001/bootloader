/**
 * @file recovery.c
 * @brief Boot failure detection and recovery actions
 */
#include "core/recovery.h"
#include "driver/console.h"
#include "driver/watchdog.h"
#include "utils/string.h"
#include "core/partition.h"
#include "core/loader.h"
#include "core/verify.h"
#include "core/boot.h"
#include "errno.h"

#include <stdarg.h>

recovery_action_t core_handle_boot_failure(uint32_t reason)
{
    const char *reason_str = "unknown";

    switch (reason) {
        case BOOT_ERR_CLOCK_INIT:
            reason_str = "clock_init";
            break;
        case BOOT_ERR_DRAM_INIT:
        case BOOT_ERR_DRAM_TEST:
            reason_str = "dram";
            break;
        case BOOT_ERR_FLASH_INIT:
            reason_str = "flash_init";
            break;
        case BOOT_ERR_PARTITION_TABLE:
            reason_str = "partition_table";
            break;
        case BOOT_ERR_IMAGE_LOAD:
            reason_str = "image_load";
            break;
        case BOOT_ERR_IMAGE_CORRUPT:
            reason_str = "image_corrupt";
            break;
        case BOOT_ERR_CRC_MISMATCH:
        case BOOT_ERR_HASH_MISMATCH:
        case BOOT_ERR_SIGNATURE_INVALID:
            reason_str = "verify_failed";
            break;
        case BOOT_ERR_VERSION_ROLLBACK:
            reason_str = "rollback";
            break;
        default:
            reason_str = "unknown";
            break;
    }

    console_puts("\n*** BOOT FAILURE ***\n");
    console_puts("Reason: ");
    console_puts(reason_str);
    console_puts("\n");

    /* Log recovery event */
    core_recovery_log("Boot failure: %s", reason_str);

    /* Decide recovery action */
    switch (reason) {
        case BOOT_ERR_SIGNATURE_INVALID:
        case BOOT_ERR_VERSION_ROLLBACK:
        case BOOT_ERR_IMAGE_CORRUPT:
            /* Try fallback first */
            return RECOVERY_FALLBACK;

        case BOOT_ERR_PARTITION_TABLE:
        case BOOT_ERR_FLASH_INIT:
            /* Serious problem - go to recovery mode */
            return RECOVERY_BOOTLOADER_MODE;

        case BOOT_ERR_CLOCK_INIT:
        case BOOT_ERR_DRAM_INIT:
        case BOOT_ERR_DRAM_TEST:
            /* Hardware problem - halt */
            return RECOVERY_HALT;

        default:
            return RECOVERY_FALLBACK;
    }
}

int core_recovery_fallback_boot(void)
{
    int ret;

    console_puts("\n=== Trying fallback partition ===\n");

    /* Try backup kernel */
    partition_entry_t *kernel_part = core_partition_find("kernel_b");
    if (!kernel_part) {
        console_puts("No backup partition found\n");
        return E_NOTFOUND;
    }

    /* Load backup image */
    image_header_t img_header;
    ret = core_load_image(kernel_part->offset, &img_header);
    if (ret < 0) {
        console_puts("Failed to load backup image\n");
        return ret;
    }

    /* Verify backup */
    image_t backup_img = {
        .addr = img_header.load_addr,
        .size = img_header.size,
        .methods = VERIFY_ALL
    };

    ret = core_verify_image(&backup_img);
    if (ret < 0) {
        console_puts("Backup verification failed\n");
        return ret;
    }

    console_puts("Backup image loaded successfully\n");
    return E_OK;
}

void core_recovery_bootloader_mode(void)
{
    console_puts("\n=== Bootloader Recovery Mode ===\n");
    console_puts("Commands: boot, help, reset\n");

    char cmd[64];

    while (1) {
        console_puts("> ");
        console_gets(cmd, sizeof(cmd));

        if (bl_strcmp(cmd, "boot") == 0) {
            console_puts("Attempting boot...\n");
            break;
        } else if (bl_strcmp(cmd, "help") == 0) {
            console_puts("Available commands:\n");
            console_puts("  boot  - Attempt boot from primary partition\n");
            console_puts("  reset - Reset the system\n");
            console_puts("  help  - Show this help\n");
        } else if (bl_strcmp(cmd, "reset") == 0) {
            console_puts("Resetting...\n");
            driver_watchdog_force_reset();
        } else {
            console_puts("Unknown command\n");
        }

        driver_watchdog_feed();
    }
}

void core_recovery_log(const char *fmt, ...)
{
    va_list ap;
    char buf[256];

    va_start(ap, fmt);

    /* Format message */
    int len = 0;
    const char *p = fmt;
    while (*p && len < (int)sizeof(buf) - 1) {
        buf[len++] = *p++;
    }
    va_end(ap);

    /* Log to console */
    console_puts("[RECOVERY] ");
    console_puts(buf);
    console_puts("\n");
}
