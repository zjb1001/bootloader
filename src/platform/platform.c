/**
 * @file platform.c
 * @brief Platform detection and init
 */
#include "platform/platform.h"
#include "platform/chip.h"
#include "hal/gpio.h"
#include "errno.h"
#include "types.h"

/* Chip ID register base (ARMv7-A CP15) */
#define CHIP_ID_BASE    0x01C20000u

#if defined(__arm__)
#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#else
#define REG32(addr) (0)  /* Stub for host builds */
#endif

platform_type_t platform_detect(void)
{
    /* Read CPU ID to determine platform */
#if defined(__arm__)
    uint32_t cpuid;
    __asm__ volatile("mrc p15, 0, %0, c0, c0, 0" : "=r"(cpuid));

    /* Extract implementer and architecture */
    uint32_t implementer = (cpuid >> 24) & 0xFF;
    uint32_t architecture = (cpuid >> 16) & 0xF;

    if (implementer == 0x41) {  /* ARM */
        if (architecture >= 7) {
            return PLATFORM_ARMV7;
        }
    }
    return PLATFORM_ARMV7;
#else
    /* For host builds (x86_64), return stub */
    return PLATFORM_ARMV7;
#endif
}

int platform_init(void)
{
    platform_type_t type = platform_detect();

    /* Platform-specific initialization */
    switch (type) {
        case PLATFORM_ARMV7:
            /* Configure GPIO for basic functionality */
            /* UART pins are already muxed in hal_uart_init */
            break;

        case PLATFORM_ARMV8:
            /* ARMv8 specific init */
            break;

        case PLATFORM_X86_64:
            /* x86 specific init */
            break;
    }

    return E_OK;
}

int platform_get_info(char *info, uint32_t len)
{
    if (!info || len == 0) return E_INVAL;

    chip_info_t chip_info;
    int ret = chip_get_info(&chip_info);
    if (ret < 0) return ret;

    /* Format platform info string */
    const char *arch_str = "Unknown";
    platform_type_t type = platform_detect();

    switch (type) {
        case PLATFORM_ARMV7: arch_str = "ARMv7-A"; break;
        case PLATFORM_ARMV8: arch_str = "ARMv8-A"; break;
        case PLATFORM_X86_64: arch_str = "x86_64"; break;
    }

    /* Simple format - in real implementation would use snprintf */
    int i = 0;
    const char *p = arch_str;
    while (*p && i < (int)len - 1) {
        info[i++] = *p++;
    }
    info[i] = '\0';

    return E_OK;
}

int platform_cleanup(void)
{
    /* Undo platform-specific settings before kernel handoff */
    /* Most settings are left as-is for the kernel */

    return E_OK;
}
