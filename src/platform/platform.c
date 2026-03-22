/**
 * @file platform.c
 * @brief Platform detection and init
 */
#include "platform/platform.h"
#include "errno.h"

platform_type_t platform_detect(void)
{
    /* TODO: read chip ID register to determine platform */
#if defined(__ARM_ARCH_7A__)
    return PLATFORM_ARMV7;
#elif defined(__aarch64__)
    return PLATFORM_ARMV8;
#elif defined(__x86_64__)
    return PLATFORM_X86_64;
#else
    return PLATFORM_ARMV7;
#endif
}

int platform_init(void)
{
    /* TODO: board-specific pin mux, voltage regulator, etc. */
    return E_OK;
}

int platform_get_info(char *info, uint32_t len)
{
    (void)info; (void)len;
    return E_OK;
}

int platform_cleanup(void)
{
    /* TODO: undo platform-specific settings before kernel handoff */
    return E_OK;
}
