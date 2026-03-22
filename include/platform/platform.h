/**
 * @file platform/platform.h
 * @brief Platform detection and adaptation
 */
#ifndef PLATFORM_PLATFORM_H
#define PLATFORM_PLATFORM_H

#include "types.h"

typedef enum {
    PLATFORM_ARMV7 = 0,
    PLATFORM_ARMV8,
    PLATFORM_X86_64,
} platform_type_t;

platform_type_t platform_detect(void);
int  platform_init(void);
int  platform_get_info(char *info, uint32_t len);
int  platform_cleanup(void);

#endif /* PLATFORM_PLATFORM_H */
