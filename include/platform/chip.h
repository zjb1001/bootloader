/**
 * @file platform/chip.h
 * @brief Chip-level identification interface
 */
#ifndef PLATFORM_CHIP_H
#define PLATFORM_CHIP_H

#include "types.h"

typedef struct {
    char     chip_name[32];
    uint32_t chip_id;
    uint32_t chip_revision;
    char     production_date[16];
} chip_info_t;

int chip_get_info(chip_info_t *info);
int chip_get_unique_id(uint8_t uid[16]);
int chip_read_register(uint32_t offset, uint32_t *value);
int chip_write_register(uint32_t offset, uint32_t value);

#endif /* PLATFORM_CHIP_H */
