/**
 * @file chip.c
 * @brief Chip identification helpers
 */
#include "platform/chip.h"
#include "errno.h"

int chip_get_info(chip_info_t *info)
{
    /* TODO: read chip ID/revision registers */
    (void)info;
    return E_OK;
}

int chip_get_unique_id(uint8_t uid[16])
{
    /* TODO: read UID from OTP/efuse */
    (void)uid;
    return E_OK;
}

int chip_read_register(uint32_t offset, uint32_t *value)
{
    (void)offset; (void)value;
    return E_OK;
}

int chip_write_register(uint32_t offset, uint32_t value)
{
    (void)offset; (void)value;
    return E_OK;
}
