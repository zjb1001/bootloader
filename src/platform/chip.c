/**
 * @file chip.c
 * @brief Chip identification helpers
 */
#include "platform/chip.h"
#include "errno.h"
#include "types.h"

/* Chip ID registers */
#define CHIP_ID_BASE    0x01C20000u
#define CHIP_ID_REG     (CHIP_ID_BASE + 0x0000)
#define CHIP_CFG_REG    (CHIP_ID_BASE + 0x0004)

#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))

/* Cached chip information */
static chip_info_t s_chip_info = {0};
static int s_chip_info_valid = 0;

int chip_get_info(chip_info_t *info)
{
    if (!info) return E_INVAL;

    if (!s_chip_info_valid) {
        /* Read chip ID registers */
        uint32_t chip_id = REG32(CHIP_ID_REG);
        uint32_t chip_cfg = REG32(CHIP_CFG_REG);

        /* Decode chip ID (platform specific) */
        s_chip_info.chip_id = chip_id;
        s_chip_info.chip_revision = (chip_cfg >> 16) & 0xFF;

        /* Set chip name based on ID */
        switch ((chip_id >> 8) & 0xFF) {
            case 0x30:
                s_chip_info.chip_name[0] = 'A';
                s_chip_info.chip_name[1] = '1';
                s_chip_info.chip_name[2] = '0';
                s_chip_info.chip_name[3] = '0';
                break;
            case 0x31:
                s_chip_info.chip_name[0] = 'A';
                s_chip_info.chip_name[1] = '2';
                s_chip_info.chip_name[2] = '0';
                s_chip_info.chip_name[3] = '0';
                break;
            default:
                s_chip_info.chip_name[0] = 'U';
                s_chip_info.chip_name[1] = 'N';
                s_chip_info.chip_name[2] = 'K';
                s_chip_info.chip_name[3] = 'N';
                break;
        }
        s_chip_info.chip_name[4] = '\0';

        /* Production date (placeholder) */
        s_chip_info.production_date[0] = '2';
        s_chip_info.production_date[1] = '0';
        s_chip_info.production_date[2] = '2';
        s_chip_info.production_date[3] = '6';
        s_chip_info.production_date[4] = '\0';

        s_chip_info_valid = 1;
    }

    *info = s_chip_info;
    return E_OK;
}

int chip_get_unique_id(uint8_t uid[16])
{
    if (!uid) return E_INVAL;

    /* Read unique ID from OTP/efuse */
    /* This is platform-specific - use placeholder values */
    uint32_t uid_base = 0x01C23800u;

    for (int i = 0; i < 4; i++) {
        uint32_t val = REG32(uid_base + i * 4);
        uid[i * 4 + 0] = (val >> 24) & 0xFF;
        uid[i * 4 + 1] = (val >> 16) & 0xFF;
        uid[i * 4 + 2] = (val >> 8) & 0xFF;
        uid[i * 4 + 3] = val & 0xFF;
    }

    return E_OK;
}

int chip_read_register(uint32_t offset, uint32_t *value)
{
    if (!value) return E_INVAL;

    *value = REG32(CHIP_ID_BASE + offset);
    return E_OK;
}

int chip_write_register(uint32_t offset, uint32_t value)
{
    REG32(CHIP_ID_BASE + offset) = value;
    return E_OK;
}
