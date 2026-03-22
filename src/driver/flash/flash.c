/**
 * @file flash.c
 * @brief Flash driver abstraction – dispatches to NOR/NAND/eMMC backends
 *
 * Main flash driver that detects flash type and routes to appropriate backend.
 */
#include "driver/flash.h"
#include "errno.h"
#include "types.h"

/* Active flash backend */
static flash_type_t s_flash_type = FLASH_TYPE_NOR_SPI;
static flash_info_t s_flash_info;

/* Backend function pointers (will be initialized based on detected type) */
extern int nor_flash_init(void);
extern int nor_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);
extern int nor_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len);
extern int nor_flash_erase(uint32_t addr, uint32_t len);

/**
 * Detect flash type by probing
 */
static flash_type_t detect_flash_type(void)
{
    /* For now, assume NOR SPI flash */
    /* In real implementation, would probe various interfaces */
    return FLASH_TYPE_NOR_SPI;
}

/**
 * Initialize flash driver
 */
int driver_flash_init(void)
{
    s_flash_type = detect_flash_type();

    switch (s_flash_type) {
        case FLASH_TYPE_NOR_SPI:
            if (nor_flash_init() < 0) {
                return BOOT_ERR_FLASH_INIT;
            }
            /* Set default NOR flash parameters */
            s_flash_info.type = FLASH_TYPE_NOR_SPI;
            s_flash_info.total_size = 16 * 1024 * 1024;  /* 16 MB */
            s_flash_info.sector_size = 4 * 1024;  /* 4 KB sectors */
            s_flash_info.page_size = 256;
            s_flash_info.block_size = 64 * 1024;  /* 64 KB blocks */
            break;

        case FLASH_TYPE_NAND_SPI:
        case FLASH_TYPE_NAND_PAR:
            /* Not implemented yet */
            return BOOT_ERR_FLASH_INIT;

        case FLASH_TYPE_EMMC:
            /* Not implemented yet */
            return BOOT_ERR_FLASH_INIT;

        default:
            return BOOT_ERR_FLASH_INIT;
    }

    return E_OK;
}

/**
 * Get flash information
 */
int driver_flash_get_info(flash_info_t *info)
{
    if (!info) return E_INVAL;

    *info = s_flash_info;
    return E_OK;
}

/**
 * Read from flash
 */
int driver_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (!buf) return E_INVAL;

    switch (s_flash_type) {
        case FLASH_TYPE_NOR_SPI:
            return nor_flash_read(addr, buf, len);
        default:
            return E_IO;
    }
}

/**
 * Write to flash
 */
int driver_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (!buf) return E_INVAL;

    switch (s_flash_type) {
        case FLASH_TYPE_NOR_SPI:
            return nor_flash_write(addr, buf, len);
        default:
            return E_IO;
    }
}

/**
 * Erase flash sectors
 */
int driver_flash_erase(uint32_t addr, uint32_t len)
{
    switch (s_flash_type) {
        case FLASH_TYPE_NOR_SPI:
            return nor_flash_erase(addr, len);
        default:
            return E_IO;
    }
}

/**
 * Read from OTP (One-Time Programmable) area
 */
int driver_otp_read(uint32_t offset, uint8_t *buf, uint32_t len)
{
    /* OTP reading is platform-specific */
    /* For now, return zeros */
    if (!buf || len == 0) return E_INVAL;

    for (uint32_t i = 0; i < len; i++) {
        buf[i] = 0;
    }

    return len;
}

/**
 * Check if NAND block is bad (only for NAND flash)
 */
int driver_flash_is_bad_block(uint32_t block_addr)
{
    if (s_flash_type == FLASH_TYPE_NOR_SPI || s_flash_type == FLASH_TYPE_EMMC) {
        return 0;  /* NOR and eMMC don't have bad blocks */
    }

    /* NAND bad block detection not implemented */
    return 0;
}
