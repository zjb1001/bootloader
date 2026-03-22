/**
 * @file driver/flash.h
 * @brief Flash storage driver interface  [THREAD]
 */
#ifndef DRIVER_FLASH_H
#define DRIVER_FLASH_H

#include "types.h"

typedef enum {
    FLASH_TYPE_NOR_SPI = 0,
    FLASH_TYPE_NAND_SPI,
    FLASH_TYPE_NAND_PAR,
    FLASH_TYPE_EMMC,
} flash_type_t;

typedef struct {
    uint32_t     total_size;     /**< Total capacity in bytes */
    uint32_t     sector_size;    /**< Erase sector size */
    uint32_t     page_size;      /**< Page size (NAND) */
    uint32_t     block_size;     /**< Block size (NAND) */
    flash_type_t type;
} flash_info_t;

int driver_flash_init(void);
int driver_flash_get_info(flash_info_t *info);
int driver_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);
int driver_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len);
int driver_flash_erase(uint32_t addr, uint32_t len);
int driver_otp_read(uint32_t offset, uint8_t *buf, uint32_t len);
int driver_flash_is_bad_block(uint32_t block_addr);

#endif /* DRIVER_FLASH_H */
