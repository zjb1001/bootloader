/**
 * @file flash.c
 * @brief Flash driver abstraction – dispatches to NOR/NAND/eMMC backends
 */
#include "driver/flash.h"
#include "errno.h"

int driver_flash_init(void)
{
    /* TODO: probe flash type, init corresponding backend */
    return E_OK;
}

int driver_flash_get_info(flash_info_t *info)
{
    (void)info;
    return E_OK;
}

int driver_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    /* TODO: dispatch to active backend */
    (void)addr; (void)buf; (void)len;
    return E_OK;
}

int driver_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    (void)addr; (void)buf; (void)len;
    return E_OK;
}

int driver_flash_erase(uint32_t addr, uint32_t len)
{
    (void)addr; (void)len;
    return E_OK;
}

int driver_otp_read(uint32_t offset, uint8_t *buf, uint32_t len)
{
    (void)offset; (void)buf; (void)len;
    return E_OK;
}

int driver_flash_is_bad_block(uint32_t block_addr)
{
    (void)block_addr;
    return 0; /* good block */
}
