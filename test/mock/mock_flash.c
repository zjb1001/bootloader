/**
 * @file mock_flash.c
 * @brief Mock Flash backend for testing
 *
 * Provides an in-memory flash simulation that tests can
 * pre-populate with partition tables, images, etc.
 */
#include "types.h"
#include "errno.h"

#define MOCK_FLASH_SIZE (4u * 1024u * 1024u) /* 4 MB */

static uint8_t mock_flash_mem[MOCK_FLASH_SIZE];

void mock_flash_clear(void)
{
    for (uint32_t i = 0; i < MOCK_FLASH_SIZE; i++) {
        mock_flash_mem[i] = 0xFF;
    }
}

int mock_flash_write_raw(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if (addr + len > MOCK_FLASH_SIZE) return E_IO;
    for (uint32_t i = 0; i < len; i++) {
        mock_flash_mem[addr + i] = data[i];
    }
    return (int)len;
}

int mock_flash_read_raw(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (addr + len > MOCK_FLASH_SIZE) return E_IO;
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = mock_flash_mem[addr + i];
    }
    return (int)len;
}
