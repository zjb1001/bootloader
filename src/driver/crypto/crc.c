/**
 * @file crc.c
 * @brief CRC32 / CRC64 computation
 */
#include "driver/crc.h"

uint32_t driver_crc32(const uint8_t *data, uint32_t len, uint32_t prev_crc)
{
    /* TODO: implement CRC32 (polynomial 0xEDB88320) with lookup table */
    (void)data; (void)len; (void)prev_crc;
    return 0;
}

uint64_t driver_crc64(const uint8_t *data, uint32_t len, uint64_t prev_crc)
{
    /* TODO: implement CRC64 */
    (void)data; (void)len; (void)prev_crc;
    return 0;
}
