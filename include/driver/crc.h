/**
 * @file driver/crc.h
 * @brief CRC32/CRC64 computation  [ISR-safe]
 */
#ifndef DRIVER_CRC_H
#define DRIVER_CRC_H

#include "types.h"

/**
 * Compute CRC32 (incremental).
 * @param data     Input data.
 * @param len      Length in bytes.
 * @param prev_crc Previous CRC value (0 for initial).
 * @return Computed CRC32 value.
 */
uint32_t driver_crc32(const uint8_t *data, uint32_t len, uint32_t prev_crc);

/**
 * Compute CRC64 (incremental).
 */
uint64_t driver_crc64(const uint8_t *data, uint32_t len, uint64_t prev_crc);

#endif /* DRIVER_CRC_H */
