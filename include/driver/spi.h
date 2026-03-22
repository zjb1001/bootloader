/**
 * @file driver/spi.h
 * @brief SPI bus driver interface  [THREAD]
 */
#ifndef DRIVER_SPI_H
#define DRIVER_SPI_H

#include "types.h"

typedef struct {
    uint32_t speed_hz;       /**< Clock speed */
    uint8_t  mode;           /**< SPI mode (0-3) */
    uint8_t  bits_per_word;  /**< Bits per transfer word */
} spi_config_t;

int driver_spi_init(uint32_t spi_id, const spi_config_t *cfg);
int driver_spi_transfer(uint32_t spi_id,
                        const uint8_t *tx, uint8_t *rx, uint32_t len);
int driver_spi_cs_select(uint32_t spi_id, uint8_t cs);
int driver_spi_cs_deselect(uint32_t spi_id, uint8_t cs);

#endif /* DRIVER_SPI_H */
