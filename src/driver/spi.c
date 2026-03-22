/**
 * @file spi.c
 * @brief SPI bus driver
 */
#include "driver/spi.h"
#include "errno.h"

int driver_spi_init(uint32_t spi_id, const spi_config_t *cfg)
{
    (void)spi_id; (void)cfg;
    return E_OK;
}

int driver_spi_transfer(uint32_t spi_id,
                        const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    (void)spi_id; (void)tx; (void)rx; (void)len;
    return E_OK;
}

int driver_spi_cs_select(uint32_t spi_id, uint8_t cs)
{
    (void)spi_id; (void)cs;
    return E_OK;
}

int driver_spi_cs_deselect(uint32_t spi_id, uint8_t cs)
{
    (void)spi_id; (void)cs;
    return E_OK;
}
