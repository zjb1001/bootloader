/**
 * @file nor_spi.c
 * @brief NOR Flash over SPI/QSPI backend
 *
 * Implements standard SPI NOR flash commands (JEDEC compatible).
 */
#include "driver/flash.h"
#include "driver/spi.h"
#include "errno.h"
#include "types.h"
#include "hal/timer.h"

/* SPI bus to use for NOR flash */
#define NOR_SPI_ID      0
#define NOR_SPI_CS      0

/* Standard SPI NOR flash commands */
#define CMD_READ_JEDEC_ID    0x9F
#define CMD_READ_DATA        0x03
#define CMD_FAST_READ        0x0B
#define CMD_READ_STATUS      0x05
#define CMD_WRITE_ENABLE     0x06
#define CMD_PAGE_PROGRAM     0x02
#define CMD_SECTOR_ERASE     0x20
#define CMD_BLOCK_ERASE_32K  0x52
#define CMD_BLOCK_ERASE_64K  0xD8
#define CMD_CHIP_ERASE       0xC7
#define CMD_WRITE_DISABLE    0x04

/* Status register bits */
#define SR_BUSY            (1u << 0)
#define SR_WEL             (1u << 1)
#define SR_BP0             (1u << 2)
#define SR_BP1             (1u << 3)
#define SR_BP2             (1u << 4)
#define SR_SRWD            (1u << 7)

/* Flash parameters (will be detected) */
static uint32_t s_flash_size = 0;
static uint32_t s_sector_size = 4 * 1024;
static uint32_t s_page_size = 256;

/**
 * Select/deselect flash CS
 */
static inline void cs_assert(void)
{
    driver_spi_cs_select(NOR_SPI_ID, NOR_SPI_CS);
}

static inline void cs_deassert(void)
{
    driver_spi_cs_deselect(NOR_SPI_ID, NOR_SPI_CS);
}

/**
 * Read flash status register
 */
static uint8_t read_status(void)
{
    uint8_t cmd = CMD_READ_STATUS;
    uint8_t status;

    cs_assert();
    driver_spi_transfer(NOR_SPI_ID, &cmd, NULL, 1);
    driver_spi_transfer(NOR_SPI_ID, NULL, &status, 1);
    cs_deassert();

    return status;
}

/**
 * Wait for flash operation to complete
 */
static void wait_ready(void)
{
    while (read_status() & SR_BUSY) {
        /* Small delay */
        hal_timer_delay_us(10);
    }
}

/**
 * Send write enable command
 */
static void write_enable(void)
{
    uint8_t cmd = CMD_WRITE_ENABLE;

    cs_assert();
    driver_spi_transfer(NOR_SPI_ID, &cmd, NULL, 1);
    cs_deassert();
}

/**
 * Initialize NOR SPI flash
 */
int nor_flash_init(void)
{
    uint8_t cmd[4];
    uint8_t id[3];

    /* Initialize SPI */
    spi_config_t spi_cfg = {
        .speed_hz = 50000000,  /* 50 MHz */
        .mode = 0,             /* Mode 0 */
        .bits_per_word = 8
    };
    driver_spi_init(NOR_SPI_ID, &spi_cfg);

    /* Read JEDEC ID */
    cmd[0] = CMD_READ_JEDEC_ID;
    cs_assert();
    driver_spi_transfer(NOR_SPI_ID, cmd, NULL, 1);
    driver_spi_transfer(NOR_SPI_ID, NULL, id, 3);
    cs_deassert();

    /* Decode manufacturer and device ID */
    /* For now, assume generic parameters */
    s_flash_size = 16 * 1024 * 1024;  /* 16 MB */
    s_sector_size = 4 * 1024;         /* 4 KB sectors */
    s_page_size = 256;                 /* 256 byte pages */

    return E_OK;
}

/**
 * Read from flash
 */
int nor_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t cmd[4];
    uint32_t offset = 0;

    while (offset < len) {
        uint32_t chunk_len = (len - offset > 4096) ? 4096 : (len - offset);

        /* Fast read command */
        cmd[0] = CMD_FAST_READ;
        cmd[1] = (addr + offset) >> 16;
        cmd[2] = (addr + offset) >> 8;
        cmd[3] = (addr + offset) & 0xFF;

        cs_assert();
        driver_spi_transfer(NOR_SPI_ID, cmd, NULL, 4);
        /* Dummy clock for fast read */
        uint8_t dummy = 0;
        driver_spi_transfer(NOR_SPI_ID, &dummy, NULL, 1);
        driver_spi_transfer(NOR_SPI_ID, NULL, buf + offset, chunk_len);
        cs_deassert();

        offset += chunk_len;
    }

    return len;
}

/**
 * Write to flash (page program)
 */
int nor_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint32_t offset = 0;

    while (offset < len) {
        uint32_t page_offset = addr % s_page_size;
        uint32_t chunk_len = s_page_size - page_offset;
        if (chunk_len > (len - offset)) {
            chunk_len = len - offset;
        }

        /* Write enable */
        write_enable();

        /* Page program command */
        uint8_t cmd[4];
        cmd[0] = CMD_PAGE_PROGRAM;
        cmd[1] = (addr + offset) >> 16;
        cmd[2] = (addr + offset) >> 8;
        cmd[3] = (addr + offset) & 0xFF;

        cs_assert();
        driver_spi_transfer(NOR_SPI_ID, cmd, NULL, 4);
        driver_spi_transfer(NOR_SPI_ID, buf + offset, NULL, chunk_len);
        cs_deassert();

        /* Wait for write to complete */
        wait_ready();

        offset += chunk_len;
    }

    return len;
}

/**
 * Erase flash sectors
 */
int nor_flash_erase(uint32_t addr, uint32_t len)
{
    /* Align to sector size */
    uint32_t sector_addr = addr & ~(s_sector_size - 1);
    uint32_t num_sectors = (len + s_sector_size - 1) / s_sector_size;

    for (uint32_t i = 0; i < num_sectors; i++) {
        /* Write enable */
        write_enable();

        /* Sector erase command */
        uint8_t cmd[4];
        cmd[0] = CMD_SECTOR_ERASE;
        cmd[1] = (sector_addr + (i * s_sector_size)) >> 16;
        cmd[2] = (sector_addr + (i * s_sector_size)) >> 8;
        cmd[3] = (sector_addr + (i * s_sector_size)) & 0xFF;

        cs_assert();
        driver_spi_transfer(NOR_SPI_ID, cmd, NULL, 4);
        cs_deassert();

        /* Wait for erase to complete */
        wait_ready();
    }

    return E_OK;
}
