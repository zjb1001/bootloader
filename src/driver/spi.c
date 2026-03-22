/**
 * @file spi.c
 * @brief SPI bus driver
 *
 * Platform-specific SPI implementation for ARMv7-A with SPI controller.
 */
#include "driver/spi.h"
#include "hal/gpio.h"
#include "hal/clock.h"
#include "errno.h"
#include "types.h"

/* SPI register base addresses */
#define SPI0_BASE   0x01C05000u
#define SPI1_BASE   0x01C06000u

/* SPI register offsets */
#define SPI_RXDAT    0x0000  /* RX data */
#define SPI_TXDAT    0x0004  /* TX data */
#define SPI_CTRL     0x0008  /* Control register */
#define SPI_CTRL0    0x000C  /* Control 0 */
#define SPI_CTRL1    0x0010  /* Control 1 */
#define SPI_STAT     0x0014  /* Status */
#define SPI_FIFO_STA 0x0018  /* FIFO status */
#define SPI_IRQ_EN   0x001C  /* IRQ enable */
#define SPI_IRQ_STA  0x0020  /* IRQ status */

/* Control register bits */
#define SPI_CTRL_RST        (1u << 31)  /* Software reset */
#define SPI_CTRL_EN         (1u << 0)   /* Enable SPI */
#define SPI_CTRL_MASTER     (1u << 1)   /* Master mode */

/* Control 0 bits */
#define SPI_CTRL0_CPHA      (1u << 0)   /* Clock phase */
#define SPI_CTRL0_CPOL      (1u << 1)   /* Clock polarity */
#define SPI_CTRL0_SPOL      (1u << 2)   /* Chip select polarity */
#define SPI_CTRL0_SDC       (0x3 << 4)  /* Chip select (2 bits) */
#define SPI_CTRL0_SDC_SHIFT  4
#define SPI_CTRL0_SDM       (0x3 << 6)  /* Data mode (2 bits) */
#define SPI_CTRL0_SDM_SHIFT  6
#define SPI_CTRL0_DHB       (1u << 8)   /* Discard first RX byte */
#define SPI_CTRL0_FBS       (1u << 12)  /* FIFO byte selection */
#define SPI_CTRL0_XCH       (1u << 15)  /* Exchange */

/* Control 1 bits */
#define SPI_CTRL1_TBW       (0x3 << 0)  /* Burst wait (2 bits) */
#define SPI_CTRL1_TBS       (0x7 << 2)  /* Burst counter (3 bits) */
#define SPI_CTRL1_TBS_SHIFT  2
#define SPI_CTRL1_MST       (1u << 5)   /* Master mode */
#define SPI_CTRL1_CPHA      (1u << 6)   /* Clock phase */
#define SPI_CTRL1_CPOL      (1u << 7)   /* Clock polarity */
#define SPI_CTRL1_LMTF      (1u << 8)   /* Limit transmit FIFO */

/* Status bits */
#define SPI_STAT_BUSY       (1u << 0)   /* Controller busy */
#define SPI_STAT_TX_EMPTY   (1u << 2)   /* TX FIFO empty */
#define SPI_STAT_TX_FULL    (1u << 3)   /* TX FIFO full */
#define SPI_STAT_RX_EMPTY   (1u << 4)   /* RX FIFO empty */
#define SPI_STAT_RX_FULL    (1u << 5)   /* RX FIFO full */

#define REG32(addr) (*(volatile uint32_t *)(addr))

/* Get SPI base address */
static inline uint32_t spi_get_base(uint32_t spi_id)
{
    switch (spi_id) {
        case 0: return SPI0_BASE;
        case 1: return SPI1_BASE;
        default: return 0;
    }
}

/**
 * Initialize SPI controller
 */
int driver_spi_init(uint32_t spi_id, const spi_config_t *cfg)
{
    uint32_t base = spi_get_base(spi_id);

    if (!cfg || !base) return E_INVAL;

    /* Enable SPI clock */
    hal_clock_enable_periph(2, 1);  /* SPI0 clock */
    hal_clock_enable_periph(3, 1);  /* SPI1 clock */

    /* Reset SPI controller */
    REG32(base + SPI_CTRL) |= SPI_CTRL_RST;
    while (REG32(base + SPI_CTRL) & SPI_CTRL_RST);

    /* Configure SPI mode */
    uint32_t ctrl0 = 0;

    /* Set CPHA and CPOL based on SPI mode */
    switch (cfg->mode) {
        case 0: ctrl0 |= 0; break;  /* CPOL=0, CPHA=0 */
        case 1: ctrl0 |= SPI_CTRL0_CPHA; break;  /* CPOL=0, CPHA=1 */
        case 2: ctrl0 |= SPI_CTRL0_CPOL; break;  /* CPOL=1, CPHA=0 */
        case 3: ctrl0 |= SPI_CTRL0_CPOL | SPI_CTRL0_CPHA; break;  /* CPOL=1, CPHA=1 */
        default: return E_INVAL;
    }

    /* Set data width (assuming 8-bit for now) */
    ctrl0 |= (0x1 << SPI_CTRL0_SDM_SHIFT);  /* 8-bit data */

    REG32(base + SPI_CTRL0) = ctrl0;

    /* Set master mode and clock divider */
    uint32_t clk = hal_clock_get_freq(CLOCK_PLL_CORE) * 1000000u;
    uint32_t div = (clk + cfg->speed_hz - 1) / cfg->speed_hz;
    if (div < 2) div = 2;
    if (div > 0x1000) div = 0x1000;

    uint32_t ctrl1 = SPI_CTRL1_MST | ((div - 1) << 12);
    REG32(base + SPI_CTRL1) = ctrl1;

    /* Enable SPI */
    REG32(base + SPI_CTRL) |= SPI_CTRL_EN | SPI_CTRL_MASTER;

    return E_OK;
}

/**
 * SPI transfer (full duplex)
 */
int driver_spi_transfer(uint32_t spi_id,
                        const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    uint32_t base = spi_get_base(spi_id);

    if (!base) return E_INVAL;
    if (!len) return E_OK;

    /* Set burst length */
    uint32_t ctrl1 = REG32(base + SPI_CTRL1);
    ctrl1 &= ~SPI_CTRL1_TBS;
    ctrl1 |= ((len - 1) << SPI_CTRL1_TBS_SHIFT);
    REG32(base + SPI_CTRL1) = ctrl1;

    /* Clear FIFOs */
    REG32(base + SPI_FIFO_STA) |= 0x3;

    uint32_t tx_idx = 0;
    uint32_t rx_idx = 0;

    while (tx_idx < len || rx_idx < len) {
        uint32_t stat = REG32(base + SPI_STAT);

        /* TX: fill FIFO if not full and we have data to send */
        if ((stat & SPI_STAT_TX_FULL) == 0 && tx_idx < len) {
            uint8_t data = tx ? tx[tx_idx] : 0xFF;
            REG32(base + SPI_TXDAT) = data;
            tx_idx++;
        }

        /* RX: read FIFO if not empty and we need data */
        if ((stat & SPI_STAT_RX_EMPTY) == 0 && rx_idx < len) {
            uint8_t data = REG32(base + SPI_RXDAT) & 0xFF;
            if (rx) {
                rx[rx_idx] = data;
            }
            rx_idx++;
        }
    }

    /* Wait for transaction to complete */
    while (REG32(base + SPI_STAT) & SPI_STAT_BUSY);

    return len;
}

/**
 * Select chip (assert CS)
 */
int driver_spi_cs_select(uint32_t spi_id, uint8_t cs)
{
    uint32_t base = spi_get_base(spi_id);

    if (!base || cs > 3) return E_INVAL;

    uint32_t ctrl0 = REG32(base + SPI_CTRL0);
    ctrl0 &= ~SPI_CTRL0_SDC;
    ctrl0 |= ((cs & 0x3) << SPI_CTRL0_SDC_SHIFT);
    REG32(base + SPI_CTRL0) = ctrl0;

    return E_OK;
}

/**
 * Deselect chip (deassert CS)
 */
int driver_spi_cs_deselect(uint32_t spi_id, uint8_t cs)
{
    (void)spi_id;
    (void)cs;

    /* Most hardware automatically deselects after transfer */
    return E_OK;
}
