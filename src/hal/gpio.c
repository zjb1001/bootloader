/**
 * @file gpio.c
 * @brief GPIO HAL implementation
 *
 * Provides GPIO control for platform with memory-mapped GPIO ports.
 * Implementation for ARMv7-A with typical GPIO controller.
 */
#include "hal/gpio.h"
#include "errno.h"
#include "types.h"

/* GPIO register base addresses */
#define GPIOA_BASE  0x01C20800u
#define GPIOB_BASE  0x01C20824u
#define GPIOC_BASE  0x01C20848u
#define GPIOD_BASE  0x01C2086Cu
#define GPIOE_BASE  0x01C20890u
#define GPIOF_BASE  0x01C208B4u
#define GPIOG_BASE  0x01C208D8u
#define GPIOH_BASE  0x01C208FCu

/* GPIO register offsets */
#define GPIO_CFG_OFFSET   0x00   /* Port configuration */
#define GPIO_DATA_OFFSET  0x10   /* Data register */
#define GPIO_DRV_OFFSET   0x14   /* Drive level */
#define GPIO_PULL_OFFSET  0x18   /* Pull up/down */

#define REG32(addr) (*(volatile uint32_t *)(addr))

/* Maximum number of pins per port */
#define PINS_PER_PORT  32
#define NUM_PORTS      8

/* Pin number to port and bit position */
static inline int gpio_decode_pin(uint32_t pin, uint32_t *port_base, uint32_t *bit)
{
    if (pin >= NUM_PORTS * PINS_PER_PORT) return E_INVAL;

    uint32_t port = pin / PINS_PER_PORT;
    uint32_t bit_pos = pin % PINS_PER_PORT;

    switch (port) {
        case 0: *port_base = GPIOA_BASE; break;
        case 1: *port_base = GPIOB_BASE; break;
        case 2: *port_base = GPIOC_BASE; break;
        case 3: *port_base = GPIOD_BASE; break;
        case 4: *port_base = GPIOE_BASE; break;
        case 5: *port_base = GPIOF_BASE; break;
        case 6: *port_base = GPIOG_BASE; break;
        case 7: *port_base = GPIOH_BASE; break;
        default: return E_INVAL;
    }

    *bit = bit_pos;
    return E_OK;
}

/* Configuration values for mode */
#define CFG_INPUT    0x0
#define CFG_OUTPUT   0x1
#define CFG_ALT_2    0x2
#define CFG_ALT_3    0x3
#define CFG_ALT_4    0x4
#define CFG_ALT_5    0x5

/* Pull up/down values */
#define PULL_NONE    0x0
#define PULL_UP      0x1
#define PULL_DOWN    0x2

/**
 * Initialize GPIO pin
 */
int hal_gpio_init(uint32_t pin, gpio_mode_t mode, gpio_pull_t pull)
{
    uint32_t base, bit;
    int ret;

    ret = gpio_decode_pin(pin, &base, &bit);
    if (ret < 0) return ret;

    volatile uint32_t *cfg_reg = (volatile uint32_t *)(base + GPIO_CFG_OFFSET + (bit / 8) * 4);
    uint32_t shift = (bit % 8) * 4;
    uint32_t mask = 0xF << shift;
    uint32_t cfg_val;

    /* Set mode */
    switch (mode) {
        case GPIO_MODE_INPUT:
            cfg_val = CFG_INPUT;
            break;
        case GPIO_MODE_OUTPUT:
            cfg_val = CFG_OUTPUT;
            break;
        case GPIO_MODE_ALT_FUNC:
            cfg_val = CFG_ALT_2;  /* Default to alt function 2 */
            break;
        default:
            return E_INVAL;
    }

    uint32_t reg_val = *cfg_reg;
    reg_val = (reg_val & ~mask) | (cfg_val << shift);
    *cfg_reg = reg_val;

    /* Set pull up/down */
    volatile uint32_t *pull_reg = (volatile uint32_t *)(base + GPIO_PULL_OFFSET + (bit / 16) * 4);
    shift = (bit % 16) * 2;
    mask = 0x3 << shift;

    uint32_t pull_val;
    switch (pull) {
        case GPIO_PULL_NONE: pull_val = PULL_NONE; break;
        case GPIO_PULL_UP:   pull_val = PULL_UP;   break;
        case GPIO_PULL_DOWN: pull_val = PULL_DOWN; break;
        default: return E_INVAL;
    }

    reg_val = *pull_reg;
    reg_val = (reg_val & ~mask) | (pull_val << shift);
    *pull_reg = reg_val;

    return E_OK;
}

/**
 * Set GPIO output value
 */
int hal_gpio_set(uint32_t pin, uint8_t value)
{
    uint32_t base, bit;
    int ret;

    ret = gpio_decode_pin(pin, &base, &bit);
    if (ret < 0) return ret;

    volatile uint32_t *data_reg = (volatile uint32_t *)(base + GPIO_DATA_OFFSET);

    if (value) {
        *data_reg |= (1u << bit);
    } else {
        *data_reg &= ~(1u << bit);
    }

    return E_OK;
}

/**
 * Get GPIO input value
 */
int hal_gpio_get(uint32_t pin)
{
    uint32_t base, bit;
    int ret;

    ret = gpio_decode_pin(pin, &base, &bit);
    if (ret < 0) return ret;

    volatile uint32_t *data_reg = (volatile uint32_t *)(base + GPIO_DATA_OFFSET);

    return (*data_reg >> bit) & 1;
}

/**
 * Set GPIO alternate function
 */
int hal_gpio_set_alt_func(uint32_t pin, uint32_t func)
{
    uint32_t base, bit;
    int ret;

    ret = gpio_decode_pin(pin, &base, &bit);
    if (ret < 0) return ret;

    if (func > 7) return E_INVAL;

    volatile uint32_t *cfg_reg = (volatile uint32_t *)(base + GPIO_CFG_OFFSET + (bit / 8) * 4);
    uint32_t shift = (bit % 8) * 4;
    uint32_t mask = 0xF << shift;

    uint32_t reg_val = *cfg_reg;
    reg_val = (reg_val & ~mask) | ((func & 0x7) << shift);
    *cfg_reg = reg_val;

    return E_OK;
}
