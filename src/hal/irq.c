/**
 * @file irq.c
 * @brief Interrupt controller HAL implementation
 *
 * Generic interrupt controller (GIC) support for ARMv7-A platforms.
 * Compatible with ARM GICv1/v2 specifications.
 */
#include "hal/irq.h"
#include "errno.h"
#include "types.h"

#define MAX_IRQS  1024

/* GIC register base addresses (platform specific) */
#define GIC_DIST_BASE    0x01C81000u  /* Distributor */
#define GIC_CPU_BASE     0x01C82000u  /* CPU interface */

/* Distributor register offsets */
#define GICD_CTLR        0x0000  /* Distributor Control Register */
#define GICD_TYPER       0x0004  /* Interrupt Controller Type Register */
#define GICD_ISENABLER   0x0100  /* Interrupt Set-Enable Registers */
#define GICD_ICENABLER   0x0180  /* Interrupt Clear-Enable Registers */
#define GICD_ISPENDR     0x0200  /* Interrupt Set-Pending Registers */
#define GICD_ICPENDR     0x0280  /* Interrupt Clear-Pending Registers */
#define GICD_ICFGR       0x0C00  /* Interrupt Configuration Registers */

/* CPU interface register offsets */
#define GICC_CTLR        0x0000  /* CPU Interface Control Register */
#define GICC_PMR         0x0004  /* Interrupt Priority Mask Register */
#define GICC_BPR         0x0008  /* Binary Point Register */
#define GICC_IAR         0x000C  /* Interrupt Acknowledge Register */
#define GICC_EOIR        0x0010  /* End of Interrupt Register */
#define GICC_RPR         0x0014  /* Running Priority Register */
#define GICC_HPPIR       0x0018  /* Highest Priority Pending Interrupt Register */

#define REG32(addr) (*(volatile uint32_t *)(addr))

/* IRQ handler table */
static struct {
    irq_handler_t handler;
    void         *arg;
} s_irq_table[MAX_IRQS];

/* Maximum IRQ number from GIC type register */
static uint32_t s_max_irq = 0;

/* ARM CPSR/PRIMASK manipulation */
static inline uint32_t irq_save(void)
{
    uint32_t cpsr;
    __asm__ volatile ("mrs %0, cpsr" : "=r"(cpsr));
    __asm__ volatile ("cpsid i" ::: "memory");
    return cpsr;
}

static inline void irq_restore(uint32_t cpsr)
{
    __asm__ volatile ("msr cpsr_c, %0" :: "r"(cpsr) : "memory");
}

/**
 * Initialize interrupt controller
 */
int hal_irq_init(void)
{
    uint32_t i, num_irqs;

    /* Read the number of implemented IRQs */
    uint32_t typer = REG32(GIC_DIST_BASE + GICD_TYPER);
    num_irqs = ((typer & 0x1F) + 1) * 32;  /* ITLinesNumber field */
    if (num_irqs > MAX_IRQS) num_irqs = MAX_IRQS;
    s_max_irq = num_irqs;

    /* Disable all interrupts */
    for (i = 0; i < num_irqs; i += 32) {
        REG32(GIC_DIST_BASE + GICD_ICENABLER + (i / 8)) = 0xFFFFFFFF;
    }

    /* Clear all pending interrupts */
    for (i = 0; i < num_irqs; i += 32) {
        REG32(GIC_DIST_BASE + GICD_ICPENDR + (i / 8)) = 0xFFFFFFFF;
    }

    /* Set all SPIs to level-sensitive (default) */
    for (i = 32; i < num_irqs; i += 16) {
        REG32(GIC_DIST_BASE + GICD_ICFGR + (i / 4)) = 0;
    }

    /* Initialize handler table */
    for (i = 0; i < MAX_IRQS; i++) {
        s_irq_table[i].handler = NULL;
        s_irq_table[i].arg = NULL;
    }

    /* Enable distributor */
    REG32(GIC_DIST_BASE + GICD_CTLR) = 1;

    /* Enable CPU interface */
    REG32(GIC_CPU_BASE + GICC_CTLR) = 1;

    /* Set priority mask to allow all interrupts */
    REG32(GIC_CPU_BASE + GICC_PMR) = 0xFF;

    /* Set binary point to 0 (all priority bits used) */
    REG32(GIC_CPU_BASE + GICC_BPR) = 0;

    return E_OK;
}

/**
 * Register interrupt handler
 */
int hal_irq_register(uint32_t irq, irq_handler_t handler, void *arg)
{
    if (irq >= s_max_irq) return E_INVAL;

    uint32_t cpsr = irq_save();
    s_irq_table[irq].handler = handler;
    s_irq_table[irq].arg = arg;
    irq_restore(cpsr);

    return E_OK;
}

/**
 * Enable specific IRQ
 */
int hal_irq_enable(uint32_t irq)
{
    if (irq >= s_max_irq) return E_INVAL;

    uint32_t reg = GIC_DIST_BASE + GICD_ISENABLER + (irq / 32) * 4;
    uint32_t bit = 1u << (irq % 32);

    REG32(reg) = bit;

    return E_OK;
}

/**
 * Disable specific IRQ
 */
int hal_irq_disable(uint32_t irq)
{
    if (irq >= s_max_irq) return E_INVAL;

    uint32_t reg = GIC_DIST_BASE + GICD_ICENABLER + (irq / 32) * 4;
    uint32_t bit = 1u << (irq % 32);

    REG32(reg) = bit;

    return E_OK;
}

/**
 * Configure interrupt trigger mode
 */
int hal_irq_set_trigger(uint32_t irq, irq_trigger_t trigger)
{
    if (irq < 32 || irq >= s_max_irq) return E_INVAL;  /* SGI/PPI don't have configurable trigger */

    uint32_t reg = GIC_DIST_BASE + GICD_ICFGR + (irq / 16) * 4;
    uint32_t shift = ((irq % 16) * 2) + 1;
    uint32_t mask = 0x3 << shift;
    uint32_t cfg;

    switch (trigger) {
        case IRQ_LEVEL_LOW:
        case IRQ_LEVEL_HIGH:
            cfg = 0 << shift;  /* Level-sensitive */
            break;
        case IRQ_EDGE_RISING:
        case IRQ_EDGE_FALLING:
            cfg = 2 << shift;  /* Edge-triggered */
            break;
        default:
            return E_INVAL;
    }

    uint32_t reg_val = REG32(reg);
    REG32(reg) = (reg_val & ~mask) | cfg;

    return E_OK;
}

/**
 * Disable all interrupts and return previous state
 */
uint32_t hal_irq_disable_all(void)
{
    return irq_save();
}

/**
 * Restore interrupt state
 */
void hal_irq_restore(uint32_t state)
{
    irq_restore(state);
}

/**
 * Generic IRQ handler (called from assembly vector)
 */
void hal_irq_handler_entry(void)
{
    /* Read interrupt acknowledge */
    uint32_t iar = REG32(GIC_CPU_BASE + GICC_IAR);
    uint32_t irq = iar & 0x3FF;

    /* Handle spurious interrupts */
    if (irq >= 1020) {
        return;
    }

    /* Call registered handler */
    if (irq < s_max_irq && s_irq_table[irq].handler) {
        s_irq_table[irq].handler(irq, s_irq_table[irq].arg);
    }

    /* End of interrupt */
    REG32(GIC_CPU_BASE + GICC_EOIR) = iar;
}
