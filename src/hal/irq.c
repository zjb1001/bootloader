/**
 * @file irq.c
 * @brief Interrupt controller HAL implementation
 */
#include "hal/irq.h"
#include "errno.h"

#define MAX_IRQS 128

static struct {
    irq_handler_t handler;
    void         *arg;
} s_irq_table[MAX_IRQS];

int hal_irq_init(void)
{
    /* TODO: set up vector table, clear pending */
    for (int i = 0; i < MAX_IRQS; i++) {
        s_irq_table[i].handler = NULL;
        s_irq_table[i].arg = NULL;
    }
    return E_OK;
}

int hal_irq_register(uint32_t irq, irq_handler_t handler, void *arg)
{
    if (irq >= MAX_IRQS) return E_INVAL;
    s_irq_table[irq].handler = handler;
    s_irq_table[irq].arg = arg;
    return E_OK;
}

int hal_irq_enable(uint32_t irq)
{
    /* TODO: write to interrupt controller enable register */
    (void)irq;
    return E_OK;
}

int hal_irq_disable(uint32_t irq)
{
    (void)irq;
    return E_OK;
}

int hal_irq_set_trigger(uint32_t irq, irq_trigger_t trigger)
{
    (void)irq; (void)trigger;
    return E_OK;
}

uint32_t hal_irq_disable_all(void)
{
    /* TODO: read CPSR/PRIMASK, disable */
    return 0;
}

void hal_irq_restore(uint32_t state)
{
    (void)state;
}
