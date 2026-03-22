/**
 * @file hal/irq.h
 * @brief Interrupt controller HAL interface
 */
#ifndef HAL_IRQ_H
#define HAL_IRQ_H

#include "types.h"

/** IRQ handler callback signature. */
typedef void (*irq_handler_t)(uint32_t irq_num, void *arg);

typedef enum {
    IRQ_LEVEL_LOW = 0,
    IRQ_LEVEL_HIGH,
    IRQ_EDGE_RISING,
    IRQ_EDGE_FALLING,
} irq_trigger_t;

/** Initialize the interrupt controller.  [INIT] */
int hal_irq_init(void);

/** Register handler for a specific IRQ.  [INIT] */
int hal_irq_register(uint32_t irq, irq_handler_t handler, void *arg);

/** Enable a specific IRQ.  [ISR-safe] */
int hal_irq_enable(uint32_t irq);

/** Disable a specific IRQ.  [ISR-safe] */
int hal_irq_disable(uint32_t irq);

/** Configure trigger mode.  [INIT] */
int hal_irq_set_trigger(uint32_t irq, irq_trigger_t trigger);

/** Globally disable all interrupts; returns previous state.  [ISR-safe] */
uint32_t hal_irq_disable_all(void);

/** Restore interrupt state from a previous disable_all call.  [ISR-safe] */
void hal_irq_restore(uint32_t state);

#endif /* HAL_IRQ_H */
