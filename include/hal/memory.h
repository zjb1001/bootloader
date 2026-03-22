/**
 * @file hal/memory.h
 * @brief MMU, cache, and DRAM HAL interface
 */
#ifndef HAL_MEMORY_H
#define HAL_MEMORY_H

#include "types.h"

/* MMU entry attributes (bitfield) */
#define MMU_ATTR_READ   0x01u
#define MMU_ATTR_WRITE  0x02u
#define MMU_ATTR_EXEC   0x04u
#define MMU_ATTR_DEVICE 0x08u

typedef struct {
    uint32_t virt_addr;
    uint32_t phys_addr;
    uint32_t size;
    uint32_t attr;           /**< Combination of MMU_ATTR_* */
} mmu_entry_t;

/* ─── MMU ─── */
int hal_mmu_init(void);
int hal_mmu_enable(void);
int hal_mmu_disable(void);
int hal_mmu_map(const mmu_entry_t *entry);
int hal_mmu_tlb_flush(void);

/* ─── Cache ─── */
int hal_cache_enable(uint32_t level);
int hal_cache_disable(uint32_t level);
int hal_cache_flush(uint32_t level);

/* ─── DRAM ─── */
int hal_dram_init(uint32_t dram_size);
int hal_dram_test(uint32_t start, uint32_t size);

#endif /* HAL_MEMORY_H */
