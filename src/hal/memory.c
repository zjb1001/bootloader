/**
 * @file memory.c
 * @brief MMU, cache, and DRAM HAL implementation
 */
#include "hal/memory.h"
#include "errno.h"

int hal_mmu_init(void)
{
    /* TODO: build initial page table */
    return E_OK;
}

int hal_mmu_enable(void)
{
    /* TODO: set SCTLR.M bit */
    return E_OK;
}

int hal_mmu_disable(void)
{
    return E_OK;
}

int hal_mmu_map(const mmu_entry_t *entry)
{
    (void)entry;
    return E_OK;
}

int hal_mmu_tlb_flush(void)
{
    return E_OK;
}

int hal_cache_enable(uint32_t level)
{
    (void)level;
    return E_OK;
}

int hal_cache_disable(uint32_t level)
{
    (void)level;
    return E_OK;
}

int hal_cache_flush(uint32_t level)
{
    (void)level;
    return E_OK;
}

int hal_dram_init(uint32_t dram_size)
{
    /* TODO: DDR PHY training, set timing parameters */
    (void)dram_size;
    return E_OK;
}

int hal_dram_test(uint32_t start, uint32_t size)
{
    /* TODO: walking-ones, address bus test */
    (void)start; (void)size;
    return E_OK;
}
