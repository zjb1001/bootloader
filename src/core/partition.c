/**
 * @file partition.c
 * @brief Partition table read and lookup
 */
#include "core/partition.h"
#include "driver/flash.h"
#include "errno.h"

static partition_table_t s_ptable;
static int s_ptable_loaded = 0;

int core_partition_read(partition_table_t *table)
{
    /* TODO: read from CONFIG_PART_OFFSET, validate magic */
    (void)table;
    s_ptable_loaded = 0;
    return E_OK;
}

partition_entry_t *core_partition_find(const char *name)
{
    if (!s_ptable_loaded) return NULL;
    /* TODO: linear search s_ptable.entries by name */
    (void)name;
    (void)s_ptable;
    return NULL;
}

uint32_t core_partition_get_addr(const char *name)
{
    partition_entry_t *e = core_partition_find(name);
    return e ? e->offset : 0;
}

uint32_t core_partition_get_size(const char *name)
{
    partition_entry_t *e = core_partition_find(name);
    return e ? e->size : 0;
}
