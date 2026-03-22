/**
 * @file partition.c
 * @brief Partition table read and lookup
 */
#include "core/partition.h"
#include "driver/flash.h"
#include "errno.h"
#include "config.h"
#include "utils/string.h"

static partition_table_t s_ptable;
static int s_ptable_loaded = 0;

int core_partition_read(partition_table_t *table)
{
    int ret;

    if (!table) return E_INVAL;

    /* Read partition table from flash */
    ret = driver_flash_read(CONFIG_PART_OFFSET,
                            (uint8_t *)&s_ptable,
                            sizeof(partition_table_t));
    if (ret < 0) {
        return BOOT_ERR_PARTITION_TABLE;
    }

    /* Verify magic */
    if (s_ptable.magic != PARTITION_TABLE_MAGIC) {
        return BOOT_ERR_PARTITION_TABLE;
    }

    /* Verify entry count */
    if (s_ptable.entry_count > MAX_PARTITIONS) {
        return BOOT_ERR_PARTITION_TABLE;
    }

    /* Copy to output */
    *table = s_ptable;
    s_ptable_loaded = 1;

    return E_OK;
}

partition_entry_t *core_partition_find(const char *name)
{
    if (!name) return NULL;
    if (!s_ptable_loaded) return NULL;

    for (uint32_t i = 0; i < s_ptable.entry_count; i++) {
        if (bl_strcmp(s_ptable.entries[i].name, name) == 0) {
            return &s_ptable.entries[i];
        }
    }

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
