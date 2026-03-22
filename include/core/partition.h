/**
 * @file core/partition.h
 * @brief Partition table management  [THREAD]
 */
#ifndef CORE_PARTITION_H
#define CORE_PARTITION_H

#include "types.h"
#include "config.h"

typedef struct {
    char     name[32];
    uint32_t offset;
    uint32_t size;
    uint32_t flags;
} partition_entry_t;

typedef struct {
    uint32_t          magic;       /**< Must equal PARTITION_TABLE_MAGIC */
    uint32_t          version;
    uint32_t          entry_count;
    partition_entry_t entries[MAX_PARTITIONS];
} partition_table_t;

/** Read partition table from Flash. */
int core_partition_read(partition_table_t *table);

/** Lookup a partition by name. @return entry pointer or NULL. */
partition_entry_t *core_partition_find(const char *name);

/** Convenience: get partition start offset. @return offset or 0 on not-found. */
uint32_t core_partition_get_addr(const char *name);

/** Convenience: get partition size. @return size or 0 on not-found. */
uint32_t core_partition_get_size(const char *name);

#endif /* CORE_PARTITION_H */
