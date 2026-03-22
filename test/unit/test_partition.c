/**
 * @file test_partition.c
 * @brief Unit tests for partition table parsing
 */
#include "test_framework.h"
#include "core/partition.h"
#include "config.h"

TEST_CASE(partition_read_valid)
{
    partition_table_t table;
    int rc = core_partition_read(&table);
    /* TODO: pre-populate mock flash with a valid partition table */
    (void)rc;
}

TEST_CASE(partition_find_kernel)
{
    partition_entry_t *entry = core_partition_find("kernel");
    /* TODO: verify after mock setup */
    (void)entry;
}

TEST_CASE(partition_find_nonexistent)
{
    partition_entry_t *entry = core_partition_find("nonexistent");
    TEST_ASSERT(entry == NULL);
}

TEST_MAIN_BEGIN
    RUN_TEST(partition_read_valid);
    RUN_TEST(partition_find_kernel);
    RUN_TEST(partition_find_nonexistent);
TEST_MAIN_END
