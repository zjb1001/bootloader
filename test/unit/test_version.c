/**
 * @file test_version.c
 * @brief Unit tests for version / rollback protection
 */
#include "test_framework.h"
#include "core/verify.h"
#include "errno.h"

TEST_CASE(version_boundary_min)
{
    TEST_ASSERT_EQ(core_verify_version(0, 0), E_OK);
}

TEST_CASE(version_boundary_max)
{
    TEST_ASSERT_EQ(core_verify_version(0xFFFFFFFF, 0xFFFFFFFF), E_OK);
}

TEST_CASE(version_rollback_by_one)
{
    TEST_ASSERT_EQ(core_verify_version(99, 100), E_ROLLBACK);
}

TEST_MAIN_BEGIN
    RUN_TEST(version_boundary_min);
    RUN_TEST(version_boundary_max);
    RUN_TEST(version_rollback_by_one);
TEST_MAIN_END
