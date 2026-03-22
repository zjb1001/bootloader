/**
 * @file test_verify.c
 * @brief Unit tests for image verification pipeline
 */
#include "test_framework.h"
#include "core/verify.h"
#include "errno.h"

TEST_CASE(verify_version_ok)
{
    int rc = core_verify_version(120, 100);
    TEST_ASSERT_EQ(rc, E_OK);
}

TEST_CASE(verify_version_rollback)
{
    int rc = core_verify_version(90, 100);
    TEST_ASSERT_EQ(rc, E_ROLLBACK);
}

TEST_CASE(verify_version_equal)
{
    int rc = core_verify_version(100, 100);
    TEST_ASSERT_EQ(rc, E_OK);
}

TEST_CASE(verify_image_null)
{
    /* Passing NULL should be handled gracefully */
    /* TODO: depends on core_verify_image null-check implementation */
}

TEST_MAIN_BEGIN
    RUN_TEST(verify_version_ok);
    RUN_TEST(verify_version_rollback);
    RUN_TEST(verify_version_equal);
    RUN_TEST(verify_image_null);
TEST_MAIN_END
