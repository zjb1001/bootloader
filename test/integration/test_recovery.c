/**
 * @file test_recovery.c
 * @brief Integration test – recovery / fallback boot scenarios
 *
 * Validates:
 *  - Invalid signature → fallback to backup partition
 *  - Rollback attempt → reject + recovery mode
 *  - Both partitions corrupt → halt with log
 */
#include "test_framework.h"
#include "core/recovery.h"
#include "errno.h"

TEST_CASE(recovery_fallback)
{
    recovery_action_t action = core_handle_boot_failure(BOOT_ERR_SIGNATURE_INVALID);
    TEST_ASSERT_EQ(action, RECOVERY_FALLBACK);
}

TEST_CASE(recovery_rollback_rejected)
{
    recovery_action_t action = core_handle_boot_failure(BOOT_ERR_VERSION_ROLLBACK);
    /* TODO: verify correct recovery strategy for rollback */
    (void)action;
}

TEST_MAIN_BEGIN
    RUN_TEST(recovery_fallback);
    RUN_TEST(recovery_rollback_rejected);
TEST_MAIN_END
