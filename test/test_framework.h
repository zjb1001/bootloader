/**
 * @file test_framework.h
 * @brief Minimal unit-test framework (no external deps)
 *
 * Usage:
 *   TEST_CASE(name) { ... TEST_ASSERT(...); ... }
 *   TEST_MAIN_BEGIN
 *       RUN_TEST(name);
 *   TEST_MAIN_END
 */
#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <string.h>

static int _test_pass_count;
static int _test_fail_count;
static const char *_current_test;

#define TEST_CASE(name) static void test_##name(void)

#define TEST_ASSERT(cond)                                               \
    do {                                                                \
        if (!(cond)) {                                                  \
            printf("  FAIL: %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
            _test_fail_count++;                                         \
            return;                                                     \
        }                                                               \
    } while (0)

#define TEST_ASSERT_EQ(a, b) TEST_ASSERT((a) == (b))
#define TEST_ASSERT_NE(a, b) TEST_ASSERT((a) != (b))
#define TEST_ASSERT_GT(a, b) TEST_ASSERT((a) > (b))
#define TEST_ASSERT_LT(a, b) TEST_ASSERT((a) < (b))

#define TEST_ASSERT_MEM_EQ(a, b, n) TEST_ASSERT(memcmp((a), (b), (n)) == 0)

#define RUN_TEST(name)                          \
    do {                                        \
        _current_test = #name;                  \
        int _prev_fail = _test_fail_count;      \
        test_##name();                          \
        if (_test_fail_count == _prev_fail) {   \
            _test_pass_count++;                 \
            printf("  PASS: %s\n", #name);      \
        }                                       \
    } while (0)

#define TEST_MAIN_BEGIN                                     \
    int main(void) {                                        \
        _test_pass_count = 0;                               \
        _test_fail_count = 0;                               \
        printf("=== Running tests ===\n");

#define TEST_MAIN_END                                       \
        printf("=== Results: %d passed, %d failed ===\n",  \
               _test_pass_count, _test_fail_count);         \
        return _test_fail_count > 0 ? 1 : 0;               \
    }

#endif /* TEST_FRAMEWORK_H */
