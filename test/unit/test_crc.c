/**
 * @file test_crc.c
 * @brief Unit tests for CRC32/CRC64
 */
#include "test_framework.h"
#include "driver/crc.h"

TEST_CASE(crc32_empty)
{
    uint32_t crc = driver_crc32(NULL, 0, 0);
    /* TODO: verify known CRC32 of empty input */
    (void)crc;
}

TEST_CASE(crc32_known_vector)
{
    const uint8_t data[] = "123456789";
    uint32_t crc = driver_crc32(data, 9, 0);
    /* Expected CRC32 of "123456789" = 0xCBF43926 */
    TEST_ASSERT_EQ(crc, 0xCBF43926u);
}

TEST_CASE(crc32_incremental)
{
    const uint8_t data[] = "123456789";
    uint32_t crc = driver_crc32(data, 5, 0);
    crc = driver_crc32(data + 5, 4, crc);
    /* Should equal one-shot result */
    uint32_t expected = driver_crc32(data, 9, 0);
    TEST_ASSERT_EQ(crc, expected);
}

TEST_CASE(crc64_known_vector)
{
    const uint8_t data[] = "123456789";
    uint64_t crc = driver_crc64(data, 9, 0);
    /* TODO: fill in expected CRC64 value */
    (void)crc;
}

TEST_MAIN_BEGIN
    RUN_TEST(crc32_empty);
    RUN_TEST(crc32_known_vector);
    RUN_TEST(crc32_incremental);
    RUN_TEST(crc64_known_vector);
TEST_MAIN_END
