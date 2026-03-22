/**
 * @file test_sha256.c
 * @brief Unit tests for SHA-256
 */
#include "test_framework.h"
#include "driver/sha256.h"

TEST_CASE(sha256_empty)
{
    uint8_t digest[SHA256_DIGEST_SIZE];
    driver_sha256((const uint8_t *)"", 0, digest);
    /* SHA256("") = e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855 */
    const uint8_t expected[SHA256_DIGEST_SIZE] = {
        0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14,
        0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f, 0xb9, 0x24,
        0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c,
        0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55,
    };
    TEST_ASSERT_MEM_EQ(digest, expected, SHA256_DIGEST_SIZE);
}

TEST_CASE(sha256_abc)
{
    uint8_t digest[SHA256_DIGEST_SIZE];
    driver_sha256((const uint8_t *)"abc", 3, digest);
    /* SHA256("abc") = ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad */
    const uint8_t expected[SHA256_DIGEST_SIZE] = {
        0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea,
        0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
        0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c,
        0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad,
    };
    TEST_ASSERT_MEM_EQ(digest, expected, SHA256_DIGEST_SIZE);
}

TEST_CASE(sha256_incremental)
{
    sha256_ctx_t ctx;
    uint8_t digest1[SHA256_DIGEST_SIZE], digest2[SHA256_DIGEST_SIZE];

    /* Incremental */
    driver_sha256_init(&ctx);
    driver_sha256_update(&ctx, (const uint8_t *)"a", 1);
    driver_sha256_update(&ctx, (const uint8_t *)"bc", 2);
    driver_sha256_final(&ctx, digest1);

    /* One-shot */
    driver_sha256((const uint8_t *)"abc", 3, digest2);

    TEST_ASSERT_MEM_EQ(digest1, digest2, SHA256_DIGEST_SIZE);
}

TEST_MAIN_BEGIN
    RUN_TEST(sha256_empty);
    RUN_TEST(sha256_abc);
    RUN_TEST(sha256_incremental);
TEST_MAIN_END
