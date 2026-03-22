/**
 * @file test_sha256_perf.c
 * @brief Performance benchmark – SHA-256 throughput
 *
 * Target: >= 50 MB/s on target hardware.
 * On host this just validates the test harness works.
 */
#include "test_framework.h"
#include "driver/sha256.h"

#include <time.h>

#define BENCH_SIZE (1u * 1024u * 1024u)  /* 1 MB */

static uint8_t bench_buf[BENCH_SIZE];

TEST_CASE(sha256_throughput)
{
    /* Fill buffer with pattern */
    for (uint32_t i = 0; i < BENCH_SIZE; i++) {
        bench_buf[i] = (uint8_t)(i & 0xFF);
    }

    uint8_t digest[SHA256_DIGEST_SIZE];
    clock_t start = clock();

    for (int iter = 0; iter < 10; iter++) {
        driver_sha256(bench_buf, BENCH_SIZE, digest);
    }

    clock_t end = clock();
    double seconds = (double)(end - start) / CLOCKS_PER_SEC;
    double mb_per_sec = (10.0 * BENCH_SIZE / (1024.0 * 1024.0)) / seconds;

    printf("  SHA256 throughput: %.1f MB/s\n", mb_per_sec);
    /* On host stub this will be near-instant; real target must be >= 50 */
}

TEST_MAIN_BEGIN
    RUN_TEST(sha256_throughput);
TEST_MAIN_END
