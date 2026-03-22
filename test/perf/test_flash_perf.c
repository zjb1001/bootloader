/**
 * @file test_flash_perf.c
 * @brief Performance benchmark – Flash sequential read speed
 *
 * Target: >= 25 MB/s on target hardware.
 */
#include "test_framework.h"
#include "driver/flash.h"

#include <time.h>

#define READ_SIZE (1u * 1024u * 1024u) /* 1 MB */

static uint8_t read_buf[READ_SIZE];

TEST_CASE(flash_read_throughput)
{
    int rc = driver_flash_init();
    TEST_ASSERT_EQ(rc, 0);

    clock_t start = clock();

    for (int iter = 0; iter < 10; iter++) {
        driver_flash_read(0, read_buf, READ_SIZE);
    }

    clock_t end = clock();
    double seconds = (double)(end - start) / CLOCKS_PER_SEC;
    double mb_per_sec = (seconds > 0)
        ? (10.0 * READ_SIZE / (1024.0 * 1024.0)) / seconds
        : 0;

    printf("  Flash read throughput: %.1f MB/s\n", mb_per_sec);
}

TEST_MAIN_BEGIN
    RUN_TEST(flash_read_throughput);
TEST_MAIN_END
