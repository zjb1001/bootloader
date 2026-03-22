/**
 * @file sha256.c
 * @brief SHA-256 hash implementation
 */
#include "driver/sha256.h"

void driver_sha256_init(sha256_ctx_t *ctx)
{
    /* TODO: set initial hash values H0..H7 */
    (void)ctx;
}

void driver_sha256_update(sha256_ctx_t *ctx, const uint8_t *data, uint32_t len)
{
    /* TODO: process 64-byte blocks */
    (void)ctx; (void)data; (void)len;
}

void driver_sha256_final(sha256_ctx_t *ctx, uint8_t digest[SHA256_DIGEST_SIZE])
{
    /* TODO: pad, finalize, output */
    (void)ctx; (void)digest;
}

void driver_sha256(const uint8_t *data, uint32_t len, uint8_t digest[SHA256_DIGEST_SIZE])
{
    sha256_ctx_t ctx;
    driver_sha256_init(&ctx);
    driver_sha256_update(&ctx, data, len);
    driver_sha256_final(&ctx, digest);
}
