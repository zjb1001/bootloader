/**
 * @file driver/sha256.h
 * @brief SHA-256 hash computation  [THREAD]
 */
#ifndef DRIVER_SHA256_H
#define DRIVER_SHA256_H

#include "types.h"

#define SHA256_DIGEST_SIZE  32
#define SHA256_BLOCK_SIZE   64

typedef struct {
    uint32_t state[8];
    uint32_t count[2];
    uint8_t  buffer[SHA256_BLOCK_SIZE];
} sha256_ctx_t;

void driver_sha256_init(sha256_ctx_t *ctx);
void driver_sha256_update(sha256_ctx_t *ctx, const uint8_t *data, uint32_t len);
void driver_sha256_final(sha256_ctx_t *ctx, uint8_t digest[SHA256_DIGEST_SIZE]);

/** One-shot convenience wrapper. */
void driver_sha256(const uint8_t *data, uint32_t len, uint8_t digest[SHA256_DIGEST_SIZE]);

#endif /* DRIVER_SHA256_H */
