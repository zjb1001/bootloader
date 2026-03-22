/**
 * @file driver/ecdsa.h
 * @brief ECDSA-P256 signature verification  [THREAD]
 */
#ifndef DRIVER_ECDSA_H
#define DRIVER_ECDSA_H

#include "types.h"

typedef struct {
    uint8_t x[32];        /**< Public key X coordinate */
    uint8_t y[32];        /**< Public key Y coordinate */
} ecdsa_public_key_t;

typedef struct {
    uint8_t r[32];        /**< Signature R component */
    uint8_t s[32];        /**< Signature S component */
} ecdsa_signature_t;

/**
 * Verify an ECDSA-P256 signature.
 * @return 0 = valid, 1 = invalid, < 0 = error.
 */
int driver_ecdsa_verify(const uint8_t *message, uint32_t msg_len,
                        const ecdsa_signature_t *signature,
                        const ecdsa_public_key_t *public_key);

#endif /* DRIVER_ECDSA_H */
