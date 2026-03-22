/**
 * @file driver/rsa.h
 * @brief RSA-2048 signature verification  [THREAD]
 */
#ifndef DRIVER_RSA_H
#define DRIVER_RSA_H

#include "types.h"

#define RSA_KEY_BYTES   256   /* 2048 bits */

typedef struct {
    uint8_t exponent[4];              /**< Public exponent (usually 65537) */
    uint8_t modulus[RSA_KEY_BYTES];   /**< Public modulus */
} rsa_public_key_t;

typedef struct {
    uint8_t signature[RSA_KEY_BYTES];
} rsa_signature_t;

/**
 * Verify an RSA-2048 signature.
 * @return 0 = valid, 1 = invalid, < 0 = error.
 */
int driver_rsa_verify(const uint8_t *message, uint32_t msg_len,
                      const rsa_signature_t *signature,
                      const rsa_public_key_t *public_key);

#endif /* DRIVER_RSA_H */
