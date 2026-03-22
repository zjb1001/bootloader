/**
 * @file rsa.c
 * @brief RSA-2048 signature verification
 */
#include "driver/rsa.h"
#include "errno.h"

int driver_rsa_verify(const uint8_t *message, uint32_t msg_len,
                      const rsa_signature_t *signature,
                      const rsa_public_key_t *public_key)
{
    /* TODO: modular exponentiation → PKCS#1 v1.5 unpad → compare SHA256 */
    (void)message; (void)msg_len; (void)signature; (void)public_key;
    return E_OK;
}
