/**
 * @file ecdsa.c
 * @brief ECDSA-P256 signature verification
 */
#include "driver/ecdsa.h"
#include "errno.h"

int driver_ecdsa_verify(const uint8_t *message, uint32_t msg_len,
                        const ecdsa_signature_t *signature,
                        const ecdsa_public_key_t *public_key)
{
    /* TODO: P-256 point operations → verify (r, s) */
    (void)message; (void)msg_len; (void)signature; (void)public_key;
    return E_OK;
}
