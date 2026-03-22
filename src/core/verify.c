/**
 * @file verify.c
 * @brief Multi-stage image verification (CRC → SHA → Sig → Version)
 */
#include "core/verify.h"
#include "driver/crc.h"
#include "driver/sha256.h"
#include "driver/rsa.h"
#include "driver/ecdsa.h"
#include "errno.h"

int core_verify_image(const image_t *image)
{
    /* TODO: implement verification pipeline per design */
    (void)image;
    return E_OK;
}

int core_verify_version(uint32_t current_version, uint32_t min_allowed_version)
{
    if (current_version < min_allowed_version) {
        return E_ROLLBACK;
    }
    return E_OK;
}

uint32_t core_get_signature_status(const image_t *image)
{
    /* TODO: return bitmask of passed checks */
    (void)image;
    return 0;
}
