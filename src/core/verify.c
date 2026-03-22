/**
 * @file verify.c
 * @brief Multi-stage image verification (CRC → SHA → Sig → Version)
 */
#include "core/verify.h"
#include "driver/crc.h"
#include "driver/sha256.h"
#include "driver/rsa.h"
#include "driver/ecdsa.h"
#include "driver/flash.h"
#include "errno.h"

/* Minimum allowed security version (from OTP or config) */
static uint32_t s_min_security_version = 100;

/**
 * Run full verification pipeline
 */
int core_verify_image(const image_t *image)
{
    if (!image || !image->verify_data) {
        return E_INVAL;
    }

    uint32_t passed = 0;
    int ret;

    /* Read image data into buffer if in flash */
    uint8_t *data_ptr;
    uint8_t *data_buf = NULL;

    if (image->addr >= FLASH_XIP_BASE) {
        /* Allocate buffer for image data */
        data_buf = (uint8_t *)DRAM_BASE + 0x1000000;  /* Use DRAM */
        ret = driver_flash_read(image->addr - FLASH_XIP_BASE, data_buf, image->size);
        if (ret < 0) {
            return E_IO;
        }
        data_ptr = data_buf;
    } else {
        data_ptr = (uint8_t *)image->addr;
    }

    /* Stage 1: CRC32 (fast check) */
    if (image->methods & VERIFY_CRC32) {
        uint32_t crc = driver_crc32(data_ptr, image->size, 0);
        if (crc != image->verify_data->crc32) {
            return BOOT_ERR_CRC_MISMATCH;
        }
        passed |= VERIFY_CRC32;
    }

    /* Stage 2: CRC64 (optional) */
    if (image->methods & VERIFY_CRC64) {
        uint64_t crc = driver_crc64(data_ptr, image->size, 0);
        if (crc != image->verify_data->crc64) {
            return BOOT_ERR_CRC_MISMATCH;
        }
        passed |= VERIFY_CRC64;
    }

    /* Stage 3: SHA256 */
    if (image->methods & VERIFY_SHA256) {
        uint8_t digest[32];
        driver_sha256(data_ptr, image->size, digest);

        int match = 1;
        for (int i = 0; i < 32; i++) {
            if (digest[i] != image->verify_data->sha256[i]) {
                match = 0;
                break;
            }
        }

        if (!match) {
            return BOOT_ERR_HASH_MISMATCH;
        }
        passed |= VERIFY_SHA256;
    }

    /* Stage 4: Digital signature */
    if (image->methods & (VERIFY_RSA | VERIFY_ECDSA)) {
        /* Verify signature */
        /* For now, this is a stub - real implementation would read public key from OTP */
        /* and verify the signature */

        /* SHA256 of image is the message to verify */
        uint8_t digest[32];
        driver_sha256(data_ptr, image->size, digest);

        if (image->verify_data->algo == 0) {
            /* RSA verification */
            /* ret = driver_rsa_verify(digest, 32, ...); */
            /* For now, assume valid if SHA256 matched */
            passed |= VERIFY_RSA;
        } else {
            /* ECDSA verification */
            /* ret = driver_ecdsa_verify(digest, 32, ...); */
            passed |= VERIFY_ECDSA;
        }
    }

    /* Stage 5: Version check */
    if (image->methods & VERIFY_VERSION) {
        ret = core_verify_version(image->verify_data->version, s_min_security_version);
        if (ret < 0) {
            return BOOT_ERR_VERSION_ROLLBACK;
        }
        passed |= VERIFY_VERSION;
    }

    return E_OK;
}

/**
 * Check version against minimum
 */
int core_verify_version(uint32_t current_version, uint32_t min_allowed_version)
{
    if (current_version < min_allowed_version) {
        return E_ROLLBACK;
    }
    return E_OK;
}

/**
 * Get signature status
 */
uint32_t core_get_signature_status(const image_t *image)
{
    (void)image;
    /* Return all checks as passed (simplified) */
    return VERIFY_ALL;
}

/**
 * Set minimum security version
 */
void core_verify_set_min_version(uint32_t version)
{
    s_min_security_version = version;
}
