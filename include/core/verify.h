/**
 * @file core/verify.h
 * @brief Image verification service  [THREAD]
 *
 * Multi-stage verification: CRC → SHA256 → Signature → Version.
 */
#ifndef CORE_VERIFY_H
#define CORE_VERIFY_H

#include "types.h"

/** Verification method bitmask. */
typedef enum {
    VERIFY_CRC32   = 0x01,
    VERIFY_CRC64   = 0x02,
    VERIFY_SHA256  = 0x04,
    VERIFY_RSA     = 0x08,
    VERIFY_ECDSA   = 0x10,
    VERIFY_VERSION = 0x20,
    VERIFY_ALL     = 0x3F,
} verify_method_t;

/** Verification reference data embedded in / alongside the image. */
typedef struct {
    uint32_t crc32;
    uint64_t crc64;
    uint8_t  sha256[32];
    uint8_t  signature[256];
    uint32_t version;
    uint8_t  key_id;
    uint8_t  algo;           /**< 0 = RSA-2048, 1 = ECDSA-P256 */
} image_verify_data_t;

/** Describes an image to be verified. */
typedef struct {
    uint32_t              addr;        /**< Image address (Flash or RAM) */
    uint32_t              size;        /**< Image payload size */
    image_verify_data_t  *verify_data; /**< Reference checksums / sigs */
    verify_method_t       methods;     /**< Which checks to run */
} image_t;

/**
 * Run all requested verification checks on an image.
 * @return E_OK on full pass, E_VERIFY on any check failure, < 0 on error.
 */
int core_verify_image(const image_t *image);

/**
 * Check version against rollback threshold.
 * @return E_OK if acceptable, E_ROLLBACK if too low.
 */
int core_verify_version(uint32_t current_version, uint32_t min_allowed_version);

/**
 * Return a bitmask of which individual checks passed.
 */
uint32_t core_get_signature_status(const image_t *image);

#endif /* CORE_VERIFY_H */
