/**
 * @file errno.h
 * @brief Unified error codes for the bootloader
 *
 * Convention:
 *  - int-returning functions: 0 = success, < 0 = error.
 *  - ssize_t read/write: >=0 = byte count, < 0 = error code.
 *  - ISR-safe functions may only return E_OK / E_INVAL / E_PERM.
 */
#ifndef BOOTLOADER_ERRNO_H
#define BOOTLOADER_ERRNO_H

/* ─── Generic error codes ─── */
#define E_OK            0    /**< Success */
#define E_INVAL        -1    /**< Invalid parameter */
#define E_IO           -2    /**< Device / IO error */
#define E_NOMEM        -3    /**< Static resource exhausted */
#define E_TIMEOUT      -4    /**< Operation timed out */
#define E_PERM         -5    /**< Not permitted in current state */
#define E_VERIFY       -6    /**< Verification failed (CRC/SHA/sig) */
#define E_ROLLBACK     -7    /**< Version rollback rejected */
#define E_NOTFOUND     -8    /**< Partition / image not found */

/* ─── Hardware errors (HAL layer) ─── */
#define BOOT_ERR_HW_NOT_READY      -10
#define BOOT_ERR_CLOCK_INIT        -11
#define BOOT_ERR_DRAM_INIT         -12
#define BOOT_ERR_DRAM_TEST         -13

/* ─── Flash / storage errors ─── */
#define BOOT_ERR_FLASH_INIT        -20
#define BOOT_ERR_FLASH_READ        -21
#define BOOT_ERR_FLASH_WRITE       -22
#define BOOT_ERR_FLASH_ERASE       -23
#define BOOT_ERR_BAD_BLOCK         -24

/* ─── Verification errors ─── */
#define BOOT_ERR_CRC_MISMATCH      -30
#define BOOT_ERR_HASH_MISMATCH     -31
#define BOOT_ERR_SIGNATURE_INVALID -32
#define BOOT_ERR_CERT_INVALID      -33
#define BOOT_ERR_VERSION_ROLLBACK  -34

/* ─── Image errors ─── */
#define BOOT_ERR_IMAGE_NOT_FOUND   -40
#define BOOT_ERR_IMAGE_CORRUPT     -41
#define BOOT_ERR_IMAGE_LOAD        -42
#define BOOT_ERR_DTB_INVALID       -43

/* ─── Partition errors ─── */
#define BOOT_ERR_PARTITION_ERROR   -50
#define BOOT_ERR_PARTITION_TABLE   -51
#define BOOT_ERR_NO_VALID_BOOT     -52

/* ─── System errors ─── */
#define BOOT_ERR_WATCHDOG          -60
#define BOOT_ERR_EXCEPTION         -61
#define BOOT_ERR_SECURE_BOOT       -62

#endif /* BOOTLOADER_ERRNO_H */
