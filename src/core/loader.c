/**
 * @file loader.c
 * @brief Kernel / DTB image loading
 */
#include "core/loader.h"
#include "driver/flash.h"
#include "driver/crc.h"
#include "errno.h"
#include "config.h"
#include "utils/string.h"

static char s_bootargs[MAX_BOOTARGS_LEN] = {0};

/* Image header format (simplified) */
typedef struct {
    uint32_t magic;          /* Magic number */
    uint32_t size;           /* Image size */
    uint32_t load_addr;      /* Load address */
    uint32_t entry_addr;     /* Entry point */
    uint32_t crc32;          /* CRC32 */
    uint8_t  format;         /* Format type */
    uint8_t  compression;    /* Compression */
    uint16_t reserved;
} raw_image_header_t;

#define IMAGE_MAGIC  0x494D4749  /* "IMGI" */

int core_load_image(uint32_t flash_addr, image_header_t *header)
{
    int ret;
    raw_image_header_t raw_header;

    if (!header) return E_INVAL;

    /* Read image header from flash */
    ret = driver_flash_read(flash_addr, (uint8_t *)&raw_header, sizeof(raw_header));
    if (ret < 0) {
        return BOOT_ERR_IMAGE_LOAD;
    }

    /* Verify magic */
    if (raw_header.magic != IMAGE_MAGIC) {
        return BOOT_ERR_IMAGE_CORRUPT;
    }

    /* Fill output header */
    header->load_addr = raw_header.load_addr;
    header->entry_addr = raw_header.entry_addr;
    header->size = raw_header.size;
    header->format = (image_format_t)raw_header.format;
    header->compression = raw_header.compression;

    /* Read image data to load address */
    uint32_t payload_size = raw_header.size;
    uint32_t data_offset = flash_addr + sizeof(raw_image_header_t);

    ret = driver_flash_read(data_offset, (uint8_t *)(uintptr_t)header->load_addr, payload_size);
    if (ret < 0) {
        return BOOT_ERR_IMAGE_LOAD;
    }

    /* Verify CRC */
    uint32_t crc = driver_crc32((uint8_t *)(uintptr_t)header->load_addr, payload_size, 0);
    if (crc != raw_header.crc32) {
        return BOOT_ERR_IMAGE_CORRUPT;
    }

    return E_OK;
}

int core_load_dtb(uint32_t dtb_addr, uint32_t load_addr)
{
    int ret;
    uint32_t magic;

    /* Read DTB magic */
    ret = driver_flash_read(dtb_addr, (uint8_t *)&magic, sizeof(magic));
    if (ret < 0) {
        return BOOT_ERR_IMAGE_LOAD;
    }

    /* Verify DTB magic (0xD00DFEED) */
    if (magic != 0xD00DFEED) {
        return BOOT_ERR_DTB_INVALID;
    }

    /* Read DTB size */
    uint32_t size;
    ret = driver_flash_read(dtb_addr + 4, (uint8_t *)&size, sizeof(size));
    if (ret < 0) {
        return BOOT_ERR_IMAGE_LOAD;
    }

    /* Clamp size */
    if (size > DTB_MAX_SIZE) {
        size = DTB_MAX_SIZE;
    }

    /* Read entire DTB */
    ret = driver_flash_read(dtb_addr, (uint8_t *)(uintptr_t)load_addr, size);
    if (ret < 0) {
        return BOOT_ERR_IMAGE_LOAD;
    }

    return E_OK;
}

int core_set_bootargs(const char *key, const char *value)
{
    if (!key || !value) return E_INVAL;

    uint32_t current_len = bl_strlen(s_bootargs);
    uint32_t add_len = bl_strlen(key) + bl_strlen(value) + 2;  /* "key=value\0" */

    if (current_len + add_len >= MAX_BOOTARGS_LEN) {
        return E_NOMEM;
    }

    /* Append space if not empty */
    if (current_len > 0) {
        bl_strcat(s_bootargs, " ", MAX_BOOTARGS_LEN);
    }

    /* Append key=value */
    bl_strcat(s_bootargs, key, MAX_BOOTARGS_LEN);
    bl_strcat(s_bootargs, "=", MAX_BOOTARGS_LEN);
    bl_strcat(s_bootargs, value, MAX_BOOTARGS_LEN);

    return E_OK;
}

const char *core_get_bootargs(void)
{
    return s_bootargs;
}
