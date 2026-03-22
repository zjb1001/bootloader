/**
 * @file core/loader.h
 * @brief Kernel / DTB image loader  [THREAD]
 */
#ifndef CORE_LOADER_H
#define CORE_LOADER_H

#include "types.h"

typedef enum {
    IMAGE_FORMAT_RAW = 0,
    IMAGE_FORMAT_UIMAGE,
    IMAGE_FORMAT_FIT,
    IMAGE_FORMAT_ELF,
} image_format_t;

typedef struct {
    uint32_t       load_addr;    /**< Target RAM address */
    uint32_t       entry_addr;   /**< Execution entry point */
    uint32_t       size;         /**< Payload size */
    image_format_t format;
    uint8_t        compression;  /**< 0 = none, 1 = gzip, 2 = lz4 */
} image_header_t;

/** Load an image from Flash into RAM and parse its header. */
int core_load_image(uint32_t flash_addr, image_header_t *header);

/** Load a Device Tree Blob from Flash into RAM. */
int core_load_dtb(uint32_t dtb_addr, uint32_t load_addr);

/** Append a key=value pair to the kernel command line. */
int core_set_bootargs(const char *key, const char *value);

/** Get assembled boot command line. */
const char *core_get_bootargs(void);

#endif /* CORE_LOADER_H */
