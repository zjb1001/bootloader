/**
 * @file loader.c
 * @brief Kernel / DTB image loading
 */
#include "core/loader.h"
#include "driver/flash.h"
#include "errno.h"
#include "config.h"

static char s_bootargs[MAX_BOOTARGS_LEN];

int core_load_image(uint32_t flash_addr, image_header_t *header)
{
    /* TODO: read image header, copy payload to load_addr */
    (void)flash_addr;
    (void)header;
    return E_OK;
}

int core_load_dtb(uint32_t dtb_addr, uint32_t load_addr)
{
    /* TODO: read DTB from flash, verify magic, copy */
    (void)dtb_addr;
    (void)load_addr;
    return E_OK;
}

int core_set_bootargs(const char *key, const char *value)
{
    /* TODO: append key=value to s_bootargs */
    (void)key;
    (void)value;
    return E_OK;
}

const char *core_get_bootargs(void)
{
    return s_bootargs;
}
