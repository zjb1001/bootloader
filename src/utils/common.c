/**
 * @file common.c
 * @brief Miscellaneous common helpers
 */
#include "types.h"
#include "hal/uart.h"
#include "config.h"
#include "errno.h"

/* Hex dump - display memory in hexadecimal format */
void bl_hexdump(const void *data, uint32_t len)
{
    const uint8_t *bytes = (const uint8_t *)data;
    char line[128];
    uint32_t offset = 0;
    extern int hal_uart_write(uint32_t, const uint8_t *, uint32_t);

    while (offset < len) {
        int pos = 0;

        /* Address offset */
        line[pos++] = '0';
        line[pos++] = 'x';
        for (int i = 28; i >= 0; i -= 4) {
            uint8_t nibble = (offset >> i) & 0xF;
            line[pos++] = nibble < 10 ? '0' + nibble : 'a' + nibble - 10;
        }
        line[pos++] = ':';
        line[pos++] = ' ';

        /* Hex bytes */
        uint32_t line_bytes = (len - offset > 16) ? 16 : (len - offset);
        for (uint32_t i = 0; i < line_bytes; i++) {
            uint8_t b = bytes[offset + i];
            line[pos++] = "0123456789abcdef"[b >> 4];
            line[pos++] = "0123456789abcdef"[b & 0xF];
            line[pos++] = ' ';
            if (i == 7) line[pos++] = ' ';
        }

        /* Pad remaining hex positions */
        for (uint32_t i = line_bytes; i < 16; i++) {
            line[pos++] = ' ';
            line[pos++] = ' ';
            line[pos++] = ' ';
            if (i == 7) line[pos++] = ' ';
        }

        line[pos++] = ' ';
        line[pos++] = '|';

        /* ASCII representation */
        for (uint32_t i = 0; i < line_bytes; i++) {
            uint8_t c = bytes[offset + i];
            line[pos++] = (c >= 32 && c <= 126) ? c : '.';
        }

        line[pos++] = '|';
        line[pos++] = '\r';
        line[pos++] = '\n';

        hal_uart_write(DEFAULT_UART_ID, (const uint8_t *)line, pos);
        offset += line_bytes;
    }
}

/* Delay loop - simple busy wait */
void bl_udelay(uint32_t us)
{
    /* Very rough approximation - assumes ~1.2GHz CPU */
    /* This should be calibrated per platform */
    volatile uint32_t count = us * 1200 / 4;
    while (count--) {
        __asm__ volatile ("nop");
    }
}

void bl_mdelay(uint32_t ms)
{
    bl_udelay(ms * 1000);
}

/* Calculate CRC-8 (Maxim/Dallas variant) */
uint8_t crc8_maxim(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0x00;

    while (len--) {
        uint8_t byte = *data++;
        for (int i = 0; i < 8; i++) {
            uint8_t fb = (byte ^ crc) & 0x01;
            crc >>= 1;
            byte >>= 1;
            if (fb) crc ^= 0x8C;  /* x^8 + x^5 + x^4 + 1 */
        }
    }

    return crc;
}

/* Simple checksum - sum all bytes and return two's complement */
uint8_t checksum8(const uint8_t *data, uint32_t len)
{
    uint32_t sum = 0;

    while (len--) {
        sum += *data++;
    }

    /* Two's complement */
    return -(uint8_t)sum;
}

/* Check if pointer is aligned to boundary */
int is_aligned(const void *ptr, uint32_t alignment)
{
    return ((uintptr)ptr & (alignment - 1)) == 0;
}

/* Find first set bit in a 32-bit value (1-based index, 0 if none) */
int ffs_u32(uint32_t x)
{
    if (x == 0) return 0;

#if defined(__GNUC__) || defined(__clang__)
    return __builtin_ffs(x);
#else
    int n = 1;
    if ((x & 0xFFFF) == 0) { n += 16; x >>= 16; }
    if ((x & 0xFF) == 0) { n += 8; x >>= 8; }
    if ((x & 0xF) == 0) { n += 4; x >>= 4; }
    if ((x & 0x3) == 0) { n += 2; x >>= 2; }
    if ((x & 0x1) == 0) { n += 1; }
    return n;
#endif
}

/* Parse integer from string */
int atoi_simple(const char *str)
{
    int value = 0;
    int sign = 1;

    /* Skip whitespace */
    while (*str == ' ' || *str == '\t') str++;

    /* Handle sign */
    if (*str == '-') {
        sign = -1;
        str++;
    } else if (*str == '+') {
        str++;
    }

    /* Convert digits */
    while (*str >= '0' && *str <= '9') {
        value = value * 10 + (*str - '0');
        str++;
    }

    return sign * value;
}

/* Compare memory with tolerance (for fuzzy matching) */
int memcmp_tolerance(const uint8_t *a, const uint8_t *b, uint32_t len, uint32_t tolerance)
{
    uint32_t diff = 0;

    for (uint32_t i = 0; i < len; i++) {
        if (a[i] != b[i]) {
            uint32_t d = (a[i] > b[i]) ? (a[i] - b[i]) : (b[i] - a[i]);
            diff += d;
            if (diff > tolerance) return (int)diff;
        }
    }

    return (int)diff;
}

/* Reverse byte order in a buffer */
void memreverse(void *ptr, uint32_t len)
{
    uint8_t *p = (uint8_t *)ptr;
    uint8_t *end = p + len - 1;

    while (p < end) {
        uint8_t tmp = *p;
        *p = *end;
        *end = tmp;
        p++;
        end--;
    }
}
