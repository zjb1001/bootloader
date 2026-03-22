/**
 * @file string.c
 * @brief Minimal string/memory utilities (no libc dependency)
 */
#include "utils/string.h"
#include "types.h"

void *bl_memcpy(void *dst, const void *src, size_t n)
{
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    while (n--) {
        *d++ = *s++;
    }
    return dst;
}

void *bl_memset(void *s, int c, size_t n)
{
    uint8_t *p = (uint8_t *)s;
    while (n--) {
        *p++ = (uint8_t)c;
    }
    return s;
}

int bl_memcmp(const void *s1, const void *s2, size_t n)
{
    const uint8_t *a = (const uint8_t *)s1;
    const uint8_t *b = (const uint8_t *)s2;
    while (n--) {
        if (*a != *b) return *a - *b;
        a++; b++;
    }
    return 0;
}

size_t bl_strlen(const char *s)
{
    size_t len = 0;
    while (*s++) len++;
    return len;
}

int bl_strcmp(const char *s1, const char *s2)
{
    while (*s1 && (*s1 == *s2)) {
        s1++; s2++;
    }
    return *(const uint8_t *)s1 - *(const uint8_t *)s2;
}

char *bl_strncpy(char *dst, const char *src, size_t n)
{
    size_t i;
    for (i = 0; i < n && src[i]; i++) {
        dst[i] = src[i];
    }
    for (; i < n; i++) {
        dst[i] = '\0';
    }
    return dst;
}

char *bl_strcat(char *dst, const char *src, size_t maxlen)
{
    size_t dlen = bl_strlen(dst);
    size_t i;
    for (i = 0; src[i] && (dlen + i + 1) < maxlen; i++) {
        dst[dlen + i] = src[i];
    }
    dst[dlen + i] = '\0';
    return dst;
}

/* Standard C library names for bare-metal builds (-nostdlib) */
void *memcpy(void *dst, const void *src, size_t n)
{
    return bl_memcpy(dst, src, n);
}

void *memset(void *s, int c, size_t n)
{
    return bl_memset(s, c, n);
}

int memcmp(const void *s1, const void *s2, size_t n)
{
    return bl_memcmp(s1, s2, n);
}

size_t strlen(const char *s)
{
    return bl_strlen(s);
}
