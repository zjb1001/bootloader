/**
 * @file utils/string.h
 * @brief Minimal string/memory utilities (no libc dependency)
 */
#ifndef UTILS_STRING_H
#define UTILS_STRING_H

#include "types.h"

void   *bl_memcpy(void *dst, const void *src, size_t n);
void   *bl_memset(void *s, int c, size_t n);
int     bl_memcmp(const void *s1, const void *s2, size_t n);
size_t  bl_strlen(const char *s);
int     bl_strcmp(const char *s1, const char *s2);
char   *bl_strncpy(char *dst, const char *src, size_t n);
char   *bl_strcat(char *dst, const char *src, size_t maxlen);

#endif /* UTILS_STRING_H */
