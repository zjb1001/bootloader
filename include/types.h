/**
 * @file types.h
 * @brief Common type definitions for the bootloader
 *
 * Provides fixed-width integer types and common type aliases.
 * All modules MUST use these types instead of built-in C types
 * to ensure portability across ARM and x86 targets.
 */
#ifndef BOOTLOADER_TYPES_H
#define BOOTLOADER_TYPES_H

#include <stdint.h>
#include <stddef.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

/* Convenience aliases */
typedef uint8_t   u8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef uint64_t  u64;
typedef int8_t    s8;
typedef int16_t   s16;
typedef int32_t   s32;
typedef int64_t   s64;

/* Register-width type (matches platform word size) */
typedef unsigned long uintptr;

/* Physical / virtual address types */
typedef u32 paddr_t;
typedef u32 vaddr_t;

#endif /* BOOTLOADER_TYPES_H */
