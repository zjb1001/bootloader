/**
 * @file math.c
 * @brief Minimal math utilities
 *
 * Provides division/modulo helpers for platforms without hardware divider.
 * Also includes common operations like alignment, min/max, etc.
 */
#include "types.h"

/**
 * Divide two unsigned 32-bit integers.
 * Returns quotient and stores remainder in *rem if non-NULL.
 */
uint32_t bl_divmod_u32(uint32_t dividend, uint32_t divisor, uint32_t *rem)
{
    if (divisor == 0) {
        /* Division by zero - return max value as error indicator */
        if (rem) *rem = 0;
        return 0xFFFFFFFFu;
    }

    if (rem) *rem = 0;

    /* Use built-in if available (most modern compilers) */
#if defined(__ARM_ARCH) && (__ARM_ARCH < 7)
    /* Software division for ARMv6 and earlier */
    uint32_t quotient = 0;
    uint32_t remainder = 0;

    for (int i = 31; i >= 0; i--) {
        remainder = (remainder << 1) | ((dividend >> i) & 1);
        if (remainder >= divisor) {
            remainder -= divisor;
            quotient |= (1u << i);
        }
    }

    if (rem) *rem = remainder;
    return quotient;
#else
    /* Hardware division available */
    uint32_t quotient = dividend / divisor;
    if (rem) *rem = dividend - (quotient * divisor);
    return quotient;
#endif
}

/**
 * Signed 32-bit division.
 */
int32_t bl_div_s32(int32_t dividend, int32_t divisor)
{
    /* Handle special cases */
    if (divisor == 0) return 0x7FFFFFFF;  /* Max positive as error */
    if (dividend == 0) return 0;
    if (dividend == INT32_MIN && divisor == -1) return INT32_MIN;  /* Overflow */

    uint32_t dividend_abs = (dividend < 0) ? -(uint32_t)dividend : dividend;
    uint32_t divisor_abs = (divisor < 0) ? -(uint32_t)divisor : divisor;
    uint32_t quotient_abs = dividend_abs / divisor_abs;

    /* Apply sign */
    return ((dividend < 0) ^ (divisor < 0)) ? -(int32_t)quotient_abs : (int32_t)quotient_abs;
}

/**
 * Count leading zeros in a 32-bit value.
 */
int clz_u32(uint32_t x)
{
    if (x == 0) return 32;

#if defined(__GNUC__) || defined(__clang__)
    return __builtin_clz(x);
#else
    int n = 0;
    if ((x & 0xFFFF0000) == 0) { n += 16; x <<= 16; }
    if ((x & 0xFF000000) == 0) { n += 8;  x <<= 8; }
    if ((x & 0xF0000000) == 0) { n += 4;  x <<= 4; }
    if ((x & 0xC0000000) == 0) { n += 2;  x <<= 2; }
    if ((x & 0x80000000) == 0) { n += 1; }
    return n;
#endif
}

/**
 * Count trailing zeros in a 32-bit value.
 */
int ctz_u32(uint32_t x)
{
    if (x == 0) return 32;

#if defined(__GNUC__) || defined(__clang__)
    return __builtin_ctz(x);
#else
    int n = 0;
    if ((x & 0x0000FFFF) == 0) { n += 16; x >>= 16; }
    if ((x & 0x000000FF) == 0) { n += 8;  x >>= 8; }
    if ((x & 0x0000000F) == 0) { n += 4;  x >>= 4; }
    if ((x & 0x00000003) == 0) { n += 2;  x >>= 2; }
    if ((x & 0x00000001) == 0) { n += 1; }
    return n;
#endif
}

/**
 * Round up to next power of 2.
 * Returns 0 for x > 0x80000000.
 */
uint32_t round_up_pow2(uint32_t x)
{
    if (x == 0) return 1;
    if (x > 0x80000000u) return 0;

    x--;
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    x++;

    return x;
}

/**
 * Check if value is a power of 2.
 */
int is_pow2(uint32_t x)
{
    return (x != 0) && ((x & (x - 1)) == 0);
}

/**
 * Align value up to alignment (must be power of 2).
 */
uint32_t align_up(uint32_t value, uint32_t alignment)
{
    return (value + alignment - 1) & ~(alignment - 1);
}

/**
 * Align value down to alignment.
 */
uint32_t align_down(uint32_t value, uint32_t alignment)
{
    return value & ~(alignment - 1);
}

/**
 * Byte swap 16-bit value (big endian <-> little endian).
 */
uint16_t bswap_16(uint16_t x)
{
    return (x << 8) | (x >> 8);
}

/**
 * Byte swap 32-bit value.
 */
uint32_t bswap_32(uint32_t x)
{
    return ((x & 0xFF) << 24) |
           ((x & 0xFF00) << 8) |
           ((x & 0xFF0000) >> 8) |
           ((x & 0xFF000000) >> 24);
}

/**
 * Convert from big endian to host byte order.
 */
uint16_t be16_to_cpu(uint16_t x)
{
#if defined(__ORDER_BIG_ENDIAN__)
    return x;
#else
    return bswap_16(x);
#endif
}

uint32_t be32_to_cpu(uint32_t x)
{
#if defined(__ORDER_BIG_ENDIAN__)
    return x;
#else
    return bswap_32(x);
#endif
}

/**
 * Convert from host byte order to big endian.
 */
uint16_t cpu_to_be16(uint16_t x)
{
    return be16_to_cpu(x);
}

uint32_t cpu_to_be32(uint32_t x)
{
    return be32_to_cpu(x);
}

/**
 * Convert from little endian to host byte order.
 */
uint16_t le16_to_cpu(uint16_t x)
{
#if defined(__ORDER_LITTLE_ENDIAN__)
    return x;
#else
    return bswap_16(x);
#endif
}

uint32_t le32_to_cpu(uint32_t x)
{
#if defined(__ORDER_LITTLE_ENDIAN__)
    return x;
#else
    return bswap_32(x);
#endif
}

/**
 * Convert from host byte order to little endian.
 */
uint16_t cpu_to_le16(uint16_t x)
{
    return le16_to_cpu(x);
}

uint32_t cpu_to_le32(uint32_t x)
{
    return le32_to_cpu(x);
}

/**
 * Integer square root (Newton's method).
 */
uint32_t isqrt_u32(uint32_t x)
{
    if (x == 0) return 0;

    uint32_t res = 0;
    uint32_t bit = 1u << 30;  /* Highest power of 4 <= x */

    while (bit > x) {
        bit >>= 2;
    }

    while (bit != 0) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }

    return res;
}

/**
 * Calculate average of two values without overflow.
 */
uint32_t average_u32(uint32_t a, uint32_t b)
{
    return (a & b) + ((a ^ b) >> 1);
}
