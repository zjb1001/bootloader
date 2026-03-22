/**
 * @file mock_hal.c
 * @brief Mock HAL/Driver/Core layer for host-side testing
 */
#include "types.h"
#include "errno.h"
#include <stdio.h>
#include <string.h>

/* Mock state that tests can inspect / configure */
static uint32_t mock_timer_ms = 0;

void mock_hal_set_timer_ms(uint32_t ms)  { mock_timer_ms = ms; }
uint32_t mock_hal_get_timer_ms(void)     { return mock_timer_ms; }

/* ─── HAL UART ─── */
int hal_uart_init(uint32_t id, const void *cfg) { (void)id; (void)cfg; return 0; }
int hal_uart_write(uint32_t id, const uint8_t *data, uint32_t len)
{
    (void)id;
    for (uint32_t i = 0; i < len; i++) putchar(data[i]);
    return (int)len;
}

/* ─── HAL Clock ─── */
int hal_clock_init(const void *cfg) { (void)cfg; return 0; }
int hal_clock_get_freq(uint32_t id) { (void)id; return 1200; }

/* ─── HAL Memory ─── */
int hal_mmu_init(void) { return 0; }
int hal_mmu_enable(void) { return 0; }
int hal_mmu_disable(void) { return 0; }
int hal_cache_enable(uint32_t l) { (void)l; return 0; }
int hal_cache_disable(uint32_t l) { (void)l; return 0; }
int hal_cache_flush(uint32_t l) { (void)l; return 0; }
int hal_dram_init(uint32_t sz) { (void)sz; return 0; }
int hal_dram_test(uint32_t start, uint32_t sz) { (void)start; (void)sz; return 0; }

/* ─── HAL IRQ ─── */
int hal_irq_init(void) { return 0; }
uint32_t hal_irq_disable_all(void) { return 0; }

/* ─── HAL Timer ─── */
int hal_timer_init(void) { return 0; }

/* ─── Console ─── */
void console_init(uint32_t baud) { (void)baud; }
void console_puts(const char *s) { if (s) fputs(s, stdout); }
int  console_gets(char *buf, uint32_t max) { if (buf && max) buf[0] = '\0'; return 0; }

/* ─── Watchdog ─── */
int  driver_watchdog_init(uint32_t ms)   { (void)ms; return 0; }
int  driver_watchdog_feed(void)          { return 0; }
int  driver_watchdog_disable(void)       { return 0; }
void driver_watchdog_force_reset(void)   { /* no-op */ }

/* ─── Flash ─── */
extern int mock_flash_read_raw(uint32_t, uint8_t *, uint32_t);
int driver_flash_init(void)              { return 0; }
int driver_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    return mock_flash_read_raw(addr, buf, len);
}
int driver_flash_get_info(void *info)    { (void)info; return 0; }

/* ─── Core Boot ─── */
int  core_boot_check(void)              { return 0; }
void core_boot_kernel(const void *p)    { (void)p; }

/* ─── Crypto stubs (RSA/ECDSA not yet fully implemented) ─── */
int driver_rsa_verify(const uint8_t *data, uint32_t len,
                      const uint8_t *sig, uint32_t sig_len,
                      const void *key)
{
    (void)data; (void)len; (void)sig; (void)sig_len; (void)key;
    return 0;  /* pass */
}

int driver_ecdsa_verify(const uint8_t *data, uint32_t len,
                        const uint8_t *sig, uint32_t sig_len,
                        const void *key)
{
    (void)data; (void)len; (void)sig; (void)sig_len; (void)key;
    return 0;  /* pass */
}
