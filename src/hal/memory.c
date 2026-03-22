/**
 * @file memory.c
 * @brief MMU, cache, and DRAM HAL implementation
 *
 * Provides MMU page table management, cache control, and DRAM initialization
 * for ARMv7-A platforms.
 */
#include "hal/memory.h"
#include "platform/platform.h"
#include "errno.h"
#include "types.h"

/* Page table location (aligned to 16KB) */
static uint32_t s_page_table[4096] __attribute__((aligned(16384)));

/* ARMv7-A System Control Register bits */
#define SCTLR_M      (1u << 0)   /* MMU enable */
#define SCTLR_A      (1u << 1)   /* Alignment check */
#define SCTLR_C      (1u << 2)   /* Data cache enable */
#define SCTLR_Z      (1u << 11)  /* Branch prediction */
#define SCTLR_I      (1u << 12)  /* Instruction cache enable */
#define SCTLR_V      (1u << 13)  /* Vectors relocated to 0xFFFF0000 */
#define SCTLR_RR     (1u << 14)  /* Round Robin select */

/* Level 1 page table descriptor bits (Section mapping) */
#define L1_DESC_SEC           0x2   /* Section descriptor */
#define L1_DESC_SEC_AP_SHIFT  10    /* Access permission bits */
#define L1_DESC_SEC_TEX_SHIFT 12    /* Type extension */
#define L1_DESC_SEC_DOMAIN    0     /* Domain 0 */
#define L1_DESC_SEC_C         (1u << 3)  /* Cacheable */
#define L1_DESC_SEC_B         (1u << 2)  /* Bufferable */
#define L1_DESC_SEC_XN        (1u << 4)  /* Execute never */

/* Cache registers */
#define CSSELR_CSSELR_INDEX   0x0
#define CCSIDR_LINESIZE_SHIFT 0
#define CCSIDR_ASSOC_SHIFT    3
#define CCSIDR_NUMSETS_SHIFT  13
#define CCSIDR_NUMSETS_MASK   0x7FFFu

/* Co-Processor access */
#define mrc(p, op1, crn, crm, op2, val) \
    __asm__ volatile("mrc p" #p ", " #op1 ", %0, c" #crn ", c" #crm ", " #op2 : "=r"(val))
#define mcr(p, op1, crn, crm, op2, val) \
    __asm__ volatile("mcr p" #p ", " #op1 ", %0, c" #crn ", c" #crm ", " #op2 :: "r"(val))

/**
 * Read SCTLR
 */
static inline uint32_t read_sctlr(void)
{
    uint32_t val;
    mrc(15, 0, 1, 0, 0, val);
    return val;
}

/**
 * Write SCTLR
 */
static inline void write_sctlr(uint32_t val)
{
    mcr(15, 0, 1, 0, 0, val);
    __asm__ volatile("isb");
}

/**
 * Read TTBR0 (Translation Table Base Register 0)
 */
static inline uint32_t read_ttbr0(void)
{
    uint32_t val;
    mrc(15, 0, 2, 0, 0, val);
    return val;
}

/**
 * Write TTBR0
 */
static inline void write_ttbr0(uint32_t val)
{
    mcr(15, 0, 2, 0, 0, val);
    __asm__ volatile("isb");
}

/**
 * Invalidate entire TLB
 */
static inline void tlbiall(void)
{
    __asm__ volatile("mcr p15, 0, %0, c8, c7, 0" :: "r"(0));
    __asm__ volatile("dsb");
    __asm__ volatile("isb");
}

/**
 * Initialize MMU with identity mapping
 */
int hal_mmu_init(void)
{
    uint32_t i;

    /* Clear page table */
    for (i = 0; i < sizeof(s_page_table) / sizeof(uint32_t); i++) {
        s_page_table[i] = 0;
    }

    /* Build identity-mapped page table (Section entries, 1MB each) */
    for (i = 0; i < 4096; i++) {
        uint32_t desc = L1_DESC_SEC;

        /* Physical address = virtual address (identity mapping) */
        desc |= (i << 20);

        /* Access permissions: PL1 RW, PL0 none */
        desc |= (0x3 << L1_DESC_SEC_AP_SHIFT);

        /* Memory attributes */
        if (i < 256) {  /* First 256MB - device memory */
            desc |= (1u << L1_DESC_SEC_TEX_SHIFT);  /* TEX[0] = 1 */
        } else {  /* DRAM - normal memory, cacheable */
            desc |= L1_DESC_SEC_C | L1_DESC_SEC_B;
            desc |= (1u << L1_DESC_SEC_TEX_SHIFT);  /* TEX[0] = 1 */
        }

        s_page_table[i] = desc;
    }

    /* Set page table base address */
    write_ttbr0((uint32_t)s_page_table);

    return E_OK;
}

/**
 * Enable MMU
 */
int hal_mmu_enable(void)
{
    uint32_t sctlr = read_sctlr();
    sctlr |= SCTLR_M;      /* Enable MMU */
    sctlr |= SCTLR_C;      /* Enable data cache */
    sctlr |= SCTLR_I;      /* Enable instruction cache */
    sctlr |= SCTLR_Z;      /* Enable branch prediction */
    sctlr &= ~SCTLR_A;     /* Disable alignment check (for performance) */
    sctlr &= ~SCTLR_V;     /* Vectors at 0x00000000 */

    write_sctlr(sctlr);

    return E_OK;
}

/**
 * Disable MMU
 */
int hal_mmu_disable(void)
{
    uint32_t sctlr = read_sctlr();
    sctlr &= ~SCTLR_M;
    sctlr &= ~SCTLR_C;
    sctlr &= ~SCTLR_I;
    write_sctlr(sctlr);

    /* Invalidate TLB */
    tlbiall();

    return E_OK;
}

/**
 * Map a memory region (simplified - doesn't split existing sections)
 */
int hal_mmu_map(const mmu_entry_t *entry)
{
    if (!entry || !entry->virt_addr || !entry->phys_addr) return E_INVAL;

    uint32_t virt_base = entry->virt_addr >> 20;
    uint32_t phys_base = entry->phys_addr >> 20;
    uint32_t num_sections = (entry->size + 0xFFFFF) >> 20;

    for (uint32_t i = 0; i < num_sections; i++) {
        uint32_t desc = L1_DESC_SEC;
        desc |= ((phys_base + i) << 20);

        /* Set attributes */
        if (entry->attr & MMU_ATTR_WRITE) {
            desc |= (0x3 << L1_DESC_SEC_AP_SHIFT);  /* RW */
        } else {
            desc |= (0x1 << L1_DESC_SEC_AP_SHIFT);  /* RO */
        }

        if (entry->attr & MMU_ATTR_EXEC) {
            /* Executable - don't set XN */
        } else {
            desc |= L1_DESC_SEC_XN;  /* No execute */
        }

        if (entry->attr & MMU_ATTR_DEVICE) {
            desc |= (1u << L1_DESC_SEC_TEX_SHIFT);
        } else {
            desc |= L1_DESC_SEC_C | L1_DESC_SEC_B;
            desc |= (1u << L1_DESC_SEC_TEX_SHIFT);
        }

        s_page_table[virt_base + i] = desc;
    }

    return E_OK;
}

/**
 * Flush entire TLB
 */
int hal_mmu_tlb_flush(void)
{
    tlbiall();
    return E_OK;
}

/**
 * Enable cache level
 */
int hal_cache_enable(uint32_t level)
{
    if (level > 1) return E_INVAL;

    /* Caches are enabled via SCTLR - see hal_mmu_enable */
    (void)level;
    return E_OK;
}

/**
 * Disable cache level
 */
int hal_cache_disable(uint32_t level)
{
    uint32_t sctlr = read_sctlr();

    if (level == 0) {
        /* Disable instruction cache */
        sctlr &= ~SCTLR_I;
    } else {
        /* Disable data cache */
        sctlr &= ~SCTLR_C;
    }

    write_sctlr(sctlr);
    return E_OK;
}

/**
 * Flush cache by set/way
 */
int hal_cache_flush(uint32_t level)
{
    uint32_t csselr, ccsidr, sets, ways, way, set, line_size;

    (void)level;  /* Flush all levels */

    /* Select data cache */
    csselr = 0;
    mcr(15, 2, 0, 0, 0, csselr);
    __asm__ volatile("isb");

    /* Read cache characteristics */
    mrc(15, 1, 0, c0, c0, ccsidr);
    line_size = 4 << ((ccsidr >> CCSIDR_LINESIZE_SHIFT) & 0x7);
    sets = ((ccsidr >> CCSIDR_NUMSETS_SHIFT) & CCSIDR_NUMSETS_MASK) + 1;
    ways = ((ccsidr >> CCSIDR_ASSOC_SHIFT) & 0x3FF) + 1;

    /* Flush by set/way */
    for (way = 0; way < ways; way++) {
        for (set = 0; set < sets; set++) {
            uint32_t val = (way << 3) | (set << (32 - 10)) | (1);
            __asm__ volatile("mcr p15, 0, %0, c7, c10, 2" :: "r"(val));
        }
    }

    __asm__ volatile("dsb");
    __asm__ volatile("isb");

    return E_OK;
}

/**
 * Initialize DRAM
 */
int hal_dram_init(uint32_t dram_size)
{
    /* DRAM initialization is highly platform-specific */
    /* This is a placeholder that would call platform-specific code */

    /* TODO: DDR PHY training */
    /* TODO: Set timing parameters based on DRAM type */
    /* TODO: Configure memory controller */

    (void)dram_size;
    return E_OK;
}

/**
 * Test DRAM with walking pattern
 */
int hal_dram_test(uint32_t start, uint32_t size)
{
    volatile uint32_t *ptr = (volatile uint32_t *)start;
    uint32_t words = size / sizeof(uint32_t);
    uint32_t i;
    uint32_t expected;
    uint32_t pattern[] = {0x55555555, 0xAAAAAAAA, 0xFFFFFFFF, 0x00000000};

    /* Test first and last 1KB */
    uint32_t test_words = (size > 2048) ? 1024 / sizeof(uint32_t) : words;

    /* Write pattern 1 */
    for (i = 0; i < test_words; i++) {
        ptr[i] = pattern[0];
    }
    if (words > test_words) {
        for (i = words - test_words; i < words; i++) {
            ptr[i] = pattern[0];
        }
    }

    /* Read back and verify */
    for (i = 0; i < test_words; i++) {
        if (ptr[i] != pattern[0]) return E_IO;
    }
    if (words > test_words) {
        for (i = words - test_words; i < words; i++) {
            if (ptr[i] != pattern[0]) return E_IO;
        }
    }

    /* Test with other patterns */
    for (int p = 1; p < 4; p++) {
        for (i = 0; i < test_words; i++) {
            ptr[i] = pattern[p];
        }
        for (i = 0; i < test_words; i++) {
            if (ptr[i] != pattern[p]) return E_IO;
        }
    }

    /* Clear memory */
    for (i = 0; i < test_words; i++) {
        ptr[i] = 0;
    }

    return E_OK;
}
