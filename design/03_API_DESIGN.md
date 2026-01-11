# Bootloader API抽象设计

**版本**: 1.0  
**日期**: 2026年1月  
**分类**: API接口规范

---

## 通用约束与快速指引
- 语言/依赖：C99；禁止动态分配；避免 heavy libc；使用 stdint 定长类型。
- 错误码：统一 errno.h，返回值 <0 表示错误；size_t/ssize_t 接口错误时返回负值映射。
- 上下文：标注可在中断/线程上下文调用的接口；中断上下文禁止阻塞/长耗时。
- 日志：可编译开关，生产默认最小化；调试日志受安全策略控制。
- 配置：平台差异集中 platform/；Kconfig/CMake 选项与板级配置解耦。
- 最小调用序列（示例）：`hal_clock_init → hal_uart_init → driver_flash_init → verify_image → load_image → boot_start_kernel`。

---

## 0. 统一错误码规范（建议）

| 名称 | 值 | 含义 |
|------|----|------|
| E_OK | 0 | 成功 |
| E_INVAL | -1 | 参数无效 |
| E_IO | -2 | 设备/IO错误 |
| E_NOMEM | -3 | 资源不足（仅静态资源）|
| E_TIMEOUT | -4 | 超时 |
| E_PERM | -5 | 权限/状态不允许 |
| E_VERIFY | -6 | 校验失败（CRC/SHA/签名）|
| E_ROLLBACK | -7 | 版本回滚拒绝 |
| E_NOTFOUND | -8 | 分区/镜像不存在 |

返回约定：
- `int` 型函数返回上述错误码；`size_t/ssize_t` 型读写函数错误返回负值（转换成 `int` 错码）。
- 中断上下文函数仅返回 `E_OK` 或快速错误（`E_INVAL`/`E_PERM`）。

---

## 0.1 上下文可用性注记（API级标识）
- [ISR-safe]：允许在中断上下文调用（不阻塞、可重入）。
- [THREAD]：仅线程上下文调用（可能阻塞）。
- [INIT]：仅初始化阶段调用（Stage2）。

示例：`hal_irq_enable` [ISR-safe]；`driver_flash_read` [THREAD]；`hal_dram_init` [INIT]。

---

## 0.2 最小可用示例（片段）

```c
// 示例：加载并启动内核（线程上下文）
int boot_sequence(void) {
    clock_config_t cfg = { .core_freq=1200, .ahb_freq=400, .apb_freq=200, .dram_freq=400 };
    if (hal_clock_init(&cfg) < 0) return E_INVAL;
    if (driver_flash_init() < 0) return E_IO;

    // 定位分区并加载镜像（伪代码）
    image_t img; partition_t part;
    if (core_partition_find("kernel", &part) < 0) return E_NOTFOUND;
    if (core_image_load(&part, &img) < 0) return E_IO;

    // 校验（CRC→SHA→签名→版本）
    if (core_verify_crc(&img) < 0) return E_VERIFY;
    if (core_verify_sha256(&img) < 0) return E_VERIFY;
    if (core_verify_signature(&img) < 0) return E_VERIFY;
    if (core_verify_version(&img) < 0) return E_ROLLBACK;

    // 传递 DTB/参数并跳转
    core_prepare_kernel_params(&img);
    return boot_start_kernel();
}
```

---

## 1. API分层架构

```
┌─────────────────────────────────────┐
│      Core Bootstrap API             │
│  boot_start_kernel(), ...           │
└─────────────────┬───────────────────┘
                  │
┌─────────────────▼───────────────────┐
│   Service Layer API                 │
│  verify_image(), load_image(), ...  │
└─────────────────┬───────────────────┘
                  │
┌─────────────────▼───────────────────┐
│   Driver Layer API                  │
│  flash_read(), crc32(), sha256()... │
└─────────────────┬───────────────────┘
                  │
┌─────────────────▼───────────────────┐
│   HAL Layer API                     │
│  gpio_set(), clock_init(), ...      │
└─────────────────┬───────────────────┘
                  │
┌─────────────────▼───────────────────┐
│   Hardware (Registers, CPU, etc.)   │
└─────────────────────────────────────┘
```

---

## 2. 硬件抽象层（HAL）API

### 2.1 时钟管理
```c
/**
 * hal/clock.h - 时钟管理接口
 */

typedef enum {
    CLOCK_OSC_24M = 0,      // 外部振荡器 24MHz
    CLOCK_OSC_32K,          // 低速振荡器 32kHz
    CLOCK_PLL_CORE,         // 内核PLL
    CLOCK_PLL_DDR,          // DDR PLL
} clock_source_t;

typedef struct {
    uint32_t core_freq;     // 内核频率 (MHz)
    uint32_t ahb_freq;      // AHB频率 (MHz)
    uint32_t apb_freq;      // APB频率 (MHz)
    uint32_t dram_freq;     // DRAM频率 (MHz)
} clock_config_t;

/**
 * 初始化系统时钟
 * @param cfg: 时钟配置
 * @return: 0-成功, <0-失败
 */
int hal_clock_init(const clock_config_t *cfg);

/**
 * 获取当前时钟频率
 * @param src: 时钟源
 * @return: 频率(MHz) 或 <0表示错误
 */
int hal_clock_get_freq(clock_source_t src);

/**
 * 设置分频系数
 * @param domain: 时钟域 (CORE/AHB/APB/DRAM)
 * @param divisor: 分频值
 * @return: 0-成功, <0-失败
 */
int hal_clock_set_divisor(uint32_t domain, uint32_t divisor);

/**
 * 使能/禁用外设时钟
 * @param periph_id: 外设ID
 * @param enable: 1-使能, 0-禁用
 * @return: 0-成功, <0-失败
 */
int hal_clock_enable_periph(uint32_t periph_id, uint8_t enable);
```

### 2.2 GPIO管理
```c
/**
 * hal/gpio.h - GPIO接口
 */

typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALT_FUNC,
} gpio_mode_t;

typedef enum {
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
} gpio_pull_t;

/**
 * 初始化GPIO引脚
 * @param pin: 引脚号
 * @param mode: 模式 (输入/输出/复用)
 * @param pull: 上下拉配置
 * @return: 0-成功, <0-失败
 */
int hal_gpio_init(uint32_t pin, gpio_mode_t mode, gpio_pull_t pull);

/**
 * 设置GPIO输出电平
 * @param pin: 引脚号
 * @param value: 0或1
 * @return: 0-成功, <0-失败
 */
int hal_gpio_set(uint32_t pin, uint8_t value);

/**
 * 读取GPIO输入电平
 * @param pin: 引脚号
 * @return: 0/1-引脚电平, <0-失败
 */
int hal_gpio_get(uint32_t pin);

/**
 * 配置GPIO复用功能
 * @param pin: 引脚号
 * @param func: 复用功能号
 * @return: 0-成功, <0-失败
 */
int hal_gpio_set_alt_func(uint32_t pin, uint32_t func);
```

### 2.3 UART接口
```c
/**
 * hal/uart.h - 串口接口
 */

typedef struct {
    uint32_t baudrate;      // 波特率
    uint8_t data_bits;      // 数据位 (5-8)
    uint8_t stop_bits;      // 停止位 (1-2)
    uint8_t parity;         // 奇偶校验 (0-无, 1-奇, 2-偶)
} uart_config_t;

/**
 * 初始化UART
 * @param uart_id: UART编号 (0-3)
 * @param cfg: 配置参数
 * @return: 0-成功, <0-失败
 */
int hal_uart_init(uint32_t uart_id, const uart_config_t *cfg);

/**
 * 发送数据
 * @param uart_id: UART编号
 * @param data: 数据指针
 * @param len: 数据长度
 * @return: 实际发送字节数, <0-失败
 */
int hal_uart_write(uint32_t uart_id, const uint8_t *data, uint32_t len);

/**
 * 接收数据（阻塞）
 * @param uart_id: UART编号
 * @param data: 数据缓冲区
 * @param len: 最多接收字节数
 * @return: 实际接收字节数, <0-失败
 */
int hal_uart_read(uint32_t uart_id, uint8_t *data, uint32_t len);

/**
 * 查询是否有数据可读
 * @param uart_id: UART编号
 * @return: >0-有数据, 0-无数据, <0-失败
 */
int hal_uart_available(uint32_t uart_id);

/**
 * 设置UART中断
 * @param uart_id: UART编号
 * @param enable: 1-使能, 0-禁用
 * @return: 0-成功, <0-失败
 */
int hal_uart_set_interrupt(uint32_t uart_id, uint8_t enable);
```

### 2.4 中断管理
```c
/**
 * hal/irq.h - 中断控制接口
 */

typedef void (*irq_handler_t)(uint32_t irq_num, void *arg);

typedef enum {
    IRQ_LEVEL_LOW = 0,
    IRQ_LEVEL_HIGH,
    IRQ_EDGE_RISING,
    IRQ_EDGE_FALLING,
} irq_trigger_t;

/**
 * 初始化中断系统
 * @return: 0-成功, <0-失败
 */
int hal_irq_init(void);

/**
 * 注册中断处理函数
 * @param irq: 中断号
 * @param handler: 处理函数指针
 * @param arg: 处理函数参数
 * @return: 0-成功, <0-失败
 */
int hal_irq_register(uint32_t irq, irq_handler_t handler, void *arg);

/**
 * 使能中断
 * @param irq: 中断号
 * @return: 0-成功, <0-失败
 */
int hal_irq_enable(uint32_t irq);

/**
 * 禁用中断
 * @param irq: 中断号
 * @return: 0-成功, <0-失败
 */
int hal_irq_disable(uint32_t irq);

/**
 * 配置中断触发方式
 * @param irq: 中断号
 * @param trigger: 触发方式
 * @return: 0-成功, <0-失败
 */
int hal_irq_set_trigger(uint32_t irq, irq_trigger_t trigger);

/**
 * 全局禁用所有中断
 * @return: 之前的中断状态
 */
uint32_t hal_irq_disable_all(void);

/**
 * 恢复中断状态
 * @param state: 之前保存的状态
 */
void hal_irq_restore(uint32_t state);
```

### 2.5 内存管理
```c
/**
 * hal/memory.h - 内存接口
 */

typedef struct {
    uint32_t virt_addr;     // 虚拟地址
    uint32_t phys_addr;     // 物理地址
    uint32_t size;          // 大小
    uint32_t attr;          // 属性 (可读/可写/可执行)
} mmu_entry_t;

/**
 * 初始化MMU
 * @return: 0-成功, <0-失败
 */
int hal_mmu_init(void);

/**
 * 启用MMU
 * @return: 0-成功, <0-失败
 */
int hal_mmu_enable(void);

/**
 * 禁用MMU
 * @return: 0-成功, <0-失败
 */
int hal_mmu_disable(void);

/**
 * 添加页表映射
 * @param entry: MMU条目
 * @return: 0-成功, <0-失败
 */
int hal_mmu_map(const mmu_entry_t *entry);

/**
 * 刷新TLB
 * @return: 0-成功, <0-失败
 */
int hal_mmu_tlb_flush(void);

/**
 * 启用缓存
 * @param level: 缓存级别 (1-L1, 2-L2)
 * @return: 0-成功, <0-失败
 */
int hal_cache_enable(uint32_t level);

/**
 * 禁用缓存
 * @param level: 缓存级别
 * @return: 0-成功, <0-失败
 */
int hal_cache_disable(uint32_t level);

/**
 * 清空缓存
 * @param level: 缓存级别
 * @return: 0-成功, <0-失败
 */
int hal_cache_flush(uint32_t level);

/**
 * DRAM初始化
 * @param dram_size: DRAM大小(字节)
 * @return: 0-成功, <0-失败
 */
int hal_dram_init(uint32_t dram_size);

/**
 * DRAM测试
 * @param start: 起始地址
 * @param size: 测试大小
 * @return: 0-通过, <0-失败
 */
int hal_dram_test(uint32_t start, uint32_t size);
```

---

## 3. 驱动层API

### 3.1 Flash驱动
```c
/**
 * driver/flash.h - Flash存储接口
 */

typedef enum {
    FLASH_TYPE_NOR_SPI = 0,
    FLASH_TYPE_NAND_SPI,
    FLASH_TYPE_NAND_PAR,
    FLASH_TYPE_EMMC,
} flash_type_t;

typedef struct {
    uint32_t total_size;    // 总大小
    uint32_t sector_size;   // 扇区大小
    uint32_t page_size;     // 页大小 (NAND)
    uint32_t block_size;    // 块大小 (NAND)
    flash_type_t type;      // 存储类型
} flash_info_t;

/**
 * 初始化Flash
 * @return: 0-成功, <0-失败
 */
int driver_flash_init(void);

/**
 * 获取Flash信息
 * @param info: 信息结构体指针
 * @return: 0-成功, <0-失败
 */
int driver_flash_get_info(flash_info_t *info);

/**
 * 读取Flash数据
 * @param addr: 起始地址
 * @param buf: 数据缓冲区
 * @param len: 读取长度
 * @return: 实际读取字节数, <0-失败
 */
int driver_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);

/**
 * 写入Flash数据
 * @param addr: 起始地址
 * @param buf: 数据缓冲区
 * @param len: 写入长度
 * @return: 实际写入字节数, <0-失败
 */
int driver_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len);

/**
 * 擦除Flash扇区
 * @param addr: 起始地址
 * @param len: 擦除长度
 * @return: 0-成功, <0-失败
 */
int driver_flash_erase(uint32_t addr, uint32_t len);

/**
 * 读取OTP区域
 * @param offset: 偏移
 * @param buf: 缓冲区
 * @param len: 长度
 * @return: 实际读取字节数, <0-失败
 */
int driver_otp_read(uint32_t offset, uint8_t *buf, uint32_t len);

/**
 * 检查坏块 (NAND)
 * @param block_addr: 块地址
 * @return: 0-好块, 1-坏块, <0-错误
 */
int driver_flash_is_bad_block(uint32_t block_addr);
```

### 3.2 校验算法
```c
/**
 * driver/crc.h - CRC计算
 */

/**
 * 计算CRC32
 * @param data: 数据指针
 * @param len: 数据长度
 * @param prev_crc: 前一个CRC值 (初始为0)
 * @return: CRC32值
 */
uint32_t driver_crc32(const uint8_t *data, uint32_t len, uint32_t prev_crc);

/**
 * 计算CRC64
 * @param data: 数据指针
 * @param len: 数据长度
 * @param prev_crc: 前一个CRC值 (初始为0)
 * @return: CRC64值
 */
uint64_t driver_crc64(const uint8_t *data, uint32_t len, uint64_t prev_crc);

/**
 * driver/sha256.h - SHA256计算
 */

typedef struct {
    uint32_t state[8];
    uint32_t count[2];
    uint8_t buffer[64];
} sha256_ctx_t;

/**
 * 初始化SHA256上下文
 * @param ctx: 上下文指针
 */
void driver_sha256_init(sha256_ctx_t *ctx);

/**
 * 更新SHA256（增量计算）
 * @param ctx: 上下文指针
 * @param data: 数据指针
 * @param len: 数据长度
 */
void driver_sha256_update(sha256_ctx_t *ctx, const uint8_t *data, uint32_t len);

/**
 * 完成SHA256计算
 * @param ctx: 上下文指针
 * @param digest: 输出摘要 (32字节)
 */
void driver_sha256_final(sha256_ctx_t *ctx, uint8_t *digest);

/**
 * 一次性计算SHA256
 * @param data: 数据指针
 * @param len: 数据长度
 * @param digest: 输出摘要 (32字节)
 */
void driver_sha256(const uint8_t *data, uint32_t len, uint8_t *digest);
```

### 3.3 密钥和签名
```c
/**
 * driver/rsa.h - RSA签名验证
 */

typedef struct {
    uint8_t exponent[4];    // 公钥指数 (通常为65537)
    uint8_t modulus[256];   // 公钥模数 (2048位)
} rsa_public_key_t;

typedef struct {
    uint8_t signature[256]; // 签名数据
    uint8_t reserved[256];  // 预留
} rsa_signature_t;

/**
 * RSA签名验证
 * @param message: 消息数据
 * @param msg_len: 消息长度
 * @param signature: 签名数据
 * @param public_key: 公钥
 * @return: 0-签名有效, 1-签名无效, <0-错误
 */
int driver_rsa_verify(const uint8_t *message, uint32_t msg_len,
                      const rsa_signature_t *signature,
                      const rsa_public_key_t *public_key);

/**
 * driver/ecdsa.h - ECDSA签名验证
 */

typedef struct {
    uint8_t x[32];          // 公钥X坐标
    uint8_t y[32];          // 公钥Y坐标
} ecdsa_public_key_t;

typedef struct {
    uint8_t r[32];          // 签名R值
    uint8_t s[32];          // 签名S值
} ecdsa_signature_t;

/**
 * ECDSA签名验证 (P-256)
 * @param message: 消息数据
 * @param msg_len: 消息长度
 * @param signature: 签名数据
 * @param public_key: 公钥
 * @return: 0-签名有效, 1-签名无效, <0-错误
 */
int driver_ecdsa_verify(const uint8_t *message, uint32_t msg_len,
                        const ecdsa_signature_t *signature,
                        const ecdsa_public_key_t *public_key);
```

### 3.4 看门狗
```c
/**
 * driver/watchdog.h - 看门狗接口
 */

/**
 * 初始化看门狗
 * @param timeout_ms: 超时时间(毫秒)
 * @return: 0-成功, <0-失败
 */
int driver_watchdog_init(uint32_t timeout_ms);

/**
 * 喂狗（清除看门狗计时器）
 * @return: 0-成功, <0-失败
 */
int driver_watchdog_feed(void);

/**
 * 禁用看门狗
 * @return: 0-成功, <0-失败
 */
int driver_watchdog_disable(void);

/**
 * 强制复位
 * @return: 不返回
 */
void driver_watchdog_force_reset(void);

/**
 * 获取看门狗状态
 * @return: 位图: bit0=已启用, bit1=已触发复位, bit[31:2]=计数器值
 */
uint32_t driver_watchdog_status(void);
```

---

## 4. 核心服务层API

### 4.1 启动验证
```c
/**
 * core/verify.h - 镜像验证接口
 */

typedef enum {
    VERIFY_CRC32 = 0x01,
    VERIFY_CRC64 = 0x02,
    VERIFY_SHA256 = 0x04,
    VERIFY_RSA = 0x08,
    VERIFY_ECDSA = 0x10,
    VERIFY_VERSION = 0x20,
    VERIFY_ALL = 0x3F,
} verify_method_t;

typedef struct {
    uint32_t crc32;
    uint64_t crc64;
    uint8_t sha256[32];
    uint8_t signature[256];
    uint32_t version;
} image_verify_data_t;

typedef struct {
    uint32_t addr;          // 镜像地址
    uint32_t size;          // 镜像大小
    image_verify_data_t *verify_data;
    verify_method_t methods;  // 验证方法
} image_t;

/**
 * 验证镜像完整性
 * @param image: 镜像信息
 * @return: 0-全部通过, >0-部分通过(位图), <0-失败
 */
int core_verify_image(const image_t *image);

/**
 * 验证版本号（防回滚）
 * @param current_version: 当前版本
 * @param min_allowed_version: 最低允许版本
 * @return: 0-版本可接受, 1-版本太低, <0-错误
 */
int core_verify_version(uint32_t current_version, uint32_t min_allowed_version);

/**
 * 获取镜像签名状态
 * @param image: 镜像信息
 * @return: 签名状态位图
 */
uint32_t core_get_signature_status(const image_t *image);
```

### 4.2 镜像加载
```c
/**
 * core/loader.h - 镜像加载接口
 */

typedef enum {
    IMAGE_FORMAT_RAW = 0,
    IMAGE_FORMAT_UIMAGE,
    IMAGE_FORMAT_FIT,
    IMAGE_FORMAT_ELF,
} image_format_t;

typedef struct {
    uint32_t load_addr;     // 加载地址
    uint32_t entry_addr;    // 入口地址
    uint32_t size;          // 大小
    image_format_t format;  // 格式
    uint8_t compression;    // 压缩算法
} image_header_t;

/**
 * 加载镜像
 * @param flash_addr: Flash中的地址
 * @param header: 镜像头指针
 * @return: 0-成功, <0-失败
 */
int core_load_image(uint32_t flash_addr, image_header_t *header);

/**
 * 加载Device Tree
 * @param dtb_addr: DTB在Flash中的地址
 * @param load_addr: 加载到内存中的地址
 * @return: 0-成功, <0-失败
 */
int core_load_dtb(uint32_t dtb_addr, uint32_t load_addr);

/**
 * 设置内核启动参数
 * @param key: 参数名
 * @param value: 参数值
 * @return: 0-成功, <0-失败
 */
int core_set_bootargs(const char *key, const char *value);

/**
 * 获取启动参数字符串
 * @return: 指向bootargs字符串的指针
 */
const char *core_get_bootargs(void);
```

### 4.3 启动控制
```c
/**
 * core/boot.h - 启动控制接口
 */

typedef enum {
    BOOT_MODE_NORMAL = 0,
    BOOT_MODE_RECOVERY,
    BOOT_MODE_FASTBOOT,
} boot_mode_t;

typedef struct {
    uint32_t kernel_addr;      // 内核地址
    uint32_t dtb_addr;         // DTB地址
    uint32_t ramdisk_addr;     // Ramdisk地址 (可选)
    const char *bootargs;      // 启动参数
    boot_mode_t mode;          // 启动模式
} boot_params_t;

/**
 * 初始化启动系统
 * @return: 0-成功, <0-失败
 */
int core_boot_init(void);

/**
 * 执行启动检查
 * @return: 0-通过, 1-降级, <0-失败
 */
int core_boot_check(void);

/**
 * 启动内核
 * @param params: 启动参数
 * @return: 不返回（内核接管控制权）
 */
void core_boot_kernel(const boot_params_t *params);

/**
 * 进入恢复模式
 * @return: 不返回
 */
void core_boot_recovery(void);

/**
 * 设置启动模式
 * @param mode: 启动模式
 * @return: 0-成功, <0-失败
 */
int core_set_boot_mode(boot_mode_t mode);

/**
 * 获取启动模式
 * @return: 启动模式
 */
boot_mode_t core_get_boot_mode(void);
```

### 4.4 分区管理
```c
/**
 * core/partition.h - 分区表接口
 */

typedef struct {
    char name[32];          // 分区名称
    uint32_t offset;        // 偏移地址
    uint32_t size;          // 大小
    uint32_t flags;         // 标志
} partition_entry_t;

typedef struct {
    uint32_t magic;         // 魔数
    uint32_t version;       // 版本
    uint32_t entry_count;   // 条目数
    partition_entry_t entries[16];
} partition_table_t;

/**
 * 读取分区表
 * @param table: 分区表指针
 * @return: 0-成功, <0-失败
 */
int core_partition_read(partition_table_t *table);

/**
 * 查找分区
 * @param name: 分区名称
 * @return: 分区条目指针, 或NULL
 */
partition_entry_t *core_partition_find(const char *name);

/**
 * 获取分区地址
 * @param name: 分区名称
 * @return: 分区地址, 或0-未找到
 */
uint32_t core_partition_get_addr(const char *name);

/**
 * 获取分区大小
 * @param name: 分区名称
 * @return: 分区大小, 或0-未找到
 */
uint32_t core_partition_get_size(const char *name);
```

### 4.5 故障处理
```c
/**
 * core/recovery.h - 故障恢复接口
 */

typedef enum {
    RECOVERY_FALLBACK = 0,      // 使用备用分区
    RECOVERY_FACTORY_RESET,     // 恢复出厂设置
    RECOVERY_BOOTLOADER_MODE,   // Bootloader模式
    RECOVERY_HALT,              // 停止
} recovery_action_t;

/**
 * 处理启动失败
 * @param reason: 失败原因
 * @return: 恢复行为
 */
recovery_action_t core_handle_boot_failure(uint32_t reason);

/**
 * 从备用分区启动
 * @return: 0-成功, <0-失败
 */
int core_recovery_fallback_boot(void);

/**
 * 进入Bootloader模式
 * @return: 不返回
 */
void core_recovery_bootloader_mode(void);

/**
 * 记录故障信息
 * @param fmt: 格式字符串
 * @param ...: 参数列表
 */
void core_recovery_log(const char *fmt, ...);
```

---

## 5. 平台适配层API

### 5.1 平台初始化
```c
/**
 * platform/platform.h - 平台适配接口
 */

typedef enum {
    PLATFORM_ARMV7 = 0,
    PLATFORM_ARMV8,
    PLATFORM_X86_64,
} platform_type_t;

/**
 * 检测平台类型
 * @return: 平台类型
 */
platform_type_t platform_detect(void);

/**
 * 平台特定初始化
 * @return: 0-成功, <0-失败
 */
int platform_init(void);

/**
 * 获取平台信息
 * @param info: 信息字符串缓冲区
 * @param len: 缓冲区长度
 * @return: 0-成功, <0-失败
 */
int platform_get_info(char *info, uint32_t len);

/**
 * 平台特定清理（启动前）
 * @return: 0-成功, <0-失败
 */
int platform_cleanup(void);
```

### 5.2 芯片适配
```c
/**
 * platform/chip.h - 芯片特定接口
 */

typedef struct {
    char chip_name[32];
    uint32_t chip_id;
    uint32_t chip_revision;
    char production_date[16];
} chip_info_t;

/**
 * 获取芯片信息
 * @param info: 芯片信息指针
 * @return: 0-成功, <0-失败
 */
int chip_get_info(chip_info_t *info);

/**
 * 获取唯一设备标识符
 * @param uid: 唯一ID缓冲区 (16字节)
 * @return: 0-成功, <0-失败
 */
int chip_get_unique_id(uint8_t uid[16]);

/**
 * 读取芯片特定寄存器
 * @param offset: 寄存器偏移
 * @param value: 值指针
 * @return: 0-成功, <0-失败
 */
int chip_read_register(uint32_t offset, uint32_t *value);

/**
 * 写入芯片特定寄存器
 * @param offset: 寄存器偏移
 * @param value: 值
 * @return: 0-成功, <0-失败
 */
int chip_write_register(uint32_t offset, uint32_t value);
```

---

## 6. 错误代码定义

```c
/**
 * include/errno.h - 错误代码
 */

#define BOOT_SUCCESS                0    // 成功
#define BOOT_ERR_UNKNOWN           -1   // 未知错误
#define BOOT_ERR_INVALID_PARAM     -2   // 无效参数
#define BOOT_ERR_TIMEOUT           -3   // 超时
#define BOOT_ERR_NO_MEMORY         -4   // 内存不足

// 硬件相关错误
#define BOOT_ERR_HW_NOT_READY      -10  // 硬件未准备好
#define BOOT_ERR_CLOCK_INIT        -11  // 时钟初始化失败
#define BOOT_ERR_DRAM_INIT         -12  // DRAM初始化失败
#define BOOT_ERR_DRAM_TEST         -13  // DRAM测试失败

// Flash相关错误
#define BOOT_ERR_FLASH_INIT        -20  // Flash初始化失败
#define BOOT_ERR_FLASH_READ        -21  // Flash读失败
#define BOOT_ERR_FLASH_WRITE       -22  // Flash写失败
#define BOOT_ERR_FLASH_ERASE       -23  // Flash擦除失败
#define BOOT_ERR_BAD_BLOCK         -24  // 坏块检测

// 校验相关错误
#define BOOT_ERR_CRC_MISMATCH      -30  // CRC不匹配
#define BOOT_ERR_HASH_MISMATCH     -31  // 哈希不匹配
#define BOOT_ERR_SIGNATURE_INVALID -32  // 签名无效
#define BOOT_ERR_CERT_INVALID      -33  // 证书无效
#define BOOT_ERR_VERSION_ROLLBACK  -34  // 版本回滚

// 镜像相关错误
#define BOOT_ERR_IMAGE_NOT_FOUND   -40  // 镜像未找到
#define BOOT_ERR_IMAGE_CORRUPT     -41  // 镜像损坏
#define BOOT_ERR_IMAGE_LOAD        -42  // 镜像加载失败
#define BOOT_ERR_DTB_INVALID       -43  // DTB无效

// 分区相关错误
#define BOOT_ERR_PARTITION_ERROR   -50  // 分区错误
#define BOOT_ERR_PARTITION_TABLE   -51  // 分区表错误
#define BOOT_ERR_NO_VALID_BOOT     -52  // 无有效启动分区

// 系统相关错误
#define BOOT_ERR_WATCHDOG          -60  // 看门狗超时
#define BOOT_ERR_EXCEPTION         -61  // 异常发生
#define BOOT_ERR_SECURE_BOOT       -62  // 安全启动失败
```

---

## 7. 宏定义和常量

```c
/**
 * include/config.h - 配置常量
 */

// 版本信息
#define BOOTLOADER_VERSION_MAJOR   2
#define BOOTLOADER_VERSION_MINOR   0
#define BOOTLOADER_VERSION_PATCH   0

// 地址定义
#define BOOTLOADER_BASE            0x00000000
#define BOOTLOADER_SIZE            0x20000    // 128KB
#define CONFIG_BASE                0x00040000
#define KERNEL_BASE                0x00080000
#define DTB_BASE                   0x00D00000
#define APP_BASE                   0x01800000
#define OTP_BASE                   0xFFF00000

// 大小限制
#define MAX_KERNEL_SIZE            0x10000000 // 256MB
#define MAX_DTB_SIZE               0x100000   // 1MB
#define MAX_BOOTARGS_LEN           1024

// 超时设置
#define BOOT_TIMEOUT_STAGE1        100    // ms
#define BOOT_TIMEOUT_STAGE2        300    // ms
#define BOOT_TIMEOUT_STAGE3        500    // ms
#define BOOT_TIMEOUT_TOTAL         2000   // ms

// 看门狗
#define WATCHDOG_TIMEOUT           30000  // ms

// 启动标志
#define BOOT_FLAG_SECURE_BOOT      0x01
#define BOOT_FLAG_VERBOSE          0x02
#define BOOT_FLAG_RECOVERY         0x04
#define BOOT_FLAG_FASTBOOT         0x08
```

