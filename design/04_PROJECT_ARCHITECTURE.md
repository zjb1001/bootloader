# Bootloader项目源码架构

**版本**: 1.0  
**日期**: 2026年1月  
**分类**: 项目结构设计

---

## 一屏摘要
- 目的：指导代码落地、构建、测试与工具使用；目录即职责。
- MVP（可启动到 UART Hello）：src/arch/*/boot.s, src/core/crt0.c, src/hal/clock.c, src/hal/uart.c, CMakeLists/Toolchain 基础。
- 阶段扩展：Stage2 增 DRAM/MMU/IRQ/SPI/WDT；Stage3 增 CRC/SHA/签名；Stage4 增 loader/dtb/recovery。
- 产物：boot/bootloader.bin, bootloader.map；工具链：mkimage → sign → mkpart → otp_write。
- 测试入口：test/unit、test/integration、test/perf，配套 mock/ 数据。

---

## 最小可构建集合（MVP）
- 目录：`src/arch/*/boot.s`、`src/core/crt0.c`、`src/core/startup.c`（精简）、`src/hal/clock.c`、`src/hal/uart.c`、`boot/bootloader.ld`、根 `CMakeLists.txt`。
- 构建要求：交叉工具链、链接脚本、栈/段符号、基础启动入口 `_start`。
- 产出：`boot/bootloader.bin`、`boot/bootloader.map`。

示例构建步骤（参考）：

```bash
# 配置与构建（示意）
cmake -B build -DCMAKE_TOOLCHAIN_FILE=scripts/cmake/Toolchains.cmake -DPLATFORM=armv7
cmake --build build -j

# 生成镜像与签名（工具链示意）
python3 tools/mkimage.py --in build/bootloader.bin --out boot/boot.img
python3 tools/sign.py --in boot/boot.img --key tools/keys/pub.pem --out boot/boot.signed
python3 tools/mkpart.py --kernel boot/boot.signed --dtb boot/dtb.bin --out boot/flash.img
python3 tools/otp_write.py --pubkey tools/keys/pub.pem --minver 100
```

---

## 交付物清单与验证
- 二进制：`boot/bootloader.bin`（尺寸 <256KB），`.map`。
- 镜像包：签名后的 `boot/boot.signed`、分区镜像 `boot/flash.img`。
- 元数据：`key_id`、`security_version`、`sha256` 摘要文件。
- 验收：与 01_SYSTEM_PROPERTIES 指标与 06_TEST_STRATEGY 用例对齐（性能/安全/回滚）。

---

## 目录职责速览（对齐架构与API）
- `src/hal/`：时钟/内存/中断/UART 等 HAL 实现（对应 03_API_DESIGN HAL）。
- `src/driver/`：Flash/crypto/watchdog/spi（对应 03_API_DESIGN Driver）。
- `src/core/`：verify/loader/image/partition/recovery/boot（对应 02 启动链）。
- `src/platform/`：板级 SoC 差异化配置（clock/memory/pin）。
- `tools/`：镜像生成/签名/OTP 写入。
- `test/`：unit/integration/perf，mock 支持。

## 1. 项目整体结构

```
bootloader/
├── docs/                       # 文档目录
│   ├── DESIGN.md              # 架构设计文档
│   ├── API.md                 # API参考
│   ├── BUILDING.md            # 编译指南
│   ├── PORTING.md             # 移植指南
│   └── TROUBLESHOOTING.md     # 故障排查
│
├── design/                    # 设计文档（已创建）
│   ├── 01_SYSTEM_PROPERTIES.md
│   ├── 02_BOOTLOADER_ARCHITECTURE.md
│   ├── 03_API_DESIGN.md
│   ├── 04_PROJECT_ARCHITECTURE.md
│   ├── 05_SECURITY_POLICY.md
│   └── 06_TEST_STRATEGY.md
│
├── include/                   # 公共头文件
│   ├── boot.h                 # 启动系统接口
│   ├── config.h               # 配置常量
│   ├── errno.h                # 错误代码
│   ├── types.h                # 类型定义
│   ├── version.h              # 版本信息
│   │
│   ├── hal/                   # 硬件抽象层接口
│   │   ├── clock.h
│   │   ├── gpio.h
│   │   ├── uart.h
│   │   ├── irq.h
│   │   ├── memory.h
│   │   └── timer.h
│   │
│   ├── driver/                # 驱动层接口
│   │   ├── flash.h
│   │   ├── crc.h
│   │   ├── sha256.h
│   │   ├── rsa.h
│   │   ├── ecdsa.h
│   │   ├── watchdog.h
│   │   └── spi.h
│   │
│   └── core/                  # 核心服务接口
│       ├── verify.h
│       ├── loader.h
│       ├── boot.h
│       ├── partition.h
│       └── recovery.h
│
├── src/                       # 源代码目录
│   ├── arch/                  # 架构特定代码
│   │   ├── arm/
│   │   │   ├── CMakeLists.txt
│   │   │   ├── armv7/         # ARMv7实现
│   │   │   │   ├── boot.s
│   │   │   │   ├── irq.s
│   │   │   │   ├── cache.s
│   │   │   │   ├── mmu.s
│   │   │   │   └── sys.c
│   │   │   └── armv8/         # ARMv8实现
│   │   │       ├── boot.s
│   │   │       ├── irq.s
│   │   │       ├── cache.s
│   │   │       ├── mmu.s
│   │   │       └── sys.c
│   │   │
│   │   └── x86/
│   │       ├── CMakeLists.txt
│   │       ├── boot.s
│   │       ├── gdt.s
│   │       ├── idt.s
│   │       └── sys.c
│   │
│   ├── hal/                   # 硬件抽象层实现
│   │   ├── CMakeLists.txt
│   │   ├── clock.c
│   │   ├── gpio.c
│   │   ├── uart.c
│   │   ├── irq.c
│   │   ├── memory.c
│   │   ├── timer.c
│   │   └── debug.c
│   │
│   ├── driver/                # 驱动实现
│   │   ├── CMakeLists.txt
│   │   ├── flash/
│   │   │   ├── flash.c
│   │   │   ├── nor_spi.c
│   │   │   ├── nand_spi.c
│   │   │   ├── emmc.c
│   │   │   └── ecc.c
│   │   │
│   │   ├── crypto/
│   │   │   ├── crc.c
│   │   │   ├── sha256.c
│   │   │   ├── rsa.c
│   │   │   ├── ecdsa.c
│   │   │   └── bignum.c      # 大数运算库
│   │   │
│   │   ├── watchdog.c
│   │   ├── spi.c
│   │   └── console.c
│   │
│   ├── core/                  # 核心服务实现
│   │   ├── CMakeLists.txt
│   │   ├── boot.c
│   │   ├── verify.c
│   │   ├── loader.c
│   │   ├── image.c
│   │   ├── partition.c
│   │   ├── recovery.c
│   │   ├── crt0.c             # C运行时
│   │   ├── startup.c          # 启动序列
│   │   └── main.c
│   │
│   ├── platform/              # 平台适配
│   │   ├── CMakeLists.txt
│   │   ├── platform.c
│   │   ├── chip.c
│   │   └── <platform>/        # 具体平台目录
│   │       ├── board.h
│   │       ├── clock_config.c
│   │       ├── memory_config.c
│   │       └── pin_config.c
│   │
│   └── utils/                 # 工具函数
│       ├── CMakeLists.txt
│       ├── string.c
│       ├── math.c
│       ├── log.c
│       ├── assert.c
│       └── common.c
│
├── test/                      # 测试目录
│   ├── CMakeLists.txt
│   ├── unit/                  # 单元测试
│   │   ├── test_crc.c
│   │   ├── test_sha256.c
│   │   ├── test_rsa.c
│   │   ├── test_partition.c
│   │   └── test_verify.c
│   │
│   ├── integration/           # 集成测试
│   │   ├── test_boot_flow.c
│   │   ├── test_flash_access.c
│   │   └── test_recovery.c
│   │
│   ├── mock/                  # Mock对象
│   │   ├── mock_hal.c
│   │   ├── mock_driver.c
│   │   └── mock_flash.c
│   │
│   └── data/                  # 测试数据
│       ├── test_images/
│       ├── test_keys/
│       └── test_config/
│
├── tools/                     # 工具脚本
│   ├── build.sh              # 编译脚本
│   ├── sign.py               # 签名工具
│   ├── mkimage.py            # 镜像生成工具
│   ├── mkpart.py             # 分区表生成工具
│   ├── otp_write.py          # OTP烧写工具
│   ├── crc_calc.py           # CRC计算工具
│   └── gen_keys.py           # 密钥生成工具
│
├── config/                    # 配置文件
│   ├── default.conf           # 默认配置
│   ├── armv7.conf             # ARMv7配置
│   ├── armv8.conf             # ARMv8配置
│   ├── x86_64.conf            # x86_64配置
│   └── <board>.conf           # 具体板卡配置
│
├── boot/                      # 启动脚本和固件
│   ├── bootloader.ld          # 链接脚本
│   ├── bootloader.map         # 链接映射
│   └── bootloader.bin         # 生成的二进制
│
├── scripts/                   # 构建脚本
│   ├── cmake/
│   │   ├── FindArm.cmake
│   │   ├── FindHost.cmake
│   │   └── Toolchains.cmake
│   │
│   ├── common.cmake           # 公共CMake配置
│   ├── warnings.cmake         # 警告配置
│   └── sanitizers.cmake       # Sanitizer配置
│
├── CMakeLists.txt            # 根CMakeLists
├── Makefile                   # 备用Makefile
├── .gitignore                 # Git忽略规则
├── .clang-format              # 代码格式化
├── README.md                  # 项目说明
└── CHANGELOG.md               # 变更日志
```

---

## 2. 核心源文件详细说明

### 2.1 启动相关文件

#### src/arch/arm/armv7/boot.s
```assembly
; ARMv7启动代码 (~150-200行)
; 功能：
;   1. 禁用中断和缓存
;   2. 设置堆栈指针
;   3. 清零BSS段
;   4. 初始化全局数据
;   5. 跳转到_main()

; 关键符号：
; - _start: 入口点
; - _bss_start, _bss_end: BSS段边界
; - _stack_top: 堆栈顶部
; - _main: C代码入口
```

#### src/core/crt0.c
```c
// C运行时环境初始化 (~50-100行)
// 功能：
//   1. 全局对象构造（C++）
//   2. 静态初始化
//   3. 调用main()
//   4. 全局对象析构
//   5. 系统停止

extern void _start(void);
extern void boot_main(void);
int main(void);
```

#### src/core/main.c
```c
// Bootloader主程序 (~100-150行)
// 功能：
//   1. 初始化所有系统模块
//   2. 执行启动序列
//   3. 处理错误和恢复

int main(void) {
    // 执行启动序列
    return boot_main();
}
```

#### src/core/startup.c
```c
// 启动序列 (~200-300行)
// 功能：
//   1. Stage 1: 早期硬件初始化
//   2. Stage 2: 完整的硬件初始化
//   3. Stage 3: 系统自检
//   4. Stage 4: 固件加载和验证
//   5. Stage 5: 内核启动

int boot_main(void);
int boot_stage1_early_init(void);
int boot_stage2_hw_init(void);
int boot_stage3_selftest(void);
int boot_stage4_load_verify(void);
int boot_stage5_start_kernel(void);
```

### 2.2 硬件抽象层

#### src/hal/clock.c
```c
// 时钟初始化 (~150-200行)
// 功能：
//   1. 配置PLL参数
//   2. 设置时钟分频
//   3. 验证时钟频率

int hal_clock_init(const clock_config_t *cfg);
int hal_clock_get_freq(clock_source_t src);
```

#### src/hal/uart.c
```c
// UART驱动 (~150-200行)
// 功能：
//   1. 串口初始化
//   2. 收发数据
//   3. 中断处理

int hal_uart_init(uint32_t uart_id, const uart_config_t *cfg);
int hal_uart_write(uint32_t uart_id, const uint8_t *data, uint32_t len);
int hal_uart_read(uint32_t uart_id, uint8_t *data, uint32_t len);
```

#### src/hal/irq.c
```c
// 中断系统 (~200-250行)
// 功能：
//   1. 中断向量表设置
//   2. 中断处理函数注册
//   3. 中断使能/禁用
//   4. 异常处理

int hal_irq_init(void);
int hal_irq_register(uint32_t irq, irq_handler_t handler, void *arg);
void hal_irq_handler(uint32_t irq);
```

#### src/hal/memory.c
```c
// 内存管理 (~200-300行)
// 功能：
//   1. MMU初始化
//   2. 页表配置
//   3. DRAM初始化
//   4. DRAM测试

int hal_mmu_init(void);
int hal_mmu_enable(void);
int hal_dram_init(uint32_t dram_size);
int hal_dram_test(uint32_t start, uint32_t size);
```

### 2.3 驱动层

#### src/driver/flash/flash.c
```c
// Flash通用驱动 (~200-250行)
// 功能：
//   1. 驱动初始化
//   2. 类型检测
//   3. 读写操作分发

int driver_flash_init(void);
int driver_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);
int driver_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len);
```

#### src/driver/flash/nor_spi.c
```c
// NOR Flash SPI驱动 (~300-400行)
// 功能：
//   1. SPI初始化
//   2. NOR Flash命令
//   3. 高速读写

int nor_flash_init(void);
int nor_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);
```

#### src/driver/crypto/sha256.c
```c
// SHA256实现 (~200-250行)
// 功能：
//   1. SHA256初始化
//   2. 数据摘要计算
//   3. 流式处理

void driver_sha256_init(sha256_ctx_t *ctx);
void driver_sha256_update(sha256_ctx_t *ctx, const uint8_t *data, uint32_t len);
void driver_sha256_final(sha256_ctx_t *ctx, uint8_t *digest);
```

#### src/driver/crypto/rsa.c
```c
// RSA签名验证 (~150-200行)
// 功能：
//   1. RSA初始化
//   2. 模幂运算
//   3. 签名验证

int driver_rsa_verify(const uint8_t *message, uint32_t msg_len,
                      const rsa_signature_t *signature,
                      const rsa_public_key_t *public_key);
```

#### src/driver/watchdog.c
```c
// 看门狗驱动 (~100-150行)
// 功能：
//   1. 看门狗初始化
//   2. 超时设置
//   3. 喂狗操作

int driver_watchdog_init(uint32_t timeout_ms);
int driver_watchdog_feed(void);
```

### 2.4 核心服务

#### src/core/verify.c
```c
// 镜像验证 (~200-300行)
// 功能：
//   1. CRC校验
//   2. SHA256校验
//   3. 签名验证
//   4. 版本检查

int core_verify_image(const image_t *image);
int core_verify_version(uint32_t current_version, uint32_t min_allowed);
uint32_t core_get_signature_status(const image_t *image);
```

#### src/core/loader.c
```c
// 镜像加载 (~200-250行)
// 功能：
//   1. 镜像头解析
//   2. 内存加载
//   3. 压缩解压
//   4. DTB加载

int core_load_image(uint32_t flash_addr, image_header_t *header);
int core_load_dtb(uint32_t dtb_addr, uint32_t load_addr);
```

#### src/core/partition.c
```c
// 分区管理 (~150-200行)
// 功能：
//   1. 分区表读取
//   2. 分区查找
//   3. 地址转换

int core_partition_read(partition_table_t *table);
partition_entry_t *core_partition_find(const char *name);
uint32_t core_partition_get_addr(const char *name);
```

#### src/core/boot.c
```c
// 启动控制 (~250-300行)
// 功能：
//   1. 启动检查
//   2. 参数准备
//   3. 内核启动

int core_boot_init(void);
int core_boot_check(void);
void core_boot_kernel(const boot_params_t *params);
void core_boot_recovery(void);
```

#### src/core/recovery.c
```c
// 故障恢复 (~200-250行)
// 功能：
//   1. 故障检测
//   2. 备用启动
//   3. 恢复模式

recovery_action_t core_handle_boot_failure(uint32_t reason);
int core_recovery_fallback_boot(void);
void core_recovery_bootloader_mode(void);
```

### 2.5 平台适配

#### src/platform/platform.c
```c
// 平台检测 (~100-150行)
// 功能：
//   1. 检测处理器架构
//   2. 获取平台信息
//   3. 平台初始化

platform_type_t platform_detect(void);
int platform_init(void);
```

#### src/platform/<board>/clock_config.c
```c
// 板卡时钟配置 (~50-100行)
// 功能：
//   1. 特定板卡的PLL配置
//   2. 时钟分频设置
//   3. 时钟验证
```

### 2.6 工具函数

#### src/utils/log.c
```c
// 日志系统 (~100-150行)
// 功能：
//   1. 日志输出
//   2. 错误记录
//   3. 调试追踪

int printf(const char *fmt, ...);
int fprintf(FILE *f, const char *fmt, ...);
```

#### src/utils/string.c
```c
// 字符串函数 (~100-150行)
// 功能：
//   1. 字符串复制
//   2. 字符串比较
//   3. 内存操作

void *memcpy(void *dst, const void *src, size_t n);
void *memset(void *s, int c, size_t n);
int strcmp(const char *s1, const char *s2);
```

---

## 3. 文件大小和代码量估计

| 模块 | 文件数 | 代码行数 | 说明 |
|------|-------|--------|------|
| **汇编代码** | 6-8 | 800-1200 | 架构特定 |
| **HAL层** | 6-8 | 1500-2000 | 硬件抽象 |
| **驱动层** | 10-15 | 2500-3500 | 外设驱动 |
| **密码库** | 4-6 | 1500-2000 | SHA256/RSA/ECDSA |
| **核心服务** | 6-8 | 2000-2500 | 启动和验证 |
| **平台适配** | 3-5 | 400-600 | 板卡特定 |
| **工具函数** | 5-7 | 800-1000 | 通用函数 |
| **总计** | **40-52** | **9500-13000** | - |

---

## 4. 构建系统设计

### 4.1 CMakeLists.txt 结构

```cmake
# 根 CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(bootloader VERSION 2.0.0 LANGUAGES C ASM)

# 设置项目变量
set(CMAKE_C_STANDARD 99)
set(CMAKE_ASM_COMPILE_OBJECT "<CMAKE_C_COMPILER> <DEFINES> <INCLUDES> <FLAGS> -o <OBJECT> -c <SOURCE>")

# 包含工具链配置
include(scripts/cmake/Toolchains.cmake)

# 定义源代码子目录
add_subdirectory(src)
add_subdirectory(test)

# 定义目标
add_executable(bootloader.elf
    $<TARGET_OBJECTS:arch>
    $<TARGET_OBJECTS:hal>
    $<TARGET_OBJECTS:driver>
    $<TARGET_OBJECTS:core>
    $<TARGET_OBJECTS:platform>
    $<TARGET_OBJECTS:utils>
)

# 链接配置
target_link_options(bootloader.elf PRIVATE -T${CMAKE_SOURCE_DIR}/boot/bootloader.ld)
target_link_options(bootloader.elf PRIVATE -Wl,--gc-sections)
target_link_options(bootloader.elf PRIVATE -nostdlib)

# 生成二进制文件
add_custom_command(TARGET bootloader.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary bootloader.elf bootloader.bin
)
```

### 4.2 编译命令

```bash
# 配置构建
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../scripts/cmake/armv7.cmake \
      -DBOARD=arm_cortex_a9 \
      -DCMAKE_BUILD_TYPE=Release \
      ..

# 编译
make -j4

# 查看大小
arm-none-eabi-size bootloader.elf

# 生成符号表
arm-none-eabi-nm bootloader.elf > bootloader.map
```

---

## 5. 链接脚本（boot/bootloader.ld）

```ld
/* 链接脚本 - 定义内存布局 */
MEMORY {
    SRAM (rwx) : ORIGIN = 0x00000000, LENGTH = 64K
    DRAM (rwx) : ORIGIN = 0x80000000, LENGTH = 128M
    FLASH (r)  : ORIGIN = 0x00000000, LENGTH = 512M
}

SECTIONS {
    .text : {
        *(.text.start)
        *(.text .text.*)
    } > FLASH AT > FLASH

    .data : {
        *(.data .data.*)
    } > DRAM AT > FLASH

    .bss (NOLOAD) : {
        *(.bss .bss.*)
        *(COMMON)
    } > DRAM

    .rodata : {
        *(.rodata .rodata.*)
    } > FLASH
}
```

---

## 6. 编译配置文件

### 6.1 config/armv7.conf
```ini
[COMPILER]
ARCH = arm
CPU = cortex-a9
CROSS = arm-none-eabi

[FEATURES]
ENABLE_SECURE_BOOT = yes
ENABLE_SIGNATURE = yes
ENABLE_ROLLBACK_PROTECT = yes
ENABLE_RECOVERY = yes

[SIZES]
BOOTLOADER_SIZE = 128K
STACK_SIZE = 32K
HEAP_SIZE = 16K

[HARDWARE]
UART_BAUDRATE = 115200
WATCHDOG_TIMEOUT = 30000
DRAM_SIZE = 512M
```

### 6.2 config/<board>.conf
```ini
[BOARD]
NAME = ARM Cortex-A9 板卡
VENDOR = Example
REVISION = 1.0

[CLOCK]
OSC_FREQ = 24M
PLL_CORE = 1200M
PLL_DDR = 533M

[MEMORY]
BOOTLOADER_BASE = 0x00000000
KERNEL_BASE = 0x00080000
DRAM_BASE = 0x80000000
DRAM_SIZE = 2G

[DEVICES]
UART_PORT = /dev/ttyS0
SPI_SPEED = 100M
```

---

## 7. 测试文件结构

```c
// test/unit/test_crc.c
#include <assert.h>
#include "driver/crc.h"

void test_crc32_basic() {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint32_t crc = driver_crc32(data, sizeof(data), 0);
    assert(crc == 0x12345678);  // 预期值
}

void test_crc32_incremental() {
    // 测试流式计算
    uint32_t crc = 0;
    crc = driver_crc32(data1, len1, crc);
    crc = driver_crc32(data2, len2, crc);
    // 应与一次性计算相同
}

int main() {
    test_crc32_basic();
    test_crc32_incremental();
    return 0;
}
```

---

## 8. 文件命名规范

```
源文件命名：
- module_feature.c      (如: flash_nor.c, crypto_sha256.c)
- module_feature.h      (如: flash.h, sha256.h)

汇编文件命名：
- module.s              (如: mmu.s, irq.s)

测试文件命名：
- test_module.c         (如: test_crc.c, test_rsa.c)

宏定义：
- MODULE_CONSTANT_NAME  (如: UART_BAUDRATE_115200)

函数命名：
- layer_module_action   (如: hal_uart_init, driver_flash_read)
```

---

## 9. 依赖关系

```
启动入口
    ↓
汇编代码 (boot.s)
    ↓
C运行时 (crt0.c)
    ↓
平台检测 (platform.c) → 时钟初始化 (clock.c)
    ↓
硬件初始化序列
    ├→ DRAM初始化 (memory.c)
    ├→ 中断初始化 (irq.c)
    ├→ UART初始化 (uart.c)
    └→ 看门狗初始化 (watchdog.c)
    ↓
启动主程序 (boot.c)
    ├→ 系统自检 (startup.c)
    ├→ 分区加载 (partition.c)
    ├→ 镜像验证 (verify.c)
    │   ├→ CRC计算 (crc.c)
    │   ├→ SHA256计算 (sha256.c)
    │   └→ 签名验证 (rsa.c)
    ├→ 镜像加载 (loader.c)
    └→ 内核启动 (boot.c)
```

