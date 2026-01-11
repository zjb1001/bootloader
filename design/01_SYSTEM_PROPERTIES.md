# Bootloader系统属性与边界定义

**版本**: 1.0  
**日期**: 2026年1月  
**分类**: 系统设计基础

---

## 1. 目标系统属性

### 1.1 处理器架构
| 属性 | 值 | 备注 |
|------|-----|------|
| **架构** | ARM Cortex-A9/A7 或 x86_64 | 支持两种架构 |
| **位宽** | 32-bit / 64-bit | 可配置 |
| **指令集** | ARM Thumb-2 / x86 | 支持标准指令 |
| **最大频率** | 1.5GHz | 初始化不超过1.2GHz |
| **缓存** | L1(32KB) + L2(512KB) | 可配置启用/禁用 |
| **MMU** | 支持 | 虚拟地址映射 |

### 1.2 内存配置
| 类型 | 大小 | 地址范围 | 用途 |
|------|------|---------|------|
| **SRAM** | 64KB | 0x00000000-0x0000FFFF | 早期启动栈/代码 |
| **DRAM** | 2GB | 0x80000000-0xFFFFFFFF | 内核/应用 |
| **FLASH(XIP窗口)** | 256MB | 0x10000000-0x1FFFFFFF | Bootloader/只读段映射 |
| **FLASH(块访问)** | 4GB | 由控制器寻址 | 镜像/配置存储（非内存映射） |
| **OTP** | 8KB | 0x40000000-0x40001FFF | 密钥/版本 |
| **CACHE** | 512KB | 片上集成 | L2缓存 |

### 1.3 存储设备
| 存储类型 | 容量 | 接口 | 特性 |
|---------|------|------|------|
| **NOR Flash** | 512MB | SPI/QSPI | 随机访问 |
| **NAND Flash** | 2GB | SPI/Parallel | 顺序访问 |
| **eMMC** | 4GB | eMMC 4.5 | 块设备 |
| **SD/TF卡** | 可扩展 | SDIO | 可选 |

### 1.4 外设支持
| 外设 | 数量 | 用途 |
|------|------|------|
| **UART** | 2 | 调试/日志 |
| **SPI** | 2 | Flash访问 |
| **I2C** | 1 | 配置设备 |
| **GPIO** | 32+ | 控制/检测 |
| **看门狗** | 1 | 系统保护 |
| **定时器** | 3 | 时间管理 |
| **中断控制器** | 1 | IRQ分发 |

### 1.5 系统约束
```c
// 启动阶段时间约束
#define BOOT_STAGE1_MAX_TIME     100   // ms - 第一阶段
#define BOOT_STAGE2_MAX_TIME     300   // ms - 初始化
#define BOOT_STAGE3_MAX_TIME     500   // ms - 自检
#define BOOT_STAGE4_MAX_TIME     600   // ms - 加载
#define BOOT_STAGE5_MAX_TIME     500   // ms - handoff/跳转
#define BOOT_TOTAL_MAX_TIME      2000  // ms - 总启动时间（含 Stage1-5）

// 内存约束
#define BOOTLOADER_MAX_SIZE      256   // KB - Bootloader 可执行镜像（Flash 占用）
#define BOOTLOADER_STACK_SIZE    32    // KB
#define BOOTLOADER_BSS_SIZE      64    // KB
#define BOOTLOADER_HEAP_SIZE     16    // KB
#define BOOTLOADER_TOTAL         368   // KB - 运行期 RAM 占用（代码+栈+BSS+堆）

// 存储约束
#define BOOTLOADER_PARTITION     512   // KB - 留足镜像+签名头+对齐
#define CONFIG_PARTITION         256   // KB
#define KERNEL_PARTITION_MAX     128   // MB
#define RECOVERY_PARTITION_MAX   64    // MB

// 性能约束
#define MIN_DRAM_SPEED           400   // MHz
#define MIN_FLASH_READ_SPEED     25    // MB/s
#define MAX_SHA256_TIME          100   // ms (128MB固件)
```

---

## 2. 功能边界定义

### 2.1 Bootloader范围（包括）
```
✓ 第一阶段启动代码（汇编）
  - CPU寄存器初始化
  - 堆栈建立
  - 跳转到C代码

✓ 硬件初始化（HAL层）
  - 时钟配置
  - 内存(DRAM)初始化
  - 中断系统初始化
  - 外设驱动（UART、SPI、GPIO、WDT）

✓ 系统自检（POST）
  - 内存完整性检测
  - Flash存储可访问性检测
  - 时钟精度验证
  - CRC自检

✓ 存储访问
  - Flash读取驱动
  - 分区表管理
  - 坏块处理（NAND）

✓ 固件验证
  - CRC32/64快速检验
  - SHA256安全校验
  - RSA/ECDSA数字签名验证
  - 版本号检查（防回滚）

✓ 内核加载
  - 从分区读取内核镜像
  - 解压缩（可选）
  - Device Tree加载

✓ 启动参数处理
  - 命令行参数传递
  - Device Tree配置

✓ 故障处理
  - 异常捕获
  - 备用固件启动
  - 恢复模式支持
```

### 2.2 Bootloader范围（不包括）
```
✗ 内核功能
  - 进程管理
  - 内存管理
  - 文件系统
  - 网络协议栈

✗ 应用程序
  - 业务逻辑
  - 用户界面
  - 第三方库

✗ 运行时系统
  - Shell环境
  - 包管理器
  - 系统工具

✗ 硬件设备树详细配置
  - 仅加载和传递DTB，不执行硬件配置

✗ 动态扩展加载
  - Bootloader固定大小，不支持模块加载
```

### 2.3 功能矩阵
| 功能 | 必需 | 可选 | 不支持 | 备注 |
|------|------|------|--------|------|
| 第一阶段启动 | ✓ | - | - | 汇编代码 |
| 时钟初始化 | ✓ | - | - | 关键路径 |
| DRAM初始化 | ✓ | - | - | 关键路径 |
| 中断初始化 | ✓ | - | - | 必需 |
| UART驱动 | ✓ | - | - | 调试必需 |
| SPI驱动 | ✓ | - | - | Flash访问 |
| CRC校验 | ✓ | - | - | 快速检测 |
| SHA256 | ✓ | - | - | 安全性 |
| 数字签名 | ✓ | - | - | 安全启动 |
| 内存自检 | - | ✓ | - | 可选 |
| SD卡启动 | - | ✓ | - | 恢复模式 |
| USB启动 | - | - | ✓ | 不支持 |
| 网络启动 | - | - | ✓ | 后续版本 |
| 文件系统 | - | - | ✓ | 内核责任 |

---

## 3. 开发目标定义

### 3.1 总体目标
```
实现一个工业级安全启动系统，满足：
1. 功能完整性：全面支持ARM/x86平台启动
2. 安全可靠：包含完整的校验和防护机制
3. 性能高效：启动时间 < 2秒
4. 易于维护：清晰的架构和API接口
5. 可扩展性：支持不同硬件平台适配
```

### 3.2 具体量化目标

#### Phase 1: 基础启动（第1-2周）
**目标**: 系统能够启动到能执行C代码
- 第一阶段启动代码完成（汇编）
- 堆栈初始化
- 基本硬件初始化
- **输出**: 能输出Hello信息到UART

**交付物**:
```
├── src/arch/arm/boot.s          // 汇编启动代码
├── src/core/crt0.c              // C运行时初始化
├── src/hal/clock.c              // 时钟初始化
└── src/hal/uart.c               // UART驱动
```

#### Phase 2: 硬件初始化（第3-4周）
**目标**: 完整的硬件初始化和自检
- DRAM初始化和测试
- MMU启用
- 中断系统
- SPI/Flash驱动
- 看门狗初始化

**交付物**:
```
├── src/core/mmu.c               // 内存管理
├── src/core/irq.c               // 中断系统
├── src/driver/flash.c           // Flash驱动
├── src/driver/watchdog.c        // 看门狗
└── src/hal/dram.c               // DRAM初始化
```

**测试目标**: 能成功初始化系统，输出诊断信息

#### Phase 3: 校验机制（第5-6周）
**目标**: 实现完整的校验验证
- CRC32/SHA256实现
- RSA/ECDSA签名验证
- 版本检查逻辑
- 备用固件支持

**交付物**:
```
├── src/crypto/crc.c             // CRC计算
├── src/crypto/sha256.c          // SHA256算法
├── src/crypto/rsa.c             // RSA验证
├── src/verify/verify.c          // 验证主体
└── src/verify/rollback.c        // 回滚保护
```

**测试目标**: 能验证内核镜像的完整性和真实性

#### Phase 4: 内核启动（第7-8周）
**目标**: 完整启动链
- 内核镜像加载
- Device Tree处理
- 启动参数传递
- 故障恢复

**交付物**:
```
├── src/loader/loader.c          // 加载管理
├── src/loader/image.c           // 镜像格式解析
├── src/loader/dtb.c             // Device Tree
└── src/recovery/recovery.c      // 恢复模式
```

**测试目标**: 系统能成功启动Linux内核

#### Phase 5: 优化和测试（第9周）
**目标**: 性能优化、文档完善、生产部署
- 性能优化（启动时间 < 2s）
- 安全审计
- 文档完成
- 生产版本发布

---

## 4. 平台支持矩阵

### 4.1 主要支持平台
| 平台 | 架构 | 状态 | 交付阶段 |
|------|------|------|---------|
| ARM Cortex-A9 | 32-bit | 支持 | Phase 4 |
| ARM Cortex-A7 | 32-bit | 支持 | Phase 4 |
| ARM Cortex-A53 | 64-bit | 支持 | Phase 5 |
| x86_64 | 64-bit | 计划 | 后续版本 |

### 4.2 芯片支持（演进规划）
```
V1.0（当前）
├── 支持ARM通用平台
├── SPI NOR Flash
└── eMMC支持基础

V1.5（Q2 2026）
├── NAND Flash支持
├── 多芯片适配
└── x86_64初步支持

V2.0（Q3 2026）
├── TEE集成
├── 完整x86_64支持
└── 高级启动选项
```

---

## 5. API抽象层级

### 5.1 硬件抽象层（HAL）
```c
// 最底层：硬件直接操作
// 包括：寄存器操作、中断处理、时钟管理

int hal_clock_init(void);
int hal_gpio_set(uint32_t pin, uint32_t value);
int hal_uart_write(uint8_t *data, uint32_t len);
```

### 5.2 驱动层（DRIVER）
```c
// 中间层：标准化硬件驱动
// 包括：Flash、UART、SPI、看门狗

int driver_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);
int driver_uart_init(uint32_t baudrate);
int driver_watchdog_feed(void);
```

### 5.3 核心服务层（CORE）
```c
// 上层：系统级功能
// 包括：启动管理、校验、加载

int boot_verify_image(uint32_t addr, uint32_t len);
int boot_load_kernel(void);
int boot_start_kernel(void);
```

### 5.4 平台适配层（PLATFORM）
```c
// 平台特定：硬件差异处理
// 包括：不同芯片的初始化差异

int platform_clock_init_armv7(void);
int platform_clock_init_armv8(void);
```

---

## 6. 质量指标

### 6.0 验收口径概览
- 指标来源：本章表格 + 02/03/04 设计的接口与流程约束。
- 验证方式：单测/集成/板测/性能基准，映射见第7章。
- 合格线：启动总时长 < 2s；Bootloader 占用 < 256KB；签名验证全覆盖；回滚防护生效。

### 6.1 功能覆盖率
| 模块 | 覆盖率目标 | 测试方式 |
|------|----------|---------|
| 启动代码 | 100% | 汇编代码审查 |
| HAL层 | ≥95% | 单元测试 |
| 驱动层 | ≥90% | 集成测试 |
| 核心服务 | ≥95% | 功能测试 |
| 总体 | ≥92% | 完整测试套件 |

### 6.2 代码质量
| 指标 | 目标 | 工具 |
|------|------|------|
| 代码复杂度 | McCabe < 10 | 静态分析 |
| 注释覆盖 | ≥80% | 代码审查 |
| 内存泄漏 | 0个 | Valgrind |
| 栈溢出 | 0个 | 静态分析 |
| 缓冲区溢出 | 0个 | Fuzzing |

### 6.3 安全指标
| 指标 | 目标 | 验证方式 |
|------|------|---------|
| 签名覆盖 | 100% | 代码审查 |
| 校验有效性 | 99.99% | 测试 |
| 密钥安全 | 满足FIPS | 安全审计 |
| 版本保护 | 无回滚 | 测试 |

### 6.4 性能指标
| 指标 | 目标 | 环境 |
|------|------|------|
| 启动时间 | < 2s | 标准配置 |
| 内存占用 | < 256KB | Bootloader |
| 校验速度 | > 50MB/s | SHA256计算 |
| Flash读速 | > 25MB/s | QSPI |

---

## 7. 约束和假设

### 7.0 验证映射表（对齐 TEST_STRATEGY）
| 需求/指标 | 验证方式 | 入口 | 判定标准 |
|-----------|----------|------|----------|
| 启动时间 < 2s | 板测性能用例 | test/integration/test_boot_flow.c | 95% 样本 < 2s，最大值 < 2.2s |
| Bootloader 尺寸 < 256KB | 构建产物检查 | boot/bootloader.bin | 二进制大小 ≤ 256KB |
| SHA256 吞吐 > 50MB/s | 性能基准 | test/perf/test_sha256.c | 1MB 块 ≥ 50MB/s |
| QSPI 读速 > 25MB/s | 板测 IO 基准 | test/perf/test_flash_access.c | 顺序读 ≥ 25MB/s |
| 签名覆盖 100% | 安全测试/审计 | TEST_STRATEGY 安全组 | 所有镜像/DTB 必须签名且验证 |
| 防回滚生效 | 集成用例 | test/integration/test_recovery.c | 旧版本拒绝启动，触发备用/恢复 |
| 看门狗阶段喂狗 | 板测 | test/integration/test_boot_flow.c | 各阶段满足超时预算、无误复位 |


### 7.1 假设
```
1. 存储设备始终可用且功能正常
2. OTP区域已正确写入密钥
3. 硬件平台支持标准中断机制
4. DRAM尺寸 ≥ 512MB
5. 系统时钟 ≥ 100MHz
6. 看门狗可定时触发复位
```

### 7.2 已知限制
```
1. 不支持USB启动（第一版本）
2. 不支持网络启动（第一版本）
3. 不支持UEFI启动（保留未来版本）
4. OTP一旦写入无法修改
5. 签名密钥存储在片上OTP，无法更新
```

### 7.3 风险
```
高风险:
- 签名密钥泄露 → 实现完整的密钥管理
- Flash读取失败 → 多冗余备份
- 时序超期 → 看门狗保护

中风险:
- 内存不足 → 内存监控和优化
- 性能不达标 → 性能测试和优化
- 兼容性问题 → 广泛测试

低风险:
- 文档不完善 → 持续改进
- 用户易用性 → 反馈改进
```

