# TEST_STRATEGY

**版本**: 0.1 (草案)
**日期**: 2026-01
**目的**: 定义单测/集成/板测/性能/安全用例矩阵，验证 01_SYSTEM_PROPERTIES 与安全/架构要求。

---

## 1. 测试分层
- 单元测试：crypto、partition、verify 等纯函数模块；Mock HAL/Driver。
- 集成测试：boot_flow、flash_access、recovery，使用轻量板级或仿真。
- 板上冒烟：时钟/DRAM/IRQ/UART 基础通路；看门狗喂狗验证。
- 性能基准：SHA256 吞吐、Flash 读速、启动总时长。
- 安全测试：签名必需、回滚防护、调试开关、篡改检测。

## 2. 用例矩阵（示例）
| 领域 | 用例 | 入口 | 判定 | 指标来源 |
|------|------|------|------|----------|
| 启动流程 | boot_flow | test/integration/test_boot_flow.c | 启动 <2s，阶段喂狗 | 01_SYSTEM_PROPERTIES |
| 校验 | verify_signature | test/integration/test_verify.c | 错签拒绝，回退触发 | 02_BOOTLOADER_ARCHITECTURE |
| 性能 | sha256_perf | test/perf/test_sha256.c | ≥50MB/s | 01_SYSTEM_PROPERTIES |
| 存储 | flash_access | test/perf/test_flash_access.c | ≥25MB/s 顺序读 | 01_SYSTEM_PROPERTIES |
| 回滚 | rollback_protect | test/integration/test_recovery.c | 旧版拒绝，备用成功 | SECURITY_POLICY |
| 调试控制 | debug_lock | test/integration/test_debug.c | 无认证拒绝调试 | SECURITY_POLICY |

### 2.1 目录与数据布局（建议）
```
test/
	unit/
		test_crc.c
		test_sha256.c
		test_partition.c
	integration/
		test_boot_flow.c
		test_flash_access.c
		test_recovery.c
		test_debug.c
	perf/
		test_sha256.c
		test_flash_access.c
	mock/
		mock_hal.c
		mock_driver.c
	data/
		test_images/
		test_keys/
		test_config/
```

---

## 3. 覆盖率目标
- 单测：HAL/Driver/核心服务关键路径 ≥90%。
- 集成：启动链关键场景 ≥95%。
- 安全：签名/回滚/调试用例 100% 覆盖。

## 4. 性能与超时预算
- 阶段超时：S1 100ms，S2 300ms，S3 500ms，S4 600ms，总 2000ms。
- SHA256：≥50MB/s；QSPI：≥25MB/s；启动：<2s（95% 分位）。

### 4.1 测量方法（示例）
- SHA256 吞吐：以 1MB/4MB/16MB 块在板上测量，剔除 IO 时间，计算哈希纯耗时吞吐。
- QSPI 读速：顺序读 128KB/1MB/16MB，缓冲在 DRAM；统计平均与 95% 分位。
- 启动时间：从上电到内核入口打印，累计总时长；记录各阶段里程碑时间戳。

---

## 5. 测试流程
- 构建：开启测试配置，使用 mock 覆盖 IO；性能/板测使用真实硬件配置。
- 工具链：mkimage/sign/mkpart 生成测试镜像；otp_write 写入公钥/版本。
- 报告：输出性能与安全摘要，作为发布门禁。

### 5.1 通过/失败判据
- 启动：95% 样本 < 2s；最大值 < 2.2s；阶段超时均未触发看门狗复位。
- 安全：错签/篡改/回滚用例全部拒绝并进入回退/恢复；调试认证用例全部符合策略。
- 性能：SHA256/QSPI 达标；Flash 可靠性用例通过（坏块处理）。

> 与 02_BOOTLOADER_ARCHITECTURE 的阶段/超时、05_SECURITY_POLICY 的安全要求、01_SYSTEM_PROPERTIES 的指标表保持一致。
