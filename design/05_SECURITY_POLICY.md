# SECURITY_POLICY

**版本**: 0.1 (草案)
**日期**: 2026-01
**目的**: 规范密钥管理、签名流程、调试控制与生产烧录，以支撑安全启动验收。

---

## 1. 根信任与密钥管理
- 根密钥存储：OTP/安全区，单向写入；禁止导出。
- 公钥版本：记录 key_id 与最小安全版本，随镜像发布。
- 密钥轮换：仅允许公钥更新；私钥独立离线存储，签名机隔离。
- 访问控制：签名工具需认证；审计日志保留。

## 2. 签名与校验流程
- 算法：RSA-2048 / ECDSA-P256，摘要 SHA256。
- 产物：镜像头含 sha256 与 signature，携带 key_id 与 algo。
- 流程：`build → hash → sign → attach header → verify (offline) → 发布`。
- 校验点：Bootloader 验证自身（如适用）、内核、DTB；失败触发回退/恢复。

### 2.1 镜像元数据示例（JSON）
```json
{
	"image": {
		"type": "kernel",
		"version": 120,
		"sha256": "<32-byte-hex>",
		"signature": "<base64>",
		"algo": "RSA-2048",
		"key_id": 3
	},
	"rollback": {
		"security_version": 100,
		"timestamp": 1736200000
	}
}
```

### 2.2 CLI 工作流（示例）
```bash
# 生成镜像头与签名
python3 tools/mkimage.py --in build/bootloader.bin --out boot/boot.img --type kernel --version 120
python3 tools/sign.py --in boot/boot.img --key tools/keys/priv.pem --out boot/boot.signed --algo RSA-2048 --key-id 3
python3 tools/mkpart.py --kernel boot/boot.signed --dtb boot/dtb.bin --out boot/flash.img
python3 tools/otp_write.py --pubkey tools/keys/pub.pem --minver 100 --key-id 3
```

### 2.3 密钥撤销与应急响应
- 撤销表：维护 `revoked_key_ids` 与 `min_security_version` 列表，存储于 OTP 受保护区；Bootloader 启动前加载。
- 拒绝策略：若 `key_id` 在撤销表中或 `image.version < min_security_version`，则拒绝并进入备用固件/恢复模式。
- 应急更新：提供安全维护镜像（经未撤销 key 签名）写入新的撤销表与最小版本；更新后需重新校验并记录审计事件。
- 审计与告警：撤销触发和拒绝启动事件写入只追加日志，并可选通过 UART/LED 告警（不泄露密钥数据）。

## 3. 防回滚策略
- 元数据：current_version、security_version、timestamp。
- 规则：new_version >= security_version，否则拒绝并尝试备用固件。
- 存储：版本信息存储于只读/受保护区域，写入需校验。

## 4. 调试与日志控制
- 生产默认关闭调试口；启用需认证令牌或物理跳线。
- 日志级别：生产仅保留错误/告警；调试版可开启详细日志。
- 敏感信息：禁止在日志中输出密钥/摘要/地址等敏感数据。

### 4.1 调试认证策略
- 认证令牌：一次性挑战/响应；令牌使用 ECDSA 验签；有效期短。
- 物理防护：跳线/按键进入维护模式需上电窗口与本地确认。
- 审计：所有调试开启/关闭事件写入审计日志（只追加）。

## 5. 生产烧录与发布
- 烧录流程：`mkimage → sign → mkpart → otp_write`；每步需校验摘要与签名。
- 物料：公钥版本表、最小安全版本、镜像包（含签名）、烧录脚本。
- 发布检查单：签名有效；回滚门限正确；调试关闭；尺寸/性能符合 01_SYSTEM_PROPERTIES。

### 5.1 发布检查单（细化）
- [ ] 镜像 `sha256` 与签名匹配；`key_id` 与 OTP 一致。
- [ ] 最小安全版本 `security_version` 写入并验证。
- [ ] 调试开关关闭；日志级别符合生产策略。
- [ ] 尺寸/性能/启动时间满足 01_SYSTEM_PROPERTIES；板测报告存档。

## 6. 威胁与缓解（摘要）
- 密钥泄露 → OTP-only、公钥轮换、离线签名、审计。
- 篡改/伪造 → 全链签名、SHA256 校验、只读存储。
- 回滚攻击 → 最小安全版本门限、备用固件验证。
- 旁路调试 → 认证开关、物理防护、日志裁剪。

> 与 02_BOOTLOADER_ARCHITECTURE 的安全链一致；与 06_TEST_STRATEGY 的安全用例联动。
