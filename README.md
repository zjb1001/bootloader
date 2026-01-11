# Bootloader

Industrial-grade secure boot stack targeting ARM (A7/A9/A53) and x86_64 with a <2s cold-boot budget and <256KB bootloader footprint. Provides staged startup, layered HAL/driver/core architecture, crypto-backed integrity, and rollback-safe dual-partition fallback with DTB handoff.

## Highlights
- Staged chain: ROM → Stage1 (asm) → Stage2 (HAL init) → Stage3 (POST) → Stage4 (Load + Verify) → Stage5 (handoff).
- Security: CRC + SHA256 + RSA/ECDSA, rollback protection via security version, OTP-stored keys/revocation list, debug lock with audit.
- Resilience: dual partitions, recovery mode, watchdog-aligned timing (total 2000ms budget).
- Portability: HAL/Driver/Core/Platform layers; ARM and x86_64 roadmap; XIP window + block access flash model.
- Footprint/perf: bootloader <256KB, partition 512KB, startup <2s, SHA256 >50MB/s, QSPI read >25MB/s.

## Documentation
- System properties and constraints: design/01_SYSTEM_PROPERTIES.md
- Architecture and stages: design/02_BOOTLOADER_ARCHITECTURE.md
- API specification: design/03_API_DESIGN.md
- Project layout and build targets: design/04_PROJECT_ARCHITECTURE.md
- Security policy and key management: design/05_SECURITY_POLICY.md
- Test strategy (unit/integration/perf/board): design/06_TEST_STRATEGY.md

## Build & Tooling (planned shape)
- Toolchain: CMake + cross toolchains (scripts/cmake/Toolchains.cmake).
- Artifacts: boot/bootloader.bin, bootloader.map; tools for mkimage/sign/mkpart/otp_write.
- Tests: test/unit, test/integration, test/perf with mocks and board targets.

## Quick status
This repo currently contains design documents; source, tools, and tests will follow the documented structure in design/04_PROJECT_ARCHITECTURE.md.
