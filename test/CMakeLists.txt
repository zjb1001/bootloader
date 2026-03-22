# ─── Test Build ───
# Tests are compiled with the host (native) compiler, linking against
# the same source modules but with mock HAL/driver backends.

# All source files needed for tests (we reuse production code)
set(COMMON_SOURCES
    ${CMAKE_SOURCE_DIR}/src/core/startup.c
    ${CMAKE_SOURCE_DIR}/src/core/verify.c
    ${CMAKE_SOURCE_DIR}/src/core/loader.c
    ${CMAKE_SOURCE_DIR}/src/core/partition.c
    ${CMAKE_SOURCE_DIR}/src/core/recovery.c
    ${CMAKE_SOURCE_DIR}/src/core/boot.c
    ${CMAKE_SOURCE_DIR}/src/core/image.c
    ${CMAKE_SOURCE_DIR}/src/driver/crypto/crc.c
    ${CMAKE_SOURCE_DIR}/src/driver/crypto/sha256.c
    ${CMAKE_SOURCE_DIR}/src/driver/crypto/rsa.c
    ${CMAKE_SOURCE_DIR}/src/driver/crypto/ecdsa.c
    ${CMAKE_SOURCE_DIR}/src/driver/flash/flash.c
    ${CMAKE_SOURCE_DIR}/src/driver/watchdog.c
    ${CMAKE_SOURCE_DIR}/src/driver/spi.c
    ${CMAKE_SOURCE_DIR}/src/hal/clock.c
    ${CMAKE_SOURCE_DIR}/src/hal/gpio.c
    ${CMAKE_SOURCE_DIR}/src/hal/uart.c
    ${CMAKE_SOURCE_DIR}/src/hal/irq.c
    ${CMAKE_SOURCE_DIR}/src/hal/memory.c
    ${CMAKE_SOURCE_DIR}/src/hal/timer.c
    ${CMAKE_SOURCE_DIR}/src/platform/platform.c
    ${CMAKE_SOURCE_DIR}/src/platform/chip.c
    ${CMAKE_SOURCE_DIR}/src/utils/string.c
    ${CMAKE_SOURCE_DIR}/src/utils/log.c
    ${CMAKE_SOURCE_DIR}/src/utils/assert.c
    ${CMAKE_SOURCE_DIR}/src/utils/common.c
)

set(MOCK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/mock/mock_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mock/mock_flash.c
)

# Include path for test framework header
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# ─── Unit tests ───
function(add_unit_test NAME SOURCE)
    add_executable(${NAME} ${SOURCE} ${COMMON_SOURCES} ${MOCK_SOURCES})
    target_compile_definitions(${NAME} PRIVATE UNIT_TEST=1)
    add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

add_unit_test(test_crc         unit/test_crc.c)
add_unit_test(test_sha256      unit/test_sha256.c)
add_unit_test(test_partition    unit/test_partition.c)
add_unit_test(test_verify       unit/test_verify.c)
add_unit_test(test_version      unit/test_version.c)

# ─── Integration tests ───
function(add_integration_test NAME SOURCE)
    add_executable(${NAME} ${SOURCE} ${COMMON_SOURCES} ${MOCK_SOURCES})
    target_compile_definitions(${NAME} PRIVATE INTEGRATION_TEST=1)
    add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

add_integration_test(test_boot_flow   integration/test_boot_flow.c)
add_integration_test(test_recovery    integration/test_recovery.c)

# ─── Performance tests ───
function(add_perf_test NAME SOURCE)
    add_executable(${NAME} ${SOURCE} ${COMMON_SOURCES} ${MOCK_SOURCES})
    target_compile_definitions(${NAME} PRIVATE PERF_TEST=1)
    add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

add_perf_test(test_sha256_perf    perf/test_sha256_perf.c)
add_perf_test(test_flash_perf     perf/test_flash_perf.c)
