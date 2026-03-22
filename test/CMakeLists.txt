# ─── Test Build ───
# Tests use mock HAL/driver backends for host-side testing

# For host builds, only include architecture-independent source files
# All hardware-specific functionality is mocked

set(TEST_SOURCES
    # Core logic (hardware-agnostic)
    ${CMAKE_SOURCE_DIR}/src/core/partition.c
    ${CMAKE_SOURCE_DIR}/src/core/loader.c
    ${CMAKE_SOURCE_DIR}/src/core/recovery.c
    ${CMAKE_SOURCE_DIR}/src/core/startup.c
    ${CMAKE_SOURCE_DIR}/src/core/verify.c

    # Crypto algorithms (portable)
    ${CMAKE_SOURCE_DIR}/src/driver/crypto/crc.c
    ${CMAKE_SOURCE_DIR}/src/driver/crypto/sha256.c

    # Utilities (portable)
    ${CMAKE_SOURCE_DIR}/src/utils/string.c
    ${CMAKE_SOURCE_DIR}/src/utils/math.c
    ${CMAKE_SOURCE_DIR}/src/utils/assert.c
    ${CMAKE_SOURCE_DIR}/src/utils/common.c
)

set(MOCK_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/mock/mock_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mock/mock_flash.c
)

# Include path for test framework header
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)

# Disable -Werror for tests to allow compilation on host
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-error")

# ─── Unit tests ───
function(add_unit_test NAME SOURCE)
    add_executable(${NAME} ${SOURCE} ${TEST_SOURCES} ${MOCK_SOURCES})
    target_compile_definitions(${NAME} PRIVATE UNIT_TEST=1)
    target_compile_definitions(${NAME} PRIVATE PLATFORM=host)
    add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

add_unit_test(test_crc         unit/test_crc.c)
add_unit_test(test_sha256      unit/test_sha256.c)

# ─── Integration tests ───
function(add_integration_test NAME SOURCE)
    add_executable(${NAME} ${SOURCE} ${TEST_SOURCES} ${MOCK_SOURCES})
    target_compile_definitions(${NAME} PRIVATE INTEGRATION_TEST=1)
    target_compile_definitions(${NAME} PRIVATE PLATFORM=host)
    add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

add_integration_test(test_boot_flow   integration/test_boot_flow.c)
add_integration_test(test_recovery    integration/test_recovery.c)

# ─── Performance tests ───
function(add_perf_test NAME SOURCE)
    add_executable(${NAME} ${SOURCE} ${TEST_SOURCES} ${MOCK_SOURCES})
    target_compile_definitions(${NAME} PRIVATE PERF_TEST=1)
    target_compile_definitions(${NAME} PRIVATE PLATFORM=host)
    add_test(NAME ${NAME} COMMAND ${NAME})
endfunction()

add_perf_test(test_sha256_perf    perf/test_sha256_perf.c)
add_perf_test(test_flash_perf     perf/test_flash_perf.c)
