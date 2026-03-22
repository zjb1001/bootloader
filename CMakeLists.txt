cmake_minimum_required(VERSION 3.10)
project(bootloader VERSION 2.0.0 LANGUAGES C ASM)

# ─── C Standard ───
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# ─── Options ───
option(ENABLE_SECURE_BOOT    "Enable secure boot chain"        ON)
option(ENABLE_SIGNATURE      "Enable digital signature verify" ON)
option(ENABLE_ROLLBACK       "Enable rollback protection"      ON)
option(ENABLE_RECOVERY       "Enable recovery mode"            ON)
option(ENABLE_WATCHDOG       "Enable watchdog timer"           ON)
option(ENABLE_POST           "Enable Power-On Self Test"       ON)
option(BUILD_TESTS           "Build unit/integration tests"    OFF)

set(PLATFORM "armv7" CACHE STRING "Target platform: armv7, armv8, x86_64, host")
set(BOARD    ""      CACHE STRING "Board name (optional)")

# ─── Compile definitions from options ───
if(ENABLE_SECURE_BOOT)
    add_compile_definitions(CONFIG_SECURE_BOOT=1)
endif()
if(ENABLE_SIGNATURE)
    add_compile_definitions(CONFIG_SIGNATURE=1)
endif()
if(ENABLE_ROLLBACK)
    add_compile_definitions(CONFIG_ROLLBACK_PROTECT=1)
endif()
if(ENABLE_RECOVERY)
    add_compile_definitions(CONFIG_RECOVERY=1)
endif()
if(ENABLE_WATCHDOG)
    add_compile_definitions(CONFIG_WATCHDOG=1)
endif()
if(ENABLE_POST)
    add_compile_definitions(CONFIG_POST=1)
endif()

# ─── Include paths ───
include_directories(${CMAKE_SOURCE_DIR}/include)

# ─── Compiler warnings ───
include(scripts/cmake/warnings.cmake)

# ─── Source subdirectories ───
add_subdirectory(src)

# ─── Tests ───
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()

# ─── Firmware target (non-test build) ───
if(NOT BUILD_TESTS)
    add_executable(bootloader.elf
        $<TARGET_OBJECTS:arch>
        $<TARGET_OBJECTS:hal>
        $<TARGET_OBJECTS:driver>
        $<TARGET_OBJECTS:core>
        $<TARGET_OBJECTS:platform>
        $<TARGET_OBJECTS:utils>
    )

    target_link_options(bootloader.elf PRIVATE
        -T${CMAKE_SOURCE_DIR}/boot/bootloader.ld
        -Wl,--gc-sections
        -nostdlib
        -Wl,-Map=${CMAKE_BINARY_DIR}/bootloader.map
    )

    # Generate raw binary
    add_custom_command(TARGET bootloader.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:bootloader.elf>
            ${CMAKE_BINARY_DIR}/bootloader.bin
        COMMENT "Generating bootloader.bin"
    )

    # Print size info
    add_custom_command(TARGET bootloader.elf POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:bootloader.elf>
        COMMENT "Bootloader size:"
    )
endif()
