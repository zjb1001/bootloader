# ─── ARM Cortex-A cross-compilation toolchain ───
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CROSS_COMPILE "arm-none-eabi-" CACHE STRING "Cross compiler prefix")

set(CMAKE_C_COMPILER   ${CROSS_COMPILE}gcc)
set(CMAKE_ASM_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_OBJCOPY      ${CROSS_COMPILE}objcopy)
set(CMAKE_OBJDUMP      ${CROSS_COMPILE}objdump)
set(CMAKE_SIZE         ${CROSS_COMPILE}size)

set(CMAKE_C_FLAGS_INIT   "-mcpu=cortex-a9 -mthumb -mfloat-abi=soft")
set(CMAKE_ASM_FLAGS_INIT "-mcpu=cortex-a9 -mthumb")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
