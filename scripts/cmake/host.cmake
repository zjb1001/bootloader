# ─── Host (native) toolchain for tests ───
set(CMAKE_SYSTEM_NAME ${CMAKE_HOST_SYSTEM_NAME})
set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_HOST_SYSTEM_PROCESSOR})

set(CMAKE_OBJCOPY objcopy)
set(CMAKE_OBJDUMP objdump)
set(CMAKE_SIZE    size)
