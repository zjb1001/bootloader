# ─── Compiler warnings ───
add_compile_options(
    -Wall
    -Wextra
    -Werror
    -Wno-unused-parameter
    -Wshadow
    -Wdouble-promotion
    -Wformat=2
    -Wformat-truncation
    -Wundef
    -fno-common
    -ffunction-sections
    -fdata-sections
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g3 -O0)
    add_compile_definitions(DEBUG=1)
else()
    add_compile_options(-Os)
    add_compile_definitions(NDEBUG=1)
endif()
