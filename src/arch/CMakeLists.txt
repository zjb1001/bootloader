# ─── Architecture-specific objects ───
if(PLATFORM STREQUAL "armv7")
    set(ARCH_SOURCES
        arm/armv7/boot.S
        arm/armv7/irq.S
        arm/armv7/cache.S
        arm/armv7/mmu.S
        arm/armv7/sys.c
    )
elseif(PLATFORM STREQUAL "armv8")
    set(ARCH_SOURCES
        arm/armv8/boot.S
        arm/armv8/irq.S
        arm/armv8/cache.S
        arm/armv8/mmu.S
        arm/armv8/sys.c
    )
elseif(PLATFORM STREQUAL "x86_64")
    set(ARCH_SOURCES
        x86/boot.S
        x86/gdt.S
        x86/idt.S
        x86/sys.c
    )
else()
    # Host / test stub
    set(ARCH_SOURCES
        stub/arch_stub.c
    )
endif()

add_library(arch OBJECT ${ARCH_SOURCES})
