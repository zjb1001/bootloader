/**
 * @file main.c
 * @brief Bootloader entry point after C runtime init
 */
#include "boot.h"

int main(void)
{
    return boot_main();
}
