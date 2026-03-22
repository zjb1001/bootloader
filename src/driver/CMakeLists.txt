add_library(driver OBJECT
    flash/flash.c
    flash/nor_spi.c
    flash/nand_spi.c
    flash/emmc.c
    flash/ecc.c
    crypto/crc.c
    crypto/sha256.c
    crypto/rsa.c
    crypto/ecdsa.c
    crypto/bignum.c
    watchdog.c
    spi.c
    console.c
)
