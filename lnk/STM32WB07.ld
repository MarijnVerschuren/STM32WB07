MEMORY {
    FLASH(rx) : ORIGIN = 0x10040000, LENGTH = 256K
    SRAM(rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

INCLUDE "lnk/flash.ld"
INCLUDE "lnk/sram.ld"