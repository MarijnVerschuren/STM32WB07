/* entry point */
ENTRY(reset_handler)

/* memory map */
MEMORY {
    FLASH(rx)	: ORIGIN = 0x10040000, LENGTH = 256K
    SRAM(rwx)	: ORIGIN = 0x20000000, LENGTH = 64K
	NVM (rx)	: ORIGIN = 0x10079000, LENGTH = 4K
	ROM (rx)	: ORIGIN = 0x10000000, LENGTH = 6K
}

/* defines */
_min_heap_size = 0x0;
_min_stack_size = 0xC00;

/* section scripts */
INCLUDE "lnk/flash.ld"
INCLUDE "lnk/sram.ld"