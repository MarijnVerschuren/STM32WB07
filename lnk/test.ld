_MEMORY_RAM_BEGIN_   = 0x20000000;
_MEMORY_RAM_SIZE_    = 0x10000;			/*  64KB  */
_MEMORY_RAM_END_     = 0x2000FFFF;

_MEMORY_FLASH_BEGIN_ = 0x10040000;
_MEMORY_FLASH_SIZE_  = 0x40000;          /* 256KB */
_MEMORY_FLASH_END_   = 0x1007FFFF;

_MEMORY_ROM_BEGIN_   = 0x10000000;
_MEMORY_ROM_SIZE_    = 0x01800;          /* 6KB */
_MEMORY_ROM_END_     = 0x100017FF;


/* Reserved for BTLE stack non volatile memory */
FLASH_NVM_DATASIZE   = (4*1024);


MEMORY_FLASH_APP_OFFSET = DEFINED(MEMORY_FLASH_APP_OFFSET) ? (MEMORY_FLASH_APP_OFFSET) : (0) ;
MEMORY_FLASH_APP_SIZE = DEFINED(MEMORY_FLASH_APP_SIZE) ? (MEMORY_FLASH_APP_SIZE) : (_MEMORY_FLASH_SIZE_ - FLASH_NVM_DATASIZE - MEMORY_FLASH_APP_OFFSET);


/* Entry Point */
ENTRY(reset_handler)


_min_heap_size = 0x0; /* required amount of heap  */
_min_stack_size = 0xC00; /* required amount of stack */

/* Specify the memory areas */
MEMORY {
  RAM (xrw)              : ORIGIN = _MEMORY_RAM_BEGIN_, LENGTH = _MEMORY_RAM_SIZE_
  FLASH (rx)             : ORIGIN = _MEMORY_FLASH_BEGIN_ + MEMORY_FLASH_APP_OFFSET, LENGTH = MEMORY_FLASH_APP_SIZE
  REGION_NVM (rx)               : ORIGIN = _MEMORY_FLASH_END_ + 1 - FLASH_NVM_DATASIZE, LENGTH = FLASH_NVM_DATASIZE
  REGION_ROM (rx)               : ORIGIN = _MEMORY_ROM_BEGIN_, LENGTH = _MEMORY_ROM_SIZE_
}

/* Define output sections */
SECTIONS {
	_vector_table = LOADADDR(.vector);

	.vector (ORIGIN(FLASH)): {
		. = ALIGN(4);
		_IVT_start = .;
		KEEP(*(.vector))
		. = ALIGN(4);
		_ITV_end = .;
	} > FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
	_text_start = .;
    KEEP(*(TAG_OTA_START))

    KEEP(*(.cmd_call_table))
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(i.*)             /* i.* sections (code) */
    *(.constdata)
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
	. = ALIGN(4);
	_text_end = .;
  } >FLASH

	.rodata ALIGN(4) : {
		KEEP(*(.rodata))
		KEEP(*(.rodata*))
		. = ALIGN(4);
	} > FLASH

  .ARM.extab (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    *(.ARM.extab*)
  } >FLASH

  .ARM (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  /* used by the startup to initialize data */
  _data_load = LOADADDR(.data);

  /* RAM preamble, uninitialized */
  .ram_preamble 0x20000004 (NOLOAD) :
  {
    KEEP(*(.ram_vr))
  }  >RAM

  /* RAM preamble, unininitialized */
  .ram_preamble_2 0x20000034 (NOLOAD) :
  {
    KEEP(*(.crash_info_ram_vr))
  }  >RAM
  /* Uninitialized data section */

  .bss.blueRAM 0x200000C0 :
  {
    . = ALIGN(4);
    _sbssblue = .;         /* define a global symbol at .bss.blueRAM start */
    KEEP(*(.bss.__blue_RAM))
    . = ALIGN(4);
    _ebssblue = .;         /* define a global symbol at .bss.blueRAM end */
  } >RAM

  .bss DEFINED(MEMORY_RAM_APP_OFFSET) ? (ORIGIN(RAM) + MEMORY_RAM_APP_OFFSET) : . :
  {
    . = ALIGN(4);
    _bss_start = .;         /* define a global symbol at bss start */
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _bss_end = .;         /* define a global symbol at bss end */
  } >RAM

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _data_start = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    . = ALIGN(4);
    _data_end = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH

  /* Data section that will not be initialized to any value. */
  .noinit (NOLOAD):
  {
    . = ALIGN(4);
    *(.noinit)
    . = ALIGN(4);
  } >RAM

  .ota_region_end :
  {
	. = ALIGN(4);
	KEEP(*(TAG_OTA_END))
	. = ALIGN(4);
  } >FLASH

  .heap (NOLOAD):
  {
    . = ALIGN(4);
    _heap_start = .;
    PROVIDE( end = . );
    PROVIDE( _end = . );
    . = . + _min_heap_size;
    . = ALIGN(4);
    _heap_end = .;
  } >RAM

  .stack (ORIGIN(RAM) + LENGTH(RAM) - _min_stack_size) (NOLOAD) :
  {
    . = ALIGN(4);
    . = . + _min_stack_size;
    . = ALIGN(4);
    _stack_end = .;         /* define a global symbol at stack end */
    . = ALIGN(4);
  } > RAM


  .rom_info (NOLOAD) :
  {
    . = ALIGN(4);
    KEEP(*(.rom_info))
    . = ALIGN(4);
  } >REGION_ROM

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
