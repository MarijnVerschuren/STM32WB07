SECTIONS {
	_stack_end = ORIGIN(SRAM) + LENGTH(SRAM);
	_data_load = LOADADDR(.data);
	. = ORIGIN(SRAM);

	.RF_RAM ALIGN(4) : {
		_RF_RAM_start = .;
		KEEP(*(.RF_RAM))
		. = ALIGN(4);
		_RF_RAM_end = .;
	} > SRAM

	.data ALIGN(4) : {
		_data_start = .;
		*(.data)
		*(.data*)
		. = ALIGN(4);
		_data_end = .;
	} > SRAM AT > FLASH

	.bss ALIGN(4) (NOLOAD) : {
		_bss_start = .;
		*(.bss)
		*(.bss*)
		. = ALIGN(4);
		_bss_end = .;
	} > SRAM

	.noinit ALIGN(4) (NOLOAD) : {  /* TODO: needed? */
		*(.noinit)
		. = ALIGN(4);
	} > SRAM

	. = ALIGN(4);
	_heap_start = .;
}