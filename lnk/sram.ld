SECTIONS {
	_data_load = LOADADDR(.data);

	.ram_preamble 0x20000004 (NOLOAD) : {
		KEEP(*(.ram_vr))
	} > SRAM
	.ram_preamble_2 0x20000034 (NOLOAD) : {
		KEEP(*(.crash_info_ram_vr))
	} > SRAM

	.RF_RAM 0x200000C0 : {
		. = ALIGN(4);
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
	} > SRAM

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

	.heap ALIGN(4) (NOLOAD) : {
		PROVIDE(_heap_end = .);
		. = . + _min_heap_size;
		. = ALIGN(4);
	} > SRAM

	.stack (ORIGIN(SRAM) + LENGTH(SRAM) - _min_stack_size) (NOLOAD) : {
		. = ALIGN(4);
		. = . + _min_stack_size;
		. = ALIGN(4);
		_stack_end = .;
		. = ALIGN(4);
	} > SRAM

	. = ALIGN(4);
	_heap_start = .;
}