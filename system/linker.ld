ENTRY(Reset_Handler)

MEMORY {
	flash (rx) : ORIGIN = 0x00000000, LENGTH = 1024K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

_stack_top = ORIGIN(ram) + LENGTH(ram);

SECTIONS {
	.text : {
		KEEP(*(VectorTable))
		*(.text .text.*)
		*(.rodata .rodata.*)
		_etext = .;
	} > flash

	.data : {
		_sdata = .;
		*(.data .data.*)
		_edata = .;
	} > ram AT > flash

	.bss (NOLOAD) : {
		_sbss = .;
		*(.bss .bss.*)
		*(COMMON)
		_ebss = .;
	} > ram
}