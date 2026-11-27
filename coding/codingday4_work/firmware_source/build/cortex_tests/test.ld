ENTRY(test_entry)
SECTIONS { . = 0x10000; .text : { *(.text*) *(.rodata*) }
 .ARM.extab : { *(.ARM.extab*) } .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; }
 . = 0x20000000; .data : { *(.data*) } .bss : { *(.bss*) *(COMMON) }
 . = ALIGN(8); end = .; _end = .; }
