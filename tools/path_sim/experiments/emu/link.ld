ENTRY(bench_main)
MEMORY { RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 12M }
SECTIONS {
  .text : { *(.text*) *(.time_critical*) *(.rodata*) *(.ARM.extab*) } > RAM
  .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > RAM
  .preinit_array : { __preinit_array_start = .; KEEP(*(.preinit_array*)) __preinit_array_end = .; } > RAM
  .init_array : { __init_array_start = .; KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) __init_array_end = .; } > RAM
  .fini_array : { __fini_array_start = .; KEEP(*(.fini_array*)) __fini_array_end = .; } > RAM
  .data : { *(.data*) } > RAM
  .bss : { __bss_start__ = .; *(.bss*) *(COMMON) __bss_end__ = .; } > RAM
  . = ALIGN(8);
  end = .;
  __end__ = .;
}
