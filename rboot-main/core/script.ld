MEMORY
{
 FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 0x2000
 RAM (rw) : ORIGIN = 0x20000000, LENGTH = 0x10000000
}

SECTIONS
{
 .text :
 {
  KEEP(*(.isr_vectors));
  KEEP(*(.reset));
  *(.text .text.* .gnu.linkonce.t.*)
  *(.glue_7t) *(.glue_7)
  *(.rodata .rodata.* .gnu.linkonce.r.*)
  . = ALIGN(4);
  _etext = .;
 } > FLASH

 _end = . ;
}