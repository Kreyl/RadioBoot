
MEMORY
{
   RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x10000
}

SECTIONS
{
    .text :
    {
        KEEP(*(.text.flash_update));
        *(.text .text.* .gnu.linkonce.t.*)
        *(.glue_7t) *(.glue_7)
        *(.rodata .rodata.* .gnu.linkonce.r.*)
        . = ALIGN(4);
        _etext = .;
    } > RAM

    _end = . ;
}
