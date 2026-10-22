/* Host boot branches to the base of L2, not to the ELF entry recorded in a boot table.
   Put the existing test startup there regardless of object-file link order. */
MEMORY
{
    L2 : origin = 0x00800000, length = 0x00200000
}

SECTIONS
{
    .text 0x00800000 : { crt.obj(.text) *(.text) }
    .const      > L2
    .rodata     > L2
    .switch     > L2
    .cinit      > L2
    .pinit      > L2
    .init_array > L2
    .binit      > L2
    .cio        > L2
    .data       > L2
    .neardata   > L2
    .fardata    > L2
    .bss        > L2
    .far        > L2
    .stack      > L2
    .sysmem     > L2
}
