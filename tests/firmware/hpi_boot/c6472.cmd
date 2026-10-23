/* The C6472 host boot starts each released core at its own DSP_BOOT_ADDRn, a 1 KB-aligned local
   L2 address (SPRUEC6G section 4.3.2). Put the existing test startup at the entry regardless of
   object-file link order; the test moves the entry by changing the `.text` address below, and
   then pins `.guard` at the base of L2, where a core that ignored its boot address would start.
   The mailbox is in shared L2 so one host-side window sees every core's result; the host
   zero-fills it once rather than once per core. */
MEMORY
{
    SL2 : origin = 0x00200000, length = 0x000C0000
    L2  : origin = 0x00800000, length = 0x00098000
}

SECTIONS
{
    .text 0x00800000 : { crt.obj(.text) *(.text) }
    .guard      > L2
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
    .shared     > SL2, type = NOINIT
}
