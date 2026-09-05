/* The linker command file SPRU862B Example 2-1 gives for a 256 KB L2 cache - 1792K of SRAM at
   0080 0000h - with the region the cache would take defined as well, which is exactly what the
   Note beside it forbids: "Do not define memory that is to be used or boots up as cache under
   the MEMORY directive. This memory is not valid for the linker to place code or data in."

   Defining it is the point. The program links one array into that region and watches it stop
   being memory when the cache is configured, which is the only way to observe from software
   that the aperture is a function of the cache configuration. */
MEMORY
{
    L2      : origin = 0x00800000, length = 0x001C0000
    /* Figure 2-1's base for a 4-way 256K byte L2 cache in a 2048K byte L2. */
    L2CACHE : origin = 0x009C0000, length = 0x00040000
}

SECTIONS
{
    .text       > L2
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
    .carved     > L2CACHE
}
