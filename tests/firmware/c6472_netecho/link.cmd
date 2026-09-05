/* Linker command file for the C6472 six-core echo firmware.

   Each megamodule has 608 KB of local L2 SRAM at 0x00800000 and all six share 768 KB of L2 RAM
   at 0x00200000 (SPRS612G Table 2-2). The image is loaded into every core's local L2, so code,
   data and stack are placed there and each core gets its own copy. The descriptor rings and the
   frame buffers go in the shared window instead, for two reasons: every core reaches them at the
   same address, and the shared window has a global alias the MAC's DMA can use, which a
   megamodule's own L2 window does not. They are uninitialised so the loader does not write
   220 KB into six cores' images.
*/
MEMORY
{
    SL2     : origin = 0x00200000, length = 0x000C0000
    L2      : origin = 0x00800000, length = 0x00098000
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
    .shared     > SL2, type = NOINIT
}
