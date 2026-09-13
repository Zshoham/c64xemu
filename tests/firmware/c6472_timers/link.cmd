/* Linker command file for the C6472 timer program.

   Each megamodule has 608 KB of local L2 SRAM at 0x00800000, and all six share 768 KB of L2 RAM
   at 0x00200000 (SPRS612G Table 2-2). The image runs in the boot core's local L2, and the run's
   counters are left in the shared window at a fixed address the harness dumps - at a fixed one so
   that the dump does not depend on how the linker orders things.
*/
MEMORY
{
    SHARED : origin = 0x00200000, length = 0x00000100
    L2     : origin = 0x00800000, length = 0x00090000
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
    .shared_fixed > SHARED, type = NOINIT
}
