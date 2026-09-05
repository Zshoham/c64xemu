/* Linker command file for the C6472 multicore firmware.

   Each megamodule has 608 KB of local L2 SRAM at 0x00800000, and all six share 768 KB of L2
   RAM at 0x00200000 (SPRS612G Table 2-2). The image is loaded into every core's local L2, so
   code, data and stack are placed there and each core gets its own copy; the rendezvous
   buffer lives in the shared window, uninitialised so the loader does not write it six times.
*/
MEMORY
{
    SL2       : origin = 0x00200000, length = 0x000C0000
    L2        : origin = 0x00800000, length = 0x00010000
    /* DSP_BOOT_ADDRn carries only the top 22 bits of the address (SPRS612G section 3.9.5), so
       a core can only be started on a 1 KB boundary. The entry the other five cores are given
       gets a window of its own at one. */
    SECONDARY : origin = 0x00810000, length = 0x00001000
    L2_REST   : origin = 0x00811000, length = 0x00087000
}

SECTIONS
{
    .secondary  > SECONDARY
    .text       > L2 | L2_REST
    .const      > L2 | L2_REST
    .rodata     > L2 | L2_REST
    .switch     > L2 | L2_REST
    .cinit      > L2 | L2_REST
    .pinit      > L2 | L2_REST
    .init_array > L2 | L2_REST
    .binit      > L2 | L2_REST
    .cio        > L2 | L2_REST
    .data       > L2 | L2_REST
    .neardata   > L2 | L2_REST
    .fardata    > L2 | L2_REST
    .bss        > L2 | L2_REST
    .far        > L2 | L2_REST
    .stack      > L2_REST
    .sysmem     > L2_REST
    .shared     > SL2, type = NOINIT
}
