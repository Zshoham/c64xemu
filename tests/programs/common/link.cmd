/* Shared linker command file for emulator program tests.
   Single L2 SRAM window matching the C6455 memory map at 0x00800000. */
MEMORY
{
    L2 : origin = 0x00800000, length = 0x00200000
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
}
