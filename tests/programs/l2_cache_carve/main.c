#include <stdio.h>
/* What configuring L2 as cache does to the L2 SRAM aperture.
 *
 * SPRU862B section 2.3: "when the L2 cache size is increased, the memory is taken from the high
 * memory addresses", and its Figure 2-1 puts a 4-way 256K byte cache in a 2048K byte L2 at
 * 009C 0000h. The Note beside that figure says what the consequence is for a program: "Do not
 * define memory that is to be used or boots up as cache under the MEMORY directive. This memory
 * is not valid for the linker to place code or data in."
 *
 * This program's linker command file defines it anyway and links one array into it, because
 * disregarding that Note is the only way software can observe the aperture move. Four phases,
 * each printing what it saw:
 *
 *   1. All of L2 is SRAM out of reset (SPRU871K section 1.2.4: L2 "is always initiated to all
 *      SRAM after reset"), so the array is ordinary memory and reads back what it was given.
 *   2. Configure 256 KB of L2 cache. The array's addresses are the ones the cache took, so what
 *      was written to them is gone - SPRU871K section 2.4.3 calls it "data loss due to portions
 *      of L1P RAM becoming cache" and SPRU862B Table 2-6 is the procedure that avoids it, which
 *      this program deliberately does not follow.
 *   3. They are not memory either: a store to them does not come back.
 *   4. Shrink the cache to nothing and the aperture returns, so a store sticks again. What was
 *      there before does not return with it: the cache had those bits.
 *
 * The array is a whole number of 128-byte L2 lines and line-aligned, so nothing here depends on
 * a partial line. Every access is volatile, so each phase makes the access it reads like.
 */

#define L2CFG        0x01840000u
#define L2_256K_CACHE 4u
#define L2_ALL_SRAM   0u

#define WORDS 64u
#define PATTERN 0x5AC0DE00u

#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(carved, 128)
#pragma DATA_SECTION(carved, ".carved")
#endif
static volatile unsigned int carved[WORDS];

static void poke(unsigned int address, unsigned int value)
{
    *(volatile unsigned int *)address = value;
}

static void fill(unsigned int seed)
{
    unsigned int index;
    for (index = 0; index < WORDS; index++) carved[index] = seed + index;
}

/* How many words hold what the last fill put there. */
static unsigned int intact(unsigned int seed)
{
    unsigned int index;
    unsigned int count = 0;
    for (index = 0; index < WORDS; index++)
        if (carved[index] == seed + index) count++;
    return count;
}

int main(void)
{
    fill(PATTERN);
    printf("all sram: %u of %u\n", intact(PATTERN), WORDS);

    poke(L2CFG, L2_256K_CACHE);
    printf("cache configured: %u of %u\n", intact(PATTERN), WORDS);

    fill(PATTERN);
    printf("cache configured after a store: %u of %u\n", intact(PATTERN), WORDS);

    poke(L2CFG, L2_ALL_SRAM);
    printf("cache released: %u of %u\n", intact(PATTERN), WORDS);

    fill(PATTERN);
    printf("cache released after a store: %u of %u\n", intact(PATTERN), WORDS);
    return 0;
}
