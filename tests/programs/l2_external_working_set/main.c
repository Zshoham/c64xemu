#include <stdio.h>
/* A working set that lives in external memory, which is the only memory the L2 cache caches.
 *
 * Every other program in this suite is linked into L2 SRAM, so the cached rerun (`--cache`,
 * 32 KB of L1P, 32 KB of L1D and 256 KB of L2) exercises L1P and L1D thoroughly and reaches the
 * L2 cache barely at all: SPRU862B section 2.2 has L2 cache external addresses only, and L2 SRAM
 * accesses go straight past it. This program's data is in DDR2 instead, and it is twice the size
 * of the largest L2 cache a C64x+ megamodule has, so the same three loops that give the answer
 * also drive L2 allocation, capacity eviction, conflict eviction and the write-back path.
 *
 * MAR224 is what makes the DDR2 window cacheable at all (SPRU871K chapter 4). With its PC bit
 * clear, `--cache` would configure 256 KB of L2 cache that nothing here could allocate in.
 *
 * The three loops, and what each one asks of the cache:
 *
 *   1. Fill.  WORDS stores over a 256 KB cache. The lines the first half allocates are evicted
 *      dirty before the pass ends, so the data only reaches DDR2 through the write-back path.
 *   2. Read back in the same order and fold every word. The words at the start of the array are
 *      no longer in any cache; a write-back that dropped them reads as whatever DDR2 holds.
 *   3. Conflict. Eight words WAY_STRIDE apart index one L2 set - the array is eight way-strides
 *      long and the cache has four ways - so each group evicts four times, and read-modify-write
 *      makes those evictions dirty. Stepping the base by one L2 line sweeps every set of the
 *      cache. 64 KB is also four L1D way strides, so each group conflicts in L1D too.
 *   4. Fold the words loop 3 touched. Their increments had to survive both cache levels.
 *
 * Every access is volatile: one word per access, no widening to LDDW and no value carried in a
 * register across the pass, so the loops make the memory traffic they read like they make.
 */

/* 512 KB: twice the 256 KB of L2 cache, so the fill pass evicts everything it allocated. */
#define WORDS      131072u
/* 64 KB in words: 256 KB of L2 cache over its four ways, so this stride indexes one set. */
#define WAY_STRIDE 16384u
/* WORDS / WAY_STRIDE: how many words of the array compete for each of the cache's sets. */
#define COMPETITORS 8u
/* 128 bytes in words: one L2 line, so stepping the base by this sweeps distinct sets. */
#define LINE_WORDS 32u

#if defined(__TI_COMPILER_VERSION__)
#define MAR224 (0x01848000u + 224u * 4u)
#define EXTERNAL ((volatile unsigned int *)0xE0000000u)
static void make_external_cacheable(void)
{
    *(volatile unsigned int *)MAR224 = 1u;
}
#else
static unsigned int host_external[WORDS];
#define EXTERNAL ((volatile unsigned int *)host_external)
static void make_external_cacheable(void)
{
}
#endif

/* A value per index with every byte significant, so a lost line cannot pass as a coincidence. */
static unsigned int mix(unsigned int index)
{
    unsigned int value = index * 2654435761u + 0x9E3779B9u;
    value ^= value >> 15;
    return value * 2246822519u;
}

static unsigned int fold(unsigned int accumulator, unsigned int value)
{
    accumulator = (accumulator << 5) | (accumulator >> 27);
    return accumulator ^ value;
}

int main(void)
{
    volatile unsigned int *external = EXTERNAL;
    unsigned int checksum = 0x12345678u;
    unsigned int index;
    unsigned int base;
    unsigned int competitor;

    make_external_cacheable();

    for (index = 0; index < WORDS; index++) external[index] = mix(index);

    for (index = 0; index < WORDS; index++) checksum = fold(checksum, external[index]);

    for (base = 0; base < WAY_STRIDE; base += LINE_WORDS)
        for (competitor = 0; competitor < COMPETITORS; competitor++) {
            index = base + competitor * WAY_STRIDE;
            external[index] = external[index] + index;
        }

    for (base = 0; base < WAY_STRIDE; base += LINE_WORDS)
        for (competitor = 0; competitor < COMPETITORS; competitor++)
            checksum = fold(checksum, external[base + competitor * WAY_STRIDE]);

    printf("external working set checksum %u\n", checksum);
    return 0;
}
