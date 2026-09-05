#include <stdio.h>
/* L1P is outside the snoop protocol. SPRU862B says it plainly: "The snoop coherence protocol on
 * C64x+ devices does not maintain coherence between L1P cache and L2 SRAM, as is the case on
 * C64x devices. This is the responsibility of the programmer." So code a DMA master writes into
 * L2 SRAM while L1P holds the old copy keeps executing the old copy until software invalidates
 * L1P - the self-modifying code case of section 2.6.
 *
 * IDMA channel 1 is the master here: it moves words between the local memories without the CPU
 * (SPRU871K chapter 5). The program
 *
 *   1. enables L1P as a 32K cache and calls a routine in L2 SRAM, which caches it,
 *   2. has IDMA1 overwrite that routine with one that returns a different value,
 *   3. calls it again, which must still run the cached copy,
 *   4. invalidates L1P by range through L1PIBAR/L1PIWC and calls it once more, which must now
 *      run the new code.
 *
 * The two routines are the same length and differ only in the constant they return, so the copy
 * is a plain block move. The checksum folds all three results, so a model that snoops L1P - or
 * one that never caches the routine at all - produces a different number.
 */

#define L1PCFG        0x01840020u
#define L1PIBAR       0x01844020u
#define L1PIWC        0x01844024u
#define L1P_32K_CACHE 4u

#define IDMA1_SOURCE 0x01820108u
#define IDMA1_DEST   0x0182010Cu
#define IDMA1_COUNT  0x01820110u
#define IDMA1_STAT   0x01820100u

#define ROUTINE_WORDS 8u

static void poke(unsigned int address, unsigned int value)
{
    *(volatile unsigned int *)address = value;
}

static unsigned int peek(unsigned int address)
{
    return *(volatile unsigned int *)address;
}

static unsigned int fold(unsigned int accumulator, unsigned int value)
{
    accumulator = (accumulator << 5) | (accumulator >> 27);
    return accumulator ^ value;
}

/* The routine that gets overwritten, and its replacement. Both are called through a pointer so
 * the compiler cannot inline the first one away. */
int first(void);
int second(void);

int main(void)
{
    int (*routine)(void) = first;
    unsigned int checksum = 0;
    unsigned int cached;
    unsigned int stale;
    unsigned int fresh;

    poke(L1PCFG, L1P_32K_CACHE);

    /* Run it once so its fetch packets land in L1P. */
    cached = (unsigned int)routine();
    checksum = fold(checksum, cached);

    /* IDMA1 copies the replacement over it. Bit 16 of COUNT is the fill flag, which stays
     * clear; the low bits are the byte count, and writing COUNT starts the transfer. */
    poke(IDMA1_SOURCE, (unsigned int)second);
    poke(IDMA1_DEST, (unsigned int)first);
    poke(IDMA1_COUNT, ROUTINE_WORDS * 4u);
    while ((peek(IDMA1_STAT) & 1u) != 0u) {
    }

    /* L1P still holds the old fetch packets, so this must be the old answer. */
    stale = (unsigned int)routine();
    checksum = fold(checksum, stale);

    /* A range invalidate of L1P over the routine, and only then the new answer. */
    poke(L1PIBAR, (unsigned int)first);
    poke(L1PIWC, ROUTINE_WORDS);
    fresh = (unsigned int)routine();
    checksum = fold(checksum, fresh);

    printf("l1p stale code %u %u %u checksum %u\n", cached, stale, fresh, checksum);
    return 0;
}
