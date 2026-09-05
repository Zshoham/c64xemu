#include <stdio.h>
/* EDMA3 transfer chaining (SPRU966C section 2.4): a channel whose OPT has TCCHEN set does not
 * only report completion, it sets the bit its TCC field names in the chained event register CER,
 * which the channel controller then treats as a trigger for that channel. One manual trigger
 * therefore runs a chain of transfers with no CPU involvement between them, which is how a driver
 * builds a multi-stage move.
 *
 * Three PaRAM sets are linked into a chain here:
 *
 *   channel 1  copies the source block into stage one, with TCCHEN and TCC = 2,
 *   channel 2  copies stage one into stage two while adding nothing, with TCCHEN and TCC = 3,
 *   channel 3  copies stage two into the destination, with TCINTEN and TCC = 3.
 *
 * Only channel 1 is ever triggered, by a write to the event set register. The program then waits
 * for the interrupt pending bit that the last link raises - it never enables the interrupt in the
 * controller, so IPR is the only place the completion shows - and checks that every stage moved
 * and that the chained event register is clear again afterwards, since a chained event is
 * consumed when the channel it names is served.
 *
 * The checksum folds the three destination sums, the pending register and the chained event
 * register read after the chain drained. Nothing timing-dependent goes into it - how many spins
 * the wait took depends on the optimisation level - but a chain that stops after one link, or one
 * that leaves its chained event standing, changes the result.
 */

#define REG(address) (*(volatile unsigned int *)(address))

/* EDMA3 channel controller (SPRU966C). */
#define EDMA_DCHMAP(n) (0x02A00100u + (n) * 4u)
#define EDMA_ESR       0x02A01010u
#define EDMA_CER        0x02A01018u
#define EDMA_IPR       0x02A01068u
#define EDMA_ICR       0x02A01070u
#define EDMA_PARAM(n)  (0x02A04000u + (n) * 32u)

#define OPT_TCINTEN (1u << 20)
#define OPT_TCCHEN  (1u << 22)
#define OPT_TCC(code) ((unsigned int)(code) << 12)
#define OPT_STATIC  (1u << 3)

#define WORDS 16u

static volatile unsigned int source[WORDS];
static volatile unsigned int stage_one[WORDS];
static volatile unsigned int stage_two[WORDS];
static volatile unsigned int destination[WORDS];

static unsigned int fold(unsigned int accumulator, unsigned int value)
{
    accumulator = (accumulator << 5) | (accumulator >> 27);
    return accumulator ^ value;
}

/* One A-synchronised transfer of the whole block, in a single array, with a null link. */
static void program_set(unsigned int set, unsigned int opt, const volatile unsigned int *from,
                        const volatile unsigned int *to)
{
    REG(EDMA_PARAM(set) + 0u) = opt | OPT_STATIC;
    REG(EDMA_PARAM(set) + 4u) = (unsigned int)from;
    REG(EDMA_PARAM(set) + 8u) = (WORDS * 4u) | (1u << 16);
    REG(EDMA_PARAM(set) + 12u) = (unsigned int)to;
    REG(EDMA_PARAM(set) + 16u) = 0u;
    REG(EDMA_PARAM(set) + 20u) = 0xFFFFu;
    REG(EDMA_PARAM(set) + 24u) = 0u;
    REG(EDMA_PARAM(set) + 28u) = 1u;
}

int main(void)
{
    unsigned int checksum = 0;
    unsigned int index;
    unsigned int spins;
    unsigned int sums[3] = {0u, 0u, 0u};
    unsigned int pending;
    unsigned int chained;

    for (index = 0; index < WORDS; index++) {
        source[index] = 0x01010101u * (index + 1u);
        stage_one[index] = 0u;
        stage_two[index] = 0u;
        destination[index] = 0u;
    }

    /* Channels 1, 2 and 3 take their parameters from sets 8, 9 and 10. */
    REG(EDMA_DCHMAP(1)) = 8u << 5;
    REG(EDMA_DCHMAP(2)) = 9u << 5;
    REG(EDMA_DCHMAP(3)) = 10u << 5;
    program_set(8u, OPT_TCCHEN | OPT_TCC(2), source, stage_one);
    program_set(9u, OPT_TCCHEN | OPT_TCC(3), stage_one, stage_two);
    program_set(10u, OPT_TCINTEN | OPT_TCC(3), stage_two, destination);

    /* One manual trigger, and the chain does the rest. */
    REG(EDMA_ESR) = 1u << 1;

    for (spins = 0; spins < 20000u; spins++) {
        if ((REG(EDMA_IPR) & (1u << 3)) != 0u) {
            break;
        }
    }
    pending = REG(EDMA_IPR);
    REG(EDMA_ICR) = pending;
    chained = REG(EDMA_CER);

    for (index = 0; index < WORDS; index++) {
        sums[0] += stage_one[index];
        sums[1] += stage_two[index];
        sums[2] += destination[index];
    }
    checksum = fold(checksum, sums[0]);
    checksum = fold(checksum, sums[1]);
    checksum = fold(checksum, sums[2]);
    checksum = fold(checksum, pending);
    checksum = fold(checksum, chained);

    printf("edma chaining checksum %u\n", checksum);
    return 0;
}
