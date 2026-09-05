/* A timer-triggered EDMA3 transfer, driven the way a real driver would drive it.

   Timer 0's lower-half event (TEVTLO0) is EDMA3 synchronisation event 1 (SPRS276M Table 7-10).
   The firmware maps channel 1 onto PaRAM set 5, programs an A-synchronised block copy, enables
   the channel's event and its completion interrupt, then starts the timer in continuous mode and
   waits for the completion bit in IPR. Nothing but the EDMA3 ever writes the destination, so the
   checksum the program returns is proof that the timer event reached the channel controller and
   that the controller moved the data.

   The expected checksum is the sum of the source words, computed by hand in the spec. */

#define REG(address) (*(volatile unsigned int *)(address))

/* EDMA3 channel controller (SPRU966C, addresses from SPRS276M Table 2-2). */
#define EDMA_DCHMAP1 0x02A00104u
#define EDMA_EESR 0x02A01030u
#define EDMA_IESR 0x02A01060u
#define EDMA_IPR 0x02A01068u
#define EDMA_ICR 0x02A01070u
#define EDMA_PARAM5 0x02A040A0u /* PaRAM base + 32 * 5 */

/* Timer 0 (SPRU968). */
#define TIMER0_PRDLO 0x02940018u
#define TIMER0_TCR 0x02940020u
#define TIMER0_TGCR 0x02940024u

/* OPT: transfer complete interrupt enabled, completion code 1 (SPRU966C Table 2-4). */
#define OPT_TCINTEN (1u << 20)
#define OPT_TCC(code) ((unsigned int)(code) << 12)

#define WORDS 16
#define CHANNEL 1

static volatile unsigned int source[WORDS];
static volatile unsigned int destination[WORDS];

int main(void)
{
    unsigned int index;
    unsigned int sum = 0;
    unsigned int spins;

    for (index = 0; index < WORDS; index++) {
        source[index] = 0x01010101u * (index + 1u);
        destination[index] = 0u;
    }

    /* Channel 1 takes its parameters from PaRAM set 5 (DCHMAP.PAENTRY is bits 13:5). */
    REG(EDMA_DCHMAP1) = 5u << 5;

    REG(EDMA_PARAM5 + 0u) = OPT_TCINTEN | OPT_TCC(CHANNEL);
    REG(EDMA_PARAM5 + 4u) = (unsigned int)&source[0];
    /* ACNT = the whole block, BCNT = 1: one event moves everything. */
    REG(EDMA_PARAM5 + 8u) = (WORDS * 4u) | (1u << 16);
    REG(EDMA_PARAM5 + 12u) = (unsigned int)&destination[0];
    REG(EDMA_PARAM5 + 16u) = 0u;
    /* A null link, so the set is emptied when the transfer finishes. */
    REG(EDMA_PARAM5 + 20u) = 0xFFFFu;
    REG(EDMA_PARAM5 + 24u) = 0u;
    REG(EDMA_PARAM5 + 28u) = 1u;

    REG(EDMA_IESR) = 1u << CHANNEL;
    REG(EDMA_EESR) = 1u << CHANNEL;

    /* A 64-bit general-purpose timer, both halves out of reset, counting continuously to a
       short period so its event arrives quickly. */
    REG(TIMER0_PRDLO) = 3u;
    REG(TIMER0_TGCR) = 3u;
    REG(TIMER0_TCR) = 2u << 6;

    for (spins = 0; spins < 20000u; spins++) {
        if ((REG(EDMA_IPR) & (1u << CHANNEL)) != 0u) {
            break;
        }
    }
    if ((REG(EDMA_IPR) & (1u << CHANNEL)) == 0u) {
        return -1;
    }

    /* Stop the timer and acknowledge the completion, as a driver's ISR would. */
    REG(TIMER0_TCR) = 0u;
    REG(EDMA_ICR) = 1u << CHANNEL;

    for (index = 0; index < WORDS; index++) {
        sum += destination[index];
    }
    if ((REG(EDMA_IPR) & (1u << CHANNEL)) != 0u) {
        return -2;
    }
    return (int)sum;
}
