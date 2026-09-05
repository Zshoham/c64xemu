/* Both IDMA channel 1 modes, each finishing with its completion interrupt.
 *
 * IDMA channel 1 copies or fills word-aligned local memory in the background while the CPU keeps
 * running (SPRU871K chapter 5). Writing COUNT starts the transfer; bit 16 selects fill mode, where
 * SOURCE holds the value rather than an address, and bit 28 asks for the completion interrupt.
 * That interrupt is IDMAINT1, system event 14, which INTMUX1 routes to CPU interrupt 7 here.
 *
 * The program starts a copy, waits for its interrupt, then starts a fill and waits for the second
 * one, and returns the two block sums plus the number of interrupts. The CPU never writes either
 * destination, so the sums are proof that the channel ran, and the count is proof that each
 * transfer announced itself exactly once.
 */

#define REG(address) (*(volatile unsigned int *)(address))

/* IDMA channel 1 (SPRU871K chapter 5). */
#define IDMA1_STAT 0x01820100u
#define IDMA1_SOURCE 0x01820108u
#define IDMA1_DEST 0x0182010Cu
#define IDMA1_COUNT 0x01820110u

#define IDMA_FILL (1u << 16)
#define IDMA_INT (1u << 28)

/* Megamodule interrupt controller (SPRU871K chapter 7). */
#define INTMUX1 0x01800104u
/* INTSEL4 = 4, INTSEL5 = 5, INTSEL6 = 6, INTSEL7 = 14 (IDMAINT1). */
#define INTMUX1_INT7_IDMA 0x0E060504u

#define IER_NMIE (1u << 1)
#define IER_INT7 (1u << 7)

#define WORDS 16
#define FILL_VALUE 0x00010001u

extern void intc_setup(unsigned int istp, unsigned int ier);
extern void intc_enable(void);
extern unsigned int ist_table;

static volatile unsigned int source[WORDS];
static volatile unsigned int copied[WORDS];
static volatile unsigned int filled[WORDS];
static volatile unsigned int served;
static volatile unsigned int busy_seen;

interrupt void idma_isr(void)
{
    /* The channel is idle again by the time it announces the transfer. */
    busy_seen |= REG(IDMA1_STAT) & 1u;
    served = served + 1u;
}

static int wait_for(unsigned int count)
{
    unsigned int spins;

    for (spins = 0; spins < 20000u && served < count; spins++) {
    }
    return served == count;
}

int main(void)
{
    unsigned int index;
    unsigned int sum = 0;

    served = 0u;
    busy_seen = 0u;
    for (index = 0; index < WORDS; index++) {
        source[index] = 0x01010101u * (index + 1u);
        copied[index] = 0u;
        filled[index] = 0u;
    }

    REG(INTMUX1) = INTMUX1_INT7_IDMA;
    intc_setup((unsigned int)&ist_table, IER_INT7 | IER_NMIE);
    intc_enable();

    /* A background copy of the whole block, with its completion interrupt. */
    REG(IDMA1_SOURCE) = (unsigned int)&source[0];
    REG(IDMA1_DEST) = (unsigned int)&copied[0];
    REG(IDMA1_COUNT) = IDMA_INT | (WORDS * 4u);
    if (!wait_for(1u)) {
        return -1;
    }

    /* The same channel in fill mode: SOURCE is the value, not an address. */
    REG(IDMA1_SOURCE) = FILL_VALUE;
    REG(IDMA1_DEST) = (unsigned int)&filled[0];
    REG(IDMA1_COUNT) = IDMA_INT | IDMA_FILL | (WORDS * 4u);
    if (!wait_for(2u)) {
        return -2;
    }
    if (busy_seen != 0u) {
        return -3;
    }

    for (index = 0; index < WORDS; index++) {
        sum += copied[index];
    }
    for (index = 0; index < WORDS; index++) {
        sum += filled[index];
    }
    return (int)(sum + served);
}
