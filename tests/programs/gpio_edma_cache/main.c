/* A GPIO edge that moves a block through the EDMA3, with the cache in the way.
 *
 * Pin 0 is configured as an output with rising-edge detection, so a software toggle raises GPINT0
 * exactly as an external signal would; GPINT0 is EDMA3 synchronisation event 48 (SPRU966C
 * Table 2-6). L1D is switched to 32K cache first, and both cache operations a driver has to
 * perform are load-bearing here:
 *
 *   - the source words are written, read (which allocates their lines) and written again, so the
 *     new values sit dirty in L1D while L2 still holds the first pass; without the writeback the
 *     EDMA3 would copy the stale words, because an external master reads memory rather than the
 *     CPU's cache;
 *   - the destination is read while it is still zero, which leaves clean zero lines in L1D;
 *     without the invalidate the CPU would read those zeros back instead of what the EDMA3 wrote.
 *
 * So the checksum this returns is proof of the edge, the transfer and both cache operations.
 */

#define REG(address) (*(volatile unsigned int *)(address))

/* C64x+ cache control (SPRU871K chapter 4). L1DMODE 100b selects the 32K cache. */
#define L1DCFG 0x01840040u
#define L1DWB 0x01845040u
#define L1DINV 0x01845048u
#define L1D_32K_CACHE 4u

/* GPIO (SPRU724A). */
#define GPIO_BINTEN 0x02B00008u
#define GPIO_DIR 0x02B00010u
#define GPIO_SET_DATA 0x02B00018u
#define GPIO_CLR_DATA 0x02B0001Cu
#define GPIO_SET_RIS_TRIG 0x02B00024u

/* EDMA3 channel controller (SPRU966C); channel 48's event enable is in the high register. */
#define EDMA_DCHMAP48 0x02A001C0u
#define EDMA_EESRH 0x02A01034u
#define EDMA_IESR 0x02A01060u
#define EDMA_IPR 0x02A01068u
#define EDMA_ICR 0x02A01070u
#define EDMA_PARAM11 0x02A04160u /* PaRAM base + 32 * 11 */

#define TCC 5
#define WORDS 16

static volatile unsigned int source[WORDS];
static volatile unsigned int destination[WORDS];

int main(void)
{
    unsigned int index;
    unsigned int stale = 0;
    unsigned int zeros = 0;
    unsigned int sum = 0;
    unsigned int spins;

    REG(L1DCFG) = L1D_32K_CACHE;

    /* Nothing is cached yet, so this pass lands in L2. */
    for (index = 0; index < WORDS; index++) {
        source[index] = 0x01010101u * (index + 1u);
        destination[index] = 0u;
    }
    /* Reading the source allocates its lines. */
    for (index = 0; index < WORDS; index++) {
        stale += source[index];
    }
    /* These writes hit those lines and leave them dirty; L2 still holds the first pass. */
    for (index = 0; index < WORDS; index++) {
        source[index] = 0x02020202u * (index + 1u);
    }
    /* Reading the destination leaves clean zero lines behind. */
    for (index = 0; index < WORDS; index++) {
        zeros += destination[index];
    }

    /* Channel 48 takes its parameters from PaRAM set 11. */
    REG(EDMA_DCHMAP48) = 11u << 5;
    REG(EDMA_PARAM11 + 0u) = (1u << 20) | ((unsigned int)TCC << 12);
    REG(EDMA_PARAM11 + 4u) = (unsigned int)&source[0];
    REG(EDMA_PARAM11 + 8u) = (WORDS * 4u) | (1u << 16);
    REG(EDMA_PARAM11 + 12u) = (unsigned int)&destination[0];
    REG(EDMA_PARAM11 + 16u) = 0u;
    REG(EDMA_PARAM11 + 20u) = 0xFFFFu;
    REG(EDMA_PARAM11 + 24u) = 0u;
    REG(EDMA_PARAM11 + 28u) = 1u;
    REG(EDMA_IESR) = 1u << TCC;
    REG(EDMA_EESRH) = 1u << (48u - 32u);

    /* Hand the dirty source words to memory before the transfer reads them. */
    REG(L1DWB) = 1u;

    /* Pin 0 an output, its rising edge enabled, and the bank's interrupt enable set. */
    REG(GPIO_DIR) = 0xFFFEu;
    REG(GPIO_SET_RIS_TRIG) = 1u;
    REG(GPIO_BINTEN) = 1u;
    REG(GPIO_CLR_DATA) = 1u;
    /* The detector samples on the peripheral clock, the CPU clock divided by six, so the pin has
       to be low for a whole peripheral clock before an edge exists (SPRU724A Figure 2). */
    for (spins = 0; spins < 20u; spins++) {
    }
    REG(GPIO_SET_DATA) = 1u;

    for (spins = 0; spins < 20000u; spins++) {
        if ((REG(EDMA_IPR) & (1u << TCC)) != 0u) {
            break;
        }
    }
    if ((REG(EDMA_IPR) & (1u << TCC)) == 0u) {
        return -1;
    }
    REG(EDMA_ICR) = 1u << TCC;

    /* Drop the cached zeros so the transfer's result is visible. */
    REG(L1DINV) = 1u;

    for (index = 0; index < WORDS; index++) {
        sum += destination[index];
    }
    if (zeros != 0u || stale != 0x88888888u) {
        return -2;
    }
    return (int)sum;
}
