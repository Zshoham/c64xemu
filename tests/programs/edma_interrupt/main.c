/* An EDMA3 completion taken as a real CPU interrupt.
 *
 * Timer 0's lower-half event (EDMA3 synchronisation event 1) triggers channel 1, whose PaRAM set
 * asks for a completion interrupt with code 1. The channel controller's global completion
 * interrupt is system event 24 (SPRS276M Table 7-9); the interrupt selector routes it to CPU
 * interrupt 5 through INTMUX1, and the service routine acknowledges it in the channel controller
 * and in the megamodule event flags.
 *
 * Nothing but the EDMA3 writes the destination buffer, and nothing but the interrupt increments
 * the service counter, so the value the program returns is proof of the whole path: timer event,
 * transfer, completion interrupt, vectoring through the interrupt service table, and the return
 * from interrupt.
 */

#define REG(address) (*(volatile unsigned int *)(address))

/* EDMA3 channel controller (SPRU966C). */
#define EDMA_DCHMAP1 0x02A00104u
#define EDMA_EESR 0x02A01030u
#define EDMA_IESR 0x02A01060u
#define EDMA_IPR 0x02A01068u
#define EDMA_ICR 0x02A01070u
#define EDMA_PARAM7 0x02A040E0u /* PaRAM base + 32 * 7 */

/* Timer 0 (SPRU968). */
#define TIMER0_PRDLO 0x02940018u
#define TIMER0_TCR 0x02940020u
#define TIMER0_TGCR 0x02940024u

/* Megamodule interrupt controller (SPRU871). */
#define EVTCLR0 0x01800040u
#define INTMUX1 0x01800104u
/* INTSEL4 = 4 (its reset value), INTSEL5 = 24, INTSEL6 = 6, INTSEL7 = 7. */
#define INTMUX1_INT5_GINT 0x07061804u

#define GINT_EVENT 24u
#define CHANNEL 1
#define WORDS 16

#define IER_NMIE (1u << 1)
#define IER_INT5 (1u << 5)

extern void intc_setup(unsigned int istp, unsigned int ier);
extern void intc_enable(void);
extern unsigned int ist_table;

static volatile unsigned int source[WORDS];
static volatile unsigned int destination[WORDS];
static volatile unsigned int served;
static volatile unsigned int pending_seen;

interrupt void edma_isr(void)
{
    pending_seen = REG(EDMA_IPR);
    /* Acknowledge the completion code, then the megamodule event it raised. */
    REG(EDMA_ICR) = 1u << CHANNEL;
    REG(EVTCLR0) = 1u << GINT_EVENT;
    served = served + 1u;
}

int main(void)
{
    unsigned int index;
    unsigned int sum = 0;
    unsigned int spins;

    served = 0u;
    pending_seen = 0u;
    for (index = 0; index < WORDS; index++) {
        source[index] = 0x01010101u * (index + 1u);
        destination[index] = 0u;
    }

    /* Channel 1 takes its parameters from PaRAM set 7. */
    REG(EDMA_DCHMAP1) = 7u << 5;
    REG(EDMA_PARAM7 + 0u) = (1u << 20) | ((unsigned int)CHANNEL << 12);
    REG(EDMA_PARAM7 + 4u) = (unsigned int)&source[0];
    REG(EDMA_PARAM7 + 8u) = (WORDS * 4u) | (1u << 16);
    REG(EDMA_PARAM7 + 12u) = (unsigned int)&destination[0];
    REG(EDMA_PARAM7 + 16u) = 0u;
    REG(EDMA_PARAM7 + 20u) = 0xFFFFu;
    REG(EDMA_PARAM7 + 24u) = 0u;
    REG(EDMA_PARAM7 + 28u) = 1u;

    REG(EDMA_IESR) = 1u << CHANNEL;
    REG(EDMA_EESR) = 1u << CHANNEL;

    REG(INTMUX1) = INTMUX1_INT5_GINT;
    intc_setup((unsigned int)&ist_table, IER_INT5 | IER_NMIE);
    intc_enable();

    REG(TIMER0_PRDLO) = 3u;
    REG(TIMER0_TGCR) = 3u;
    REG(TIMER0_TCR) = 2u << 6;

    for (spins = 0; spins < 20000u && served == 0u; spins++) {
    }
    REG(TIMER0_TCR) = 0u;

    if (served == 0u) {
        return -1;
    }
    /* The routine ran because of the completion, and it left the pending bit clear. */
    if ((pending_seen & (1u << CHANNEL)) == 0u) {
        return -2;
    }
    if ((REG(EDMA_IPR) & (1u << CHANNEL)) != 0u) {
        return -3;
    }

    for (index = 0; index < WORDS; index++) {
        sum += destination[index];
    }
    return (int)(sum + served);
}
