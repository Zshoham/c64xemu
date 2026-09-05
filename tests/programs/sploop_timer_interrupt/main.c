/* A software-pipelined loop interrupted over and over by a timer.
 *
 * At -O2 and -O3 cl6x turns this dot product into an SPLOOP kernel - checked with dis6x - so every
 * interrupt that arrives while it runs has to drain the loop buffer, service the routine and pipe
 * the loop back up without losing or repeating an iteration (SPRUFE8B 7.13); the other levels
 * check the same arithmetic against ordinary interrupted code. Timer 0 counts to a short
 * period in continuous mode and its lower-half interrupt, TINTLO0 - system event 67 (SPRS276M
 * Table 7-9) - is routed to CPU interrupt 5, so the loop is interrupted many times.
 *
 * The dot product is the sum of the squares of 1 to 256, which is hand-computable; the number of
 * interrupts is not deterministic, so the program only requires that at least one arrived.
 */

#define REG(address) (*(volatile unsigned int *)(address))

/* Timer 0 (SPRU968). */
#define TIMER0_PRDLO 0x02940018u
#define TIMER0_TCR 0x02940020u
#define TIMER0_TGCR 0x02940024u
#define TCR_CONTINUOUS (2u << 6)
#define TGCR_BOTH_HALVES_OUT_OF_RESET 3u

/* Megamodule interrupt controller (SPRU871K chapter 7). */
#define INTMUX1 0x01800104u
/* INTSEL4 = 4, INTSEL5 = 67 (TINTLO0), INTSEL6 = 6, INTSEL7 = 7. */
#define INTMUX1_INT5_TINTLO0 0x07064304u

#define IER_NMIE (1u << 1)
#define IER_INT5 (1u << 5)

#define COUNT 256
#define PERIOD 8u

extern void intc_setup(unsigned int istp, unsigned int ier);
extern void intc_enable(void);
extern unsigned int ist_table;

static unsigned int left[COUNT];
static unsigned int right[COUNT];
static volatile unsigned int served;

interrupt void timer_isr(void)
{
    served = served + 1u;
}

int main(void)
{
    unsigned int index;
    unsigned int total = 0;

    served = 0u;
    for (index = 0; index < COUNT; index++) {
        left[index] = index + 1u;
        right[index] = index + 1u;
    }

    REG(INTMUX1) = INTMUX1_INT5_TINTLO0;
    intc_setup((unsigned int)&ist_table, IER_INT5 | IER_NMIE);
    intc_enable();

    REG(TIMER0_PRDLO) = PERIOD;
    REG(TIMER0_TGCR) = TGCR_BOTH_HALVES_OUT_OF_RESET;
    REG(TIMER0_TCR) = TCR_CONTINUOUS;

    /* The pipelined loop: interrupts land inside it repeatedly. */
    for (index = 0; index < COUNT; index++) {
        total += left[index] * right[index];
    }

    REG(TIMER0_TCR) = 0u;
    if (served == 0u) {
        return -1;
    }
    return (int)total;
}
