/* The UTOPIA clock detector serviced as a real interrupt, three failures in a row.
 *
 * With the receive section enabled and no ATM master driving URCLK, the clock detector fails once
 * every RCCNT peripheral clocks and sets RCFP in EIPR (SPRUE48 Table 8). EIER lets that reach the
 * CPU as UINT, system event 36 (SPRS276M Table 7-9), which INTMUX1 routes to CPU interrupt 6.
 *
 * The service routine records the pending bits it saw and clears them, which is what allows the
 * next failure to be announced: a pending bit left standing keeps the interrupt line asserted and
 * the CPU only recognises a fresh assertion (SPRU871K 7.4.2). After the third failure the routine
 * masks the error itself, so the count the program returns is exact rather than timing-dependent.
 */

#define REG(address) (*(volatile unsigned int *)(address))

/* UTOPIA (SPRUE48 section 12). */
#define UTOPIA_UCR 0x02B40000u
#define UTOPIA_CDR 0x02B40014u
#define UTOPIA_EIER 0x02B40018u
#define UTOPIA_EIPR 0x02B4001Cu

#define UCR_RESET 0x20004000u
#define UCR_UREN (1u << 0)
#define EIPR_RCFP (1u << 1)
/* The four write-one-to-clear clock bits: XCPP, XCFP, RCPP, RCFP. */
#define EIPR_CLOCK_BITS ((1u << 18) | (1u << 17) | (1u << 2) | (1u << 1))

/* Megamodule interrupt controller (SPRU871K). */
#define INTMUX1 0x01800104u
/* INTSEL4 = 4, INTSEL5 = 5, INTSEL6 = 36 (UINT), INTSEL7 = 7. */
#define INTMUX1_INT6_UINT 0x07240504u

#define IER_NMIE (1u << 1)
#define IER_INT6 (1u << 6)

#define FAILURES 3u
/* Eight peripheral clocks between failures: long enough that the routine finishes first. */
#define RCCNT 8u

extern void intc_setup(unsigned int istp, unsigned int ier);
extern void intc_enable(void);
extern unsigned int ist_table;

static volatile unsigned int served;
static volatile unsigned int seen;

interrupt void utopia_isr(void)
{
    unsigned int pending = REG(UTOPIA_EIPR);

    /* Acknowledge whatever stands, whether or not it is still wanted: an interrupt already
       latched in IFR is taken even after the source has been masked. */
    REG(UTOPIA_EIPR) = pending & EIPR_CLOCK_BITS;
    if (served >= FAILURES) {
        return;
    }
    seen |= pending;
    served = served + 1u;
    if (served >= FAILURES) {
        /* Stop the detector and mask the error: the count is complete. */
        REG(UTOPIA_UCR) = UCR_RESET;
        REG(UTOPIA_EIER) = 0u;
    }
}

int main(void)
{
    unsigned int spins;

    served = 0u;
    seen = 0u;

    REG(UTOPIA_CDR) = RCCNT;
    REG(UTOPIA_EIER) = EIPR_RCFP;

    REG(INTMUX1) = INTMUX1_INT6_UINT;
    intc_setup((unsigned int)&ist_table, IER_INT6 | IER_NMIE);
    intc_enable();

    /* Take the receive section out of reset: from here on it expects a URCLK edge. */
    REG(UTOPIA_UCR) = UCR_RESET | UCR_UREN;

    for (spins = 0; spins < 40000u && served < FAILURES; spins++) {
    }
    REG(UTOPIA_UCR) = UCR_RESET;

    if (served != FAILURES) {
        return -1;
    }
    /* Only the receive clock can fail here: nothing drives the transmit section. */
    return (int)((seen << 8) | served);
}
