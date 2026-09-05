#include <stdio.h>
/* Interrupts taken while software-pipelined loops run. System event 5 is unconnected on the
   C6455, so firmware owns it: the service routine re-arms it through EVTSET0 until it has run
   a fixed number of times, which peppers interrupts across the whole kernel including the
   middle of its SPLOOP kernels. The kernel's result must be exactly what the same C computes
   with no interrupts at all, and the interrupt count must come out exact. */

#define EVTFLAG0 0x01800000u
#define EVTSET0  0x01800020u
#define EVTCLR0  0x01800040u
#define INTMUX1  0x01800104u

#define EVENT     5u
#define CPU_INT5  5u
#define INTERRUPTS 300

extern void intc_setup(unsigned int istp, unsigned int ier);
extern void intc_enable(void);
extern void clear_ifr(unsigned int mask);
extern unsigned int ist_table;
extern int fir_kernel(void);

static volatile int served;
static volatile unsigned int witness;

static void poke(unsigned int address, unsigned int value)
{
    *(volatile unsigned int *)address = value;
}

static unsigned int peek(unsigned int address)
{
    return *(volatile unsigned int *)address;
}

interrupt void event_isr(void)
{
    int count = served + 1;
    served = count;
    /* Fold the interrupt number in so a lost or doubled interrupt shows up in the output. */
    witness = witness * 1103515245u + (unsigned int)count;
    /* Clear the event, let the two-cycle path to IFR settle, then drop any flag it latched;
       otherwise the level would deliver one extra interrupt after this routine returns. */
    poke(EVTCLR0, 1u << EVENT);
    (void)peek(EVTFLAG0);
    (void)peek(EVTFLAG0);
    clear_ifr(1u << CPU_INT5);
    if (count < INTERRUPTS) {
        poke(EVTSET0, 1u << EVENT);
    }
}

int main(void)
{
    int result;

    poke(EVTCLR0, 1u << EVENT);
    /* INTMUX1 selects the events driving INT4 to INT7; INTSEL5 sits in bits 14-8. */
    poke(INTMUX1, EVENT << 8);
    intc_setup((unsigned int)&ist_table, (1u << CPU_INT5) | 2u);
    intc_enable();
    poke(EVTSET0, 1u << EVENT);

    result = fir_kernel();

    while (served < INTERRUPTS) {
    }
    printf("interrupt_sploop=%d served=%d witness=%u flag=%u\n", result, served, witness,
           peek(EVTFLAG0) & (1u << EVENT));
    return 0;
}
