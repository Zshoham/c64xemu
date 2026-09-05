#include <stdio.h>
/* A mailbox served over the host port interface, driven by a process on the host.
 *
 * The host owns the HPI: it writes a request block into DSP memory through HPID, raises DSPINT,
 * and waits for HINT. This firmware takes DSPINT as a real CPU interrupt - system event 15 routed
 * through INTMUX to CPU interrupt 5 - and its service routine reads the request, answers it, and
 * raises HINT for the host, acknowledging DSPINT in the same register the way SPRU969B section 5.1
 * requires: the CPU writes one to it, which forces it to zero and lets the host interrupt again.
 *
 * The request block is four words at REQUEST: a command, a length in words, and the first two
 * operands; the payload follows. The reply block is two words at REPLY: the request's sequence
 * number and the answer. Commands are deliberately trivial - a sum and a byte-swap of the first
 * payload word - because what is being tested is the path, not the arithmetic.
 */

#define REG(address) (*(volatile unsigned int *)(address))

/* HPI (SPRU969B section 8; the C6455 places the module at 0x02880000). */
#define HPIC 0x02880030u
#define HPIC_HINT (1u << 2)
#define HPIC_DSPINT (1u << 1)

/* Megamodule interrupt controller (SPRU871K chapter 7). */
#define EVTCLR0 0x01800040u
#define INTMUX1 0x01800104u
/* INTSEL4 selects event 15 - the vector table's fifth entry is the one that branches to the
 * service routine - and INTSEL5 to INTSEL7 keep their reset values. */
#define INTMUX1_INT4_HPI 0x0706050Fu
#define HPI_EVENT 15u

#define IER_NMIE (1u << 1)
#define IER_INT4 (1u << 4)

/* The mailbox, at fixed addresses the host also knows. */
/* Well clear of the image: the linker fills L2 SRAM from its base, and the mailbox must not be
 * anywhere the program's own data, heap or stack can reach. */
#define REQUEST 0x00840000u
#define REPLY   0x00840100u
#define PAYLOAD 0x00840200u

#define COMMAND_SUM 1u
#define COMMAND_SWAP 2u
#define EXCHANGES 4u

extern void intc_setup(unsigned int istp, unsigned int ier);
extern void intc_enable(void);
extern unsigned int ist_table;

static volatile unsigned int served;
static volatile unsigned int last_command;

interrupt void hpi_isr(void)
{
    unsigned int command = REG(REQUEST + 0u);
    unsigned int words = REG(REQUEST + 4u);
    unsigned int sequence = REG(REQUEST + 8u);
    unsigned int answer = 0;
    unsigned int index;

    if (command == COMMAND_SUM) {
        for (index = 0; index < words; index++) {
            answer += REG(PAYLOAD + index * 4u);
        }
    } else if (command == COMMAND_SWAP) {
        unsigned int word = REG(PAYLOAD);

        answer = ((word & 0x000000FFu) << 24) | ((word & 0x0000FF00u) << 8) |
                 ((word & 0x00FF0000u) >> 8) | ((word & 0xFF000000u) >> 24);
    } else {
        answer = 0xDEAD0000u | command;
    }

    REG(REPLY + 0u) = sequence;
    REG(REPLY + 4u) = answer;
    last_command = command;
    served = served + 1u;

    /* Acknowledge the host's interrupt and raise ours, then clear the megamodule event so the
     * next DSPINT can be announced. */
    REG(HPIC) = HPIC_DSPINT;
    REG(HPIC) = HPIC_HINT;
    REG(EVTCLR0) = 1u << HPI_EVENT;
}

int main(void)
{
    unsigned int spins;

    served = 0u;
    last_command = 0u;
    REG(REPLY + 0u) = 0u;
    REG(REPLY + 4u) = 0u;

    REG(INTMUX1) = INTMUX1_INT4_HPI;
    intc_setup((unsigned int)&ist_table, IER_INT4 | IER_NMIE);
    intc_enable();

    /* The host must be let in: the software reset holds every host access off until the CPU
     * clears it (SPRU969B section 3.2). */
    REG(HPIC) = 0u;

    printf("hpi mailbox ready\n");
    fflush(stdout);

    for (spins = 0; spins < 2000000000u && served < EXCHANGES; spins++) {
    }

    printf("hpi mailbox served %u last %u\n", served, last_command);
    fflush(stdout);
    return 0;
}
