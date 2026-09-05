/* An EMAC receive driver that never touches the megamodule event flags.
 *
 * SPRU871K section 7.4.2: a service routine for an event selected directly by the interrupt
 * selector - as MACINT is here - needs neither to read nor to clear EVTFLAG. This driver takes
 * that literally: it quiets the EMAC by writing the receive completion pointer and nothing else,
 * so each of the three injected frames has to produce its own interrupt through a fresh assertion
 * of MACINT rather than through a flag the routine cleared.
 *
 * The value returned is a rolling hash over every received byte, which is computed independently
 * in the spec, and the program refuses to report it unless all three frames were served without
 * an error.
 */

#define EMAC_BASE 0x02C80000u
#define TXCONTROL (EMAC_BASE + 0x004u)
#define RXCONTROL (EMAC_BASE + 0x014u)
#define RXINTMASKSET (EMAC_BASE + 0x0A8u)
#define RXMBPENABLE (EMAC_BASE + 0x100u)
#define RXBUFFEROFFSET (EMAC_BASE + 0x110u)
#define RX0FREEBUFFER (EMAC_BASE + 0x140u)
#define RX0HDP (EMAC_BASE + 0x620u)
#define RX0CP (EMAC_BASE + 0x660u)
#define EWCTL 0x02C81004u
#define RXUNICASTSET (EMAC_BASE + 0x104u)
#define RXMAXLEN (EMAC_BASE + 0x10Cu)
#define MACCONTROL (EMAC_BASE + 0x160u)
#define MACADDRLO (EMAC_BASE + 0x500u)
#define MACADDRHI (EMAC_BASE + 0x504u)
#define MACINDEX (EMAC_BASE + 0x508u)

#define MACADDR_VALID (1u << 20)
#define MACADDR_MATCH (1u << 19)
#define MACCONTROL_GMIIEN (1u << 5)
#define RXMBP_CSFEN (1u << 23)
#define RXMBP_BROADCAST (1u << 13)

#define INTMUX1 0x01800104u
#define MACINT_EVENT 17u
#define CPU_INT4 4u

#define CPPI_OWNER (1u << 29)

#define FRAMES 3
#define BUFFER_SIZE 192

struct descriptor {
    volatile unsigned int next;
    volatile unsigned int buffer;
    volatile unsigned int offset_length;
    volatile unsigned int flags_length;
};

extern void intc_setup(unsigned int istp, unsigned int ier);
extern void intc_enable(void);
extern unsigned int ist_table;

static struct descriptor rx_descriptors[FRAMES];
static unsigned char rx_buffers[FRAMES][BUFFER_SIZE];
static volatile int served;
static volatile unsigned int hash;
static volatile int errors;

static void poke(unsigned int address, unsigned int value)
{
    *(volatile unsigned int *)address = value;
}

static void program_address(void)
{
    poke(MACINDEX, 0);
    /* Station address 02:00:00:11:22:33 (SPRU975E sections 5.42 to 5.44). */
    poke(MACADDRHI, (0x00u << 24) | (0x11u << 16) | (0x22u << 8) | 0x33u);
    poke(MACADDRLO, (0x02u << 8) | MACADDR_VALID | MACADDR_MATCH);
    poke(RXUNICASTSET, 1);
    poke(RXMAXLEN, 1518);
    poke(MACCONTROL, MACCONTROL_GMIIEN);
}

static void post_receive(int index)
{
    rx_descriptors[index].next = 0;
    rx_descriptors[index].buffer = (unsigned int)&rx_buffers[index][0];
    rx_descriptors[index].offset_length = BUFFER_SIZE;
    rx_descriptors[index].flags_length = CPPI_OWNER;
    poke(RX0FREEBUFFER, 1);
    poke(RX0HDP, (unsigned int)&rx_descriptors[index]);
}

interrupt void emac_isr(void)
{
    int index = served;
    unsigned int length;
    unsigned int i;

    if (index >= FRAMES) {
        return;
    }
    if ((rx_descriptors[index].flags_length & CPPI_OWNER) != 0) {
        /* Nothing of ours yet; the EMAC will assert again when a frame lands. */
        return;
    }
    length = rx_descriptors[index].flags_length & 0xFFFFu;
    if (length < 14u || length > BUFFER_SIZE) {
        errors = errors + 1;
        poke(RX0CP, (unsigned int)&rx_descriptors[index]);
        return;
    }
    for (i = 0; i < length; i++) {
        hash = hash * 31u + (unsigned int)rx_buffers[index][i];
    }
    /* The only acknowledgement: the receive completion pointer. */
    poke(RX0CP, (unsigned int)&rx_descriptors[index]);
    served = index + 1;
    if (served < FRAMES) {
        post_receive(served);
    }
}

int main(void)
{
    unsigned int spins;

    served = 0;
    errors = 0;
    hash = 0u;

    poke(INTMUX1, MACINT_EVENT);
    intc_setup((unsigned int)&ist_table, (1u << CPU_INT4) | 2u);
    poke(EWCTL, 1);
    program_address();
    poke(RXMBPENABLE, RXMBP_BROADCAST | 1u | RXMBP_CSFEN);
    poke(RXBUFFEROFFSET, 0);
    poke(RXINTMASKSET, 1);
    poke(RXCONTROL, 1);
    poke(TXCONTROL, 1);
    post_receive(0);
    intc_enable();

    for (spins = 0; spins < 2000000u && served < FRAMES; spins++) {
    }
    if (served != FRAMES || errors != 0) {
        return -1;
    }
    return (int)hash;
}
