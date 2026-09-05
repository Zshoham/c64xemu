#include <stdio.h>
/* One interrupt line, two queues, and an edge-latched selector.
 *
 * The EMAC drives a single system event for the whole module - event 17, MACINT on the C6455 -
 * and the transmit and receive queues both raise it (SPRU975E section 2.16). The megamodule's
 * interrupt selector is fed by the raw event line, so what reaches IFR is the line's rising
 * edge; a service routine for a directly selected event needs neither to read nor to clear
 * EVTFLAG (SPRU871K section 7.4.2), and this program clears nothing, as `ethernet_no_evtclr`
 * already does for the receive queue alone.
 *
 * Put those two facts together and a driver that enables both masks takes on an obligation:
 * every pass has to acknowledge *both* queues. A completed transmit descriptor left
 * unacknowledged holds MACINT high, so a receive completion that arrives afterwards produces no
 * edge and no interrupt - the receive path simply stops being woken. That is the rule
 * `tests/firmware/ndk_tcp/nimu_emac.c` is written around, and this is the case that pins it.
 *
 * Three frames arrive. The service routine handles the first, echoes it, waits for the transmit
 * to complete so MACINT is held by the transmit queue, acknowledges only the receive queue, and
 * posts the next receive descriptor - so the second frame completes into memory with the line
 * already high. Nothing more happens until the program acknowledges the transmit queue as well.
 * Once both are quiet the line falls, the third frame raises it again, and the routine runs a
 * second time.
 */

#define EMAC_BASE      0x02C80000u
#define TXCONTROL      (EMAC_BASE + 0x004u)
#define RXCONTROL      (EMAC_BASE + 0x014u)
#define TXINTSTATRAW   (EMAC_BASE + 0x080u)
#define TXINTMASKSET   (EMAC_BASE + 0x088u)
#define RXINTSTATRAW   (EMAC_BASE + 0x0A0u)
#define RXINTMASKSET   (EMAC_BASE + 0x0A8u)
#define RXMBPENABLE    (EMAC_BASE + 0x100u)
#define RXUNICASTSET   (EMAC_BASE + 0x104u)
#define RXMAXLEN       (EMAC_BASE + 0x10Cu)
#define RXBUFFEROFFSET (EMAC_BASE + 0x110u)
#define RX0FREEBUFFER  (EMAC_BASE + 0x140u)
#define MACCONTROL     (EMAC_BASE + 0x160u)
#define MACADDRLO      (EMAC_BASE + 0x500u)
#define MACADDRHI      (EMAC_BASE + 0x504u)
#define MACINDEX       (EMAC_BASE + 0x508u)
#define TX0HDP         (EMAC_BASE + 0x600u)
#define RX0HDP         (EMAC_BASE + 0x620u)
#define TX0CP          (EMAC_BASE + 0x640u)
#define RX0CP          (EMAC_BASE + 0x660u)
#define EWCTL          0x02C81004u

#define MACADDR_VALID     (1u << 20)
#define MACADDR_MATCH     (1u << 19)
#define MACCONTROL_GMIIEN (1u << 5)

#define INTMUX1      0x01800104u
#define MACINT_EVENT 17u
#define CPU_INT4     4u

#define CPPI_SOP   (1u << 31)
#define CPPI_EOP   (1u << 30)
#define CPPI_OWNER (1u << 29)

#define FRAMES 3
#define BUFFER_SIZE 128
/* Long enough that the frames are not undersized on the wire, so nothing here depends on the
 * short-frame copy path: the wire length is four CRC bytes more than the injected data. */
#define MIN_FRAME 60

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
static struct descriptor tx_descriptor;
static unsigned char rx_buffers[FRAMES][BUFFER_SIZE];
static unsigned char tx_buffer[BUFFER_SIZE];
static volatile int entries;
static volatile int rx_next;
static volatile int errors;

static void poke(unsigned int address, unsigned int value)
{
    *(volatile unsigned int *)address = value;
}

static unsigned int peek(unsigned int address)
{
    return *(volatile unsigned int *)address;
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

/* Echo the frame in a receive buffer, and wait for the transmit to complete without
 * acknowledging it: on return MACINT is held high by the transmit queue. */
static void echo_without_acknowledging(int index, unsigned int length)
{
    unsigned int i;

    for (i = 0; i < 6u; i++) {
        tx_buffer[i] = rx_buffers[index][6u + i];
        tx_buffer[6u + i] = rx_buffers[index][i];
    }
    for (i = 12u; i < length; i++) {
        tx_buffer[i] = rx_buffers[index][i];
    }
    tx_descriptor.next = 0;
    tx_descriptor.buffer = (unsigned int)&tx_buffer[0];
    tx_descriptor.offset_length = length;
    tx_descriptor.flags_length = CPPI_SOP | CPPI_EOP | CPPI_OWNER | length;
    poke(TX0HDP, (unsigned int)&tx_descriptor);
    while ((tx_descriptor.flags_length & CPPI_OWNER) != 0) {
    }
}

interrupt void emac_isr(void)
{
    int index = rx_next;
    unsigned int length;

    if (index >= FRAMES || (rx_descriptors[index].flags_length & CPPI_OWNER) != 0) {
        errors = errors + 1;
        return;
    }
    length = rx_descriptors[index].flags_length & 0xFFFFu;
    if (length < MIN_FRAME || length > BUFFER_SIZE) {
        errors = errors + 1;
        return;
    }
    if (entries == 0) {
        /* Transmit first, so the transmit queue is holding the line before the receive queue
         * lets go of it. Acknowledging the receive queue while nothing else held MACINT would
         * drop the line and the next frame would produce a legitimate edge. */
        echo_without_acknowledging(index, length);
        poke(RX0CP, (unsigned int)&rx_descriptors[index]);
        rx_next = index + 1;
        /* The second frame now completes into memory with MACINT already high. */
        post_receive(index + 1);
    } else {
        poke(RX0CP, (unsigned int)&rx_descriptors[index]);
        rx_next = index + 1;
    }
    entries = entries + 1;
}

int main(void)
{
    unsigned int spins;
    int first;
    int suppressed;
    unsigned int rx_pending;
    unsigned int tx_pending;
    int woken;

    poke(INTMUX1, MACINT_EVENT);
    /* NMIE (IER bit 1) gates every maskable interrupt, so real init code sets it too. */
    intc_setup((unsigned int)&ist_table, (1u << CPU_INT4) | 2u);
    poke(EWCTL, 1);

    poke(MACINDEX, 0);
    poke(MACADDRHI, (0x00u << 24) | (0x11u << 16) | (0x22u << 8) | 0x33u);
    poke(MACADDRLO, (0x02u << 8) | 0x00u | MACADDR_VALID | MACADDR_MATCH);
    poke(RXUNICASTSET, 1);
    poke(RXMAXLEN, 1518);
    poke(RXBUFFEROFFSET, 0);
    poke(RXMBPENABLE, 1u);
    poke(MACCONTROL, MACCONTROL_GMIIEN);
    poke(TXINTMASKSET, 1);
    poke(RXINTMASKSET, 1);
    poke(RXCONTROL, 1);
    poke(TXCONTROL, 1);
    post_receive(0);
    intc_enable();

    /* The first frame's interrupt. */
    for (spins = 0; entries < 1 && spins < 400000u; spins++) {
    }
    first = entries;

    /* The second frame is in memory and unacknowledged, and so is the transmit. Neither can
     * raise the line again, because it never fell. */
    for (spins = 0; spins < 200000u; spins++) {
    }
    suppressed = entries;
    rx_pending = peek(RXINTSTATRAW) & 1u;
    tx_pending = peek(TXINTSTATRAW) & 1u;

    /* Acknowledge both queues. Only now does MACINT fall. */
    poke(TX0CP, (unsigned int)&tx_descriptor);
    poke(RX0CP, (unsigned int)&rx_descriptors[1]);
    rx_next = 2;
    /* The third frame raises it again, from low, so the routine runs a second time. */
    post_receive(2);
    for (spins = 0; entries < 2 && spins < 400000u; spins++) {
    }
    woken = entries;

    printf("shared macint: first=%d suppressed=%d rx=%u tx=%u woken=%d errors=%d\n", first,
           suppressed, rx_pending, tx_pending, woken, errors);
    return 0;
}
