#include <stdio.h>
/* Receive filtering, statistics and channel teardown: the station address goes into the
   address RAM, broadcast and hashed multicast reception are enabled, RXMAXLEN rejects long
   frames, and the receive channel is torn down at the end. */

#define EMAC_BASE      0x02C80000u
#define TXCONTROL      (EMAC_BASE + 0x004u)
#define RXTEARDOWN     (EMAC_BASE + 0x018u)
#define RXCONTROL      (EMAC_BASE + 0x014u)
#define RXMBPENABLE    (EMAC_BASE + 0x100u)
#define RXUNICASTSET   (EMAC_BASE + 0x104u)
#define RXMAXLEN       (EMAC_BASE + 0x10Cu)
#define MACCONTROL     (EMAC_BASE + 0x160u)
#define MACHASH1       (EMAC_BASE + 0x1D8u)
#define MACHASH2       (EMAC_BASE + 0x1DCu)
#define RXGOODFRAMES   (EMAC_BASE + 0x200u)
#define RXBCASTFRAMES  (EMAC_BASE + 0x204u)
#define RXMCASTFRAMES  (EMAC_BASE + 0x208u)
#define RXOVERSIZED    (EMAC_BASE + 0x218u)
#define RXFILTERED     (EMAC_BASE + 0x228u)
#define TXGOODFRAMES   (EMAC_BASE + 0x234u)
#define RXBUFFEROFFSET (EMAC_BASE + 0x110u)
#define RX0FREEBUFFER  (EMAC_BASE + 0x140u)
#define TX0HDP         (EMAC_BASE + 0x600u)
#define RX0HDP         (EMAC_BASE + 0x620u)
#define TX0CP          (EMAC_BASE + 0x640u)
#define RX0CP          (EMAC_BASE + 0x660u)
#define MACADDRLO      (EMAC_BASE + 0x500u)
#define MACADDRHI      (EMAC_BASE + 0x504u)
#define MACINDEX       (EMAC_BASE + 0x508u)
#define EWCTL          0x02C81004u

#define RXMBP_MULTICAST (1u << 5)
#define RXMBP_BROADCAST (1u << 13)
#define MACADDR_VALID     (1u << 20)
#define MACADDR_MATCH     (1u << 19)
#define MACCONTROL_GMIIEN (1u << 5)

#define CPPI_SOP        (1u << 31)
#define CPPI_EOP        (1u << 30)
#define CPPI_OWNER      (1u << 29)
#define CPPI_TDOWNCMPLT (1u << 27)
#define TEARDOWN_MARKER 0xFFFFFFFCu

#define SLOTS 4
#define BUFFER_SIZE 160
#define MULTICAST_HASH 17u

struct descriptor {
    volatile unsigned int next;
    volatile unsigned int buffer;
    volatile unsigned int offset_length;
    volatile unsigned int flags_length;
};

static struct descriptor rx_descriptors[SLOTS];
static struct descriptor tx_descriptors[SLOTS];
static unsigned char rx_buffers[SLOTS][BUFFER_SIZE];
static unsigned char tx_buffers[SLOTS][BUFFER_SIZE];

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

int main(void)
{
    int index;
    int served = 0;
    int checksum = 0;
    unsigned int teardown_cp;
    unsigned int teardown_flags;

    poke(EWCTL, 1);
    poke(MACINDEX, 0);
    /* Station address 02:00:00:11:22:33. */
    poke(MACADDRHI, 0x11000002u);
    poke(MACADDRLO, 0x3322u | MACADDR_VALID | MACADDR_MATCH);
    poke(RXUNICASTSET, 1);
    /* Only the hash bucket of 01:00:5e:11:22:33 is accepted. */
    poke(MACHASH1, 1u << MULTICAST_HASH);
    poke(MACHASH2, 0);
    poke(RXMAXLEN, 100);
    poke(MACCONTROL, MACCONTROL_GMIIEN);
    poke(RXMBPENABLE, RXMBP_BROADCAST | RXMBP_MULTICAST);
    poke(RXBUFFEROFFSET, 0);
    poke(RXCONTROL, 1);
    poke(TXCONTROL, 1);

    for (index = 0; index < 3; index++) {
        unsigned int spins = 0;
        unsigned int length;
        unsigned char *in;
        unsigned char *out;
        unsigned int i;

        post_receive(index);
        while ((rx_descriptors[index].flags_length & CPPI_OWNER) != 0 && spins < 200000u) {
            spins++;
        }
        if ((rx_descriptors[index].flags_length & CPPI_OWNER) != 0) {
            break;
        }
        length = rx_descriptors[index].flags_length & 0xFFFFu;
        poke(RX0CP, (unsigned int)&rx_descriptors[index]);
        in = &rx_buffers[index][0];
        out = &tx_buffers[index][0];
        for (i = 0; i < 6u; i++) {
            out[i] = in[6u + i];
            out[6u + i] = in[i];
        }
        for (i = 12u; i < length; i++) {
            out[i] = in[i];
        }
        for (i = 0; i < length; i++) {
            checksum = checksum * 31 + (int)in[i];
        }
        tx_descriptors[index].next = 0;
        tx_descriptors[index].buffer = (unsigned int)out;
        tx_descriptors[index].offset_length = length;
        tx_descriptors[index].flags_length = CPPI_SOP | CPPI_EOP | CPPI_OWNER | length;
        poke(TX0HDP, (unsigned int)&tx_descriptors[index]);
        while ((tx_descriptors[index].flags_length & CPPI_OWNER) != 0) {
        }
        poke(TX0CP, (unsigned int)&tx_descriptors[index]);
        served++;
    }

    /* Tear the receive channel down: the head pointer is cleared, the pending descriptor is
       marked and the completion pointer reads the teardown marker. */
    post_receive(3);
    poke(RXTEARDOWN, 0);
    teardown_cp = peek(RX0CP);
    teardown_flags = rx_descriptors[3].flags_length;
    poke(RX0CP, TEARDOWN_MARKER);

    /* An address-matching frame longer than RXMAXLEN is a long frame: it counts as oversized
       and is only transferred with RXCEFEN set, so it never reaches RXFILTERED, which counts
       address-match rejections alone (SPRU975E sections 2.10.7, 5.49.7 and 5.49.11). */
    printf("ethernet_filtering=%d served=%d good=%u bcast=%u mcast=%u filtered=%u "
           "oversized=%u tx=%u\n",
           checksum, served, peek(RXGOODFRAMES), peek(RXBCASTFRAMES), peek(RXMCASTFRAMES),
           peek(RXFILTERED), peek(RXOVERSIZED), peek(TXGOODFRAMES));
    printf("teardown cp=%u marked=%u hdp=%u\n", teardown_cp == TEARDOWN_MARKER ? 1u : 0u,
           (teardown_flags & CPPI_TDOWNCMPLT) != 0u ? 1u : 0u, peek(RX0HDP));
    return 0;
}
