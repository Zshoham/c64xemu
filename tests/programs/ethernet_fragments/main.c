#include <stdio.h>
/* A receive ring of small buffers: a packet spans one descriptor per fragment, so the driver
   walks the chain from SOP to EOP to reassemble it, and transmits it back as two fragments. */

#define EMAC_BASE      0x02C80000u
#define TXCONTROL      (EMAC_BASE + 0x004u)
#define RXCONTROL      (EMAC_BASE + 0x014u)
#define RXMBPENABLE    (EMAC_BASE + 0x100u)
#define RXUNICASTSET   (EMAC_BASE + 0x104u)
#define RXMAXLEN       (EMAC_BASE + 0x10Cu)
#define RXBUFFEROFFSET (EMAC_BASE + 0x110u)
#define RX0FREEBUFFER  (EMAC_BASE + 0x140u)
#define MACCONTROL     (EMAC_BASE + 0x160u)
#define TX0HDP         (EMAC_BASE + 0x600u)
#define RX0HDP         (EMAC_BASE + 0x620u)
#define TX0CP          (EMAC_BASE + 0x640u)
#define RX0CP          (EMAC_BASE + 0x660u)
#define MACADDRLO      (EMAC_BASE + 0x500u)
#define MACADDRHI      (EMAC_BASE + 0x504u)
#define MACINDEX       (EMAC_BASE + 0x508u)
#define EWCTL          0x02C81004u

#define MACADDR_VALID     (1u << 20)
#define MACADDR_MATCH     (1u << 19)
#define MACCONTROL_GMIIEN (1u << 5)
#define RXMBP_BROADCAST   (1u << 13)

#define CPPI_SOP   (1u << 31)
#define CPPI_EOP   (1u << 30)
#define CPPI_OWNER (1u << 29)
#define CPPI_EOQ   (1u << 28)

#define RING 6
#define FRAGMENT 32
#define OFFSET 4
#define ASSEMBLED 256

struct descriptor {
    volatile unsigned int next;
    volatile unsigned int buffer;
    volatile unsigned int offset_length;
    volatile unsigned int flags_length;
};

static struct descriptor rx_ring[RING];
static struct descriptor tx_ring[2];
static unsigned char rx_buffers[RING][FRAGMENT + OFFSET];
static unsigned char assembled[ASSEMBLED];
static unsigned char tx_buffer[ASSEMBLED];

static void poke(unsigned int address, unsigned int value)
{
    *(volatile unsigned int *)address = value;
}

static unsigned int peek(unsigned int address)
{
    return *(volatile unsigned int *)address;
}

static void post_ring(void)
{
    int i;
    for (i = 0; i < RING; i++) {
        rx_ring[i].next = (i + 1 < RING) ? (unsigned int)&rx_ring[i + 1] : 0u;
        rx_ring[i].buffer = (unsigned int)&rx_buffers[i][0];
        rx_ring[i].offset_length = FRAGMENT;
        rx_ring[i].flags_length = CPPI_OWNER;
    }
    poke(RX0FREEBUFFER, RING);
    poke(RX0HDP, (unsigned int)&rx_ring[0]);
}

int main(void)
{
    unsigned int spins = 0;
    unsigned int packet_length;
    unsigned int total = 0;
    int fragments = 0;
    int index;
    int checksum = 0;
    unsigned int i;
    unsigned int eoq_seen = 0;

    poke(EWCTL, 1);
    poke(MACINDEX, 0);
    poke(MACADDRHI, (0x00u << 24) | (0x11u << 16) | (0x22u << 8) | 0x33u);
    poke(MACADDRLO, (0x02u << 8) | 0x00u | MACADDR_VALID | MACADDR_MATCH);
    poke(RXUNICASTSET, 1);
    poke(RXMAXLEN, 1518);
    poke(MACCONTROL, MACCONTROL_GMIIEN);
    poke(RXMBPENABLE, RXMBP_BROADCAST);
    /* The packet starts four bytes into the first buffer. */
    poke(RXBUFFEROFFSET, OFFSET);
    poke(RXCONTROL, 1);
    poke(TXCONTROL, 1);
    post_ring();

    while ((rx_ring[0].flags_length & CPPI_OWNER) != 0 && spins < 400000u) {
        spins++;
    }
    if ((rx_ring[0].flags_length & CPPI_OWNER) != 0) {
        printf("ethernet_fragments=timeout\n");
        return 0;
    }
    packet_length = rx_ring[0].flags_length & 0xFFFFu;
    for (index = 0; index < RING; index++) {
        unsigned int flags = rx_ring[index].flags_length;
        unsigned int offset = rx_ring[index].offset_length >> 16;
        unsigned int length = rx_ring[index].offset_length & 0xFFFFu;
        unsigned char *source = (unsigned char *)(rx_ring[index].buffer + offset);
        fragments++;
        for (i = 0; i < length && total < ASSEMBLED; i++) {
            assembled[total++] = source[i];
        }
        if ((flags & CPPI_EOQ) != 0u) {
            eoq_seen = 1;
        }
        if ((flags & CPPI_EOP) != 0u) {
            break;
        }
    }
    poke(RX0CP, (unsigned int)&rx_ring[index]);

    for (i = 0; i < total; i++) {
        checksum = checksum * 31 + (int)assembled[i];
    }
    /* Send it back as two fragments, addresses swapped. */
    for (i = 0; i < 6u; i++) {
        tx_buffer[i] = assembled[6u + i];
        tx_buffer[6u + i] = assembled[i];
    }
    for (i = 12u; i < total; i++) {
        tx_buffer[i] = assembled[i];
    }
    tx_ring[0].next = (unsigned int)&tx_ring[1];
    tx_ring[0].buffer = (unsigned int)&tx_buffer[0];
    tx_ring[0].offset_length = 20;
    tx_ring[0].flags_length = CPPI_SOP | CPPI_OWNER | total;
    tx_ring[1].next = 0;
    tx_ring[1].buffer = (unsigned int)&tx_buffer[20];
    tx_ring[1].offset_length = total - 20u;
    tx_ring[1].flags_length = CPPI_EOP | CPPI_OWNER;
    poke(TX0HDP, (unsigned int)&tx_ring[0]);
    while ((tx_ring[1].flags_length & CPPI_OWNER) != 0) {
    }
    poke(TX0CP, (unsigned int)&tx_ring[1]);

    printf("ethernet_fragments=%d length=%u total=%u fragments=%d eoq=%u hdp=%u\n", checksum,
           packet_length, total, fragments, eoq_seen, peek(RX0HDP));
    return 0;
}
