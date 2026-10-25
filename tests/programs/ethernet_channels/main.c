#include <stdio.h>
/* Multi-channel EMAC: two unicast addresses routed to different receive channels by the CHANNEL
   field of their address RAM entries, broadcast steered by RXBROADCH, hashed multicast by
   RXMULTCH, one address deliberately unmatched, and two transmit channels. Every routing rule
   here is SPRU975E section 2.10 and Table 49. */

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
#define RXFREEBUFFER(c) (EMAC_BASE + 0x140u + 4u * (c))
#define MACCONTROL     (EMAC_BASE + 0x160u)
#define MACHASH1       (EMAC_BASE + 0x1D8u)
#define MACHASH2       (EMAC_BASE + 0x1DCu)
#define RXFILTERED     (EMAC_BASE + 0x228u)
#define MACADDRLO      (EMAC_BASE + 0x500u)
#define MACADDRHI      (EMAC_BASE + 0x504u)
#define MACINDEX       (EMAC_BASE + 0x508u)
#define TXHDP(c)       (EMAC_BASE + 0x600u + 4u * (c))
#define RXHDP(c)       (EMAC_BASE + 0x620u + 4u * (c))
#define TXCP(c)        (EMAC_BASE + 0x640u + 4u * (c))
#define RXCP(c)        (EMAC_BASE + 0x660u + 4u * (c))
#define EWCTL          0x02C81004u

#define MACADDR_VALID     (1u << 20)
#define MACADDR_MATCH     (1u << 19)
#define MACCONTROL_GMIIEN (1u << 5)

#define RXMULTEN     (1u << 5)
#define RXBROADEN    (1u << 13)
/* Receive copy short frames: frames below the 64-byte wire minimum are only transferred to
   memory when this bit is set (SPRU975E Table 49). */
#define RXCSFEN      (1u << 23)

#define CPPI_SOP   (1u << 31)
#define CPPI_EOP   (1u << 30)
#define CPPI_OWNER (1u << 29)

#define CHANNELS 4
#define FRAMES 5
#define BUFFER_SIZE 128

struct descriptor {
    volatile unsigned int next;
    volatile unsigned int buffer;
    volatile unsigned int offset_length;
    volatile unsigned int flags_length;
};

static struct descriptor rx_descriptors[CHANNELS];
static unsigned char rx_buffers[CHANNELS][BUFFER_SIZE];
static struct descriptor tx_descriptors[2];
static unsigned char tx_buffers[2][BUFFER_SIZE];

static void poke(unsigned int address, unsigned int value)
{
    *(volatile unsigned int *)address = value;
}

static unsigned int peek(unsigned int address)
{
    return *(volatile unsigned int *)address;
}

/* Program one address RAM slot: wire bytes 5 and 4 plus VALID, MATCH and the channel go in
   MACADDRLO, bytes 3, 2, 1, 0 in MACADDRHI (SPRU975E Tables 71 and 72). */
static void program_address(unsigned int slot, const unsigned char *address, unsigned int channel)
{
    /* MACINDEX selects which slot the next MACADDRLO/HI write lands in, so each write has to
       reach the peripheral before the next one is issued. Reading the index back between the
       writes makes that ordering explicit; at -O3 the compiler is otherwise free to hoist the
       next index write above the previous address write. */
    poke(MACINDEX, slot);
    (void)peek(MACINDEX);
    poke(MACADDRHI, ((unsigned int)address[3] << 24) | ((unsigned int)address[2] << 16)
                        | ((unsigned int)address[1] << 8) | address[0]);
    (void)peek(MACINDEX);
    poke(MACADDRLO, ((unsigned int)address[5] << 8) | address[4] | MACADDR_VALID | MACADDR_MATCH
                        | (channel << 16));
    (void)peek(MACADDRLO);
}

static void post_receive(unsigned int channel)
{
    rx_descriptors[channel].next = 0;
    rx_descriptors[channel].buffer = (unsigned int)&rx_buffers[channel][0];
    rx_descriptors[channel].offset_length = BUFFER_SIZE;
    rx_descriptors[channel].flags_length = CPPI_OWNER;
    poke(RXFREEBUFFER(channel), 1);
    poke(RXHDP(channel), (unsigned int)&rx_descriptors[channel]);
}

static const unsigned char station_a[6] = {0x02, 0x00, 0x00, 0x11, 0x22, 0x33};
static const unsigned char station_b[6] = {0x02, 0x00, 0x00, 0x44, 0x55, 0x66};

int main(void)
{
    unsigned int total = 0;
    unsigned int channel;
    unsigned int spins;
    unsigned int received = 0;
    unsigned int i;

    poke(EWCTL, 1);
    program_address(0, station_a, 0);
    program_address(1, station_b, 2);
    /* Broadcast to channel 3, hash-matching multicast to channel 1, nothing promiscuous.
       RXCSFEN is set because the shortest injected frame is below the 64-byte wire minimum
       and would otherwise be filtered as undersized (SPRU975E Table 49). */
    poke(RXMBPENABLE,
         RXBROADEN | (3u << 8) | RXMULTEN | (1u << 0) | RXCSFEN);
    /* The multicast address 01:00:5e:01:02:03 hashes to bit 15 of MACHASH1. */
    poke(MACHASH1, 1u << 15);
    poke(MACHASH2, 0);
    /* Only channels 0 and 2 accept address-matched frames. */
    poke(RXUNICASTSET, (1u << 0) | (1u << 2));
    poke(RXMAXLEN, 1518);
    poke(RXBUFFEROFFSET, 0);
    poke(MACCONTROL, MACCONTROL_GMIIEN);
    poke(RXINTMASKSET, 0xff);
    poke(TXINTMASKSET, 0xff);
    for (channel = 0; channel < CHANNELS; channel++) {
        post_receive(channel);
    }
    poke(RXCONTROL, 1);
    poke(TXCONTROL, 1);

    /* Four of the five injected frames are routed, one is filtered. */
    for (spins = 0; spins < 200000u && received < 4u; spins++) {
        for (channel = 0; channel < CHANNELS; channel++) {
            unsigned int flags = rx_descriptors[channel].flags_length;
            if ((flags & CPPI_OWNER) != 0 || (flags & CPPI_SOP) == 0) {
                continue;
            }
            total = total * 31u + channel;
            total = total * 31u + (flags & 0xFFFFu);
            total = total * 31u + rx_buffers[channel][14];
            total = total * 31u + (peek(RXINTSTATRAW) & 0xffu);
            poke(RXCP(channel), (unsigned int)&rx_descriptors[channel]);
            rx_descriptors[channel].flags_length = 0;
            received++;
        }
    }

    /* Transmit one frame on channel 0 and one on channel 5, then check both completions. */
    for (i = 0; i < 2u; i++) {
        unsigned int slot = i == 0 ? 0u : 5u;
        unsigned int length = 60u + 4u * i;
        unsigned int j;
        for (j = 0; j < length; j++) {
            tx_buffers[i][j] = (unsigned char)(j + 16u * i);
        }
        tx_descriptors[i].next = 0;
        tx_descriptors[i].buffer = (unsigned int)&tx_buffers[i][0];
        tx_descriptors[i].offset_length = length;
        tx_descriptors[i].flags_length = CPPI_SOP | CPPI_EOP | CPPI_OWNER | length;
        poke(TXHDP(slot), (unsigned int)&tx_descriptors[i]);
        for (spins = 0; spins < 200000u; spins++) {
            if ((tx_descriptors[i].flags_length & CPPI_OWNER) == 0) {
                break;
            }
        }
        total = total * 31u + (peek(TXINTSTATRAW) & 0xffu);
        poke(TXCP(slot), (unsigned int)&tx_descriptors[i]);
        total = total * 31u + (peek(TXINTSTATRAW) & 0xffu);
    }

    printf("ethernet_channels=%u received=%u filtered=%u\n", total, received, peek(RXFILTERED));
    return 0;
}
