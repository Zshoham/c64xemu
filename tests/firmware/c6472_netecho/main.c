/* Six cores echoing Ethernet frames at once through the C6472's shared EMAC.
 *
 * The device has one MAC per port and six megamodules behind it (SPRS612G Table 7-105), so the
 * MAC's register file - its address RAM, its eight receive channels and its eight transmit
 * channels - is one thing every core writes. This firmware uses it the way the hardware is
 * designed for: core 0 brings the MAC up and binds receive channel N to unicast address slot N,
 * then every core owns one channel and echoes the frames addressed to its own MAC address back
 * to the sender. Nothing here is shared between cores except the MAC and the descriptor memory,
 * and no core touches another core's channel.
 *
 * Two device properties make the addresses in this file what they are:
 *
 *   - A DMA master's address on a multicore device is always global (SPRS612G Table 2-2 marks a
 *     megamodule's local window "DATA SPACE ON DMA"), so every descriptor and buffer pointer the
 *     MAC is given is the global alias of the shared L2 address the CPU uses: the same bytes,
 *     0x1000 0000 higher.
 *   - A receive queue that reaches a null next pointer sets EOQ, clears RXnHDP and stays
 *     stopped (SPRU975E section 4.3.2). The rings here are circular, so the queue never reaches
 *     one, and the EOQ case is handled anyway.
 *
 * The frames are raw Ethernet with a local experimental type, because what is being measured is
 * six cores moving frames concurrently and not a protocol stack: the payload carries the core
 * number and a sequence number, and the host peer checks that every byte it sent to core N came
 * back from core N in order.
 */

#include <c6x.h>

#define CORES 6

/* EMAC0 and its register offsets (SPRS612G Table 2-2, SPRU975E Table 12). The EMAC control
 * module window belongs to the per-core interrupt controller on this device, not to the MAC, so
 * nothing here writes it: the driver polls. */
#define EMAC_BASE 0x02C80000u
#define TXCONTROL (*(volatile unsigned *)(EMAC_BASE + 0x004u))
#define RXCONTROL (*(volatile unsigned *)(EMAC_BASE + 0x014u))
#define RXMBPENABLE (*(volatile unsigned *)(EMAC_BASE + 0x100u))
#define RXUNICASTSET (*(volatile unsigned *)(EMAC_BASE + 0x104u))
#define RXMAXLEN (*(volatile unsigned *)(EMAC_BASE + 0x10Cu))
#define RXBUFFEROFFSET (*(volatile unsigned *)(EMAC_BASE + 0x110u))
#define MACCONTROL (*(volatile unsigned *)(EMAC_BASE + 0x160u))
#define SOFTRESET (*(volatile unsigned *)(EMAC_BASE + 0x174u))
#define MACHASH1 (*(volatile unsigned *)(EMAC_BASE + 0x1D8u))
#define MACHASH2 (*(volatile unsigned *)(EMAC_BASE + 0x1DCu))
#define MACADDRLO (*(volatile unsigned *)(EMAC_BASE + 0x500u))
#define MACADDRHI (*(volatile unsigned *)(EMAC_BASE + 0x504u))
#define MACINDEX (*(volatile unsigned *)(EMAC_BASE + 0x508u))
#define TXHDP(channel) (*(volatile unsigned *)(EMAC_BASE + 0x600u + 4u * (channel)))
#define RXHDP(channel) (*(volatile unsigned *)(EMAC_BASE + 0x620u + 4u * (channel)))
#define TXCP(channel) (*(volatile unsigned *)(EMAC_BASE + 0x640u + 4u * (channel)))
#define RXCP(channel) (*(volatile unsigned *)(EMAC_BASE + 0x660u + 4u * (channel)))

#define MACCONTROL_FULLDUPLEX (1u << 0)
#define MACCONTROL_GMIIEN (1u << 5)
#define RXMBP_BROADCAST (1u << 13)
/* An address slot decides a frame's channel: VALID and MATCH make it a matching slot and the
 * channel is bits 18:16 of MACADDRLO (SPRU975E Table 71). */
#define MACADDR_VALID (1u << 20)
#define MACADDR_MATCH (1u << 19)

#define CPPI_SOP (1u << 31)
#define CPPI_EOP (1u << 30)
#define CPPI_OWNER (1u << 29)
#define CPPI_EOQ (1u << 28)

/* What a DMA master adds to a shared L2 address to reach the same bytes (SPRS612G Table 2-2:
 * 0020 0000h in a core's own view is 1020 0000h from the switch fabric). */
#define DMA_ALIAS 0x10000000u

#define RX_SLOTS 16
#define TX_SLOTS 1
#define FRAME_MAX 1536
#define FRAME_TYPE_HI 0x88u
#define FRAME_TYPE_LO 0xB5u
/* The sequence number a core's announcement carries, which is how the host peer learns that a
 * core has armed its channel and is ready to be paced. */
#define HELLO_SEQUENCE 0xFFFFu

typedef struct Descriptor {
    volatile unsigned next;
    volatile unsigned buffer;
    volatile unsigned offset_length;
    volatile unsigned flags_length;
} Descriptor;

/* The rings and the buffers live in the shared L2 window, which is the one memory every core
 * and every DMA master reaches: a core reads and writes them at 0020 0000h and the MAC is given
 * their global alias. `NOINIT` keeps them out of the image. */
#pragma DATA_SECTION(rx_ring, ".shared")
Descriptor rx_ring[CORES][RX_SLOTS];
#pragma DATA_SECTION(tx_ring, ".shared")
Descriptor tx_ring[CORES][TX_SLOTS];
#pragma DATA_SECTION(rx_frame, ".shared")
unsigned char rx_frame[CORES][RX_SLOTS][FRAME_MAX];
#pragma DATA_SECTION(tx_frame, ".shared")
unsigned char tx_frame[CORES][TX_SLOTS][FRAME_MAX];
/* Core 0 raises this once the MAC answers; the other five wait for it before arming a channel. */
#pragma DATA_SECTION(mac_ready, ".shared")
volatile unsigned mac_ready;
/* How many frames each core has echoed, for a memory dump to read. */
#pragma DATA_SECTION(echoed, ".shared")
volatile unsigned echoed[CORES];

static unsigned dma_address(volatile void *pointer)
{
    return (unsigned)pointer + DMA_ALIAS;
}

/* The MAC address core `core` answers to: 02:00:00:64:72:0N. The host peer builds the same six
 * addresses from the same rule. */
static void core_address(unsigned core, unsigned char *address)
{
    address[0] = 0x02u;
    address[1] = 0x00u;
    address[2] = 0x00u;
    address[3] = 0x64u;
    address[4] = 0x72u;
    address[5] = (unsigned char)core;
}

/* Bring the MAC up and bind one receive channel per core. Core 0 alone runs this: the register
 * file is the device's, not a core's. */
static void mac_setup(void)
{
    unsigned index;
    unsigned char address[6];

    /* A known state first: the reset clears every channel, mask and address slot. */
    SOFTRESET = 1u;
    for (index = 0; index < 8u; index++) {
        TXHDP(index) = 0u;
        RXHDP(index) = 0u;
        TXCP(index) = 0u;
        RXCP(index) = 0u;
    }

    /* Address slot N matches core N's address and names receive channel N. */
    for (index = 0; index < CORES; index++) {
        core_address(index, address);
        MACINDEX = index;
        MACADDRHI = ((unsigned)address[3] << 24) | ((unsigned)address[2] << 16) |
                    ((unsigned)address[1] << 8) | (unsigned)address[0];
        MACADDRLO = ((unsigned)address[5] << 8) | (unsigned)address[4] | MACADDR_VALID |
                    MACADDR_MATCH | (index << 16);
    }
    /* Unicast reception is per channel, and nothing else routes these frames. */
    RXUNICASTSET = (1u << CORES) - 1u;
    MACHASH1 = 0u;
    MACHASH2 = 0u;
    RXMAXLEN = 1518u;
    RXBUFFEROFFSET = 0u;
    /* Broadcast goes to channel 0, which is core 0's; nothing in this exchange needs it, but a
     * peer that ARPs first should not be silently dropped. */
    RXMBPENABLE = RXMBP_BROADCAST;
    MACCONTROL = MACCONTROL_GMIIEN | MACCONTROL_FULLDUPLEX;
    RXCONTROL = 1u;
    TXCONTROL = 1u;
}

/* Arm this core's receive channel with a circular chain of owned descriptors. */
static void ring_setup(unsigned core)
{
    unsigned index;

    for (index = 0; index < RX_SLOTS; index++) {
        Descriptor *descriptor = &rx_ring[core][index];

        descriptor->next = dma_address(&rx_ring[core][(index + 1u) % RX_SLOTS]);
        descriptor->buffer = dma_address(&rx_frame[core][index][0]);
        descriptor->offset_length = FRAME_MAX;
        descriptor->flags_length = CPPI_OWNER;
    }
    for (index = 0; index < TX_SLOTS; index++) {
        tx_ring[core][index].next = 0u;
        tx_ring[core][index].buffer = 0u;
        tx_ring[core][index].offset_length = 0u;
        tx_ring[core][index].flags_length = 0u;
    }
    echoed[core] = 0u;
    RXHDP(core) = dma_address(&rx_ring[core][0]);
}

/* Put one frame on this core's transmit channel and wait for the previous one to have left.
 * One descriptor per core is enough: a frame is echoed only when one has arrived, and the
 * channel completes a packet per arbitration tick. */
static void transmit(unsigned core, unsigned length)
{
    Descriptor *descriptor = &tx_ring[core][0];

    descriptor->next = 0u;
    descriptor->buffer = dma_address(&tx_frame[core][0][0]);
    descriptor->offset_length = length;
    descriptor->flags_length = CPPI_SOP | CPPI_EOP | CPPI_OWNER | length;
    TXHDP(core) = dma_address(descriptor);
    /* Wait for the MAC to hand the descriptor back, then acknowledge it so the channel's
     * completion interrupt would drop if anyone were taking it. */
    while ((descriptor->flags_length & CPPI_OWNER) != 0u) {
    }
    TXCP(core) = dma_address(descriptor);
}

/* The announcement a core sends once its channel is armed: a broadcast frame whose payload
 * names the core and carries the reserved sequence number. */
static void announce(unsigned core)
{
    unsigned char *out = &tx_frame[core][0][0];
    unsigned char address[6];
    unsigned index;

    core_address(core, address);
    for (index = 0; index < 6u; index++) {
        out[index] = 0xFFu;
        out[6u + index] = address[index];
    }
    out[12] = FRAME_TYPE_HI;
    out[13] = FRAME_TYPE_LO;
    out[14] = (unsigned char)core;
    out[15] = 0u;
    out[16] = (unsigned char)(HELLO_SEQUENCE >> 8);
    out[17] = (unsigned char)(HELLO_SEQUENCE & 0xFFu);
    for (index = 18; index < 60u; index++) {
        out[index] = 0u;
    }
    transmit(core, 60u);
}

/* Echo one received frame back to whoever sent it: the addresses swap, everything else - the
 * type, the core number, the sequence number and the payload - is returned unchanged. */
static void echo_frame(unsigned core, const unsigned char *frame, unsigned length)
{
    unsigned char *out = &tx_frame[core][0][0];
    unsigned index;

    for (index = 0; index < 6u; index++) {
        out[index] = frame[6u + index];
        out[6u + index] = frame[index];
    }
    for (index = 12u; index < length; index++) {
        out[index] = frame[index];
    }
    transmit(core, length);
}

/* This core's share of the work: poll its own receive channel and echo what arrives. The frames
 * are single-descriptor - a buffer holds the longest frame the MAC accepts - so the descriptor
 * the queue fills next is always the next one round the ring. */
static void echo_loop(unsigned core)
{
    unsigned index = 0;

    for (;;) {
        Descriptor *descriptor = &rx_ring[core][index];
        unsigned flags = descriptor->flags_length;
        unsigned length;

        if ((flags & CPPI_OWNER) != 0u) {
            continue;
        }
        length = flags & 0xFFFFu;
        RXCP(core) = dma_address(descriptor);
        if (length > 14u && length <= FRAME_MAX) {
            echo_frame(core, &rx_frame[core][index][0], length);
            echoed[core] = echoed[core] + 1u;
        }
        /* Hand the descriptor back to the MAC. The chain is circular, so the queue only stops
         * if it ran out of owned descriptors while this core was busy; EOQ says it did, and the
         * head pointer has to be written again to restart it. */
        descriptor->offset_length = FRAME_MAX;
        descriptor->flags_length = CPPI_OWNER;
        if ((flags & CPPI_EOQ) != 0u) {
            RXHDP(core) = dma_address(&rx_ring[core][(index + 1u) % RX_SLOTS]);
        }
        index = (index + 1u) % RX_SLOTS;
    }
}

int main(void)
{
    unsigned core = DNUM;

    if (core == 0u) {
        mac_ready = 0u;
        mac_setup();
        mac_ready = 1u;
    } else {
        while (mac_ready == 0u) {
        }
    }
    ring_setup(core);
    announce(core);
    echo_loop(core);
    return 0;
}
