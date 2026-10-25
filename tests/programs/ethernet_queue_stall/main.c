#include <stdio.h>
/* A receive queue that has run to the end of its chain stays stopped (SPRU975E section 4.3.2):
 * the DMA marks the last descriptor EOQ, clears its own head pointer, and does not notice
 * descriptors linked onto the tail afterwards. Only a write of RXnHDP restarts it, and the
 * frames that arrived while it was stopped are still in the receive FIFO, so nothing is lost.
 *
 * This is the contract TI's NDK stack depends on and the one a driver most easily gets wrong:
 * refilling a descriptor without linking it and without restarting the queue silently stops
 * reception after exactly one pass over the chain, which is what happened to the NIMU driver
 * in firmware/ndk_tcp before its refill learned to link onto the tail.
 *
 * Three frames are injected. Two descriptors take the first two; the second reports EOQ. The
 * program then refills and relinks both descriptors but leaves RXnHDP alone and spins: the
 * third frame must not arrive. Writing RXnHDP then delivers it, whole. The checksum folds the
 * length and status of every frame, the spin's outcome and the final frame's bytes.
 */

#define EMAC_BASE      0x02C80000u
#define TXCONTROL      (EMAC_BASE + 0x004u)
#define RXCONTROL      (EMAC_BASE + 0x014u)
#define RXMBPENABLE    (EMAC_BASE + 0x100u)
#define RXUNICASTSET   (EMAC_BASE + 0x104u)
#define RXMAXLEN       (EMAC_BASE + 0x10Cu)
#define RXBUFFEROFFSET (EMAC_BASE + 0x110u)
#define RX0FREEBUFFER  (EMAC_BASE + 0x140u)
#define MACCONTROL     (EMAC_BASE + 0x160u)
#define RX0HDP         (EMAC_BASE + 0x620u)
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

#define DESCRIPTORS 2
#define BUFFER_SIZE 256
#define SPIN 4000u

typedef struct Desc {
    volatile unsigned int next;
    volatile unsigned int buffer;
    volatile unsigned int offset_length;
    volatile unsigned int flags_length;
} Desc;

static Desc descriptors[DESCRIPTORS];
static unsigned char buffers[DESCRIPTORS][BUFFER_SIZE];

static void poke(unsigned int address, unsigned int value)
{
    *(volatile unsigned int *)address = value;
}

static unsigned int peek(unsigned int address)
{
    return *(volatile unsigned int *)address;
}

static unsigned int fold(unsigned int accumulator, unsigned int value)
{
    accumulator = (accumulator << 5) | (accumulator >> 27);
    return accumulator ^ value;
}

/* Hand one descriptor back to the EMAC, linking it onto the tail of the chain. */
static void refill(unsigned int index)
{
    descriptors[index].next = 0;
    descriptors[index].buffer = (unsigned int)buffers[index];
    descriptors[index].offset_length = BUFFER_SIZE;
    descriptors[index].flags_length = CPPI_OWNER;
    descriptors[(index + DESCRIPTORS - 1u) % DESCRIPTORS].next =
        (unsigned int)&descriptors[index];
}

/* Wait for the descriptor to complete, up to a bounded spin. Returns its flags, or zero. */
static unsigned int await_frame(unsigned int index)
{
    unsigned int spins;

    for (spins = 0; spins < SPIN; spins++) {
        unsigned int flags = descriptors[index].flags_length;

        if ((flags & CPPI_OWNER) == 0u) {
            return flags;
        }
    }
    return 0;
}

int main(void)
{
    unsigned int checksum = 0;
    unsigned int index;
    unsigned int flags;
    unsigned int held;

    poke(EWCTL, 1);
    poke(MACINDEX, 0);
    poke(MACADDRHI, 0x11000002u);
    poke(MACADDRLO, 0x3322u | MACADDR_VALID | MACADDR_MATCH);
    poke(RXUNICASTSET, 1);
    poke(RXMAXLEN, 1518);
    poke(RXBUFFEROFFSET, 0);
    poke(RXMBPENABLE, RXMBP_BROADCAST | 1u);
    poke(MACCONTROL, MACCONTROL_GMIIEN);

    for (index = 0; index < DESCRIPTORS; index++) {
        descriptors[index].next = 0;
        descriptors[index].buffer = (unsigned int)buffers[index];
        descriptors[index].offset_length = BUFFER_SIZE;
        descriptors[index].flags_length = CPPI_OWNER;
    }
    descriptors[0].next = (unsigned int)&descriptors[1];
    poke(RX0FREEBUFFER, DESCRIPTORS);
    poke(RXCONTROL, 1);
    poke(TXCONTROL, 1);
    poke(RX0HDP, (unsigned int)&descriptors[0]);

    /* The first two frames fill the chain, and the second one ends it. */
    for (index = 0; index < DESCRIPTORS; index++) {
        flags = await_frame(index);
        if (flags == 0u) {
            return -1;
        }
        checksum = fold(checksum, flags & 0xFFFFu);
        checksum = fold(checksum, (flags >> 28) & 0xFu);
        checksum = fold(checksum, buffers[index][14]);
        poke(RX0CP, (unsigned int)&descriptors[index]);
    }

    /* Refill and relink both, but leave the head pointer alone: the queue stays stopped and
     * the third frame waits in the FIFO. */
    for (index = 0; index < DESCRIPTORS; index++) {
        refill(index);
    }
    held = 0;
    for (index = 0; index < DESCRIPTORS; index++) {
        if ((descriptors[index].flags_length & CPPI_OWNER) != 0u) {
            held++;
        }
    }
    checksum = fold(checksum, held);
    checksum = fold(checksum, await_frame(0) == 0u ? 1u : 0u);
    checksum = fold(checksum, peek(RX0HDP));

    /* Restarting the queue delivers the frame that was waiting, whole. */
    poke(RX0HDP, (unsigned int)&descriptors[0]);
    flags = await_frame(0);
    if (flags == 0u) {
        return -2;
    }
    checksum = fold(checksum, flags & 0xFFFFu);
    checksum = fold(checksum, (flags >> 28) & 0xFu);
    for (index = 0; index < 16u; index++) {
        checksum = fold(checksum, buffers[0][index]);
    }
    poke(RX0CP, (unsigned int)&descriptors[0]);
    printf("queue stall checksum %u\n", checksum);
    return (int)checksum;
}
