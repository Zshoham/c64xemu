#include <stdio.h>
/* The one coherence guarantee the C64x+ megamodule does make: a DMA master accessing L2 SRAM is
 * kept coherent with L1D by the L2 controller (SPRU862B chapter 2). A DMA write snoop-writes
 * into a line L1D is holding, so the CPU sees the new data without invalidating; a DMA read of a
 * line L1D holds dirty is served from L1D, so a buffer the CPU has only written into its cache
 * still goes out on the wire correctly. External memory gets neither guarantee, which is why
 * firmware there must writeback and invalidate by hand.
 *
 * The EMAC's own CPPI DMA is the master here, and both directions are checked with L1D enabled
 * as a 32K cache and the buffers deliberately resident in it:
 *
 *   receive  - the buffer is filled with a sentinel and read back, so its lines sit in L1D
 *              clean. The DMA then writes a frame into it. Reading the buffer again without any
 *              cache operation must show the frame, not the sentinel.
 *   transmit - the buffer is read (allocating its lines), then written, which leaves the lines
 *              dirty in L1D and stale in memory. The frame the DMA sends must carry the written
 *              bytes rather than what memory holds.
 *
 * The checksum folds the bytes the CPU read back after the receive, so a model that lets the CPU
 * keep its stale line fails here; the transmitted frame the harness compares covers the read
 * direction, and it is the sentinel that would go out if the dirty line were ignored.
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
#define TX0HDP         (EMAC_BASE + 0x600u)
#define RX0HDP         (EMAC_BASE + 0x620u)
#define TX0CP          (EMAC_BASE + 0x640u)
#define RX0CP          (EMAC_BASE + 0x660u)
#define MACADDRLO      (EMAC_BASE + 0x500u)
#define MACADDRHI      (EMAC_BASE + 0x504u)
#define MACINDEX       (EMAC_BASE + 0x508u)
#define EWCTL          0x02C81004u

/* C64x+ cache control (SPRU871K chapter 4): L1DMODE 100b is the whole 32K as cache. */
#define L1DCFG         0x01840040u
#define L1D_32K_CACHE  4u

#define MACADDR_VALID     (1u << 20)
#define MACADDR_MATCH     (1u << 19)
#define MACCONTROL_GMIIEN (1u << 5)
#define RXMBP_BROADCAST   (1u << 13)

#define CPPI_SOP   (1u << 31)
#define CPPI_EOP   (1u << 30)
#define CPPI_OWNER (1u << 29)

#define BUFFER_SIZE 256
#define FRAME_LENGTH 64
#define SENTINEL 0xA5u

struct descriptor {
    volatile unsigned int next;
    volatile unsigned int buffer;
    volatile unsigned int offset_length;
    volatile unsigned int flags_length;
};

static struct descriptor rx_descriptor;
static struct descriptor tx_descriptor;
/* Volatile so that the compiler cannot sink a buffer store past the descriptor write that
 * arms the DMA - at -O3 it will, and then the frame goes out with a byte missing on hardware
 * too. Volatile does not make these uncached; that is the point of the test. */
static volatile unsigned char rx_buffer[BUFFER_SIZE];
static volatile unsigned char tx_buffer[BUFFER_SIZE];

static void poke(unsigned int address, unsigned int value)
{
    *(volatile unsigned int *)address = value;
}

static unsigned int fold(unsigned int accumulator, unsigned int value)
{
    accumulator = (accumulator << 5) | (accumulator >> 27);
    return accumulator ^ value;
}

int main(void)
{
    unsigned int checksum = 0;
    unsigned int index;
    unsigned int resident = 0;
    unsigned int length;

    poke(L1DCFG, L1D_32K_CACHE);
    poke(EWCTL, 1);
    poke(MACINDEX, 0);
    poke(MACADDRHI, 0x11000002u);
    poke(MACADDRLO, 0x3322u | MACADDR_VALID | MACADDR_MATCH);
    poke(RXUNICASTSET, 1);
    poke(RXMAXLEN, 1518);
    poke(RXBUFFEROFFSET, 0);
    poke(RXMBPENABLE, RXMBP_BROADCAST | 1u);
    poke(MACCONTROL, MACCONTROL_GMIIEN);
    poke(RXCONTROL, 1);
    poke(TXCONTROL, 1);

    /* Fill both buffers with the sentinel, then read them: L1D does not allocate on a write,
     * so it is the reads that leave the lines resident. */
    for (index = 0; index < BUFFER_SIZE; index++) {
        rx_buffer[index] = (unsigned char)SENTINEL;
        tx_buffer[index] = (unsigned char)SENTINEL;
    }
    for (index = 0; index < BUFFER_SIZE; index++) {
        resident += rx_buffer[index] + tx_buffer[index];
    }
    checksum = fold(checksum, resident);

    /* Receive: the DMA writes over lines L1D is holding, and the CPU must see the frame. */
    rx_descriptor.next = 0;
    rx_descriptor.buffer = (unsigned int)rx_buffer;
    rx_descriptor.offset_length = BUFFER_SIZE;
    rx_descriptor.flags_length = CPPI_OWNER;
    poke(RX0FREEBUFFER, 1);
    poke(RX0HDP, (unsigned int)&rx_descriptor);
    while ((rx_descriptor.flags_length & CPPI_OWNER) != 0u) {
    }
    length = rx_descriptor.flags_length & 0xFFFFu;
    poke(RX0CP, (unsigned int)&rx_descriptor);
    checksum = fold(checksum, length);
    for (index = 0; index < 16u; index++) {
        checksum = fold(checksum, rx_buffer[index]);
    }

    /* Transmit: the buffer's lines are already resident, so writing them leaves memory stale
     * and the DMA has to take the data out of L1D. */
    for (index = 0; index < 6u; index++) {
        tx_buffer[index] = rx_buffer[6u + index];
        tx_buffer[6u + index] = rx_buffer[index];
    }
    for (index = 12u; index < FRAME_LENGTH; index++) {
        tx_buffer[index] = (unsigned char)(0x40u + index);
    }
    tx_descriptor.next = 0;
    tx_descriptor.buffer = (unsigned int)tx_buffer;
    tx_descriptor.offset_length = FRAME_LENGTH;
    tx_descriptor.flags_length = CPPI_SOP | CPPI_EOP | CPPI_OWNER | FRAME_LENGTH;
    poke(TX0HDP, (unsigned int)&tx_descriptor);
    while ((tx_descriptor.flags_length & CPPI_OWNER) != 0u) {
    }
    poke(TX0CP, (unsigned int)&tx_descriptor);
    checksum = fold(checksum, tx_descriptor.flags_length & 0xFFFFu);

    printf("dma coherence checksum %u\n", checksum);
    return 0;
}
