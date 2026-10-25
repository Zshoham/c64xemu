#include <stdio.h>
/* The same EMAC traffic as ethernet_dma_coherence, but with the buffers in DDR2 instead of L2
 * SRAM - the memory the snoop protocol does not cover. SPRU862B section 2.4 is explicit that
 * hardware maintains coherence only between L1D and L2 SRAM, and Table 2-2 lists the operations
 * a program has to issue for external memory itself. Both are exercised here from C, through the
 * range registers a driver actually uses:
 *
 *   receive  - the buffer is primed and read, so its lines are resident. The EMAC writes a frame
 *              into it. Reading the buffer again still gives the sentinel, because nothing
 *              snooped; a range invalidate through L1DIBAR and L1DIWC - no writeback, or the
 *              stale line would be pushed back over the frame - then shows the frame.
 *   transmit - the buffer's lines are resident, so writing them leaves memory stale. Arming the
 *              descriptor straight away would send what memory holds, so the program issues a
 *              range writeback through L1DWBAR and L1DWWC first, and the frame carries the new
 *              bytes.
 *
 * MAR224 is what makes the DDR2 window cacheable at all (SPRU871K chapter 4); with its PC bit
 * clear there would be nothing to maintain and nothing to test.
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

#define L1DCFG         0x01840040u
#define L1D_32K_CACHE  4u
#define L1DWBAR        0x01844040u
#define L1DWWC         0x01844044u
#define L1DIBAR        0x01844048u
#define L1DIWC         0x0184404Cu
#define MAR224         (0x01848000u + 224u * 4u)

#define MACADDR_VALID     (1u << 20)
#define MACADDR_MATCH     (1u << 19)
#define MACCONTROL_GMIIEN (1u << 5)
#define RXMBP_BROADCAST   (1u << 13)

#define CPPI_SOP   (1u << 31)
#define CPPI_EOP   (1u << 30)
#define CPPI_OWNER (1u << 29)

/* Descriptors stay in L2 SRAM, where the snoop keeps them coherent; only the frame buffers are
 * external. That is how a driver is written: the descriptor ring is local, the payload is not. */
#define DDR2_BASE   0xE0000000u
#define RX_BUFFER   (DDR2_BASE + 0x2000u)
#define TX_BUFFER   (DDR2_BASE + 0x3000u)
#define BUFFER_SIZE 256u
#define FRAME_LENGTH 64u
#define SENTINEL 0xA5u

struct descriptor {
    volatile unsigned int next;
    volatile unsigned int buffer;
    volatile unsigned int offset_length;
    volatile unsigned int flags_length;
};

static struct descriptor rx_descriptor;
static struct descriptor tx_descriptor;

static void poke(unsigned int address, unsigned int value)
{
    *(volatile unsigned int *)address = value;
}

static unsigned char load(unsigned int address)
{
    return *(volatile unsigned char *)address;
}

static void store(unsigned int address, unsigned char value)
{
    *(volatile unsigned char *)address = value;
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

    poke(MAR224, 1);
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

    for (index = 0; index < BUFFER_SIZE; index++) {
        store(RX_BUFFER + index, (unsigned char)SENTINEL);
        store(TX_BUFFER + index, (unsigned char)SENTINEL);
    }
    for (index = 0; index < BUFFER_SIZE; index++) {
        resident += load(RX_BUFFER + index) + load(TX_BUFFER + index);
    }
    checksum = fold(checksum, resident);

    rx_descriptor.next = 0;
    rx_descriptor.buffer = RX_BUFFER;
    rx_descriptor.offset_length = BUFFER_SIZE;
    rx_descriptor.flags_length = CPPI_OWNER;
    poke(RX0FREEBUFFER, 1);
    poke(RX0HDP, (unsigned int)&rx_descriptor);
    while ((rx_descriptor.flags_length & CPPI_OWNER) != 0u) {
    }
    length = rx_descriptor.flags_length & 0xFFFFu;
    poke(RX0CP, (unsigned int)&rx_descriptor);
    checksum = fold(checksum, length);

    /* Nothing snooped, so these are still the sentinel. */
    for (index = 0; index < 8u; index++) {
        checksum = fold(checksum, load(RX_BUFFER + index));
    }

    /* Invalidate the buffer's range without writeback, and only now is the frame readable. */
    poke(L1DIBAR, RX_BUFFER);
    poke(L1DIWC, BUFFER_SIZE / 4u);
    for (index = 0; index < 8u; index++) {
        checksum = fold(checksum, load(RX_BUFFER + index));
    }

    /* The transmit buffer's lines are resident, so these writes leave memory stale. */
    for (index = 0; index < 6u; index++) {
        store(TX_BUFFER + index, load(RX_BUFFER + 6u + index));
        store(TX_BUFFER + 6u + index, load(RX_BUFFER + index));
    }
    for (index = 12u; index < FRAME_LENGTH; index++) {
        store(TX_BUFFER + index, (unsigned char)(0x40u + index));
    }
    poke(L1DWBAR, TX_BUFFER);
    poke(L1DWWC, BUFFER_SIZE / 4u);

    tx_descriptor.next = 0;
    tx_descriptor.buffer = TX_BUFFER;
    tx_descriptor.offset_length = FRAME_LENGTH;
    tx_descriptor.flags_length = CPPI_SOP | CPPI_EOP | CPPI_OWNER | FRAME_LENGTH;
    poke(TX0HDP, (unsigned int)&tx_descriptor);
    while ((tx_descriptor.flags_length & CPPI_OWNER) != 0u) {
    }
    poke(TX0CP, (unsigned int)&tx_descriptor);
    checksum = fold(checksum, tx_descriptor.flags_length & 0xFFFFu);

    printf("external coherence checksum %u\n", checksum);
    return 0;
}
