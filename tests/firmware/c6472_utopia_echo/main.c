/* An ATM cell echo on the C6472's UTOPIA slave, through its PDMA, driven by an external master.
 *
 * The C6455 firmware (`utopia_echo`) copies each cell from the receive queue to the transmit
 * queue with the CPU. The C6472 maps no CPU window onto either queue (SPRS612G Table 2-2 reserves
 * the C6455's), so here both directions go through the UTOPIA's PDMA and memory: a receive
 * channel moves each cell packet from the receive queue into a buffer in core 0's L2, and a
 * transmit channel moves the same packet from that buffer into the transmit queue (SPRUEG2E
 * sections 4.4 and 4.5). Both queues hold the cell packet - dummy bytes in front of the header
 * up to whole words (Table 6) - so the master reads back the cell it sent.
 *
 * Both channels run in block mode, one cell per block, and raise their interrupt when the block
 * completes (Table 5): PRINT is system event 30 and PXINT event 31 (SPRS612G Table 7-10), and
 * SINT = 0 sends both to core 0. The CPU sees them latched in its event flags and clears them
 * itself, so no interrupt handler is needed. The transmit group is set up as Example 1 sets it:
 * PE = 1, "use the current PDMA channel on transfer events".
 *
 * After ECHOES cells the master sends one more, which ends the run: it is received but not
 * echoed, so the firmware never exits with a cell the master has not yet taken.
 */

#define REG(address) (*(volatile unsigned int *)(address))

/* UTOPIA (SPRUEG2E section 15.1). U16M (bit 13) is left clear: an 8-bit port, 53-byte cells. */
#define UCR 0x02B40000u
#define UCR_SLID_SHIFT 24
#define UCR_UXEN (1u << 16)
#define UCR_MPHY (1u << 14)
#define UCR_UREN (1u << 0)

/* PDMA (SPRUEG2E Table 27 and section 15.2). */
#define PDMA 0x02BC0000u
#define GCSR (PDMA + 0x04u)
#define PCR0 (PDMA + 0x60u)
#define PCR1 (PDMA + 0x64u)
#define TXC0 (PDMA + 0x100u)
#define RXC0 (PDMA + 0x180u)
#define GCSR_STRT (1u << 31)

/* PCR (Table 32): PE, SYNC, DIR, PBLEN and SBLEN as 14 - 1 words, PMOD constant. */
#define PCR_PE_CURRENT (1u << 22)
#define PCR_PE_EVENT_ID (2u << 22)
#define PCR_SYNC (1u << 20)
#define PCR_DIR_PERIPHERAL_TO_SWITCH (1u << 16)
#define PCR_CELL_BURSTS ((13u << 12) | (1u << 11) | (13u << 4))

/* TCR (Table 39): STRT, IE, SINT 0, 32-bit elements on both sides. */
#define TCR_START ((1u << 31) | (1u << 27) | (2u << 22) | (2u << 20))

/* The queues' ports on the PDMA's peripheral bus (Table 18). */
#define URQ 0u
#define UXQ 4u

/* Core 0's event flags (SPRU871K sections 7.5.1.1 and 7.5.1.3). */
#define EVTFLAG0 0x01800000u
#define EVTCLR0 0x01800040u
#define PRINT (1u << 30)
#define PXINT (1u << 31)

/* A 53-byte cell travels as a 56-byte packet (SPRUEG2E section 2). */
#define PACKET_BYTES 56u
#define PACKET_WORDS (PACKET_BYTES / 4u)

#define SLAVE 9u
#define ECHOES 3
#define SPIN_LIMIT 20000000u

/* One buffer per cell, so every echo comes from memory the receive channel wrote. */
static unsigned int cells[ECHOES + 1][PACKET_WORDS];

/* The address the PDMA, a switch master, uses for core 0's local L2: its global alias. */
static unsigned int global(const void *local)
{
    return 0x10000000u | (unsigned int)local;
}

static void start(unsigned int proxy, unsigned int switch_address, unsigned int port)
{
    REG(proxy) = switch_address;
    REG(proxy + 4u) = port;
    REG(proxy + 8u) = PACKET_BYTES;
    REG(proxy + 12u) = TCR_START;
}

static int completed(unsigned int event)
{
    unsigned int spins;
    for (spins = 0; (REG(EVTFLAG0) & event) == 0; spins++) {
        if (spins == SPIN_LIMIT) {
            return 0;
        }
    }
    REG(EVTCLR0) = event;
    return 1;
}

int main(void)
{
    int echoed;

    REG(PCR0) = PCR_PE_CURRENT | PCR_SYNC | PCR_CELL_BURSTS;
    REG(PCR1) = PCR_PE_EVENT_ID | PCR_SYNC | PCR_DIR_PERIPHERAL_TO_SWITCH | PCR_CELL_BURSTS;
    REG(GCSR) = GCSR_STRT;
    REG(UCR) = (SLAVE << UCR_SLID_SHIFT) | UCR_MPHY | UCR_UXEN | UCR_UREN;
    for (echoed = 0; echoed < ECHOES; echoed++) {
        start(RXC0, global(cells[echoed]), URQ);
        if (!completed(PRINT)) {
            return -1;
        }
        start(TXC0, global(cells[echoed]), UXQ);
        if (!completed(PXINT)) {
            return -2;
        }
    }
    start(RXC0, global(cells[ECHOES]), URQ);
    if (!completed(PRINT)) {
        return -3;
    }
    return echoed;
}
