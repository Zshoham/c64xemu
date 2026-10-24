/* An ATM cell echo on the C6455's UTOPIA slave, driven by an external master on the host.
 *
 * Every cell the master sends is read out of the receive queue and written into the transmit
 * queue word for word. Both queues hold the same cell packet - dummy bytes in front of the header
 * up to whole words (SPRUE48 Figures 9-14) - so the copy is the identity on the wire, and the
 * master reads back the cell it sent. UREVT is EDMA3 event 32 (SPRS276M Table 7-9); with no
 * channel enabled the event register still latches it, which is how the CPU sees a cell waiting.
 * UREVT stays asserted until the first word of the cell is read, so the latch is cleared only
 * once the cell has been copied.
 *
 * After ECHOES cells the master sends one more, which ends the run: it is not echoed, so the
 * firmware never exits with a cell the master has not yet taken.
 */

#define REG(address) (*(volatile unsigned int *)(address))

/* UTOPIA (SPRUE48 section 12, SPRS276M Table 2-2). */
#define UCR 0x02B40000u
#define UCR_RESET 0x20004000u
#define UCR_SLID_SHIFT 24
#define UCR_UXEN (1u << 16)
#define UCR_UREN (1u << 0)
#define RECEIVE_QUEUE 0x3C000000u
#define TRANSMIT_QUEUE 0x3C000400u
#define PACKET_WORDS 14u

/* EDMA3 event registers for channels 32-63 (SPRU966C Table 4-1). */
#define ERH 0x02A01004u
#define ECRH 0x02A0100Cu
#define UREVT (1u << 0)

#define SLAVE 9u
#define ECHOES 3
#define SPIN_LIMIT 20000000u

static int cell_waiting(void)
{
    unsigned int spins;
    for (spins = 0; (REG(ERH) & UREVT) == 0; spins++) {
        if (spins == SPIN_LIMIT) {
            return 0;
        }
    }
    return 1;
}

int main(void)
{
    unsigned int word;
    int echoed;

    REG(UCR) = UCR_RESET | (SLAVE << UCR_SLID_SHIFT) | UCR_UXEN | UCR_UREN;
    for (echoed = 0; echoed < ECHOES; echoed++) {
        if (!cell_waiting()) {
            return -1;
        }
        for (word = 0; word < PACKET_WORDS; word++) {
            REG(TRANSMIT_QUEUE) = REG(RECEIVE_QUEUE);
        }
        REG(ECRH) = UREVT;
    }
    if (!cell_waiting()) {
        return -2;
    }
    return echoed;
}
