/* An I2C loopback stream drained by the EDMA3 without the CPU touching the data.
 *
 * The I2C module in digital loopback mode copies each byte written to ICDXR into ICDRR and raises
 * its receive event, which is EDMA3 synchronisation event 44 (SPRS276M Table 7-9). A channel
 * whose source is ICDRR itself - a peripheral register, not memory - moves one byte per event
 * into a buffer, so the bytes only reach memory if the receive event reaches the channel
 * controller and the transfer controller reads the register through the peripheral's slave port.
 *
 * The CPU writes the bytes and reads them back packed; it never copies them.
 */

#define REG(address) (*(volatile unsigned int *)(address))

/* I2C (SPRU974A). */
#define I2C_ICSTR 0x02B04008u
#define I2C_ICCNT 0x02B04014u
#define I2C_ICDRR 0x02B04018u
#define I2C_ICDXR 0x02B04020u
#define I2C_ICMDR 0x02B04024u
#define I2C_ICPSC 0x02B04030u

#define ICSTR_ICXRDY (1u << 4)
/* IRS out of reset, master transmitter in digital loopback, START and STOP (Table 14). */
#define ICMDR_START ((1u << 13) | (1u << 11) | (1u << 10) | (1u << 9) | (1u << 6) | (1u << 5))

/* EDMA3 channel controller (SPRU966C). Channel 44's event enable is in the high register. */
#define EDMA_DCHMAP44 0x02A001B0u
#define EDMA_EESRH 0x02A01034u
#define EDMA_IESR 0x02A01060u
#define EDMA_IPR 0x02A01068u
#define EDMA_ICR 0x02A01070u
#define EDMA_PARAM9 0x02A04120u /* PaRAM base + 32 * 9 */

#define TCC 11
#define BYTES 4

static volatile unsigned char received[BYTES];

int main(void)
{
    static const unsigned int sent[BYTES] = {0x11u, 0x22u, 0x33u, 0x44u};
    unsigned int index;
    unsigned int spins;
    unsigned int packed = 0;

    for (index = 0; index < BYTES; index++) {
        received[index] = 0u;
    }

    /* Channel 44 takes its parameters from PaRAM set 9: one byte per event, four in all, the
       source fixed on ICDRR and the destination walking the buffer. */
    REG(EDMA_DCHMAP44) = 9u << 5;
    REG(EDMA_PARAM9 + 0u) = (1u << 20) | ((unsigned int)TCC << 12);
    REG(EDMA_PARAM9 + 4u) = I2C_ICDRR;
    REG(EDMA_PARAM9 + 8u) = 1u | ((unsigned int)BYTES << 16);
    REG(EDMA_PARAM9 + 12u) = (unsigned int)&received[0];
    REG(EDMA_PARAM9 + 16u) = 1u << 16;
    REG(EDMA_PARAM9 + 20u) = 0xFFFFu;
    REG(EDMA_PARAM9 + 24u) = 0u;
    REG(EDMA_PARAM9 + 28u) = 1u;
    REG(EDMA_IESR) = 1u << TCC;
    REG(EDMA_EESRH) = 1u << (44u - 32u);

    /* The fastest documented bit clock: the byte leaves ICDXR after (prescaler + 1) * 8 DSP
       cycles (SPRU974A section 2.7). */
    REG(I2C_ICPSC) = 0u;
    REG(I2C_ICCNT) = BYTES;
    REG(I2C_ICMDR) = ICMDR_START;

    for (index = 0; index < BYTES; index++) {
        for (spins = 0; spins < 20000u; spins++) {
            if ((REG(I2C_ICSTR) & ICSTR_ICXRDY) != 0u) {
                break;
            }
        }
        if ((REG(I2C_ICSTR) & ICSTR_ICXRDY) == 0u) {
            return -1;
        }
        REG(I2C_ICDXR) = sent[index];
    }

    /* The transfer completes when the fourth event has been serviced. */
    for (spins = 0; spins < 20000u; spins++) {
        if ((REG(EDMA_IPR) & (1u << TCC)) != 0u) {
            break;
        }
    }
    if ((REG(EDMA_IPR) & (1u << TCC)) == 0u) {
        return -2;
    }
    REG(EDMA_ICR) = 1u << TCC;

    for (index = 0; index < BYTES; index++) {
        packed = (packed << 8) | received[index];
    }
    return (int)packed;
}
