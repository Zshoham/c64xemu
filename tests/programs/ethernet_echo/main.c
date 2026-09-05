#include <stdio.h>
/* Ethernet echo firmware: programs the CPPI descriptor rings of the C6455 EMAC, waits for
   frames to arrive, swaps their addresses and sends them back, acknowledging both queues the
   way the hardware protocol requires. */

#define EMAC_BASE      0x02C80000u
#define TXCONTROL      (EMAC_BASE + 0x004u)
#define RXCONTROL      (EMAC_BASE + 0x014u)
#define TXINTSTATRAW   (EMAC_BASE + 0x080u)
#define TXINTMASKSET   (EMAC_BASE + 0x088u)
#define RXINTSTATRAW   (EMAC_BASE + 0x0A0u)
#define RXINTMASKSET   (EMAC_BASE + 0x0A8u)
#define RXMBPENABLE    (EMAC_BASE + 0x100u)
#define RXBUFFEROFFSET (EMAC_BASE + 0x110u)
#define RX0FREEBUFFER  (EMAC_BASE + 0x140u)
#define TX0HDP         (EMAC_BASE + 0x600u)
#define RX0HDP         (EMAC_BASE + 0x620u)
#define TX0CP          (EMAC_BASE + 0x640u)
#define RX0CP          (EMAC_BASE + 0x660u)
#define EWCTL          0x02C81004u
#define RXUNICASTSET   (EMAC_BASE + 0x104u)
#define RXMAXLEN       (EMAC_BASE + 0x10Cu)
#define MACCONTROL     (EMAC_BASE + 0x160u)
#define MACHASH1       (EMAC_BASE + 0x1D8u)
#define MACHASH2       (EMAC_BASE + 0x1DCu)
#define RXGOODFRAMES   (EMAC_BASE + 0x200u)
#define RXBCASTFRAMES  (EMAC_BASE + 0x204u)
#define RXFILTERED     (EMAC_BASE + 0x228u)
#define TXGOODFRAMES   (EMAC_BASE + 0x234u)
#define MACADDRLO      (EMAC_BASE + 0x500u)
#define MACADDRHI      (EMAC_BASE + 0x504u)
#define MACINDEX       (EMAC_BASE + 0x508u)

#define MACADDR_VALID     (1u << 20)
#define MACADDR_MATCH     (1u << 19)
#define MACCONTROL_GMIIEN (1u << 5)

#define RXMBP_BROADCAST_ENABLE (1u << 13)
#define RXMBP_UNICAST_ENABLE   (1u << 0)
/* Receive copy short frames: frames below the 64-byte wire minimum are only transferred to
   memory when this bit is set (SPRU975E Table 49). */
#define RXMBP_CSFEN            (1u << 23)

#define CPPI_SOP   (1u << 31)
#define CPPI_EOP   (1u << 30)
#define CPPI_OWNER (1u << 29)
#define CPPI_EOQ   (1u << 28)

#define FRAMES 3
#define BUFFER_SIZE 128

struct descriptor {
    volatile unsigned int next;
    volatile unsigned int buffer;
    volatile unsigned int offset_length;
    volatile unsigned int flags_length;
};

static struct descriptor rx_descriptors[FRAMES];
static struct descriptor tx_descriptors[FRAMES];
static unsigned char rx_buffers[FRAMES][BUFFER_SIZE];
static unsigned char tx_buffers[FRAMES][BUFFER_SIZE];

static void poke(unsigned int address, unsigned int value)
{
    *(volatile unsigned int *)address = value;
}

static unsigned int peek(unsigned int address)
{
    return *(volatile unsigned int *)address;
}

/* Program the station address into slot zero of the address RAM and let channel zero receive
   unicast frames that match it (SPRU975E sections 5.42 to 5.44). */
static void program_address(void)
{
    poke(MACINDEX, 0);
    /* Station address 02:00:00:11:22:33: bytes 2 to 5 live in MACADDRHI. */
    poke(MACADDRHI, (0x00u << 24) | (0x11u << 16) | (0x22u << 8) | 0x33u);
    poke(MACADDRLO, (0x02u << 8) | 0x00u | MACADDR_VALID | MACADDR_MATCH | (0u << 16));
    poke(RXUNICASTSET, 1);
    poke(RXMAXLEN, 1518);
    poke(MACCONTROL, MACCONTROL_GMIIEN);
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

static int echo_frame(int index)
{
    unsigned int length;
    unsigned int i;
    unsigned char *in = &rx_buffers[index][0];
    unsigned char *out = &tx_buffers[index][0];
    int checksum = 0;

    /* Wait for the receive queue to hand the descriptor back. */
    while ((rx_descriptors[index].flags_length & CPPI_OWNER) != 0) {
    }
    length = rx_descriptors[index].flags_length & 0xFFFFu;
    if (length < 14u || length > BUFFER_SIZE) {
        return -1;
    }
    /* Acknowledge the receive queue with the descriptor the hardware completed. */
    poke(RX0CP, (unsigned int)&rx_descriptors[index]);

    /* Swap the addresses and copy the payload through. */
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
    return checksum;
}

int main(void)
{
    int total = 0;
    int index;

    poke(EWCTL, 1);
    program_address();
    poke(RXMBPENABLE,
         RXMBP_BROADCAST_ENABLE | RXMBP_UNICAST_ENABLE | RXMBP_CSFEN);
    poke(RXBUFFEROFFSET, 0);
    poke(TXINTMASKSET, 1);
    poke(RXINTMASKSET, 1);
    poke(RXCONTROL, 1);
    poke(TXCONTROL, 1);

    for (index = 0; index < FRAMES; index++) {
        int result;
        post_receive(index);
        result = echo_frame(index);
        if (result < 0) {
            printf("ethernet_echo=error\n");
            return 0;
        }
        total = (total ^ result) + index;
    }
    printf("ethernet_echo=%d rx=%u tx=%u\n", total, peek(RXINTSTATRAW), peek(TXINTSTATRAW));
    return 0;
}
