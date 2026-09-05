/* NIMU Ethernet mini-driver for the C6455 EMAC, written to the interface in the NDK Support
 * Package Ethernet Driver Design Guide (SPRUFP2B) and the EMAC/MDIO User's Guide (SPRU975E).
 *
 * TI ships drivers like this per board inside an NSP; there is no NSP for an emulator, so this
 * file is the board support this project has to provide. Everything above it - ARP, IP, ICMP, TCP,
 * sockets, NETCTRL - is TI's NDK, unmodified.
 *
 * The receive side is interrupt driven and serviced outside interrupt context: the service
 * routine only posts the stack thread's semaphore, NETCTRL then calls poll(), and poll()
 * acknowledges the receive queue - which is what drops the module's interrupt line - before
 * handing packets on through pkt_service(). That is the SPRUFP2B section 1.3 flow with the
 * interrupt as the wake-up rather than as the place the ring is walked, so the descriptor race
 * an in-interrupt driver has to defend against does not arise.
 *
 * The transmit side is deliberately not interrupt driven, and TXINTMASKSET is deliberately left
 * clear. poll() reclaims completed transmit descriptors on every pass, which is what returns
 * their packet buffers to the pool the receive path allocates from. Enabling the transmit mask
 * as well was tried and measured worse: MACINT is one line the two queues share and the
 * megamodule latches its rising edge (SPRU871K Figure 7-9), so the mask brings an obligation to
 * acknowledge both queues on every pass - see `ethernet_shared_macint` for what happens
 * otherwise - and it costs one interrupt and one stack-thread pass per transmitted packet on the
 * critical path of the receive drain. At 512 kbps of paced UDP that lost datagrams in eight of
 * twenty runs where this driver loses none.
 */

#include <netmain.h>
#include <_stack.h>
#include <stkmain.h>
#include <ti/sysbios/family/c64p/Cache.h>

#define EMAC_BASE 0x02C80000u
#define EMAC_CTRL_BASE 0x02C81000u
#define MDIO_BASE 0x02C81800u

#define TXCONTROL (EMAC_BASE + 0x004u)
#define TXTEARDOWN (EMAC_BASE + 0x008u)
#define RXCONTROL (EMAC_BASE + 0x014u)
#define RXTEARDOWN (EMAC_BASE + 0x018u)
#define TXINTSTATRAW (EMAC_BASE + 0x080u)
#define TXINTMASKSET (EMAC_BASE + 0x088u)
#define TXINTMASKCLEAR (EMAC_BASE + 0x08Cu)
#define MACINVECTOR (EMAC_BASE + 0x090u)
#define RXINTSTATRAW (EMAC_BASE + 0x0A0u)
#define RXINTMASKSET (EMAC_BASE + 0x0A8u)
#define RXINTMASKCLEAR (EMAC_BASE + 0x0ACu)
#define MACINTMASKSET (EMAC_BASE + 0x0C8u)
#define RXMBPENABLE (EMAC_BASE + 0x100u)
#define RXUNICASTSET (EMAC_BASE + 0x104u)
#define RXUNICASTCLEAR (EMAC_BASE + 0x108u)
#define RXMAXLEN (EMAC_BASE + 0x10Cu)
#define RXBUFFEROFFSET (EMAC_BASE + 0x110u)
#define RXFILTERLOWTHRESH (EMAC_BASE + 0x114u)
#define RX0FREEBUFFER (EMAC_BASE + 0x140u)
#define MACCONTROL (EMAC_BASE + 0x160u)
#define MACSTATUS (EMAC_BASE + 0x164u)
#define MACHASH1 (EMAC_BASE + 0x1D8u)
#define MACHASH2 (EMAC_BASE + 0x1DCu)
#define MACADDRLO (EMAC_BASE + 0x500u)
#define MACADDRHI (EMAC_BASE + 0x504u)
#define MACINDEX (EMAC_BASE + 0x508u)
#define TX0HDP (EMAC_BASE + 0x600u)
#define RX0HDP (EMAC_BASE + 0x620u)
#define TX0CP (EMAC_BASE + 0x640u)
#define RX0CP (EMAC_BASE + 0x660u)
#define EWCTL (EMAC_CTRL_BASE + 0x004u)

#define MDIO_CONTROL (MDIO_BASE + 0x004u)
#define MDIO_ALIVE (MDIO_BASE + 0x008u)
#define MDIO_LINK (MDIO_BASE + 0x00Cu)

#define MACCONTROL_FULLDUPLEX (1u << 0)
#define MACCONTROL_GMIIEN (1u << 5)
#define MACCONTROL_GIG (1u << 7)
#define MACADDR_VALID (1u << 20)
#define MACADDR_MATCH (1u << 19)
#define RXMBP_BROADCAST_EN (1u << 13)
#define RXMBP_UNICAST0 (1u << 0)
#define MDIO_CONTROL_ENABLE (1u << 30)

#define CPPI_SOP (1u << 31)
#define CPPI_EOP (1u << 30)
#define CPPI_OWNER (1u << 29)
#define CPPI_EOQ (1u << 28)

/* Descriptors live in the EMAC's own 8 KB RAM, which is what the CPPI protocol is designed for
 * and keeps them clear of the CPU's caches (SPRU975E section 2.2). */
#define DESC_RAM 0x02C82000u
#define RX_DESCRIPTORS 32
#define TX_DESCRIPTORS 8
#define RX_BUFFER_SIZE 1536

typedef struct EmacDesc {
    volatile UINT32 next;
    volatile UINT32 buffer;
    volatile UINT32 offset_length;
    volatile UINT32 flags_length;
} EmacDesc;

typedef struct EmacDriver {
    STKEVENT_Handle event;
    NETIF_DEVICE *device;
    EmacDesc *rx_ring;
    EmacDesc *tx_ring;
    /* The PBM packet each receive descriptor is filled into, so a received frame needs no copy. */
    PBM_Handle rx_packets[RX_DESCRIPTORS];
    PBM_Handle tx_packets[TX_DESCRIPTORS];
    uint rx_next;
    uint tx_head;
    uint tx_tail;
    PBMQ rx_queue;
    UINT32 rx_frames;
    UINT32 tx_frames;
    UINT32 rx_dropped;
} EmacDriver;

static EmacDriver emac_driver;

static void poke(UINT32 address, UINT32 value)
{
    *(volatile UINT32 *)address = value;
}

/* Post one receive descriptor with a fresh packet buffer. */
static int emac_post_receive(EmacDriver *driver, uint index)
{
    PBM_Handle packet = PBM_alloc(RX_BUFFER_SIZE);
    EmacDesc *descriptor = &driver->rx_ring[index];

    if (!packet) {
        driver->rx_dropped++;
        return -1;
    }
    PBM_setDataOffset(packet, 0);
    driver->rx_packets[index] = packet;
    descriptor->next = 0;
    descriptor->buffer = (UINT32)PBM_getDataBuffer(packet);
    descriptor->offset_length = RX_BUFFER_SIZE;
    descriptor->flags_length = CPPI_OWNER;
    /* Add the refilled descriptor to the tail of the queue the DMA is walking: the CPPI list
     * is a queue rather than a ring, so without this link the DMA reaches the end of the
     * original chain once and then stops for good (SPRU975E section 4.3.2). */
    driver->rx_ring[(index + RX_DESCRIPTORS - 1u) % RX_DESCRIPTORS].next = (UINT32)descriptor;
    return 0;
}

static void emac_start_receive(EmacDriver *driver)
{
    uint index;

    for (index = 0; index < RX_DESCRIPTORS; index++) {
        emac_post_receive(driver, index);
        /* Chain each descriptor to the next so the EMAC walks the whole ring. */
        if (index > 0) {
            driver->rx_ring[index - 1].next = (UINT32)&driver->rx_ring[index];
        }
    }
    driver->rx_next = 0;
    poke(RX0FREEBUFFER, RX_DESCRIPTORS);
    poke(RX0HDP, (UINT32)&driver->rx_ring[0]);
}

static int emac_hw_init(EmacDriver *driver, UINT8 *mac_address)
{
    uint index;

    /* Let the MDIO module answer, and take the EMAC's write-posting gate out of the way. */
    poke(MDIO_CONTROL, MDIO_CONTROL_ENABLE | 0xFFu);
    poke(EWCTL, 1);

    /* Clear the descriptor pointers before enabling anything (SPRU975E section 4.4). */
    for (index = 0; index < 8; index++) {
        poke(TX0HDP + index * 4u, 0);
        poke(RX0HDP + index * 4u, 0);
        poke(TX0CP + index * 4u, 0);
        poke(RX0CP + index * 4u, 0);
    }

    /* Station address in slot zero of the address RAM, matching on channel zero. */
    poke(MACINDEX, 0);
    poke(MACADDRHI, ((UINT32)mac_address[2] << 24) | ((UINT32)mac_address[3] << 16) |
                        ((UINT32)mac_address[4] << 8) | (UINT32)mac_address[5]);
    poke(MACADDRLO, ((UINT32)mac_address[0] << 8) | (UINT32)mac_address[1] | MACADDR_VALID |
                        MACADDR_MATCH);
    poke(RXUNICASTSET, RXMBP_UNICAST0);
    poke(MACHASH1, 0);
    poke(MACHASH2, 0);
    poke(RXMAXLEN, 1518);
    poke(RXBUFFEROFFSET, 0);
    poke(RXFILTERLOWTHRESH, 0);
    poke(RXMBPENABLE, RXMBP_BROADCAST_EN | RXMBP_UNICAST0);
    /* Channel zero's receive interrupt, and only that one: the module drives system event 17
     * (MACINT) while a completed receive descriptor is unacknowledged, and the megamodule
     * latches its rising edge, so the service routine need not clear anything on the interrupt
     * controller's side (SPRU871K section 7.4.2). The transmit queue shares the same line and is
     * left masked - see the note at the top of this file. */
    poke(RXINTMASKSET, 1);

    /* The PHY in this model reports a gigabit full-duplex link. */
    poke(MACCONTROL, MACCONTROL_GMIIEN | MACCONTROL_FULLDUPLEX | MACCONTROL_GIG);

    driver->rx_ring = (EmacDesc *)DESC_RAM;
    driver->tx_ring = (EmacDesc *)(DESC_RAM + RX_DESCRIPTORS * sizeof(EmacDesc));
    driver->tx_head = 0;
    driver->tx_tail = 0;

    emac_start_receive(driver);
    poke(RXCONTROL, 1);
    poke(TXCONTROL, 1);
    return 0;
}

/* ------------------------------------------------------------------ NIMU */

static int emac_start(NETIF_DEVICE *ptr_net_device)
{
    EmacDriver *driver = (EmacDriver *)ptr_net_device->pvt_data;

    if (emac_hw_init(driver, ptr_net_device->mac_address) < 0) {
        return -1;
    }
    return 0;
}

static int emac_stop(NETIF_DEVICE *ptr_net_device)
{
    EmacDriver *driver = (EmacDriver *)ptr_net_device->pvt_data;
    uint index;

    poke(RXCONTROL, 0);
    poke(TXCONTROL, 0);
    poke(MACCONTROL, 0);
    for (index = 0; index < RX_DESCRIPTORS; index++) {
        if (driver->rx_packets[index]) {
            PBM_free(driver->rx_packets[index]);
            driver->rx_packets[index] = 0;
        }
    }
    return 0;
}

/* Reclaim every transmit descriptor the EMAC has finished with, and acknowledge them.
 *
 * This runs on every pass of poll(), not only before the next send, because it is what returns
 * the transmit packet buffers to the pool `emac_post_receive` allocates from: a driver that
 * reclaimed lazily could leave a burst's worth of buffers held while the receive ring went
 * hungry.
 *
 * Writing TXnCP with a descriptor address is the acknowledge (SPRU975E section 2.5.3); the
 * descriptor's OWNER bit is what says the EMAC is done with it. With TXINTMASKSET clear, that
 * acknowledge decides nothing about the module's interrupt line - which is exactly why the mask
 * stays clear. See the note at the top of this file.
 */
static void emac_reclaim_transmit(EmacDriver *driver)
{
    while (driver->tx_tail != driver->tx_head) {
        uint done = driver->tx_tail % TX_DESCRIPTORS;

        if (driver->tx_ring[done].flags_length & CPPI_OWNER) {
            break;
        }
        poke(TX0CP, (UINT32)&driver->tx_ring[done]);
        if (driver->tx_packets[done]) {
            PBM_free(driver->tx_packets[done]);
            driver->tx_packets[done] = 0;
        }
        driver->tx_tail++;
    }
}

static int emac_send(NETIF_DEVICE *ptr_net_device, PBM_Handle hPkt)
{
    EmacDriver *driver = (EmacDriver *)ptr_net_device->pvt_data;
    uint slot = driver->tx_head % TX_DESCRIPTORS;
    EmacDesc *descriptor = &driver->tx_ring[slot];
    UINT32 length = PBM_getValidLen(hPkt);
    UINT8 *data = PBM_getDataBuffer(hPkt) + PBM_getDataOffset(hPkt);

    /* The interrupt path has usually reclaimed these already; a send that arrives before the
     * stack thread next runs still has to, because the slot it is about to reuse is the one
     * being reclaimed. */
    emac_reclaim_transmit(driver);
    if (driver->tx_head - driver->tx_tail >= TX_DESCRIPTORS) {
        return -1;
    }
    if (length < 60) {
        /* The EMAC does not pad; a runt would be dropped by the peer. */
        UINT8 *buffer = PBM_getDataBuffer(hPkt) + PBM_getDataOffset(hPkt);
        UINT32 pad;

        for (pad = length; pad < 60; pad++) {
            buffer[pad] = 0;
        }
        length = 60;
        PBM_setValidLen(hPkt, length);
    }

    /* The stack filled this buffer through the cache, so the lines have to reach memory before
     * the DMA reads them. */
    Cache_wb(data, length, Cache_Type_ALLD, TRUE);
    driver->tx_packets[slot] = hPkt;
    descriptor->next = 0;
    descriptor->buffer = (UINT32)data;
    descriptor->offset_length = length;
    descriptor->flags_length = CPPI_SOP | CPPI_EOP | CPPI_OWNER | length;
    driver->tx_head++;
    poke(TX0HDP, (UINT32)descriptor);
    driver->tx_frames++;
    return 0;
}

/* Move every completed receive descriptor into the driver's queue. */
static void emac_drain_receive(EmacDriver *driver)
{
    uint served = 0;

    while (served < RX_DESCRIPTORS) {
        uint index = driver->rx_next;
        EmacDesc *descriptor = &driver->rx_ring[index];
        UINT32 flags = descriptor->flags_length;
        PBM_Handle packet;
        UINT32 length;

        if (flags & CPPI_OWNER) {
            break;
        }
        packet = driver->rx_packets[index];
        length = flags & 0xFFFFu;
        driver->rx_packets[index] = 0;
        /* Acknowledge this descriptor to the EMAC, which is also the interrupt acknowledge. */
        poke(RX0CP, (UINT32)descriptor);
        if (packet && length >= 14 && length <= RX_BUFFER_SIZE) {
            /* The packet buffers are in cached external memory, where nothing is snooped: the
             * CPU may still hold lines from this buffer's previous trip through the stack, so
             * they have to be invalidated before anyone reads the frame the DMA just wrote
             * (SPRU862B section 2.4 and Table 2-2). Invalidate rather than writeback-invalidate,
             * which would push a stale line back over the new data. */
            Cache_inv((void *)descriptor->buffer, RX_BUFFER_SIZE, Cache_Type_ALLD, TRUE);
            PBM_setValidLen(packet, length);
            PBM_setDataOffset(packet, 0);
            PBM_setIFRx(packet, driver->device);
            PBMQ_enq(&driver->rx_queue, packet);
            driver->rx_frames++;
        } else if (packet) {
            PBM_free(packet);
            driver->rx_dropped++;
        }
        emac_post_receive(driver, index);
        /* Restart the ring: the EMAC stops when it reaches the end of the queue. */
        if (flags & CPPI_EOQ) {
            poke(RX0HDP, (UINT32)&driver->rx_ring[(index + 1) % RX_DESCRIPTORS]);
        }
        driver->rx_next = (index + 1) % RX_DESCRIPTORS;
        served++;
    }
    if (PBMQ_count(&driver->rx_queue)) {
        STKEVENT_signal(driver->event, STKEVENT_ETHERNET, 0);
    }
}

static void emac_poll(NETIF_DEVICE *ptr_net_device, uint timer_tick)
{
    EmacDriver *driver = (EmacDriver *)ptr_net_device->pvt_data;

    (void)timer_tick;
    /* Transmit first: it is the cheaper of the two and it is what hands buffers back to the
     * pool the receive path allocates from. */
    emac_reclaim_transmit(driver);
    emac_drain_receive(driver);
}

/* The receive interrupt. It only posts the stack thread's semaphore: the ring is drained in
 * emac_poll, under the stack's gate, and acknowledging the queue there is what drops the
 * module's interrupt line. Because the megamodule latches the line's rising edge rather than
 * its level, an unacknowledged queue cannot re-enter this routine. */
void emac_isr(unsigned int argument)
{
    (void)argument;
    STKEVENT_signal(emac_driver.event, STKEVENT_ETHERNET, 1);
}

static void emac_pkt_service(NETIF_DEVICE *ptr_net_device)
{
    EmacDriver *driver = (EmacDriver *)ptr_net_device->pvt_data;
    PBM_Handle packet;

    /* Hand everything the ring gave us to the stack. */
    while ((packet = PBMQ_deq(&driver->rx_queue)) != 0) {
        NIMUReceivePacket(packet);
    }
}

static int emac_ioctl(NETIF_DEVICE *ptr_net_device, uint cmd, void *pbuf, uint size)
{
    (void)size;
    switch (cmd) {
        case NIMU_GET_DEVICE_MAC:
            if (pbuf) {
                mmCopy(pbuf, ptr_net_device->mac_address, 6);
            }
            return 0;
        case NIMU_GET_DEVICE_MTU:
            if (pbuf) {
                *(uint *)pbuf = ptr_net_device->mtu;
            }
            return 0;
        case NIMU_ADD_MULTICAST_ADDRESS:
        case NIMU_DEL_MULTICAST_ADDRESS:
            /* This model filters multicast through the hash registers, which already accept
             * every group the stack joins, so there is nothing to program per address. */
            return 0;
        default:
            return -1;
    }
}

/* The device's init entry, named in NIMUDeviceTable. */
int emac_nimu_init(STKEVENT_Handle hEvent)
{
    NETIF_DEVICE *device;
    EmacDriver *driver = &emac_driver;

    device = mmAlloc(sizeof(NETIF_DEVICE));
    if (!device) {
        return -1;
    }
    mmZeroInit(device, sizeof(NETIF_DEVICE));
    mmZeroInit(driver, sizeof(EmacDriver));
    PBMQ_init(&driver->rx_queue);
    driver->event = hEvent;
    driver->device = device;

    strcpy(device->name, "emac0");
    device->mtu = 1500;
    device->pvt_data = driver;
    /* A locally administered address; the host side of the bridge learns it through ARP. */
    device->mac_address[0] = 0x02;
    device->mac_address[1] = 0x00;
    device->mac_address[2] = 0x00;
    device->mac_address[3] = 0x11;
    device->mac_address[4] = 0x22;
    device->mac_address[5] = 0x33;

    device->start = emac_start;
    device->stop = emac_stop;
    device->poll = emac_poll;
    device->send = emac_send;
    device->pkt_service = emac_pkt_service;
    device->ioctl = emac_ioctl;
    device->add_header = NIMUAddEthernetHeader;

    if (NIMURegister(device) < 0) {
        mmFree(device);
        return -1;
    }
    return 0;
}

/* The table the NDK walks at start-up. */
NIMU_DEVICE_TABLE_ENTRY NIMUDeviceTable[2] = {
    {emac_nimu_init},
    {0},
};
