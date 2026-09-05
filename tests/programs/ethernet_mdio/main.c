#include <stdio.h>
/* MDIO bring-up: enable the management interface, find the PHY, read its identifier, wait for
   auto-negotiation, work out the negotiated speed and duplex and configure MACCONTROL from
   it - the sequence a real driver runs before it enables the EMAC. */

#define EMAC_BASE      0x02C80000u
#define MACCONTROL     (EMAC_BASE + 0x160u)

#define MDIO_BASE      0x02C81800u
#define MDIO_VERSION   (MDIO_BASE + 0x000u)
#define MDIO_CONTROL   (MDIO_BASE + 0x004u)
#define MDIO_ALIVE     (MDIO_BASE + 0x008u)
#define MDIO_LINK      (MDIO_BASE + 0x00Cu)
#define MDIO_LINKINTRAW (MDIO_BASE + 0x010u)
#define MDIO_USERACCESS0 (MDIO_BASE + 0x080u)

#define MDIO_CONTROL_ENABLE (1u << 30)
#define MDIO_GO    (1u << 31)
#define MDIO_WRITE (1u << 30)
#define MDIO_ACK   (1u << 29)

#define PHY 0u
#define BMCR   0u
#define BMSR   1u
#define PHYID1 2u
#define PHYID2 3u
#define ANAR   4u
#define ANLPAR 5u
#define GBSR   10u

#define BMSR_LINK        (1u << 2)
#define BMSR_AN_COMPLETE (1u << 5)
#define MACCONTROL_FULLDUPLEX (1u << 0)
#define MACCONTROL_GMIIEN     (1u << 5)
#define MACCONTROL_GIG        (1u << 7)

static void poke(unsigned int address, unsigned int value)
{
    *(volatile unsigned int *)address = value;
}

static unsigned int peek(unsigned int address)
{
    return *(volatile unsigned int *)address;
}

/* One clause-22 read: start the transaction and poll GO until the module clears it. */
static unsigned int phy_read(unsigned int phy, unsigned int reg, unsigned int *polls)
{
    unsigned int status;
    poke(MDIO_USERACCESS0, MDIO_GO | (reg << 21) | (phy << 16));
    do {
        status = peek(MDIO_USERACCESS0);
        *polls = *polls + 1u;
    } while ((status & MDIO_GO) != 0u);
    if ((status & MDIO_ACK) == 0u) {
        return 0xFFFFFFFFu;
    }
    return status & 0xFFFFu;
}

static void phy_write(unsigned int phy, unsigned int reg, unsigned int value)
{
    poke(MDIO_USERACCESS0, MDIO_GO | MDIO_WRITE | (reg << 21) | (phy << 16) | (value & 0xFFFFu));
    while ((peek(MDIO_USERACCESS0) & MDIO_GO) != 0u) {
    }
}

int main(void)
{
    unsigned int polls = 0;
    unsigned int alive_before;
    unsigned int identifier;
    unsigned int status;
    unsigned int partner;
    unsigned int gigabit_status;
    unsigned int control = MACCONTROL_GMIIEN;
    unsigned int missing;
    unsigned int round_trip;
    unsigned int spins = 0;

    alive_before = peek(MDIO_ALIVE);
    poke(MDIO_CONTROL, MDIO_CONTROL_ENABLE | 0xFFu);
    while (peek(MDIO_ALIVE) == 0u && spins < 100000u) {
        spins++;
    }

    identifier = (phy_read(PHY, PHYID1, &polls) << 16) | phy_read(PHY, PHYID2, &polls);
    status = phy_read(PHY, BMSR, &polls);
    if ((status & BMSR_AN_COMPLETE) == 0u) {
        /* Restart auto-negotiation and wait for it. */
        phy_write(PHY, BMCR, 0x1200u);
        do {
            status = phy_read(PHY, BMSR, &polls);
        } while ((status & BMSR_AN_COMPLETE) == 0u);
    }
    partner = phy_read(PHY, ANLPAR, &polls);
    gigabit_status = phy_read(PHY, GBSR, &polls);
    if ((gigabit_status & (1u << 11)) != 0u) {
        control |= MACCONTROL_GIG | MACCONTROL_FULLDUPLEX;
    } else if ((partner & (1u << 8)) != 0u) {
        control |= MACCONTROL_FULLDUPLEX;
    }
    poke(MACCONTROL, control);

    /* A read of an address with no PHY must not acknowledge, and a writable register must
       round trip. */
    missing = phy_read(7u, PHYID1, &polls);
    phy_write(PHY, ANAR, 0x0061u);
    round_trip = phy_read(PHY, ANAR, &polls);

    printf("ethernet_mdio alive=%u->%u id=%u link=%u status=%u partner=%u gig=%u\n",
           alive_before, peek(MDIO_ALIVE), identifier != 0u && identifier != 0xFFFFFFFFu ? 1u : 0u,
           peek(MDIO_LINK), (status & (BMSR_LINK | BMSR_AN_COMPLETE)) ==
           (BMSR_LINK | BMSR_AN_COMPLETE) ? 1u : 0u, partner, gigabit_status);
    printf("maccontrol=%u missing=%u anar=%u polls=%u linkint=%u\n", peek(MACCONTROL),
           missing == 0xFFFFFFFFu ? 1u : 0u, round_trip, polls > 6u ? 1u : 0u,
           peek(MDIO_LINKINTRAW));
    return 0;
}
