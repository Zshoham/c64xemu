---
name: c6455-emac-ethernet
description: "Model the C6455 Ethernet MAC and MDIO, including CPPI descriptors, queue handshakes, interrupts, receive filtering, PHY behavior, and host bridging."
---

# C6455 EMAC / MDIO

*Modelling the C6455 Ethernet MAC and MDIO — CPPI buffer descriptors, the HDP/CP queue and interrupt-acknowledge protocol, TX/RX descriptor flags, receive filtering, the single shared MACINT and EWCTL gating, the MDIO/PHY model, and how to bridge emulated Ethernet to a host TAP or pcap.*

This is the peripheral the whole project exists for: packets in, packets out, DSP code in
between. Get the descriptor protocol and the interrupt-acknowledge handshake exactly right
and everything else follows.

## Address map (C6455)

| Base | Size | Contents |
|---|---|---|
| `0x02C8_0000` | 4 KB | **EMAC port registers** (offsets in Table 29 of SPRU975E) |
| `0x02C8_1000` | 2 KB | **EMAC control module**: `+4h` EWCTL, `+8h` EWINTTCNT |
| `0x02C8_1800` | 2 KB | **MDIO registers** |
| `0x02C8_2000` | 8 KB | **descriptor RAM** — 2K × 32 bits = 512 descriptors of 16 bytes |

System event: **17 (MACINT)** — see `skills/c64xplus-interrupts-and-exceptions/SKILL.md`.

SPRU975E §2.2 phrases the descriptor RAM as holding "up to 512 Ethernet packets"; that is
true only for single-fragment packets. The hard limit is **512 descriptors**, and a
fragmented packet consumes one per fragment.

Descriptors placed in DDR2 rather than L2 SRAM or the EMAC's own RAM are subject to the
memory attribute registers — if the MAR for that region has PC = 1 they are cached, and every
descriptor read and write becomes a coherence problem. See `skills/c64xplus-memory-and-cache/SKILL.md`.

## Buffer descriptors (CPPI)

Four 32-bit words, 32-bit aligned:

```
word 0:  Next Descriptor Pointer      (0 = end of list)
word 1:  Buffer Pointer               (byte-aligned)
word 2:  [31:16] Buffer Offset   [15:0] Buffer Length
word 3:  [31:16] Flags           [15:0] Packet Length
```

```c
typedef struct _EMAC_Desc {
    struct _EMAC_Desc *pNext;
    uint8_t           *pBuffer;
    uint32_t           BufOffLen;   /* offset << 16 | length */
    uint32_t           PktFlgLen;   /* flags  << 16 | pktlen */
} EMAC_Desc;
```

Flags (word 3, upper half):

| Bit | TX | RX |
|---|---|---|
| 31 | SOP | SOP |
| 30 | EOP | EOP |
| 29 | OWNER | OWNER |
| 28 | EOQ | EOQ |
| 27 | TDOWNCMPLT | TDOWNCMPLT |
| 26 | PASSCRC | PASSCRC |
| 25 | — | JABBER |
| 24 | — | OVERSIZE |
| 23 | — | FRAGMENT |
| 22 | — | UNDERSIZED |
| 21 | — | CONTROL |
| 20 | — | OVERRUN |
| 19 | — | CODEERROR |
| 18 | — | ALIGNERROR |
| 17 | — | CRCERROR |
| 16 | — | NOMATCH |

As constants: `SOP 0x80000000, EOP 0x40000000, OWNER 0x20000000, EOQ 0x10000000,
TDOWNCMPLT 0x08000000, PASSCRC 0x04000000`.

### Field rules that the model must enforce

- **Ownership is per *packet*, not per descriptor.** Only the SOP descriptor's `OWNER` bit is
  meaningful. The EMAC clears OWNER on the SOP descriptor once it has finished every
  descriptor up to and including the one with EOP set.
- **Buffer Offset** is only checked on the SOP descriptor and is never modified by the EMAC.
  Legal range 0 … (Buffer Length − 1).
- **Buffer Length** counts valid data bytes, excluding offset bytes. Set by software on TX;
  written by the EMAC on RX (where it initially holds the empty buffer's size).
- **Packet Length** is only meaningful on SOP. On TX it must equal the sum of all fragments'
  buffer lengths.
- `pNext` and `pBuffer` are **never** altered by the EMAC.
- `PASSCRC` on TX means software already appended the 4-byte FCS and the EMAC must not
  generate one; in that case the length fields *include* the CRC bytes.

## Queues: HDP and CP

Eight TX channels and eight RX channels. Per channel:

- `TXnHDP` / `RXnHDP` — head descriptor pointer. **Write the pointer to start the channel.**
- `TXnCP` / `RXnCP` — completion pointer, doubling as the interrupt acknowledge register.

Rules:

1. After reset and before enabling, software must write **zero to all 16 HDP registers**.
2. The last descriptor of a submitted list must have `pNext == 0`.
3. **`HDP` must never be written a second time while a list is still active.** To append,
   software patches the NULL `pNext` of the previous last descriptor.
4. That patch races with the EMAC reading `pNext` as NULL. If the EMAC lost the race it
   halts the channel and sets **EOQ** on the last EOP descriptor it processed. Software
   detects EOQ and restarts by writing the new list head to `HDP`.

Model this literally — the race is real and NDK's driver depends on the EOQ behaviour.
Your DMA engine should read `pNext` once, and if it read NULL, halt the channel and set EOQ
on that descriptor. Do not "helpfully" re-read.

## Interrupt / acknowledge protocol

`TXnCP`/`RXnCP` serve two purposes:

- **Read** → the pointer to the last descriptor the *EMAC* processed.
- **Write** → the pointer to the last descriptor the *software* processed.

**The channel interrupt is active whenever those two values differ.** That is the whole
mechanism — an interrupt cannot be lost, because it is a state comparison, not an edge.

Layered on top:

- `TXINTMASKSET` / `TXINTMASKCLEAR`, `RXINTMASKSET` / `RXINTMASKCLEAR` — per-channel enables
- `TXINTSTATRAW` / `RXINTSTATRAW` — unmasked state, readable regardless of enable
- `TXINTSTATMASKED` / `RXINTSTATMASKED` — after masking
- `MACINTSTATRAW` / `MACINTSTATMASKED`, `MACINTMASKSET` / `MACINTMASKCLEAR` — MAC-level
  (host error, statistics rollover)

### EWCTL — the single shared interrupt

All EMAC and MDIO interrupts funnel into **one** CPU event (17, MACINT), gated by
`EWCTL.INTEN` at `0x02C8_1004`. The documented flow is:

1. On entry to the ISR, software clears `EWCTL.INTEN` → the interrupt line drops to zero.
2. Software services and clears every pending source.
3. Software sets `EWCTL.INTEN` again before returning.
4. If the control module still sees pending interrupts, it **re-asserts the line, generating
   a new edge** the DSP can recognise.

Model the line as level-derived-but-edge-delivered: compute
`line = INTEN && (any_masked_source_pending)`, and raise system event 17 on each 0→1
transition. Without the INTEN dance a level source would never re-trigger.

`EWINTTCNT` (`0x02C8_1008`) is an interrupt pacing timer — a minimum inter-interrupt gap.
Implement it as a countdown that holds the line low; firmware that sets it and never sees
interrupts is a good early test of the model.

## Key EMAC port registers (offsets from `0x02C8_0000`)

| Offset | Register |
|---|---|
| 0x00 / 0x10 | TXIDVER / RXIDVER |
| 0x04 / 0x14 | TXCONTROL / RXCONTROL (enable bits) |
| 0x08 / 0x18 | TXTEARDOWN / RXTEARDOWN |
| 0x80 / 0xA0 | TXINTSTATRAW / RXINTSTATRAW |
| 0x84 / 0xA4 | TXINTSTATMASKED / RXINTSTATMASKED |
| 0x88 / 0xA8 | TXINTMASKSET / RXINTMASKSET |
| 0x8C / 0xAC | TXINTMASKCLEAR / RXINTMASKCLEAR |
| 0x90 | MACINVECTOR |
| 0xB0-0xBC | MACINTSTATRAW / MACINTSTATMASKED / MACINTMASKSET / MACINTMASKCLEAR |
| 0x100 | RXMBPENABLE — multicast/broadcast/promiscuous and per-channel enables |
| 0x104 / 0x108 | RXUNICASTSET / RXUNICASTCLEAR |
| 0x10C | RXMAXLEN |
| 0x110 | RXBUFFEROFFSET |
| 0x114 | RXFILTERLOWTHRESH |
| 0x120-0x13C | RX0-7FLOWTHRESH |
| 0x140-0x15C | RX0-7FREEBUFFER |
| 0x160 / 0x164 | MACCONTROL / MACSTATUS |
| 0x168 | EMCONTROL (emulation) |

TXnHDP/RXnHDP and TXnCP/RXnCP live further up the block — read the full Table 29 in
SPRU975E §5 when you implement them.

`RXBUFFEROFFSET` is a global offset the EMAC writes into every RX descriptor's Buffer Offset
field — NDK uses it to leave room for headers.

## Receive filtering

`RXMBPENABLE` controls, per channel: unicast enable, multicast enable, broadcast enable,
promiscuous mode and which channel promiscuous/multicast/broadcast traffic lands on.
`RXUNICASTSET`/`CLEAR` enable per-channel unicast matching against the MAC address registers.

`NOMATCH` in the RX flags means the frame was accepted only because of promiscuous mode.
`RXMAXLEN` sets the oversize threshold (`OVERSIZE` flag).

A minimal but honest model: implement address matching (unicast against the configured MAC
addresses, broadcast, multicast hash if firmware uses it, promiscuous), `RXMAXLEN`, and the
`NOMATCH` flag. Skip the error flags you cannot generate (`CODEERROR`, `ALIGNERROR`,
`JABBER`) unless you add a fault-injection mode — which is worth having, since firmware
error paths are otherwise never exercised.

## Statistics

The EMAC has a large block of statistics registers (frames received/transmitted by type,
collisions, errors). Implement them as counters incremented in the datapath; a rollover
raises a MAC interrupt. Firmware often reads them at init and expects sane values.

## MDIO and the PHY

MDIO registers at `0x02C8_1800`:

| Offset | Register |
|---|---|
| 0x00 | VERSION |
| 0x04 | CONTROL |
| 0x08 | ALIVE — bit per PHY address that responded |
| 0x0C | LINK — bit per PHY address with link up |
| 0x10 / 0x14 | LINKINTRAW / LINKINTMASKED |
| 0x20 / 0x24 | USERINTRAW / USERINTMASKED |
| 0x28 / 0x2C | USERINTMASKSET / USERINTMASKCLEAR |
| 0x80 / 0x84 | USERACCESS0 / USERPHYSEL0 |
| 0x88 / 0x8C | USERACCESS1 / USERPHYSEL1 |

`USERACCESSn` is the transaction register: software writes GO + read/write + PHY address +
register address (+ data for a write); the module clears GO and sets DATA when done, then
raises the user-interrupt.

**Model a simple IEEE 802.3 clause-22 PHY.** Firmware will not bring the link up without one.
Minimum registers:

| Reg | Name | Model |
|---|---|---|
| 0 | BMCR | accept writes; clear the reset bit immediately; honour speed/duplex/autoneg-restart bits |
| 1 | BMSR | link up, autoneg capable, **autoneg complete**, extended capability |
| 2, 3 | PHYID1/2 | any plausible OUI (e.g. a Marvell or National ID) |
| 4 | ANAR | echo what was written |
| 5 | ANLPAR | advertise 1000/100/10 full duplex as the "link partner" |
| 9, 10 | 1000BASE-T control/status | if firmware checks gigabit |

Set the matching bit in `ALIVE` and `LINK` for whichever PHY address the board expects, and
raise `LINKINTRAW` on a link-state change. Getting this wrong is the classic "the driver
initialises and then waits forever" failure.

The `MACCONTROL` register's GIG/FULLDUPLEX/RMIISPEED bits must be consistent with what the
PHY reports, or the driver may loop.

## Host bridging

The emulated MAC has to reach a real network. Options, roughly in order of usefulness here:

1. **TAP device** (`/dev/net/tun`, `IFF_TAP | IFF_NO_PI`) — a layer-2 interface on the host.
   The emulator reads/writes raw Ethernet frames. Best default: no root needed if the TAP is
   pre-created and owned by the user, and it composes with bridges and other emulator
   instances.
2. **AF_PACKET raw socket** bound to a real NIC — for talking to physical peers.
3. **UDP tunnel / multicast group** — the simplest way to connect several emulated DSP cores
   to each other, which is exactly the stated use case ("communicating with other cores only
   through Ethernet"). Each instance joins a group; frames are encapsulated in UDP.
4. **pcap file replay** — deterministic, replayable test input. Invaluable for regression
   tests; make it the first one you build.

Design the interface as a small vtable so all four are interchangeable:

```c
typedef struct {
    int  (*rx_poll)(void *ctx, uint8_t *buf, size_t max);  /* 0 if nothing */
    void (*tx)(void *ctx, const uint8_t *buf, size_t len);
    void *ctx;
} eth_backend;
```

### Threading and timing

The CPU model is single-threaded and cycle-stepped; the host network is asynchronous. Use a
lock-free ring between a reader thread and the emulator loop, and have the EMAC model poll
that ring at a fixed cycle interval (e.g. every N emulated cycles). Do **not** deliver frames
directly from the reader thread into descriptor RAM — that reintroduces the exact race the
descriptor protocol is designed around, in a place where you cannot debug it.

For deterministic replay, drive frame arrival off the emulated cycle counter rather than
wall-clock time, and make the backend choice determine which.

## Descriptor memory placement

Descriptors may live in the EMAC's own 8 KB RAM at `0x02C8_2000` **or** in L2 SRAM. TI notes
the trade-off: descriptors in EMAC RAM give better EMAC throughput; descriptors in L2 give
better cache behaviour for the CPU.

**This matters for the emulator's coherence model.** When descriptors are in L2 SRAM, the
EMAC's accesses to them are DMA accesses that must go through the L2 snoop path (see
`skills/c64xplus-memory-and-cache/SKILL.md`). When they are in EMAC RAM, the CPU's accesses are uncached
peripheral accesses. Both paths must work, and firmware may use either.

Packet buffers are usually in L2 SRAM or DDR2. For DDR2 buffers there is **no hardware
coherence** — firmware must writeback before TX and invalidate before reading RX data. Model
that faithfully; it is the highest-value bug class this emulator can catch.

## Files that expand on this

**Documents**

- `docs/reference/spru975e.pdf` (SPRU975E) — the EMAC/MDIO user's guide
  - §2.2, p. 16 — the internal descriptor RAM
  - §2.4, pp. 24-25 — Ethernet frame format and the multiple-access protocol
  - §2.5.1-2.5.3, pp. 25-27 — descriptors, the HDP/CP queue mechanism, the EOQ race, and the
    interrupt/acknowledge protocol
  - §2.5.4-2.5.5, pp. 27-33 — full TX and RX descriptor formats with every flag defined
  - §2.6-2.17 — receive filtering, flow control, teardown, statistics, emulation
  - §3, pp. 61-62 — the control module: EWCTL and EWINTTCNT
  - §4, pp. 62-79 — MDIO registers, including USERACCESSn transaction handling
  - §5, pp. 79+ — Table 29, the complete EMAC port register list with offsets
- `docs/reference/spraa90.pdf` (SPRAA90) — software operation of the gigabit EMAC,
  i.e. the driver-level sequence the register list does not spell out
- `docs/reference/spraau2a.pdf` (SPRAAU2A) — a full EMAC implementation guide
- `docs/reference/sprs276m.pdf` §2.3 and §7.5.1 — where the EMAC lives in the
  memory map and that it is system event 17
- `docs/reference/sprz234r.pdf` — errata; the EMAC entries are worth reading before
  deciding the model is wrong
- `docs/reference/sprufp2.pdf` (SPRUFP2) — how TI's NDK drives this EMAC through
  the NIMU layer
- `docs/reference/spru523k.pdf` / `spru524k.pdf` — the NDK user's guide and API
  reference, the stack most C6455 firmware runs
- `docs/reference/spraax4.pdf` — getting started with the NDK, useful for building
  a first end-to-end test image
- `docs/reference/sprui04g.pdf` §7.9.5 / §7.9.7 — the `DATA_ALIGN` and `DATA_SECTION` pragmas used to
  align descriptors and packet buffers to cache lines; misuse of these is what makes the
  coherence bugs this model is meant to expose

**Source**

- [`drivers/net/ethernet/ti/davinci_emac.c`](https://github.com/torvalds/linux/blob/v5.10/drivers/net/ethernet/ti/davinci_emac.c) — the Linux driver for **the same IP block**;
  the clearest executable specification of descriptor handling, filtering and interrupt ack
- [`drivers/net/ethernet/ti/davinci_cpdma.c`](https://github.com/torvalds/linux/blob/v5.10/drivers/net/ethernet/ti/davinci_cpdma.c) and [`davinci_cpdma.h`](https://github.com/torvalds/linux/blob/v5.10/drivers/net/ethernet/ti/davinci_cpdma.h) — the CPPI DMA engine
  in isolation: descriptor pools, submit, and the completion path
- [`drivers/net/ethernet/ti/davinci_mdio.c`](https://github.com/torvalds/linux/blob/v5.10/drivers/net/ethernet/ti/davinci_mdio.c) — MDIO transactions and PHY polling
- [`drivers/net/ti/davinci_emac.c`](https://github.com/u-boot/u-boot/blob/v2025.01/drivers/net/ti/davinci_emac.c) and [`arch/arm/mach-davinci/include/mach/emac_defs.h`](https://github.com/u-boot/u-boot/blob/v2025.01/arch/arm/mach-davinci/include/mach/emac_defs.h) — a much smaller
  driver covering the same registers; the easier one to read first
