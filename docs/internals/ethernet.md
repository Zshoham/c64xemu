# Ethernet

The EMAC, the MDIO module and the host backends behind them: `src/emac.rs` holds the MAC and its
two DMA engines, `src/mdio.rs` the management interface and the PHY on the other end of it,
`src/emic.rs` the C6472's per-core interrupt-control module, and `src/netio.rs` the Ethernet
backend, including its concrete socket adapter and pcap I/O through `src/pcap.rs`. This document
is the model; the operator's view — which flag opens which socket, how to drive a peer — is
[Networking and host I/O](../guide/networking-and-host-io.md).

## Where the modules sit

`Emac::new(base, wrapper_control)` builds one port. `base` is subtracted on the way in and every
register constant in `src/emac.rs` stays written as a C6455 address, so the same model serves a
C6455's only MAC at `0x02C8_0000` and a C6472's EMAC0 and EMAC1 at `0x02C8_0000` and
`0x02CC_0000`. The bases come from the device profile (`emac_bases` and `emic_bases` in
`src/device.rs`), so a device difference is a profile field rather than a branch.
`wrapper_control` says which interrupt gate the device has: the C6455's `EWCTL` word inside the
EMAC control module, or the C6472's EMIC, which owns that same address and is a separate slave
port.

Both are ordinary slave ports (`SlavePort for Emac`, `SlavePort for Mdio`), so the CPU, the EDMA3
and the HPI all reach them through the router. The MAC is also a DMA master: `Emac::tick` takes
`&mut impl DmaMemory`, and the fabric lends the MAC out for its cycle
(`Fabric::lend_emac`) so it can read and write descriptor memory — including another
peripheral's registers — while the rest of the device stays reachable. See
[Architecture](architecture.md) for the lending protocol and the tick model.

`Emac::active()` is what keeps a dormant MAC out of the tick: it answers true only while a frame
is queued in the receive FIFO, a transmit channel has work, a teardown is outstanding, or either
pause timer is running. Programmed receive descriptors alone are not activity — nothing happens
until a frame arrives. `Mdio::active()` is true only while a management transaction is in
flight. Both sit behind bits of the fabric's activity summary (`ACTIVE_EMAC`, `ACTIVE_MDIO` in
`src/system.rs`).

## Register map

Offsets are from the port's base. Eight-word blocks are indexed by channel; `channel_register`
decodes them, `statistics_register` decodes the statistics block.

| Offset | Register | Behaviour in the model |
|---|---|---|
| `0x0000`, `0x0010` | TXIDVER, RXIDVER | both read `0x000C_0A08`; writes ignored |
| `0x0004`, `0x0014` | TXCONTROL, RXCONTROL | bit 0 only: the DMA enables |
| `0x0008`, `0x0018` | TXTEARDOWN, RXTEARDOWN | write channel in bits 2-0; read `0` |
| `0x0080`-`0x008C` | TXINTSTATRAW/MASKED, TXINTMASKSET/CLEAR | eight bits, one per channel |
| `0x0090` | MACINVECTOR | composed by the router, not by the port |
| `0x00A0`-`0x00AC` | RXINTSTAT/MASK | eight bits, one per channel |
| `0x00B0`-`0x00BC` | MACINTSTAT/MASK | two bits: STATPEND, HOSTPEND |
| `0x0100` | RXMBPENABLE | stored whole; the honoured bits are below |
| `0x0104`, `0x0108` | RXUNICASTSET, RXUNICASTCLEAR | eight-bit set/clear pair |
| `0x010C` | RXMAXLEN | 16 bits, reset `1518` |
| `0x0110` | RXBUFFEROFFSET | 16 bits |
| `0x0114` | RXFILTERLOWTHRESH | 8 bits |
| `0x0120`+4n | RXFLOWTHRESH*n* | 8 bits per channel |
| `0x0140`+4n | RXFREEBUFFER*n* | a write **adds** to the count, modulo 65536 |
| `0x0160` | MACCONTROL | stored whole; honoured bits below |
| `0x0164` | MACSTATUS | composed; reading clears both error-channel fields |
| `0x016C` | FIFOCONTROL | keeps `0x007F_001F`; reset `0x0002_0018` |
| `0x0174` | SOFTRESET | reads `0`; a write with bit 0 resets every register |
| `0x01D0`, `0x01D4` | MACSRCADDRLO/HI | the six source bytes of a pause frame |
| `0x01D8`, `0x01DC` | MACHASH1, MACHASH2 | the 64 multicast hash buckets |
| `0x01E8`, `0x01EC` | RXPAUSE, TXPAUSE | read the two pause timers; writes ignored |
| `0x0200`-`0x028C` | statistics | see [Statistics](#statistics) |
| `0x0500`, `0x0504` | MACADDRLO/HI | the address RAM entry MACINDEX selects |
| `0x0508` | MACINDEX | 5 bits: 32 entries |
| `0x0600`+4n | TXHDP*n* | a write also arms or disarms the channel |
| `0x0620`+4n | RXHDP*n* | |
| `0x0640`+4n, `0x0660`+4n | TXCP*n*, RXCP*n* | the acknowledge half of the handshake |
| `0x1004` | EWCTL | on the C6455 only; the C6472's EMIC owns this address |

MACSTATUS packs the transmit error code in bits 23-20 and its channel in 18-16, the receive error
code in 15-12 and its channel in 10-8, then RXQOSACT, RXFLOWACT and TXFLOWACT in bits 2, 1 and 0.
The two channel fields read as zero once the host has seen them; the codes stay until a reset.

The honoured MACCONTROL bits are FULLDUPLEX (0), RXBUFFERFLOWEN (3), TXFLOWEN (4), GMIIEN (5),
TXPTYPE (9) and RXFIFOFLOWEN (12). GMIIEN is the outer gate on the wire — with it clear no frame
is taken off the wire, none leaves, and no pause frame goes out — and the DMA enables TXCONTROL
and RXCONTROL are the inner gate, which is the order TI's own initialization sequence writes them
in. The honoured RXMBPENABLE bits are RXMULTEN (5), RXBROADEN (13), RXCAFEN (21), RXCEFEN (22),
RXCSFEN (23), RXCMFEN (24), RXNOCHAIN (28) and RXQOSEN (29), plus the three channel fields
RXMULTCH (2-0), RXBROADCH (10-8) and RXPROMCH (18-16), read by `mbp_channel`.

There are eight receive and eight transmit channels (`CHANNELS`), each with a head descriptor
pointer, a completion pointer, a free-buffer count and a flow threshold.

## The CPPI descriptor

A descriptor is four words. `DmaMemory::dma_read_u32` and `dma_write_u32` are little-endian, so
each word is read and written as the CPU stored it.

| Word | Field | Transmit | Receive |
|---|---|---|---|
| 0 | next descriptor | walked once per packet | walked once per packet |
| 1 | buffer pointer | `0` is host error 4h | `0` is host error 4h |
| 2 | offset (31-16) / length (15-0) | offset honoured on the SOP descriptor only, length is bytes to send | length read as the buffer's capacity, written back as bytes filled |
| 3 | flags (31-16) / packet length (15-0) | packet length read from the SOP descriptor | packet length written into the SOP descriptor |

The flags:

| Bit | Name | Meaning in the model |
|---|---|---|
| 31 | SOP | first descriptor of a packet |
| 30 | EOP | last descriptor of a packet |
| 29 | OWNER | set by software when it hands a descriptor over; the DMA clears it |
| 28 | EOQ | the DMA reached a null next pointer and stopped |
| 27 | TDOWNCMPLT | this descriptor was reached by a teardown |
| 25-16 | JABBER, OVERSIZE, FRAGMENT, UNDERSIZED, CONTROL, OVERRUN, CODEERROR, ALIGNERROR, CRCERROR, NOMATCH | receive status, written into the SOP descriptor |

Bit 26, PASSCRC, is not modelled: frames in this model carry no frame check sequence, so there is
no CRC to pass on. The wire errors a frame can arrive with instead come in with it —
`RX_ERROR_CRC`, `RX_ERROR_ALIGN` and `RX_ERROR_CODE`, injected by the backend or by a conformance
vector, because nothing inside the module can produce them.

A transmit packet may span descriptors: `process_tx` walks from `TXHDP`*n*, concatenating each
buffer's `length` bytes, until it finds EOP, then truncates the result to the packet length the
SOP descriptor declared. A receive packet fills as many descriptors as it needs: each takes as
much as its `length` field says it holds, the first gets SOP, the status flags, RXBUFFEROFFSET in
its word 2 and the whole packet's length in word 3, and the last gets EOP. `a_packet_spans_one_descriptor_per_fragment`
in `src/emac.rs` pins a 20-byte frame across three 8-byte buffers, two bytes into the first.
With RXNOCHAIN set the model uses one buffer per frame, discards whatever does not fit, and
reports the whole frame's byte count in the buffer-length field.

## The queue handshake and the EOQ race

Software writes the head of a chain to `TXHDP`*n* or `RXHDP`*n*; the DMA processes descriptors and
writes the address of the last one it finished to `TXCP`*n* or `RXCP`*n*. `tx_raw` and `rx_raw`
build the raw interrupt status by comparing, per channel, the completion pointer the DMA wrote
against the value software last wrote to CP: bit *n* stands while they differ. That is the whole
acknowledge protocol, and it means writing a stale pointer leaves the interrupt asserted —
`the_raw_interrupt_status_names_the_channel` pins both halves.

The EOQ race is the queue's one sharp edge, and the model takes the losing side of it every time.
The DMA reads word 0 of a descriptor exactly once. If that read returns null, the packet ends
there: `process_tx` writes back `(flags & !OWNER) | EOQ`, clears `tx_hdp[channel]` and drops the
channel out of `tx_active`; the receive walk sets EOQ on the descriptor it finished on when
`next_after == 0` and leaves `rx_hdp[channel]` at zero. Nothing re-reads the pointer, so a
descriptor software links onto the tail afterwards is never noticed. The channel is stopped until
software writes HDP again — and on the receive side the frames that arrive while it is stopped are
not lost: `process_rx` returns before `pop_received()` when `rx_hdp[channel] == 0` or the head
descriptor's OWNER bit is clear, so they wait in the receive FIFO and are delivered whole when the
queue restarts. `tests/programs/ethernet_queue_stall` is that sequence as a firmware program, and
both drivers in the tree (`tests/firmware/ndk_tcp/nimu_emac.c` and
`tests/firmware/c6472_netecho/main.c`) restart the ring on EOQ because they have to.

## Receive filtering

`receive_channel` decides which of the eight receive channels a frame belongs to, from its first
six bytes, and returns `None` for a frame the address logic rejects. In order:

1. Any address RAM entry that is VALID with MATCHFILT clear and whose six bytes equal the
   destination rejects the frame outright — even in promiscuous mode.
2. An entry that is VALID with MATCHFILT set and matches names the channel in its CHANNEL field,
   provided that channel's bit is set in RXUNICASTSET.
3. A broadcast destination goes to RXBROADCH if RXBROADEN is set.
4. Any other group destination (bit 0 of byte 0 set) goes to RXMULTCH if RXMULTEN is set **and**
   its hash bucket is set in MACHASH1/MACHASH2.
5. Whatever is left goes to RXPROMCH if RXCAFEN is set, carrying the NOMATCH status flag.

An address RAM entry is six bytes plus the control bits of MACADDRLO: CHANNEL in 18-16, MATCHFILT
in 19, VALID in 20 (`AddressSlot`, mask `0x001F_0000`). MACINDEX selects one of 32 entries;
For wire-order bytes `w[0..6]`, MACADDRLO bits 15:8 hold `w[5]` and bits 7:0 hold `w[4]`;
MACADDRHI holds `w[3]`, `w[2]`, `w[1]`, `w[0]` from most to least significant byte
(SPRU975E Tables 71/72). MACSRCADDRLO/HI use the same address layout. The C6472 fused MAC ID
instead holds `w[0..4]` most significant first in MACID0 and `w[4]`, `w[5]` in MACID1
bits 31:24 and 23:16, as read by TI IBL ed3f098 `c6472.c::deviceLoadDefaultEthAddress`.

`address_hash` folds the 48-bit destination into six bits: bit *i* is the exclusive-or of address
bits *i*, *i*+6, … *i*+42. (The comment above the function says every eighth bit; the code and
`the_address_hash_folds_the_destination_into_six_bits` both step by six — the test pins 17, 27 and
0 for three concrete addresses.) The result indexes 64 buckets across the two hash words.

## Frame classification and the copy enables

`classify` looks only at length, in wire bytes — the frame data plus `CRC_LENGTH` — and answers
`Proper`, `Undersized` below 64, or `Oversized` above a non-zero RXMAXLEN. Class does not affect
routing; it decides which copy enable has to be set for the frame to reach memory at all, and
which status bit and which statistic it carries:

| Class | With wire errors | Copy enable required | SOP status |
|---|---|---|---|
| short | no | RXCSFEN | UNDERSIZED |
| short | yes | RXCSFEN **and** RXCEFEN | FRAGMENT |
| long | no | RXCEFEN | OVERSIZE |
| long | yes | RXCEFEN | JABBER |
| proper | yes | RXCEFEN | CRCERROR / ALIGNERROR / CODEERROR |
| MAC control frame | — | RXCMFEN | CONTROL |

A long frame that is copied is truncated to RXMAXLEN wire bytes. A frame whose enable is missing
is dropped after its class statistic is counted, which is what `record_class` is for: an overrun
or a missing enable changes nothing about which counter a frame lands in.

A MAC control frame is acted on whether or not it is copied. `is_control_frame` requires a
proper-length frame of at least 18 bytes, the reserved destination `01:80:C2:00:00:01` or an
address RAM match, and `88 08 00 01` at bytes 12 to 15. `process_pause_frame` runs before the DMA
each tick: it counts the frame, counts RXPAUSEFRAMES, loads TXPAUSE from bytes 16-17 when
MACCONTROL has both FULLDUPLEX and TXFLOWEN, and then either drops the frame or sets
`control_acted` so RXCMFEN can still copy it without the pause action happening twice. A pause
frame that arrived with any wire error is not a pause frame: neither the statistic nor the action
applies.

## Receive QOS

With RXQOSEN set, a proper frame whose channel has `RXFREEBUFFER`*n* at or below
RXFILTERLOWTHRESH is dropped and counted in RXQOSFILTERED — unless it is high priority.
`low_priority` reads that from the frame: anything shorter than 16 bytes, anything whose bytes 12
and 13 are not `81 00`, and any VLAN tag whose three priority bits are below 4. QOS never looks at
a short or long frame. MACSTATUS bit 2 (`qos_active`) stands while QOS is enabled and any enabled
channel is at or below the threshold.

## Flow control and pause frames

`flow_triggered` has two independent sources:

- **Buffer flow.** MACCONTROL.RXBUFFERFLOWEN, and any enabled channel whose `RXFREEBUFFER`*n* is
  at or below its `RXFLOWTHRESH`*n*.
- **FIFO flow.** MACCONTROL.RXFIFOFLOWEN together with FULLDUPLEX, a FIFOCONTROL threshold field
  (bits 22-16) in 1 to 66, and a receive FIFO occupancy at or above it.

`update_flow_control` runs on every register write and twice per tick. On a transition it updates
`rx_flow_active`, which MACSTATUS reports in bit 1, and in full duplex it transmits a pause frame:
destination `01:80:C2:00:00:01`, MACSRCADDR as source, `88 08` `00 01`, the pause time, padded to
60 bytes, counted in TXPAUSEFRAMES and pushed onto the transmitted queue like any other frame.
A pause of `0xFFFF` on assertion, a pause of `0` on release, and RXPAUSE is loaded with `0xFF00`
so the tick counts it down and re-sends the pause when it expires with flow control still active.
Half duplex is a state change without a frame: back-pressure by collision is not modelled.

## FIFO and DMA overruns

The receive FIFO is 68 cells of 64 bytes (`RX_FIFO_CELLS`), and a frame occupies
`ceil(len / 64)` cells, at least one. `inject_frame_with_errors` refuses a frame that does not
fit and counts the refusal: RXSOFOVERRUNS when no cell was free at all, so reception never
started, and RXMOFOVERRUNS when there was room for part of the frame. Running out of *descriptors*
mid-frame is a different fault: it counts RXDMAOVERRUNS, and only RXCEFEN hands the partial frame
over — with the OVERRUN status bit set in its SOP descriptor. Without RXCEFEN the frame is
dropped and the descriptors it began to fill stay owned by the EMAC.

`rx_fifo_has_room` exists for the host backends. A Unix socket has no rate of its own, so the run
loop asks before delivering rather than letting a frame the emulator was slow to read overrun a
FIFO that a real line could never have filled that fast.

## Transmit arbitration

One packet leaves per tick. MACCONTROL.TXPTYPE selects the arbitration: fixed priority takes the
highest-numbered channel with work (channel 7 wins), round robin resumes from `tx_cursor` and
advances it past the channel it served. A channel has work if `TXHDP`*n* is non-zero or a teardown
is pending on it. After the selection, `process_tx` checks in order: the DMA enable and the host
error latch, the teardown, GMIIEN, the TXPAUSE timer, and only then the descriptor.

## Statistics

`statistics_register` claims every word from `0x0200` to `0x028C`. Reads answer the counter's
value, zero if it was never touched. Writes behave two different ways, which
`statistics_are_write_to_decrement_once_gmiien_is_set` pins: with MACCONTROL.GMIIEN clear a
statistics register is an ordinary read/write location, and with it set the written value is
*subtracted*, saturating at zero. That is how a driver clears a counter it has just read without
losing the events that arrived in between.

Counted on receive: RXGOODFRAMES, RXBCASTFRAMES, RXMCASTFRAMES, RXPAUSEFRAMES, RXCRCERRORS,
RXALIGNCODEERRORS, RXOVERSIZED, RXJABBER, RXUNDERSIZED, RXFRAGMENTS, RXFILTERED, RXQOSFILTERED,
RXOCTETS, the three overrun counters, and one of the six frame-size buckets FRAME64 through
FRAME1024TUP. Counted on transmit: TXGOODFRAMES, TXBCASTFRAMES, TXMCASTFRAMES, TXPAUSEFRAMES,
TXOCTETS and the same size bucket. NETOCTETS counts both directions. Octet counts and size
buckets use the wire length, four bytes more than the frame data the model carries.

Only a data frame the address logic rejected counts as RXFILTERED; a fragment is counted as a
fragment whether or not any address matched it. Alignment and code errors outrank a CRC error,
which the manual defines as excluding them. The half-duplex counters that sit in the same address
range — deferrals, collisions, carrier-sense and excessive-collision errors — are readable and
writable locations that nothing ever increments: this model has no half-duplex collision
behaviour to count.

## Teardown and host errors

Writing a channel number to TXTEARDOWN or RXTEARDOWN sets a bit in a pending mask. The next tick
clears the head pointer, and, if it was non-zero, marks the descriptor it pointed at with OWNER
clear and TDOWNCMPLT set, then reports `0xFFFF_FFFC` (`TEARDOWN_MARKER`) in the channel's
completion pointer — which raises the channel's interrupt, because it differs from whatever
software last acknowledged. Software acknowledges a teardown by writing that same marker.
Teardown is a DMA command and completes even with GMIIEN clear.

A host error is a descriptor the DMA cannot use. `raise_tx_error` and `raise_rx_error` latch the
first code and its channel and leave them latched: `host_error()` then stops both DMA engines
until a soft reset, and HOSTPEND stands in MACINTSTATRAW.

| Code | Direction | Cause |
|---|---|---|
| 1h | transmit | the head descriptor of a packet has no SOP |
| 2h | transmit | the head descriptor's OWNER bit is clear |
| 3h | transmit | the chain ended without an EOP |
| 4h | transmit, receive | a descriptor with a null buffer pointer |
| 5h | transmit | a descriptor with a zero buffer length |

A zero *packet* length is not reported, because the hardware does not detect it either.

## Interrupts

`update_interrupt` recomputes one line after every register write and at the end of every tick:
`interrupt_line` is set when EWCTL bit 0 is set and any of the three masked sources is asserted —
`tx_raw & tx_mask`, `rx_raw & rx_mask`, or `mac_int_raw & mac_int_mask`. MACINT is a level, not a
pulse: it stays asserted until software writes the completion pointer of the descriptor the
hardware finished, so the megamodule keeps latching the event for as long as the work is
unacknowledged. On the C6455 that single line is system event 17 for every core
(`C6455_EVENTS.emac[0].common` in `src/events.rs`), and the fabric raises it from
`latch_emac_interrupts` and replays it through the idle level-event set so a dormant device does
not lose it. [Interrupts and exceptions](interrupts.md) has the megamodule side.

The two MACINT sources are STATPEND (bit 0) and HOSTPEND (bit 1). STATPEND stands while any
statistics register has its most significant bit set — the half-full level, not an edge — and
HOSTPEND while a host error is latched.

MACINVECTOR at offset `0x90` is the one register no port answers. It sits in the EMAC's map but
reports the MDIO's interrupts too, and only the control module sees both, so the router decodes it
itself (`Fabric::mac_input_vector`): the eight transmit bits in 7-0, the eight receive bits in
15-8, STATPEND in bit 16, HOSTPEND in bit 17 from `Emac::input_vector`, then the MDIO's link
change in bit 30 and its user-access completion in bit 31. On a C6472 there are two of these and
one MDIO, and each vector carries that MAC's own interrupts with the shared MDIO's.

## The C6472 interrupt-control module

A C6472 replaces the C6455's single EWCTL gate with one EMIC per MAC, at `0x02C8_1000` and
`0x02CC_1000`, 2 KB each. `src/emic.rs` models the three blocks the guide describes: a prescaler
(PSCFG at offset `4`), sixteen pacing blocks (RPCFG/RPSTAT at `0x100`/`0x120`, TPCFG/TPSTAT at
`0x140`/`0x160`, eight receive and eight transmit), and a common interrupt combiner. `EW_INTCTL0`
through `EW_INTCTL5` at `0x20` are the per-core enables — receive channels in bits 23-16,
transmit in 15-8, then MDIO_USER, MDIO_LINT, STAT and HOST in bits 4 to 1 (mask `0x00FF_FF1E`).
The pacer configuration is shared by all six cores; only the enable downstream of it is per core.

Each pacing block is a timed-delay state machine and a divide-by-N state machine, mutually
exclusive: a non-zero TIME_CFG holds the first event off and emits a pulse TIME_CFG prescaler
periods later, absorbing further events meanwhile; with TIME_CFG zero, a non-zero CNT_CFG emits
one pulse per CNT_CFG events; both zero passes the event straight through. TIME_CFG and CNT_CFG
are key-protected — a write only lands if the TU or CU bit accompanies it — and TR and CR reset
the two counters. RPSTAT/TPSTAT report TIME, CNT and the two state-machine states, and are read
only.

The seam is `Sources` in and `Lines` out. `latch_emac_interrupts` reads `Emac::masked_sources()`
— the eight masked receive bits, the eight masked transmit bits, STATPEND and HOSTPEND, all after
the MAC's own masks — adds the shared MDIO's two bits, and calls `update`. Receive and transmit
are consumed as pulses: `update` takes the rising edge of each bit into that channel's pacer.
The four common sources are levels, and the combiner converts the *gated* level to a pulse, which
is what makes the documented idiom work — clearing and re-setting an EW_INTCTL bit inside a
service routine re-triggers MACINT while the source is still asserted. Pacing does not apply to
the common group. `Lines` is three one-cycle pulses per core, raised into that core's megamodule
as events 5/6/70 for EMAC0 and 7/8/71 for EMAC1. The prescaler and every timed pacer advance from
`tick_device_blocks`, once per device cycle.

This is what lets six cores share one Ethernet connection: one core owns the descriptor ring, and
the receive interrupt follows whichever core enabled that channel. [The six-core
device](../guide/multicore.md) has the operator's version.

## MDIO and the PHY

One MDIO module per device, at `0x02C8_1800`, shared by both MACs on a C6472. MDIOVER reads
`0x0007_0102` on a C6455 and `0x0007_0103` on a C6472, from the device profile. CONTROL bit 30 is
the enable; ALIVE and LINK answer a single bit for PHY address 0 and read as zero while the module
is disabled. LINKINTRAW latches a link change and is cleared by writing the bit back; USERINTRAW
latches a completed transaction per channel and is masked by USERINTMASKSET/CLEAR.

USERACCESS0 and USERACCESS1 carry clause-22 transactions: GO in bit 31, WRITE in 30, ACK in 29,
the register number in 25-21, the PHY address in 20-16 and the data in 15-0. A transaction does
not complete instantly — the driver is expected to poll GO — so `start` records the result and
`tick` delivers it `TRANSACTION_CYCLES` (16) cycles later, setting the user interrupt. A
transaction to any address other than 0, or to a disabled module, completes with the whole
register reading zero: no PHY answered, so there is no acknowledge.

The PHY behind it is a plausible gigabit part with auto-negotiation already complete: BMCR
`0x1140`, BMSR advertising link up and negotiation complete, PHYID `0x0141`/`0x0CC2`, ANAR and
ANLPAR `0x01E1`, GBCR `0x0300`, GBSR `0x0C00`. Status and identifier registers ignore writes; a
BMCR reset returns BMCR and BMSR to their defaults, and a restart of auto-negotiation completes
immediately. Registers 13 and 14 are the clause-45 indirect pair, with the device address in bits
4-0 of register 13, the function in bits 15-14, and post-increment on function 2 (reads and
writes) or 3 (writes only); the MMD map holds a PMA/PMD link status and identity for the same
part. There is no auto-negotiation timing, no link partner to negotiate with, and no MDIO clock:
the divider in CONTROL is stored and never consulted.

## How a frame crosses to the host

The Ethernet backend, `Ethernet` in `src/netio.rs`, owns host-side frame lifetimes. The machine
supplies its clock and asks for polling, inspection or finish; the fabric retains the MAC's
descriptor, DMA and interrupt behavior. The concrete Unix-datagram `Bridge` is private to the
backend, not a second interface front ends must coordinate.

Transmit. `process_tx` pushes the assembled packet onto the MAC's `transmitted` vector.
`Ethernet::collect` drains every MAC through `Fabric::take_tx_frames`, retains the complete
archive and records each frame into the separate bounded history the dashboard reads.
Inspection does not consume the archive or advance the send cursor. Polling and finish share
one handoff path, so repeated state dumps or finish calls do not resend frames. `pcap::write`
writes the complete archive at finish, regardless of live-history eviction.

Receive. The socket adapter reads every waiting datagram, dropping anything shorter than an
Ethernet header. The backend retains those frames until `Fabric::emac_rx_has_room` says the
selected MAC's FIFO can take the next one, then `inject_frame_on` delivers it. `--pcap-in`
frames are injected at load instead, all at once. Enabling history records that preload
without injecting it again.

The run loop polls on a cycle boundary, not from a thread. `Ethernet::service` does nothing
until its `PollSchedule` is due, then advances it by `--net-poll` cycles (default 1024). The
machine uses `poll_delay` to clip each batch to the next poll without inspecting socket or
queue state; the HPI host link clips with its own `PollSchedule` the same way. The clock
remains the focused core's cycle count, or the device's while a ROM boot holds core 0. One
poll drains the socket, delivers what fits, collects transmitted frames, sends them, and
flushes anything a full peer socket refused earlier. `--net-port` selects the MAC.

Reload first collects what the old fabric transmitted, then preserves the host backlog, socket
and complete TX archive, while resetting the poll deadline and live-history sequence and
preloading the `--pcap-in` frames into the rebuilt fabric.

## What the suite pins

The unit tests in `src/emac.rs` and `src/mdio.rs` cover the pieces reachable without firmware:

```
$ cargo nextest run -E 'test(emac::tests) + test(mdio::tests)'
    Starting 10 tests across 1 binary (670 tests skipped)
        PASS [   0.006s] ( 1/10) c64xemu::bin/c64xemu emac::tests::statistics_are_write_to_decrement_once_gmiien_is_set
        PASS [   0.006s] ( 2/10) c64xemu::bin/c64xemu mdio::tests::no_phy_answers_when_the_module_is_disabled_or_the_address_is_wrong
        ...
        PASS [   0.009s] (10/10) c64xemu::bin/c64xemu emac::tests::each_class_of_frame_goes_to_the_channel_its_rules_name
     Summary [   0.010s] 10 tests run: 10 passed, 670 skipped
```

Eleven firmware programs under `tests/programs/ethernet_*` drive the module from C through its
registers — channels, filtering, fragments, the queue stall, DMA and external coherence, the
shared MACINT, MDIO — each with an expectation derived from the manual rather than from the
emulator. Above them, `tests/test_network.py` holds real TCP connections with TI's NDK stack
running as firmware. Every frame in that suite crosses the model described here:

```
$ ./target/release/c64xemu run tests/firmware/ndk_tcp/build/ndk_tcp.out \
    --max-cycles 4000000 --pcap-out /tmp/ndk-tx.pcap --state-out /tmp/ndk.json
c64xemu ndk firmware starting
Service Status: DHCPC    : Enabled  :          : 000
...
echo: listening on port 3000
udp: listening on port 3001
nocopy: listening on port 3002
c64xemu: cycle limit 4000000 reached at PC 0x00835a38
$ jq -r '.tx_frames | length as $n | "frames: \($n)", (.[] | "\(. | length / 2 | floor) bytes  dst=\(.[0:12]) src=\(.[12:24]) type=\(.[24:28])")' /tmp/ndk.json
frames: 1
590 bytes  dst=ffffffffffff src=020000112233 type=0800
```

That one frame is the stack's DHCP discover, transmitted from channel 0 and drained by
`take_tx_frames` at exit. With a peer on the other side of the socket the whole exchange runs:

```
$ python tests/tools/tcp_peer.py --emulator ./target/release/c64xemu \
    --image tests/firmware/ndk_tcp/build/ndk_tcp.out --report --timeout 120
{"state": "closed", "echoed": 28, "payload_matched": true, ... "dhcp": ["discover", "offer",
"request", "ack"], "firmware": ["c64xemu ndk firmware starting", ...,
"echo: connection from 192.168.64.1:40000", "echo: connection closed after 28 bytes"]}
```

[Running the tests](../guide/testing.md) explains how these tiers relate; [Running the
tests](../guide/testing.md) is how to run them.
