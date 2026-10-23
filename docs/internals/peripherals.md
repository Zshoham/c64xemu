# Peripherals

The slave-port seam every peripheral answers, and each modelled register file behind it: EDMA3
and IDMA, the timers, GPIO, I2C, HPI, UTOPIA, the McBSPs, the PLL controllers, DDR2, EMIFA,
device configuration, and the C6472-only blocks. Ethernet has its own document,
[Ethernet](ethernet.md).

## The seam

`src/port.rs` is thirty lines and the whole contract:

```rust
pub(crate) trait SlavePort {
    fn read(&mut self, address: u32, context: PortContext) -> Option<u32>;
    fn write(&mut self, address: u32, value: u32, context: PortContext) -> PortWrite;
}
```

A port answers `None` or `PortWrite::Unhandled` for an address it does not own, so the router in
`src/system.rs` is a list of ports rather than twenty signatures that agree by convention. The
router is the single point both master kinds — the CPU and the DMA engines — cross to reach a
peripheral.

`PortContext` carries what a port needs but does not own. Today that is only the endianness:
three ports report their registers byte-swapped when the device is strapped big-endian, and
`Fabric::set_endian` makes it runtime state, so a port cannot cache it.

`PortWrite` is the write's outcome: `Unhandled`, `Done`, or `Event(usize)` — a completion event
the router must latch, which is IDMA's channels. Interrupt latching itself stays in the router,
because a follow-up the megamodules are owed is not part of a port's shape.

## EDMA3

`src/edma.rs` is the channel controller at `02A0 0000h`, with the PaRAM at `02A0 4000h`;
`src/edma_tc.rs` is the four transfer controllers. The reference is
[SPRU966C](../reference/spru966c.pdf) for the C6455 and
[SPRU727E](../reference/spru727e.pdf) for the C6472.

It is a RAM-based transfer engine: every channel names a 32-byte PaRAM set holding a source, a
destination, three counts, four indexes and a link address. A trigger submits one transfer
request per synchronisation event, and the channel controller updates the set between requests.
The model performs one transfer request per tick, which is what makes a driver's "wait for the
completion interrupt" loop behave the way it does on hardware rather than completing
instantaneously.

The shadow regions are at `SHADOW_STRIDE` `0x200` apart with a `SHADOW_LIMIT` of `0x98`, which is
the register window a region may reach; the access enables decide which channels a region owns.

Which peripheral event drives which channel is not in this module — it is the device event map in
`src/events.rs`, tabulated in [Interrupts and exceptions](interrupts.md). On the C6455 the timers
drive channels 1, 2, 16 and 17, GPIO 48 to 63, the I2C 44 and 45, the McBSPs 12 to 15 and the
UTOPIA 32 and 40; the C6472 renumbers all of them.

The transfer controllers identify themselves, report the FIFO geometry they were built with, and
expose channel status and error registers with their own error interrupts (events 81-84 on the
C6455, 59-62 on the C6472). Their source-active and destination-FIFO register sets are read-only
debug windows — SPRU966C §2.12.4 calls them a way "to derive a brief history of TRs serviced
through the transfer controller", which is what they hold here. Because the model completes a
transfer request inside the tick that submits it, the activity fields PROGBUSY, SRCACTV, WSACTV
and DSTACTV always read idle. That is not a gap: SRCACTV idle is documented as meaning "source
active register set contains a previously processed transfer request".

## IDMA

`src/idma.rs` is the two megamodule internal DMA channels of
[SPRU871K](../reference/spru871k.pdf) chapter 5, at `0182 0000h`. Channel 0 moves configuration
registers, channel 1 moves blocks between local memories. IDMA is not routed like a DMA master:
it lives inside the megamodule, so it is reached through the megamodule's own configuration
space, and its completion is the `PortWrite::Event` case above — the port raises the event and
the router latches it. Its interaction with the caches is in
[Memory and caches](memory-and-caches.md).

## Timers

`src/timer.rs`, [SPRU968](../reference/spru968.pdf). Each timer is a pair of 32-bit counter and
period registers that work as one 64-bit up-counter, as two independent 32-bit timers — one
behind a four-bit prescaler — or as a watchdog. Reaching the period raises the timer's CPU
interrupt and its EDMA3 event, and drives the timer output as a pulse or a square wave.

The clock is SYSCLK3, the CPU clock divided by six ([SPRS276M](../reference/sprs276m.pdf) §7.6),
which is the same clock GPIO, the I2C and the UTOPIA run on. A C6472 megamodule's local timer is
also gated by that megamodule's PSC module ([SPRUEG3B](../reference/sprueg3b.pdf) §4.1).

The timer is not stepped on every edge: the SYSCLK3 schedule (see
[Architecture](architecture.md)) steps it only on the edge of its next period match, which is
the only edge a timer raises anything on, and derives everything in between - counters,
prescaler positions, pulse countdowns, TSTAT, one-shot stops - when an access or that edge
catches it up. The derivation follows the per-edge model to its quirks: a 32-bit half compares
the incremented count before truncating it, so wrapping to a zero period does not match and the
half then sits at its period; an unchained TDDRHI written above PSCHI counts up past 15 and
carries into TGCR bit 16; a one-shot half that stops freezes its prescaler with it; and a
watchdog timeout forced by a bad key is due on the very next edge.

| Device | Timers | Bases |
|---|---|---|
| C6455 | 2 | `0294 0000h`, `0298 0000h` (SPRS276M Table 2-3) |
| C6472 | 12 | `025E 0000h` on 64 KB boundaries; timers 0-5 belong to megamodules 0-5, timers 6-11 are shared ([SPRS612G](../reference/sprs612g.pdf) §7.16.2) |

## GPIO

`src/gpio.rs` at `02B0 0000h`, [SPRU724A](../reference/spru724a.pdf). Sixteen pins, each an input
or an output, with edge detection per pin raising a CPU interrupt and an EDMA3 event. The edge
detector runs on the peripheral clock (CPU/6, SPRU724A Figure 2) and sees output pins too, so
firmware toggling OUT_DATA generates events exactly as an external signal would — which is what
makes GPIO usable as a stimulus source in a test with no board attached.

The detector only does something on an edge where the pins differ from what it last sampled, so
the SYSCLK3 schedule steps it only on the edge after an access - a register write, or an external
driver on the inputs - changed them.

## I2C

`src/i2c.rs` at `02B0 4000h`, [SPRU974A](../reference/spru974a.pdf). A master/slave controller:
ICDXR feeding an internal shift register, ICDRR filled from it, a write-one-to-clear status
register, a mask register, and a vector register that reports and clears the highest-priority
pending interrupt.

Digital loopback is the one data path that is self-contained, and the model implements exactly
what the manual specifies for it: data written to ICDXR arrives in ICDRR after
(prescaler + 1) × 8 DSP cycles by an internal path. The same register file is what the ROM's I2C
EEPROM boot drives — see [Booting an image](../guide/booting.md).

## HPI

`src/hpi.rs` at `0288 0000h`, [SPRU969B](../reference/spru969b.pdf). Both sides are modelled. The
host owns HPIC, HPID and the two address registers and reaches DSP memory through the HPI's DMA
logic; the CPU has read-only access to almost all of it and exactly three things it may write —
the software reset that clears the FIFOs, the HINT bit that interrupts the host, and a one to
DSPINT that acknowledges an interrupt from the host.

There is no pin interface: a host access names one of the four types the HCNTL pins select
(SPRU969B Table 4) and the model performs it at once. What *is* modelled is everything the
protocol makes observable — word-addressed HPIA with autoincrement, single and dual HPIA mode,
the eight-word FIFOs with their four-word bursts and documented flush conditions, and both
interrupt handshakes. DSPINT is system event 15 on the C6455 and 85 on the C6472. The host side
of the socket is in [Networking and host I/O](../guide/networking-and-host-io.md).

## UTOPIA

`src/utopia.rs` at `02B4 0000h`, [SPRUE48](../reference/sprue48.pdf). A UTOPIA 2 ATM slave: UCR
configures the port, CDR watches for the master's clocks, and the EIER/EIPR pair reports errors
behind CPU interrupt UINT. Transmit and receive queues sit in their own data-port windows and
drive the UXEVT and UREVT EDMA3 events on the C6455.

The clock-detect bits say what the absence of a master means: nothing drives UXCLK or URCLK, so
RCPP and XCPP stay clear and an enabled section with a non-zero clock count reaches the
clock-failure condition after that many peripheral clocks (SPRUE48 Table 8). The cell path is
modelled against an external ATM master pulling cells out of the transmit queue.

That master reaches the slave through the UTOPIA Level 2 address phase (SPRUE48 sections 3.1 and
3.7). With UCR.MPHY set, the reset state, the slave decodes the master's PHY address against
UCR.SLID: polled at that address it drives UXCLAV with whether a complete cell is queued and
URCLAV with whether a cell fits, and only a transfer selected at that address moves a cell. At
any other address, and in a section held in reset, the pins stay high-impedance and the cell
stays where it is - the master is talking to another PHY. SLID 1Fh is the null PHY no address
reaches, and the address a case leaves unnamed is 1Fh too, the level the pins are pulled to
(Table 4). With MPHY clear the bus is not decoded at all. The conformance harness drives this with
`utopia_master_poll`, `utopia_master_read` and `utopia_master_write`, each taking an optional
five-bit `address`.

Under the SYSCLK3 schedule the port is stepped on every edge while an enabled section's event or
UINT is asserted - the device tick treats an asserted UINT as movement on every edge - and
otherwise only on the edge a section's clock count runs out; its idle counters are derived in
between.

## McBSP

`src/mcbsp.rs`, C6455 only (`has_mcbsp`). The register file and bit layouts come from the CSL
register appendix ([SPRU401J](../reference/spru401j.pdf) Tables B-210 to B-224) and the addresses
from SPRS276M Tables 7-57 and 7-58; each port answers on its configuration bus, and DRR and DXR
answer again on the EDMA data bus.

Digital loopback is again the path that runs without a device on the pins: with SPCR.DLB set the
transmit serialiser feeds the receive serialiser, so a word written to DXR arrives in DRR a
word's worth of serial clocks later. The clock is the sample rate generator's CLKG, dividing the
CPU clock by CLKGDV + 1 with SRGR.CLKSM set, and SPCR.GRST holds the generator in reset until
software releases it. The double buffering is the documented DXR→XSR and RSR→DRR pair, with
XRDY, RRDY and RFULL following from it. The C6472 has three TSIPs instead, which are not modelled.

## PLL controllers

`src/pll.rs`: PLL1 at `029A 0000h` and PLL2 at `029C 0000h`,
[SPRUE56](../reference/sprue56.pdf) with the device register set in SPRS276M §7.7.3. A
multiplier, a pre-divider, the two programmable system-clock dividers this device exposes, and
the GO operation that moves a newly written ratio into effect: writing a different ratio flags
the divider in DCHANGE, and GO applies exactly those dividers and clears the flags. D2 and D3,
which clock the megamodule and the peripherals, are fixed at /3 and /6 and have no registers.

RSTYPE, the reset controller's only register, shares the PLL1 range and latches which reset last
happened; this model always starts from a power-on reset. PLL2 has a fixed ×20 multiplier and one
programmable divider, D1, which generates SYSCLK1 for the EMAC.

## DDR2 and EMIFA

`src/ddr2.rs` at `7800 0000h` ([SPRU970G](../reference/spru970g.pdf) §4) and `src/emifa.rs` at
`7000 0000h` ([SPRU971E](../reference/spru971e.pdf) §12, C6455 only — the C6472 has no
asynchronous EMIF, DDR2 is its only external memory).

Both are register contracts, not data paths: the memory behind them is plain backing store, and
DDR2 latency, refresh and EMIFA strobe timing are deliberately not simulated. What is modelled is
what firmware programs and reads back at boot — module identification, the endianness each
inherited, SDRAM configuration and timing with the locks that guard them, and the refresh-rate
floor the controller substitutes when software programs one that is too small. EMIFA adds the
burst priority, four chip-enable configuration registers, the asynchronous wait configuration and
the time-out interrupt; with no external device to hold ARDY the time-out has no injectable
source, but the mask registers and their write-one-to-clear pairing are exactly as documented.

## Device configuration

`src/devcfg.rs`, C6455 only: the chip-level block at `02A8 0000h` and the device-state block at
`02AC 0000h` (SPRS276M §3.4 to §3.6 and §4.2). DEVSTAT reports what the configuration pins
latched at reset, JTAGID identifies the silicon, PRI_ALLOC sets the bus priority of masters
without their own priority register, and PERLOCK/PERCFG0 turn peripherals on and off with
PERSTAT0/PERSTAT1 reporting the transition.

The PERLOCK window and the PERSTAT transition are SYSCLK3 countdowns that raise nothing, so the
SYSCLK3 schedule never steps this block: an access catches the countdowns up first.

Two device-level choices are documented rather than invented: the silicon is revision 3.1, the
last one [SPRZ234R](../reference/sprz234r.pdf) Table 3 lists, and the configuration pins are what
a board populating this memory map would strap — EMIFA and DDR2 enabled, everything else at its
internal-resistor default. LENDIAN is not a strap: it follows the endianness of the loaded image,
which is real state the model has.

## The C6472-only blocks

| Block | Base | What it does |
|---|---|---|
| Boot controller, `src/bootctl.rs` | `02AB 0000h`, 32K | RESET_STAT, the six set-only BOOT_COMPLETE_STAT bits whose 0→1 transition releases a core from reset, DSP_BOOT_ADDRn, and BOOTPROGRESS — a plain scratch word SPRS612G §3.9.3 gives no fields at all |
| Chip configuration, `src/chipcfg.rs` | `02A8 0000h`, 128K window | the latched pin configuration, switch-fabric priority, peripheral enables behind key locks (`DEVCTL_UNLOCK`, `PRIVKEY_UNLOCK`), the supervisor/user permission map, and IPCGRn/IPCARn/NMIGRn — the inter-processor interrupts, arriving as system event 84 in the target core |
| Power/sleep controller, `src/psc.rs` | `02AE 0000h` | a global PSC owning every register plus one local PSC per clock domain; software stages MDCTLn.NEXT, writes the domain's GO bit in PTCMD, PTSTAT.GOSTAT reads 1 in flight, MDSTATn.STATE reports what was reached ([SPRUEG3B](../reference/sprueg3b.pdf) §4.1.2 and §9.6) |
| Shared-memory controller, `src/smc.rs` | control at `02AA 8000h` (32K), profilers SMCP0-5 at `02C2 0000h` + 16K per core | the SL2 register file and the six per-core profilers on the UMAP1 interface |

The SMC module is the register file plus the queries the device model drives it with. It models
neither the SL2 storage nor the prefetch buffers, arbitration, write token controller or wait
states of [SPRUEG5D](../reference/sprueg5d.pdf) §2.3 to §2.5 — those live in the memory path,
which reads the levels this block exposes (`prefetchable_pages`, `page_sleep_state`) and pushes
events into it (`note_protection_fault`, `SharedMemoryProfiler::note_access`). SPRUEG5D gives no
base addresses at all; both come from SPRS612G Table 2-2.

The C6455's equivalent of the chip configuration block is `src/devcfg.rs` at a different address
with different fields. The two devices share no register there beyond the idea.
