# c64xemu

A cycle-stepped emulator of TI C64x+ DSPs - the single-core TMS320C6455 and the six-core TMS320C6472 - covering the CPU cores, their memory and caches, and the device's peripheral set, validated against TI's own tools.

## Language

**instruction decoder**:
The module that answers "what instruction is this" for every encoding the CPU can execute, both 32-bit and compact.
_Avoid_: disassembler, decode table

**family identification**:
The reading of an instruction encoding that decides which instruction family a word belongs to. Its result belongs to the instruction and is the single source for execution, naming, operand reading, and resource accounting; those consumers never identify the family again.
_Avoid_: opcode lookup, decode

**operand reading**:
Reading the field values of a decoded instruction — registers, constants, cross-path selectors. Consumers of the family identification do this themselves; it is deliberately separate from identification.

**instruction view**:
The projection over the family identification that answers "what an instruction claims" without executing it: the unit, data path and cross path it uses, its predicate and operands as TI's tools print them, and the resource accounting the packet validator enforces. Every consumer that reads an instruction without executing it — the differential, the decode CLI, the trace, the debugger, the dashboard, the GDB stub — crosses the view's seam; the executor consults it for validation, so display and validation cannot disagree.
_Avoid_: disassembler output, renderer

**instruction**:
The value the instruction view presents: an encoding with the fetch-packet header pairing resolved once at construction, so naming, resource accounting and operand reading all dispatch on the same value. A compact instruction cannot be constructed without its header; a 32-bit one carries none. Its serialization is the JSON the differential and the decode CLI consume.
_Avoid_: decoded word, fetch entry

**memory hierarchy**:
The per-core module owning one megamodule's L1P, L1D and L2 caches, its MAR cacheability bits and its cache configuration registers, and the coherence protocol DMA masters share (L2 SRAM is snooped against L1D, nothing else is, L1P is never snooped). It also owns the timing of the L1 caches: the L1D bank conflict rule, the write buffer, the read-miss costs and the L1P fill occupancy. The CPU says what it accessed and which clocks it is on, and is told how many stall cycles that costs; it carries none of that arithmetic itself. It holds no storage: the bytes behind the caches are the backing store, which the fabric passes in on every access.
_Avoid_: memory system, memory subsystem

**write buffer**:
The L1D's four-entry, 128-bit buffer between L1D and L2 that holds write misses. It lives in the memory hierarchy beside the caches, because what it costs is cycles rather than bytes: the bytes are already written through. Entries merge only into the immediately preceding quad-word and never when the L2 cache may hold the line, drain in order at two cycles an entry to L2 SRAM and six to the L2 cache, and a read miss waits for the whole buffer.
_Avoid_: store buffer, victim buffer (the victim buffer is a separate path that flushes dirty L1D lines)

**data clock**:
The two clocks the CPU hands the memory hierarchy with every data access: the execute cycle, which stall cycles do not advance and which "the cycle right after another miss" is counted in, and the elapsed cycles, which they do and which the write buffer drains on, plus the stall cycles already queued for the access. The hierarchy never reads the CPU's counters itself.
_Avoid_: timestamp, time base

**backing store**:
The storage every master shares: each megamodule's L2, L1P and L1D SRAM, the shared L2 RAM and ROM, the DDR2 window and everything sparse. Addresses arrive as the requester issued them and are resolved through the address map, so a core's local window and the global alias of the same memory land on the same bytes while remaining two distinct cache tags.
_Avoid_: memory, RAM, physical memory

**address map**:
The per-device table that answers which physical memory an access lands in, given the requester and the address: the local windows a megamodule sees, the global alias block a multicore device gives every core's memories, the shared memories, and the DDR2 window. It deliberately does not canonicalise the aliases away, because two addresses for one byte are two cache tags on silicon.
_Avoid_: memory map, decoder

**privilege**:
The identity memory protection judges an access by: a PrivID (a core's index, 0 for the system masters, the programmer's for an EDMA3 transfer) and a user or supervisor mode. It is not the requester, which only chooses the address view an access resolves through in the address map; the two travel separately because one master can issue accesses under another's privilege.
_Avoid_: AID for the PrivID itself (an AID is the page-attribute bit a PrivID is checked against)

**megamodule**:
One core's private half of the device: its memory hierarchy, its interrupt controller and event latches, its memory protection units, its bandwidth manager, its power-down control and its IDMA. A C6455 has one, a C6472 six.
_Avoid_: core state, CPU context

**fabric**:
The device: the megamodules, the shared peripherals and the backing store, and the routing that decides which of the three an access belongs to. It owns the device tick, so the peripherals advance once per device cycle however many cores are running, and it raises each system event into the megamodules the event map says can see it.
_Avoid_: bus, interconnect, system

**core bus**:
The fabric as one core sees it - the seam the CPU crosses for every access it makes. The device lends the fabric to the core whose cycle is running, so a core reaches the shared peripherals without holding a reference to any other core.
_Avoid_: port, view

**device profile**:
What distinguishes one modelled device from another: core count, address map, event map, which peripherals exist and where, memory protection page geometry, and the identification registers. Every device difference is a field of it rather than a branch in the model.
_Avoid_: device config, variant

**focused core**:
The core every front end drives on a multicore device: the one whose registers `info` prints, whose memory `mem` dumps and which `stepi` advances. The fabric follows the focus, so a front end written for one core works unchanged; a breakpoint hit on another core does not change the focused core, and the CIO handshake is answered on whichever core reached it rather than on the focused one.
_Avoid_: current core, active core

**device event map**:
The routing table from peripheral source to megamodule system event and EDMA3 synchronisation channel, as the datasheet numbers it (SPRS276M Tables 7-9 to 7-11). The router's tick and latch methods consult it instead of hardcoding the numbers.
_Avoid_: interrupt map, event table

**slave port**:
The interface a peripheral presents to the router, as the `SlavePort` trait: `read(address, context) -> Option<u32>` and `write(address, value, context) -> PortWrite`, answering `None` or `Unhandled` for an address it does not own. The router is the single seam the CPU and DMA masters cross to reach it. The context carries what a port needs but does not own - the device's endianness, which three ports report their registers under - and the write outcome carries the completion event IDMA raises; the interrupt latchers stay in the router, because a follow-up the megamodules are owed is not part of a port's shape.
_Avoid_: register interface, MMIO interface

**master access**:
How a system master - the HPI's DMA, each megamodule's IDMA, the EDMA3, the EMACs and the UTOPIA PDMA - reaches the device: a privilege, an address, and bytes or a word, through the one `MasterAccess` trait, of which the fabric is the only production implementation (`system/master.rs`). A master says what it moves and under whose identity; everything else is decided behind the seam, once. An address that is memory goes to the backing store, and one inside a peripheral aperture reaches that peripheral's slave port after the privilege is presented to the EDMA3; the megamodules' own registers read zero for a master and ignore its writes (SPRU871K section 5.3.1); the memory protection unit of the megamodule that owns the page judges the access, and the first protected byte latches the fault and refuses the whole access; L2 SRAM is snooped against L1D under both tags; and a word is assembled from its bytes in the device's byte order in one place, a PDMA element narrower than a word being its low bytes right-justified. A master that is lent out of the fabric for its own transfer reaches its own aperture as plain memory, which is how the EMAC's descriptor RAM is read. The CPU's loads and stores cross the core bus and the debugger the host read; neither is master access.
_Avoid_: DMA path, bus master interface, DMA memory

**SYSCLK3 schedule**:
How the fabric clocks the blocks that run on SYSCLK3 (the CPU clock divided by six): the timers, the GPIO edge detector, the UTOPIA port and the device configuration block. Each block says how many coming edges are certain to be quiet - no event, no EDMA3 trigger, nothing that counts as movement - and can skip that many at once to exactly where single steps would leave it. The fabric steps a block only on the edge it is due, and catches it up whenever an access or a clock-gating change is about to look at it. Every SYSCLK3 edge is still a full device tick; only the calls into quiet blocks are elided. `sysclk3.rs`.
_Avoid_: lazy timers, event scheduler

**boot ROM**:
The device's on-chip bootloader, modelled by what it does rather than by its instructions:
the boot mode's register programming, the frames it puts on the wire, the memory it writes,
the cores it releases and the entry point it branches to. It runs in place of core 0, which
is held for the duration, because the ROM's own code is not something this project has.
_Avoid_: bootloader firmware, boot code image

**boot table**:
The format the boot ROM loads memory from, whether it sits in memory or arrives over a link:
an entry point, then a byte count, a destination and that many bytes per section, ended by a
zero count. A destination is any address a CPU access reaches, so a table places images and
writes registers - which is how one starts the other cores.
_Avoid_: image, load record

**boot source**:
What the strapped boot mode asks for, resolved once against the device profile: no boot (the
image is placed directly), the HPI host, a serial EEPROM over I2C, the UTOPIA ATM master, or
Ethernet on one MAC port. The pins themselves still reach the device, which latches them into
its registers and lets the boot ROM read them; every host-side decision - which connections a
boot needs, whether the image is loaded, which boot starts - reads the boot source instead.
_Avoid_: boot mode (for the resolved value; the boot mode is the pins)

**board**:
What surrounds the device and what it supplies: the BOOTMODE and configuration pins, the serial
EEPROM on the I2C bus, the UTOPIA PHY address, the fused MAC ID. It resolves the boot source and
refuses a command line that straps a boot it does not supply, or supplies something nothing it
straps reads. The device is built on a board, and a reload builds the same board again.
`Board` in `board.rs`.
_Avoid_: board config, strap options

**boot**:
The boot a board straps, as the device runs it and after it has: the boot ROM running in place
of core 0, the HPI boot holding the cores for the host, or nothing. The device ticks it and
defers core 0's release to it; the machine asks it whether it holds core 0 and what it reports.
No other module knows which boot is running. `Boot` in `board.rs`.
_Avoid_: ROM boot (for the whole; that is one kind), boot state

**machine**:
The module every front end drives: a loaded image, the CPU, the host connections the device
needs, the CIO handshake and the trace sink. It owns the run clock, the halt rule and the state
snapshot, so the batch runner, the console debugger, the browser dashboard and the GDB stub all
run through the same seam. Before each cycle, or batch of cycles, it serves the host connections
and then the CIO handshake, the same way on both run paths.
_Avoid_: emulator, session, driver

**host connections**:
The host-side connections attached to the fabric rather than to any core: the Ethernet backend,
the HPI host and the UTOPIA's ATM master. The machine lends them the fabric and the run clock;
they serve whatever is due in one fixed order (Ethernet, HPI, UTOPIA) and say how many cycles the
device may run before the next is due. Each is an adapter over a socket or file; the HPI and ATM
datagram protocols live behind them, so the machine knows no wire format. The CIO handshake is
not one: it is answered on whichever core reached it. `HostConnections` in `host.rs`.
_Avoid_: host link (that is the datagram channel under two of them), peripherals, host I/O

**Ethernet backend**:
The host-side module that exchanges frames with the fabric: it admits live received frames when the MAC has room, retains every transmitted frame for capture, and keeps a separate bounded history for inspection. It is one of the host connections; its socket adapter moves datagrams, while the machine supplies the emulated clock and the fabric owns the MAC's silicon behavior.
_Avoid_: network service, frame broker

**halt rule**:
The combination of a driver's breakpoints and the program's exit point into the fetch-halt
set the CPU enforces, dropping the address the target sits on when it is about to move off
it. The machine owns arming and restoration as well as the rule, so every front end stops at
the same addresses without managing the halt lifecycle.
_Avoid_: breakpoint logic, stop logic

**stop condition**:
One member of the set the machine's run loop evaluates on every cycle: a cycle budget, a
packet count, or a driver-supplied check. The machine-native conditions — a fault, the
exit point and the armed halt list — are always members and are checked first, and the
reason a run stopped names the member that fired, including the core and address for a breakpoint.
_Avoid_: stop event, run condition

**batch boundary**:
A point where a batch of cycles the machine asked the device for must end before the next cycle: a core out of reset has halted on an armed address, or any core is ready to fetch the CIO breakpoint (exempt on the first cycle after the machine has just answered the handshake). The device keeps both per core as cores step instead of asking every core each cycle, so the machine sees the halt or services CIO on exactly the cycle a per-cycle check would.
_Avoid_: stop point, batch end

**compact instruction**:
A 16-bit instruction encoding, which cannot be named or executed without its fetch-packet header.
_Avoid_: halfword instruction, short instruction

**fetch packet**:
A 32-byte aligned unit of program memory holding up to eight 32-bit instructions, or up to sixteen compact instructions with their header.
_Avoid_: line, cache line

**fetch-packet header**:
The last word of a fetch packet; the field compact instructions decode against, carrying the data size, saturation, branch and register-set bits.

**execute packet**:
The set of instructions a fetch packet (or loop buffer) issues in one cycle, by parallel bit.
_Avoid_: instruction group, bundle

**write-back queue**:
The per-core holding area for every effect an instruction makes after the cycle it issues: register destinations waiting out their delay slots, stores landing at the end of their execute cycle, MVC control-register writes, and saturation flags landing a cycle after their result. It decides where an effect waits, which of two colliding register writes survives (same issue cycle: a resource fault; otherwise the later-issued write replaces), and the order due effects come back in (registers, stores, control writes, saturations). The CPU performs the effects at the write-back that ends each cycle. `WritebackQueue` in `writeback.rs`.
_Avoid_: pending writes, delay buckets

**functional unit**:
The execution resource an instruction claims — L, S, D or M on side 1 or 2. Packet validity is unit-conflict checking.
_Avoid_: unit slot, pipe

**interrupt entry**:
The one way a cycle boundary vectors to a handler, whatever won it: a maskable interrupt, NMI,
or an exception. The source decides only the return address, which register pair saves the
interrupted context, and what becomes of a software-pipelined loop; everything else - no branch
surviving, Supervisor mode, annulled NOPs, the pipeline refill - is the same for every source.
_Avoid_: take, vectoring, exception entry (for the whole)

**cycle boundary**:
The one place a CPU cycle ends, told only how it ended: frozen by a stall, committed, or committed
with a SPLOOP stage to complete. A frozen cycle ticks the device and counts a stall, nothing
else; a committed one lands the write-back queue's due effects, completes the loop stage if any,
ticks the device, samples the interrupt lines for interrupt entry and advances the cycle - in
that order, whichever exit of `Cpu::step` it came from. `cycle_boundary` in `cpu.rs`.
_Avoid_: finish cycle, end of step
