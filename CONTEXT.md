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
The per-core module owning one megamodule's L1P, L1D and L2 caches, its MAR cacheability bits and its cache configuration registers, and the coherence protocol DMA masters share (L2 SRAM is snooped against L1D, nothing else is, L1P is never snooped). It holds no storage: the bytes behind the caches are the backing store, which the fabric passes in on every access.
_Avoid_: memory system, memory subsystem

**backing store**:
The storage every master shares: each megamodule's L2, L1P and L1D SRAM, the shared L2 RAM and ROM, the DDR2 window and everything sparse. Addresses arrive as the requester issued them and are resolved through the address map, so a core's local window and the global alias of the same memory land on the same bytes while remaining two distinct cache tags.
_Avoid_: memory, RAM, physical memory

**address map**:
The per-device table that answers which physical memory an access lands in, given the requester and the address: the local windows a megamodule sees, the global alias block a multicore device gives every core's memories, the shared memories, and the DDR2 window. It deliberately does not canonicalise the aliases away, because two addresses for one byte are two cache tags on silicon.
_Avoid_: memory map, decoder

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
The core every front end drives on a multicore device: the one whose registers `info` prints, whose memory `mem` dumps and which `stepi` advances. The fabric follows the focus, so a front end written for one core works unchanged; a breakpoint hit on another core does not change the focused core.
_Avoid_: current core, active core

**device event map**:
The routing table from peripheral source to megamodule system event and EDMA3 synchronisation channel, as the datasheet numbers it (SPRS276M Tables 7-9 to 7-11). The router's tick and latch methods consult it instead of hardcoding the numbers.
_Avoid_: interrupt map, event table

**slave port**:
The interface a peripheral presents to the router, as the `SlavePort` trait: `read(address, context) -> Option<u32>` and `write(address, value, context) -> PortWrite`, answering `None` or `Unhandled` for an address it does not own. The router is the single seam the CPU and DMA masters cross to reach it. The context carries what a port needs but does not own - the device's endianness, which three ports report their registers under - and the write outcome carries the completion event IDMA raises; the interrupt latchers stay in the router, because a follow-up the megamodules are owed is not part of a port's shape.
_Avoid_: register interface, MMIO interface

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

**machine**:
The module every front end drives: a loaded image, the CPU, and the host connections the
device needs — the CIO handshake, the Ethernet backend, the HPI host side, the trace sink.
It owns the halt rule and the state snapshot, so the batch runner, the console debugger,
the browser dashboard and the GDB stub all run through the same seam.
_Avoid_: emulator, session, driver

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

**functional unit**:
The execution resource an instruction claims — L, S, D or M on side 1 or 2. Packet validity is unit-conflict checking.
_Avoid_: unit slot, pipe
