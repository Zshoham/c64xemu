# c64xemu

A standalone emulator for **TMS320C6455**- and **TMS320C6472**-class **C64x+** DSPs: an 8-issue VLIW core with an
exposed pipeline, its caches and memory protection, its interrupt controller, the EDMA3
controller, the timers, GPIO, I2C, HPI, UTOPIA and the two PLL controllers, the DDR2 and EMIFA
register files, the device configuration block, the ROM bootloader's tables, and the Ethernet MAC
and MDIO module the device talks to the world through. It loads TI C6000 EABI ELF images produced
by TI's own toolchain and runs them cycle by cycle.

The emulator is written to be *checkable*. Native Rust tests pin decoder, CPU, timing and
peripheral behavior to the manuals and binutils opcode tables. Integration cases under
`tests/` compile target firmware and execute the resulting ELF through the same
`c64xemu run` command used outside the test suite.

```console
$ devenv shell
$ just build
$ just run firmware.out --max-cycles 8000000 --state-out state.json
```

## Development

The devenv shell supplies the Rust toolchain, `cargo-nextest`, pytest, and `just`. `just` is the
cross-language command entry point; the underlying runners remain directly usable.

```console
$ just test-fast                         # native unit and component conformance tests
$ just test                              # native tests plus every firmware integration case
$ just test-rust component_conformance   # select the native low-level conformance cases
$ just test-conformance -m network       # select a firmware integration suite
$ just test-conformance -m programs -k packed_simd  # select target programs by pytest node ID
$ just check                             # format check, Clippy, and the fast tests
$ just ci                                # complete pre-merge quality gate
```

### Fresh checkout

Everything the build and the test suites need beyond the checked-in corpus is fetched and
build on demand: `tests/conftest.py` provisions `tests/build/` with
`tests/tools/fetch_deps.sh` (XDCtools, NDK, SYS/BIOS and the C6000 code generation tools, TI's
DSPLIB for C64x+, and the tic6x vectors from a binutils release) and builds the firmware images
(and the EABI NDK libraries) before the suites that need them run. A fresh checkout plus
`python3 -m pytest` builds everything; nothing toolchain-sized or third-party is versioned.

Without devenv the equivalent commands are:

```console
$ cargo build --release --locked --bin c64xemu  # the emulator the suites drive
$ cargo fmt --all -- --check
$ cargo clippy --all-targets --all-features --locked -- -D warnings
$ cargo nextest run --all-targets --all-features --locked  # or: cargo test
$ python3 -m pytest                            # conformance and firmware integration
```

The Rust side needs only a stable toolchain (MSRV 1.80) plus `cargo-nextest`; the Python side
is the standard library plus pytest. CI runs the same split: the `check` job is the format,
Clippy and native-test gate, and the `conformance` job builds the release emulator and runs
the pytest suite with the paced high-bandwidth network cases deselected
(`-m "not high_bandwidth"`), since a shared runner cannot be relied on to sustain them.
Locally, `python3 -m pytest` runs everything.

### Rebuilding the firmware

The firmware images under `tests/firmware/*/build/*.out` are build products, not checked in: the
test suite builds them (and the EABI C64x+ NDK libraries they link) automatically when they
are missing, and `just firmware` does the same by hand. Building needs TI's XDCtools, NDK, SYS/BIOS and C6000 code generation tools in `tests/build/`;
`tests/tools/fetch_deps.sh` downloads and unpacks the exact releases the builds use from TI's
public software server (about 600 MB of downloads, kept in `tests/build/.downloads`), and
`tests/tools/build_ndk_libs.sh` builds the NDK libraries (TI ships none in any NDK release) from
`tests/tools/patches/`. Then:

```console
\$ tests/firmware/bios_kernel/build.sh
\$ tests/firmware/ndk_tcp/build.sh
```

Two portability notes: the SYS/BIOS component ships as a 32-bit installer, and the XDCtools
launchers are dynamically linked — from the devenv shell the script runs both inside the
`ti-tools-fhs` FHS environment, and on other distributions the 32-bit loader
(`sudo apt install libc6-i386`) is required.

### Published builds

Pushing a `v*` tag runs `.github/workflows/release.yml`: it builds `c64xemu` for
`x86_64-unknown-linux-musl`, `aarch64-unknown-linux-musl`, `x86_64-apple-darwin` and
`aarch64-apple-darwin`, smoke-tests every binary it can execute on its runner, and publishes one
`.tar.gz` per target (binary plus `README.md` and `LICENSE`) with a `SHA256SUMS` manifest as a
GitHub release. Windows is not a supported target. The Linux builds are statically linked
against musl — the dependency tree is pure Rust — so they carry no glibc baseline from the
runner. The job refuses to publish when the tag does not match the `version` in `Cargo.toml`.

Running the workflow manually (`workflow_dispatch`) builds and packages the same set from the
selected ref and leaves the archives as workflow artifacts, named after the commit instead of a
version, without touching releases.

## What is modelled

**Instruction set and execution.** Fetch packets and execute packets, the header-based compact
16-bit encodings with their register-set remapping and PROT/SAT/DSZ header fields, predication,
functional-unit assignment across L, S, M and D on both register files, cross-path reads, delay
slots through a write-back queue, multicycle NOPs, branches in flight, CALLP, the SPLOOP buffer
(SPLOOP, SPLOOPD, SPLOOPW, SPKERNEL, SPKERNELR, SPMASK, SPMASKR, nested loops reloading from
RILC), 40-bit long arithmetic, the packed SIMD set, the Galois field unit, the complex-multiply
family, circular addressing through AMR, and the saturation status bits in CSR and SSR.
Each fetched instruction is classified once. Execution, naming, predicate and operand rendering,
and packet-resource validation project the same stored family; invalid encodings remain unclassified
and take the same fault or raw-word path in every consumer.

**Pipeline timing.** Cycle counts follow the documented stall rules rather than an instruction
count: cross-path stalls (SPRUFE8B 3.8.4), L1D bank conflicts, read-miss and miss-pipelining
penalties for the C6455's L2 (SPRU862B Table 3-2), the four-entry L1D write buffer draining at
two cycles an entry (SPRU871 3.5.3), PROT fetch-packet NOP insertion, and the nine-cycle
interrupt overhead (SPRUFE8B 5.5.1).

**Memory and caches.** The C6455 memory map, L1P direct-mapped and L1D two-way, both as
configurable SRAM/cache, L2 four-way, L2 SRAM, the cache configuration and control registers with
their freeze modes, memory attribute registers, the write buffer, IDMA, the memory protection
units, and the bandwidth-management and power-down registers. A cache size decides two things and
the model applies both: the cache's capacity, and how much SRAM is left. The cache is carved out
of the memory from the top down - SPRU862B section 2.3 for L2, SPRU871K sections 2.4 and 3.3 for
L1P and L1D - so the address map is a function of the cache configuration, and an address the
cache has taken answers as nothing to the CPU, to a DMA master and to an image loader alike.
Both sides cost cycles: L1D read misses priced by whether the line comes from
L2 SRAM or the L2 cache, bank conflicts and write-buffer drains from SPRU862B's Table 3-2, and L1P line fills with the miss pipelining of its
Table 1-3, whose sixteen averages for this device's L2 are reproduced by one occupancy per line.

**Interrupts and exceptions.** The megamodule interrupt controller - event flags, the event
combiners, INTMUX routing to CPU interrupts 4 to 15, dropped-interrupt reporting - and the CPU
side: IER, IFR/ISR/ICR, GIE and NMIE, ISTP vectoring, TSR/ITSR/NTSR with the SPLX bit, IRP and
NRP, and the loop-buffer pipe-down and pipe-up sequences an interrupt taken inside an SPLOOP
requires.

**EDMA3.** The channel controller with its 256 PaRAM sets, DMA and QDMA channels, linking,
chaining, the eight shadow regions behind their access enables, the completion and error
interrupts, and the four transfer controllers with their error reporting. Peripherals feed it
real synchronisation events: the timers on channels 1, 2, 16 and 17, GPIO on 48 to 63 and the
I2C on 44 and 45.

**Timers, GPIO, I2C, HPI, UTOPIA and the PLLs.** Both 64-bit timers in 64-bit and dual 32-bit
modes, clocked at the documented CPU/6, with their CPU interrupts and EDMA events; the sixteen
GPIO pins with edge-triggered interrupts and events; the I2C module with its digital loopback
path, interrupt vector and DMA events; the HPI as the CPU sees it; the UTOPIA 2 ATM slave with its
clock detector, error interrupt and transmit event; and both PLL controllers with the GO operation
that applies a new divider ratio.

**External memory and device configuration.** The DDR2 controller's register file with its boot
and timing locks and refresh-rate floor, the EMIFA registers with their chip-enable configurations
and time-out interrupt, DEVSTAT and the JTAG ID, the bus priority allocation, and the PERLOCK key
that opens one PERCFG0 write with PERSTAT0/PERSTAT1 reporting the transition.

**Booting.** The ROM bootloader's boot table - entry point, count/address/data sections, zero
terminator - and boot configuration tables that set, clear and toggle through CPU accesses.
`c64xemu run IMAGE --boot-table FILE:ADDR --boot-config FILE:ADDR` loads nothing but those tables
and starts where the boot table says.

**Booting through HPI.** C6455 `--boot-mode 1 --hpi-unix EMU.sock` is the hardware host boot of
SPRS276M section 2.4, so no ROM code is involved. Strapped to it, the HPI comes out of reset
enabled with HPIRST clear, and core 0 is held in reset while an external host loads code and data
through the address and data ports. The host's first DSPINT releases the core at the base of L2,
`0x00800000`, with reset CPU state (GIE clear). DSPINT stays set until the application clears
it. There is no boot table and the positional ELF supplies only symbols. Both L1 caches are
32 KB and L2 is SRAM (SPRUEC6G Table 2).

`tests/tools/hpi_host.py --boot --image IMAGE --report` launches the emulator, transfers and
verifies the ELF load segments, flushes partial FIFO bursts, then raises DSPINT. Link the
image's entry at L2 base; the host tool rejects other entry points. `--no-dspint` loads and
verifies without releasing the CPU. `--state-out FILE` records the final state, including
`hpi_boot.entry`, which is null until release. Mailbox mode remains the host tool's default.
The C6472's mode 1 is a different HPI boot protocol and is refused.

**Booting over Ethernet.** The C6472's ROM Ethernet boot (SPRUEC6G section 4.3.5), which is how a
board with no boot EEPROM comes up: `c64xemu run IMAGE --device c6472 --boot-mode 9` loads nothing
at all. Core 0 is held while the boot code runs in its place - bringing up the MAC port the
BOOTMODE pins chose, posting a ten-buffer receive ring, sending the BOOTP-shaped Ethernet-ready
announcement once - and every byte of memory then arrives in UDP frames carrying boot-table
fragments, checked frame by frame as the manual specifies: MAC header (DIX or 802.3 with SNAP/LLC,
with or without a VLAN tag), IPv4, UDP ports and checksum, then the 0x544B magic, the opcode and a
sequence number that must not skip. The table is what places each core's image, through the global
aliases, and what starts the cores, because writing DSP_BOOT_ADDRn and BOOT_COMPLETE_STAT is just
a section like any other; when the terminator arrives the boot branches core 0 to the entry point
the table named. The positional ELF supplies only symbols, so anything that runs came off the
wire. `tests/tools/emac_boot.py --socket HOST.sock --peer EMU.sock --core 0:image.out --core
1:image.out` is the host side: it waits for the announcement, turns ELF load segments into a boot
table addressed to each core, and sends it. `--mac-id` fuses the address the device names itself
by, `--boot-udp-port` moves the port it listens on, and `--boot-mode 10` puts the wire on the
second MAC, which is where the backend's frames then arrive.

**Booting off an I2C EEPROM.** The ROM's master I2C boot (SPRUEC6G section 2.3.4), which is how a
board with a boot EEPROM comes up: `c64xemu run IMAGE --boot-mode 5 --i2c-eeprom FILE` loads
nothing from the ELF either. The file is the EEPROM's contents, put on the bus at the 0x50 the
bootloader requires, and the boot reads it by driving the modelled I2C master's own registers -
two internal address bytes as a master transmit, then a repeated START and a master receive - so
the sequence exercises ICSAR, ICCNT, ICMDR, ICDXR and ICDRR the way firmware does. Everything on
the EEPROM is blocked: a 16-bit size counting the four-byte header and a 16-bit ones-complement
checksum, which is verified and re-read once on failure. The boot parameter block at
`0x80 * --boot-param-index` names what follows, and for a boot table the sections are streamed
block by block into memory before core 0 is branched to the entry point.

**Six cores on one device.** `--device c6472` is a TMS320C6472: six C64x+ megamodules, each with
its own 32 KB L1P, 32 KB L1D and 608 KB L2 SRAM, its own interrupt controller, memory protection
units, bandwidth manager and IDMA, around one set of peripherals. 768 KB of shared L2 RAM at
0020 0000h is common to all six; each megamodule's memories also carry the global address
(1n80 0000h for core *n*) that another core, an EDMA3 transfer controller or an Ethernet MAC must
use, and a DMA access through that alias is snooped against the owning core's L1D exactly as a
local one would be. The boot controller holds cores 1-5 in reset until core 0 gives them a start
address in DSP_BOOT_ADDRn and releases them through BOOT_COMPLETE_STAT, DNUM tells one image which
core it is running on, IPCGRn/IPCARn raise and acknowledge an interrupt in another core (system
event 84), and the power/sleep controller can stop a core's clock. Twelve 64-bit timers - six
private, six shared - and the chip-level register file with its key-locked device control and
permission map are modelled from SPRS612G; the C6455 remains the default device and behaves
exactly as it did.

**Ethernet.** The EMAC with all eight receive and transmit channels, CPPI descriptor rings with
SOP/EOP/EOQ/OWNER and fragmented packets, receive filtering (address RAM with per-entry channel
and match/filter bits, the six-bit multicast hash, broadcast and promiscuous channel selects),
frame classification with the copy enables that decide whether a short, long or control frame
reaches memory at all, flow control and pause frames, receive QOS, the FIFO and DMA overrun
rules, transmit queue arbitration, the network statistics with their write-to-decrement
behaviour, channel teardown, host error reporting, MACINVECTOR and the single level-sensitive
MACINT with its EWCTL gate, and the MDIO module with a clause-22 PHY behind it. On the C6472
there are two MACs and one MDIO between them, and the interrupt-control module in front of each
MAC decides per core which channels that core is interrupted by - which is how six cores share a
single Ethernet connection: one owns the descriptor ring, and the receive interrupt follows
whichever core enabled the channel.

**Host I/O.** CIO for `printf` through the TI runtime, pcap in and out so a firmware's traffic
can be injected and captured, and a live Ethernet backend: raw frames over a Unix datagram
socket, polled from the run loop on a cycle boundary.

**A real RTOS.** SYS/BIOS 6.42 runs as built by TI's own tools, and its scheduler, interrupt
dispatcher and timebase are exercised directly: `tests/firmware/bios_kernel` hands work down a chain of
tasks, posts a Swi from a hardware interrupt, wakes a task from interrupt context, takes a second
interrupt inside the first one's service routine, shows that an interrupt raised while its enable
bit is clear is still delivered when the bit returns, and drives periodic Clock callbacks,
`Task_sleep` and a Mailbox. Every count is fixed by the program rather than by a timer, so each is
asserted exactly.

**A host on the other side of the HPI.** The host port interface is driven from a process on the
host over a datagram socket, one datagram per host access, so firmware and host exchange a mailbox
exactly as a real board would: the host writes a request through HPID, raises DSPINT, and the
firmware's interrupt service routine answers in memory and raises HINT.
`python3 tests/tools/hpi_host.py --image <image>` runs four rounds of it.

**A real network stack.** TI's NDK 2.22 TCP/IP stack on SYS/BIOS 6.42, built by TI's own tools
from TI's own sources, runs as firmware in the emulator with a NIMU driver written against the
modelled EMAC. `python3 tests/tools/tcp_peer.py --image tests/firmware/ndk_tcp/build/ndk_tcp.out` brings the
interface up - the firmware has no static address, so its DHCP client asks and the peer's DHCP
server leases one - resolves ARP, opens a TCP connection to the echo server on port 3000,
exchanges a payload and closes it: a host process holding a connection with firmware on the
emulated
C6455. The driver takes the EMAC's receive interrupt (system event 17 through INTMUX), so the
whole exchange costs well under a second of wall time and runs as part of the suite. The same
firmware runs the NDK's HTTP server, so `--port 80 --request 'GET /index.html HTTP/1.0\r\n\r\n'`
fetches a page the application put in the stack's embedded file system.

The same firmware also registers three of TI's own server daemons out of the NDK's
`tools/servers`, which the library it links has always contained: `--port 1000 --datasrv 4096`
asks `dtask_tcp_datasrv` for 4096 bytes and requires the exact bytes TI's daemon sends, out of a
buffer it fills itself, and `--port 7 --bytes 4096` runs its no-copy TCP echo daemon. The
expectation in each is TI's code's rather than this project's.

## Layout

| Path | What |
|---|---|
| `src/cpu.rs` | the core: fetch, decode dispatch, execution, write-back queue, SPLOOP, stalls |
| `src/decoder.rs`, `src/view.rs` | single-pass family identification from the binutils opcode tables, and its shared execution, validation and display projections |
| `src/cache.rs`, `src/memory.rs` | the caches, and the backing store every master shares |
| `src/system.rs`, `src/port.rs` | the fabric: megamodules, shared peripherals, routing and the device tick, and the slave-port trait every peripheral answers |
| `src/device.rs`, `src/space.rs` | the device profiles (C6455, C6472) and their address maps |
| `src/bootctl.rs`, `src/chipcfg.rs`, `src/psc.rs`, `src/smc.rs`, `src/emic.rs` | the C6472's boot controller, chip-level registers, power/sleep controller, shared-memory controller and EMAC interrupt control |
| `src/control.rs` | the CPU control register file |
| `src/emac.rs`, `src/mdio.rs`, `src/pcap.rs` | Ethernet |
| `src/edma.rs`, `src/edma_tc.rs`, `src/idma.rs` | the EDMA3 channel and transfer controllers, IDMA |
| `src/timer.rs`, `src/gpio.rs`, `src/i2c.rs`, `src/hpi.rs`, `src/utopia.rs`, `src/mcbsp.rs`, `src/pll.rs` | the timers, GPIO, I2C, HPI, UTOPIA, the two McBSPs and the PLL controllers |
| `src/ddr2.rs`, `src/emifa.rs`, `src/devcfg.rs`, `src/boot.rs`, `src/romboot.rs` | the external-memory controllers, device configuration, boot tables, and the ROM bootloader's Ethernet boot |
| `src/elf.rs`, `src/cio.rs` | image loading and the host I/O handshake |
| `src/machine.rs` | the loaded machine and the options that build it: what `run` and `debug` share |
| `src/runner.rs` | the batch runner: the loop and the state report |
| `src/debugger.rs`, `src/ui.rs`, `src/ui.html`, `src/gdb.rs` | the three front ends of `debug`: console REPL, browser dashboard, GDB stub |
| `src/trace.rs` | the execution trace: text or JSON Lines, whole-run or a post-mortem ring |
| `build.rs` | generates one native `#[test]` per component-vector file for nextest |
| `src/component_conformance.rs`, `tests/vectors/component/` | native low-level conformance executor and one JSON vector per migrated case |
| `tests/vectors/component-python-baseline.json` | historical record of the former Python conformance cases; current contracts are exercised by the per-case vectors |
| `tests/conftest.py`, `tests/common.py`, `tests/target_programs.py` | shared pytest fixtures, case machinery, and the target-program build pipeline |
| `tests/test_*.py` | target-program, RTOS, network, HPI and decoder-differential integration cases, one module per domain |
| `justfile`, `devenv.nix`, `pyproject.toml` | reproducible tools, cross-language tasks, and pytest configuration |
| `tests/programs/` | 85 programs, each built at the optimisation levels its spec names |
| `tests/firmware/ndk_tcp/` | TI's NDK TCP/IP stack on SYS/BIOS as firmware: the application, a NIMU driver for the emulated EMAC, and the build |
| `tests/firmware/bios_kernel/` | a SYS/BIOS kernel exerciser: three tasks on two semaphores, a Swi posted from an interrupt, and one interrupt nested inside another |
| `tests/firmware/hpi_mailbox/` | firmware that serves a mailbox over the host port interface, taking DSPINT as a real interrupt |
| `tests/firmware/c6472_multicore/` | firmware for the six-core device: core 0 gives cores 1-5 a start address, releases them through BOOT_COMPLETE_STAT, announces itself with an inter-processor interrupt, and meets all five in the shared L2 |
| `tests/tools/tcp_peer.py`, `tests/tools/tap_bridge.py` | a host TCP peer over the emulator's Unix-socket Ethernet backend, and a TAP relay |
| `tests/tools/hpi_host.py` | a host process driving the HPI: writes DSP memory, raises DSPINT, waits for HINT |
| `tests/tools/decode_sweep.py`, `tests/tools/dis6x_diff.py` | the two decoder differentials: binutils' whole vector corpus, and TI's own `dis6x` over every built image |
| `tests/tools/fetch_deps.sh`, `tests/tools/build_ndk_libs.sh` | fetch everything the suite builds with and reads into `tests/build/` - the TI toolsets, DSPLIB and binutils' tic6x vectors - and build the EABI NDK libraries from a patch |
| `tests/build/dsplib-c64plus/`, `tests/build/binutils-tic6x-vectors/` | the two third-party trees the suite reads as input, provisioned not checked in: DSPLIB 3.4.0.0, whose natural-C kernels 23 program specs compile, and binutils' 231 tic6x vector files |
| `docs/` | this project's own documentation: `docs/guide/` is how to use the emulator, `docs/internals/` how it works, and `docs/reference/` every TI document the work cites - each under a descriptive name with a literature-number symlink beside it - plus the reference sources linked at the pinned upstream revision each was checked against |
| `skills/` | the nine notes this work was derived from, each citing its sources |

## How it is validated

`cargo nextest run --all-targets --all-features` runs the Rust unit and low-level component
conformance suites. `python3 -m pytest` builds target firmware and drives it through normal
`c64xemu run` invocations; each integration expectation keeps its case ID as the pytest node ID.

The suite has six kinds of case:

- **Decode vectors** - exact instruction word expectations from the binutils tic6x test suite,
  checked mnemonic by mnemonic.
- **Differentials against TI's own tools** - three of them, run inside every pass and asserted as
  cases: all 2195 instruction words in binutils' vectors, and 436813 instructions across the
  firmware and program images compared with `dis6x` on seven dimensions - mnemonic, functional
  unit, cross-path marking, predicate, memory data path, every memory access's address operand,
  and every operand list the model reads, 285856 of them, which is every family except the
  PC-relative label forms. Four of the defects
  found in this emulator came from these rather than from reading manuals: seven C67x instructions
  the device does not have, a compact subtract that saturated on the wrong functional unit,
  cross-path reads that went unrecorded so packets TI's assembler refuses were accepted, and a
  40-bit subtraction from a constant that raised a fault although `cl6x` assembles it.
- **Behavioural cases** - hand-computed CPU, cache, memory, interrupt, EDMA3, timer, GPIO, I2C,
  Ethernet and MDIO expectations, each citing the manual section it comes from.
- **Timing cases** - cycle counts derived from the documented stall rules before the emulator was
  measured, never fitted to it. The cache tables are the strongest of those rules: SPRU862B's
  Table 1-3 gives sixteen averages for this device's L2, and solving them for a single line
  occupancy - 14/3 cycles out of L2 SRAM, six out of the L2 cache - reproduces every entry, which
  is what closed the program-fetch side. Table 3-2 does the same for the data side, where the cost
  of a miss depends on whether the line comes from L2 SRAM or the L2 cache.
- **Live network cases** - a host process leases the firmware an address over DHCP and then holds
  TCP connections with TI's NDK stack, over the emulator's Unix-socket Ethernet backend: one
  segment, 64 KB echoed in both directions at once, one where an echoed segment is dropped and
  never acknowledged so only the firmware's own retransmission timer can complete the stream, a
  datagram echoed through the socket API's UDP path, an HTTP GET answered by the NDK's own web
  server from its embedded file system, a telnet session through TI's own daemon and console,
  two of TI's own server daemons answering the protocols TI's own client sources define,
  and a paced bandwidth set: for every power of two from 16 to 4096 kbps, one TCP stream and
  one UDP datagram burst carrying a second's worth of traffic at the configured rate. All of
  them
  assert the bytes that came back and the lines the firmware printed rather than any cycle
  count; the bandwidth cases additionally require the whole exchange to complete at the
  configured rate, which is the emulator's wall-clock ceiling measured honestly - the rates
  above 256 kbps fail until the model gets faster.
- **Programs** - 85 programs compiled with TI's toolchain at up to five optimisation
  levels, from `sum_loop` to the DSPLIB FFTs, five Ethernet firmwares and two that boot from a
  ROM table built by the toolchain's own hex conversion utility. Each is checked for building,
  running without a fault, halting, and producing the exact expected value - and then run a
  second time with `--cache`, with L1P, L1D and L2 configured as caches, and required to give the
  same answer, which puts allocation, eviction, the write buffer and the snoop protocol under
  every program the suite has. One program opts out through its spec, because its expectation is
  tied to its own cache configuration: it maintains external buffers with L1D operations alone,
  which is right where L2 is SRAM and wrong where L2 is a cache.

Program expectations come from an oracle that is independent of the emulator:

- the same C source built with the **host** compiler and run natively, which is what makes a
  wrong instruction show up as a wrong number rather than as a plausible one;
- a **host model of a documented rule** for things the compiler never emits, such as AMR
  circular addressing;
- the **manual's own worked examples**, used directly as unit tests for the packed, Galois,
  complex-multiply and pair-writing instruction families;
- **Python** computations over injected frames for the Ethernet firmwares.

That discipline is what found the defects this emulator no longer has: instructions decoded into
the wrong space, a compact one-bit destination remapped by the header register-set bit, a
draining loop losing an iteration, in-flight writes dropped when an interrupt arrived, and a
dozen more. Several of those only appear at one optimisation level, which is why every program
is built at all five.

## Running a firmware

Three subcommands: `decode` names raw instruction words, `run` executes an image to its end and
reports, and `debug` opens a session on it. `run` and `debug` use the same machine - image, memory
loading, Ethernet and HPI backends, tracing - so an option means the same thing in both and the
choice is only how you want to drive it.

```
target/release/c64xemu run image.out \
    --max-cycles 16000000 \
    --state-out state.json \      # registers, PC, cycles, fault, CIO output, transmitted frames
    --pcap-in frames.pcap \       # frames handed to the EMAC
    --pcap-out out.pcap           # frames the firmware transmitted
```

The shared machine options are `--device c6455|c6472` (default `c6455`), `--boot-mode` (the
BOOTMODE pins: 0 starts every core, 5 is the master I2C boot, 9 and 10 boot the C6472 over MAC
port 0 or 1, any other mode starts core 0 and leaves it to release the rest), `--mac-id` and
`--boot-udp-port` for that Ethernet boot, `--i2c-eeprom FILE` and `--boot-param-index` for the
I2C one, `--stop-pc` and `--break` (both take an address or a symbol),
`--boot-table FILE:ADDR` and `--boot-config FILE:ADDR`, `--pcap-in`/`--pcap-out`,
`--net-unix`/`--net-peer`/`--net-poll`/`--net-port` for the live Ethernet backend,
`--hpi-unix`/`--hpi-peer`/`--hpi-poll` for the host port, the `--trace*` family,
`--trace-exceptions`, `--profile` and `--cache`, which programs every core's caches before the
image runs - 32 KB of L1P, 32 KB of L1D and 256 KB of L2, the most a C64x+ megamodule has -
through the same configuration registers firmware writes, leaving the MAR bits alone.
`run` adds what only a batch run can mean: `--max-cycles`
as a whole-run budget, `--state-out`, `--dump ADDR:LEN`, `--pc-trail N` and `--trap-oob`. A run
that booted over Ethernet also reports an `ethernet_boot` object in `--state-out`: the frames the
boot accepted and discarded, the boot-table bytes it processed, and the entry point it branched to.
A run that booted off an EEPROM reports an `i2c_boot` object the same way: the parameter and table
blocks it read, the boot-table bytes it processed, any checksum failure, and the entry point.

On a multicore device every core out of reset runs each cycle and the shared peripherals advance
once, a breakpoint stops the device whichever core reaches it, and an image whose segments land
in the local L2 window is loaded into every core's copy of it - which is what a host or the boot
ROM does when six cores run one program. `--state-out` then carries a `cores` array with each core's PC, cycle count, halt state and
whether the boot controller still holds it in reset. The console debugger's `core` command lists
the cores and switches which one every other command drives.

A run ends at `C$$EXIT` (or `abort`), at `--stop-pc`, at a `--break` address, on a fault, or
when the cycle budget runs out. A fault is a hard error: an unknown opcode, a resource conflict
or an illegal packet stops the run rather than being ignored, which is the point - firmware
that would fail on silicon fails here too.

`target/release/c64xemu conformance` reads a JSON request array on stdin and answers with
observations; that is how the harness asks for single-instruction execution, cycle counts,
memory-mapped register behaviour and decodes.

## Debugging a firmware

`debug` has three front ends onto one session, because a console prompt, a browser page and a
GDB client are three ways to ask the same machine the same questions. They are selected by
flag rather than by subcommand, and are exclusive: each owns the breakpoint set and the run
loop, and sharing those between two clients would need a design of its own.

**The console REPL** - `c64xemu debug image.out`: `break`, `delete`, `continue`, `stepc`
(cycles), `stepi` (execute packets), `regs`, `reg`, `mem`, `write`, `disas`, `syms`, `watch` on
a register, `cache`, `reset`, `trace on FILE`, `state [FILE]` for the same JSON `run` writes,
and `info` for a state summary - PC and symbol, cycle and packet counts, CSR/IERR/IFR/IER, the
SPLOOP buffer, the cumulative counters and the instructions that occupied the datapath on the
last issuing cycle. Breakpoints halt inside the CPU at a fetch boundary rather than by patching
an opcode, so a branch that resolves straight onto one still stops, and inspection reads go
through the host view: looking at memory neither fills a cache line nor moves a replacement
clock.

**The browser dashboard** - `c64xemu debug image.out --ui [ADDR]`, default `127.0.0.1:8080`:
the persistent transport bar drives execution in bounded chunks - step a cycle or packet, run,
or run at a fixed instruction rate - and, on a multicore device, carries a core selector that
moves every view to another core. Core, Memory & cache, Ethernet and CIO output
workspaces keep the inspection surfaces focused. Core shows both register files with per-step
change flags, control and interrupt state, functional-unit use, the SPLOOP buffer, disassembly,
breakpoints and hotspots. Memory & cache shows capacity and occupancy, hit/miss and maintenance
counters, filterable resident lines with their cached bytes, host-view memory, and all 256 MAR
segments. Ethernet keeps bounded incoming/outgoing histories with decoded endpoints,
protocols and full frame dumps. CIO output follows stdout and stderr written through the TI
runtime host-I/O channel.

**GDB** - `c64xemu debug image.out --gdb [ADDR]`, default `127.0.0.1:1234`: the remote serial
protocol with GDB's own tic6x register layout, the target description over `qXfer`, `^C` to
interrupt a running target, `--max-cycles` as the budget for one continue, and the program's
exit reported as an exited process carrying A4. On a multicore device each core is a thread, so
`info threads` lists them and `thread 4` moves the session to core 3. Set `osabi none`, or gdb
takes the image for a tic6x-uclinux one and tries to read a DSBT loadmap that a bare-metal
target does not have.

```
$ gdb -q image.out -ex 'set osabi none' -ex 'target remote :1234'
(gdb) break main
(gdb) continue
(gdb) x/4i $pc
(gdb) stepi
```

**Tracing and profiling**, in any of them. `--trace FILE` records every issued packet with its
unit assignment, every register write-back, store and control-register write, every branch,
interrupt, loop start and fault; `--trace-format json` renders the same events as JSON Lines,
`--trace-start` skips the early cycles, and `--trace-ring N` keeps only the last N events and
flushes them at exit, which answers "what happened just before the fault" without recording ten
million cycles. `--profile` keeps the per-address packet histogram and reports it as `profile`
in the state dump; the same dump includes `cache_stats` with each cache's geometry, occupancy,
dirty-line count, hits, misses, fills, evictions, writebacks and invalidations. The dashboard
shows both profiles live. Tracing and inspection are passive: no counter, cache line or cycle
count changes when they are on.

## Roadmap

See [ROADMAP.md](ROADMAP.md) for what is deliberately not modelled yet and what the next
experiments would be.
