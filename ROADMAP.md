# Roadmap

What this emulator does not model yet, why it matters, and how each item would be checked. The
ordering inside each section is roughly by value per unit of work.

The method that found nearly every defect so far, and that these items should keep using: **one
program per instruction class or peripheral feature, compiled at every optimisation level, with
an expectation derived from something other than the emulator.** A per-operation probe that
prints each result for several inputs localises a mismatch in two runs; the manual's own worked
examples make good unit tests; a host build of the same C source is the strongest oracle for
anything the compiler can express.

Current state: 1934 suite cases, all of them passing, plus 292 Rust unit tests and 388 component
vectors driven from them. Beyond the cases, three differentials against TI's own tools run inside
every suite pass: all 2195 instruction words in binutils' tic6x vectors, and 436813 instructions
across the firmware and program images compared with `dis6x` on seven dimensions - mnemonic,
functional unit, cross-path marking, predicate, memory data path, every memory access's address
operand, and every operand list the model reads, 285856 of them.

There is no `[ ]` item left, and the method at the head of *Where to look next* is what the last
two rounds have been: **two rules each modelled and each pinned, with nothing asking what they do
together.** It has now found six defects. The cache manual's worked examples found a 4% error
between a correct miss cost and a correct write buffer. Making a cache size carve its SRAM
aperture found that a megamodule was caching its own L1 memory, which two sections of SPRU871K
forbid. Enabling the EMAC's transmit mask found that a shared, edge-latched interrupt line makes
acknowledging one queue unsafe. Going looking on purpose found two more in one pass: an L1D
victim was allocating L2 lines the hardware never allocates, and the write buffer was merging
writes the hardware never merges. And the CPU and DMA halves of one memory protection block,
each pinned on its own, disagreed about which fault a block keeps: the CPU half overwrote the
record on every refused access, where the manual gives a block one fault and one exception until
software clears it. Every one of them had both halves right.

Two of that round's items also came back with an answer other than the one the item predicted,
which is the same discipline pointing the other way: the transmit interrupt was measured, found to
cost the paced UDP case eight runs in twenty and buy no headroom at any rate, and rejected in
favour of the reclaim it was built on; and RMII_RST turned out not to be able to hold this
device's MAC at all, because it resets the RMII logic and the straps select MII - the enable that
does gate the wire was GMIIEN, one register away, and twenty-six component vectors had been moving
frames without it.

Every item's bold lead starts with a box, and the box is the item's state:

- `[x]` **done** - modelled, and pinned by a suite case, a unit test or a differential the item
  names. A fixed defect is done.
- `[~]` **partial** - part of the item is modelled and the item says which part. The remainder is
  its own item directly below it, or an entry in *What is still not modelled* that the item names.
- `[ ]` **planned** - not modelled, and an expectation independent of this emulator is already
  available: a manual's table, a host build, TI's own tools, or a case the suite already fails.
  The work is the work.
- `[-]` **unplanned** - not modelled and not scheduled, because what is missing is not code. Two
  reasons recur: a number no document gives, where a mechanism is described with a bound instead of
  a value; and a second device model, without which nothing can observe the behaviour. An item
  moves to `[ ]` the day something outside the emulator can say what the right answer is, and that
  is the only promotion rule.
- `[!]` **note** - a standing caution rather than work: something about the tools or the compiler
  that new programs have to be written around.

*Where to look next* carries no boxes. It is a priority ordering over the items above, and an
item's state lives with the item.

## Network stack

**[x] TI's NDK on the emulator.** *Done.* NDK 2.22 on SYS/BIOS 6.42, both built by TI's tools from
TI's sources, run as firmware with a NIMU driver over the modelled EMAC: the interface comes up
with an address its own DHCP client leases from a server in the host peer - discover, offer,
request, acknowledge, which `network:ndk-dhcp-lease` pins - answers ARP, and `tools/tcp_peer.py` - a host process speaking raw frames
over the Unix-socket backend - completes a three-way handshake to the echo server on port 3000,
gets its payload back and closes the connection. Two CPU defects and one driver defect stood in
the way, and the whole exchange takes about 23 seconds of wall time.

The driver defect is worth stating because it is a hardware contract, not a bug in the model: a
CPPI receive queue that reaches a null next pointer sets EOQ, clears RXnHDP and *stays stopped*.
Descriptors linked onto its tail afterwards are not noticed, and the frames that arrive while it
is stopped wait in the receive FIFO. A refill that neither links nor restarts therefore receives
exactly one pass over the chain and then goes deaf - which is what the NIMU driver did, and why
every unicast frame vanished while broadcast ARP still worked. `ethernet_queue_stall` pins the
contract from both sides: the stalled queue delivers nothing, and the restart delivers the
waiting frame whole.

**[x] The driver takes the receive interrupt.** *Done.* System event 17 routed through INTMUX to CPU
interrupt 5 by SYS/BIOS's `Hwi`, with `RXINTMASKSET` enabling channel zero: the service routine
only posts the stack thread's semaphore, because acknowledging the queue in the poll routine is
what drops the module's interrupt line, and the megamodule latches that line's rising edge rather
than its level. Servicing a frame on arrival instead of on the NDK's tick took the exchange from
23 seconds of wall time to under half a second, which is what made it a suite case:
`network:ndk-tcp-echo`, `network:ndk-tcp-bulk`, `network:ndk-tcp-retransmit`,
`network:ndk-udp-echo` and `network:ndk-http-get` run `tools/tcp_peer.py` and check the echoed
bytes and the firmware's own lines. The bulk case sends 64 KB with a 4 KB window in 1 KB
segments and gets all of it back byte-exact, which wraps both descriptor rings many times over;
256 KB works too and was tried by hand, but 64 KB keeps the suite inside three seconds. The
retransmit case drops one echoed segment and never acknowledges it, so the stream can only
complete when the stack's own retransmission timer expires: that puts timer 0, the SYS/BIOS clock
it drives, the NDK tick and TCP's recovery in one assertion, at the cost of about twelve seconds
of wall time waiting for a timeout that is some seventy emulated milliseconds long. The HTTP case
adds the NDK's own web server and embedded file system to what runs: the reply asserted is the
exact header the server writes, sixty-five bytes of it, followed by the page. That server is a
large body of TI code the emulator would otherwise never execute, which is the reason to run it.

**[x] The telnet daemon and its console.** *Done.* `network:ndk-telnet-console` holds a session
through TI's `telnetd` and the NDK console behind it: the daemon accepts on port 23 and calls
`ConsoleOpen`, which spawns the console task on a pipe, and the console prints the application's
banner, the peer's own address, its help listing and a goodbye. The peer drives the dialogue by
content rather than timing - it waits for the marker of each stage before typing the next line,
and waits for the *last* line of the help listing before quitting, because quitting on the first
would cut the listing off mid-print - so all thirty console lines are exact, as are the 1069
bytes. This was the last of TI's own source the suite had never run, save the server daemons the
item below registers; the NDK's own benchmark programs are not runnable at all, because the
release ships their manifests and results without any application source.

**[x] Three more protocol contracts.** *Done.* `network:ndk-icmp-ping` is one echo request and the
stack's reply: NDK's `icmpin.c` reflects the request with the identifier, sequence and payload
intact, so the raw ICMP receive/reply path through the EMAC is asserted end to end.
`network:ndk-tcp-reset` SYN's a port nothing listens on and requires the RST|ACK back - the
`dropwithreset` path in `tcpin.c` - so a stack that silently swallowed the SYN would fail the
case. `network:ndk-tcp-out-of-order` sends the payload's segments newest-first and never
retransmits them, so the echo can only complete if `tcpin.c`'s reassembly queue (`pReasm`)
holds the later segments until their predecessors arrive; the peer's usual resend fallback is
disabled in that mode so retransmission cannot paper over a broken queue. All three run in the
same peer and are content-checked like the rest.

**[x] Bandwidth ceiling.** *Measured, and the ceiling now clears the top rate.*
`network:ndk-bw-tcp-*k` and
`network:ndk-bw-udp-*k` pace one second's worth of traffic at every power of two from 16 to
4096 kbps and require the whole exchange to complete at that rate: the TCP stream must come
back byte-exact, and every 500-byte UDP datagram must be echoed - there is no retransmission
on the datagram path, so a drop anywhere in the emulator's receive path fails the case. The
peer is the pacer (a token bucket over wall clock) and fails fast at a deadline of one and a
half times the configured duration, so a case is the honest claim that the emulator sustains
the rate. Every TCP rate through 4096 kbps now passes, which is the whole set. The rates at
1024 kbps and above
carry pytest's `high_bandwidth` mark so CI can deselect them, because a shared runner cannot be
relied on to sustain the emulator's pace. UDP stops at 512 kbps by design rather than by failure:
above it the firmware's UDP receive queue overflows once the paced datagram rate outruns the echo
task's drain, which is faithful modelling and not a useful regression claim
(`tests/test_network.py`). The UDP task in `firmware/ndk_tcp` serves 2048 datagrams where it served
one, and prints only the first, because a paced burst would otherwise flood the console pipe
faster than the emulator drains it.

**[x] Sustaining 4096 kbps.** *Done, by making the emulator faster.* The case was echoing about
470000 of its 512000 bytes before the peer's deadline, and nothing about the expectation needed
work: the byte count was the measurement. Two numbers drove the work, both taken from the NDK
firmware itself rather than from a microbenchmark - host instructions retired per emulated cycle
(`perf stat` over a fixed 60-million-cycle run) and emulated cycles per host second while the
paced peer drives it (CPU time to a fixed cycle budget under real traffic). The first went from
901.8 to 742.6 instructions per emulated cycle; the second from 12.74 to 14.90 million cycles a
second, which is 6.9% past the pre-multicore commit the case last passed on. Six changes, each
kept only because it measured:

- **The device tick is asked, not walked.** Every full tick re-derived the level-triggered
  interrupt sources and the activity summary by interrogating six peripherals, most of which
  cannot have changed. Both are now maintained on change: an access that leaves the memory
  windows or a frame a host delivers marks the device touched, each latcher reports whether it
  asserted anything, and each activity bit is recomputed only by the branch that ticked it. A
  dormant device answers both questions from the cache.
- **The dormant short tick is out of line.** The tick a dormant device takes five cycles out of
  six now returns from `Fabric::tick` before the device body's frame is built, and it answers
  the short-tick question by return value instead of through a per-core bitmask.
- **The exception combiner's output is a cached bit.** It was ORed out of four flag banks and
  four masks on every cycle; only a flag or mask write can change it.
- **`commit_writes` exits before entering four empty queues**, and asks the trace once per
  cycle rather than once per write.
- **`finish_cycle` skips the exception and interrupt blocks as a group** when nothing is
  pending, which is every cycle the loop buffer keeps out of the existing fast path.
- **The per-access seams stopped repeating themselves**: one L1D lookup answers both "resident"
  and "would miss", the MPU check's permissive path is inline with the fault path outlined, the
  packet's SPMASK and PROT questions are answered while it is fetched rather than by two more
  walks, and the halt list's one-address case is compared inline.

Both host seams also stopped punishing a scheduling hiccup, which is what turned the case from
bimodal into monotonic. A frame the backend delivered while the modelled 24-cell receive FIFO was
full counted as a start-of-frame overrun, and a frame the peer's socket buffer had no room for was
counted dropped: either loss costs a retransmission timeout - the peer's is a second, the
firmware's is tens of emulated milliseconds, which is seconds of wall time - and the whole
deadline is one and a half seconds. A Unix socket has no rate of its own, so neither loss was a
property of the modelled device. `Machine` now holds an inbound frame until the MAC has room, and
the backend queues an outbound frame the peer refused and retries it in order on the next poll.
What used to collapse to a third of the payload now falls short by a few per cent under heavy
competing load and passes otherwise: two consecutive full-suite runs are green.

**[x] Six cores echoing at once.** *Done.* Everything above measures one core through one MAC.
The C6472 puts six megamodules behind the same MAC (SPRS612G Table 7-105), and the claim a
single-core device cannot make is that all six move frames concurrently:
`tests/firmware/c6472_netecho` is a bare-metal firmware - no NDK, no SYS/BIOS - in which core 0
brings the MAC up, binds unicast address slot N to receive channel N for each of the six
addresses 02:00:00:64:72:0N, and every core then polls its own channel and echoes what arrives.
`tests/tools/multicore_echo.py` paces a deterministic stream at each of the six addresses with
the pacer and the one-and-a-half-times deadline the single-core cases use, and requires every
byte back from the core that owns that address, in order.
`test_c6472_echoes_paced_traffic_on_all_six_cores_at_once` runs it at the same nine powers of two
the single-core cases use, `[16k]` through `[4096k]`, per core: the top rate is 4096 kbps on each
of six cores, 24.6 Mbps of payload through one MAC, and the whole ladder passes. Where it stops
is measured rather than assumed - the exchange completes at 6144 kbps a core and does not at
7168, so the top case runs at two thirds of the boundary - and so is what is busy at that rate:
the emulator holds 89% of the run's wall clock against the host peer's 34%, which is what makes
the number a property of the device model rather than of the harness. A raw echo costs a fraction
per byte of what the NDK's TCP stack costs, which is why six cores reach a rate one core does
not. Everything through 1024 kbps a core passes with three competing CPU hogs, and so does the
top rate; the two rates from 2048 kbps up carry `high_bandwidth` and CI deselects them, because
that is where the headroom to the boundary falls below four times and a shared runner cannot be
relied on to sustain a pace that close.

Two device properties are what the firmware is written around, and a model that got either wrong
would fail the case rather than the reverse. A DMA master's address on this device is always
global, so every descriptor and buffer pointer the MAC is given is the shared L2 address the CPU
uses plus 1000 0000h; and one MAC's register file - its address RAM, its eight receive channels,
its eight transmit channels - is one thing all six cores write, so the setup happens once on core
0 and each core touches only its own channel afterwards.
`test_c6472_echo_needs_every_core_out_of_reset` is the control: in a ROM boot mode nothing
releases cores 1-5, so only core 0 announces a channel and the exchange never starts, which is
what makes the six-core result mean six cores.

**[x] The transmit side reclaims on every pass.** *Done, and it is the better half of what the
transmit-interrupt item asked for.* The reclaim loop was a prologue inside `emac_send`, so a
transmit descriptor - and the packet buffer it holds - came back only when the next send looked.
It is now `emac_reclaim_transmit`, and `emac_poll` calls it before draining the receive ring,
which matters because the buffer it hands back goes to the pool `emac_post_receive` allocates
from: a burst of transmits could otherwise hold buffers while the receive ring went hungry. The
measurement says it helps. Interleaved with the old driver on an idle machine at 512 kbps of
paced UDP - the case nearest its limit, and the one standing intermittent failure in this suite -
it echoed all 128 datagrams in 81 of 82 runs against 57 of 62.

**[-] Transmit interrupts in the NIMU driver.** *Tried, measured, and rejected.* Writing
`TXINTMASKSET` beside `RXINTMASKSET` makes the driver woken rather than looking, which was the
whole appeal, and it is a two-line change on top of the reclaim above. It is also slower.
Interleaved against the shipped driver, twenty runs each, the paced UDP case echoed everything in
12 of 20 runs against 20 of 20; at the rates above 4096 kbps both plateau at the same place,
about 530000 bytes inside the deadline, so it buys no headroom either. The mechanism is clear
enough: MACINT is one line the two queues share, so a transmit completion wakes the stack thread,
and that wake plus its scheduling pass lands on the critical path of the receive drain - the very
thing the case is short of. The transmit reclaim was never the bottleneck.

What the attempt did leave behind is a rule of the interrupt controller now pinned by a case,
because enabling that mask brings an obligation with it. The megamodule's selector is fed by the
raw event line, so what reaches IFR is its rising edge (SPRU871K Figure 7-9 and section 7.4.2),
and a completed transmit descriptor left unacknowledged holds the shared line high - after which
a receive completion produces no edge at all, and the receive interrupt this driver depends on
simply stops arriving. `ethernet_shared_macint` shows it: three frames arrive, the service
routine echoes the first and waits for the transmit to complete so the transmit queue is holding
the line, then acknowledges only the receive queue and posts the next descriptor - and the second
frame completes into memory with nothing happening at all, one entry with both raw status bits
set. Acknowledge both queues and the line falls; the third frame raises it from low and the
routine runs again. Acknowledging the transmit queue inside the routine instead turns the same
program's answer into two entries with both status bits clear, which is the control that says the
case measures the line rather than the frames. That is also why `TXINTMASKSET` staying clear is
what makes the shipped driver's single acknowledge safe.

**[x] The NDK's own server daemons.** *Done, and the benchmark applications turn out not to be
here to run.* The item wanted "the last of TI's own source this suite has never run", for the
reason that "a number TI's code prints is an oracle this project did not write". The programs it
meant are TI's benchmarks, and the NDK release notes say why they cannot be run: the examples are
no longer in the core product, and what the fetched tree carries under `ti/ndk/benchmarks` is
manifests and result spreadsheets with no application source. SPRAAQ5A describes the nonCopyTCP
tester and testee, and its `package.bld.xml` names `nonCopyTCPTestee.c`, but that file is not in
the release.

What is here, and had also never run, is better suited to the point: TI's own server daemons in
`tools/servers`, compiled into the `servers_min_ipv4` library this firmware has always linked and
never started. `app_network_open` now registers three of them through `DaemonNew`, and two carry
a contract precise enough to be a case.

`network:ndk-ti-datasrv` asks `dtask_tcp_datasrv` on port 1000 for 4096 bytes. The reply is not
an echo and not a number this project chose: the daemon reads a four-byte length - little-endian
on the wire whatever the DSP is, because its own big-endian branch byte-swaps what it receives
under the comment "Size comes in as little endian!" - and answers with `size / 512` blocks of a
buffer it fills with `data[i] = ' ' + i`. `tools/tcp_peer.py` reconstructs that rule from the
daemon's source and requires every one of the 4096 bytes to match; perturbing the reconstructed
block by one fails the case. 4096 is eight whole blocks on purpose, which keeps the daemon's own
`remainder` branch - it sends the leftover bytes out of the request word rather than the buffer,
which is what the code says - out of the expectation.

`network:ndk-ti-tcp-echo` runs `dtask_tcp_echo` on port 7 over 4096 bytes, and registering it was
the part that needed TI's source rather than TI's headers. It reads with `recvnc`, so it needs the
no-copy stream socket its own comment names - "dtask_tcp_echo() - TCP Echo Server Daemon Function
(SOCK_STREAMNC)". On an ordinary `SOCK_STREAM` it accepts the connection, fails its first read and
closes, which is exactly what the first attempt did: connection open, zero bytes echoed.
`dtask_udp_echo` is registered alongside them and left unasserted; the application's own UDP echo
already covers that path.

One thing the extra daemons cost, and it was not throughput. Three more start-up lines pushed the
peer's report past the eight firmware lines it kept, so the line every exchange asserts last fell
off the end of the report rather than out of the firmware. The bound is nine now, which is the
seven start-up lines and the two lines of a connection, and still excludes the "done" line whose
arrival depends on when the emulator is stopped. The paced cases run at the same rate they did.

**[-] A TAP relay demo.** Needs `/dev/net/tun` privileges this environment does not have. Nothing
about the model is missing - the case cannot be run here.

**[-] The console's own `ping` command.** Raw ICMP sockets in `conping.c` would be a natural
protocol case, but the command waits a fixed two seconds per probe in firmware time - five probes,
ten emulated seconds - which is minutes of wall time at this emulator's speed. It stays out of the
suite until the emulator is fast enough for it to fit, which makes it a consequence of the item
above rather than work of its own.

## Caches

**[x] L1P ignores the MAR bits.** *Fixed defect.* SPRU862B section 2.3 says that on C64x+ devices
"program fetches to external memory addresses are always cached in L1P, regardless of the
cacheability setting", where C64x had MAR govern all three caches. The model gated all three on
MAR, so external code with MAR clear was refetched from memory every time - and since L1P is never
snooped, the difference is observable: `cpu:l1p-caches-external-code-with-mar-clear` stores a zero
word over an ADD six words ahead in its own fetch packet and falls through to it, and the ADD
still runs. The companion case leaves L1PCFG at reset, where the refetch does see the store.

**[x] The snoop coherence protocol.** *Done.* SPRU862B section 2.4.2 is explicit about the only
coherence hardware maintains, and the model now implements both halves of it. A DMA write to L2
SRAM is forwarded into any L1D line holding that address - "the dirty bit is not affected by this
operation" - so the CPU sees the new data with no cache operation at all; a DMA read of L2 SRAM
is served from L1D when that line is dirty, so a buffer the CPU has only written into its cache
still goes out on the wire. Outside L2 SRAM nothing is snooped whatever the MAR says, which is
the case firmware has to handle itself.

`ethernet_external_coherence` is the same traffic with the buffers in DDR2 instead, where the
rules invert: the frame the EMAC wrote is invisible until an L1D range invalidate through
L1DIBAR/L1DIWC, and a buffer written only into L1D goes out as whatever memory holds until a
range writeback through L1DWBAR/L1DWWC. Removing either operation changes the result, which is
what makes the case load-bearing rather than decorative. The NIMU driver in `firmware/ndk_tcp`
issues the same two operations through SYS/BIOS's `Cache` module, because its packet buffers are
in cached DDR2 - though with the pool large enough for the stack to work at all, buffer reuse
outruns L1D and the omission is not observable there.

One protocol covers every master, so IDMA and the EDMA3 transfer controllers now take the same
path the EMAC does. That fixed two more divergences in their old ad-hoc version: it invalidated
L1P on a DMA write, where the C64x+ explicitly leaves L1P to the programmer, and it wrote dirty
lines back on a DMA read instead of forwarding them, which loses the fact that the line is still
dirty afterwards. `idma_l1p_stale_code` pins the first: a routine cached in L1P keeps running
after IDMA1 overwrites it in L2 SRAM, and only a range invalidate through L1PIBAR and L1PIWC
brings the replacement in. A DMA access to external memory now bypasses the CPU's caches
entirely rather than allocating in them, which is what a master's access does.

Finding this needed a real DMA master driving cached memory: `ethernet_dma_coherence` enables
L1D as a 32K cache, leaves both buffers resident in it and runs a frame through the EMAC in each
direction. Before the fix the program hung - the CPU polled a descriptor line the DMA had
written and never saw the ownership bit clear, which is exactly the "works on hardware, hangs in
the emulator" failure in reverse. Two follow-on facts came out of it: a host read is the debug
port and must see dirty lines too, or `printf` from a program with L1D enabled prints nothing;
and one committed expectation - that the EMAC transmits stale data from a dirty L1D line - was
simply wrong and is now `cache:emac-reads-dirty-l1d-line-through-snoop`, with the external-memory
counterpart pinned beside it.

**[x] The cache size encodings above the maximum.** *Fixed defect.* L1PCFG, L1DCFG and L2CFG each
carry a three-bit mode, and the model read every encoding past the largest documented size as
reserved and left that cache disabled. TI's own kernel says otherwise: `L1_cache_on` in
[`arch/c6x/platforms/cache.c`](https://github.com/torvalds/linux/blob/v5.10/arch/c6x/platforms/cache.c) writes 7 to L1PCFG and L1DCFG to turn the
L1 caches on, and `L2_cache_set_mode(L2MODE_256K_CACHE)` writes 7 for L2 under the comment "Set
L2 caches on the the whole L2 SRAM memory". SYS/BIOS reads the same field the other way round -
its `Cache_L2Size_512K` and `_1024K` are 5 and 6, for devices with that much L2, and its own
`Cache_startup` clamps anything above 256 KB - so the field counts sizes and then tops out. Every
encoding above 4 is now the megamodule's maximum: 32 KB of L1P, 32 KB of L1D, 256 KB of L2.
Booting linux-c6x on this model would have run with all three caches off.

**[x] The caches as a program differential.** *Done.* `--cache` programs every core's L1P, L1D and L2
to that maximum before the image runs, through the same configuration registers firmware writes,
and the program suite runs every program a second time with it and requires the same answer: the
same absence of a fault, the same halt, the same A4, the same CIO text and the same transmitted
frames. That is 360 `program:*:cached` cases over the 85 programs and their optimisation levels,
and it costs about two seconds of a run, because the compilation the suite already does is what
takes the time.

Two honest limits. The rerun exercises L1P and L1D thoroughly - allocation, eviction, the write
buffer and the snoop protocol against L2 SRAM, which is where every program's code and data live -
but reaches the L2 cache only in the programs that touch DDR2, since L2 caches external memory
only. And the budgets: a cold cache turns every first access into a miss, so the cached run gets
four times the program's cycle budget; the widest inflation measured across the suite is 1.9x, on
the shortest programs where the cold cache is most of the run, and the median is 1.05x.

One program opts out through its spec, and the reason is the interesting part.
`ethernet_external_coherence` configures L1D as a cache and maintains its DDR2 buffers with L1D
range operations alone, which is exactly what SPRU862B Table 2-2 asks of it on a device whose L2
is all SRAM. Configure L2 as a cache as well and the L1D writeback lands in L2 rather than in
memory, the EMAC reads what memory still holds, and the checksum changes - which is what the
hardware would do too. The expectation is tied to that program's own cache configuration, so its
spec carries `cache_rerun: false` and a `cache_rerun_source` saying why.

**[x] The L2 dimension of the cached program rerun.** *Done.* Every other program is linked into L2
SRAM, which L2 does not cache, so the rerun reached the L2 cache barely at all.
`l2_external_working_set` puts its working set in DDR2 instead and makes it 512 KB - twice the
256 KB the `--cache` rerun configures - so the loops that produce the answer are also the ones
that drive L2. It sets MAR224's PC bit itself, because without it there would be 256 KB of cache
nothing could allocate in. Four loops: a fill whose first half is evicted dirty before the pass
ends, so the data reaches DDR2 only through the write-back path; a read-back fold over words that
no longer live in any cache; a read-modify-write over eight words a 64 KB way stride apart, which
is one L2 set in a four-way cache and four L1D way strides, stepped by one L2 line so it sweeps
every set; and a fold of what that loop touched. The cached rerun reports 18432 L2 line fills,
16384 evictions and 8192 write-backs against 12288 read and 6144 write misses, where the rest of
the suite reports none.

The expectation is a host build of the identical C source through the `__TI_COMPILER_VERSION__`
split the intrinsic programs already use - the host branch puts the same working set in a static
array - and it agrees at every optimisation level on both compilers. The case is load-bearing
rather than merely green: dropping the last byte of every written-back L2 line moves the checksum
from 902724403 to 4006509363 and the uncached run of that same build still answers correctly, so
what the `cached` check catches is the L2 write-back and nothing else. Two optimisation levels
carry it, the straight-line and software-pipelined orderings of the same volatile loops, because
what this program varies is the cache configuration rather than the code generation.

**[x] What the cache configuration decides besides capacity.** *Done, and it found a second
defect.* Configuring L2, L1P or L1D as cache used to resize the cache object and nothing else, so
the SRAM the cache is carved out of stayed addressable. The address map is now a function of the
cache configuration, at the one seam every master crosses: `Backing::placement` resolves the
requester's aliases through the static map and then answers `ConvertedToCache` for a local offset
past the SRAM its megamodule has left, so the CPU, a DMA master, the snoop path and the image
loader all agree about what is no longer there.

The geometry is quoted rather than guessed. SPRU862B section 2.3: "when the L2 cache size is
increased, the memory is taken from the high memory addresses", and its Figure 2-1 puts the four
cache sizes in a 2048 KB L2 at 009F 8000h, 009F 0000h, 009E 0000h and 009C 0000h. SPRU871K
section 2.4: L1P "converts memory from RAM to cache by starting at the top of the L1P memory map
and working downwards", and section 3.3 says the same of L1D. What an access to those addresses
returns is the one thing no document in this set gives, so the model answers as nothing - reads
give zero, writes are dropped - which is the only claim TI's own Note licenses: that memory "is
not valid for the linker to place code or data in". It also stops holding what was written to it,
because SPRU871K section 2.4.3 names the consequence of growing a cache over live data as "data
loss" and SPRU862B Table 2-6 is the copy-out procedure that avoids it; keeping the bytes safe
behind the aperture would be the one behaviour silicon does not offer.

The second defect came out of the first case that tried to observe it. A write to an L1D address
the cache had taken still came back, because the L1D *cache* had allocated a line for it: the
model let a megamodule cache its own L1 memory. SPRU871K section 2.3.1 forbids exactly that -
"L1P memory can not be cached within Level 1 data (L1D) cache, Level 1 program (L1P) cache, or
Level 2 (L2) cache within the same megamodule" - and section 3.2.1 says the same of L1D memory,
so all three caches now refuse this core's own L1 windows and the aliases of them inside its own
global alias block. Another core's L1 through that block is ordinary memory to this one and stays
cacheable. The predicate is on the hottest path in the emulator, so it is one shift and four
compares against the windows' high bits rather than four range checks: both L1 windows are a
power-of-two size aligned to it. Measured over the 8 M cycles of `l2_external_working_set`, the
range-check form cost 7% and this one costs nothing.

Three cases, each pinning a different half. `memory:cache-configuration-carves-the-sram-aperture`
walks all four L2 sizes and the maximal encoding, requiring at each that the word below the
carve's base is still the SRAM it was, that the word at the base holds nothing, and that a store
to it does not stick - the third being the one that separates the aperture from the data loss, and
the one that failed until the own-L1 defect was fixed. Removing either carve arm from the model
fails it. `a_cache_size_moves_the_end_of_the_sram_it_came_out_of` does the same through a DMA
master over both device profiles, because the C6472's 608 KB L2 puts the same rule at a different
top and because "the CPU cannot read from L1P, even if L1P is memory-mapped" (SPRU871K section
2.3.1.2) leaves DMA as the only requester that can ask about L1P at all. And `l2_cache_carve` is
the deliberate overlap: its linker command file is SPRU862B Example 2-1 with the region the Note
forbids defined as well, one array linked into it, and five printed phases - all 64 words while L2
is all SRAM, none once the cache has taken them, none after a store, none when the cache is
released, all 64 after a store then. Its spec carries `cache_rerun: false`, because a program that
observes the transition has to start from the reset configuration.

## CPU core

**[x] Two writes landing on one register in one cycle.** *Fixed.* SPRUFE8B section 3.8.8 says the
assembler cannot detect this across execute packets and that "if the pipeline does receive
commands to perform multiple writes to the same register, the result is undefined" - there is no
exception for it. The model raised an internal exception with IERR.RCX instead, which broke legal
compiler output: cl6x issues a speculative load whose result is dead and overwrites the register
exactly when the load would land. Now the later-issued instruction wins, which is what that code
requires, and only two writes inside one execute packet - the case the manual calls easily
detectable, and the assembler rejects - still report a resource conflict.
`cpu:later-write-beats-a-landing-load` and `cpu:landing-load-beats-an-earlier-write` are the same
pair of instructions one cycle apart, so each ordering wins once. Found by running TI's SYS/BIOS
kernel: `xdc_runtime_Text_visitRope2` hits it, and until it was fixed every pass through that
function took an exception.

**[x] The kernel exerciser.** `firmware/bios_kernel` is the lever that found it, and the `rtos` domain
splits its one run into the contracts it establishes: task hand-offs through semaphores, nested
interrupt dispatch, a Swi and a task woken from interrupt context, an interrupt delivered after its
enable bit returns, periodic Clock callbacks with `Task_sleep`, and a Mailbox. Two more surfaces
were added afterwards and found nothing, which is worth recording as a negative result: an `Event`
whose and-mask needs one half posted by a task and the other from interrupt context, and a nested
`Hwi_disable` where the inner `Hwi_restore` must leave interrupts off because the outer disable
still stands - the pending interrupt arrives only when that one is undone too. The kernel surface
is now covered end to end, and so is the NDK's: its telnet daemon and console are the last of TI's
own source this suite had never executed, and `network:ndk-telnet-console` now holds a session
through them, and TI's own server daemons in `tools/servers` run beside them. Nothing of TI's
source is left untried here: the NDK's benchmark programs ship as manifests and spreadsheets with
no application source, which the server-daemon item above documents.

**[x] The long ADDA's functional unit.** *Fixed.* `adda_long` encodes `s` as the functional unit
where the long loads and stores encode it as the data path, so `ADDAW .D1X B15, ucst15, dst`
runs on .D1 and reads its B14/B15 base across the 1X cross path. The model treated all three
long forms as .D2, so TI's own `Hwi_dispatchAlways` packet - a shared 1X read by .S1 and .D1
next to a .D2 load - was rejected as a functional unit conflict, and SYS/BIOS reported
`IERR=0x10` from its exception handler. The long loads and stores also claim their T path now.
`cross_path_rules` pins the manual's three cases: the shared read is legal, two different 1X
operands are not, and neither is a third unit reading even the same operand.

**[x] When a register-sourced branch reads its register.** *Fixed.* `B .S2 src2` reads src2 in E1,
so a write landing during the branch's five delay slots cannot redirect it; the model sampled
the register when the branch resolved instead. TI's `TaskSupport_swap` branches through B4 and
reloads B3, A14 and A15 in those delay slots. `branch_register_issue` pins both halves.

**[x] The SPLOOP buffer's nested reload.** *Closed.* Two defects, both found through `DSP_autocor`.
The register a predicated `SPLOOP(D)` names is a reload *selector* read at the reload point,
not merely a nested-loop marker - binutils calls the field "use op field as a predicate address
register selector" and the compiler puts the outer loop's own branch in the same execute packet
on the same register, `SPMASKR || [B0] BNOP outer`, so the buffer replays exactly when the outer
loop takes another pass. Reloading unconditionally ran one inner loop too many on every last
outer pass. And the reload's program-memory overlap has to cover the outer branch's whole
five-cycle shadow: one cycle short and the packet that stores the second result word and resets
the accumulators lands after the replay has already summed into them, so every result from the
second outer pass reads back as zero.

The reproducer is worth remembering: shrinking from nx=128/nr=32 to nx=8/nr=4 gives one outer
and one inner iteration, where the whole body executes straight-line with no buffer replay at
all - which is what proved the corruption came from the spurious reload and not from the
buffered body. Two standing cautions: keep instrumentation out of hot paths (a committed
env-var check in the per-cycle path once slowed the binary past cycle limits and produced
phantom failures), and verify the build succeeded before interpreting probe output.

**[x] Bit-field instruction coverage.** *Closed.* `bitfield_regs` drives EXT, EXTU, SET and CLR in
their register form on both sides, across the cross path and under both predicate polarities,
plus the constant form at csta/cstb = 0 and 31, through hand-written assembly - cl6x lowers
`_ext`/`_extu` with a run-time field specifier to a SHL/SHR pair and never reaches the register
encodings. The model was already correct on all of them.

**[x] The unsigned compares' constant.** *Fixed.* CMPGTU and CMPLTU are the only L unit constant
forms whose src1 is unsigned - the binutils opcode table encodes opfields 0x4e and 0x5e with
`ENC(src1, ucst)` over the range 16 to 31, which is how the compiler realises a `x <u 16` loop
bound as `CMPGTU 16,x,dst`. The emulator sign-extended every 5-bit constant on that path, so the
bound became 0xfffffff0 and the compare answered "true" forever: an unoptimised counted loop with
a bound of 16 or more never terminated. Found by the new `edma_timer_event` firmware at `-O0`,
where the fill loop spun past its last element; the long-operand forms were already right.
`cpu:unsigned-compare-constant-{below,above}-the-bound` pin both sides of the bound.

**[x] A 32-bit NEG of the most negative int.** *Fixed.* The model saturated `SUB 0,0x80000000`
- every constant-zero SUB, which is the word NEG assembles to - to 0x7fffffff, reading the NEG
page's execution line, "0 -s src2", as the definition. The same word is SUB's .L opfield 0000110,
whose execution is plain `src1 - src2`, and TI's compiler settles which one silicon does: cl6x
negates an `unsigned int` with NEG, where C requires -0x80000000u to be 0x80000000u, and the high
word of every 64-bit negation is this instruction. Found from the Zephyr port, where cbprintf
printed INT64_MIN as -9223372032559808512 and two of upstream's JSON cases failed on it.
`cpu::tests::neg_of_the_most_negative_int_wraps` pins it; the 40-bit form is untouched, and the
entry below is about that one.

**[!] A known codegen corner for new `_lssub` programs.** cl6x realises a saturating long
subtraction as saturating `NEG` followed by `SADD`, which loses one ulp when the subtrahend
is exactly INT40_MIN. The host oracle disagrees with the compiled form there regardless of
emulator behaviour, so test programs should avoid that pairing or exclude the corner
explicitly in their `expected_source`.

**[x] Big-endian execution.** *Done.* ELF EI_DATA selects byte order for headers, symbols, code,
data, control-register CSR.EN, IDMA and EMAC descriptors. Header-based fetch packets preserve
logical compact-slot order even though the two halfwords exchange physical byte addresses.
`cpu:big-endian-load-and-fetch` checks raw data and CSR.EN; `big_endian` executes the same
hand-derived sum at `-O0` and `-O2`, including compact branch-heavy code; `big_endian_rts`
runs `_c_int00`, auto-initialisation and target-endian CIO `printf`.

**[x] Control register file.** *Complete for C64x+.* DIER implements its WSEL-protected
NMI/EXCEP/INT mask, TSCL write-enables the counter, a TSCL read latches the coherent high word
returned by TSCH, and DNUM reports core zero. `cpu:timestamp-latch-and-dier` checks the
counter, high-word latch and full DIER mask alongside the existing MVC/privilege cases.

**[x] Linked atomics.** *Done.* LL records the aligned word and its memory-write generation, SL
buffers a matching word without exposing it, and CMTL reports success after load latency and
commits only if no CPU, IDMA, EMAC or debugger write changed the reservation. The paired
`cpu:linked-atomic-*` cases cover a successful update and failure after an intervening STW.

**[x] Exceptions.** *Done.* `SWE`, `SWENR` and internal exceptions are delivered:
EFR/ECR, IERR, NRP and NTSR carry the documented context, `SWENR` enters at REP, the loop
buffer goes idle without an epilog and records SPLX in NTSR, and a nested exception saves into
the interrupt pair and vectors through the service table's RESET entry. `exceptions` checks
all of it against an independent model. Two facts came from linux-c6x rather than the manual:
NRP names the packet *after* a completed `SWE` but the offending packet itself for an internal
exception, and IERR is a plain read/write register. Building it found a real defect - every
error raised while decoding a packet's resources reported `IERR.RCX`, so an illegal opcode
claimed a resource conflict instead of `IERR.OPX`.

The external `EXCEP` input is done too: the megamodule exception combiner is modelled with
`EXPMASK0-3` and `MEXPFLAG0-3`, and the CPU takes the exception when `TSR.XEN`, `TSR.GEE` and
`IER.NMIE` all allow it, setting `EFR.EXF` and leaving the flag standing so a still-asserted
input cannot re-enter a handler that has not acknowledged it. `exception_combiner` drives an
unmasked event, a masked one and a later unmasking of the latched flag.

The interrupt selector resets INT4-15 to events 4-15 and implements INTXSTAT, INTXCLR and
INTDMASK. A second synchronized event arriving while its IFR bit remains set records the
dropped CPU interrupt and source event and raises INTERR event 96. EXPMASK resets fully
masked, while AEGMUX0/1 reset to events 0-7 and accept every documented trigger selector.
`cpu:dropped-interrupt-*` and `intc:reset-masks-and-aeg-mux` pin these paths.

**[x] How a routed event reaches IFR.** *Fixed.* The interrupt selector is fed by the raw system
event lines while the latched flags feed the event combiner (SPRU871K Figure 7-9), and a service
routine for a directly selected event needs neither to read nor to clear EVTFLAG (7.4.2). The
model used to announce the interrupt for as long as the *latched flag* stayed set, so every
handler that returned before writing EVTCLR was re-entered once: an EDMA completion routine ran
twice. Now the selector presents the level of the event this cycle - a level source holds its
line while it stays pending, a pulse source raises it for one cycle - and IFR latches on the
0-to-1 transition only, which is also what makes the dropped-interrupt case a second transition
rather than a persistent level. Found by `edma_interrupt`, whose ISR count is part of its result.
Fixing it also removed a panic: an interrupt pending on the exact cycle an SPLOOP started
underflowed the drain-offset arithmetic, which the same firmware hit at `-O2`. The
`ethernet_no_evtclr` firmware pins the promise from the other side: an EMAC receive driver that
acknowledges only the receive completion pointer, never EVTFLAG, still takes one interrupt per
frame.

NMI is delivered too through an edge-latched external pin. With GEE clear it follows the
interrupt pipe-down rules, including branch shadows and SPLOOP drain; with GEE set it is the
highest-priority exception and is not blocked by branches. Both paths save NTSR/NRP, clear NMIE
and vector through ISTP+0x20; exception mode additionally sets `EFR.NXF`. The two
`cpu:nmi-*context` cases check the distinct TSR contexts and flags against SPRUFE8B 5.4.5-5.4.7
and SPRU732J 6.1.1.

**[x] Privilege.** *Done for the CPU.* `TSR.CXM` is tracked, User mode is entered the only way the
architecture allows - returning from a context whose CXM is set - and the restored mode takes
effect at the branch target rather than in the delay slots, so a restricted instruction in a
`B NRP` delay slot is still legal. `MVC` naming a fully restricted register raises `IERR.RAX`
with `IERR.PRX` and does not perform the access; `B IRP`, `B NRP` and `IDLE` raise `IERR.OPX`
with `IERR.PRX`; CSR and TSR are partially writable, so a User write of all ones to TSR reaches
GIE and SGIE alone. `privilege` checks all of it against an independent model.

**[x] Memory protection.** *Done for CPU accesses and IDMA.* Each of L2, L1P and L1D has its
thirty-two per-page attribute registers, fault address, status and clear registers, and system
event. Refused reads, writes and fetches are discarded and latched. CPU accesses use LOCAL plus
the supervisor/user permission; IDMA accesses carry requestor ID 0 and require AID0 plus the
supervisor read/write permission. `memory_protection` drives a CPU write without SW, and
`memory:idma-obeys-aid0` proves a DMA write without AID0 is discarded with the ID in MPFSR.

The four-word protection key, exact KEYR/LOCK/UNLOCK sequence and lock status are modelled
for every memory unit; a wrong key leaves the lock engaged and raises the unit's protection
event. `memory:protection-lock-sequence` checks both failed and successful unlocks. EMAC
descriptor and buffer traffic now uses external-master AIDX, records FID 6 and raises each
unit's DMA-side protection event; `memory:emac-obeys-aidx` proves a denied transmit source
returns no protected data.

**[x] Which fault a protection block keeps.** *Fixed defect, and the method's sixth find.* Each
unit's fault registers hold one fault, and both halves of the model latched one - the CPU path on
every refused access, the DMA path only into empty registers - with nothing asking what the pair
does when faults of both kinds arrive at one block. SPRU871K 8.4.2.1.3 settles it: "The MPFAR and
MPFSR registers only store information for one fault... each individual memory generates no more
than one exception until the CPU clears that memory's MPFAR and MPFSR registers." So the CPU half
was wrong twice over - it replaced the recorded address on every later fault and re-raised the
event each time, which is a second exception for the same unserviced block - and the DMA half was
right by accident, because the one documented replacement runs the other way.

That replacement is a per-unit rule, quoted rather than generalised. L1P (2.9.2.3.3) "allows a
local fault to replace a remote fault and generate a new exception", and L2 (4.6.2.4.3) states the
same as a test on the register: "If the LOCAL field of the MPFSR register = 0, and the pending
exception sets it to 1, the hardware records the new fault and signals the new exception." L1D's
own section (3.7.2.3.3) carries neither sentence and only the general rule, so L1D keeps its first
fault whatever its source. `local_replaces_remote` is where that asymmetry lives, with both quotes
beside it; a single `Megamodule::record_mpu_fault` now decides for the CPU path, the DMA path and a
refused lock command alike, so the three cannot drift apart again.

MPFCLR was the third divergence and the reason the cases can be written at all. The model treated
a write as a mask over the status bits and left MPFAR alone, where the register has one command
bit: Table 4-40 gives bit 0 as "Clear the L2MPFAR and the L2MPFCR registers" and 8.4.2.1.3 adds
that "Writing a 1 to any other bit of the MPFCR register has no effect". A handler that wrote the
status value it had just read - which the `memory_protection` program does - therefore cleared
nothing on hardware and everything here.

Three vectors, each falsified against the rule it pins.
`memory:a-local-fault-replaces-a-remote-one` faults an L2 page from IDMA, clears the event flag,
then faults it from the CPU: the address and the LOCAL bit move to the CPU's fault and event 124
is raised again, a second CPU fault changes nothing, and a later IDMA fault neither replaces the
local record nor raises event 125.
`memory:l1d-holds-its-first-fault-whatever-its-source` is the same sequence on L1D, where the
local fault must not replace and must not signal, and the clear command re-arms the block.
`memory:fault-clear-takes-its-command-bit-only` writes the status value and a reserved bit and
requires both to do nothing. Removing the replacement fails the first, extending it to L1D fails
the second, and the old masked clear fails the third and the second.

What makes the vectors possible is one operation: a component vector's `write32` and `read32` are
the debug port, which no page attribute is allowed to refuse, so `cpu_access` asks the fabric for
a load, store or fetch as the core issues it and answers whether the page granted it.

## Timing

**[x] Cache geometry.** *Fixed defect.* All three caches replaced by capacity: a line could live in
any frame and eviction took the globally least recently used one, which is a fully associative
cache and none of these are. SPRU862B Table 1-2 has L1P direct-mapped, section 1.8 and the device
data have L1D two-way LRU and L2 four-way, so a line can only occupy the set its address indexes
and only that set's ways compete. Conflict misses exist now, and they are observable two ways.
`timing:l1d-two-way-set-conflict` loads three lines 16K apart into one set of a 32K L1D and then
reloads the first: four misses where the old model had three, against a companion case whose four
lines index different sets and whose last load hits. `cpu:l1p-direct-mapped-frame-holds-one-line`
uses the fact that L1P is never snooped: a program overwrites the ADD six words into its own first
line, falls through 4K of NOPs so the far line refills that frame, and branches back - the stale
copy runs exactly once, where a cache holding both lines would run it every time around.

Resizing needs care for the same reason: a size change re-indexes every set, so a line can be
left in a set that is now over-occupied even when the total still fits. `configure` evicts those,
which is why TI's own procedure for changing cache sizes has software write back and invalidate
first (Table 2-6).

**[x] L1D miss sources.** *Done.* The read-miss numbers were the L2 SRAM column applied to every
miss, but Table 3-2 costs a line by where it comes from: out of the L2 cache a single miss is
14.5 rather than 12.5, a pipelined one adds seven rather than three, a parallel one eight rather
than four, and the write buffer drains at six cycles an entry rather than two. All four now depend
on the source, and a cycle that misses on both sides pays the dearer line's price, since L2 serves
one request at a time. The table's fourth row also made the pipelined number conditional: M
consecutive *parallel* misses cost 12.5 + 4(M-1) out of L2 SRAM, which is four for every miss
after the first, not the 3 + 4 per cycle the model charged - with four banks a paired miss costs
what any other does, so the pipelining discount only exists while misses arrive one per cycle.
Six `timing:` cases pin the cache column, the parallel row and both drain rates. How the half
cycle in 12.5 is spent - as an alternation over successive misses rather than by the parity of
the issuing cycle - is the worked-examples item below, which is what got it right.

**[~] L1P miss timing.** *Done, and the number is defended rather than fitted.* SPRU862B gives no
per-miss penalty, which is why this stayed open, but its Table 1-3 gives something better: the
average stalls per execute packet for a straight-line miss stream at each packet width, thirty-two
numbers over two L2 configurations. One occupancy per line explains all of them. A 32-byte line is
one fetch packet; the fetch path holds one fill at a time and starts the next line as soon as it
goes free - what the manual calls miss pipelining - so a width-n packet consumes a line in 8/n
cycles and stalls `max(0, k - 8/n)` per line, which is `max(0, k*n/8 - 1)` per packet.

Which column is the C6455's is a question the device answers: its L2 is 2048K at 0080 0000h with
2-cycle latency in a 4 x 128-bit bank configuration (SPRS276M section 4), so the 1-wait-state
column - the same one the L1D read-miss numbers above come from. Solving against it gives k = 14/3
out of L2 SRAM and k = 6 out of the L2 cache, and those two reproduce all sixteen entries of that
column pair: measured over 384-packet streams the model returns 0.013, 0.177, 0.758, 1.339, 1.922,
2.505, 3.089 and 3.669 against the table's 0.000, 0.167, 0.751, 1.329, 1.915, 2.501, 3.079 and
3.665, and 0.016, 0.510, 1.258, 2.005, 2.755, 3.505, 4.255 and 5.003 against the cache column's
0.000, 0.499, 1.249, 1.999, 2.749, 3.499, 4.249 and 4.999. Every residue is the first line's fill,
which nothing overlaps, spread over the stream. The 14/3 is spent as a repeating 5, 5, 4, the way
the L1D read miss spends its half cycle as alternating 12 and 13; the average is documented, the
phase is not.

The accounting runs off the fills the fetch path actually performed rather than a residency
question asked before the fetch, because a C64x+ execute packet may span two fetch packets and so
fill two lines at once. Five `timing:l1p-*` cases pin short streams where that first fill still
shows, including the L2-cache cost and the L1P-as-SRAM baseline.

**[-] External memory latency in the miss path.** A line the L2 cache misses is charged as an L2
cache line, because Table 1-3 and Table 3-2 quantify the L2 columns and nothing else: they do not
reach past L2, and an invented number would be worse than an understated one. There is nowhere
else in the model for it to come from either, since the DDR2 and EMIFA models have no timing. The
number belongs to a memory part on a board rather than to the device, which is why this is
unplanned rather than merely unfinished.

**[x] The cache manual's own worked examples as timing cases.** *Done, and they found a defect.*
SPRU862B section 3.4.7 states its two routines' L1D cost three times over, and all three numbers
are now pinned by `the_manuals_vecaddc_read_miss_and_write_buffer_total` and
`the_manuals_dotprod_read_miss_total`: "64 misses x 12.5 stalls = 800 stall cycles" for vecaddc's
input array alone, "12.5 + ((12.5 + 1) x 63) = 863" once Table 3-4's write buffer interaction is
in the picture, and "128 misses x 12.5 cycles = 1600 stall cycles" for dotprod's 4096 elements.
The transcription is the loop the section describes rather than Example 3-10's register
allocation - a load packet reading 16 bytes of one array alternating with a store packet writing
16 bytes of another, 256 times, which is the manual's "eight results every 2 cycles" over 2048
elements: 512 execute cycles, 4096 bytes, 64 lines. The line counts came out exactly right
first time, 64 and 128, and so did Table 3-4's interaction: the difference the cases assert
between the two vecaddc runs is 63 cycles, one drain stall per miss after the first, with no
fourth-entry stall anywhere because the buffer fills at exactly the rate it drains.

The totals were 895 and 1664 against the manual's 863 and 1600 - high by exactly half a cycle per
miss, 32 and 64. The half cycle in Table 3-2's 12.5 was being read off the parity of the cycle the
miss issued in, which ties it to the loop's period: a loop whose misses arrive an even number of
cycles apart lands on the same phase every time and pays the dearer one at every miss. Only the
average is documented, so the model now spends it as an alternation over successive full-price
misses, exactly the way the L1P fill occupancy spends its third as a repeating 5, 5, 4. The
pipelined and parallel costs are whole cycles and take no phase. Both totals are integers under
that rule - 64 and 128 misses split evenly between the two phases - which is what makes them
expectations rather than ranges, and every one of the eleven existing `timing:l1d-*` and
write-buffer cases still passes, because a single miss on a fresh core is the first miss either
way.

This is what the item promised and the reason it was ranked first: neither rule was wrong on its
own - the miss costs were solved out of Table 1-3 and Table 3-2, and both were pinned - but no
case here had ever asked what the two of them do to a loop, and the answer was off by 4%.

**[~] The victim buffer.** *Half done, and honestly so.* SPRU862B Table 3-2 says a victim buffer
flush on a read miss "disrupts miss pipelining plus maximum 11 stalls" - a mechanism and a bound,
not a number. The disruption needs no number and is modelled: a fill that evicts a dirty line
clears the pipelining credit, so the next miss pays full price. The flush's own stalls are not
charged, because "maximum" is not a value. `a_victim_buffer_flush_disrupts_miss_pipelining` pins
the modelled half by running the same 130 instructions twice, once with the stores hitting the
lines a load allocated and once with them aimed at L1D SRAM, so the only difference between the
two runs is whether the eviction has anything to write back.

**[-] The victim buffer flush's own stalls.** The unmodelled half of the item above. Eleven charged
as a cost would be an upper bound presented as a measurement, and no other table in the manual set
counts a flush, so the number would have to come off silicon. The half with a mechanism behind it
is already charged, which is what makes this an inventory entry rather than unfinished work.

**[x] What an L1D victim does to L2.** *Fixed defect, and the method's fourth find.* An eviction
writes a dirty L1D line down, and L2 allocates a line on a write that misses it. Both halves were
modelled and pinned - `a_victim_buffer_flush_disrupts_miss_pipelining` for the eviction,
`l2_external_working_set` and the six-cycle drain vector for L2 allocation - and composing them
sent every victim through the CPU's own write path, so a victim whose line L2 had already evicted
allocated it again. SPRU871K section 4.3.8.3 gives victims a policy of their own, in three parts:
"L1D victim writebacks do not trigger line allocations in L2. L1D victims are written directly to
external memory if they miss L2. L1D victim writebacks also do not update L2's LRU if they hit in
L2. They do update L2's dirty status as needed." All three now hold: `Cache::write_victim`
refuses a miss so the caller takes the line past L2 to memory, and on a hit it updates the data
and the dirty bit without touching the replacement clock.

The cost of getting it wrong was invisible in every answer and plain in the cache: over
`l2_external_working_set` the L2 cache was making 2048 line allocations the hardware never makes
and evicting 2048 lines it would have kept - 18432 fills against 16384, 16384 evictions against
14336 - while the program's checksum came out the same either way. That is exactly the shape this
method keeps finding, and it is why the case for it counts rather than compares: `cache:` has no
way to see an allocation, so the pin is two unit tests.
`a_victim_writeback_neither_allocates_nor_promotes_the_line_it_lands_in` takes the three sentences
one at a time, and `an_l1d_victim_that_misses_l2_does_not_allocate_a_line_there` composes them
through the real hierarchy. The second one took the geometry rather than a big loop, which is the
part worth remembering: a linear sweep never reaches the state at all, because the L1D victim
trails the read pointer by an L1D capacity and its line is therefore always still in L2. An L2 set
spans two L1D sets - the L2 set field is bits 12:7 and L1D's is bits 10:6, so only bit 6 is free
inside one L2 set - so conflicting in L2 through the far half of each line evicts the victim's L2
line while leaving its L1D line alone.

**[x] The victim's LRU rule, seen from outside the model.** *Done.* The item above closed with two
unit tests and a stated gap: nothing observed the LRU half of that policy from a case, so a
regression that promoted a victim's L2 line would have shown up nowhere a program can look. The
channel it needed is the one the coherence cases already use - a DMA master reading external
memory bypasses both caches, so what the wire carries is exactly what has been written back.

`cache:an-l1d-victim-does-not-promote-the-l2-line-it-lands-in` puts a 60-byte frame in DDR2,
configures 4 KB of L1D and 32 KB of L2, dirties one word of the frame in L1D, and then uses the
geometry the unit test worked out: three conflicting reads through the far half of each line fill
the victim's L2 set without touching its L1D line, so the victim's L2 line is the set's least
recently used one when the writeback lands on it. Two more reads evict the victim from L1D, one
more read allocates a fifth line in that L2 set, and the MAC then transmits the frame out of DDR2.
Under the rule the evicted line is the victim's own, so the frame carries the stored word; promote
the line on the writeback and a clean line goes instead, the dirty one stays in L2, and the frame
carries what was loaded. Adding a clock update to `Cache::write_victim` fails the case and nothing
else in the cache domain, which is what makes the frame the assertion rather than decoration.

**[x] Which write misses the write buffer merges.** *Fixed defect, from the same pair.* The buffer
merged any write into any entry holding the same 128-bit quad-word. SPRU871K section 3.5.3 allows
that only under six conditions, and two of them are about which pair of writes is eligible rather
than the address: "The new write miss resides within the same 128-bit quad-word as the immediately
preceding write miss" and "The two writes are to locations in L2 SRAM (not to locations that may
be held in L2 cache)". The second is where this meets L2 cacheability - the rule that already
decided the drain rate, pinned by the six-cycle vector - and it is the one that costs cycles:
four stores to one quad-word in a cacheable external address take four entries at six cycles, not
one. `timing:l2-cache-addresses-never-merge-in-the-write-buffer` is the case, derived rather than
fitted: it is the six-cycle vector's own program with the four store addresses moved into a single
quad-word, so it must cost what that vector costs. Under the old rule it does not, and every
existing write-buffer pin passes either way.

The immediately-preceding condition is obeyed too and deliberately not pinned, which is worth
writing down because a case for it was written and then deleted: it passed under both rules.
Merging changes the queue's depth, and depth changes a cycle count only when the drain rate is
slower than the fill rate - the six-cycle L2 cache rate, which is precisely where merging is
forbidden. At the two-cycle L2 SRAM rate the buffer drains as fast as stores arrive, so one entry
more or fewer costs nothing. A case that cannot fail is worse than no case, so the reason lives in
the code instead.

**[x] Interrupt latency out of a loop.** *Pinned.* A hand-written II=1 loop with a one-cycle
body gives an unambiguous zero-cycle epilog. `timing:sploop-interrupt-pipe-down-and-up` raises
INT5 well inside that loop and accounts independently for the cycle from detection to drain
start, nine-cycle entry, six-cycle `B IRP` service packet and re-executed SPLOOP packet. It
reaches the post-loop marker in 119 cycles versus 102 without interruption, while preserving
all one hundred loop iterations. `sploop_timer_interrupt` checks the same contract on
compiler-generated code: a dot product that cl6x pipelines into an SPLOOP kernel at -O2 and -O3
is interrupted repeatedly by TINTLO0 and still sums exactly one pass per iteration.

**[~] Contention stalls.** *Registers done, arbitration timing open.* CPUARB, IDMAARB, SDMAARB,
UCARB and MDMAARB exist at every documented L1D/L2/EMC address with their reset values and
writable fields; `memory:bandwidth-arbitration-registers` checks all twelve. IDMA advances in
the background, but simultaneous CPU, IDMA and external DMA requests do not yet arbitrate for
L2 cycles, so throughput-measuring firmware still sees optimistic numbers. What is missing is not
the arbitration rule - SPRU871 gives it, a bounded wait counted in the requestor's MAXWAIT units -
but the cost of the access being arbitrated for: no table here says what one DMA access to L2
costs the CPU, and without that the rule produces no cycle count worth asserting.

**[-] L2 arbitration timing between the CPU and the DMA masters.** The unmodelled half of the item
above. One number would unblock it - what a DMA access to L2 costs the CPU - because SPRU871's
bounded wait follows directly from it. `MAXWAIT` appears in no source file for exactly that reason:
the registers hold it, nothing spends it.

**[-] The tag update buffer.** SPRU862B section 3.2.1 notes that a stream of write hits turning clean
lines dirty at a high rate stalls, because the clean-to-dirty transitions queue to L2's shadow
copy of the L1D tag RAM - the same shadow copy the snoop protocol needs. It is a documented
mechanism with neither a rate nor a depth attached, so it is inventory rather than a plan; the
tables that made the L1P and L1D miss costs tractable have no equivalent for this one.

## Memory system

**[x] IDMA.** *Done functionally.* Both channels expose their documented register files, active and
pending status, completion events and invalid-parameter event. IDMA0 copies masked 32-word
configuration blocks; IDMA1 copies or fills word-aligned local memory in the background. DMA
accesses snoop dirty cache data and invalidate destination lines. Five cases cover block copy,
fill, masked peripheral configuration, invalid parameters and AID0 protection, and the
`idma_interrupt` firmware runs both channel-1 modes from C, each waiting for its own IDMAINT1
completion interrupt routed to CPU interrupt 7.

**[~] Cache controls and coherence.** *Done functionally.* Block and global writeback/invalidate
registers update line state, and L1PCC/L1DCC freeze modes continue servicing resident hits while
blocking allocation and report the prior OPER state. IDMA snoops dirty sources and invalidates
destinations; external DMA bypasses stale CPU lines. `cache:l1d-freeze-blocks-allocation` and
the paired `cache:emac-*dirty-buffer` cases pin both contracts, and the `gpio_edma_cache`
firmware drives them from C: dropping its L1D writeback makes the EDMA3 copy stale words and
dropping its invalidate makes the CPU read cached zeros, so its checksum checks both. Large-region
operation timing and simultaneous CPU/DMA arbitration remain timing-only gaps: the arbitration half
is the timing item above, the other half is the item below.

**[-] Large-region cache operation timing.** A block or global writeback/invalidate updates every
line it names in the cycle it is issued. Which lines it touches and what state they end in are
modelled and pinned; how long the operation takes is not, and no table in this project attaches a
duration to a range operation. SPRU862B says what each operation does and which one a program must
issue (Table 2-2); it does not say what one costs, and a global operation over 256 KB of L2
plainly costs more than nothing.

**[~] External memory.** DDR2 and EMIFA windows are functionally backed, and MAR PC now gates
allocation in L1P, L1D and L2 together: uncached external reads observe DMA immediately, while
cached reads require coherence. `cache:mar-controls-l1-allocation` pins both worlds. The DDR2
controller's register file is implemented too - module identity, the interface-ready status that
follows IFRESET, the endianness it inherits, the configuration register with its boot and timing
locks, both timing registers, the burst priority and the refresh control that substitutes twice
the refresh cycle time for a rate below 100h - and `memory:ddr2-controller-registers` pins it.
DDR2 and EMIFA latency and refresh remain unmodelled, so timing studies that reach off-chip are
optimistic. The EMIFA register file is present as well - module identity, the status register with
its endianness bit and its always-one bit, the burst priority, the four chip-enable
configurations at their slowest documented strobes, the asynchronous wait configuration and the
time-out interrupt with its mask set/clear pair - pinned by `memory:emifa-controller-registers`.
Nothing external holds ARDY on this model, so the time-out itself has no injectable source.

**[-] DDR2 and EMIFA access latency, refresh and ARDY.** Both register files are implemented and
pinned and both windows are functionally backed, so the wait-state, turnaround and refresh fields
are stored and reported without changing anything observable, and the EMIFA time-out has nothing
to fire on. The latency belongs to a memory part on a board rather than to the device, and the
thing that would hold ARDY or make a refresh cost cycles is the external bus device in *What is
still not modelled*. Timing studies that reach off-chip stay optimistic until then.

**[~] Power management.** *Done functionally.* PDCCMD exposes MEGPD with its reserved reset ones,
IDLE already halts fetch until an enabled interrupt, and L2PDWAKE/SLEEP/STAT track the four
logical L2 pages. Power state does not add wake latency; the registers and command/status
transitions are pinned by `memory:power-down-control-registers`.

**[-] Power-down wake latency.** Entering or leaving a power state costs nothing here: the command
and status transitions are modelled, the time they take is not. No document in this project's set
attaches a cycle count to the transition, so the cost would be invented, and no case would change
answer if it were.

**[x] EDMA3.** *Channel controller, transfer controllers, event queues and chaining done.* The
EDMA3CC identification and configuration registers, all 256 PaRAM sets, the DMA and QDMA
channel maps, the event, event-enable, secondary, missed, chained, interrupt-enable and
interrupt-pending register pairs and the region access registers are implemented. A trigger
submits one transfer request per tick: A-synchronised requests move one array, AB-synchronised
requests a whole frame, the parameter updates and link reload follow Table 2-5, and a null set
reports a missed event. QDMA channels trigger on a write to the trigger word their QCHMAP names
- including the write a link update performs - chaining queues the channel its completion code
names, the eight shadow regions view the same physical registers through DRAE and QRAE with ICR
left unfiltered, and the completion and error interrupts reach the megamodule as events 24,
71-78 and 79. All four transfer controllers report their identity, the FIFO geometry they were
built with and an idle channel status, latch MMR address errors and the transfer request errors
the channel controller hands them - a constant-addressing transfer that is not 256-bit aligned
moves no data - and pulse their own error interrupts, events 81 to 84. The four event queues are
real: a channel's DMAQNUM field decides which queue its trigger joins, each entry records the
trigger type and channel number for the debug windows, QSTAT reports the head, the occupancy
and the high-water mark, QUEPRI decides which queue is served first, and reaching a QWMTHRA
threshold raises the queue threshold error. Active memory protection is in place too: the page
attribute registers guard the global range, each shadow region's window and its PaRAM octant,
a refused access leaves the register alone and records its address and the failing permission in
MPFAR and MPFSR behind the memory protection interrupt, event 80, and a PaRAM write stamps the
set with its programmer's privilege and requestor id. Eleven `memory:edma-*` cases pin all of
it, and the `edma_timer_event` firmware drives the whole path from C at every optimisation level:
it maps channel 1 onto a PaRAM set, arms timer 0 in continuous mode and waits for the completion
bit its OPT names, so the checksum it returns can only come from the timer event reaching the
channel controller, and `edma_interrupt` takes the same completion as a real CPU interrupt routed
through INTMUX1 to INT5, counting the entries into its service routine.

**[x] The transfer controllers' debug windows.** *Done.* SPRU966C §2.12.4 says the source-active and
destination-FIFO register sets exist "to derive a brief history of TRs serviced through the
transfer controller", and that is what they now report. A serviced request leaves its parameters
behind: SAOPT and DFOPT carry its chaining, interrupt, completion code, FIFO width and
addressing bits plus the priority QUEPRI gave its queue; SACNT and DFCNT read zero because a
completed request has nothing left to move; SACNTRLD and DFCNTRLD hold the original ACNT; SASRC
and SASRCBREF have walked to the last array read, DFDST and DFDSTBREF to the last array written;
SAMPPRXY and DFMPPRXY report the privilege level and requestor id the PaRAM set was stamped with;
and SADST, SADSTBREF, DFSRC, DFSRCBREF and the SABIDX and DFBIDX halves the manual calls not
applicable read zero. The destination FIFO is the circular buffer of §2.12.4.1: DFSTRTPTR
advances one entry per request and wraps at DSTREGDEPTH, so TC0 keeps the last two requests and
the others the last four, and the activity fields PROGBUSY, SRCACTV, WSACTV and DSTACTV stay
idle, which for SRCACTV is documented as the set holding a previously processed request. Three
`memory:edma-*window*` and `-destination-fifo-*` cases pin the fields, the wrap on TC0 against
the fourth entry on TC1, and constant addressing freezing the source pointer at the start.
One granularity note in the model: the manual defines the count and pointer registers in terms
of read and write commands, and this model issues one of each per array.

**[-] Synchronisation events from peripherals this model does not have.** An EDMA3 channel can be
triggered by any peripheral's event, and every peripheral modelled here drives its own - the
timers, GPIO, I2C, McBSP and UTOPIA events all reach the channel controller and are pinned there.
The events still unassigned belong to devices that do not exist on the model, the coprocessors and
the external bus devices in *What is still not modelled*, so the gap is those device models rather
than anything in the channel controller.

**[x] Chaining driven from firmware.** *Done.* `edma_chaining` links three channels: one manual
trigger of channel 1 sets the CER bit its TCC names, which the controller serves as a trigger for
channel 2, which chains to channel 3, and only the last link asks for a completion interrupt. The
block arrives three stages later with CER clear again, because a chained event is consumed when
its channel is served, and clearing TCCHEN on the first link changes the checksum - the chain is
what moves the data, not the trigger.

**[~] 64-bit timers.** *All four modes done.* Both timers expose EMUMGT_CLKSPD with the documented
CLKDIV of six, the counter and period pairs, TCR, TGCR and WDTCR. They count on SYSCLK3, the CPU
clock divided by six, as a 64-bit up-counter, as dual 32-bit timers with the high half behind its
four-bit prescaler, as dual 32-bit timers chained - where the high half is a 32-bit prescaler for
the low one, resetting to zero the cycle after it matches its period and stepping the timer then,
which `interrupts:timer-chained-prescaler-worked-example` pins against the manual's own worked
example - and as a watchdog. A period match raises TSTAT for the programmed pulse width or toggles
it in clock mode, stops a one-shot half, and drives both the CPU interrupt (events 67 to 70) and
the EDMA3 synchronisation event (channels 1, 2, 16 and 17).

The watchdog is the state machine of SPRU968 Figure 12: WDEN with the first service key moves it
to the pre-active state, where every register but WDKEY is write-protected; the second key
activates it with its counter and flag cleared; the sequence A5C6h then DA7Eh restarts the count;
and anything else written to WDKEY times it out on the spot. A timeout sets WDFLAG, clears WDEN,
resets the counter and raises the timer interrupt, and the watchdog cannot be enabled again
without a hardware reset - which is what `interrupts:timer-watchdog-service-and-timeout` and
`interrupts:timer-watchdog-wrong-key-times-out-at-once` pin. Before the active state a period
match is only an interrupt, exactly as the manual's note says. The device reset a timeout can
drive on the TOUTL pin is still outside this model - the pins-as-signals entry in *What is still
not modelled*.

**[x] GPIO.** *Done.* All sixteen pins, the direction register with its all-inputs reset state, the
output data register and its set and clear windows, IN_DATA reading the pin rather than the
register, the internal rising and falling edge triggers behind their four windows, and BINTEN
gating the lot. The edge detector samples on the peripheral clock - the CPU clock divided by
six - so a software toggle of an output raises GPINT*n* exactly as an external signal would:
CPU events 51 to 66 and EDMA3 channels 48 to 63.
`interrupts:gpio-output-edge-raises-its-event` and `memory:gpio-event-triggers-edma-transfer`
pin both paths. Input pins now have a driver: an external agent holds a level on them, which
reaches only the pins configured as inputs, because IN_DATA reads the pin and an output pin is
driven by the device.  `interrupts:gpio-input-edges-from-an-external-driver` takes both edges of
a driven pin through the detector, and `memory:gpio-output-pin-ignores-an-external-driver` shows
the direction register deciding who owns each pin.

**[x] I2C.** *Done, both directions, against a device on the bus.* The
whole register file is implemented with its documented reset values, write-one-to-clear status
flags and the IRS software reset that returns them to those defaults. A master START loads the
internal data counter from ICCNT, marks the bus busy and clears STT; a byte written to ICDXR
leaves the register after the documented (prescaler + 1) x 8 DSP cycles, and in digital
loopback mode it arrives in ICDRR with the receive flag, the overrun flag when the previous
byte was never read, and the receive event. The counter reaching zero reports a STOP with STP
set and register-access-ready without it, ICIVR reports and clears the highest-priority masked
interrupt, and the module drives CPU event 16 and EDMA3 channels 44 and 45.
`interrupts:i2c-digital-loopback-and-vector` and `memory:i2c-receive-event-triggers-edma` pin
both paths.

A device can now be attached to the bus at one address, which is what makes the master a real
one rather than a transmitter shouting into an empty bus. It acknowledges its own address, so a
byte the master writes draws no NACK and arrives at the device; it answers a master read - MST
set with TRX clear, which previously clocked nothing at all - with the bytes it holds, one per
(prescaler + 1) x 8 cycles, and 0xff once it runs out, which is what an open bus reads as. The
receiver stalls a full ICDRR rather than overwriting it, ICCNT paces both directions and STP
ends the transfer the same way in each. `memory:i2c-slave-acknowledges-a-master-write`,
`memory:i2c-master-read-takes-the-slave-bytes` and `interrupts:i2c-unanswered-address-nacks`
pin the three outcomes, the last of them showing that an address no device answers still NACKs
and clocks nothing in, in both directions, until ICSAR names the device that is there.

**[~] McBSP.** *Both ports done, in digital loopback.* The register file and its bit layouts come
from the CSL register appendix (SPRU401J Tables B-210 to B-224) and the addresses from SPRS276M
Tables 7-57 and 7-58, so each port answers on its configuration bus and DRR and DXR answer again
on the EDMA data bus - which the DMA path now reaches, as it does the UTOPIA queues, because those
windows sit outside the configuration space and were previously served as plain memory. What moves
data is digital loopback, the one path a port can run with nothing on its pins: the transmit
serialiser feeds the receive serialiser, paced by the sample rate generator's CLKG, which is the
CPU clock divided by CLKGDV + 1 while GRST is out of reset. The double buffering is the documented
pair - DXR hands its word to XSR when the shifter is free, which clears XRDY and sets it again, and
RSR hands its word to DRR, setting RRDY or RFULL when the previous word was never read - and RJUST
justifies a short word three ways. XEVT and REVT are EDMA3 events 12 to 15 and XINT/RINT system
events 40 to 43, the ready flags driving them in interrupt mode 0. Four cases pin a sixteen-bit
word taking one serial clock per bit, the word lengths with justification and the overrun, the
ready flags reaching the CPU with a generator in reset moving nothing, and TI's own driver pattern:
the EDMA servicing the port over the data bus, one word per event, after a single manual kick.
Multichannel selection through MCR and the channel-enable file, mu-law and A-law companding and the
frame-sync generator's period and width are stored and decide nothing, because each needs a framing
device on the pins: that is the entry in *What is still not modelled*.

**[~] HPI.** *Both sides done.* The CPU's view is unchanged - HPIRST set out of reset, a write of one
to HINT raising the interrupt to the host, a write of one to DSPINT acknowledging the host's, and
read-only data and address registers - and the host's side is now modelled as the four access
types the HCNTL pins select rather than as pins. What that makes observable is all pinned:
word-addressed HPIA registers with their autoincrement, single-HPIA mode moving both registers
against dual-HPIA mode keeping the directions apart, the eight-word FIFOs with their four-word
bursts - three posted words stay in the write FIFO and the fourth commits all four - the flush
that a non-autoincrement access forces, the two interrupt handshakes with their zero-to-one and
write-one-to-clear rules, and the software reset holding every host access off until the CPU
releases it. The HPI's DMA goes through the same path every other master uses, so a host write
into L2 SRAM is snooped into a line the CPU is holding, which `cache:hpi-write-snoops-a-resident-l1d-line`
pins. A process on the host drives all of it: `--hpi-unix` gives the run loop a datagram socket where
one datagram is one host access, and `tools/hpi_host.py` uses it to exchange a mailbox with
`firmware/hpi_mailbox`, which takes DSPINT as a real CPU interrupt through INTMUX and answers with
HINT. The `host` domain asserts the answers, their order and the firmware's own count.

The 16-bit multiplexed mode is modelled too: a host cycle is two halfword transfers, HHWIL low
for the first and high for the second, and HWOB decides which end of the word the first one
carries. `memory:hpi-16-bit-halfwords-most-significant-first` and its least-significant sibling
drive HPIA and HPID both ways and read a word back out, which also pins that the transfer happens
once per cycle: a read's second halfword comes from the word the first one fetched, not from a
second DMA access. Still open: the read-burst timeout counter, which nothing on this model holds
HRDY long enough to reach - the entry in *What is still not modelled* says why a case could only
assert that it never fires.

**[~] UTOPIA 2.** *Done, including the cell path against an external ATM master.* UCR keeps its
reserved one above BEND and comes up multi-PHY, CDR starts both clock
watches at 255 peripheral clocks, and EIER/EIPR pair up with only the four clock bits
write-one-to-clear. Because nothing drives UXCLK or URCLK here, the clock-present bits stay clear
and an enabled section with a non-zero count always fails its detector after that many SYSCLK3
cycles, which is CPU/6; the failure reaches CPU event 36 only once EIER allows it. The transmit
queue has space for a cell, so UXEVT keeps raising EDMA3 channel 40, and a read of the empty
receive queue records the RQSP stall that only the interface can clear.
`interrupts:utopia-clock-failure-and-transmit-event` pins all of it, and the `utopia_clock_isr`
firmware services three failures in a row from C: clearing the pending bit in the routine is what
lets the next one be announced, so the count it returns also checks the interrupt rule.

The cell path is modelled too, against an external ATM master that polls the transmit queue and
offers cells to the receive queue. Both queues hold two cells - SPRS276M section 7.19.1 gives the
depth the peripheral guide leaves out - a standard cell is 53 bytes in a fourteen-word packet, and
XUDC/RUDC add one to eleven header bytes, taking the packet to fifteen or sixteen words. The
events follow SPRUE48 sections 3.3 and 3.5: UXEVT is asserted while the queue has room for a whole
cell, which the packet being written does not leave, so the next cell's transfer can start before
the current one finishes; UREVT is asserted while a complete cell waits and drops on the first
read of one. A write to a full queue is held rather than lost - the manual says the write stalls
until space appears and that data is not overwritten - and XQSP reports the outstanding stall
until the held word lands. UCR.BEND decides which end of a word the first wire byte comes from.
Four `memory:utopia-*` cases pin the two directions, the stall and its release, the two-cell
limit refusing a third cell, and the user-defined cell lengths. What the cell path does not need
and does not model is the multi-PHY address phase - SLID is stored and reported, and this master
addresses its one slave directly - which is the entry in *What is still not modelled*.

**[~] Device configuration.** *Done.* The chip-level block reports DEVSTAT with the straps a board
that populates this memory map would set - EMIFA and DDR2 enabled, LENDIAN following the loaded
image's endianness - the JTAG ID of silicon revision 3.1 (SPRZ234R Table 3), and PRI_ALLOC with
its read-only reserved field. The device-state block implements the PERLOCK key that opens one
PERCFG0 write inside sixteen SYSCLK3 cycles, PERSTAT0/PERSTAT1 reporting enable- and
disable-in-progress until that transition finishes, PERCFG1's write-once EMIFACTL and DDR2CTL,
EMACCFG's RMII_RST and EMUBUFPD. `memory:device-configuration-and-peripheral-lock` pins the whole
sequence. Peripheral register access is not gated on PERCFG0 here and RMII_RST reaches nothing,
for the reasons the two items below give; the enable that does gate this device's wire is
MACCONTROL.GMIIEN, and it is modelled.

**[x] What a peripheral enable decides.** *Done for the one gate the manuals put on the wire, and
it was not the one this item named.* The claim was that RMII_RST is the tractable half - "a MAC
held in reset should transmit and receive nothing". Reading the register says otherwise. SPRS276M
section 3.4.5: EMACCFG "is used to assert and deassert the reset of the Reduced Media Independent
Interface (RMII) logic of the EMAC", and Table 3-11 gives bit 18 as "RMII reset bit. This bit is
used to reset the RMII logic of the EMAC", 0 released and 1 asserted, reset 1. It is not a MAC
reset, and TI's own initialization sequence agrees: SPRU975E section 2.15.4 step 19 is "When using
RMII, release the interface logic from reset by clearing the RMII_RST field". This device is not
using RMII - MACSEL[1:0] on AEA[10:9] selects the interface and both pins are internally pulled
down, which SPRS276M Table 3-1 reads as "10/100 EMAC/MDIO with MII Interface [default]" - so
RMII_RST has no logic here to hold and stays stored and reported.

The gate that does decide something on this interface is one register away, and the model was
ignoring it. SPRU975E Table 57 gives MACCONTROL.GMIIEN, reset 0, as "GMII RX and TX are held in
reset" for 0 and "GMII RX and TX are enabled for receive and transmit" for 1, and section 2.15.4
separates it from the DMA enables on purpose: step 12 says "do not set the GMIIEN bit yet" and
step 17 "Enable the receive and transmit DMA controllers by setting the RXEN bit in the RXCONTROL
register and the TXEN bit in the TXCONTROL register. Then set the GMIIEN bit in MACCONTROL." Two
gates, and the model had only the inner one: GMIIEN existed but decided nothing except whether a
statistics write decrements. It now holds both directions, and pause-frame generation with them,
because a pause frame goes out over the same interface. Channel teardown still completes, since
that is a DMA command rather than wire traffic. Frames already in the receive FIFO stay there
rather than being dropped, which is the same position the FIFO takes everywhere else in this
model: it has no wire clock, so it buffers.

`emac:gmiien-holds-both-directions-in-reset` pins it from both sides. A transmit descriptor and a
receive descriptor are both ready and both DMA controllers enabled - everything TI's sequence asks
for short of step 17 - and after a tick nothing has left, nothing has arrived, both descriptors
are still owned by the MAC and the receive buffer is still zero. Set GMIIEN and the next tick
moves both. Making the gate unconditional fails the case.

The cost was twenty-six cases, and what it says about them is the interesting part: twenty-six
component vectors moved frames without ever writing MACCONTROL, so they were asserting correct
answers from a configuration the interface does not run in. Each gained one `MACCONTROL = GMIIEN`
write where TI's sequence puts it, after the DMA enables - and one of them needed a second, after
its own SOFTRESET, because section 2.14.1 says "After a software reset operation, all the EMAC
registers need to be re-initialized". All twenty-six are inside the frozen Python migration
record, which is why this is written down: every case id and every expectation was unchanged,
so the amendment was to the setup and not to an answer. That historical record remains in
`tests/vectors/component-python-baseline.json`; the current per-case vectors are executable.

**[-] PERCFG0 gating.** The registers are modelled and pinned - the lock sequence, PERSTAT0 and
PERSTAT1 with the five documented states, the write-once EMIFACTL/DDR2CTL - and what the control
bits decide is still nothing. Two reasons, and the first is not the one this item used to give.
SPRS276M chapter 3 says what the disabled state *is* - Table 3-4: "Peripheral is held in reset and
clock is turned off. Default state for all peripherals not in static powerdown mode" - and it says
every peripheral starts there: "All peripherals on the C6455 device are disabled by default,
except when used for boot, and must be enabled through software before being used." What it never
says is what a register access to one of them does. There is no sentence about a stalled bus in
this manual; that was a guess, and the honest statement is that the behaviour is undocumented.
Second, gating on it would mean every program and every firmware image in this suite growing the
PERLOCK/PERCFG0 sequence before it touches a timer, the I2C, a McBSP, the GPIO or the MAC - which
is what silicon requires and what the model would then require, but it is a change to every
target in the suite rather than to the device. The GMIIEN work above is the part of this item that
had a documented answer behind it.

**[x] DMA access to peripheral registers.** *Fixed.* A DMA master reading or writing inside a
peripheral aperture now goes through that peripheral's register, with the side effects a CPU
access would have, instead of the memory map behind it. That is what lets an EDMA3 channel
drain ICDRR on the I2C receive event, which the `i2c_edma_drain` firmware does from C: four bytes
go through the loopback path and reach memory without the CPU copying them.

**[~] PLL1 controller.** *Register behaviour done.* PLLCTL with its fixed bits and the PLL held in
reset, PLLM, PREDIV, the two programmable dividers PLLDIV4 and PLLDIV5 with their reset ratios,
the read-only ALNCTL and SYSTAT this device fixes, DCHANGE flagging a changed ratio, and the GO
operation applying exactly the flagged dividers and clearing the flags.
`memory:pll-controller-go-operation` pins the sequence SPRUE56 section 3.2.1.2 prescribes. The
clock ratios have no further effect here: the megamodule and the peripherals hang off the fixed
/3 and /6 dividers, and nothing in this model is clocked from SYSCLK4 or SYSCLK5. The reset
controller's RSTYPE register reports the power-on reset this model always comes out of, and the
PLL2 controller - the fixed x20 multiplier plus divider D1, which clocks the EMAC - implements
PLLDIV1, the command and status registers, ALNCTL, DCHANGE and SYSTAT with the same GO protocol.
`memory:pll2-controller-and-reset-type` pins both.

**[-] Clock ratios that change a clock this model counts.** PLLDIV4, PLLDIV5 and PLL2's D1 are
programmable and their GO protocol is pinned, but the megamodule and every peripheral here hang off
the fixed /3 and /6 dividers, so nothing this model counts is clocked from SYSCLK4 or SYSCLK5.
Making the ratios load-bearing means giving each peripheral its own clock and re-deriving its
timing from it, and no case in the suite would change answer - work with nothing to check it.

**[-] A reset other than power-on.** RSTYPE reports the power-on reset this model always comes out
of, because there is no source for any other kind. The two candidates are pin paths in *What is
still not modelled*: the watchdog's device reset on TOUTL and the EMAC's RMII_RST.

**[~] Boot tables.** *Table processing done; the ROM code itself is not emulated.* The boot table's
entry-point record, its count/address/data sections and the zero count that ends it are processed
exactly as SPRUEC6G section 6.2.2 describes, and a boot configuration table's address/set/clear
triples set, clear and toggle through ordinary CPU accesses, so an entry can configure a
peripheral register. An entry with two zero masks names a function the ROM calls with the return
address in B3; this layer reports those addresses to its caller instead of branching.
`memory:boot-table-and-configuration-table` pins both formats, and two programs go the whole way:
the checked-in `hex6x -boot` builds the ROM table from the linked image, and `c64xemu run IMAGE
--boot-table FILE:ADDR [--boot-config FILE:ADDR]` loads nothing but those tables - the ELF supplies
only symbols. `boot_table` reaches its hand-derived result that way, and `boot_config` returns a
word that exists only because the configuration table's set, clear and toggle entries wrote it
before the application started. The CLI path refuses a configuration entry that names a function
to call rather than pretending to run it. A section's bytes are copied a word at a time wherever
the destination is aligned, as the ROM's own loop does, so a section whose destination is a
register writes that register - which is what SPRUEC6G section 4.3.3 relies on when it starts the
other cores from a standard boot table.

**[-] The boot ROM's own code.** Everything the ROM does that anything can observe is modelled, but
its instructions are not executed: TI ships them in on-chip ROM this project has no copy of, and
the manual warns their addresses move between silicon revisions. Without that image there is
nothing to run and nothing to compare against, which is why this layer reports a called function's
address to its caller instead of branching to it.

**[~] Ethernet boot.** *Done for the C6472's MAC boot modes; remaining boot sources are listed
below.* `--boot-mode 9`/`10` on a C6472 is the ROM's boot over MAC port 0 or 1
(SPRUEC6G section 4.3.5). The ROM's instructions are not modelled - TI ships them in on-chip ROM
this project has no copy of, and the manual warns their addresses move between silicon revisions -
but everything they do that anything can observe is: `src/romboot.rs` holds core 0 in reset and, in
its place, resets the MAC, programs the station address and the broadcast filter, posts the ten
1600-byte receive buffers section 4.3.5 describes, transmits the BOOTP-shaped Ethernet-ready
announcement of section 4.3.5.2 exactly once, and then applies every frame check of section
4.3.5.3 - DIX or 802.3 with SNAP/LLC, with or without a VLAN tag, IPv4, the UDP ports and a
non-zero UDP checksum, the 0x544B magic, the opcode, and a sequence number that must not skip -
before handing each payload to `BootStream`, the same boot-table format as a table in memory but
consumed as fragments that split any record. Two deliberate departures, both forced by the device
model: the ring and its buffers live at the top of core 0's L2 through its global alias rather than
in local L1D, which SPRS612G Table 2-2 marks "DATA SPACE ON DMA" and no DMA master here can reach,
and the station address comes from a fused MAC ID this emulator's `--mac-id` writes because no real
fuse value is public. Sections reach registers because a boot table's copy is word-wide wherever
it is aligned, so the table itself writes DSP_BOOT_ADDRn and BOOT_COMPLETE_STAT and thereby starts
the other cores, exactly as section 4.3.3 says it may. `romboot::an_ethernet_boot_places_both_cores
_images_and_starts_them` drives the whole boot at the device level, and `tests/test_netboot.py`
drives it from the command line through a Unix-socket Ethernet backend with `tests/tools/emac_boot
.py` as the host: an ELF turned into a table addressed to each core's global alias, six cores
running code that could only have come off the wire, and a case that cuts the table into one-word
frames so that every record straddles a frame boundary and the receive ring is recycled twenty
times over. Boot mode 10 is the same boot on the second MAC, and the backend follows the strap -
the wire is on the port the board cabled, or on whichever `--net-port` names - which the fourth
case pins along with a fused address and a UDP port that are not the defaults.

**[x] I2C EEPROM boot.** *Done, through the I2C master's own registers.* BOOTMODE[3:0] = 0101b is
what SPRUEC6G Table 3 calls "Master I2C boot", and `--boot-mode 5 --i2c-eeprom FILE` is it: the
file is a serial EEPROM's contents, put on the bus at the address section 2.3.4 requires - "The
bootloader requires the I2C EEPROM slave address to be 0x50" - and read by driving ICSAR, ICCNT,
ICMDR, ICDXR and ICDRR exactly as firmware drives them. That was the point of doing it this way:
the read sequence exercises the modelled master rather than going round it.

`i2c.rs` gained the device the same section requires of a board, which is a serial EEPROM and not
the queue the existing cases put there: one that "uses two bytes for internal addressing" and
"has the capability to auto-increment its internal address counter such that the contents of the
memory device can be read sequentially". What tells it whether the bytes after a START are an
address or a read is the direction of that START, which is what makes writing an address and
then reading from it work.

Everything on the EEPROM is blocked (section 2.3.4.1): a 16-bit size that counts the 4-byte
header, a 16-bit ones-complement checksum "including the block size and checksum fields" whose
valid values "are 0 and -0", and up to 124 bytes of data. The boot reads the parameter block
first - "The boot parameters begin at EEPROM address 0 ... address = 0x80 * CFGGP[2:0]" - takes
its options field, which "A value of 1 indicates a boot table", and then reads the table from the
address that block names, feeding each block's data to the same `BootStream` the Ethernet boot
feeds and branching core 0 to the entry point when the zero count arrives.

Three places the manual stops short, each about the shape of a transaction rather than an answer,
and each written down where the choice is made. Section 2.3.4 never says "repeated START" and
gives no wire address bytes, so the sequence comes from SPRU974A section 2.5 - "When BB is set to
1 and STT is set to 1, a repeated START condition is generated" - and Table 15's finite form. The
manual gives a block's maximum and its size field but not how many bytes a read asks for, so this
model reads the maximum and takes the real length from what came back. And the bus frequency is
"initially set to CLKIN1/6600" and reprogrammed from the parameter table, but with no register
values given: the boot programs the dividers from the two frequency fields and leaves the
prescaler alone, because the prescaler is the one field of the three this model's transfer time
depends on and no document says what to put in it.

`program:i2c_eeprom_boot:*` is the end-to-end case: the same `sum_loop` the memory-resident boot
table case uses, its table built by TI's own hex conversion utility, wrapped by the harness into
an EEPROM whose blocks and checksums are computed in Python from Table 5. One parameter block,
two table blocks, 144 bytes over the bus, and A4 = 55. Four unit tests cover what a program
cannot reach: the whole boot with a section straddling a block boundary, a corrupt block re-read
once and then reported rather than re-read forever, a zero checksum field disabling the check,
and the configuration index selecting the second parameter block instead of the first.

One thing the boot found on the way in. A single-core run took `Cpu::run_cycles` directly, so the
ROM boot state machine was never ticked at all - the Ethernet boot only ever ran on the six-core
device, which takes the device path. A run now takes the device path whenever the ROM is booting,
and the halt rule ignores a core held in reset: a held core is not stopped on a breakpoint, and
its PC is an address the boot has not branched it to yet.

**[x] C6455 HPI boot.** *Done, as the hardware boot it is.* SPRS276M section 2.4: "the HPI host
boot is a hardware boot mode", and "the CPU is actually held in reset until a DSP interrupt is
generated by the host". `--boot-mode 1 --hpi-unix` straps that at reset:

- PERCFG0.HPICTL is 1 and PERSTAT0 reports the HPI enabled (SPRS276M Table 3-7).
- HPIC.HPIRST is 0 (SPRU969B Figure 31).
- L1P and L1D are 32 KB of cache and L2 is all SRAM (SPRUEC6G Table 2).
- Core 0 is held in reset at `0x00800000`.

The host's first DSPINT requests the core's release. The core starts with reset CPU state, and
DSPINT stays set for the application to clear. The positional ELF supplies symbols only. An
earlier version of this work modelled a ROM sequence instead: a PERLOCK/PERCFG0 write, a powerup
wait, an HPIRST release and the ROM acknowledging DSPINT. The data manual describes none of that
for HPI boot, so it was removed.

Building it exposed a general bug. An event raised while a core was held in reset stayed queued
for the selector and landed in IFR on the core's first cycle. That made the boot's own DSPINT a
pending INT15 the application had never asked for. A held core now drops its event receptions;
EVTFLAG still latches.

`tests/tools/hpi_host.py --boot` transfers ELF load segments and zero-fill tails through the HPI,
flushes the final partial FIFO burst, verifies the memory, then signals DSPINT. It fails if the
emulator faults or a released application never reaches its exit. `tests/test_hpiboot.py` runs
little- and big-endian images to the independently known sum 55. The run includes an unaligned
final three-word transfer and proves that loading without DSPINT executes nothing. A native
device case additionally pins:

- the reset straps and the fixed entry;
- the partial FIFO flush;
- that DSPINT stays set and never reaches IFR;
- that the host cannot raise another DSPINT until the application clears it, after which it is
  an ordinary interrupt.

Every DSPINT is also EDMA3 event DSP_EVT, channel 0 (SPRS276M section 7.4.1 and Table 7-3
note 2), so the boot's DSPINT leaves ER bit 0 set for the firmware; the channel controller is
not held with the core. `c6455_dspint_is_edma_event_zero_from_the_boot_on` checks that the
released firmware finds ER bit 0 pending with nothing enabled, missed or transferred, and that
enabling channel 0 without ECR runs a transfer on that stale event. It then checks that each
later host DSPINT both raises system event 15 and triggers an enabled channel 0, even after the
channel controller went dormant. A disabled channel only latches the event, and ECR clears it.
The C6472 maps no EDMA3 channel to the HPI (SPRS612G Table 7-4), and its case confirms DSPINT
leaves ER clear.

**[x] C6472 HPI boot.** *Done.* Mode 1 holds all six cores with HPIRST clear and the boot cache
configuration on every core (SPRUEC6G section 4.3.2 and Table 21). HPI word accesses reach
chip-level peripheral registers as well as memory, checked by memory protection as privilege
ID 0; the megamodules' own registers stay the CPU's alone (SPRU871K section 5.3.1), for the
HPI, the EDMA, the EMAC and the IDMA alike. The host loads each core through its global aliases,
programs `DSP_BOOT_ADDRn`, then releases selected cores by setting their `BOOT_COMPLETE_STAT`
bits. DSPINT is not a release mechanism on this device.

`tests/tools/hpi_host.py --device c6472 --boot` loads and verifies ELF segments, including zero
fill, for all six cores or a `--cores` subset, replicating core-local segments and writing
shared ones once. `tests/test_hpiboot.py` boots all six to independently known results,
releases just cores 0/3 or 2/4, uses both the L2 base and a nondefault 1 KB-aligned entry in
both byte orders, and proves that loading alone or DSPINT alone executes nothing. Away from the
L2 base the probe puts a guard there, so a core started at the default address rather than its
programmed one fails the run. A native device case covers both byte orders, per-core cache
straps, short final bursts, the host's exclusion from megamodule registers, selective release
and set-only BC behavior.

**[-] SRIO and UTOPIA boot.** SRIO still lacks a device model; UTOPIA's ATM master speaks the bus
but not its boot protocol.

## Ethernet and MDIO

**[x] Module identity and reset.** *Done.* TXIDVER/RXIDVER report the C645x RTL/major/minor
values, RXMAXLEN and FIFOCONTROL use their hardware resets, and SOFTRESET atomically clears
queues, masks, filters, timers, counters and control state.
`emac:identification-defaults-and-reset` checks the register-visible contract.

**[x] Flow control and pause frames.** *Done for buffer and transmit flow.* Per-channel free-buffer
counts and thresholds drive RXFLOWACT and full-duplex maximum/zero pause frames with the
programmed source address; RXPAUSE tracks refresh. Valid incoming pause frames load TXPAUSE and
block new data transmissions until it expires. The paired `emac:*-pause-*` cases check frame
bytes, timers, status and pause statistics. FIFO-occupancy flow remains with the FIFO item below.

**[x] Receive QOS.** *Done.* RXQOSEN classifies untagged/VLAN priorities 0-3 as low, compares
the selected channel's free-buffer count with RXFILTERLOWTHRESH, reports RXQOSACT and counts
RXQOSFILTERED. `emac:receive-qos-filters-low-priority` queues a low frame ahead of a VLAN
priority-4 frame and proves only the latter reaches its descriptor.

**[x] Receive frame classification.** *Done.* The wire length is four CRC bytes more than the
injected data, so frames under 64 wire bytes are undersized and frames over RXMAXLEN are
oversized; each counts in its own statistic instead of RXFILTERED, which now counts only
address-match rejections. RXCSFEN, RXCEFEN and RXCMFEN decide whether an accepted short, long
or MAC control frame reaches memory at all, a long one truncated to RXMAXLEN, and the SOP
descriptor carries UNDERSIZED, OVERSIZE, CONTROL or NOMATCH accordingly. Three paired
`emac:receive-*-copy-enable`/`-truncates-to-rxmaxlen` cases and the reworked `ethernet_*`
firmwares check both halves.

**[x] Receive wire errors.** *Done.* A frame now arrives with the CRC, alignment and code errors
nothing inside the module can produce, which is what the jabber, fragment and error counters
need. An errored frame of proper length is not a good frame: it counts in RXCRCERRORS or in
RXALIGNCODEERRORS, contributes no octets, and reaches memory only with RXCEFEN, where its SOP
descriptor carries CRCERROR, ALIGNERROR or CODEERROR. Alignment and code errors outrank a CRC
error in the counters, because SPRU975E section 5.49.5 defines a CRC error as one without them,
while the descriptor flags are per-error and can both be set. An error changes what each length
class is: a short frame becomes a fragment rather than undersized and needs RXCSFEN and RXCEFEN
together, a long one becomes jabber rather than oversized and is still truncated to RXMAXLEN,
and a fragment counts whatever address it carried, because address matching does not enter its
definition (section 5.49.10) - an error-free short frame the filter rejects still counts as
filtered instead. A pause frame with any of the three errors is not acted upon. Four
`emac:receive-*error*`, `-fragment-and-jabber-*` and `-fragment-counts-*` cases pin the rules.

**[x] Receive buffering and overruns.** *Done.* RXNOCHAIN puts a frame in one buffer, discards the
rest and still reports the whole byte count. Running out of descriptors mid-frame counts a DMA
overrun and filters the frame unless RXCEFEN hands over what fitted with OVERRUN set, and the
FIFO reports a start-of-frame overrun only when no cell was free, a middle-of-frame overrun
otherwise. Neither overrun disturbs the class statistics.
`emac:receive-no-chain-uses-one-buffer`, `emac:receive-dma-overrun-needs-error-copy` and
`emac:receive-fifo-middle-of-frame-overrun` pin the three rules.

**[x] Interrupt reporting and host errors.** *Done.* MACINVECTOR reports the eight transmit and
eight receive pending bits together with STATPEND, HOSTPEND and the MDIO link and user bits
that only the control module sees, and MACINTSTATRAW/MASKED/MASKSET/MASKCLEAR gate the two MAC
sources into the single MACINT line. STATPEND follows the documented statistics level, which
is reachable now that a statistics register is an ordinary read/write location while GMIIEN is
clear. Transmit descriptor errors 1h to 5h and the receive zero-buffer error latch HOSTPEND
with their code and channel in MACSTATUS, clear the channel field on read, halt both DMA
engines and survive until a soft reset. The receive unowned-buffer error is deliberately not
reported, because this model treats an unowned posted buffer as the queue-empty case.

**[~] The FIFOs and TX arbitration.** *Done functionally.* Sixty-eight 64-byte receive cells bound
host input, drive RXFIFOFLOWTHRESH pause generation and increment SOF/MOF overruns. The
twenty-four-cell transmit capacity and TXCELLTHRESH reset value are exposed; one complete legal
packet drains per arbitration tick. TXPTYPE selects channel-0-first round robin or fixed
priority with channel 7 highest, covered by the paired `emac:*-transmit-arbitration` cases.

**[-] Wire-rate cell latency in the FIFOs.** The cell counts, the thresholds and the arbitration
order are modelled and pinned; a packet drains in one arbitration tick rather than in the time its
bytes would take on the wire. Charging that time means giving the model a wire clock, and nothing
here measures the wire: frames are exchanged with a host process over a Unix socket, which has no
rate of its own. The paced network cases measure the emulator's speed instead, which is the
different question they are for.

What the absence of a wire clock does force is back-pressure at both host seams. The backend
delivers a frame the moment the peer writes it, so a receive FIFO that is momentarily full would
report a start-of-frame overrun that a 100 Mbit/s line could not have caused at that rate, and a
peer whose socket buffer is momentarily full would take a transmitted frame off the wire
altogether. `Machine` therefore asks the MAC whether the FIFO has room and holds the frame until
it does, and `Bridge` queues a refused frame and retries it in order. Those are the wire
behaviours the model cannot do without: the losses they replace cost a paced TCP case a
retransmission timeout for every host scheduling hiccup.

**[x] MDIO clause 45.** *Done through the IEEE clause-22 MMD window.* PHY registers 13/14
implement address, data, read/write post-increment and write-only post-increment functions,
with PMA/PMD identity and live link status seeded for the board PHY.
`mdio:clause45-mmd-address-data-and-increment` checks random MMD storage and sequencing.

**[x] Statistics block.** *Done.* Every aligned register from RXGOODFRAMES through
RXDMAOVERRUNS is implemented with zero reset and GMIIEN write-to-decrement behavior. Receive,
transmit, pause, filtering, overruns, octets and wire-size buckets update on their corresponding
paths; counters without an injectable physical error source remain zero.
`emac:complete-transmit-statistics` pins broadcast, octet and size accounting.

## Decoder

**[x] Single-pass family identification.** *Complete for both encodings.* A fetched instruction is
identified once, with the compact form carrying the SAT/BR/DSZ-dependent family selected by its
fetch-packet header. Execution, mnemonic naming, predicates, address and ordinary operands, packet
resource accounting and serialization all project that stored result instead of re-running the
wide or compact decoder. An invalid encoding has no family, so execution raises the opcode fault,
the view renders its raw word and resource validation reports the same failure without a second
classification. The whole compact word/header-space classification snapshots and the `dis6x`
differentials retain the observable behavior while guarding this shared seam.

**[x] The compact subtract under a saturating packet.** *Fixed.* A fetch packet's SAT bit selects the
saturating form of the compact three-operand ALU, and the model applied it on either functional
unit. SSUB has no .S unit form: TI's assembler rejects `SSUB .S1` while accepting `SADD .S1`,
`SADD .L1` and `SSUB .L1`, binutils' opcode table has `ssub` only for the L unit's `l3_sat_1`
format, and `dis6x` reads the encoding as a plain SUB. So an S-unit compact subtract ignores the
packet's SAT bit while an L-unit one honours it, which is now what the model does.
`cpu:compact-l-unit-subtract-under-the-packet-saturation-bit` and its S-unit sibling subtract one
from the most negative integer in a saturating packet: the L unit clamps and the S unit wraps.
Found by extending the dis6x differential over the suite's own programs - DSPLIB at -Os is where
the compiler emits it - which is code the two firmware images do not contain.

**[x] ALU operands.** *Compared, both encodings.* `binary_operand_shape` now reads the two-source
format's fields - src1, src2, the destination, whether src1 is a constant, whether that constant is
unsigned, and the cross-path bit - for both the executor and a rendering, so `tools/dis6x_diff.py`
compares the operand list TI's disassembler prints wherever the mnemonics match exactly. They agree
everywhere. Two things had to be handled rather than fixed. An aliased mnemonic is skipped, because
`dis6x` prints the pseudo-operation's own operand list: `MV a10,a5` stands where the encoding is `OR
0,a10,a5`, and a subtraction of a constant reorders its sources. And CMPGTU and CMPLTU take a
five-bit unsigned constant on C64x and above - binutils' opcode table says so explicitly, noting
that SPRUFE8 does not mention it, and `cl6x` encodes 4 and 20 as different words - while `dis6x`
prints that constant masked to four bits, so the comparison drops the top bit rather than calling
the model wrong. Four `decoder:operands-*` cases pin the shapes, including that unsigned constant.

The compact arithmetic families are compared too: the three-operand ALU on the L and S units and
the two-operand add and subtract whose destination doubles as its first source, which TI's tools
write out both times rather than as a two-operand form. Their register fields go through
`compact_register`, the accessor the executor uses, so the header's RS bit remaps the window in the
rendering exactly as it does in execution - which is the property worth checking, since a window
error would be invisible in a program that only uses A0-A7. Four `decoder:compact-operands-*` cases
pin those shapes. One ordering rule had to be respected: the branch families share the arithmetic
masks and are tested first in family identification, so operand reading dispatches on the stored
`CompactOpcode` rather than re-reading those overlapping fields.

**[x] Address operands.** *Compared, both encodings.* The address generator's four mode bits decide
whether the offset is a constant or a register, whether it is added or subtracted and whether the
base is updated before or after the access (SPRUFE8B Table 3-11). `address_shape` now reads those
fields once for both the executor and a rendering in the syntax TI's tools print, so
`tools/dis6x_diff.py` compares `*+B15[134]`, `*A4++[1]`, `*--B6[2]` and the rest against `dis6x`:
5115 address operands in the firmware images and every one in the program images agree. No defect
surfaced, which is the expected outcome for a path the DSPLIB differentials already exercise
numerically - but it is now guarded, and four `decoder:address-mode-*` cases pin one shape each.

The compact shapes are compared too: the B15 window with its word-scaled offset, the stack forms
whose store post-decrements and whose load pre-increments, and the pointer forms that
post-increment, pre-decrement or take a register or constant offset. TI's tools write a compact
offset without the sign the 32-bit forms carry, which was the only difference to reconcile, and
four `decoder:compact-address-*` cases pin those shapes. What the rendering does *not* check is the
offset scaling - `dis6x` prints the raw field, so the rule that a non-aligned doubleword form counts
its offset in bytes rather than doublewords is verified by the DSPLIB execution differentials, not
by this comparison.

**[~] The rest of the operand dimension.** *Compared, both encodings.* Every family whose operands
the model reads is now rendered and compared, which took one accessor per format and one table
per family. `source_fields` reads the `1_or_2_src` field positions - src1 at bits 13-17, src2 at
18-22, the destination at 23-27, the cross-path bit at 12 - for the M unit, the shifts, the
bit-field operations, the D unit's address arithmetic and the 40-bit long forms, all of which used
to read them inline. What each family does with those fields comes from the op-field table that
already named it: `mpy_roles` says which M-unit encodings take a constant src1, which read or
write a register pair, and which two - SSHVL, SSHVR and ROTL - TI writes `src2, src1, dst` rather
than the other way round; `long_roles` says the same for the .L unit's 40-bit family, including
that its cross path feeds src1 while a 40-bit src2 is always read on the unit's own side. One
table decides the name and the operand roles together, so a rendering cannot disagree with the
mnemonic printed beside it, and the executor asserts the roles it was handed against the registers
it actually reads and writes - `debug_assert!(roles.dst_pair)` where it schedules the odd half,
and so on, which is what keeps the table honest in a build that runs the tests.

The compact families needed the same treatment for their constants: `compact_constant5`,
`compact_add_constant`, `compact_constant8` and `compact_stack_constant` replace the five
hand-copied assemblies of the same bit fields, and the executor and the rendering read them there.
`spmask_mask` and `unit_mask_bit` do it for SPMASK: the loop buffer masks instructions out by
those bits and the rendering names the units from the same table, so `spmask s1,d2` and the units
the replay actually drops cannot diverge.

With that, `tools/dis6x_diff.py` compares 57324 operand lists in the firmware images and 228532
across the 342685 instructions of the program images the suite differentiates, and nothing
disagrees; a sweep over every program at -O3 and -Os, twice the suite's set, compares 526295 of
them with the same result. What is compared now
is the multiplies including their pair destinations, the 32-bit and 40-bit shifts, EXT/EXTU/SET/CLR
in both their constant and register forms, MVK/MVKH/MVKL/ADDK, the control-register moves, the D
unit's address arithmetic and its long B14/B15 form, the .L unit's 40-bit arithmetic, the register
branches with the NOP count BNOP folds in, multicycle NOP counts, SPLOOP's iteration interval,
SPMASK's unit list, and in the compact space the shifts, the bit operations, the constant moves,
the conditional single-register forms, the moves that remap the register window and the compare
and logical forms. Twenty-three `decoder:*operands*` cases pin one rule each, most of them on
encodings taken from `cl6x -mv6400+` output so the word itself is TI's.

Three defects came out of it. `MVC` names two control registers by the direction of the access:
0x1d reads EFR and writes ECR, 0x02 reads IFR and writes ISR (SPRU732J Table 3-22), and the model
named both by their read name - which is what the model's own `write` implements, since a write to
0x1d clears exception flags. `written_register_name` now answers for the write side, and the two
`decoder:operands-control-register-*` cases pin both directions. `SUB .L1 5,A1:A0,A3:A2` - the
40-bit subtraction from a five-bit constant, which `cl6x` assembles as 0100a498h - raised the
unsupported-operation fault, because only the constant-zero form TI's tools print as NEG had an
execution arm; the constant is not what makes that form saturate, so every constant takes the same
path now. And the M unit's dispatch still carried arms for MPYI, MPYID, MPYDP and MPYSP, the C67x
slots this device does not have: their encodings have decoded to nothing since the binutils sweep,
so the arms were unreachable and are gone.

**[-] Rendering the operands the model does not read.** The PC-relative label forms - the branches,
CALLP and ADDKPC, where `dis6x` prints a symbol resolved from the image - and SPKERNEL's fstg and
fcyc fields, which tell an assembler where the kernel starts and which nothing in the model
consults. Rendering those would test the renderer rather than the model, so this one is unplanned
by choice rather than for want of an oracle: the box says it will not be done, not that it cannot.

**[x] Cross-path accounting.** *Fixed.* Two instructions in one execute packet may share a cross-path
operand but may not read different ones (SPRUFE8B section 3.8.4), and the resource accounting kept
an explicit list of the opcodes whose bit 12 selects that read. The list was incomplete: the L
unit's CMPLT and CMPLTU and ANDN, the 40-bit long arithmetic, the L-unit unary ABS and SWAP4, the
whole M unit and the reversed-operand subtract forms all read across it and were not recorded, so
packets with two different 1X reads through any of them were accepted. The 40-bit long forms also
cross src1 rather than src2, which is the operand their execution reads across, so the register the
rule compares had to come from a different field for them. In the compact space the two-operand add
and subtract carry their own cross bit, and ADDAW from B15 on the .D1 unit reads its base across
1X - the same rule the long ADDA form follows, which is where this class of defect first appeared.
The D-unit address arithmetic is deliberately not in the list: bit 12 is not its cross-path
selector, which `dis6x` confirms by printing those forms without an X.

Found with the unit dimension of the dis6x differential. The tool compares seven things against
TI's disassembler over all 436813 instructions in the firmware and program images - the mnemonic,
the functional unit, the cross-path marking, the predicate, the memory data path, the address
operand of every memory access and every operand list the model reads - and they agree. The
predicate dimension needed the compact families the model predicates: the branches,
whose selector is A0 or B0 by the side bit with the sense in bit 4, the write-zero-or-one form
whose two-bit field names both, and the nested-loop SPLOOPD whose selector is never inverted. The
M unit's compact multiplies had to be excluded explicitly, as they are in the executor: their
encoding overlaps the conditional family's mask and they are never predicated. Two cases pin the rule through the families that used to be
missed - `cpu:cross-path-multiply-and-compare-share-one-operand` is the packet TI's assembler
builds, and its `-differ` sibling the one it refuses. The conflict message now names the register
file rather than the reading side, so it reads "reads B5 after B4" instead of "reads 15 after 14".
One exception is documented in the tool: the 32-bit immediate BNOP with its unit bit clear is the
C64x+ macro form that claims no functional unit, which is how TI's own compiler pairs it with an
S-unit instruction in the same packet, while `dis6x` prints a side for it regardless.

**[x] The instruction-set boundary.** *Fixed.* `tools/decode_sweep.py` runs every one of the 2195
instruction words in binutils' own tic6x vectors through the decoder, including the files
assembled for `c674x` that the suite's sampled cases skip, and requires each mnemonic to match
binutils' rendering. Two things came out of running it.

The decoder reported family names where binutils names instructions: the .L unit's 40-bit
arithmetic all read `long`, its shifts `longshift`, the four add/subtract pairs `addsub` and both
pack pairs `dpack2`. Execution had always distinguished them - each operation field has its own
arm - so this was naming, not behaviour, but it left 200 words unusable as an oracle. The
mnemonic now comes from the operation field, the same way the multiply families already worked.

The decoder also accepted seven instructions this device does not have: MPYI, MPYID, MPYDP,
MPYSP, MPYSPDP, MPYSP2DP and ABSSP are C67x floating-point additions, and TI's own tools say so
plainly - `cl6x -mv6400+` answers "Instruction invalid for C6400+" and `dis6x` disassembles a
C6400+ object holding those words as data. They now decode to nothing and executing one raises
the opcode exception, which is what a C64x+ does. Each rejection is pinned beside the C64x+
instruction that shares its opcode field and still decodes: `decoder:mpyi-is-not-a-c64x-plus-instruction`
against `decoder:mpy32-still-decodes-beside-mpyi`, and so on, with
`cpu:c67x-opcode-raises-an-exception` checking IERR's OPX bit and EFR's IXF bit.

**[x] A differential against dis6x over linked images.** *Done, both encodings.* `tools/dis6x_diff.py`
disassembles images with TI's own `dis6x` and compares every instruction with the decoder's: 93833
instructions across the SYS/BIOS kernel exerciser and the NDK image, 38710 of them compact, and
another 159220 across fourteen builds of the suite's own programs at -O3 and -Os, all matching.
Pointing it at compiler-generated DSPLIB code is what found the compact saturation defect above,
along with two namings of mine - the non-aligned word load LDNW, and SWAP2 as the PACKLH2 it
encodes. Compact instructions needed two things first. The
decode protocol now takes the fetch packet's header word, because a compact instruction cannot be
named without it - the header carries the data size, saturation and branch fields its meaning
depends on - and `compact_mnemonic` names what `execute_compact` executes, mirroring its dispatch
test for test so the differential checks the reading of the encoding rather than a second opinion
about it. The header is the *last* word of its 32-byte packet, so the tool looks it up per packet:
the compact instructions of a packet are printed before their own header, and tracking it forward
mis-decoded 1500 of them until that was fixed.

No model defect surfaced in the compact space, which is worth recording given that a compact
defect once lived there - the `m_m3` MPY/SMPY overlap. The disagreements were all in the naming I
had written: TI names both compact branch forms BNOP, including the zero-count one, and names the
write-zero unary as a move of the constant rather than ZERO. Eight `decoder:compact-*` cases pin
what the header decides: the same multiply word saturating or not by its SAT bit, the same load
word a word or doubleword access by DSZ, and the same word a branch or a shift by the branch bit.

All three differentials run inside the suite rather than beside it: `decoder:binutils-vector-sweep`
asserts the sweep's counts, `decoder:dis6x-image-differential` that both firmware images were
disassembled with nothing disagreeing, and `decoder:dis6x-program-differential` the same over the
sixty-two program images the suite already builds at -O3 and -Os - the levels that generate the
packed and software-pipelined code worth differentiating, and where the compact saturating-subtract
defect was found. The three add about four seconds to a run. What they assert is that every image
was disassembled and that nothing disagreed, not the instruction totals, which move whenever a
program or the firmware changes.

The aliases are listed in the tool rather than hidden, because TI's assembler emits one encoding
for several mnemonics and dis6x prints the friendlier one: MV is an OR with zero, NOT an XOR with
-1, NEG and ZERO are subtractions, and - the one this differential turned up - a subtraction of a
five-bit constant is an addition of its negation, since `SUB .L2 B0,1,B13` and `ADD .L2 -1,B0,B13`
assemble to the same word.

One mnemonic pair is a genuine alias rather than a mismatch: binutils prints `or .S1 0,a2,a3`
where TI's assembler emits that very word for `MV .S1 A2,A3` and `dis6x` prints MV. The sweep
accepts either name and says why.

With those settled the sweep is clean: 2032 words match, 163 floating-point words decode to
nothing, no disagreements.

## Harness and tooling

**[x] The slave-port seam.** *Done.* The router's dispatch was twenty inherent
`read(address) -> Option<u32>` / `write(address, value) -> bool` methods chained by hand, uniform
by convention only. `src/port.rs` now holds the `SlavePort` trait every peripheral implements,
with the three deviations folded into the ports rather than into the router: the big-endian
parameter the DDR2, EMIFA and device-configuration ports took becomes the endianness in a
`PortContext`, which they read live because `Fabric::set_endian` can change it; McBSP's `owns`
pre-check moved inside its own implementation; and IDMA's bespoke `IdmaWriteResult` became the
`PortWrite` every write answers, so a completion event is part of the shape rather than a special
case. The interrupt latchers stay explicit in the router in their original order, because a
follow-up the megamodules are owed is not part of a port's shape - which is what the trait had to
be shaped around rather than against.

The value is the test the uniform shape makes possible. `no_two_slave_ports_claim_one_address`
sweeps every peripheral window each device profile places, asks every port in turn through the
trait, and requires at most one to answer - and it found a real overlap immediately.
`EWCTL` at 0x02C8 1004 is the C6455 EMAC control module's interrupt gate (SPRS276M), but on a
C6472 that word belongs to the per-MAC EMAC interrupt controller (SPRS612G Tables 7-105 and
7-106), and because the router consults the EMAC first, a C6472 write to EMIC0's PSCFG landed in
the EMAC's ewctl field. The EMAC now takes that register only on a device with no EMIC, and
`the_c6472_emac_does_not_claim_the_interrupt_controller_window` pins both halves: PSCFG round-trips
through the EMIC on a C6472 and EWCTL still round-trips on a C6455. The sweep passes both profiles
with no exclusions. It is reads only, deliberately: several ports have write-only registers and
destructive writes - a PSC reset, a boot release - so write ownership cannot be swept without
rebuilding the device for every address.

**[x] GDB stub.** *Done.* `c64xemu gdb IMAGE --listen HOST:PORT` serves the remote serial
protocol over TCP with the exact 69-register C64x+ layout from [GDB tic6x support](https://sourceware.org/git/?p=binutils-gdb.git;a=tree;f=gdb;hb=refs/tags/binutils-2_47),
architectural memory access, `g/G/p/P`, `m/M`, continue, execute-packet single-step and logical
`Z0/z0` breakpoints. It also publishes a target description through qXfer. The protocol case
checks register banks, CSR/PC/TSR/ILC/RILC ordering, memory, step and continue; a loopback TCP
smoke exercised framing, checksum acknowledgements, register/memory reads and a breakpoint.

**[x] The FFT family's intrinsic implementations.** *Done.* Ten DSPLIB kernels existed only as
intrinsic C and nothing here had ever run them: the forward and inverse transforms over 16x16,
16x32 and 32x32 data, the imaginary-real ordered variants, the real-input 16x16 transform and the
scaled 32x32 one. `dsplib_fft_intrinsics` runs each beside its natural-C reference over identical
inputs and requires them to agree bit for bit, with the reference checksum anchored by a host
build that gives the same value at four optimisation levels. All ten agreed, at every level
including the software-pipelined ones. The suite builds the program at -O0 and -O1 only: cl6x
needs about two minutes to pipeline ten FFT kernels, so -O2, -O3 and -Os were verified by hand
rather than on every run, at sixty-four points as well as sixteen.

**[x] Differential execution against DSPLIB's hand-optimised kernels.** Complete. The
`dsplib_optimised` program now runs thirty-six optimized/reference pairs: the original eight
plus add16, add32, bexp, blk_eswap16/32/64, blk_move, dotp_sqr, fir_cplx,
fir_cplx_hM4X4, fir_gen_hM17_rA8X8, fir_r4, fir_r8, fir_r8_hM16_rM8A8X8, fir_sym,
firlms2, fltoq15, iir, iir_lat, iir_ss, mat_mul, mat_mul_cplx, maxidx, minerror,
minval, neg32, q15tofl and recip16. Every input satisfies its kernel's alignment, count
and dimensional assumptions, and stateful filters run over consecutive samples or calls.

This expansion found a real compact-decoder defect: `m_m3` MPY/SMPY encodings overlap the
generic predicated-move mask and must take precedence in both execution and resource
classification. `DSP_iir_lat` at the unoptimized compact-heavy build exposed both effects:
SMPYHL produced a constant one instead of a product, and packet validation rejected its M
unit. The differential now passes at every configured optimization level.

The natural-C references are host-portable except `DSP_recip16_cn`, whose nominal reference
still calls the target-only `_norm` and `_subc` intrinsics. That kernel uses an equivalent
local scalar normalization/restoring-division reference instead. The host build remains the
independent checksum oracle.

**[x] Runtime.** Complete. Parallel target-program compilation reduced a full suite run to about
36 seconds; emulation remains under a second.

## Where to look next

Priority only, in rough order of expected yield: every entry names an item above and the item
carries the box, so nothing here can disagree with an item's state. There is no `[ ]` item left,
which is why this list is now methods rather than tasks. The last several rounds of defects came
from four of them, in this order of what they have actually paid.

1. **Where two solved rules meet and no case asks what they do together.** This keeps earning
   more than anything else on the list, and it is now the only one that has been run
   deliberately rather than stumbled into. The cache manual's worked examples found a 4% error
   between a correct miss cost and a correct write buffer. Making the cache configuration carve
   its SRAM aperture found that a megamodule was caching its own L1 memory, which SPRU871K
   forbids in two places. Enabling the transmit mask found that the shared MAC interrupt line's
   rising edge makes acknowledging one queue unsafe. Then one pass spent looking on purpose
   found two more: an L1D victim was allocating L2 lines the hardware never allocates
   (SPRU871K 4.3.8.3), and the write buffer was merging writes the hardware never merges
   (SPRU871K 3.5.3). In every case both halves were modelled and right, and their meeting point
   was not.

   What made the deliberate pass work was a bar every candidate had to clear, and it is worth
   keeping: both halves already pinned by a case; no case exercising the interaction; and a
   document that settles the interaction, quoted. The third is what does the filtering. The same
   pass turned up interactions with no documented answer - what an access to a carved-out
   aperture returns, whether a DMA read drains the write buffer, whether an IDMA transfer into
   memory that is now cache reports an error - and those are not candidates, they are `[-]`
   entries by the same rule that keeps invented numbers out. The two candidates that did clear
   the bar have both been taken since, and neither was a standalone item at the time - which is
   the failure mode worth watching in this list: work named only inside another item's prose gets
   read as commentary. One was a coverage gap and closed as one, an L1D victim versus L2's LRU now
   observed through a transmitted frame; the other was SPRU871K 4.6.2.4.3's local-replaces-remote
   rule, and it turned out to be a defect in both halves of the fault path plus a third in
   MPFCLR. Anything found this way gets its own item and its own box the day it is found.
2. **Reading a manual closely enough to solve it rather than quote it.** The L1P and L1D miss
   numbers came out of a table of averages that one occupancy explains; the cache aperture
   geometry came out of a figure. The same reading is what refuted an item this list used to
   carry: RMII_RST resets the RMII logic and this device is strapped to MII, so the enable that
   gates its wire was GMIIEN a register away.
3. **Differentials against TI's own tools and TI's own code.** Three run inside every pass, and
   the newest is not a differential but the same idea: run TI's code and require what TI's code
   says. TI's server daemons carry byte-exact contracts nobody here wrote, and registering one of
   them correctly needed its source rather than its header.
4. **Measuring a change instead of assuming it.** Two items closed this way with the opposite
   sign to the one expected. The transmit interrupt made the paced UDP case fail in eight runs of
   twenty and bought no headroom at any rate, so it is `[-]` with its numbers; reclaiming on every
   poll instead, which was the half nobody had asked for, made the flakiest case in the suite
   better. A change that is not measured is a change whose sign is unknown.
5. The `[-]` items, in a group, because none of them is blocked on code. Each is blocked on a
   number no document gives - L2 arbitration, the victim buffer flush, the tag update buffer,
   external memory's latency - or on a second device model, and the entry says which. Where only a
   bound is documented, modelling the mechanism and saying so beats inventing the number.

## What is still not modelled

Every entry here is `[-]`: an inventory, not a plan, and nothing below is in progress. Each entry
says what the device would have to do, and - the part that decides whether it is worth doing at
all - where an expectation independent of this emulator would come from. A section item that names
one of these entries carries `[~]` rather than `[-]`, because its modelled half is real.

**[-] UTOPIA 2's multi-PHY address phase.** The cell path is modelled, so what is left of this entry
is the pin-level polling: SLID, the address bus and the cell-available handshake that lets a
master pick one slave out of several. The model's master addresses the one slave directly, so
SLID is stored and reported but decides nothing. Modelling the phase needs a second slave to
choose between, which is a second device model rather than a missing rule.

**[-] McBSP's multichannel selection and companding.** Both ports are modelled - see the peripheral
section - but three of their features are stored without deciding anything, because each needs a
framing device on the pins to be observable: multichannel selection through MCR and the 128-bit
channel-enable file, mu-law and A-law companding, and the frame-sync generator's period and width.
Modelling those needs a codec on the other end of the port, which is a second device model rather
than a missing rule. Worth recording how the entry that said "no register file, no serialiser, no
frame-sync logic, and SPRU580 is not checked in" was closed anyway: the CSL register appendix
carries every McBSP bit layout, and the device data manual carries the addresses.

**[-] VCP2 and TCP2.** The Viterbi and Turbo coprocessors. Both are large, both are specified in
detail (SPRU533D, SPRU534C) and both have reference implementations in TI's own libraries, which
would make good differential oracles - but each is a decoder in its own right, closer in size to
the CPU core than to a peripheral, and neither is on any path this emulator's firmware exercises.

**[-] The HPI's read-burst timeout counter.** A host read burst that stalls long enough for the DSP
side to time out drops HRDY and reports the timeout. Nothing on this model holds HRDY: the DMA
always completes, so the counter has no injectable source and a case could only assert that it
never fires.

**[-] The GPIO and timer output pins as signals.** GPIO output pins and the timer's TOUTL drive is
observable to software through the registers, and the edge detector sees them, but no device on
the model consumes a pin as an input, so a timer output cannot clock a peripheral. The device
that would make this observable is the watchdog's reset path, and a reset the emulator takes
from a peripheral is a larger change than the pin itself.

**[-] External bus devices.** The EMIF and DDR2 controllers are modelled as memory. Neither models a
device with its own timing on the bus - an asynchronous SRAM with wait states, a NAND flash with
its command sequences - so the wait-state and turnaround fields in their timing registers are
stored and reported but change nothing observable.
