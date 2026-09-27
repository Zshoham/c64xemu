# Execution and timing

How one execute packet is fetched, issued, executed and retired; how delay slots are implemented;
what the SPLOOP buffer does; and where every cycle count in the model comes from.

`src/cpu.rs` is the cycle engine and owns every register. `src/sploop.rs` owns the loop buffer's
state machine, and `src/control.rs` the control register file. The split between the CPU and the
loop buffer is deliberate: the state used to sit on the CPU as an eighteen-field value that the
cycle step, both execute paths, the interrupt pipe-down, the exception teardown and the trace all
reached into, so each of them had to know what "loading and not yet reloading" meant.

## The shape of one cycle

`Cpu::step` is the whole cycle, and it is a sequence of early exits before the ordinary path.

| Order | Condition | What the cycle does |
|---|---|---|
| 1 | `fault.is_some()` | nothing; the core is stopped |
| 2 | `pending_stalls != 0` | consume one stall, count `stall_memory`, `freeze` |
| 3 | `idling` | count `idle_cycles`, commit writes, run the cycle boundary; entering a service routine ends the idle |
| 4 | `interrupt_stall != 0` | consume one, count `stall_interrupt`, commit writes — the pipeline drains *into* the interrupt, so writes already in flight complete before the handler runs |
| 5 | `stall_cycles != 0` | a multicycle NOP; inside an active loop it still replays the buffer, because a NOP only idles the program-memory side |
| 6 | `stops_at(pc)` | halt cleanly at a fetch boundary so the caller observes `halted()` |
| 7 | `sploop.replaying_alone()` | a reloaded nested loop replays while the outer loop keeps issuing |
| 8 | otherwise | fetch, issue, execute, charge stalls, commit, finish |

Note the difference between `pending_stalls` and `stall_cycles`. A stall freezes the whole
pipeline: `freeze` advances `stalls` rather than `cycle`, drops the record of the previous
cycle's writes, and ticks the device so the peripherals still advance. A multicycle NOP does not
freeze anything — write-backs scheduled by earlier packets still land.

`cycles()` is `cycle + stalls`, which is the wall-clock time base everything timing-related uses.

## Fetch and issue

The fetch loop reads words through `fetch` until it meets one whose parallel bit is clear, or
until eight instructions have been taken — the maximum an execute packet can hold. Two questions
about the whole packet are answered during the fetch rather than by walking it a second time: the
first `SPMASK` it carries, and whether any of its fetch packets set PROT.

The packet is built in a `MaybeUninit<[Fetched; 8]>` on the stack. That is the hot loop of the
emulator; the `unsafe` blocks are commented with the invariant that makes each sound.

What a `Fetched` carries — the family identification, the unit and cross path it claims, the
predicate — comes from the instruction view. See [Instruction decoding](decoding.md); the
executor consults the same value the display and the packet validator do, so they cannot disagree.

## Delay slots and the write-back queue

Results are not written when an instruction executes. They are scheduled into the write-back
queue (`WritebackQueue`, `src/writeback.rs`), and `commit_writes` performs the ones due this
cycle. The queue owns every delayed effect and the order they come back in; the CPU keeps the
architectural actions - writing the register, checking a store against memory protection,
narrowing a control-register write, setting SSR - and the trace events for each.

- **Register writes** wait in a ring of `SLOTS` (16) slots indexed by due cycle, so scheduling
  and draining are an index rather than a search; every delay is shorter than the ring, so a
  write never shares a slot with an earlier write-back. A slot drains from its end. Two writes
  to one register on one cycle are resolved when the second is scheduled: issued on the same
  cycle it is refused and the CPU raises the SPRUFE8B §3.8.8 resource fault, issued later it
  replaces the earlier one. A per-slot register mask proves most schedules collision-free
  without walking the slot.
- **Stores** land at the end of the cycle they execute on, so every queued store is due at the
  next write-back and they come back first, last, second to last, ..., second.
- **Control-register writes** land exactly on their due cycle - the issue cycle, or four later
  for ILC and RILC - found by a scan with `swap_remove`.
- **Saturation flags** land from the cycle after the saturating result, as one SSR mask.

An occupancy mask summarises which slots and queues hold anything, so a cycle with nothing due
costs one mask test inlined into each caller of `commit_writes`, and only a cycle with work
enters the out-of-line drain.

The commit order - register writes, then stores, then control writes, then saturations, and
within each the order above - is observable in the trace and in same-address stores, and it is
the order the CPU committed in when these were four vectors scanned with `swap_remove`.
`the_queue_commits_in_exactly_the_order_of_the_original_scans` pins it against a model of those
scans. Every cycle ends with a write-back, a packet rejected by validation included: the packet
is annulled, but what earlier packets have in flight completes as the pipeline drains into the
exception (SPRU732J 6.3), so nothing ever waits past its due cycle.

This is what makes delay slots observable in the right way: an instruction with a delay slot is
one whose write is scheduled further out, and any read in an intervening cycle sees the *old*
value because nothing has landed yet. Nothing has to model "in flight" separately.

`previous_writes` is a two-element bitmask of the registers written in the previous cycle, per
bank. It exists for the cross-path stall rule below, and `freeze` clears it because after a stall
the hazard it describes is two cycles old.

Branches are held in `branches` as (due cycle, target) pairs and resolved in `end_cycle`, at the
*end* of the cycle before they take effect — which is why `pc` between cycles always names the
next packet to fetch. When a branch fires it also ends an active loop buffer, because a taken
branch makes the buffer idle once its last delay slot has passed ([SPRU732J](../reference/spru732j.pdf)
§7.9); a nested loop is exempt, since the branch closing an outer body only redirects the frozen
fetch stream.

## Stalls, and where each number comes from

Every cost below is derived from a documented rule and pinned by a test in `src/cpu.rs` that
names the rule. None was fitted to a measurement.

| Stall | Cost | Rule |
|---|---|---|
| Cross-path read one cycle after a non-load wrote the register | 1 cycle, whole packet re-issued | [SPRUFE8B](../reference/sprufe8b.pdf) §3.8.4 |
| L1D bank conflict: two resident accesses in the same 32-bit bank | 1 cycle | [SPRU862B](../reference/spru862b.pdf) §3.2.1 |
| L1D read miss, line from L2 SRAM | 12.5 cycles — spent as alternating 12 and 13 | SPRU862B Table 3-2, 1-wait-state 4×128-bit column |
| L1D read miss pipelined behind one in the previous cycle | 3 cycles | same table |
| Each further parallel miss in the same cycle | 4 cycles | same table, M consecutive parallel misses cost 12.5 + 4(M−1) |
| The same three, out of the L2 cache instead of L2 SRAM | 14.5, 7, 8 | same table |
| Victim buffer flush on the previous miss | disrupts pipelining, so the next miss pays full price | same table |
| Write buffer drain, per entry | 2 cycles into L2 SRAM, 6 into the L2 cache | SPRU862B Table 3-2 |
| Read miss with a non-empty write buffer | waits for the buffer to drain completely | [SPRU871K](../reference/spru871k.pdf) §3.5.3 |
| L1P line fill | 14/3 cycles average out of L2 SRAM — spent as a repeating 5, 5, 4 — six out of the L2 cache | SPRU862B Table 1-3 |
| PROT fetch packet with a load | 4 NOP cycles after the load | the PROT header bit |
| Interrupt entry | 9 cycles | SPRUFE8B §5.5.1 |

Two of those deserve their reasoning, because both are averages that had to be spent somewhere.

The **half cycle** on an L1D read miss is real: the access lands on one of the two phases of the
CPU/2 L2 clock and only the average, 12.5, is documented. The model alternates 12 and 13 over
successive full-price misses rather than deriving the phase from the issue cycle's parity. That
choice is load-bearing — parity ties the phase to a loop's period, so a loop whose misses arrive
an even number of cycles apart would pay the dearer phase every time, which is exactly how
SPRU862B §3.4.7's worked totals came out 64 and 32 cycles high.

The **thirds** in the L1P fill occupancy are the same argument: 14/3 cycles per line spent as a
repeating 5, 5, 4. Miss pipelining is modelled by holding one fill at a time and starting the
next as soon as the path is free, so a sequential line is already on its way and the CPU stalls
only for the part of the fill execution has outrun, while a branch target's fill cannot begin
until the branch has redirected the fetch stream.

The write buffer holds four entries as (quad-word, completion time) pairs. Merging follows
SPRU871K §3.5.3, including the two conditions that are about *which pair* of writes is eligible
rather than the address: a write only merges into the immediately preceding entry, and an entry
the L2 cache may hold never merges at all. A fifth write when four are busy stalls until the
oldest completes.

## The SPLOOP buffer

`src/sploop.rs` holds one scheduled iteration and overlays time-shifted copies of it, so the
prolog, kernel and epilog are never written anywhere — they emerge from which stages are valid at
a given cycle offset ([SPRU732J](../reference/spru732j.pdf) chapter 7).

The buffer stores `BufferedPacket`s, each with the cycle offset from the loop start at which it
was loaded; a packet loaded at offset *n* runs again at *n* + ii, *n* + 2·ii, and so on. Deriving
the position from the loop's start cycle is a deliberate simplification of the manual's four
counters (LBC and the loading, draining and reloading counters): nothing here has to retrace load
order, because the body is already a list in load order.

What is *not* buffered is as important as what is: `buffer_loop_packet` drops the `SPMASK` itself
and the units it names, NOPs — which only occupy schedule slots, and whose multicycle form would
wrongly stall the program-memory side on replay — and `SPKERNEL`, which marks the end of the body
rather than being part of it.

The operations the CPU and the buffer meet at are `LoopStart` (what SPLOOP, SPLOOPD or SPLOOPW
asks for), `StageOutcome` (what a stage boundary asks the CPU to do) and `DrainOutcome` (what
completing a cycle asks for). `ReplayPackets` is the per-cycle set of active stages, kept on the
CPU and lent out each cycle, because a loop replaying for a million cycles must not allocate a
million times.

### Interrupts and the loop buffer

An interrupt never lands mid-iteration. `interruptible` implements SPRUFE8B §7.13.1's conditions
for a drain to start: on a stage boundary, termination condition false, not loading, not
reloading or waiting to reload, and at least four cycles into an SPLOOPD or SPLOOPW. When they
hold, `begin_interrupt_drain` stops starting iterations; the interrupt is taken only once the
buffer is idle, with the return address at the SPLOOP packet so the loop pipes back up.

Piping back up is visible in `Cpu::step`: while `pipe_up` is set, instructions in parallel with
the SPLOOP and the SPMASKed instructions arriving from program memory execute as NOPs, because
the prolog setup they carry already ran before the interrupt (SPRUFE8B §7.11.5 and §7.13.5).

An exception is different — it abandons the buffer with no epilog and records NTSR.SPLX so the
handler knows the loop cannot be resumed. See [Interrupts and exceptions](interrupts.md).

## Control registers

`Control` holds AMR, CSR (split into `csr_misc`, the endianness bit and a separate `sat` flag,
because CSR.SAT is set only by a saturating functional unit and cleared only by MVC), the
interrupt and exception registers, TSC as a pair with a latched high half, ILC and RILC, REP,
SSR, the Galois state (GPLYA, GPLYB, GFPGFR, which resets to `0x0700_001d`), and DNUM.

Two encodings name one register on read and another on write: `0x02` reads IFR and writes ISR,
`0x1d` reads EFR and writes ECR (SPRU732J Table 3-22). `register_name` and
`written_register_name` are the two sides, and they are what a trace line or a register dump
prints, because those are the names TI's tools print.

Saturation is queued like any other write-back — `saturation_sets_the_unit_flag_one_cycle_later`
is the test — which is the observable consequence of CSR.SAT being a side effect that lags by a
cycle.

## What else the executor covers

40-bit long arithmetic through register pairs, the packed SIMD set, the Galois field unit
(`GMPY`, `XORMPY` and their polynomial state), the complex-multiply family, and circular
addressing through AMR are all in `src/cpu.rs` with a test per family naming the manual example
it reproduces — `the_complex_multiplies_reproduce_the_manual_examples`,
`galois_multiply_and_bit_count_match_the_manual_examples`,
`circular_addressing_wraps_inside_the_block`, `dot_products_follow_the_manual_examples`. Where a
family's behaviour at a boundary is surprising, the test name says so:
`neg_of_the_most_negative_long_saturates`, `norm_of_a_zero_long_saturates_at_the_datapath_width`.
