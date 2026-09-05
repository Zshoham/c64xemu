---
name: c64xplus-sploop-buffer
description: "Implement the C64x+ SPLOOP hardware loop buffer, counters, loop-start variants, SPKERNEL, SPMASK, and interrupt pipe-down and pipe-up behavior."
---

# The SPLOOP buffer

*The C64x+ SPLOOP hardware loop buffer — the load/fetch/drain/reload state machine, LBC/ILC/RILC counters, SPLOOP vs SPLOOPD vs SPLOOPW, SPKERNEL and SPMASK, and the interrupt pipe-down/pipe-up protocol.*

**You cannot skip this.** The TI compiler emits `SPLOOP` for essentially every software-
pipelined inner loop on `-mv6400+`, so any real firmware — DSPLIB, NDK, codecs — will hit it
within the first few thousand instructions. It is the single most intricate part of the C64x+
core.

## What it is

A 14-execute-packet instruction buffer with its own sequencer. Instead of the compiler
emitting prolog + kernel + epilog, it emits **one scheduled iteration** of the loop, and the
hardware overlays time-shifted copies of it to build the software pipeline. This shrinks code
and — crucially — makes software-pipelined loops **interruptible**, which they are not when
the pipeline is unrolled in software.

## Terminology (SPRU732J §7.3)

- **ii** — iteration interval, cycles between successive iterations. Encoded in the
  `SPLOOP(D/W)` instruction.
- **stage** — the code executed in one iteration interval.
- **dynlen** — dynamic length: the length in cycles of one iteration of the loop body,
  counting from the cycle *after* the `SPLOOP(D)` instruction. Counts execute packets and NOP
  cycles; **does not count stall cycles**. Max 48.
- **kernel / prolog / epilog** — steady state / wind-up / wind-down. Prolog and epilog are
  each `dynlen - ii` cycles.

## State

| Name | Visible? | Role |
|---|---|---|
| loop buffer | no | 14 execute packets, each with a validity flag and the loading-counter value it was stored with |
| **LBC** ×2 | no | loop buffer count — index into the buffer. Cleared on `SPLOOP(D/W)`, incremented each cycle; when `LBC == ii` a **stage boundary** is reached, LBC resets to 0 and ILC decrements. Two of them, to support overlapped nested loops |
| loading counter | no | offset from loop start; determines dynlen |
| draining counter | no | retraces load order when invalidating |
| reloading counter | no | retraces load order when revalidating |
| **ILC** | yes (crlo 0x0d) | inner loop count, down-counter |
| **RILC** | yes (crlo 0x0e) | reload value for ILC in nested loops |
| `TSR.SPLX` | yes | 1 while the loop buffer is active |

**ILC and RILC have a 4-cycle load latency.** A value written by `MVC` is not usable for
4 cycles. This drives the SPLOOP/SPLOOPD distinction below.

## The three loop-start instructions

### `[cond] SPLOOP ii`
Iteration count comes from ILC, which must be loaded **at least 4 cycles before** the
`SPLOOP`. The optional predicate is *not* a loop condition — it marks a **nested** loop and
controls whether `SPKERNELR`/`SPMASKR` reloads ILC from RILC. Do not predicate a non-nested
`SPLOOP`.

### `[cond] SPLOOPD ii`
Same, but ILC may be loaded **in parallel with** the SPLOOPD. Because of the 4-cycle latency
the value written must be **pre-decremented** (SPRU732J Table 7-4). Saves up to 4 setup
cycles; unusable if the loop is too short to absorb the delay.

### `[cond] SPLOOPW ii`
A `do–while`. **Must** be predicated; the predicate is the loop-exit test and ILC is not
used. Two behavioural differences that matter:
- **No epilog.** When the termination condition is true at a stage boundary the loop exits
  abruptly — iterations already started but not finished simply do not finish. Loop bodies
  must be written so those partial early stages are harmless.
- The condition must be true **3 cycles before it is tested**, i.e. the register must be
  updated 4 cycles before the `SPKERNEL`.

`SPLOOPW` cannot be used for a nested loop.

All three: must be the **first instruction of their execute packet**, cannot share a packet
with any multicycle-NOP generator (`BNOP`, `NOP n`), and may otherwise only be paralleled
with plain `NOP` (§3.7.11.8).

## `SPKERNEL` / `SPKERNELR`

Marks the end of the loop body — placed **in parallel with the last execute packet** of the
body, and must be the first instruction in that packet.

- `SPKERNEL fstg, fcyc` — post-epilog code starts `fstg` complete stages plus `fcyc` cycles
  after the epilog begins. Post-epilog instructions are fetched from program memory and
  **overlaid** with epilog instructions coming from the buffer, so functional-unit conflicts
  are possible; avoid them with a large enough delay or with `SPMASK`.
- `SPKERNELR` — no arguments. Post-SPLOOP execution starts on the first epilog cycle, for
  nested loops needing perfect prolog/epilog overlap. **Mutually exclusive with `SPMASKR`
  in the same SPLOOP.**

## `SPMASK` / `SPMASKR`

Inhibits instructions on named functional units in the current execute packet. Two syntaxes:
`SPMASK D1` or marking the instruction with `||^`.

Behaviour depends on buffer state:

| Buffer state | Effect of SPMASK on the marked slot |
|---|---|
| loop running, instruction from buffer would use that unit | that buffered instruction is **inhibited** |
| buffer **loading** | the program-memory instruction **executes but is not stored into the buffer** |
| **resuming after an interrupt** | buffered instructions execute; **program-memory instructions are inhibited** and not loaded |

`SPMASKR` = `SPMASK` plus "this is the reload point" for a nested loop, when the reload
should not happen immediately after `SPKERNEL`.

`BNOP <displacement>` uses no functional unit, cannot be named by SPMASK, and is treated as
if SPMASKed (not stored in the buffer).

## The four buffer operations

| Operation | Meaning |
|---|---|
| **Load** | instruction fetched from program memory is executed **and** written into the buffer at the current LBC, marked valid on the next cycle |
| **Fetch** | valid instructions at the current LBC are read from the buffer and executed, in parallel with anything still coming from program memory |
| **Drain** | instructions at the current LBC are marked invalid this cycle and not executed |
| **Reload** | instructions at the current LBC are marked valid this cycle and executed |

Execution flow (SPRU732J Table 7-3), for a 4-stage loop:

| Phase | From program memory | From buffer |
|---|---|---|
| P0 | stage0 | — |
| P1 | stage1 | stage0 |
| P2 | stage2 | stage0, stage1 |
| K0 | stage3 | stage0, stage1, stage2 |
| Kn | — | stage0..stage3 |
| E0 | — | stage1..stage3 |
| E1 | — | stage2, stage3 |
| E2 | — | stage3 |

### Loading

On the cycle after `SPLOOP(D/W)`, LBC and the loading counter are cleared. Each cycle,
program-memory instructions are stored at index LBC together with the loading-counter value;
they become valid the next cycle. When `SPKERNEL` is seen, loading ends, `dynlen` is set to
the current loading counter, and **program memory fetch is disabled**. (If SPKERNEL lands on
the last kernel stage boundary, fetch may never actually stop.)

Errors detected at load time (assembler-checked, but worth asserting in the emulator):
- `MVC`, `ADDKPC`, or an S-unit branch in the body that is not SPMASKed
- a second `SPLOOP(D)` encountered
- loading counter reaching 48 before `SPKERNEL`
- a resource conflict when storing an instruction
- `dynlen < ii + 1` for SPLOOP(D), or `dynlen < ii` for SPLOOPW

### Draining

Starts on the cycle after either:
- the loop termination condition becomes true, or
- an interrupt is detected **and** the take-conditions are met.

The draining counter starts at 0 and increments each cycle. An instruction at index LBC is
invalidated when its stored loading-counter value equals the draining counter. When the
draining counter reaches `dynlen - ii`, draining is complete and any still-valid instructions
with loading counter `> dynlen - ii` are invalidated at once.

If draining for an interrupt, program-memory fetch stays disabled until the interrupt is
taken. If draining normally, fetch re-enables after the delay `SPKERNEL(R)` specified.

### Reloading

Starts on the cycle after the reload condition is true. The reloading counter runs 0 → dynlen,
revalidating instructions in original load order. **Reloading need not start on a stage
boundary**, and reload and drain can be at different offsets — hence the two LBCs; the one
not being used for draining is allocated to reloading.

A **hardware exception is raised** if the termination condition becomes true while reloading
and the previous invocation's draining counter has not reached `dynlen - ii` — i.e. two
invocations trying to drain at once (which happens if `RILC < ILC`).

## Termination

`SPLOOP`/`SPLOOPD`: ILC is decremented at every stage boundary; the loop starts draining on
the cycle after ILC reaches 0.

`SPLOOPW`: the predicate is tested at every stage boundary; termination skips the epilog.

## Buffer active/idle

Idle after reset. Becomes active on `SPLOOP(D/W)`. Returns to idle when:
- not reloading, and the last delay slot of a **taken branch** has passed
- the SPLOOPW stage-boundary termination condition is true
- an interrupt occurred and the loop has finished draining (before the vector is taken)
- an `SPLOOP(D)` loop finished draining and is not reloading

`TSR.SPLX` mirrors active/idle — **with one exception**: on `B IRP` returning to an
interrupted SPLOOP, ITSR is copied to TSR in E1 and `SPLX` becomes 1 from E2, before the
buffer has restarted. If the buffer state machine starts within the delay slots of a
`B IRP`/`B NRP`, it uses SPLX to decide "this is a restart, not a fresh start". SPLX is not
consulted when an SPLOOP starts outside those delay slots.

## Interrupts (SPRU732J §7.13) — get this exactly right

Interrupts are **automatically disabled 2 cycles before** an `SPLOOP(D/W)` instruction.

The buffer begins **interrupt draining** only when *all* of these hold:

- an enabled interrupt is pending and not architecturally blocked (e.g. by branch delay slots)
- the loop is **on a stage boundary**
- the termination condition is **false**
- the loop is not loading
- the loop is not draining
- the loop is not reloading or waiting to reload
- it is not within the first 3 CPU cycles after `SPLOOPD` or `SPLOOPW`
  (so ≥ 4 cycles of the loop must have run)
- for `SPLOOP`/`SPLOOPD`: `ILC >= ceil(dynlen / ii)` — prevents returning into a loop that
  would immediately early-exit. `ceil(dynlen/ii)` is the number of loading stages.

Consequence: loops starting (or reloading) with `ILC <= ceil(dynlen/ii) + 3` are **not
interruptible at all**. Interrupt latency grows by the epilog length.

### Pipe-down sequence

1. Hardware detects the interrupt.
2. Execute to the end of a stage boundary (termination condition false).
3. Drain the loop (execute the epilog).
4. Disable program-memory fetch.
5. Store the address of the execute packet **containing the `SPLOOP` instruction** into
   IRP (or NRP).
6. Copy TSR → ITSR (or NTSR) with `SPLX = 1`; clear `TSR.SPLX`.
7. Complete all pending register writes.
8. Branch to the ISR.

### Pipe-up sequence (returning)

1. `B IRP`/`B NRP` copies ITSR/NTSR → TSR (SPLX = 1).
2. Execution resumes **at the SPLOOP instruction**, and the loop pipes back up by re-running
   the prolog.
3. `SPLOOPD` executes **as `SPLOOP`** (ILC must already be right).
4. SPMASKed instructions **from program memory** execute as NOPs.
5. SPMASKed instructions **in the buffer** execute normally — the mask is ignored.
6. Instructions in parallel with the `SPLOOP(D/W)` instruction execute as NOPs.
7. `BNOP label,n` executes as `NOP n+1`.

NOP cycles from `ADDKPC`, `BNOP`, or protected `LD`s that are masked **are always executed**
when resuming.

**The ISR must save and restore ITSR/NTSR, ILC and RILC.** If the ISR modified ILC and the
return target is an unconditional `SPLOOP(D)`, ILC must be restored **4 cycles before** the
SPLOOP executes.

Software must not disable interrupts within 4 cycles of reaching dynlen while loading or
reloading: the loop can start draining for an interrupt that is no longer enabled by the time
draining finishes, in which case the loop just terminates and execution continues at the
post-SPKERNEL stream with no interrupt serviced.

### Exceptions differ from interrupts

An internal or external exception while the buffer is active:
- is recognised **immediately**; the buffer goes idle at once
- **no epilog is executed** — the loop does not drain
- TSR → NTSR with `NTSR.SPLX = 1`, `TSR.SPLX = 0`

Therefore **an SPLOOP interrupted by an exception can never be resumed**, which is exactly
why exception handlers must test `NTSR.SPLX` before `B NRP`.

NMI behaves the same way when exceptions are enabled: the loop terminates immediately and
`NTSR.SPLX` is set.

## Other restrictions

- Branches: taken branches make the buffer idle after the last delay slot (unless reloading).
- Cross-path stalls and AMR-related stalls are restricted inside SPLOOP (§7.16, §7.17).
- Only `NOP`, `NOP n` and `BNOP` may appear as unitless instructions in a loop body.
- §7.18 lists instructions that may not be placed in the loop buffer at all.

## Implementation strategy

Model the buffer explicitly rather than trying to "interpret the loop normally":

```c
typedef struct {
    bool     active;
    uint32_t ii, dynlen;
    uint32_t lbc[2];              /* one for drain, one for reload */
    uint32_t load_ctr, drain_ctr, reload_ctr;
    enum { LOADING, KERNEL, DRAINING, RELOADING } phase;
    bool     interrupt_draining;
    struct { exec_packet pkt; bool valid; uint32_t load_ctr; } slot[14];
} sploop_state;
```

Each cycle, execute the union of (valid buffer slots at the current LBC) and (the
program-memory execute packet, when fetch is enabled), applying SPMASK per the table above.
Do not special-case prolog/kernel/epilog — they *emerge* from load/fetch/drain/reload.

Build a test for this early: assemble the two-instruction copy loop from SPRU732J
Examples 7-4/7-5 and check the emulator reproduces Table 7-1's cycle-by-cycle instruction
flow exactly, then do the same for the SPLOOPW variant against Table 7-2.

## Files that expand on this

**Documents**

- `docs/reference/spru732j.pdf` (SPRU732J) ch.7, pp. 577-612 — the whole chapter is the specification
  - §7.3-7.5, pp. 579-585 — terminology, the hardware state, and the SPLOOP/SPLOOPD/SPLOOPW,
    SPKERNEL/SPKERNELR, SPMASK/SPMASKR instruction definitions
  - §7.6, pp. 586-590 — the worked copy-loop examples, **with Tables 7-1 and 7-2 giving
    complete cycle-by-cycle instruction flow**; these are the ground truth to test against
  - §7.7.3, pp. 592-596 — the load/fetch/drain/reload state machine, the loading/draining/
    reloading counters, and the load-time error conditions
  - §7.8, pp. 597-600 — the three execution patterns
  - §7.9-7.10, pp. 594-602 — termination and reload control for SPLOOP(D) and SPLOOPW
  - §7.11, pp. 602-606 — SPMASK worked examples for merging setup and reset code
  - §7.13, pp. 607-608 — the interrupt conditions and the pipe-down/pipe-up sequences
  - §7.15-7.18, pp. 609-611 — resource conflicts, stall restrictions, and the instructions
    that may not be placed in the buffer
- `docs/reference/spru732j.pdf` §3.6, p. 68 — SPMASK `||^` syntax
- `docs/reference/spru732j.pdf` §2.9.8 (ILC) and §2.9.12 (RILC), pp. 52-54 — the two visible counters
- `docs/reference/sprui04g.pdf` §4.8, p. 68 — "Utilizing the Loop Buffer Using SPLOOP": when the
  compiler emits SPLOOP and what shapes of loop body result, i.e. what you will actually run
- `docs/reference/sprui03f.pdf` — assembler handling of the SPLOOP directives and the caret marker

**Source**

- [`include/opcode/tic6x-opcode-table.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/opcode/tic6x-opcode-table.h;hb=refs/tags/binutils-2_44) — 14 `INSN()` entries
  for the sploop/spkernel/spmask family, including the `ii`, `fstgfcyc` and `mask` field
  encodings
- `tests/build/binutils-tic6x-vectors/sploop-bad-1.s` … `sploop-bad-7.s` (with matching `.d`/`.l`) —
  seven negative tests encoding the load-time rules as executable checks
- `tests/build/binutils-tic6x-vectors/insns-c674x-sploop.s` and `.d` —
  encoding coverage for the whole family
- [`gas/config/tc-tic6x.c`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gas/config/tc-tic6x.c;hb=refs/tags/binutils-2_44) — the assembler's SPLOOP state
  tracking, which mirrors the hardware rules it has to enforce
- `tests/build/dsplib-c64plus/packages/ti/dsplib/src/` — 46 kernels compiled from
  software-pipelined C; the densest available supply of real SPLOOP code
