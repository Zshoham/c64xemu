---
name: c64xplus-execution-model
description: "Implement C64x+ execution semantics, including delay slots, write-back queues, parallel packets, resource constraints, branches, and addressing modes."
---

# C64x+ execution model

*The C64x+ execution semantics an instruction-accurate emulator must reproduce — delay slots and the register write-back queue, parallel execute-packet semantics, resource constraints, branch behaviour, multicycle NOPs, and linear/circular addressing.*

The C64x+ is an 8-issue VLIW with **exposed pipeline latency**. There is no interlocking on
result availability: an instruction's result becomes visible after a fixed number of cycles,
and the compiler fills the gap. A functionally correct emulator therefore **cannot** write
results back immediately — it must model delay slots.

## The central data structure: a write-back queue

Model the machine as:

```
state:  regfile A[32], B[32], control regs, memory
        pending: list of (cycle_due, target, value, width)
```

Each cycle:
1. Fetch/decode the current execute packet.
2. **Read all sources** for every instruction in the packet from the architectural register
   file (all reads happen in E1, from the same snapshot — see "Parallel semantics").
3. Compute results.
4. Schedule each result into `pending` at `now + delay_slots`.
5. Commit everything in `pending` whose `cycle_due == now` **before** the next packet's
   reads. Order within a cycle does not matter, because writing two values to one register
   in one cycle is architecturally illegal.

### Delay slots (SPRU732J Table 3-3, p.64)

| Instruction type | Delay slots | Reads | Writes |
|---|---|---|---|
| NOP | 0 | — | — |
| Store | 0 | i | i |
| Single cycle | 0 | i | i |
| Two cycle (incl. .M non-multiply ops) | 1 | i | i+1 |
| Multiply (16×16) | 1 | i | i+1 |
| Four cycle | 3 | i | i+3 |
| Load | 4 | i | address modified at i; data written at i+4 |
| Branch | 5 | i | branch taken at i+5 |

Every instruction has **functional-unit latency 1** — a new instruction can start on the
unit every cycle regardless of its delay slots. So the unit is never a structural hazard
across cycles; only the write ports are.

The per-instruction pipeline type is in the binutils table
(`tic6x_pipeline_type`: `nop`, `1cycle`, `1616_m`, `store`, `mul_ext`, `load`, `branch`,
`4cycle`, …). The remaining types (`2cycle_dp`, `intdp`, `dpcmp`, `addsubdp`, `mpyi`,
`mpyid`, `mpydp`, `mpyspdp`, `mpysp2dp`) are C67x/C674x floating point — not on a C64x+.

**Finer-grained timing is available:** each operand in the binutils table carries
`low_first`/`low_last`/`high_first`/`high_last` cycle numbers (1 = E1), so a register pair's
halves can be written on different cycles. Use those rather than a single per-instruction
number if you want exactness.

### Timing subtleties worth encoding

- **Loads modify the address register at cycle i**, but deliver data at i+4. Code that does
  `LDW .D1 *A0++, A1` then immediately reads A0 is correct; reading A1 is not.
- **Saturation flag lag.** Saturating instructions set `CSR.SAT` **one cycle later than they
  write their result.** `SMPY` writes its result at the end of E2 (1 delay slot) but updates
  SAT after two delay slots. Model SAT as its own delayed write.
- `MVC` completes in E1 for the named register, but a write to ISR/ICR updates IFR **one
  clock later**. `B IRP` / `B NRP` update GIE / NMIE with **no** delay.
- A write to AMR immediately followed by a LD/ST/ADDA/SUBA using A4-A7 or B4-B7 for
  addressing costs a 1-cycle stall (timing only — ignorable in a functional model).

## Parallel semantics within an execute packet

All instructions in an execute packet read their sources from the **same architectural
state**, then all write. This makes swaps free:

```
    MV .L1 A0, A1
||  MV .L2 A1, A0        ; exchanges A0 and A1
```

Implement as read-all-then-commit; never let instruction *n*'s write be seen by instruction
*n+1* in the same packet.

## Resource constraints (SPRU732J §3.7, pp. 69-75)

A correct program never violates these. On a C64x+ a violation raises an **internal
exception** (`IERR.RCX` resource conflict / `IERR.RAX` resource access), so a good emulator
*checks* them rather than silently doing something. TI's own simulator flags these as errors
and halts (SPRU600I §2.3) — do the same, behind a flag.

1. **Same functional unit twice in one execute packet** — illegal.
2. **Same functional unit writing on the same cycle.** The .M unit has two 32-bit write
   ports, so a 4-cycle 32-bit result and a 2-cycle 32-bit result may land together. Any
   other combination conflicts. Notably `SMPY2` (64-bit, 3 delay slots) followed 2 cycles
   later by `MPY` (1 delay slot) attempts 96 bits through 64 bits of port — illegal.
3. **Cross paths.** Only 1X and 2X exist. Up to two units per side per execute packet may
   read via the cross path, **but only if they read the same register**. Three readers of
   the same cross-path operand is illegal; two readers of different registers on one cross
   path is illegal.
4. **Loads and stores.** Two memory operations in one packet must use different data paths
   (T1/T2). A **non-aligned** access (`LDNW`, `LDNDW`, `STNW`, `STNDW`) may not be paired
   with any other memory access; the other .D unit may still do non-memory work.
5. **Register reads:** at most **four reads of the same register** in one execute packet.
   Predicate reads do not count.
6. **Register writes:** two instructions may not write the same register on the same cycle.
7. **Multicycle NOP generators** cannot share an execute packet: `NOP n` (n>1), `IDLE`,
   `BNOP target,n` (any n, regardless of predication), `ADDKPC label,reg,n` (any n).
8. Long (40-bit) data has **no** C64x/C64x+ constraint — the C62x/C67x restrictions were
   removed by giving each functional unit its own data path.
9. Unitless instructions (DINT, RINT, IDLE, NOP n, SWE, SWENR, SPLOOP family, SPMASK family)
   have a mutual-exclusion matrix — see SPRU732J §3.7.11. In general each may only be paired
   with plain `NOP`.

## Branches

- A taken branch redirects on the cycle **after its 5 delay slots expire**. The five execute
  packets in the delay slots execute normally.
- A *conditional branch that does not take* still consumes the branch resource and still
  blocks interrupt recognition for those cycles.
- `B IRP` / `B NRP` restore TSR from ITSR / NTSR and are restricted to Supervisor mode.
- Branch target address for `B label` is PC-relative to the address of the **fetch packet**
  containing the branch.
- `CALLP` (compact, .S unit) is a call that writes the return address; `ADDKPC` computes a
  return address plus a NOP count.
- Interrupts cannot be taken in branch delay slots — this is architectural and matters, see
  `skills/c64xplus-interrupts-and-exceptions/SKILL.md`.

## Multicycle NOPs

`NOP n` stalls for n cycles. `BNOP target, n` is a branch plus n NOP cycles. `ADDKPC` and
`CALLP` likewise carry a NOP count. Model these as "the next n execute packets are
suppressed" rather than as inserted instructions, so that PC bookkeeping stays right.

`IDLE` halts the CPU until an interrupt arrives (restricted in User mode).

## Addressing modes (SPRU732J §3.8, pp. 76-79)

Address generation is on the .D units. Mode field, 4 bits (Table 3-6):

| Mode | Syntax | Effect |
|---|---|---|
| 0000 | `*-R[ucst5]` | negative offset |
| 0001 | `*+R[ucst5]` | positive offset |
| 0100 | `*-R[offsetR]` | negative register offset |
| 0101 | `*+R[offsetR]` | positive register offset |
| 1000 | `*--R[ucst5]` | pre-decrement |
| 1001 | `*++R[ucst5]` | pre-increment |
| 1010 | `*R--[ucst5]` | post-decrement |
| 1011 | `*R++[ucst5]` | post-increment |
| 1100 | `*--R[offsetR]` | pre-decrement |
| 1101 | `*++R[offsetR]` | pre-increment |
| 1110 | `*R--[offsetR]` | post-decrement |
| 1111 | `*R++[offsetR]` | post-increment |

Plus `*+B14/B15[ucst15]` — 15-bit constant offset from DP or SP, `.D2` only, no
pre/post modification.

**Scaling:** the offset is shifted left by 3/2/1/0 for doubleword/word/halfword/byte before
being added. `LDNDW`/`STNDW` also support a non-scaled form (the `sc` field), where the
offset is used as a raw byte count. `ADDA*`/`SUBA*` apply the same scaling by data size.

For pre-inc/dec and plain offsets, the computed value is the access address. For
post-inc/dec, the **pre-modification** value is the access address.

### Circular addressing

Only A4-A7 (on .D1) and B4-B7 (on .D2) can be circular. `AMR` selects per-register:
`00` linear, `01` circular using BK0, `10` circular using BK1. Block size is
`2^(N+1)` bytes where N is the 5-bit BK field — **unscaled, always bytes**. So an
8-word circular buffer needs 32 bytes, i.e. N = 4.

Implementation: compute the full linear result, then keep only bits `[N:0]` of the result
and bits `[31:N+1]` of the *original* base:

```
new = (base & ~mask) | ((base_op_offset) & mask)   where mask = (1 << (N+1)) - 1
```

The buffer must be aligned to a byte boundary equal to the block size.

**Non-aligned circular accesses** behave as the equivalent sequence of byte accesses, so a
read straddling the boundary correctly wraps and pulls bytes from both ends. On the C64x+
the circular buffer must be **at least 32 bytes** for non-aligned access; smaller is
undefined.

## Non-aligned access

`LDNW`, `LDNDW`, `STNW`, `STNDW` access words/doublewords at **any** byte boundary. There is
no alignment fault. Regular `LDW`/`LDDW` etc. assume alignment; the hardware simply ignores
the low address bits, so an emulator should mask rather than fault (or fault behind a
debug flag — this catches real firmware bugs).

## 40-bit and 64-bit values in register pairs

Pairs are `odd:even`, always written odd-first in assembly (`A1:A0`). The even register
holds the 32 LSBs.

- **40-bit long**: reads ignore the upper 24 bits of the odd register; writes **zero-fill**
  those 24 bits. The even register number is what is encoded in the opcode.
- **64-bit**: full 64 bits across the pair.
- .L and .S units have an extra 8-bit port for 40-bit long reads/writes; on .L and .S, the
  odd `dst` connects to the odd register file and the even `dst` to the even file.
- On the C64x+ .M unit, `src2` is 64 bits (it is 32 on the C64x) — relevant to
  `DDOTPL2`-class instructions.

## Cross-path stalls (timing only)

Reading a register through a cross path that was written in the **previous** cycle costs one
stall cycle, inserted automatically by hardware. No stall if the value came from a load
destination. Purely a performance effect — a functional emulator can ignore it, but note it
if you later add a cycle counter.

## Files that expand on this

**Documents**

- `docs/reference/spru732j.pdf` (SPRU732J)
  - §3.3, p. 64 — Table 3-3, the delay-slot and functional-unit-latency table
  - §3.7, pp. 69-75 — every resource constraint, including the unitless-instruction
    exclusion matrix in §3.7.11
  - §3.8, pp. 76-79 — addressing modes, the Table 3-6 mode field, and circular addressing
    including the non-aligned wrap-around worked examples
  - ch.4, pp. 509-532 — the pipeline itself (PG/PS/PW/PR/DP/DC/E1-E10), per-instruction-type
    pipeline diagrams in §4.2, and §4.4 "C64x+ DSP Differences"
  - §2.8.2, p. 35 — why MVC side effects (IFR via ISR/ICR, CSR.SAT) lag by a cycle
- `docs/reference/spru189f.pdf` (SPRU189F) ch.5 — the original, longer pipeline exposition for the
  base ISA; useful when SPRU732's condensed version is ambiguous
- `docs/reference/spru198k.pdf` (C6000 Programmer's Guide) — how the constraints look
  from the code-generation side, which is the shape of code you will actually execute

**Source**

- [`include/opcode/tic6x-opcode-table.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/opcode/tic6x-opcode-table.h;hb=refs/tags/binutils-2_44) — each `INSN()` carries
  a `tic6x_pipeline_type` and per-operand read/write cycle numbers
  (`low_first`/`low_last`/`high_first`/`high_last`, 1 = E1), which is finer-grained timing
  than Table 3-3
- [`include/opcode/tic6x.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/opcode/tic6x.h;hb=refs/tags/binutils-2_44) — `tic6x_pipeline_type` and
  `tic6x_operand_info` definitions
- [`gcc/config/c6x/c6x-sched.md`](https://github.com/gcc-mirror/gcc/blob/29a97120429f2255311285cdff86b81ba3ad8e63/gcc/config/c6x/c6x-sched.md) and [`c6x-sched.md.in`](https://github.com/gcc-mirror/gcc/blob/29a97120429f2255311285cdff86b81ba3ad8e63/gcc/config/c6x/c6x-sched.md.in) — GCC's unit-reservation and
  latency model, an independent second opinion on per-instruction timing
- [`gcc/config/c6x/c6x.md`](https://github.com/gcc-mirror/gcc/blob/29a97120429f2255311285cdff86b81ba3ad8e63/gcc/config/c6x/c6x.md) — instruction semantics expressed as RTL
- [`lib/Target/TMS320C64X/TMS320C64XSchedule.td`](https://github.com/alexjordan/LLVM-TMS320C64X/blob/e17e694eb6896d9221f617bc04619bbc51c80e14/lib/Target/TMS320C64X/TMS320C64XSchedule.td) and
  [`TMS320C64XHazardRecognizer.cpp`](https://github.com/alexjordan/LLVM-TMS320C64X/blob/e17e694eb6896d9221f617bc04619bbc51c80e14/lib/Target/TMS320C64X/TMS320C64XHazardRecognizer.cpp) — a third model, including the hazard rules
- `docs/reference/spru600i.pdf` §2.3, pp. 24-25 — the exact resource-conflict classes
  TI's own simulator detects, what it does not detect, and its error-message format
