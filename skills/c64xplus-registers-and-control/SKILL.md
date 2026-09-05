---
name: c64xplus-registers-and-control
description: "Implement C64x+ register files, wide register pairs, control-register bit layouts and MVC rules, Galois-field state, counters, and privilege."
---

# C64x+ architectural state

*The C64x+ architectural state — the two 32-register files, 40/64-bit pairs, the complete control register file with bit layouts and MVC access rules, Galois field state, the time-stamp counter, and the Supervisor/User privilege model.*

## General-purpose register files

Two files, **A0-A31** and **B0-B31**, each 32 bits. Units ending in 1 (.L1 .S1 .M1 .D1)
write file A; units ending in 2 write file B. Cross paths 1X and 2X allow one operand per
side per cycle from the opposite file.

**Register pairs** (`odd:even`, written odd-first): `A1:A0` … `A31:A30`, `B1:B0` … `B31:B30`.
32 valid pairs. The even register holds the 32 LSBs.

- **40-bit long values**: the even register holds bits 31:0, the odd register bits 39:32.
  Reads *ignore* the upper 24 bits of the odd register; writes *zero-fill* them. The opcode
  encodes the even register number.
- **64-bit values**: full pair.
- **Packed types**: four 8-bit or two 16-bit lanes in one 32-bit register; four 16-bit lanes
  in a pair.

Registers used as predicates: A0, A1, A2, B0, B1, B2 only.

Circular addressing is available only on **A4-A7** (via .D1) and **B4-B7** (via .D2).

## Control register file

Accessed **only by `MVC`, only on `.S2`**. `MVC` completes in E1 for the named register
(single cycle, both directions). Some side effects lag by one cycle — see below.

Register numbers (`crlo`, and `crhi_mask` which must AND to 0 with the instruction's crhi)
come from [`include/opcode/tic6x-control-registers.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/opcode/tic6x-control-registers.h;hb=refs/tags/binutils-2_44):

| Reg | crlo | crhi_mask | Access | ISA | Purpose |
|---|---|---|---|---|---|
| AMR | 0x00 | 0x10 | R/W | C62x | Addressing mode |
| CSR | 0x01 | 0x10 | R/W | C62x | Control/status |
| DIER | 0x19 | — | R/W | C64x+ | Debug interrupt enable (emulation) — *not* in the binutils table |
| IFR | 0x02 | 0x1d | R | C62x | Interrupt flags |
| ISR | 0x02 | 0x10 | W | C62x | Interrupt set |
| ICR | 0x03 | 0x10 | W | C62x | Interrupt clear |
| IER | 0x04 | 0x10 | R/W | C62x | Interrupt enable |
| ISTP | 0x05 | 0x10 | R/W | C62x | Interrupt service table pointer |
| IRP | 0x06 | 0x10 | R/W | C62x | Interrupt return pointer |
| NRP | 0x07 | 0x10 | R/W | C62x | NMI/exception return pointer |
| TSCL | 0x0a | 0x1f | R/W | C64x+ | Time-stamp counter low |
| TSCH | 0x0b | 0x1f | R | C64x+ | Time-stamp counter high |
| ILC | 0x0d | 0x1f | R/W | C64x+ | SPLOOP inner loop count |
| RILC | 0x0e | 0x1f | R/W | C64x+ | SPLOOP reload inner loop count |
| REP | 0x0f | 0x1f | R/W | C64x+ | Restricted entry point (SWENR target) |
| PCE1 | 0x10 | 0x0f | R | C62x | E1-phase program counter |
| DNUM | 0x11 | 0x1f | R | C64x+ | DSP core number |
| SSR | 0x15 | 0x1f | R/W | C64x+ | Saturation status |
| GPLYA | 0x16 | 0x1f | R/W | C64x+ | GMPY polynomial, A side |
| GPLYB | 0x17 | 0x1f | R/W | C64x+ | GMPY polynomial, B side |
| GFPGFR | 0x18 | 0x1f | R/W | C64x | Galois field multiply control |
| TSR | 0x1a | 0x1f | R/W | C64x+ | Task state |
| ITSR | 0x1b | 0x1f | R/W | C64x+ | Interrupt task state |
| NTSR | 0x1c | 0x1f | R/W | C64x+ | NMI/exception task state |
| EFR | 0x1d | 0x1f | R | C64x+ | Exception flags |
| ECR | 0x1d | 0x1f | W | C64x+ | Exception clear |
| IERR | 0x1f | 0x1f | R/W | C64x+ | Internal exception report |

(FADCR 0x12 / FAUCR 0x13 / FMCR 0x14 are C67x floating point — not present on a C64x+.)

Note EFR/ECR share address 0x1d (read vs. write) and IFR/ISR share 0x02, so `MVC` direction
selects the register.

**Cross-check this table against SPRU732J Table 3-22** (`docs/reference/spru732j.pdf`, p. 330, inside
the MVC instruction description), which is the authoritative list and also gives a per-register
User-mode access column. Every `crlo` above matches it. Two places where the sources disagree,
worth knowing before you trust either:

- **DIER** is documented in SPRU732J §2.9.1 and Table 3-22 (crlo `11001` = 0x19, Supervisor
  R/W, User access raises an exception) but is **absent from the binutils header**, which is
  why it has no `crhi_mask` above. Implement it from the TI table.
- **TSCL** is listed as read-only in Table 3-22, but §2.9.14.2 says a *write* to TSCL is what
  enables the counter, and binutils marks it `read_write`. The write-to-enable behaviour is
  real; treat Table 3-22's "R" as an error in that one row.

### AMR — addressing mode register

```
31-26 Reserved | 25-21 BK1 | 20-16 BK0
15-14 B7MODE 13-12 B6MODE 11-10 B5MODE 9-8 B4MODE
 7-6  A7MODE  5-4  A6MODE  3-2  A5MODE 1-0 A4MODE
```
Mode: `00` linear (reset default), `01` circular via BK0, `10` circular via BK1, `11`
reserved. Block size = `2^(N+1)` bytes where N is the BK field.

### CSR — control status register

```
31-24 CPU ID (R)   23-16 REVISION ID (R)
15-10 PWRD  9 SAT  8 EN  7-5 PCC  4-2 DCC  1 PGIE  0 GIE
```
- **CPU ID = 0x10 identifies a C64x+** (0x0C is a C64x). Return 0x10 from the emulator.
- `SAT` is set by functional units, cleared only by `MVC`. A set by a unit wins over a
  simultaneous `MVC` clear. Set **one full cycle after** the instruction writes its result.
  Not modified by a conditional instruction whose predicate is false.
- `EN` (bit 8) reads the endianness the device booted in.
- `PGIE` (bit 1) is physically the same bit as `ITSR.GIE`.
- `GIE` (bit 0) is physically the same bit as `TSR.GIE`.
- `PCC`/`DCC` are ignored on the C64x+ (legacy cache control; use L1PCFG/L1DCFG instead).
- `PWRD` field values: 0 = none, 0x09 = PD1 wake on enabled interrupt, 0x11 = PD1 wake on
  enabled or non-enabled interrupt, 0x1A = PD2 wake by reset, 0x1C = PD3 wake by reset.
- `PWRD`, `PCC`, `DCC`, `PGIE` are Supervisor-write-only.

### TSR — task state register (C64x+ only)

```
15 IB | 14 SPLX | 13-11 rsvd | 10 EXC | 9 INT | 8 rsvd | 7-6 CXM | 5 rsvd
 4 DBGM | 3 XEN | 2 GEE | 1 SGIE | 0 GIE
```

| Bit | Meaning | Writability |
|---|---|---|
| IB | interrupts were blocked in the previous cycle | hardware only |
| SPLX | an SPLOOP is executing | hardware only |
| EXC | currently processing an exception | clearable by MVC in Supervisor |
| INT | currently processing an interrupt | hardware only |
| CXM | current execution mode: 0 = Supervisor, 1 = User | hardware only |
| DBGM | emulator debug mask | writable in both modes |
| XEN | maskable exception (EXCEP) enable | Supervisor only |
| GEE | global exception enable — **settable only, never clearable except by reset** | Supervisor set |
| SGIE | saved GIE, used by DINT/RINT | both modes |
| GIE | global interrupt enable (same bit as CSR.GIE) | both modes |

`ITSR` and `NTSR` mirror TSR's layout. `NTSR` adds **bit 16 HWE** — "hardware exception
taken (NMI, EXCEP, or internal)"; set by taking a hardware exception, cleared by `SWE`/`SWENR`.
Both ITSR and NTSR are inaccessible in User mode.

### ISTP — interrupt service table pointer

```
31-10 ISTB (R/W) | 9-5 HPEINT (R) | 4-0 zero
```
`ISTB` is the base of the interrupt service table, so the IST must be 1 KB aligned.
`HPEINT` is the number of the **highest-priority pending-and-enabled interrupt** — read-only,
maintained by hardware from IFR & IER. It need not be enabled by GIE or NMIE (except NMI).
The whole register therefore reads as the address of the ISFP to branch to. Low 5 bits are
always 0 because fetch packets are 32-byte aligned.

Reset restores ISTB to a device default, so a relocated IST never serves the RESET vector.
On the C6455 the reset vector location is set by boot configuration — see the data manual.

### IERR — internal exception report

```
8 MSX | 7 LBX | 6 PRX | 5 RAX | 4 RCX | 3 OPX | 2 EPX | 1 FPX | 0 IFX
```
Missed stall / SPLOOP buffer / privilege / resource access / resource conflict / opcode /
execute packet / fetch packet / instruction fetch. Supervisor-only. Multiple bits can be
set simultaneously and the same bit may come from different causes.

### Time-stamp counter (TSCL / TSCH)

A free-running 64-bit counter that advances each CPU clock. Cleared to 0 by reset with
**counting disabled**; writing anything to `TSCL` enables it (the value is discarded) and
counting starts the cycle *after* the `MVC`, so:

```
MVC B0,TSCL   ; start
MVC TSCL,B0   ; B0 = 0
MVC TSCL,B1   ; B1 = 1
```

Once enabled it cannot be disabled by software — only reset or full CPU power-down stops it.
Read `TSCL` first, then `TSCH`: **reading TSCL latches the upper 32 bits into TSCH**, which
is the only way to get a coherent 64-bit value. Both registers are read-only via MVC in the
read direction. Emulate as `cycles_since_enable`.

### Galois field state

`GFPGFR` holds the field size and polynomial used by `GMPY4` (reset defaults: size = 7,
polynomial = 0x1D). `GPLYA`/`GPLYB` supply the polynomial for the C64x+ `GMPY` instruction,
selected by which side's .M unit executes it. Timing note: if the execute packet immediately
after an `MVC` to GFPGFR contains `GMPY4`, the **new** value applies.

### SSR — saturation status register

Per-unit saturation status on the C64x+, distinct from CSR.SAT.

### PCE1

Read-only program counter for the instruction currently in E1. Useful for the emulator's
own debug output and required by exception handlers.

## Privilege model (SPRU732J ch.8)

Two modes, tracked in `TSR.CXM`: **Supervisor** (0) and **User** (1).

Reset enters Supervisor mode. Transitions:

| Event | Effect |
|---|---|
| Interrupt | → Supervisor; old mode saved in ITSR.CXM; return via `B IRP` |
| NMI / exception | → Supervisor; old mode saved in NTSR.CXM; return via `B NRP` |
| OS service request (`SWE` / `SWENR`) | → Supervisor; return via `B NRP` |
| `B IRP` | restore mode from ITSR |
| `B NRP` | restore mode from NTSR |

The restored mode takes effect **coincident with the branch target**; the branch's delay
slots still execute in Supervisor mode.

### User-mode restrictions

**Fully restricted control registers** (access raises an exception, setting both `IERR.RAX`
and `IERR.PRX`). SPRU732J §8.2.4.1 lists: ECR, EFR, ICR, IER, IERR, IFR, ISTP, ITSR, NTSR, REP.

Table 3-22 additionally marks **ISR** and **DIER** as raising an exception on User-mode
access, and the two lists are inconsistent in the TI document itself. Table 3-22 is the more
detailed and more plausible of the two — ISR can set interrupt flags, so leaving it open to
User mode would defeat the point — so implement the union of both lists: ECR, EFR, ICR, IER,
IERR, IFR, ISR, ISTP, ITSR, NTSR, REP, DIER.

**Partially restricted** (readable, only some bits writable, no exception raised):
- `CSR`: PGIE, PWRD, PCC, DCC not writable. GIE and SAT behave normally.
- `TSR`: only GIE and SGIE writable.

**Restricted instructions** (raise an exception, setting `IERR.OPX` and `IERR.PRX`):
`B IRP`, `B NRP`, `IDLE`.

### Entering Supervisor mode from User mode

`SWE` branches to the NMI/exception vector (offset 0x20 from ISTP). `SWENR` branches to the
address in `REP` — a "restricted entry point" the supervisor pre-initialises, so User code
cannot pick where it enters the OS. Both clear `NTSR.HWE`, which is how the handler
distinguishes a software service request from a hardware exception.

## DINT / RINT

`DINT` copies GIE into `TSR.SGIE` and clears GIE. `RINT` restores GIE from SGIE. Both are
unitless and have a long list of instructions they may not be paired with (see
SPRU732J §3.7.11.3 and §3.7.11.6); only plain `NOP` is allowed alongside.

## Atomic operations (SPRU732J ch.9)

`LL` (load linked), `SL` (store linked), `CMTL` (commit linked) provide load-linked /
store-conditional semantics for spin locks, shared counters and compare-and-swap. Valid
sequences are constrained (§9.2.4). For a single-core emulator these can be implemented as
plain load/store plus a reservation flag that nothing else ever clears — but keep the
reservation state in the CPU struct so multi-core or DMA-aware variants stay possible.

## Suggested state struct

```c
typedef struct {
    uint32_t a[32], b[32];
    uint32_t amr, csr, ifr, ier, istp, irp, nrp;
    uint32_t tsr, itsr, ntsr, efr, ierr, rep, ssr;
    uint32_t ilc, rilc;
    uint32_t gfpgfr, gplya, gplyb;
    uint64_t tsc;  bool tsc_running;
    uint32_t pce1;              /* PC of the packet in E1 */
    uint32_t pc;                /* fetch pointer */
    /* delayed writes, SPLOOP buffer, memory, INTC ... */
} c64xp_cpu;
```

Keep `CSR.GIE` and `TSR.GIE` as one storage location, and `CSR.PGIE` and `ITSR.GIE` as one
storage location — the architecture says they are physically the same bits, and firmware
relies on it.

## Files that expand on this

**Documents**

- `docs/reference/spru732j.pdf` (SPRU732J)
  - §2.1-2.7, pp. 26-33 — data paths, register files, 40-bit storage scheme, functional-unit
    capability table, cross paths, Galois field background
  - §2.8, pp. 34-46 — AMR, CSR, GFPGFR, ICR, IER, IFR, IRP, ISR, ISTP, NRP, PCE1 with full
    bit tables; §2.8.1-2.8.2 cover MVC access and timing
  - §2.9, pp. 46-58 — the C64x+ extensions: DIER, DNUM, ECR, EFR, GPLYA/GPLYB, IERR, ILC,
    ITSR, NTSR, REP, RILC, SSR, TSCL/TSCH, TSR
  - ch.8, pp. 613-618 — the privilege model, the restricted-register and restricted-instruction
    lists, and OS entry via SWE/SWENR and REP
  - ch.9, pp. 619-624 — LL/SL/CMTL atomics with valid-sequence rules and worked examples
- `docs/reference/spraa84a.pdf` (SPRAA84A) — a 12-page summary of everything the `+`
  adds over a plain C64x, including the new control registers
- `docs/reference/sprufe8b.pdf` — the C674x register file description, often worded more
  clearly for the registers the two cores share

**Source**

- [`include/opcode/tic6x-control-registers.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/opcode/tic6x-control-registers.h;hb=refs/tags/binutils-2_44) — all 29 control
  registers with their `crlo` numbers, `crhi_mask`, read/write permission and ISA gating;
  this is the authoritative encoding of the MVC register numbers
- [`gdb/tic6x-tdep.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/tic6x-tdep.h;hb=refs/tags/binutils-2_47) and [`tic6x-core.xml`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/features/tic6x-core.xml;hb=refs/tags/binutils-2_47) /
  [`tic6x-gp.xml`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/features/tic6x-gp.xml;hb=refs/tags/binutils-2_47) / [`tic6x-c6xp.xml`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/features/tic6x-c6xp.xml;hb=refs/tags/binutils-2_47) — the register numbering and grouping a debugger expects to see
- [`arch/c6x/include/asm/processor.h`](https://github.com/torvalds/linux/blob/v5.10/arch/c6x/include/asm/processor.h) and [`arch/c6x/kernel/process.c`](https://github.com/torvalds/linux/blob/v5.10/arch/c6x/kernel/process.c) — how a
  real OS defines and switches this register set
- [`arch/c6x/include/asm/irqflags.h`](https://github.com/torvalds/linux/blob/v5.10/arch/c6x/include/asm/irqflags.h) — GIE/SGIE manipulation via
  DINT/RINT in practice
