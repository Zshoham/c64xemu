---
name: c64xplus-instruction-decoding
description: "Decode C64x+ fetch and execute packets, p-bits, compact 16-bit headers, predication, functional units, and binutils opcode tables."
---

# C64x+ instruction decoding

*How to decode TMS320C64x+ instructions — fetch packets, p-bits, the compact 16-bit header format, predication, functional-unit assignment, and how to build the decoder from binutils' machine-readable tables instead of transcribing the PDF opcode maps.*

## The unit of fetch is a packet, not an instruction

Program memory is read **8 words (32 bytes) at a time** — a *fetch packet*, always aligned
to a 32-byte boundary. On the C64x that is always exactly 8 instructions; **on the C64x+ a
header-based fetch packet can hold up to 14**, because seven of its words may each carry two
16-bit compact instructions and the eighth is the header. Within a fetch packet, instructions
are grouped into *execute packets*: the set that issues in the same cycle. Decoding is
therefore a two-level process:

1. Split the fetch packet into execute packets.
2. Decode each 32-bit or 16-bit instruction within them.

An execute packet holds at most 8 instructions and **every instruction in it must use a
different functional unit**. Execute packets may cross fetch-packet boundaries but are still
limited to 8 instructions total.

### p-bits (non-compact packets)

Bit 0 of each 32-bit instruction is the `p` bit. Scan left to right (low to high address):
if `p` of instruction *I* is 1, instruction *I+1* executes in parallel with *I*; if 0,
*I+1* starts the next cycle. The last instruction of an execute packet has `p = 0`.

### Branching into the middle of an execute packet

If a branch targets an address inside an execute packet, **all instructions at lower
addresses are ignored** — including ones in the same execute packet. Model this by starting
execution at the branch target word and consuming forward until a cleared p-bit; do not
rewind to the packet start.

## Compact (16-bit) instructions — the C64x+ header format

The C64x+ adds 16-bit instructions. They are enabled per-fetch-packet by replacing **word 7**
of the fetch packet with a header word. Detection (from `tic6x-dis.c:193`):

```
header = word7
is_header_based = (header & 0xF0000000) == 0xE0000000
```

Header layout (SPRU732J §3.9.2, p.81):

| Bits | Field | Meaning |
|---|---|---|
| 31-28 | `1110` | header marker |
| 27-21 | Layout `L7..L1` | bit set ⇒ that word holds **two** 16-bit instructions; clear ⇒ one 32-bit instruction. L1 = word 0 … L7 = word 6 |
| 20 | PROT | 1 ⇒ 4 NOP cycles are inserted after **every** LD in this fetch packet (32-bit LDs too) |
| 19 | RS | 0 ⇒ compact 3-bit register fields select A0-A7/B0-B7; 1 ⇒ A16-A23/B16-B23 |
| 18-16 | DSZ | primary/secondary data size for compact LD/ST (table below) |
| 15 | BR | 1 ⇒ compact .S-unit instructions decode as branches |
| 14 | SAT | 1 ⇒ ADD/SUB/SHL/MPY/MPYH/MPYLH/MPYHL decode as SADD/SUBS/SSHL/SMPY/SMPYH/SMPYLH/SMPYHL |
| 13-0 | p-bits `P13..P0` | parallel bits for the 14 possible compact slots; P0 = word 0 low half, P1 = word 0 high half, … P13 = word 6 high half |

DSZ (SPRU732J Table 3-10):

| DSZ | Primary | Secondary |
|---|---|---|
| 000 | W | BU |
| 001 | W | B |
| 010 | W | HU |
| 011 | W | H |
| 100 | DW/NDW | W |
| 101 | DW/NDW | B |
| 110 | DW/NDW | NW |
| 111 | DW/NDW | H |

Key decoding consequences:

- **The header is decode state, not just layout.** SAT, BR and DSZ change what a given
  16-bit opcode *means*. binutils folds them into the opcode word before table lookup
  (`tic6x-dis.c:174`, `tic6x_extract_16` ORs `sat`, `br`, `dsz` into the 16-bit value at
  `TIC6X_COMPACT_*_POS`). Copy that trick — it lets one flat table cover all header variants.
- For a word whose Layout bit is 0 (32-bit instruction inside a header-based packet), the
  **header p-bit must be zero** and the instruction's own p-bit is authoritative.
- When an execute packet spans two fetch packets, **each instruction uses its own fetch
  packet's header**. Two headers are live simultaneously.
- Compact instructions have **no creg/z field — they always execute unconditionally**.
- An 8-instruction execute packet cannot involve any header-based fetch packet.
- Execute packets spanning a fetch-packet boundary where either packet is header-based may
  not be a branch target (the sole exception is returning from an interrupt taken in the
  cycle before such a packet reached E1).

## Predication

The top 4 bits of every 32-bit instruction are `creg` (31-29) and `z` (28):

| creg | z | Condition |
|---|---|---|
| 000 | 0 | unconditional |
| 000 | 1 | reserved |
| 001 | z | B0 |
| 010 | z | B1 |
| 011 | z | B2 |
| 100 | z | A1 |
| 101 | z | A2 |
| 110 | z | A0 |
| 111 | x | reserved |

`z = 1` tests for zero, `z = 0` tests for non-zero. The predicate register is read **at the
beginning of E1**, i.e. with the same timing as any other source read — a value written by
the immediately preceding execute packet is visible.

Important: a false predicate suppresses the instruction's effects but the instruction still
*occupies* its functional unit and still counts for resource-conflict purposes. Mutually
exclusive instructions that share a resource are still illegal in the same execute packet.

## Building the decoder: use binutils, don't retype the PDF

[binutils 2.44 tic6x support](https://sourceware.org/git/?p=binutils-gdb.git;a=tree;f=opcodes;hb=refs/tags/binutils-2_44) contains the whole encoding spec in machine-readable form.
Generating your decoder from it is far more reliable than transcribing SPRU732 Appendices C-H.

| File | Contents |
|---|---|
| `include/opcode/tic6x-insn-formats.h` | 127 `FMT(...)` entries — 40 × 32-bit, 87 × 16-bit. Each gives `num_bits`, `cst_bits`, `mask`, and every field as `(name, low_pos, width)` with support for split/composite fields |
| `include/opcode/tic6x-opcode-table.h` | 589 `INSN(...)` entries over 235 distinct mnemonics: mnemonic, functional unit, format, pipeline type, ISA gating, flags, fixed field values, per-operand info, and field-encoding rules |
| `include/opcode/tic6x-control-registers.h` | 29 control registers with `crlo`, `crhi_mask` and read/write permission |
| `include/opcode/tic6x.h` | The structs/enums that give all of the above meaning |
| `opcodes/tic6x-dis.c` | A working reference decoder, including the header handling above |
| `gas/config/tc-tic6x.c` | The encoder — resolves ambiguity the decoder tables leave open |

### How an entry reads

```c
INSN(ldw, d, load_store, load, C62X,
     TIC6X_FLAG_LOAD|TIC6X_FLAG_NO_CROSS|TIC6X_FLAG_PREFER(1),
     FIX2(FIX(op, 6), FIX(r, 0)),
     OP2(ORMEMSW, OWDREG5),
     ENC6(ENC(s, data_fu, 0), ENC(y, fu, 0), ENC(mode, mem_mode, 0),
          ENC(offsetR, mem_offset, 0), ENC(baseR, reg, 0),
          ENC(srcdst, reg, 1)))
```

- `d` — functional unit base (`d`/`l`/`m`/`s`/`nfu`)
- `load_store` — the format, giving the bit positions
- `load` — pipeline type (`tic6x_pipeline_type`), which determines delay slots
- `C62X` — ISA gating. Filter to `TIC6X_INSN_C62X | TIC6X_INSN_C64X | TIC6X_INSN_C64XP`
  (543 of the 589 entries). Reject C67x/C674x floating-point encodings on a C64x+ core.
- `FIX2(...)` — fixed field values that select this opcode within the format
- `OP2(...)` — operand forms with **per-operand read/write cycles**
  (`low_first`, `low_last`, `high_first`, `high_last`; 1 = E1). This is exactly the timing
  information a delay-slot-accurate interpreter needs — see `skills/c64xplus-execution-model/SKILL.md`.

### Suggested build step

Write a generator that parses the three headers (they are plain C macro invocations — a
small Python parser handles them; or compile them into a C program that dumps a table as
JSON) and emits your emulator's decode tables plus a `switch` skeleton for semantics.
Regenerating is cheap, so re-run it whenever you touch the tables.

### Cross-check decoders

[capstone 4.0.2 TMS320C64x](https://github.com/capstone-engine/capstone/tree/4.0.2/arch/TMS320C64x) is an independent, LLVM-TableGen-derived decoder
(`TMS320C64xGenDisassemblerTables.inc`) with its own operand model, and
[`lib/Target/TMS320C64X/TMS320C64XInstrFormats.td`](https://github.com/alexjordan/LLVM-TMS320C64X/blob/e17e694eb6896d9221f617bc04619bbc51c80e14/lib/Target/TMS320C64X/TMS320C64XInstrFormats.td) is the
`.td` source it came from. When binutils and capstone disagree about an encoding, the TI
`dis6x` disassembler in `compiler/bin/` is the tiebreaker — see `skills/c64xemu-validation/SKILL.md`.

## Functional-unit assignment

The instruction encodes a base unit (.D/.L/.M/.S/none) plus:

- `s` bit — side (1 ⇒ A/unit 1, 2 ⇒ B/unit 2), so `.L` + `s` gives `.L1` or `.L2`
- `x` bit — cross path used for src2 (or src1/src2 on .L)
- `y` bit (loads/stores) — which data path (T1/T2) carries the data
- Some instructions are constrained: `TIC6X_FLAG_SIDE_B_ONLY` (e.g. `B IRP`, `MVC`,
  15-bit-offset loads), `TIC6X_FLAG_NO_CROSS`.

Unitless instructions (`nfu`): NOP, IDLE, the SPLOOP family, SPMASK(R), SWE, SWENR, DINT,
RINT. These have their own parallelism restrictions — see SPRU732J §3.7.11.

## Endianness

The core supports both. Endianness affects instruction word extraction (`tic6x_extract_32`),
data access, and the ABI. Little-endian is by far the common configuration; make it a
construction-time parameter of the decoder rather than a compile-time constant, since
DSPLIB and the toolchain ship both `_LE_` and `_BE_` variants.

## Files that expand on this

**Documents**

- `docs/reference/spru732j.pdf` (SPRU732J, C64x/C64x+ CPU and Instruction Set Reference)
  - §3.1-3.2, pp. 60-63 — operation/opcode notation and how to read the opcode maps
  - §3.4, pp. 65-67 — fetch packets, p-bits, the three p-bit patterns, branching into the
    middle of an execute packet
  - §3.5, p. 68 — the creg/z predication table
  - §3.9, pp. 80-86 — compact instructions: header format, Layout/Expansion/p-bits fields,
    the DSZ table, execute-packet restrictions, and the list of available compact instructions
  - §3.11, pp. 87-508 — per-instruction descriptions, each with its exact opcode diagram
  - Appendices C-H, pp. 638-681 — the 32-bit and 16-bit opcode maps per functional unit.
    Use to sanity-check generated tables, not as the source.
- `docs/reference/sprufe8b.pdf` is the C674x equivalent — the binutils format tables are ordered to
  match its appendices, so it is often the easier cross-reference when reading them.

**Source**

- [`include/opcode/tic6x-insn-formats.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/opcode/tic6x-insn-formats.h;hb=refs/tags/binutils-2_44) — 127 `FMT()` entries
  (40 × 32-bit, 87 × 16-bit) giving `cst_bits`, `mask` and every field's bit position
- [`include/opcode/tic6x-opcode-table.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/opcode/tic6x-opcode-table.h;hb=refs/tags/binutils-2_44) — 589 `INSN()` entries
  over 235 mnemonics, with ISA gating, fixed fields, operand forms and encoding rules
- [`include/opcode/tic6x.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/opcode/tic6x.h;hb=refs/tags/binutils-2_44) — the structs and enums that give
  those two files meaning (`tic6x_insn_field_id`, `tic6x_operand_form`,
  `tic6x_coding_method`, `TIC6X_INSN_*` ISA bits, `TIC6X_FLAG_*`)
- [`opcodes/tic6x-dis.c`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=opcodes/tic6x-dis.c;hb=refs/tags/binutils-2_44) — reference decoder;
  `tic6x_check_fetch_packet_header()` at line 193 and `tic6x_extract_16()` at line 174 are
  the header-handling logic to copy
- [`gas/config/tc-tic6x.c`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gas/config/tc-tic6x.c;hb=refs/tags/binutils-2_44) — the encoder, for anything the
  decoder tables leave ambiguous
- [`arch/TMS320C64x/TMS320C64xGenDisassemblerTables.inc`](https://github.com/capstone-engine/capstone/blob/4.0.2/arch/TMS320C64x/TMS320C64xGenDisassemblerTables.inc) — an independent
  decoder to cross-check against
- [`lib/Target/TMS320C64X/TMS320C64XInstrFormats.td`](https://github.com/alexjordan/LLVM-TMS320C64X/blob/e17e694eb6896d9221f617bc04619bbc51c80e14/lib/Target/TMS320C64X/TMS320C64XInstrFormats.td) and
  [`TMS320C64XInstrInfo.td`](https://github.com/alexjordan/LLVM-TMS320C64X/blob/e17e694eb6896d9221f617bc04619bbc51c80e14/lib/Target/TMS320C64X/TMS320C64XInstrInfo.td) — the TableGen source those capstone tables were generated from
- `compiler/bin/dis6x` — the vendor disassembler; the tiebreaker when the three disagree

See `skills/c64xemu-validation/SKILL.md` for how to turn these into a test suite.
