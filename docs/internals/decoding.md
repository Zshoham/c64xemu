# Instruction decoding

How a fetch packet becomes execute packets, how a 32-bit or compact instruction word is turned
into a family value, and how the instruction view projects that value into the names, operands
and resource claims every consumer reads. The code is `src/decoder.rs` (family identification),
`src/view.rs` (the instruction view), `src/decode_cli.rs` (the `decode` command) and the packet
walk in `src/cpu.rs`.

## From a fetch packet to execute packets

A fetch packet is 32 bytes, aligned. Its last word — offset 28 — is a fetch-packet header when
its top nibble is `0xE`; `is_packet_header` in `src/cpu.rs` is that one test, and nothing else
distinguishes a header from an instruction ([SPRUFE8B](../reference/sprufe8b.pdf) §3.7,
[SPRU732J](../reference/spru732j.pdf) §3.9.2). Without a header the packet is eight 32-bit
instructions; with one it is seven data words, each holding either one 32-bit instruction or two
compact instructions, plus the header.

`packet_slot(address, header, endian)` in `src/cpu.rs` answers what sits at one address inside a
header-based packet, and it is the only place that answers it: the fetch path and the debugger's
listing both call it, so a compact-encoding defect cannot be visible through one path and not the
other. It returns one of three `PacketSlot` values.

| Slot | Condition | What the walk does |
|---|---|---|
| `Header` | offset 28 | Not an instruction. The fetch path steps to `packet_base + 32`; the listing prints `(fetch packet header 0x…)`. |
| `Compact` | layout bit `21 + word_index` set | Read a halfword; take the p-bit from header bit `word_index * 2 + (offset & 2) / 2`. |
| `Wide` | layout bit clear | Read a word; its p-bit is its own bit 0, even inside a header-based packet. |

The compact slot number runs 0 to 13 in address order — slot 0 is word 0's low half, slot 13 is
word 6's high half — and it is the number of the header p-bit, not a byte address. On a big-endian
device the two halfwords of a word trade byte addresses (`halfword_address: address ^ 2`) while
keeping their slot number, which is what makes the register-set bit land on the right instruction.

Execute packets are formed by the fetch loop in `Cpu::execute_cycle` (`src/cpu.rs`, the `loop`
around `self.fetch`): fetch one instruction, take the `parallel` flag the fetch returned, and keep
going while it is set. A set p-bit means the *next* instruction issues in the same cycle as this
one. §3.4 of [SPRU732J](../reference/spru732j.pdf) draws three p-bit patterns; the loop does not
treat them as three cases.

| p-bits in the packet | Execute packets | In the loop |
|---|---|---|
| all clear | one instruction each — eight, or up to fourteen in a header-based packet | `parallel` false on the first fetch, every time |
| words 0-6 set, word 7 clear | one execute packet of eight | seven iterations with `parallel` set |
| mixed | what compiled code looks like | the general path |

Two consequences of it being a forward loop over `self.pc` rather than a walk of the packet: a
branch into the middle of an execute packet issues only the instructions at and above the target,
because nothing rewinds to the packet base; and the loop stops at eight instructions
(`length == packet_storage.len()`) even when the eighth p-bit is set, so a ninth instruction
begins the next execute packet rather than being reported as an over-long packet. What happens to
the packet once it has been formed is in [Execution and timing](execution.md).

### A real packet, hand-decoded

The first fetch packet of `__TI_printfi` in `tests/firmware/bios_kernel/build/bios_kernel.out`,
as TI's own disassembler prints it:

```
$ tests/build/ti-cgt-c6000_8.5.0.LTS/bin/dis6x -q tests/firmware/bios_kernel/build/bios_kernel.out
...
00800000   10294810           CALLP.S1      __push_rts (PC+84544 = 0x00814a40),A3
00800004   07fee053           ADDK.S2       -576,B15
00800008       b407 ||        MV.L2X        A8,B5
0080000a       4646           MV.L1         A4,A10
0080000c   028088fe ||        STW.D2T2      B5,*+B15[136]
00800010   02a80266           LDW.D1T2      *+A10[0],B5
00800014       6747           MV.L2         B6,B11
00800016       d347           MV.L2X        A6,B6
00800018   030089fe           STW.D2T2      B6,*+B15[137]
0080001c   e4800020           .fphead       n, l, W, BU, nobr, nosat, 0100100b
```

The header is `0xe4800020`. Its layout field (bits 27-21, which `dis6x` prints most-significant
first as `0100100b`) has bits 23 and 26 set, so words 2 and 5 — offsets 8 and 20 — hold two
compact instructions each and the other five hold one 32-bit instruction. Its p-bit field
(bits 13-0) is `0x0020`: only slot 5, the high half of word 2, at offset 10.

| Offset | Encoding | Slot | p-bit | Next instruction |
|---|---|---|---|---|
| 0 | `10294810` | word 0, wide | own bit 0 = 0 | starts a new execute packet |
| 4 | `07fee053` | word 1, wide | own bit 0 = 1 | issues in parallel |
| 8 | `b407` | slot 4 | header bit 4 = 0 | starts a new execute packet |
| 10 | `4646` | slot 5 | header bit 5 = 1 | issues in parallel |
| 12 | `028088fe` | word 3, wide | own bit 0 = 0 | starts a new execute packet |
| 16 | `02a80266` | word 4, wide | own bit 0 = 0 | starts a new execute packet |
| 20 | `6747` | slot 10 | header bit 10 = 0 | starts a new execute packet |
| 22 | `d347` | slot 11 | header bit 11 = 0 | starts a new execute packet |
| 24 | `030089fe` | word 6, wide | own bit 0 = 0 | starts a new execute packet |
| 28 | `e4800020` | header | — | fetch resumes at offset 32 |

So the packet issues as `{0}`, `{4, 8}`, `{10, 12}`, `{16}`, `{20}`, `{22}`, `{24}` — which is
where `dis6x` puts its `||` markers, one instruction later than the p-bit that caused them.

An execute packet may cross a fetch-packet boundary, and each instruction then decodes against its
own header. In the next packet of the same image the header is `0xec702006`, whose p-bit 13 is set:
slot 13 is the `SPKERNEL` at `0x0080003a`, and the instruction that issues with it is
`ADD.L1 1,A0,A0` at `0x00800040` — the first word of the following fetch packet, past the header
word the fetch path steps over.

## The fetch-packet header

Every field the model reads out of a header, with the function that reads it. The compact-slot
p-bits and the layout field are packet shape; SAT, BR, DSZ and RS are decode state, and change
what a given 16-bit encoding means.

| Bits | Field | Read by | Effect |
|---|---|---|---|
| 31-28 | `1110` | `is_packet_header` (`src/cpu.rs`) | Marks the word as a header rather than an instruction. |
| 27-21 | layout | `packet_slot` (`src/cpu.rs`) | Bit `21 + n` set: data word `n` holds two compact instructions. |
| 20 | PROT | `packet_protected` (`src/cpu.rs`) | Four NOP cycles follow every load in the packet. |
| 19 | RS | `compact_register` (`src/decoder.rs`) | Three-bit register fields select A16-A23 / B16-B23 instead of A0-A7 / B0-B7. |
| 18-16 | DSZ | `compact_memory_shape`, `decode_compact` (`src/decoder.rs`) | Size, signedness and alignment of the compact loads and stores. |
| 15 | BR | `decode_compact` | The compact branch and `callp` encodings exist; `s_s3i` does not. |
| 14 | SAT | `decode_compact` | The compact ADD/SUB and multiply families become their saturating forms. |
| 13-0 | p-bits | `packet_slot` | One per compact slot, slot 0 = word 0 low half. |

`dis6x` prints the same fields as `.fphead n, l, W, BU, nobr, nosat, 0100100b`: PROT (`n`/`p`),
RS (`l`/`h`), the DSZ primary and secondary sizes, BR, SAT and the layout bits.

DSZ decides a compact access's shape together with the `sz` bit — bit 9 of the compact word,
`secondary` in `compact_memory_shape` — exactly as [SPRU732J](../reference/spru732j.pdf)
Table 3-10 tabulates it:

| DSZ | `sz` = 0 | `sz` = 1 |
|---|---|---|
| 000 | word | byte, unsigned |
| 001 | word | byte, signed |
| 010 | word | halfword, unsigned |
| 011 | word | halfword, signed |
| 100 | doubleword | word |
| 101 | doubleword | byte, signed |
| 110 | doubleword | word, non-aligned |
| 111 | doubleword | halfword, signed |

Alignment and the data register come out of the same reading: a doubleword form is aligned when
bit 4 is clear (`lddw`/`stdw`) and non-aligned when it is set (`ldndw`/`stndw`), and takes its
data register from bits 5-6 doubled, so it names an even register of the window; every other form
is aligned unless DSZ is 110 with `sz` set (`ldnw`/`stnw`), and takes bits 4-6. `ldst_name` in
`src/decoder.rs` maps the resulting `LdstShape` to the name.

RS remaps only the three-bit register fields that go through `compact_register`. The one-bit
destination fields of `l_l2c` and `l_lx1c` — the two-operand logical/compare family and the
compare against a 0/1 constant — are `reg_nors` forms naming A0/A1 or B0/B1, and are read
directly, never through `compact_register`, so RS cannot push a predicate write into A16.

This is why a compact instruction cannot exist without its header. `Encoding::Compact(u16, u32)`
in `src/view.rs` carries the two together, `Instruction::new` pairs them at construction, and
`decode_compact(word, header)` takes both. The same sixteen bits name different instructions under
different headers:

```
$ ./target/release/c64xemu decode 4d8a/e4008400 4d8a/e4000400
{"mnemonic":"bnop"}
{"mnemonic":"shr","operands":"a3,2,a0","unit":"s1"}

$ ./target/release/c64xemu decode 0000/e0200000 0000/e0204000 0d3c/e0200000 0d3c/e0250000
{"mnemonic":"add","operands":"a0,a0,a0","unit":"l1"}
{"mnemonic":"sadd","operands":"a0,a0,a0","unit":"l1"}
{"address":"*a6++[1]","data_path":"t1","mnemonic":"ldw","unit":"d1"}
{"address":"*a6++[1]","data_path":"t1","mnemonic":"ldndw","unit":"d1"}

$ ./target/release/c64xemu decode 4207/e0280000 4207/e0200000
{"mnemonic":"mv","operands":"b4,b18","unit":"l2"}
{"mnemonic":"mv","operands":"b4,b2","unit":"l2"}
```

`0x4d8a` is a `bnop` under the BR header the image actually uses at `0x00800174` and an
`shr .s1` without it; clearing SAT turns `sadd` back into `add`; DSZ 101 turns the `ldw` at
`0x0d3c` into `ldndw`; RS moves `b2` to `b18`.

## Family identification

`decode(word) -> Opcode` and `decode_compact(word, header) -> Option<CompactOpcode>` in
`src/decoder.rs` are family identification: one pass over the encoding, no table build, no second
opinion. `Opcode` names 131 values including `Unknown`; `CompactOpcode` names 31, all of them
valid families, because an encoding the device does not have answers `None` instead.

The 32-bit pass masks the p-bit off first (`let instruction = word & !1;`) and then tests
progressively narrower field masks, named in the comments beside them after binutils' `tic6x`
instruction formats — `nfu_nop_idle`, `s_field`, `m_mpy`, `m_compound`, `m_1_or_2_src`,
`l_1_or_2_src_noncond`. Bits 2-6 and 10-11 alone separate the L, S, D and M groups, which is what
lets `encoded_alu_unit` (`src/view.rs`) place an instruction that exists on several units without
consulting its mnemonic. Order carries meaning, and the comments in `src/decoder.rs` say where:

- `SPKERNEL`, `SPMASK` and the `SPLOOP` family are matched before the M-unit space they would
  otherwise fall into, as are `DINT`, `RINT` and `SWE`/`SWENR`.
- `BDEC` and `BPOS` sit in fixed slots inside the S-unit branch space and must be recognised
  before the general S-unit dispatch.
- `RPACK2` takes the one encoding at its op field that no predicate can produce — `creg` zero with
  the `z` bit set; everything else there is `DMV`.
- The C67x floating-point slots are holes, not instructions: `MPYI`, `MPYID`, `MPYDP`, `MPYSP`,
  `MPYSPDP`, `MPYSP2DP` and the S-unit `ABSSP` return `Opcode::Unknown`, because `dis6x`
  disassembles those words as data on a C6400+ and `cl6x -mv6400+` refuses their mnemonics.

A family value carries the identity of the instruction and nothing else. On the 32-bit side that
is the enum variant alone; the op field stays in the word, and the projections re-read it
(`Opcode::mnemonic(self, word)` consults `long_arith_mnemonic`, `mpy_mnemonic`,
`mpy_compound_mnemonic`, `mpy_unary_mnemonic`, `mpy_extended_mnemonic` and `long_shift_mnemonic`).
On the compact side the family carries the part of the reading that depended on the header, since
that is not recoverable from the sixteen bits: `Mpy { op: MpyOp, saturating: bool }`,
`LdstPointer(LdstShape)` with size, signedness, alignment and direction, `Alu3op(AluOp)` where
`AluOp` is already `Sadd` rather than `Add`, `Sploop { delayed, nested }`, `ShiftKind`, `LogicOp`,
`CmpOp`, `SetClrOp` and `MvkForm`.

### Identification is not operand reading

Operand fields — registers, constants, cross-path selectors — are read by the consumers, not by
identification. Each field has exactly one reader, called both by the executor and by the operand
rendering, so a rendering cannot disagree with what was executed:

| Reader | Reads | Called by |
|---|---|---|
| `source_fields` (`src/view.rs`) | src1 at 13-17, src2 at 18-22, dst at 23-27, cross at 12 | `execute_wide`, `wide_operands` |
| `binary_operand_shape` (`src/view.rs`) | the ALU forms' fields plus whether src1 is a constant and whether it is unsigned | `binary_operands`, `wide_operands` |
| `address_shape` (`src/view.rs`) | a 32-bit D-unit access: side, base, offset kind, add/subtract, pre/post update | `execute_wide`, `Instruction::address_operand` |
| `long_roles`, `mpy_roles` (`src/decoder.rs`) | which operand crosses, which is a constant, which are pairs | `execute_wide`, `execute_m_unit`, `wide_operands` |
| `compact_register` | the three-bit field under RS | `execute_compact`, `compact_operands`, `compact_resources` |
| `compact_constant5`, `compact_constant8`, `compact_add_constant`, `compact_stack_constant` | the split constant fields | `execute_compact`, `compact_operands` |
| `compact_memory_shape` | size, signedness, data register, alignment under DSZ | `execute_compact`, `decode_compact` |
| `spmask_mask`, `SPMASK_UNITS` (`src/view.rs`) | the eight unit bits of an SPMASK | the loop buffer, `spmask_text` |

The roles tables buy a second property: because one op-field table decides both the name and the
roles, the execution arms can assert against them. `execute_wide`'s long arm and `execute_m_unit`
carry `debug_assert!(roles.src2_pair)`, `debug_assert!(roles.dst_pair)` and
`debug_assert!(roles.src1_constant && roles.src1_constant_unsigned)` around the registers they
actually read and schedule, so a role that disagrees with the scheduling fails a debug build.

Identification runs once per instruction word, not once per execution: the executor's exact-address
decode cache stores the whole `Instruction` keyed by address, with the raw word and the compact
header validated on every hit (`DecodedEntry` and `decoded_instruction` in `src/cpu.rs`). See
[Architecture](architecture.md) for the cache's place in the fetch path.

## The instruction view

`Instruction` in `src/view.rs` is the value every consumer sees. `Instruction::new(address, word,
header)` decides the encoding (`header.is_some()` makes it compact), identifies the family once,
and computes the resource claim once; both are cached in the value, which is `Copy`. `text()`,
`operands()`, `predicate()`, `address_operand()`, `unit()`, `data_path()`, `resources()`, `issue()`
and the `serde::Serialize` implementation are projections over those two cached answers, and none
of them identifies the family again.

| Projection | Answers |
|---|---|
| `text` | `[!b0] add .s2x b0,a1,b1` — predicate, mnemonic, dotted unit with `x` and the T-path appended, then operands or the address operand |
| `unit` | `s2x`, `d1`, `m1`; `None` for the encodings that claim no functional unit |
| `data_path` | `t1` / `t2`, only for memory accesses |
| `predicate` | The `creg`/`z` register with its sense; on the compact side only the branches, the write-zero-or-one form and the nested `SPLOOPD` are predicated |
| `operands` | The operand list for the families whose fields the model reads through a shared accessor; `None` otherwise, so nothing is guessed |
| `address_operand` | `*+b15[136]`, `*a4++[1]`, `*--b6[2]` |
| `resources` | `(unit, cross path, memory path)` — the accounting the packet validator enforces |
| `issue` | `UnitIssue`: unit letter, side, cross flag, memory side, replay flag and the text |

These consumers cross the view's seam, and there is no second instruction decoder behind any of
them:

| Consumer | Where | What it uses |
|---|---|---|
| The `decode` command | `src/decode_cli.rs` | `serde_json::to_writer` over `Instruction` |
| The differential | `tests/tools/dis6x_diff.py` | the `decode` command's JSON, against `dis6x` |
| The binutils sweep | `tests/tools/decode_sweep.py` | the `decode` command's JSON, against binutils' vectors |
| The component vectors | `src/component_conformance.rs`, `run_decode` | `serde_json::to_value` over `Instruction` |
| The trace | `src/cpu.rs`, `Event::Packet` | `inst.text()` per instruction and `inst.issue(false)` per packet |
| The debugger listing | `Cpu::disassemble` → `src/debugger.rs`, `disas` | `Instruction::new(...).text()`, wrapped in `marked` |
| The debugger unit view | `src/debugger.rs` over `state.last_issue` | `UnitIssue` |
| The dashboard | `src/ui.rs` | `Machine::disassemble` for the listing, `json!(cpu.last_issue)` for the unit view |
| The SPLOOP dump | `Cpu::sploop_dump` | `fetched.inst.text()` per buffered execute packet |
| The state snapshot | `src/machine.rs`, `Snapshot::last_issue` | carries `UnitIssue` to whichever front end reads it, the GDB stub included |
| The executor | `validate_packets`, `cross_path_stall`, `fetched_unit_mask` (`src/cpu.rs`) | `inst.resources()` |

The last row is the point of the seam: the executor consults the same `resources()` the dashboard
displays, so validation and display cannot disagree about what an encoding claims.

One divergence worth knowing when reading a listing: `marked` (`src/view.rs`) prefixes `||` when
the *instruction's own* p-bit is set, so the marker names the instruction that carries the bit,
one instruction earlier than TI's tools print it. `disas 0x00800000` on the image above prints
`|| addk .s2 -576,b15` at offset 4 and no marker at offset 8, where `dis6x` prints the reverse.
The differential strips `||` before comparing, so nothing else notices.

## Unit, cross path and memory accounting

`Resources = (Option<(u8, Bank)>, Option<(Bank, u8)>, Option<Bank>)` — the functional unit and its
side, the cross path with the register number read across it, and the memory path. `wide_resources`
and `compact_resources` in `src/view.rs` derive all three from the family:

- The unit is the family's, except where the encoding decides. Families that exist on more than
  one unit (`MAX2`, `MIN2`, `PACKH2`, `PACKLH2`, `PACKHL2`, `SHLMB`, `SHRMB`, `UNPKHU4`,
  `UNPKLU4`, `SADD`) go through `encoded_alu_unit`. The B14/B15-relative long-offset loads and
  stores are `.D2` only and `ADDKPC` is `.S2` only, because their `s` bit is not a unit selector.
  The 32-bit immediate `BNOP` with its unit bit clear claims no unit at all — it is the C64x+
  macro form the compiler pairs with an S-unit instruction in the same execute packet.
- The cross path is bit 12, but only for the families on an explicit whitelist, because bit 12 is
  part of the encoding elsewhere — `MVK`'s 16-bit constant and the constant bit-field forms would
  otherwise read as cross-path reads, and the D-unit address arithmetic is deliberately absent
  from the list because `dis6x` prints those forms without an `X`. The crossed operand is src2
  (bits 18-22) for almost everything; the 40-bit long forms cross src1 (bits 13-17), and a `.D1`
  long `ADDA` crosses to read its B14/B15 base.
- The memory path is the `y` bit for the memory families, T1 or T2 (`encoded_data_bank`).
- `NOP`, `IDLE`, `DINT`, `RINT`, `SWE`/`SWENR`, the `SPLOOP` family, `SPKERNEL`, `SPMASK` and the
  compact branches claim no unit; `issue()` falls back to the side bit — bit 1 of a 32-bit word,
  bit 0 of a compact one — so the view never prints a bogus `.?1`.

`validate_packets` in `src/cpu.rs` is packet validity, evaluated over the freshly fetched execute
packet chained with whatever the loop buffer is replaying this cycle. It keeps a fixed array of
eight unit claims — a ninth necessarily conflicts with one of the first eight — one cross-path
claim per side and one memory-path flag per side, and reports the first collision with the IERR
bit the CPU raises:

| Rule | Message | IERR |
|---|---|---|
| Two instructions claiming the same unit and side | `functional unit conflict on .L1 at 0x00000000 and 0x00000004` | bit 4, RCX |
| One side's cross path reading two different registers | `cross-path conflict on 1X at 0x00001004: reads B5 after B4` | bit 4, RCX |
| Two memory operations on one T path | `multiple memory operations on T1` | bit 4, RCX |

The cross-path rule allows the same register to be read twice on one side and rejects a different
one, which is how a packet's two readers of one crossed value pass. Validity is memoized per
packet address once every instruction in it came from the decode cache
(`packet_validation_cached`), and skipped entirely for a cycle that only replays. The related
cross-path *stall* — reading over the cross path one cycle after a non-load write — reads the same
`resources()` and belongs to [Execution and timing](execution.md).

## What an unclassified encoding does

An encoding that names no C64x+ instruction leaves `Instruction::decoded` as `None`. That single
state is what every consumer sees, and each has one defined answer:

| Consumer | Answer |
|---|---|
| `text` | `.word 0x56454314` for a 32-bit word, `.half 0x0008` for a compact one — the raw bits, never a guess |
| `serialize` | An empty JSON object, `{}` |
| `resources` | `Err(("invalid opcode at 0x00000000: 0x56454314", 1 << 3))`, or `invalid compact opcode at …: 0x0008`; IERR bit 3 is the opcode exception |
| `issue` | Unit `?`, cross and memory paths absent, side from the side bit |
| The executor | `fault_invalid_opcode` / `fault_invalid_compact` with the same message and IERR bit 3 |
| The differential | A disagreement, unless `dis6x` also printed the word as data |
| The binutils sweep | Expected, and required, for the C67x floating-point mnemonics |

```
$ ./target/release/c64xemu decode 56454314 0008/e0200000
{}
{}
```

Naming and resource accounting have to agree about which encodings are valid, or a word would
render as an instruction and then fault, or vice versa. On the compact side that agreement is
swept exhaustively — every one of the 65536 words against every header bit identification reads,
4194304 combinations — by `compact_naming_and_resources_agree_over_the_whole_word_space` in
`src/cpu.rs`, with `compact_family_counts_are_stable` freezing the per-family classification
counts beside it (407040 of those combinations name no instruction):

```
$ cargo nextest run -E 'test(compact_naming_and_resources_agree_over_the_whole_word_space) or test(compact_family_counts_are_stable) or test(same_functional_unit_in_parallel_faults)'
        PASS [   0.006s] (1/3) c64xemu::bin/c64xemu cpu::tests::same_functional_unit_in_parallel_faults
        PASS [   0.627s] (2/3) c64xemu::bin/c64xemu cpu::tests::compact_naming_and_resources_agree_over_the_whole_word_space
        PASS [   0.909s] (3/3) c64xemu::bin/c64xemu cpu::tests::compact_family_counts_are_stable
     Summary [   0.911s] 3 tests run: 3 passed, 675 skipped
```

## The decode command's JSON

`c64xemu decode` takes one or more `WORD[/HEADER]` arguments in hex, with or without a `0x`
prefix, and writes one JSON object per line in argument order. A `/HEADER` makes the word compact
and is rejected above 16 bits (`parse_instruction`, `src/decode_cli.rs`). It loads no image and
walks no packet: it names the word it is given, so the header `0xe4800020` handed over as a bare
argument is named as the 32-bit encoding those bits are — `{"mnemonic":"bpos","unit":"s1"}` — and
not as a header.

```
$ ./target/release/c64xemu decode --help
Decode instruction words without loading an image

Usage: c64xemu decode <WORD[/HEADER]>...

Arguments:
  <WORD[/HEADER]>...  Instruction word, optionally followed by its compact fetch-packet header

Options:
  -h, --help     Print help
  -V, --version  Print version

$ ./target/release/c64xemu decode 07fee053 6283e000 028088fe b407/e4800020 4646/e4800020 1c66/ec702006
{"mnemonic":"addk","operands":"-576,b15","unit":"s2"}
{"mnemonic":"sploopw","operands":"6","predicate":"b2"}
{"address":"*+b15[136]","data_path":"t2","mnemonic":"stw","unit":"d2"}
{"mnemonic":"mv","operands":"a8,b5","unit":"l2x"}
{"mnemonic":"mv","operands":"a4,a10","unit":"l1"}
{"mnemonic":"spkernel"}
```

The keys are `address`, `data_path`, `mnemonic`, `operands`, `predicate` and `unit`, emitted in
that sorted order, and each is present only when the family carries it: `addk` claims a unit but
no data path, `stw` renders an address operand instead of an operand list, `sploopw` carries a
predicate but claims no unit, and `spkernel` carries only its name. `mnemonic` is the one key
every named instruction has, so both differentials read its absence as the
not-an-instruction answer.

Both consume exactly this output, in batches of 512 words per process (`decode` in either tool).
`tests/tools/dis6x_diff.py` disassembles the built
firmware images with `dis6x`, passes each compact word with the `.fphead` word of its own fetch
packet, and compares the mnemonic, unit, predicate, data path, address operand and operand list;
`tests/tools/decode_sweep.py` runs the binutils `tic6x` test vectors through the same command and
requires the C67x mnemonics to decode to nothing:

```
$ python3 tests/tools/dis6x_diff.py
bios_kernel.out: 23263 instructions (6550 compact), 23263 matched
ndk_tcp.out: 71238 instructions (32462 compact), 71238 matched
total 94501, disagreeing 0, operand lists compared 57558

$ python3 tests/tools/decode_sweep.py
words 2195  matched 2032  floating-point skipped 163
```

Both tools list their aliases rather than hiding them, because TI's assembler emits one encoding
for several mnemonics — `MV` is an `OR` with zero, `NOT` an `XOR` with -1, `NEG` a `SUB` from
zero, `ZERO` a subtraction of a register from itself, `SWAP2` a `PACKLH2` with one source used
twice — and a subtraction of a five-bit constant is an addition of its negation. `dis6x` prints
the pseudo-operation its assembler accepted, the instruction view prints the operation the bits
encode, and both are right. Operand lists are compared only where the mnemonics agree, for the
same reason. What holds the rest of the decoding claims down, and how these two differentials fit
the other harnesses, is in [Running the tests](../guide/testing.md); the option tables for `decode` and
everything else are in [Command line](../guide/command-line.md).
