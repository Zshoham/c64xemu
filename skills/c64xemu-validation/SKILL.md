---
name: c64xemu-validation
description: "Validate a C64x+ emulator with dis6x, binutils vectors, DSPLIB differential execution, SPLOOP cycle-flow tests, and a GDB stub."
---

# Validating the emulator

*How to prove the C64x+ emulator is right — differential decoder testing against TI's dis6x and binutils' 2474 ready-made test vectors, the DSPLIB optimized-vs-reference execution corpus, the local toolchain invocation that actually works, SPLOOP cycle-flow tests, and a GDB stub register layout.*

An emulator for a 235-mnemonic VLIW ISA with exposed pipeline latency cannot be validated by
inspection. Fortunately this target has unusually good oracles available locally.

## The local toolchain works — this is the primary oracle

TI CGT 8.3.15 is in `compiler/bin/`. Verified working invocations:

```sh
export PATH=\$PWD/tests/build/ti-cgt-c6000_8.3.15/bin:$PATH
INC=\$PWD/tests/build/ti-cgt-c6000_8.3.15/include
LIB=\$PWD/tests/build/ti-cgt-c6000_8.3.15/lib

# compile only
cl6x -mv6400+ --abi=eabi -O2 -I$INC -c t.c

# disassemble: address, raw hex word, mnemonic — the decoder oracle
dis6x t.obj

# full link to a runnable ELF (needs a linker command file with a MEMORY map)
cl6x -mv6400+ --abi=eabi -O2 -I$INC hello.c -z lnk.cmd -o hello.out -i$LIB -llibc.a \
     --heap_size=0x800 --stack_size=0x800

# object/symbol inspection
ofd6x --obj_display=none,header hello.out
nm6x hello.out
```

Note `-I` is required — `cl6x` does not find its own headers by default. `-i` adds a library
search path. A minimal `lnk.cmd` that places everything in L2 at `0x00800000` is enough to
link; see `skills/c6000-elf-abi-and-boot/SKILL.md`.

Confirmed: the linked image is `ELF32 / little-endian / EXEC / EM_TI_C6000`, and exports the
symbols the emulator needs — `_c_int00`, `C$$IO$$`, `C$$EXIT`, `__CIOBUF_`.

### What `dis6x` gives you

```
0080005c   e2000000           .fphead       n, l, W, BU, nobr, nosat, 0010000b
00800060   0000a35a           MVK.L2        0,B0
00800074       128d           LDB.D2T2      *B5[0],B0
00800084   30000911    [!B0]  B.S1          $C$L400 (PC+72 = 0x008000c8)
00800088   00800fdb ||        MV.L2         B0,B1
```

Address, raw word, predicate, mnemonic with unit and cross-path, operands, `||` for
parallelism, and **it decodes fetch-packet headers and 16-bit compact instructions** (note
the 2-byte address stride at `0x00800074`). That is the complete decoder contract in one
tool.

## Test 1 — differential decoder testing

Two complementary strategies. Do both.

### 1a. Replay real code

Compile a broad corpus (see DSPLIB below), run `dis6x` over every object, and compare your
decoder's output line by line. Normalise formatting once, then diff. Any mismatch is either
a decoder bug or a mnemonic-spelling difference — resolve spelling once and it stays fixed.

This exercises exactly the encodings a compiler actually emits, which is most of what your
emulator will meet, and it covers compact instructions and headers for free.

### 1b. Random encoding fuzzing

Generate random 32-bit words, feed them to both your decoder and `dis6x`, and compare
including the "invalid instruction" verdict. This finds the encodings a compiler never emits
— which is precisely where hand-written assembly and hostile firmware live. Bias the
generator toward valid opcode-field values from the binutils tables so the hit rate is
useful, and fuzz header-based fetch packets separately (vary Layout/RS/DSZ/BR/SAT/PROT and
check every combination changes decoding as the tables say it should).

### 1c. binutils' ready-made vectors — 2474 of them, free

`tests/build/binutils-tic6x-vectors/` has 231 files. The `.d` files carry
literal `hexword → mnemonic operands` pairs:

```
[0-9a-f]+[02468ace] <[^>]*> 8c05[ \t]+stw \.D2T1 a0,\*\+b15\(0\)
[0-9a-f]+[02468ace] <[^>]*> cc35[ \t]+stw \.D2T1 a3,\*\+b15\(8\)
[0-9a-f]+[02468ace] <[^>]*> efe00000[ \t]+<fetch packet header 0xefe00000>
```

Extract with a regex and you have a decoder regression suite in an afternoon. Notable groups:

| Files | Coverage |
|---|---|
| `insns16-d-unit`, `insns16-l-unit`, `insns16-m-unit`, `insns16-s-unit`, `insns16-lsd-unit` | every 16-bit compact format, per unit |
| `insns16-doff4`, `insns16-dind`, `insns16-dinc`, `insns16-ddec` | compact addressing modes |
| `insns16-parallel`, `insns-parallel-le`, `insns-parallel-be` | p-bit and header p-bit handling, both endiannesses |
| `insns-predicate` | every creg/z combination |
| `insns-atomic` | LL/SL/CMTL |
| `insns-bad`, `parallel-bad`, `predicate-bad`, `resource-func-unit`, `sploop-bad-1..7` | **negative tests** — encodings and packets that must be rejected |
| `attr-arch-opts-c64x+`, `arch-invalid` | ISA gating |
| `align`, `endian`, `pcr-relocs`, `data-reloc`, `got-reloc`, `unwind` | assembler/linker behaviour |

The `sploop-bad-*` and `resource-func-unit` files are especially valuable: they encode the
resource-constraint rules as executable tests, which is otherwise tedious to derive.

Mind the scope: several files target `c674x`, which is a superset. Filter on the `#as:`
line's `-march=` and keep `c62x`/`c64x`/`c64x+`.

### 1d. Third decoder as a tiebreaker

[capstone 4.0.2 TMS320C64x](https://github.com/capstone-engine/capstone/tree/4.0.2/arch/TMS320C64x) is an independent LLVM-TableGen-derived decoder.
When binutils and your implementation disagree, check capstone; when all three disagree,
`dis6x` wins — it is the vendor's.

## Test 2 — execution correctness with DSPLIB

`tests/build/dsplib-c64plus/` (TI DSPLIB 3.4.0.0 for C64x+, 46 kernels) is a
purpose-built differential corpus. Each kernel ships **three** files:

| File | What |
|---|---|
| `DSP_xxx.c` | the optimized implementation, dense with C64x+ intrinsics |
| `DSP_xxx_cn.c` | a **natural-C reference** implementation, BSD-licensed |
| `DSP_xxx_d.c` | a driver with test data |

The method: compile the optimized and reference versions **both for C64x+**, run both **in
the emulator**, and compare outputs. Any difference is an emulator bug in one of the SIMD /
packed / saturating instructions — because on real silicon they agree.

This is strong coverage precisely where hand-written semantics are most error-prone:
`DOTP2`, `MPY2`, `SUB4`, `AVGU4`, `SADD2`, `PACK*`, `SHFL`, `DEAL`, `BITC4`, `GMPY4`, the
Q-format kernels, and the 16×16/32×32 multiply variants. Kernels worth starting with:
`DSP_fir_r4`, `DSP_dotprod`, `DSP_mat_mul`, `DSP_fft16x16`, `DSP_maxval`, `DSP_q15tofl`,
`DSP_blk_eswap16/32/64`, `DSP_bexp`, `DSP_recip16`.

It also exercises SPLOOP heavily, since the kernels are written to software-pipeline.

The `*_cn.c` references are BSD-licensed (the optimized `*.c` files carry TI's proprietary
notice — fine to run, but keep them out of anything you redistribute).

## Test 3 — SPLOOP cycle-flow tests

SPRU732J gives two worked examples with **complete cycle-by-cycle instruction flow tables**:

- Examples 7-4 / 7-5 with Table 7-1 (`SPLOOP`/`SPLOOPD`, ii=1, dynlen=7, 8 iterations,
  14 cycles)
- Example 7-7 with Table 7-2 (`SPLOOPW`, showing the abrupt no-epilog exit)

Assemble each, run it, log which instruction executes on which cycle, and diff against the
table. This is the only practical way to be confident about the loop buffer state machine
before you meet real compiler output. Build these tests **before** implementing SPLOOP, not
after.

Also run `insns-c674x-sploop.d` from the gas testsuite for encoding coverage of the SPLOOP
family, and `sploop-bad-1..7` for the rejection cases.

## Test 4 — full-program tests via CIO

Once `C$$IO$$` semihosting works (see `skills/c6000-elf-abi-and-boot/SKILL.md`), any C program becomes a
test. Compile the same source with the host compiler and with `cl6x`, run both, diff stdout.
Good candidates:

- integer/bit-manipulation torture tests (checks the .L/.S/.M semantics broadly)
- the DSPLIB drivers (`*_d.c`) — they already print results
- programs from `docs/reference/chassaing_dsp_applications_c6x_dsk.pdf`

Vary `-O0/-O2/-O3` and `--opt_for_speed`: higher optimization emits more SPLOOP, more
compact instructions, more parallelism, and deeper delay-slot packing. **`-O0` and `-O3`
builds of the same source exercise substantially different parts of your emulator**, so run
both.

Also build **both endiannesses** (`--endian=big`) at least once; endian bugs in the loader
and the packed-data instructions hide well.

## Test 5 — resource-conflict detection as a test

Implement the SPRU732J §3.7 constraint checks and make violations raise the correct `IERR`
bit (see `skills/c64xplus-interrupts-and-exceptions/SKILL.md`). Then run the whole corpus with checking on:
**a correct emulator running correct code should never trip a check**. Any trip is either a
decoder bug (you assigned the wrong functional unit) or a constraint implemented too
strictly. This turns the whole corpus into a second, independent test of the decoder.

TI's simulator does the same thing and reports, e.g.:

```
Register(s) B7 in Multiple Write Conflict in E1 phase.
```

SPRU600I §2.3.2 lists exactly which classes it detects and §2.3.3 what it does not
(more than four simultaneous reads of one register — so implement that one and you are
ahead of the vendor).

## Test 6 — Ethernet path

- **pcap replay** as the first backend: deterministic frames in, deterministic frames out,
  diffable. Build this before TAP.
- Drive it with TI's NDK (`docs/reference/spru523k.pdf`) or a hand-written
  minimal driver following the Linux `davinci_emac.c` flow.
- Specific behaviours to test, because firmware depends on them and they are easy to get
  wrong: the **EOQ race** on descriptor-list append; the **CP acknowledge** comparison
  (write a stale pointer and confirm the interrupt stays asserted); the **EWCTL.INTEN**
  disable/enable re-edge; and a **DDR2-resident buffer without a cache writeback**, which
  must produce corrupt transmitted data exactly as hardware would.

## A GDB stub is worth building early

[GDB tic6x support](https://sourceware.org/git/?p=binutils-gdb.git;a=tree;f=gdb;hb=refs/tags/binutils-2_47) has GDB's target support for this architecture:
[`gdb/tic6x-tdep.c`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/tic6x-tdep.c;hb=refs/tags/binutils-2_47) plus the XML target descriptions
[`tic6x-core.xml`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/features/tic6x-core.xml;hb=refs/tags/binutils-2_47), [`tic6x-gp.xml`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/features/tic6x-gp.xml;hb=refs/tags/binutils-2_47) and
[`tic6x-c6xp.xml`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/features/tic6x-c6xp.xml;hb=refs/tags/binutils-2_47). These define the exact register numbering and layout a `gdbserver` must
present. Implement the remote-serial-protocol subset (`g`, `G`, `m`, `M`, `c`, `s`, `Z0/z0`)
against that layout and you can drive the emulator from a real `tic6x-*-gdb`, single-step
through firmware, and inspect state — which is worth more than any amount of printf
debugging in the VLIW packets.

[Linux v5.10 arch/c6x](https://github.com/torvalds/linux/tree/v5.10/arch/c6x) shows how the register set is presented by a real OS, useful for
cross-checking the ordering.

## Suggested CI shape

```
make test-decode     # binutils vectors + dis6x differential over a compiled corpus
make test-fuzz       # N random words vs dis6x, seeded, reproducible
make test-sploop     # SPRU732J Tables 7-1 and 7-2 cycle-flow comparison
make test-dsplib     # optimized vs *_cn reference, all 46 kernels, LE and BE
make test-cio        # host-compiled vs emulated stdout for the C test programs
make test-emac       # pcap replay round-trips, EOQ/CP/EWCTL edge cases
```

Keep `test-decode` and `test-sploop` fast enough to run on every commit; the rest nightly.

## Files that expand on this

**Documents**

- `docs/reference/spru600i.pdf` (SPRU600I, C6000 Instruction Set Simulator Technical
  Reference) — the closest thing to a specification for this project
  - §1.3-1.4, pp. 10-13 — the tiers TI ships (CPU cycle-accurate / device functional /
    device cycle-accurate) and exactly what each models; useful for deciding scope
  - §2.2, pp. 22-23 — reserved-memory-access detection, worth copying
  - §2.3, pp. 24-25 — CPU resource conflict detection: the classes detected, the classes
    *not* detected, and the error-report format
  - §2.1, pp. 16-22 — pin-connect and port-connect, TI's mechanism for feeding external
    stimulus into a simulation
- `docs/reference/spru732j.pdf` §7.6, pp. 586-590 — Tables 7-1 and 7-2, the SPLOOP cycle-flow ground
  truth to test the loop buffer against
- `docs/reference/sprui03f.pdf` — assembler and linker invocation, linker command file syntax, and the
  directives (`.nocmp`, `.align`) the test vectors rely on
- `docs/reference/sprui04g.pdf` — compiler options: `-mv6400+`, `--abi=eabi`, `--endian=big`, and the
  optimization levels that change how much SPLOOP and how many compact instructions appear
- `docs/reference/c6xsim_cycle_accurate_simulator_c62x.pdf` — how an academic C6x simulator
  was structured and validated
- `docs/reference/chassaing_dsp_applications_c6x_dsk.pdf` — a large supply of small,
  self-contained C6x programs usable as test inputs

**Tools and corpora**

- `compiler/bin/` — `cl6x` (compiler/linker driver), `asm6x`, `dis6x` (the decoder oracle),
  `ofd6x` (object file display), `nm6x`, `hex6x`, `strip6x`, `ar6x`
- `compiler/include/c6x.h` — the intrinsic → instruction mapping, a cross-check on semantics
- `tests/build/binutils-tic6x-vectors/` — 231 files, ~2474 extractable
  `hexword → mnemonic` vectors. Positive: `insns16-*`, `insns-parallel-*`, `insns-predicate`,
  `insns-atomic`. Negative: `insns-bad`, `parallel-bad`, `predicate-bad`,
  `resource-func-unit`, `sploop-bad-1..7`. ISA gating: `attr-arch-opts-*`, `arch-invalid`.
- `tests/build/dsplib-c64plus/packages/ti/dsplib/src/` — 46 kernels, each with an
  optimized `DSP_xxx.c`, a natural-C reference `DSP_xxx_cn.c`, and a driver `DSP_xxx_d.c`
- [capstone 4.0.2 TMS320C64x](https://github.com/capstone-engine/capstone/tree/4.0.2/arch/TMS320C64x) — independent decoder for cross-checking
- [`gdb/tic6x-tdep.c`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/tic6x-tdep.c;hb=refs/tags/binutils-2_47), [`tic6x-core.xml`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/features/tic6x-core.xml;hb=refs/tags/binutils-2_47),
  [`tic6x-gp.xml`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/features/tic6x-gp.xml;hb=refs/tags/binutils-2_47), [`tic6x-c6xp.xml`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/features/tic6x-c6xp.xml;hb=refs/tags/binutils-2_47) — the register numbering and target descriptions a gdbserver stub must
  present
- [Linux v5.10 arch/c6x](https://github.com/torvalds/linux/tree/v5.10/arch/c6x) — a complete OS that runs on this core, for end-to-end testing
  once interrupts, caches and a timer work
