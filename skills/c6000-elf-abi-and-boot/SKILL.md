---
name: c6000-elf-abi-and-boot
description: "Load and run C6000 EABI ELF binaries, relocations, calling conventions, C6455 boot modes, reset state, and CIO host I/O."
---

# Loading and running C6000 code

*Getting code into the emulator and running it — the C6000 EABI ELF format and relocations, the register calling convention, C6455 boot modes and reset state, and the CIO host-I/O protocol that makes printf work in compiled test programs.*

## ELF / EABI basics

`e_machine = EM_TI_C6000` (140). The C6000 EABI (SPRAB89B) replaced the older COFF ABI;
the toolchain in `compiler/` builds EABI ELF with `--abi=eabi` (and legacy COFF otherwise —
`dis6x` output shows which you have, and DSPLIB ships both `_ELF` and `_COFF` variants).

C6000-specific ELF constructs (from [`include/elf/tic6x.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/elf/tic6x.h;hb=refs/tags/binutils-2_44)):

- `SHT_C6000_ATTRIBUTES` — build attributes (ISA variant, endianness, ABI flavour). **Read
  this in the loader** and refuse to run a C67x-only or wrong-endian image.
- `SHT_C6000_UNWIND` and `PT_C6000_PHATTR` — exception unwinding, program header attributes.
- `SHT_C6000_PREEMPTMAP`, `DT_C6000_DSBT_*` — dynamic linking with DSBT (the DSP static base
  table used for shared libraries without a real MMU). Only relevant if you load shared
  objects; a bare-metal image will not use them.

### Relocations

34 types. Grouped by what they patch:

| Family | Examples | Meaning |
|---|---|---|
| Absolute data | `ABS8`, `ABS16`, `ABS32` | plain data words |
| Absolute into instruction fields | `ABS_L16`, `ABS_H16`, `ABS_S16` | the `MVKL`/`MVKH` pair and 16-bit signed fields |
| PC-relative | `PCR_S7`, `PCR_S10`, `PCR_S12`, `PCR_S21`, `PCR_L16`, `PCR_H16` | branch displacements of various widths |
| DP-relative (small data) | `SBR_U15_B/H/W`, `SBR_S16`, `SBR_L16_B/H/W`, `SBR_H16_B/H/W` | B14-relative addressing; `_B/_H/_W` encode the scaling |
| GOT / dynamic | `SBR_GOT_U15_W`, `SBR_GOT_L16_W`, `SBR_GOT_H16_W`, `JUMP_SLOT`, `COPY`, `DSBT_INDEX` | dynamic linking |
| Pseudo | `NONE`, `ALIGN`, `FPHEAD`, `NOCMP`, `PREL31`, `EHTYPE` | alignment, fetch-packet headers, compact-instruction suppression, unwind |

For a static bare-metal loader you generally need none of these — sections are already
relocated. Implement them only if you want to load relocatable objects directly.
`bfd/elf32-tic6x.c` has all 34 fully implemented if you do.

Note `R_C6000_FPHEAD` and `R_C6000_NOCMP`: these exist because the linker may have to
**rewrite fetch-packet headers** when it moves code, and `.nocmp` regions (like the interrupt
service table) must not get compact instructions. If you ever write a linker-like tool,
these are the traps.

## Calling convention (SPRAB89B ch.3)

| Register | Alias | Callee-saved | Role |
|---|---|---|---|
| A0, A1 | | no | scratch (also predicates) |
| A2 | | no | static chain for nested functions (also predicate) |
| A3 | | no | address for a returned-by-reference struct |
| A4:A5 | | no | **1st argument; return value** (A4 = LSW, A5 = MSW) |
| A6:A7 | | no | 3rd argument |
| A8:A9 | | no | 5th argument |
| A10:A11 | | **yes** | 7th argument |
| A12:A13 | | **yes** | 9th argument |
| A14 | | **yes** | |
| A15 | FP | **yes** | frame pointer |
| A16-A31 | | no | scratch |
| B0, B1 | | no | scratch (also predicates); lazy-binder reloc offset |
| B2 | | no | scratch (also predicate) |
| B3 | | no | **return address** |
| B4:B5 | | no | 2nd argument |
| B6:B7 | | no | 4th argument |
| B8:B9 | | no | 6th argument |
| B10:B11 | | **yes** | 8th argument |
| B12:B13 | | **yes** | 10th argument |
| B14 | **DP** | **yes** | data page pointer |
| B15 | **SP** | **yes** | stack pointer |
| B16-B29 | | no | scratch |
| B30, B31 | | no! | trampoline scratch — clobbered by trampolines, so never live across one |

Argument order: **A4, B4, A6, B6, A8, B8, A10, B10, A12, B12**, then the stack starting at
`SP+4` (each argument aligned for its type — so an 8-byte-aligned first stack argument
actually lands at `SP+8`).

- 33-64 bit values use register **pairs** (even = LSW).
- `double complex` uses register **quads**: `A7:A6:A5:A4`, `B7:B6:B5:B4`, `A11:A10:A9:A8`,
  `B11:B10:B9:B8`. Registers skipped by a quad remain available for later arguments.
- Structs/unions ≤ 64 bits are passed by value; larger ones by reference (with A3 carrying
  the return-struct address).
- **SP must stay 8-byte aligned.** The stack grows down.
- Variadic functions pass the last declared argument and everything after it on the stack.
- `this` is passed in A4.

A call is: put the return address in **B3**, then branch. The return address must be in B3
**by the time the call instruction reaches E1**. `CALLP` does both in one instruction.

Knowing this convention is what lets the emulator do useful things without a debugger:
break on a symbol, print `A4`/`B4` as arguments, break on `B3` return and print `A4`.

## C6455 reset and boot (SPRUEC6G)

`BOOTMODE[3:0]` pins select the mode. For emulation the important one is:

- **`BOOTMODE = 0000` — "no boot"**: the ROM bootloader does not run and the DSP
  **begins execution at `0x0080_0000`**, the base of internal L2. This is the mode to model
  by default: load the ELF, set PC to the entry point (or to `0x00800000`), go.

Other modes: host boot (HPI/PCI — host loads memory then interrupts the CPU, which then runs
from the base of L2), EMIFA boot, I2C EEPROM boot, I2C slave boot, SRIO boot, and — on some
family members — **EMAC boot** (the DSP pulls its image over Ethernet). If the project ever
wants to emulate the full power-on path, EMAC boot is directly relevant.

### State after the bootloader (Table 1, SPRUEC6G)

- **All CPU registers except PC must be considered random.** Do not let firmware depend on
  a zeroed register file, and consider having the emulator fill registers with a poison
  pattern in a debug mode to catch such assumptions.
- Interrupts are left **enabled** (`GIE = 1`).
- `ISTP` points into ROM at `0x0010_3C00` during boot, then is restored to its default.
- L2 from `0x009F_F080`–`0x009F_FFFF` is reserved during boot (reclaimable afterwards).
- **L2 cache is always disabled** on bootloader exit.
- PLL1 is in bypass except for SRIO/PCI boot (×15).
- Peripherals enabled by the boot mode stay enabled.

These are two different states and it matters which you model:

- **True hardware reset** (SPRU732J ch.5, SPRU871K): `GIE = 0`, `NMIE = 0`, `GEE = 0`,
  `TSR.CXM` = Supervisor, TSC stopped, caches disabled, **all MAR PC bits 0 so external
  memory is non-cacheable**, ISTP at the device default.
- **After the ROM bootloader** (SPRUEC6G Table 1): interrupts *enabled* with `GIE = 1`,
  peripherals the boot mode needed left powered up, PLL possibly reconfigured, L2 cache
  disabled, and the register file arbitrary.

For a "load an ELF and run" emulator, model the **hardware reset** state and skip the
bootloader — that is exactly what `BOOTMODE = 0000` does on real silicon. So: PC = the ELF
entry point (or `0x0080_0000`), `TSR.CXM` = Supervisor, `GIE = 0`, `NMIE = 0`, caches
disabled, MARs zeroed, `ISTP` = device default, and let `_c_int00` establish SP and DP.

## The C runtime entry path

The TI RTS `_c_int00` (`compiler/lib/src/boot.c`) does, in order:

1. Set up SP from the linker-defined `__TI_STACK_END`/stack size.
2. Set DP (B14) from `__TI_STATIC_BASE`.
3. Run C auto-initialisation (`autoinit.c`) — copy/decompress `.cinit` records into `.bss`
   and `.data`, run `.init_array` constructors.
4. Call `main`, then `exit`.

If a test program hangs before reaching `main`, the usual causes are: the linker command file
placed the stack somewhere the emulator does not back with RAM, or `.cinit` processing is
walking a table the loader did not copy. Both are loader bugs, not CPU bugs — check them
first.

## CIO — host I/O, and why you want it early

The TI runtime implements `printf`, `fopen`, `time` and friends by **trapping to the host
debugger**. Implementing this in the emulator gives compiled test programs working
`printf` and file I/O with no target-side driver — which makes the entire DSPLIB validation
corpus runnable. Do it early.

The protocol is fully specified by `compiler/lib/src/trgcio.h` and `trgmsg.c`:

1. Software fills a global buffer `_CIOBUF_` (symbol `__CIOBUF_`, in section `.cio`, or
   `.bss:_CIOBUF_` under EABI; size `BUFSIZ + 32`):

   ```
   offset 0 : uint32  length        (target-endian int)
   offset 4 : uint8   command
   offset 5 : uint8   parm[8]       (byte stream, little-endian packed)
   offset 13: uint8   data[length]  (byte stream)
   ```

   `LOADSHORT`/`LOAD32` in `trgcio.h` pack multi-byte parameters **little-endian regardless
   of target endianness**, deliberately, so the host does not have to care.

2. Software issues `__TI_data_synch_WBINV(&_CIOBUF_, CIOBUFSIZ)` — a cache writeback-invalidate
   so the host sees the buffer. **Your cache model must honour this**, or CIO will read stale
   bytes.

3. Software executes a `NOP` at the global symbol **`C$$IO$$`**. That is the breakpoint the
   host watches for.

4. The host reads the buffer, performs the operation, writes the result back into `_CIOBUF_`
   in the same format, and resumes.

Commands (`trgcio.h`):

| Value | Command |
|---|---|
| 0xF0 | `_DTOPEN` |
| 0xF1 | `_DTCLOSE` |
| 0xF2 | `_DTREAD` |
| 0xF3 | `_DTWRITE` |
| 0xF4 | `_DTLSEEK` |
| 0xF5 | `_DTUNLINK` |
| 0xF6 | `_DTGETENV` |
| 0xF7 | `_DTRENAME` |
| 0xF8 | `_DTGETTIME` |
| 0xF9 | `_DTGETCLK` |
| 0xFA | `_DTGETTIME64` |
| 0xFF | `_DTSYNC` |

Implementation in the emulator:

```
at load time:  cio_bp   = symbol "C$$IO$$"
               cio_buf  = symbol "__CIOBUF_" (fall back to "_CIOBUF_")
each step:     if pc == cio_bp: handle_cio(cio_buf); continue
```

Map `_DTWRITE` on fd 1/2 to the host's stdout/stderr and you have `printf`. The remaining
commands are a thin shim over host `open`/`read`/`write`/`lseek`/`unlink`/`rename`/`time`.

Note the multi-threaded caveat in `trgmsg.c`: `_CIOBUF_` is a shared resource guarded by
`__TI_LOCK_HOST_CIO`. A single-core emulator does not care, but do not cache the buffer
contents across the breakpoint.

## Program exit

The RTS `exit` path ends in a known symbol (`C$$EXIT` / `abort`). Watch for it and stop the
emulator with the exit status from `A4` — otherwise test programs run off the end into
whatever follows.

## Files that expand on this

**Documents**

- `docs/reference/sprab89b.pdf` (SPRAB89B) — the C6000 EABI
  - §3, pp. 25-30 — Table 3-1 register conventions, argument passing, structure passing,
    the stack frame and trampolines
  - §11 and following — the relocation types, section types, and build attributes
  - §15.6 — dynamic linking and the lazy binder, if you ever load shared objects
- `docs/reference/sprui03f.pdf` (SPRUI03F) — assembler and linker: linker command files, `MEMORY` and
  `SECTIONS`, section placement, `.nocmp`, and the object file format the loader parses
- `docs/reference/sprui04g.pdf` (SPRUI04G) — the compiler: run-time environment, `--abi=eabi`, the
  intrinsics list, and how the C runtime is initialised
- `compiler/doc/SPRAB90.pdf` (SPRAB90) — COFF→EABI migration, needed if you meet COFF images
  (DSPLIB ships both `_COFF` and `_ELF` object variants)
- `docs/reference/spruec6g.pdf` (SPRUEC6G) — the bootloader
  - §2.1, p. 20 — Table 1, the state the bootloader leaves the DSP in, including
    "all CPU registers except PC must be considered random"
  - §2.3, pp. 20-28 — every C6455 boot mode; §2.3.1 is the "no boot" mode an emulator models
  - §3.3.6 and §4.3.5 — EMAC boot, if you ever emulate booting over Ethernet

**Source**

- `compiler/lib/src/trgcio.h` — the CIO command constants and the pack/unpack macros; this
  file *is* the host-I/O protocol specification
- `compiler/lib/src/trgmsg.c` — `__TI_writemsg()` / `__TI_readmsg()`: the exact `_CIOBUF_`
  layout, the cache writeback before the trap, and the `C$$IO$$` breakpoint
- `compiler/lib/src/host*.c` (`hostopen.c`, `hostread.c`, `hostwrite.c`, `hostlseek.c`,
  `hostclose.c`, `hostunlink.c`, `hostrename.c`, `hosttime.c`, `hostclock.c`,
  `hostgetenv.c`) — one file per CIO command, showing the parameter packing for each
- `compiler/lib/src/boot.c` and `autoinit.c` — `_c_int00`, stack/DP setup, and `.cinit`
  processing; the exact sequence a loaded image runs before `main`
- `compiler/lib/src/exit.c` — where `C$$EXIT` comes from and what `A4` holds at that point
- [`include/elf/tic6x.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/elf/tic6x.h;hb=refs/tags/binutils-2_44) — all 34 `R_C6000_*` relocation types,
  the C6000 section/segment/dynamic tags, and the attribute definitions
- [`bfd/elf32-tic6x.c`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=bfd/elf32-tic6x.c;hb=refs/tags/binutils-2_44) — every relocation implemented, plus
  attribute merging and the fetch-packet-header fixups (`R_C6000_FPHEAD`, `R_C6000_NOCMP`)
- [`arch/c6x/kernel/module.c`](https://github.com/torvalds/linux/blob/v5.10/arch/c6x/kernel/module.c) and [`arch/c6x/include/asm/elf.h`](https://github.com/torvalds/linux/blob/v5.10/arch/c6x/include/asm/elf.h) — relocation
  processing and the ELF contract from the OS side
