# c64xemu skills

Nine skills distilled from the documentation in `docs/`, the reference implementations linked
from `docs/reference/README.md`, and the third-party trees `tests/tools/fetch_deps.sh` provisions into
`tests/build/`.
Each lives in `skills/<name>/SKILL.md` and ends with a **Files that expand on this**
section naming the exact documents and source files it was derived from, so any claim can be
re-checked.

## Project scope they were written for

A **standalone TMS320C6455-class C64x+ DSP** emulator. Functional / instruction-accurate CPU
(no cycle model), but with **real models of the caches, the interrupt controller and the
Ethernet MAC**. The emulated core talks to the outside world only over Ethernet: packets in,
DSP processing, packets out. The goal is that firmware behaves the same in the emulator as
on silicon — including failing the same way when it forgets a cache writeback.

## The skills

**CPU core**

| Skill | Covers |
|---|---|
| `c64xplus-instruction-decoding` | Fetch/execute packets, p-bits, the compact 16-bit header, predication, and generating the decoder from binutils' tables rather than the PDF opcode maps |
| `c64xplus-execution-model` | Delay slots and the write-back queue, parallel-packet read/commit semantics, resource constraints, branches, addressing modes including circular |
| `c64xplus-registers-and-control` | Register files, 40/64-bit pairs, every control register with bit layout and MVC rules, the Supervisor/User privilege model |
| `c64xplus-sploop-buffer` | The hardware loop buffer state machine — load/fetch/drain/reload, ILC/RILC/LBC, the three loop-start instructions, and the interrupt pipe-down/pipe-up protocol |

**System**

| Skill | Covers |
|---|---|
| `c64xplus-interrupts-and-exceptions` | Both layers: the megamodule INTC (128 events, combiners, INTMUX) and the CPU side (IFR/IER/GIE/NMIE, the IST, TSR/ITSR/NTSR context save, IERR causes), plus the C6455 event map |
| `c64xplus-memory-and-cache` | C6455 memory map, L1P/L1D/L2 geometry and control registers, coherence rules and what to model vs. ignore, IDMA, memory protection |
| `c6455-emac-ethernet` | CPPI descriptors, the HDP/CP queue and interrupt-acknowledge protocol, RX filtering, the shared MACINT and EWCTL gating, the MDIO/PHY model, host bridging |

**Getting code in and proving it right**

| Skill | Covers |
|---|---|
| `c6000-elf-abi-and-boot` | EABI ELF and relocations, the calling convention, C6455 boot modes and reset state, and the CIO host-I/O protocol that makes `printf` work |
| `c64xemu-validation` | Differential decoder testing against `dis6x`, binutils' ~2474 ready-made vectors, the DSPLIB optimized-vs-reference execution corpus, SPLOOP cycle-flow tests, a GDB stub |

## Suggested build order

1. **Loader + flat memory + decoder.** Validate against `dis6x` immediately — do not write
   any semantics until the decoder round-trips a compiled corpus. (`c6000-elf-abi-and-boot`,
   `c64xplus-instruction-decoding`, `c64xemu-validation`)
2. **Execution core with the write-back queue.** Delay slots from day one; retrofitting them
   means rewriting every instruction. (`c64xplus-execution-model`,
   `c64xplus-registers-and-control`)
3. **CIO semihosting.** Cheap, and it turns every C program into a test.
4. **SPLOOP.** Build the SPRU732J Table 7-1/7-2 tests first, then implement.
   (`c64xplus-sploop-buffer`)
5. **Interrupts + INTC + a timer.** (`c64xplus-interrupts-and-exceptions`)
6. **Cache and the real memory map.** (`c64xplus-memory-and-cache`)
7. **EMAC + MDIO + a host backend.** (`c6455-emac-ethernet`)

Steps 1-3 get you running real compiled code. Step 4 is where most C64x+ emulator attempts
would stall, which is why it has its own skill.

## Where the facts came from

Every claim traces to a PDF in `docs/`, to a reference source linked at a pinned upstream
revision, or to a tree `tests/tools/fetch_deps.sh` provisions into `tests/build/`:
the TI code generation tools, DSPLIB, and binutils' tic6x vectors.
`docs/reference/README.md` is the index, and its Reference source table gives the exact revision every
source link is pinned to.
Each skill ends with a Reference section naming the document and page range, so the documents
re-check without a network, the source links without a checkout, and the provisioned trees
after one fetch.
