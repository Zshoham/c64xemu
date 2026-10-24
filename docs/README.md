# c64xemu documentation

A cycle-stepped emulator of TI C64x+ DSPs — the single-core TMS320C6455 and the six-core
TMS320C6472 — covering the CPU cores, their memory and caches, and the device's peripheral set.
It loads TI C6000 EABI ELF images produced by TI's own toolchain and runs them cycle by cycle.

This directory holds three kinds of material, one per subdirectory.

## [guide/](guide/) — how to use the emulator

| Document | What it covers |
|---|---|
| [Getting started](guide/getting-started.md) | Building the emulator, what a fresh checkout provisions, and a first run end to end. |
| [Command line](guide/command-line.md) | Every `run`, `debug` and `decode` option, the state report, memory dumps, the performance report, and exit codes. |
| [Booting an image](guide/booting.md) | ELF loading, boot/configuration tables, HPI host boot, I2C EEPROM boot, and Ethernet and UTOPIA boot with external hosts. |
| [Debugging](guide/debugging.md) | The console REPL, the browser dashboard, the GDB stub, execution traces, profiling and fault diagnosis. |
| [Networking and host I/O](guide/networking-and-host-io.md) | pcap injection and capture, the live Unix-socket Ethernet backend, the TCP peer and TAP bridge, the HPI host side, and CIO. |
| [The six-core device](guide/multicore.md) | Running `--device c6472`: core release, focus, inter-processor interrupts, shared memory, and one Ethernet connection across six cores. |
| [Running the tests](guide/testing.md) | The native and firmware suites, what each kind of case proves, how to add one, and the differentials against TI's own tools. |

## [internals/](internals/) — how the emulator works

| Document | What it covers |
|---|---|
| [Architecture](internals/architecture.md) | The seams: machine, fabric, megamodule, core bus, device profile; the tick model, the run loop, halt rules and the state snapshot. |
| [Instruction decoding](internals/decoding.md) | Family identification from the binutils opcode tables, compact encodings and their fetch-packet header, and the instruction view every consumer crosses. |
| [Execution and timing](internals/execution.md) | Fetch and execute packets, predication, the write-back queue and delay slots, branches, the SPLOOP buffer, control registers, and where cycle counts come from. |
| [Memory and caches](internals/memory-and-caches.md) | The backing store, the address map and its aliases, L1P/L1D/L2 geometry and carve-out, coherence, the write buffer, memory protection and the bandwidth manager. |
| [Interrupts and exceptions](internals/interrupts.md) | The megamodule interrupt controller, event combiners and INTMUX, the CPU-side registers, exception reporting, and interrupts taken inside a software-pipelined loop. |
| [Peripherals](internals/peripherals.md) | The slave-port seam and the router, then each modelled register file: EDMA3 and IDMA, timers, GPIO, I2C, HPI, UTOPIA, McBSP, the PLLs, DDR2, EMIFA, device configuration and the C6472-only blocks. |
| [Ethernet](internals/ethernet.md) | The EMAC, its CPPI descriptor rings, receive filtering and statistics, the MDIO module and its PHY, the C6472 interrupt-control module, and the host backends. |

## [reference/](reference/) — the documents this work is derived from

Sixty-five TI manuals, application reports and papers, each under a descriptive filename with a
literature-number symlink beside it, plus the reference source the skills cite, linked at the
pinned upstream revision each was checked against. [reference/README.md](reference/README.md) is
the index.

Nine skills in [`skills/`](../skills/) distil that material into the knowledge needed to build
the emulator, with page-level references back into it.
