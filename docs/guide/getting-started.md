# Getting started

Building the emulator, what a fresh checkout provisions before the test suites can run, and one
image executed end to end with its state report explained.

## What you need

The Rust side needs a stable toolchain (MSRV 1.80) plus `cargo-nextest`. The Python side is the
standard library plus pytest. Nothing else is required to build and run the emulator itself —
the TI toolchains are needed only to *build* target firmware, not to execute an image that
already exists.

`devenv shell` supplies all of it, and `just` is the cross-language entry point. The underlying
runners stay directly usable, so nothing here depends on either tool being present.

| Task | With `just` | Directly |
|---|---|---|
| Build the release emulator | `just build` | `cargo build --release --locked` |
| Run an image | `just run IMAGE ...` | `cargo run --release --locked -- run IMAGE ...` |
| Native tests | `just test-rust` / `just test-fast` | `cargo nextest run --all-targets --all-features --locked` |
| Firmware integration suite | `just test-conformance` | `python3 -m pytest` |
| Everything | `just test` | both of the above |
| Local gate | `just check` | `cargo fmt --all -- --check`, `cargo clippy --all-targets --all-features --locked -- -D warnings`, native tests |
| Pre-merge gate | `just ci` | the gate plus the full pytest suite |
| Build firmware by hand | `just firmware` | `tests/firmware/bios_kernel/build.sh`, `tests/firmware/ndk_tcp/build.sh` |

The recipes are in `justfile`; each one is a single command, so reading it is faster than
guessing what a recipe wraps.

## What a fresh checkout provisions

Nothing toolchain-sized or third-party is versioned. `tests/conftest.py` has a session-scoped
autouse fixture, `test_dependencies`, that runs before any suite: if any of the six trees under
`tests/build/` is missing it invokes `tests/tools/fetch_deps.sh`, and if either firmware image is
missing it builds them.

| Tree under `tests/build/` | What it is |
|---|---|
| `ti-cgt-c6000_8.5.0.LTS` | TI's C6000 code generation tools: `cl6x`, `asm6x`, `dis6x`, `ofd6x`, `hex6x`, `nm6x` |
| `xdctools_3_32_01_22_core`, `bios_6_42_03_35`, `ndk_2_22_02_16` | XDCtools, SYS/BIOS 6.42 and the NDK 2.22, which the two firmware images are built from |
| `dsplib-c64plus` | DSPLIB 3.4.0.0, whose natural-C kernels 23 target-program specs compile |
| `binutils-tic6x-vectors` | the 231 tic6x vector files from the binutils 2.44 release that the decoder differential sweeps |

That is roughly 600 MB of downloads, cached in `tests/build/.downloads` so a re-provision does not
re-fetch. The whole directory is gitignored. Firmware images under `tests/firmware/*/build/*.out`
are build products on the same footing.

Two portability notes, both from `tests/tools/fetch_deps.sh` and the build scripts: the SYS/BIOS
component ships as a 32-bit installer, and the XDCtools launchers are dynamically linked. From the
devenv shell both run inside the `ti-tools-fhs` FHS environment — `tests/conftest.py` detects
`ti-tools-fhs` on `PATH` and re-invokes the fetch script through it. On other distributions you
need the 32-bit loader (`sudo apt install libc6-i386`).

## A first run

`c64xemu run` loads an ELF image, executes it to its exit point and prints a state report. The
SYS/BIOS kernel exerciser is a good first image because it prints through CIO and then exits:

```console
$ ./target/release/c64xemu run tests/firmware/bios_kernel/build/bios_kernel.out --max-cycles 20000000
bios kernel consumed 64 relayed 64 outer 22 inner 22 nested 22 swi 22 woken 16 failures 0 ticks 4 slept 2 letters 16 masked 1 events 4 held 1 released 1 checksum 3410629677
{
  "cio_output": "bios kernel consumed 64 relayed 64 outer 22 inner 22 nested 22 swi 22 woken 16 failures 0 ticks 4 slept 2 letters 16 masked 1 events 4 held 1 released 1 checksum 3410629677\n",
  "cores": [
    {
      "core": 0,
      "cycles": 4120027,
      "fault": null,
      "focused": true,
      "halted": true,
      "held_in_reset": false,
      "pc": 8473408
    }
  ],
  "cycles": 4120027,
  "device": "c6455",
  "exit_code": 0,
  "fault": null,
  "halted": true,
  "loop_active": false,
  "net_dropped": 0,
  "pc": 8473408,
  "registers": {
    "A0": 0,
    ...
  }
}
```

The first line is the firmware's own `printf`, delivered through the CIO handshake as the program
runs. Everything after it is the state report `src/runner.rs` writes when the run ends — to stdout
by default, or to a file with `--state-out`.

| Field | Meaning |
|---|---|
| `cio_output` | everything the program wrote through CIO, repeated here so a captured report is self-contained |
| `cores` | one entry per core: its cycle count, fault, whether it is the focused core, whether it has halted, and whether it is still held in reset |
| `cycles` | the run's cycle count |
| `device` | which device profile was emulated, `c6455` or `c6472` |
| `exit_code` | what the program passed to `C$$EXIT`, which is also the process exit code |
| `fault` | the fault that stopped the run, or `null` |
| `halted` | whether the machine reached its stop condition rather than running out of budget |
| `loop_active` | whether the SPLOOP buffer was active when the run stopped |
| `net_dropped` | frames the Ethernet backend could not deliver |
| `pc` | the focused core's program counter |
| `registers` | the focused core's register file and control registers |
| `stats` | the run's counters: execute packets issued, instructions `retired` (those whose predicate held), SPLOOP replays, branches, interrupts, stores, stalls by class, faults |
| `tx_frames` | every Ethernet frame the device transmitted, hex-encoded |

A run that hits its cycle budget instead of its exit point says so on stderr and exits non-zero,
which is what `--max-cycles` is for: an image that hangs stops rather than spinning forever.

## Where to go next

| If you want to | Read |
|---|---|
| Know every option and artefact | [Command line](command-line.md) |
| Load memory some way other than the ELF segments | [Booting an image](booting.md) |
| Step, break, trace or attach GDB | [Debugging](debugging.md) |
| Give the firmware a network or a host peer | [Networking and host I/O](networking-and-host-io.md) |
| Run six cores | [The six-core device](multicore.md) |
| Run or extend the suites | [Running the tests](testing.md) |
| Understand what happens inside | [Architecture](../internals/architecture.md) |
