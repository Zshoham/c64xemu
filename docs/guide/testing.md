# Running the tests

The two suites, what each kind of case proves, how to select cases, and how to add one.

The emulator is written to be checkable, and the suites are the point rather than an afterthought:
`cargo nextest` runs 817 native tests, `pytest` collects 1992 cases that build target firmware and
drive it through the same `c64xemu run` command anyone else uses.

## Running them

```console
$ just test-fast                                    # native only: the edit-compile-test loop
$ just test                                         # native plus every firmware integration case
$ just check                                        # format check, Clippy, native tests
$ just ci                                           # the full pre-merge gate
```

Without `just`:

```console
$ cargo nextest run --all-targets --all-features --locked
$ python3 -m pytest
```

The pytest suite provisions itself. `tests/conftest.py` fetches the TI toolchains, DSPLIB and the
binutils vectors into `tests/build/` and builds the firmware images if any are missing; see
[Getting started](getting-started.md).

CI runs the same split in `.github/workflows`: a `check` job (`cargo fmt --all -- --check`,
`cargo clippy --all-targets --all-features --locked -- -D warnings`, `cargo nextest run`) and a
`conformance` job that builds the release binary and runs `python3 -m pytest -m "not high_bandwidth"`.
Locally, run everything.

## Selecting cases

Markers are declared in `pyproject.toml`, with `--strict-markers`, so a typo is an error rather
than an empty run. Collected counts today:

| Marker | Cases | What it selects |
|---|---|---|
| `programs` | 1868 | TI-compiled target program cases |
| `network` | 28 | NDK firmware and host TCP peer cases |
| `rtos` | 9 | TI-RTOS firmware cases |
| `host` | 34 | HPI mailbox, C6455/C6472 HPI boot, C6472 UTOPIA boot, and C6455 UTOPIA cell echo integration cases |
| `differential` | 3 | binutils and TI disassembler differentials |
| `slow` | — | cases that compile target programs or execute full firmware |
| `high_bandwidth` | 5 | paced TCP cases at 1024 kbps and above, deselected in CI |

```console
$ python3 -m pytest -m network                        # one domain
$ python3 -m pytest -m programs -k packed_simd        # one program, by node ID
$ cargo nextest run -E 'test(component_conformance)'  # the native low-level cases
```

Every integration expectation keeps its case ID as the pytest node ID, so a failure names the
case rather than a line number.

## What the six kinds of case prove

**Decode vectors.** Exact instruction-word expectations from the binutils tic6x test suite,
checked mnemonic by mnemonic.

**Differentials against TI's own tools.** Three of them, run inside every pass and asserted as
cases. `tests/tools/decode_sweep.py` sweeps every instruction word in binutils' vector corpus and
requires each mnemonic to match binutils' own rendering; `tests/test_differential.py` pins the
result exactly — `{"exit": 0, "words": 2195, "matched": 2032, "skipped": 163}`, the skips being
floating-point encodings this device does not have. `tests/tools/dis6x_diff.py` runs TI's own
`dis6x` over every built firmware and program image and compares seven dimensions: mnemonic,
functional unit, cross-path marking, predicate, memory data path, every memory access's address
operand, and every operand list the model reads.

That differential is not decoration. Four defects in this emulator came from it rather than from
reading manuals: seven C67x instructions the device does not have, a compact subtract that
saturated on the wrong functional unit, cross-path reads that went unrecorded so packets TI's
assembler refuses were accepted, and a 40-bit subtraction from a constant that raised a fault
although `cl6x` assembles it.

**Behavioural cases.** Hand-computed CPU, cache, memory, interrupt, EDMA3, timer, GPIO, I2C,
Ethernet and MDIO expectations, each citing the manual section it comes from.

**Timing cases.** Cycle counts derived from the documented stall rules *before* the emulator was
measured, never fitted to it. The strongest example is
[SPRU862B](../reference/spru862b.pdf) Table 1-3: sixteen averages for this device's L2, which
solving for a single line occupancy — 14/3 cycles out of L2 SRAM, six out of the L2 cache —
reproduces entry for entry. Table 3-2 does the same for the data side. The reasoning is in
[Execution and timing](../internals/execution.md).

**Live network cases.** A host process leases the firmware an address over DHCP and then holds a
real TCP connection with it; see [Networking and host I/O](networking-and-host-io.md).

**Component conformance cases.** 398 JSON vectors under `tests/vectors/component/`, executed
natively in-process by `src/component_conformance.rs`. `build.rs` walks that directory and
generates one `#[test]` per file, so each vector is its own nextest case with its own name.
`tests/vectors/component-python-baseline.json` remains a historical migration record, not an
executable equality constraint on today's fixtures. The per-case vectors exercise current
behavior; intentional interface changes update those expectations with behavioral coverage.

## Adding a target program

A program is a directory under `tests/programs/` containing `spec.json` and its sources.
`tests/target_programs.py` reads the schema:

| Key | Required | Meaning |
|---|---|---|
| `sources` | yes | source files, resolved against the program directory then the repository root |
| `weight` | yes | scheduling weight for the case |
| `includes` | | extra include directories, relative to the repository root |
| `cflags`, `link_flags` | | extra `cl6x` and linker arguments |
| `entry` | | entry symbol, default `_start` |
| `max_cycles` | | cycle budget, default 100000 |
| `opt_levels` | | optimisation levels to build at; the default set otherwise |
| `rts` | | link TI's runtime; when false, `tests/programs/common/crt.asm` is added |
| `expect` | | register or state expectations, as name → integer |
| `cio_output` | | the exact CIO text the program must print |
| `rx_frames`, `tx_frames` | | Ethernet frames to inject, and the exact frames it must transmit |
| `boot_table` | | load through a boot table at this hex address instead of the ELF segments |
| `boot_config` | | `{"address": "0x...", "entries": [[target, set, clear], ...]}` |
| `i2c_boot` | | reach the boot table through the ROM's I2C EEPROM boot; needs `boot_table` |
| `cache_rerun` | | also run with caches enabled, default true |
| `link_cmd` | | a linker command file; `tests/programs/common/link.cmd` otherwise |

Programs are compiled with `cl6x -mv6400+ --abi=eabi -q --symdebug:none` plus the spec's flags,
at each optimisation level the spec names — which matters, because higher levels emit more
SPLOOP, more compact encodings and deeper delay-slot packing.

## Adding a component vector

Drop a JSON file under `tests/vectors/component/<domain>/`. `build.rs` picks it up on the next
build and generates the test; there is no registry to edit. The shape is a request and its
expected observations:

```json
{
  "domain": "interrupts",
  "request": {
    "action": "system",
    "id": "interrupts:utopia-clock-failure-and-transmit-event",
    "operations": [
      {"op": "read32",  "address": 45350912},
      {"op": "write32", "address": 45350912, "value": 4294967295}
    ]
  },
  "expected": {
    "observations": [536887296, {"$bits": {"set": 0, "clear": 16}}]
  }
}
```

An observation is either an exact value or a `$bits` assertion naming the bits that must be set
and clear — which is what lets a vector pin the one field it is about without freezing the
reserved bits beside it.

## Adding an integration case

Integration cases live in `tests/test_*.py`, one module per domain, and share the fixtures in
`tests/conftest.py` and the helpers in `tests/common.py`. A new case joins the module for its
domain and takes the marker that module already uses; keep the case ID stable, because it is the
node ID a failure reports and the name the expectation is recorded under.
