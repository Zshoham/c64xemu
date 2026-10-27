# Debugging

The three front ends `c64xemu debug` offers — a console REPL, a browser dashboard and a GDB
remote stub — plus execution tracing, profiling and the options that turn a hang or a fault into
a diagnosis.

All three drive the same machine through the same seam, so a breakpoint, a step and a state dump
mean the same thing in each. Option syntax and defaults are in [Command line](command-line.md).

On a multicore device, breakpoint reports name the stopped core and address without changing
the focused core. Register, memory and disassembly views remain on the selected core; use
`core N`, the dashboard core selector, or GDB's thread selection to inspect the stopped core.
Reset retains the breakpoint list, and continue/step manages leaving the current halt itself.

## The console REPL

`c64xemu debug IMAGE` with neither `--ui` nor `--gdb` starts the REPL in `src/debugger.rs`. The
command table is its `help` output:

| Command | Aliases | What it does |
|---|---|---|
| `info` | `i` | machine state summary |
| `core [N]` | | show the device's cores, or drive core N |
| `regs [a\|b\|c]` | | register dump; default both banks, `c`/`control` for the control registers |
| `reg NAME VALUE` | | set an A or B register |
| `mem ADDR [LEN]` | `x` | hexdump memory (host view) |
| `disas [ADDR] [N]` | `d` | disassemble N instructions, default from the PC |
| `syms [PATTERN]` | | list symbols |
| `break [ADDR\|SYM]` | `b` | set or list breakpoints; they halt at fetch |
| `delete ID\|all` | | remove breakpoints |
| `watch REG` | | stop when a register changes |
| `unwatch ID\|all` | | remove watchpoints |
| `continue [CYCLES]` | `c` | run until breakpoint, watch, fault or budget |
| `stepc [N]` | | run N cycles |
| `stepi [N]` | `s` | dispatch N execute packets |
| `cache` | | cache geometry and resident lines |
| `state [FILE]` | | the JSON machine state `run` reports |
| `reset` | | reload the image and start over |
| `trace on FILE [json]` / `trace off` | | start or stop tracing; `off` flushes |
| `write ADDR HEXBYTES` | | patch memory |
| `help` | `?` | the table above |
| `quit` | `q` | leave |

A real session on the SYS/BIOS image, driven by piping commands in:

```console
$ printf 'info\nbreak main\ncontinue\ninfo\ndisas 0x00800000 4\nstepi 3\nquit\n' \
    | c64xemu debug tests/firmware/bios_kernel/build/bios_kernel.out
c64xemu debugger - 'help' for commands
pc=0x00812140 <_c_int00> cycle=0: mvk .l2 0,b0
(c64xemu) pc=0x00812140 <_c_int00> cycle=0 packets=0 fault=None
csr=0x10000000 (gie=0) ierr=0x00000000 ifr=0x0000 ier=0x0000
sploop inactive
packets=0 retired=0 replays=0 branches=0 interrupts=0 stores=0 faults=0
(c64xemu) breakpoint 0: 0x00802884 <main>
(c64xemu) breakpoint 0 on core 0 at 0x00802884 <main> (cycle 111098)
pc=0x00802884 <main> cycle=111098: addk .s2 -168,b15
(c64xemu) pc=0x00802884 <main> cycle=111098 packets=110346 fault=None
csr=0x10000000 (gie=0) ierr=0x00000000 ifr=0x0000 ier=0x0002
sploop inactive
packets=21014 retired=84512 replays=89332 branches=5129 interrupts=0 stores=20779 faults=0
  nop 2
breakpoints:
  0: 0x00802884 <main>
(c64xemu)    00800000 <.text:__TI_printfi:__TI_printfi>: callp .s1
   00800004 <$C$RL300>: || addk .s2 -576,b15
   00800008: mv .l2x a8,b5
   0080000a: || mv .l1 a4,a10
(c64xemu) pc=0x00802890 <main+0xc> cycle=111106: || mv .l1x b4,a4
```

Two things in that transcript are worth naming. `info` prints from a single snapshot — every line
describes the same instant, which was not true when each line asked the machine a fresh question.
And `packets` appears twice with different values because one is the run's dispatch count and the
other the statistics block's; read the labels rather than the position.

The `disas` listing puts `||` on the instruction whose own parallel bit is set. TI's tools put it
on the *following* instruction, so a listing here is shifted by one relative to `dis6x`; the
decoder differential strips the marker before comparing, so nothing catches it. See
[Instruction decoding](../internals/decoding.md).

## The browser dashboard

`--ui [ADDR]` serves the dashboard in `src/ui.rs` with the page in `src/ui.html` instead of the
REPL. It binds a TCP listener and prints the URL. Two behaviours differ from a plain session, both
deliberate: the dashboard forces profiling and Ethernet history on regardless of `--profile`,
because its hotspot and traffic views read them, and it ignores `--max-cycles`, because the
session is driven from the page.

## The GDB stub

`--gdb [ADDR]` serves the remote serial protocol over TCP and accepts exactly one connection.

```console
$ c64xemu debug IMAGE --gdb :3333
$ tic6x-uclinux-gdb IMAGE -ex 'target remote :3333'
```

The register layout is GDB's own tic6x target description, and `src/gdb.rs` states it exactly:
registers 0-31 are A0-A15 and B0-B15, 32 is CSR, 33 is PC, 34-65 are the upper halves A16-A31 and
B16-B31, then TSR, ILC and RILC. The stub publishes that description through `qXfer`, so a GDB
built for the target gets the layout from the connection.

Three properties make the stub behave like a debugger rather than an approximation of one.
Breakpoints halt in the CPU itself at a fetch boundary, so a branch resolving straight onto a
breakpoint cannot fetch past it in the same cycle. `s` dispatches exactly one execute packet. And
the machine's own step services the CIO handshake and the Ethernet and HPI backends, so a
printf-style or networked program behaves under GDB as it does under `run`. Reaching the
program's exit point is reported as an exited process, not as a target that stopped for no
reason.

A breakpoint stop uses a `T05` reply containing `thread:` (the one-based core number) and the
expedited PC register `21:` in target byte order. `?` repeats the last stop reply; it does not
select that core. Other completed steps still use `S05`.

## Execution traces

`--trace FILE` records one event stream, rendered either as aligned text or as JSON Lines
(`--trace-format json`). Tracing is observationally passive: every event is rendered from state
the emulator already committed, and every emit site is guarded, so a run without `--trace`
allocates nothing.

```console
$ c64xemu run IMAGE --max-cycles 3000 --trace /tmp/t.txt
$ head -4 /tmp/t.txt
[       0] packet 0x00812140: <L2> mvk .l2 0,b0
[       0] b0 = 0x00000000 (was 0x00000000)
[       1] packet 0x00812144: <S2> mvc .s2 b0,ier
[       1] ier = 0x00000000 (was 0x00000000)
```

The bracketed number is the cycle. A `packet` line gives the address, the functional unit each
instruction issued on, and the instruction as the instruction view names it. A bare `name = new
(was old)` line is a write-back landing — which is why writes appear *after* the packet that
scheduled them, and how a delay slot is visible in a trace at all.

The same run as JSON Lines:

```console
$ c64xemu run IMAGE --max-cycles 500 --trace /tmp/t.json --trace-format json
$ head -2 /tmp/t.json
{"cycle":0,"type":"packet","pc":"0x00812140","insn":["mvk .l2 0,b0"],"units":[{"u":"L","s":2,"x":false,"t":null,"r":false,"i":"mvk .l2 0,b0"}]}
{"cycle":0,"type":"write","reg":"b0","old":"0x00000000","new":"0x00000000"}
```

Field names are stable and words are hex-encoded, so the stream is meant to be parsed. The event
set in `src/trace.rs` is `packet`, `replay` (an SPLOOP cycle re-executing buffered packets, with
its offset), `stall` (with the reason: `memory`, `nop` or `interrupt`), `write`, `store`,
`control`, `branch`, `interrupt`, and the SPLOOP start and end markers.

Three options make tracing affordable on a long run:

- `--trace-start CYCLE` records nothing before that cycle.
- `--trace-ring N` keeps only the last N events in memory and flushes them at exit — the answer
  to "what happened right before the fault" without recording ten million cycles.
- `--trace-exceptions` reports the cause of every internal exception the firmware is left to
  handle, which is how you tell a broken image from one that is deliberately handling faults.

## Profiling

`--profile` keeps the per-address packet histogram that the dashboard's hotspot views read, and
`--perf` prints an emulated-CPU performance report when a batch run ends: cycle and packet
counts, stall breakdown, cache statistics and interrupt overhead. The two are independent —
`--perf` emits its cache blocks without `--profile`.

## Recipes

**The image hangs.** Give it a budget and a post-mortem window: `--max-cycles N --trace-ring 2000`.
The last packets before the budget expired say whether it is spinning on a poll, waiting on an
interrupt that never arrives, or looping in the boot.

**The image faults.** `--pc-trail N` keeps the last N execute-packet addresses (collapsing
consecutive repeats) and prints them with the fault. `--trap-oob` stops as soon as the program
counter leaves every loaded segment, which catches a wild branch at the branch rather than
wherever it eventually dies.

**The wrong instruction executed.** `c64xemu decode WORD[/HEADER]` names the encoding in
isolation; if the emulator and TI's `dis6x` disagree the differential in
[Running the tests](testing.md) is the place to settle it.

**A DMA buffer holds the wrong data.** `--coherence-check` reports, once per cache line, each
transfer the caches got wrong for want of a cache operation, with the core, cycle and PC:

```console
$ c64xemu run --coherence-check IMAGE
coherence: core 0 cycle 40765 pc 0x00803a30: a DMA master read memory the L1D line at 0xe0003000 holds dirty; write the range back before the transfer
```

The same run without the writeback transmits whatever memory held, exactly as silicon would; the
report is what says why. The three kinds and the rule behind each are in
[Memory and caches](../internals/memory-and-caches.md#the-coherence-check).

**The cycle count is too high.** `--perf` attributes the cycles to stall classes, and `cache` in
the REPL shows what is resident. The costs each class charges, and the manual rule behind each,
are in [Execution and timing](../internals/execution.md).
