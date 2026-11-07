# Networking and host I/O

Every way the emulated device exchanges data with the outside: pcap injection and capture, the
live Unix-datagram Ethernet backend, the two host tools built on it, the HPI host side, the
UTOPIA's ATM master, and CIO.

The EMAC model itself — descriptor rings, filtering, interrupts — is
[Ethernet](../internals/ethernet.md). This document is the seam between it and your host.

## Why Unix datagram sockets

The emulator is single-threaded and cycle-stepped while a host network is asynchronous, so
the Ethernet backend in `src/netio.rs` polls the socket from the run loop at a fixed cycle
interval and never touches descriptor memory itself: live frames and pcap input both enter
through the fabric's MAC operations. The backend also owns transmit capture and bounded live
history, so inspection cannot consume frames needed by the socket or the capture file.

Keeping the socket here rather than opening a TAP device keeps the emulator free of ioctl
bindings and of any privilege requirement. `tests/tools/tap_bridge.py` owns `/dev/net/tun` and
relays to this socket, and a test script can take the same place to drive a deterministic
exchange.

**The Ethernet framing is: one datagram is exactly one Ethernet frame.** No length prefix, no
header, nothing to parse. That is the whole protocol, and it is why writing a new host tool is a
half page of Python.

| Option | Meaning |
|---|---|
| `--net-unix PATH` | the socket the emulator binds |
| `--net-peer PATH` | the socket transmitted frames are sent to |
| `--net-poll CYCLES` | how often the run loop looks; default 1024, must be at least 1 |
| `--net-port N` | which MAC receives injected frames; default MAC 0, or the port an Ethernet boot was strapped to |

A datagram that arrives when the emulator is too far behind is counted rather than queued
forever: `net_dropped` in the state report is that counter, and a reader that far behind is not
coming back.

## pcap

`--pcap-in FILE` injects the file's frames as received traffic; `--pcap-out FILE` captures
everything the MAC transmits. Both use `src/pcap.rs` and the same injection path as the socket
backend, so a capture from a live run replays deterministically. This is the offline option: no
second process, no timing, useful for regression inputs and for feeding a firmware a frame it is
hard to provoke live.

## A real two-process exchange

`tests/tools/tcp_peer.py` is a complete host peer: a DHCP server, ARP, a TCP client, and enough
of an HTTP client to fetch a page. It starts the emulator itself, so one command is the whole
exchange:

```console
$ python3 tests/tools/tcp_peer.py --image tests/firmware/ndk_tcp/build/ndk_tcp.out
...
  "tx_frames": [
    "ffffffffffff02000011223308004500024000010000401178ad...",   # DHCP discover
    "ffffffffffff02000011223308060001080006040001020000112233c0a8400a...",  # ARP
    "0200004455660200001122330800450000440005000040067953...63363478656d7520736179732068656c6c6f206f766572205443500a",
    ...
  ]
}
peer: echo matched 28 bytes
```

That run is TI's NDK 2.22 TCP/IP stack on SYS/BIOS, built by TI's tools, leasing an address from
the host's DHCP server, resolving ARP, opening a TCP connection to the echo server on port 3000
and exchanging a payload — the ASCII at the end of the third frame is
`c64xemu says hello over TCP`. The whole exchange is about a second of wall time because the NDK
driver takes the EMAC's receive interrupt rather than polling.

Useful options, all from `tests/tools/tcp_peer.py`: `--port` and `--payload` for the TCP echo,
`--request` for an HTTP request against the stack's embedded file system, `--udp-port`,
`--dsp-ip`/`--peer-ip` for the addressing, `--net-poll` to pass a different poll interval
through to the emulator, and `--max-cycles`, `--timeout` and `--grace` for the run's limits.

`tests/tools/ibl_peer.py` is another scripted peer on this socket: the BOOTP, ARP and TFTP server
TI's IBL boots a program from - see [TI's IBL](booting.md#tis-ibl).

For a *real* host network instead of a scripted peer, `tests/tools/tap_bridge.py --tap dsp0
--socket /tmp/c64xemu.sock` relays between a TAP interface and the same socket; the emulator is
then started with `--net-unix /tmp/c64xemu.sock --net-peer /tmp/tap.sock`.

## The HPI host side

The host port interface is driven from a process over a second Unix datagram socket:
`--hpi-unix PATH`, `--hpi-peer PATH`, `--hpi-poll CYCLES`, or from a timed file with
`--hpi-replay FILE` ([below](#replaying-the-host-from-a-file)).

**One datagram is one host access**, eight bytes: `[direction, port, 0, 0, value]`, where value
is a little-endian 32-bit word, direction is `r` or `w` and port is the HCNTL encoding from
[SPRU969B](../reference/spru969b.pdf) Table 4 — 0 HPIC, 1 HPID with autoincrement, 2 HPIA, 3 HPID
without autoincrement. A read is answered with the four-byte value, or an empty datagram while
the host is held off. That is all a host needs to reach DSP memory and to exchange interrupts
with the CPU: write HPIA, then read or write HPID.

`tests/tools/hpi_host.py --image <image>` is the host half of a mailbox built on exactly that: it
writes a request block into DSP memory, raises DSPINT, waits for the firmware's HINT, clears it
and reads the reply, four rounds, checking every answer — so its exit status is the result. The
firmware side is `tests/firmware/hpi_mailbox/`, which takes DSPINT as a real interrupt (system
event 15 on the C6455).

`hpi_host.py --boot --image <image>` is the host for the C6455's HPI boot instead (see
[HPI host boot](booting.md#hpi-host-boot)). It loads and verifies the ELF's load segments, then
raises the DSPINT that releases core 0. With `--device c6472`, it instead loads the selected
cores' global aliases, programs their boot addresses and releases them through
`BOOT_COMPLETE_STAT`; `--cores 0,3` selects a subset (default: all six).
In either mode the tool fails when the emulator faults, writes no machine state, or, after
a released boot, never reaches the application's exit.

### Replaying the host from a file

`--hpi-replay FILE` replaces the host process with a timed CSV file. It is serviced from the run
loop exactly where the socket host is, through the same host accesses, so the FIFOs, bursts,
DSPINT and the C6472 boot controller behave as they do for a live host. It works in every boot
mode: an [HPI boot](booting.md#hpi-host-boot) in mode 1, or a mailbox host in mode 0.

```text
# tick,port,direction,value[,mask,within]      <- optional header, '#' optional
TICK,PORT,write,VALUE
TICK,PORT,read,-                               report the value
TICK,PORT,read,VALUE                           exact 32-bit expectation
TICK,control,wait,VALUE,MASK,WITHIN            poll HPIC until (hpic & MASK) == VALUE
TICK := N | +D        absolute tick, or D ticks after the previous row completed
PORT := control | data-auto | address | data   HCNTL 00 / 01 / 10 / 11, SPRU969B Table 4
```

- **Ticks are the run clock**: device cycles on an HPI boot (before and after release) and on a
  C6472, the core's cycles, stalls included, on a C6455 that boots nothing. Every access happens
  at the tick its row names, or the file fails saying why not, and that tick depends only on the
  image, the command line and the file: not on the front end, breakpoints, diagnostics or how the
  run loop batches. A rerun gives the same transcript and final state. Ticks are emulator time,
  not calibrated board time.
- Rows run in file order. Rows due at the same tick run back to back before that tick's cycle.
  An absolute `N` must not decrease. `+D` counts from the tick the previous row **completed**
  (a wait completes when it matches; before the first row the anchor is 0). An absolute row that
  falls due before the previous row completed fails with `missed tick`: use `+D` for anything
  that follows a wait.
- `wait` polls HPIC (control port only; a host HPIC read changes nothing), sampled at the tick it
  becomes due and at every tick through `WITHIN` ticks after it; a match on the last sample
  wins, and a sample while HPIRST holds the host off is no match. `WITHIN > 0`, `MASK != 0`, and
  `VALUE` has no bits outside `MASK`. HRDY (`0x8`) is how a host waits for the firmware to clear
  HPIRST, HINT (`0x4`) how it waits for an answer (SPRU969B sections 4.1 and 5).
- A plain read or write while HPIRST holds the host off fails the file, and so does a read whose
  value differs from the row's. Nothing flushes the write FIFO for the file at its end or on a
  failure: the file must fence (SPRU969B section 6.3), for example by writing HPIA again.
- `address` takes the word address the host writes to HPIA, not the byte address.
- The parser is strict. Whitespace around fields, blank lines and `#` comments (also trailing)
  are allowed, and so is a header as the first row. Unknown ports or directions, wrong field
  counts (4, or 6 for `wait`), numbers out of range, an empty file and aliases (`r`, `w`, numeric
  HCNTL codes, `data-auto-increment`) are load errors, `FILE line N: ...`, before anything runs.
  Numbers are decimal or `0x` hexadecimal.

Each unchecked read and each wait that matches is printed on stderr as it happens
(`hpi-replay line=22 tick=2048 port=control read=0x00000008`); passing checked reads are
silent. A failure - held off, read mismatch, wait timeout, missed tick, a tick or wait window past the
end of the 64-bit clock (tick arithmetic is checked, never saturated), or strict physical memory
refusing a row's access - stops the file and is reported as the machine's fault, so every front
end stops on it: `run` prints it, writes the state report and exits 1; the GDB stub reports
SIGILL with the text on stderr; the console and the dashboard show it. Nothing else happens at
the boundary a row failed at: no UTOPIA service, no CIO request, no cycle. The state report's
`hpi_replay` member records the progress. A batch run whose program exits before the file is
complete fails with `hpi replay: program exited before line N`. Breakpoints, stepping and
`--max-cycles` work as usual: a stop due at the same boundary as a row wins, and the row runs on
resume. The console's and the dashboard's reset start the file over.

`--trap-oob` cannot be combined usefully with an HPI boot: an image that came over the wire
reports no loaded ranges, so the run stops at cycle 0 before any row runs.

The mailbox exchange of `tests/firmware/hpi_mailbox`, as a file:

```csv
0,control,wait,0x8,0x8,50000000
+0,address,write,0x00210080
+0,data-auto,write,1
+0,data-auto,write,8
+0,address,write,0x00210080
+0,address,write,0x00210000
+0,data-auto,write,1
+0,data-auto,write,2
+0,data-auto,write,1
+0,data-auto,write,0
+0,address,write,0x00210000
+0,control,write,2
+0,control,wait,0x4,0x4,5000000
+0,control,write,4
+0,address,write,0x00210040
+0,data-auto,read,1
+0,data-auto,read,9
```

## The UTOPIA ATM master

The UTOPIA slave's master is a process on a third Unix datagram socket: `--utopia-unix PATH`
and `--utopia-poll CYCLES` (default 1024). It works on either device; replies go to each
request's sender. One datagram is one cell-level transaction of the UTOPIA Level 2 address
phase ([SPRUE48](../reference/sprue48.pdf) sections 3.1-3.7): an ASCII opcode, the five-bit PHY
address the master drives, and for a write the cell's wire bytes.

| Request | Reply |
|---|---|
| `p`, PHY | URCLAV: one byte `1` if the receive queue has room for a cell, `0` if not; an empty datagram when the pin is high-impedance (another PHY, the null PHY 31, or a receiver in reset) |
| `w`, PHY, 53-64 wire bytes | `1` if the slave took the cell, `0` if it refused it (not addressed, or no room) |
| `x`, PHY | UXCLAV: `1` if a complete cell waits in the transmit queue, `0` if not; empty when high-impedance |
| `r`, PHY | The waiting cell's wire bytes, without the packet's dummy bytes; one byte `0` when nothing was taken |
| anything else, or a PHY above 31 | One byte `FF` |

There is no host-side queue: a full receive queue refuses a write, and the master retries the
refused write rather than an unanswered one, which could duplicate a cell. The transport moves
whole cells, not UTOPIA clock edges. On the C6472 the transmit queue fills only through the
PDMA, so `x` answers `1` once a programmed transmit channel has written a cell there.

`tests/tools/utopia_boot.py` is the master for the C6472's
[UTOPIA boot](booting.md#utopia-boot); its `send` and `receive` helpers are what
`tests/test_utopia.py` drives the echo firmware with on both devices, so the master reads back
the cell it sent. `tests/firmware/utopia_echo/` is a C6455 program that copies each received
cell into the transmit queue with the CPU; `tests/firmware/c6472_utopia_echo/` has the PDMA
move each cell from the receive queue into core 0's L2 and from there into the transmit queue.

## CIO

CIO is TI's runtime host-I/O protocol, and it is what makes `printf` work in a target program.
`src/cio.rs` implements the request types the runtime issues: `DT_OPEN`, `DT_CLOSE`, `DT_READ`,
`DT_WRITE`, `DT_LSEEK`, `DT_UNLINK`, `DT_GETENV`, `DT_RENAME`, `DT_GETTIME`, `DT_GETCLK`,
`DT_GETTIME64` and `DT_SYNC`, with a one-megabyte cap on a single transfer.

Nothing needs enabling. Output appears on stdout as it is produced and again in the state
report's `cio_output`, so a captured report is self-contained. The cost is that each CIO
transaction is a breakpoint-and-handshake in the target's runtime, exactly as on hardware — a
program that prints in a loop spends real cycles on it, which is visible in a cycle count and is
the honest number.
