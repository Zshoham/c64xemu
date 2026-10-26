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
`--hpi-unix PATH`, `--hpi-peer PATH`, `--hpi-poll CYCLES`.

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
