#!/usr/bin/env python3
"""Drive the emulated C6455's or C6472's host port interface from a process on the host.

The emulator exposes the HPI's host side over a Unix datagram socket: one datagram is one host
access, `[direction, port, 0, 0, value]`, where direction is `r` or `w` and port is the HCNTL
encoding from SPRU969B Table 4 - 0 HPIC, 1 HPID with autoincrement, 2 HPIA, 3 HPID without. That
is all a host needs to reach DSP memory and to exchange interrupts with the CPU.

By default this script drives the firmware mailbox. With --boot it instead loads ELF code and
data directly through the HPI (not a boot table) and verifies the transferred bytes.

C6455 (--device c6455): host boot always starts at 0x00800000, so the ELF must place its entry
there, and the host's DSPINT releases the held core.

C6472 (--device c6472): all six cores are held in reset (SPRUEC6G section 4.3.2) and DSPINT does
not release any. The host writes --cores' images to their global aliases (local L2 at
0x1n800000 for core n; shared addresses unchanged), writes each core's DSP_BOOT_ADDRn with the
ELF entry >> 10, and sets those cores' BCn bits in BOOT_COMPLETE_STAT. The entry must therefore
be 1 KB aligned in local L2. --no-release stops before the BCn write, and --dspint raises DSPINT
as well, to show that alone starts nothing.
"""

from __future__ import annotations

import argparse
import json
import os
import select
import socket
import struct
import subprocess
import sys
import tempfile
import time

if __package__:
    from .emac_boot import (
        BOOT_COMPLETE_STAT, DSP_BOOT_ADDR0, LOCAL_WINDOWS, PT_LOAD, BootError, Image,
        global_alias, parse_elf,
    )
else:
    from emac_boot import (
        BOOT_COMPLETE_STAT, DSP_BOOT_ADDR0, LOCAL_WINDOWS, PT_LOAD, BootError, Image,
        global_alias, parse_elf,
    )

HPIC, HPID_AUTO, HPIA, HPID = 0, 1, 2, 3
HPIC_DSPINT = 1 << 1
HPIC_HINT = 1 << 2
HPIC_HRDY = 1 << 3
L2_BASE = 0x0080_0000
C6472_CORES = 6
DSP_BOOT_ADDR_STRIDE = 0x20

REQUEST = 0x0084_0000
REPLY = 0x0084_0100
PAYLOAD = 0x0084_0200

COMMAND_SUM = 1
COMMAND_SWAP = 2


class Host:
    """The host side of the HPI, one access per datagram."""

    def __init__(self, sock: socket.socket, emulator_path: str, timeout: float):
        self.sock = sock
        self.emulator_path = emulator_path
        self.timeout = timeout

    def write(self, port: int, value: int = 0) -> None:
        self.sock.sendto(struct.pack("<BBHI", ord("w"), port, 0, value & 0xFFFFFFFF), self.emulator_path)

    def read(self, port: int) -> int | None:
        self.sock.sendto(struct.pack("<BBHI", ord("r"), port, 0, 0), self.emulator_path)
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            ready, _, _ = select.select([self.sock], [], [], 0.05)
            if not ready:
                continue
            data = self.sock.recv(64)
            if len(data) < 4:
                return None
            return struct.unpack("<I", data[:4])[0]
        raise TimeoutError("the emulator never answered a host read")

    # ---- the pieces a mailbox needs -------------------------------------------------------
    def put_words(self, address: int, words: list[int]) -> None:
        """Post words through HPID with autoincrementing, then flush them with an HPIA write."""
        self.write(HPIA, address >> 2)
        for word in words:
            self.write(HPID_AUTO, word)
        # Writing an address register is a flush condition, so nothing stays in the FIFO.
        self.write(HPIA, address >> 2)

    def get_words(self, address: int, count: int) -> list[int]:
        self.write(HPIA, address >> 2)
        return [self.read(HPID_AUTO) for _ in range(count)]

    def get_word(self, address: int) -> int | None:
        """One word through HPID without autoincrementing, which prefetches nothing beyond it:
        the way to read a register whose neighbours a read could disturb."""
        self.write(HPIA, address >> 2)
        return self.read(HPID)

    def get_bytes(self, address: int, count: int, byteorder: str) -> bytes:
        base = address & ~3
        words = self.get_words(base, (address - base + count + 3) // 4)
        if any(word is None for word in words):
            raise RuntimeError("HPI was not ready during a memory read")
        data = b"".join(word.to_bytes(4, byteorder) for word in words)
        return data[address - base : address - base + count]

    def put_bytes(self, address: int, data: bytes, byteorder: str) -> None:
        """Preserve bytes outside an unaligned segment and flush even a partial FIFO."""
        if not data:
            return
        base = address & ~3
        end = (address + len(data) + 3) & ~3
        prefix = self.get_bytes(base, address - base, byteorder) if address != base else b""
        suffix = (
            self.get_bytes(address + len(data), end - address - len(data), byteorder)
            if end != address + len(data) else b""
        )
        padded = prefix + data + suffix
        self.put_words(base, [
            int.from_bytes(padded[index : index + 4], byteorder)
            for index in range(0, len(padded), 4)
        ])

    def raise_dspint(self) -> None:
        self.write(HPIC, HPIC_DSPINT)

    def wait_for_hint(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            control = self.read(HPIC)
            if control is not None and control & HPIC_HINT:
                self.write(HPIC, HPIC_HINT)  # write one to clear, acknowledging the CPU
                return True
        return False


def load_boot_image(path: str, device: str = "c6455") -> tuple[Image, str]:
    """The ELF's PT_LOADs with their zero-fill tails, and the byte order to write them in.

    A C6455 boot always starts at L2 base. A C6472 core starts wherever its DSP_BOOT_ADDRn says,
    which holds only the 22 most significant bits (SPRS612G section 3.9.5), so the entry must be
    1 KB aligned, and it is a local L2 address.
    """
    elf = parse_elf(path)
    if device == "c6455":
        if elf.entry != L2_BASE:
            raise BootError(f"{path}: HPI boot requires ELF entry 0x{L2_BASE:08x}, got 0x{elf.entry:08x}")
    else:
        l2_size = LOCAL_WINDOWS[0][1]
        if not L2_BASE <= elf.entry < L2_BASE + l2_size or elf.entry & 0x3FF:
            raise BootError(
                f"{path}: C6472 HPI boot needs a 1 KB-aligned ELF entry in local L2 "
                f"(0x{L2_BASE:08x}-0x{L2_BASE + l2_size - 1:08x}), got 0x{elf.entry:08x}"
            )
    image = elf.image(0, zero_fill=True)
    if any(segment.address + len(segment.data) > 0x1_0000_0000 for segment in image.segments):
        raise BootError(f"{path}: invalid PT_LOAD memory range")
    if not any(
        program.kind == PT_LOAD and program.address <= elf.entry < program.address + program.file_size
        for program in elf.programs
    ):
        raise BootError(f"{path}: no load segment at the HPI boot entry")
    if device == "c6472":
        try:
            place_segments(image, (0,))
        except BootError as error:
            raise BootError(f"{path}: {error}") from error
    return image, elf.byteorder


def place_segments(image: Image, cores: tuple[int, ...]) -> list[tuple[int, bytes]]:
    """Where each segment goes on a C6472: a core-local segment at every selected core's global
    alias, anything else - shared L2, DDR2 - once, at the address the ELF gives.

    A segment is local only if it lies wholly in one local window. One that starts in a window
    and runs past its end is refused: its tail would not be local memory at all, and the alias
    arithmetic would put it in another core's.
    """
    placements: list[tuple[int, bytes]] = []
    for segment in image.segments:
        start, end = segment.address, segment.address + len(segment.data)
        overlapped = [(base, size) for base, size in LOCAL_WINDOWS if start < base + size and end > base]
        if not overlapped:
            placements.append((global_alias(0, start, rom_scratch=False), segment.data))
        elif any(base <= start and end <= base + size for base, size in overlapped):
            placements.extend(
                (global_alias(core, start, rom_scratch=False), segment.data) for core in cores
            )
        else:
            raise BootError(
                f"segment 0x{start:08x}-0x{end - 1:08x} is not wholly inside one core-local window"
            )
    return placements


def parse_cores(text: str) -> tuple[int, ...]:
    try:
        cores = tuple(int(piece, 0) for piece in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"--cores must be a comma-separated list: {text}") from None
    if any(not 0 <= core < C6472_CORES for core in cores):
        raise argparse.ArgumentTypeError(f"C6472 core numbers are 0 through {C6472_CORES - 1}: {text}")
    if len(set(cores)) != len(cores):
        raise argparse.ArgumentTypeError(f"--cores names a core twice: {text}")
    return tuple(sorted(cores))


def parse_dumps(text: str) -> dict[str, list[int]]:
    """The emulator's `--dump` windows from its stderr: address text to little-endian words."""
    dumps: dict[str, list[int]] = {}
    words: list[int] | None = None
    for line in text.splitlines():
        if line.startswith("dump "):
            words = dumps.setdefault(line[5:].rstrip(":"), [])
        elif words is not None and line.startswith("  0x"):
            words.extend(int(word, 16) for word in line.split()[1:])
        else:
            words = None
    return dumps


def start_cores(host: Host, entry: int, cores: tuple[int, ...], release: bool) -> None:
    """Give each core its start address, then (when `release`) take them out of reset together.

    SPRUEC6G section 4.3.2: the host writes DSP_BOOT_ADDRn (address >> 10) and sets BCn in
    BOOT_COMPLETE_STAT; the 0 to 1 edge on a core's bit is what releases it. Read the boot
    addresses back before release. Do not read after setting BCn: the application may reach
    its exit before the next host poll.
    """
    for core in cores:
        register = DSP_BOOT_ADDR0 + DSP_BOOT_ADDR_STRIDE * core
        host.put_words(register, [entry >> 10])
        if host.get_word(register) != entry >> 10:
            raise RuntimeError(f"DSP_BOOT_ADDR{core} did not take 0x{entry >> 10:x}")
    if release:
        mask = sum(1 << core for core in cores)
        host.put_words(BOOT_COMPLETE_STAT, [mask])


def emulator_failure(state_path: str, returncode: int, require_exit: bool) -> str | None:
    """Why the emulator's own run was not sound, or None.

    Its exit status is the application's exit code, which is not a verdict (sum_loop exits 55),
    so the machine state decides: a fault, no state at all, or, when the application was
    released, a run that never reached its exit.
    """
    try:
        with open(state_path) as state_file:
            state = json.load(state_file)
    except (OSError, ValueError):
        return f"emulator exited {returncode} without writing its machine state"
    if state.get("fault"):
        return f"emulator faulted (exit {returncode}): {state['fault']}"
    if require_exit and not state.get("halted"):
        return f"emulator exited {returncode} before the application reached its exit"
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--emulator", default="target/release/c64xemu")
    parser.add_argument("--image", required=True)
    parser.add_argument("--max-cycles", type=int, default=400_000_000)
    parser.add_argument("--hpi-poll", type=int, default=256)
    parser.add_argument("--exchanges", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--report", action="store_true")
    parser.add_argument("--boot", action="store_true", help="load the ELF through HPI in boot mode 1")
    parser.add_argument("--device", choices=("c6455", "c6472"), default="c6455")
    parser.add_argument("--no-dspint", action="store_true", help="C6455: load and verify, but leave boot held")
    parser.add_argument(
        "--cores", type=parse_cores, help="C6472: cores to boot with --image, e.g. 0,3 (default all six)"
    )
    parser.add_argument(
        "--no-release", action="store_true",
        help="C6472: load and set the boot addresses, but leave every core held",
    )
    parser.add_argument("--dspint", action="store_true", help="C6472: also raise DSPINT, which releases no core")
    parser.add_argument("--dump", action="append", default=[], metavar="ADDR:LEN", help="print this memory when the run ends")
    parser.add_argument("--state-out", help="write the emulator's final machine state")
    arguments = parser.parse_args()
    c6472 = arguments.device == "c6472"
    for flag, given in (
        ("--no-dspint", arguments.no_dspint), ("--cores", arguments.cores is not None),
        ("--no-release", arguments.no_release), ("--dspint", arguments.dspint),
    ):
        if given and not arguments.boot:
            parser.error(f"{flag} requires --boot")
    if c6472 and not arguments.boot:
        parser.error("--device c6472 requires --boot; the mailbox firmware is the C6455's")
    if c6472 and arguments.no_dspint:
        parser.error("--no-dspint is the C6455's; a C6472 boot is released with --cores and not --no-release")
    if not c6472 and (arguments.no_release or arguments.dspint):
        parser.error("--no-release and --dspint are the C6472's; the C6455 boot is released by DSPINT")
    if not c6472 and arguments.cores not in (None, (0,)):
        parser.error("the C6455 has one core, 0")
    cores = (arguments.cores or tuple(range(C6472_CORES))) if c6472 else (0,)
    boot_image, byteorder = (
        load_boot_image(arguments.image, arguments.device) if arguments.boot else (None, "little")
    )

    directory = tempfile.mkdtemp(prefix="c64xemu-hpi-")
    emulator_socket = os.path.join(directory, "emu.sock")
    host_socket = os.path.join(directory, "host.sock")
    state_path = arguments.state_out or os.path.join(directory, "state.json")

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(host_socket)
    sock.settimeout(10.0)

    emulator = subprocess.Popen(
        [
            arguments.emulator,
            "run",
            arguments.image,
            "--max-cycles",
            str(arguments.max_cycles),
            "--hpi-unix",
            emulator_socket,
            "--hpi-peer",
            host_socket,
            "--hpi-poll",
            str(arguments.hpi_poll),
            *(["--boot-mode", "1"] if arguments.boot else []),
            "--device",
            arguments.device,
            *[argument for window in arguments.dump for argument in ("--dump", window)],
            "--state-out",
            state_path,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    deadline = time.monotonic() + arguments.timeout
    while not os.path.exists(emulator_socket):
        if time.monotonic() > deadline or emulator.poll() is not None:
            print("emulator never bound its socket", file=sys.stderr)
            return 1
        time.sleep(0.05)

    host = Host(sock, emulator_socket, timeout=10.0)
    exchanges: list[dict[str, int | bool]] = []
    boot: dict | None = None
    status = 1
    released = boot_image is not None and not (arguments.no_dspint or arguments.no_release)
    try:
        # Strapped to host boot, the HPI is open from reset. Otherwise the firmware clears
        # HPIRST once it is ready, and until then HPIC reads have no answer.
        while time.monotonic() < deadline:
            control = host.read(HPIC)
            if control is not None and control & HPIC_HRDY:
                break
            time.sleep(0.05)
        else:
            raise TimeoutError("HPI never became ready")

        if boot_image is not None:
            placements = (
                place_segments(boot_image, cores) if c6472
                else [(segment.address, segment.data) for segment in boot_image.segments]
            )
            transferred = 0
            for address, data in placements:
                host.put_bytes(address, data, byteorder)
                if host.get_bytes(address, len(data), byteorder) != data:
                    raise RuntimeError(f"HPI readback differs at 0x{address:08x}")
                transferred += len(data)
            boot = {
                "entry": boot_image.entry,
                "bytes": transferred,
                "segments": len(boot_image.segments),
                "verified": True,
                "released": released,
                "byteorder": byteorder,
            }
            if c6472:
                boot["cores"] = list(cores)
                if arguments.dspint:
                    host.raise_dspint()
                start_cores(host, boot_image.entry, cores, released)
            elif released:
                host.raise_dspint()
            status = 0  # the host's part succeeded; the emulator's own exit is checked below

        for index in range(0 if arguments.boot else arguments.exchanges):
            payload = [(index * 0x01010101 + step * 7 + 1) & 0xFFFFFFFF for step in range(4)]
            command = COMMAND_SUM if index % 2 == 0 else COMMAND_SWAP
            expected = (
                sum(payload) & 0xFFFFFFFF
                if command == COMMAND_SUM
                else int.from_bytes(payload[0].to_bytes(4, "little"), "big")
            )
            host.put_words(PAYLOAD, payload)
            host.put_words(REQUEST, [command, len(payload), index + 1, 0])
            host.raise_dspint()
            answered = host.wait_for_hint(timeout=30.0)
            sequence, answer = host.get_words(REPLY, 2) if answered else (None, None)
            exchanges.append(
                {
                    "command": command,
                    "answered": answered,
                    "sequence": sequence,
                    "answer": answer,
                    "correct": answered and sequence == index + 1 and answer == expected,
                }
            )
            if not exchanges[-1]["correct"]:
                break
        if not arguments.boot:
            status = 0 if len(exchanges) == arguments.exchanges and all(
                exchange["correct"] for exchange in exchanges
            ) else 1
    finally:
        failure = None
        try:
            out, err = emulator.communicate(timeout=arguments.timeout if arguments.boot else 15)
        except subprocess.TimeoutExpired:
            if arguments.boot:
                status = 1
            emulator.terminate()
            out, err = emulator.communicate()
        else:
            failure = emulator_failure(state_path, emulator.returncode, require_exit=released)
            if failure is not None:
                status = 1
        sock.close()
        for path in (host_socket, emulator_socket, *([] if arguments.state_out else [state_path])):
            try:
                os.unlink(path)
            except OSError:
                pass
        os.rmdir(directory)
        firmware = [line.strip() for line in (out or "").splitlines() if line.startswith("hpi ")]
        if arguments.report:
            print(json.dumps({
                "exchanges": exchanges, "firmware": firmware, "boot": boot, "dumps": parse_dumps(err),
            }))
        else:
            if boot is not None:
                print(f"host boot: {boot}")
            for exchange in exchanges:
                print(f"host: {exchange}")
            print("--- firmware output ---")
            print("\n".join(firmware))
            if err.strip():
                print("--- emulator stderr ---", file=sys.stderr)
                print(err.strip()[-2000:], file=sys.stderr)
        if failure is not None:
            print(f"hpi host: {failure}", file=sys.stderr)
    return status


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (BootError, OSError, TimeoutError, RuntimeError) as error:
        print(f"hpi host: {error}", file=sys.stderr)
        raise SystemExit(1)
