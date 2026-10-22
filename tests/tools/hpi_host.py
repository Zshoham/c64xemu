#!/usr/bin/env python3
"""Drive the emulated C6455's host port interface from a process on the host.

The emulator exposes the HPI's host side over a Unix datagram socket: one datagram is one host
access, `[direction, port, 0, 0, value]`, where direction is `r` or `w` and port is the HCNTL
encoding from SPRU969B Table 4 - 0 HPIC, 1 HPID with autoincrement, 2 HPIA, 3 HPID without. That
is all a host needs to reach DSP memory and to exchange interrupts with the CPU.

By default this script drives the firmware mailbox. With --boot it instead loads ELF code and
data directly through the HPI (not a boot table), verifies the transferred bytes, then raises
DSPINT. C6455 host boot always starts at 0x00800000, so the ELF must place its entry there.
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
    from .emac_boot import PT_LOAD, BootError, Image, parse_elf
else:
    from emac_boot import PT_LOAD, BootError, Image, parse_elf

HPIC, HPID_AUTO, HPIA, HPID = 0, 1, 2, 3
HPIC_DSPINT = 1 << 1
HPIC_HINT = 1 << 2
HPIC_HRDY = 1 << 3
L2_BASE = 0x0080_0000

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


def load_boot_image(path: str) -> tuple[Image, str]:
    """The ELF's PT_LOADs with their zero-fill tails, and the byte order to write them in."""
    elf = parse_elf(path)
    if elf.entry != L2_BASE:
        raise BootError(f"{path}: HPI boot requires ELF entry 0x{L2_BASE:08x}, got 0x{elf.entry:08x}")
    image = elf.image(0, zero_fill=True)
    if any(segment.address + len(segment.data) > 0x1_0000_0000 for segment in image.segments):
        raise BootError(f"{path}: invalid PT_LOAD memory range")
    if not any(
        program.kind == PT_LOAD and program.address <= L2_BASE < program.address + program.file_size
        for program in elf.programs
    ):
        raise BootError(f"{path}: no load segment at the HPI boot entry")
    return image, elf.byteorder


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
    parser.add_argument("--no-dspint", action="store_true", help="load and verify, but leave boot held")
    parser.add_argument("--state-out", help="write the emulator's final machine state")
    arguments = parser.parse_args()
    if arguments.no_dspint and not arguments.boot:
        parser.error("--no-dspint requires --boot")
    boot_image, byteorder = load_boot_image(arguments.image) if arguments.boot else (None, "little")

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
    released = boot_image is not None and not arguments.no_dspint
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
            transferred = 0
            for segment in boot_image.segments:
                host.put_bytes(segment.address, segment.data, byteorder)
                if host.get_bytes(segment.address, len(segment.data), byteorder) != segment.data:
                    raise RuntimeError(f"HPI readback differs at 0x{segment.address:08x}")
                transferred += len(segment.data)
            boot = {
                "entry": boot_image.entry,
                "bytes": transferred,
                "segments": len(boot_image.segments),
                "verified": True,
                "released": not arguments.no_dspint,
                "byteorder": byteorder,
            }
            if not arguments.no_dspint:
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
            print(json.dumps({"exchanges": exchanges, "firmware": firmware, "boot": boot}))
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
