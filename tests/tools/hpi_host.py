#!/usr/bin/env python3
"""Drive the emulated C6455's host port interface from a process on the host.

The emulator exposes the HPI's host side over a Unix datagram socket: one datagram is one host
access, `[direction, port, 0, 0, value]`, where direction is `r` or `w` and port is the HCNTL
encoding from SPRU969B Table 4 - 0 HPIC, 1 HPID with autoincrement, 2 HPIA, 3 HPID without. That
is all a host needs to reach DSP memory and to exchange interrupts with the CPU.

This script is the host half of a mailbox: it writes a request block into DSP memory, raises
DSPINT, waits for the firmware to answer with HINT, clears it and reads the reply. Every answer is
checked, so the exit status is the test result.
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

HPIC, HPID_AUTO, HPIA, HPID = 0, 1, 2, 3
HPIC_DSPINT = 1 << 1
HPIC_HINT = 1 << 2
HPIC_HRDY = 1 << 3

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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--emulator", default="target/release/c64xemu")
    parser.add_argument("--image", required=True)
    parser.add_argument("--max-cycles", type=int, default=400_000_000)
    parser.add_argument("--hpi-poll", type=int, default=256)
    parser.add_argument("--exchanges", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--report", action="store_true")
    arguments = parser.parse_args()

    directory = tempfile.mkdtemp(prefix="c64xemu-hpi-")
    emulator_socket = os.path.join(directory, "emu.sock")
    host_socket = os.path.join(directory, "host.sock")

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(host_socket)
    sock.setblocking(False)

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
    status = 1
    try:
        # The firmware clears HPIRST once it is ready; until then every host access is held off,
        # which a read of HPIC reports as no answer at all.
        while time.monotonic() < deadline:
            control = host.read(HPIC)
            if control is not None and control & HPIC_HRDY:
                break
            time.sleep(0.05)

        for index in range(arguments.exchanges):
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
        status = 0 if len(exchanges) == arguments.exchanges and all(
            exchange["correct"] for exchange in exchanges
        ) else 1
    finally:
        try:
            out, err = emulator.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            emulator.terminate()
            out, err = emulator.communicate()
        sock.close()
        for path in (host_socket, emulator_socket):
            try:
                os.unlink(path)
            except OSError:
                pass
        os.rmdir(directory)
        firmware = [line.strip() for line in (out or "").splitlines() if line.startswith("hpi ")]
        if arguments.report:
            print(json.dumps({"exchanges": exchanges, "firmware": firmware}))
        else:
            for exchange in exchanges:
                print(f"host: {exchange}")
            print("--- firmware output ---")
            print("\n".join(firmware))
            if err.strip():
                print("--- emulator stderr ---", file=sys.stderr)
                print(err.strip()[-2000:], file=sys.stderr)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
