#!/usr/bin/env python3
"""Pace Ethernet frames at all six cores of the emulated C6472 at once and check every echo.

The firmware in `tests/firmware/c6472_netecho` gives each core one receive channel of the
device's shared MAC and one unicast address, 02:00:00:64:72:0N for core N, and echoes what
arrives back to the sender. This script is the other end of that wire: it launches the emulator,
waits for all six cores to announce that their channels are armed, then sends a paced stream to
each of the six addresses at the configured rate and requires every byte back, from the right
core, in order.

The pacer is the same one the single-core bandwidth cases use - bytes are admitted as elapsed
wall time earns them, and the exchange has to finish within one and a half times the configured
duration - so passing is the claim that the emulator sustained that rate on six cores at once.
Nothing here knows anything about the emulator's internals; the only interface is the Unix
datagram socket carrying one raw Ethernet frame per datagram.
"""

from __future__ import annotations

import argparse
import json
import os
import select
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

# The local experimental type the firmware answers on, and the payload header it echoes back:
# core number, a reserved byte, then a 16-bit sequence number.
ETHER_TYPE = 0x88B5
HEADER = 4
# The announcement sequence a core sends once, from a broadcast frame, when its channel is up.
HELLO_SEQUENCE = 0xFFFF
# Data bytes per frame: 496 keeps the whole frame at 514 bytes, comfortably inside the 1518 the
# firmware programs into RXMAXLEN and well above the 60-byte minimum.
CHUNK = 496
PEER_MAC = b"\x02\x00\x00\x44\x55\x66"


def core_mac(core: int) -> bytes:
    """The address core `core` answers to, built by the same rule as the firmware's."""
    return bytes([0x02, 0x00, 0x00, 0x64, 0x72, core])


def pattern(core: int, length: int) -> bytes:
    """A deterministic per-core byte stream, so a misdirected echo cannot pass as a correct one."""
    return bytes(((index * 31 + core * 7 + 5) & 0xFF) for index in range(length))


class Wire:
    """The Unix datagram socket, as one raw Ethernet frame per datagram."""

    def __init__(self, sock: socket.socket, emulator_path: str):
        self.sock = sock
        self.emulator_path = emulator_path

    def send(self, destination: bytes, payload: bytes) -> None:
        frame = destination + PEER_MAC + struct.pack("!H", ETHER_TYPE) + payload
        if len(frame) < 60:
            frame += b"\x00" * (60 - len(frame))
        self.sock.sendto(frame, self.emulator_path)

    def poll(self, timeout: float) -> list[bytes]:
        ready, _, _ = select.select([self.sock], [], [], timeout)
        frames = []
        while ready:
            try:
                frames.append(self.sock.recv(65535))
            except BlockingIOError:
                break
            ready, _, _ = select.select([self.sock], [], [], 0)
        return frames


class CoreStream:
    """One core's half of the exchange: what has been sent to it and what has come back."""

    def __init__(self, core: int, payload: bytes):
        self.core = core
        self.mac = core_mac(core)
        self.payload = payload
        self.sent = 0
        self.echoed = 0
        self.frames_sent = 0
        self.frames_echoed = 0
        self.ready = False
        self.mismatch: str | None = None
        self.next_sequence = 0

    def send_next(self, wire: Wire) -> int:
        """Send this core's next frame, returning how many data bytes it carried."""
        piece = self.payload[self.sent : self.sent + CHUNK]
        header = struct.pack("!BBH", self.core, 0, self.frames_sent & 0xFFFF)
        wire.send(self.mac, header + piece)
        self.sent += len(piece)
        self.frames_sent += 1
        return len(piece)

    def receive(self, source: bytes, payload: bytes) -> None:
        """Check one echoed frame against what was sent, in order."""
        if self.mismatch is not None:
            return
        if source != self.mac:
            self.mismatch = f"core {self.core} echo came from {source.hex(':')}"
            return
        if len(payload) < HEADER:
            self.mismatch = f"core {self.core} echo carried {len(payload)} bytes"
            return
        core, _, sequence = struct.unpack("!BBH", payload[:HEADER])
        if core != self.core:
            self.mismatch = f"core {self.core} echoed a frame marked core {core}"
            return
        if sequence != (self.next_sequence & 0xFFFF):
            self.mismatch = (
                f"core {self.core} echoed sequence {sequence}, expected {self.next_sequence}"
            )
            return
        data = payload[HEADER:]
        expected = self.payload[self.echoed : self.echoed + len(data)]
        # A frame the peer padded to 60 bytes comes back padded; only the bytes that were sent
        # are checked.
        if data[: len(expected)] != expected:
            self.mismatch = f"core {self.core} echoed different bytes at offset {self.echoed}"
            return
        self.echoed += len(expected)
        self.frames_echoed += 1
        self.next_sequence += 1

    @property
    def complete(self) -> bool:
        return self.mismatch is None and self.echoed >= len(self.payload)

    def report(self) -> dict:
        return {
            "core": self.core,
            "ready": self.ready,
            "sent": self.sent,
            "echoed": self.echoed,
            "frames_sent": self.frames_sent,
            "frames_echoed": self.frames_echoed,
            "matched": self.mismatch is None,
            "mismatch": self.mismatch,
        }


def wait_for_cores(wire: Wire, streams: list[CoreStream], deadline: float) -> bool:
    """Every core announces itself once its channel is armed; nothing is paced before that."""
    while time.monotonic() < deadline:
        for frame in wire.poll(0.02):
            parsed = parse(frame)
            if parsed is None:
                continue
            source, payload = parsed
            core, _, sequence = struct.unpack("!BBH", payload[:HEADER])
            if sequence != HELLO_SEQUENCE or core >= len(streams):
                continue
            if source == streams[core].mac:
                streams[core].ready = True
        if all(stream.ready for stream in streams):
            return True
    return False


def parse(frame: bytes) -> tuple[bytes, bytes] | None:
    """The source address and payload of a frame of ours, or None for anything else."""
    if len(frame) < 14 + HEADER:
        return None
    if struct.unpack("!H", frame[12:14])[0] != ETHER_TYPE:
        return None
    return frame[6:12], frame[14:]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--emulator", default="target/release/c64xemu")
    parser.add_argument("--image", required=True)
    parser.add_argument("--cores", type=int, default=6)
    parser.add_argument("--max-cycles", type=int, default=1_200_000_000)
    parser.add_argument("--net-poll", type=int, default=512)
    parser.add_argument(
        "--boot-mode",
        default="0",
        help="the BOOTMODE pins: 0 starts every core, a ROM mode starts only core 0",
    )
    parser.add_argument(
        "--bandwidth-kbps",
        type=int,
        default=512,
        help="pace each core at this many kilobits per second of payload (wall clock)",
    )
    parser.add_argument(
        "--duration", type=float, default=1.0, help="seconds to send at the configured rate"
    )
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument(
        "--ready-timeout",
        type=float,
        default=30.0,
        help="seconds to wait for every core to announce its channel before giving up",
    )
    parser.add_argument(
        "--report", action="store_true", help="print one JSON line describing the exchange"
    )
    arguments = parser.parse_args()

    directory = tempfile.mkdtemp(prefix="c64xemu-multicore-")
    emulator_socket = os.path.join(directory, "emu.sock")
    peer_socket = os.path.join(directory, "peer.sock")

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(peer_socket)
    sock.setblocking(False)

    emulator = subprocess.Popen(
        [
            arguments.emulator,
            "run",
            arguments.image,
            "--device",
            "c6472",
            "--boot-mode",
            arguments.boot_mode,
            "--max-cycles",
            str(arguments.max_cycles),
            "--net-unix",
            emulator_socket,
            "--net-peer",
            peer_socket,
            "--net-poll",
            str(arguments.net_poll),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    bytes_per_core = int(arguments.bandwidth_kbps * 125.0 * arguments.duration)
    streams = [CoreStream(core, pattern(core, bytes_per_core)) for core in range(arguments.cores)]
    wire = Wire(sock, emulator_socket)
    state = "starting"
    try:
        deadline = time.monotonic() + arguments.timeout
        while not os.path.exists(emulator_socket):
            if time.monotonic() > deadline or emulator.poll() is not None:
                print("emulator never bound its socket", file=sys.stderr)
                state = "no-socket"
                raise SystemExit(1)
            time.sleep(0.02)

        ready_deadline = min(deadline, time.monotonic() + arguments.ready_timeout)
        if not wait_for_cores(wire, streams, ready_deadline):
            state = "cores-never-ready"
        else:
            state = "running"
            bytes_per_second = arguments.bandwidth_kbps * 125.0
            started = time.monotonic()
            # The deadline the single-core paced cases use: the whole exchange, echoes included,
            # has to finish inside one and a half times the time the sender was given.
            send_deadline = started + arguments.duration * 1.5
            while True:
                now = time.monotonic()
                # One frame of credit up front so the first frames go out immediately.
                allowed = int((now - started) * bytes_per_second) + CHUNK
                for stream in streams:
                    while stream.sent < len(stream.payload) and stream.sent < allowed:
                        stream.send_next(wire)
                for frame in wire.poll(0.002):
                    parsed = parse(frame)
                    if parsed is None:
                        continue
                    source, payload = parsed
                    core = payload[0]
                    if core < len(streams):
                        streams[core].receive(source, payload)
                if all(stream.complete for stream in streams):
                    state = "closed"
                    break
                if any(stream.mismatch is not None for stream in streams):
                    state = "mismatch"
                    break
                if now > send_deadline:
                    state = "failed"
                    break
    finally:
        emulator.terminate()
        try:
            out, err = emulator.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            emulator.kill()
            out, err = emulator.communicate()
        sock.close()
        report = {
            "state": state,
            "sent": sum(stream.sent for stream in streams),
            "echoed": sum(stream.echoed for stream in streams),
            "expected": sum(len(stream.payload) for stream in streams),
            "cores": [stream.report() for stream in streams],
        }
        if arguments.report:
            print(json.dumps(report))
        else:
            print(json.dumps(report, indent=2))
            if err.strip():
                print("--- emulator stderr ---", file=sys.stderr)
                print(err.strip()[-4000:], file=sys.stderr)
            if out.strip():
                print("--- emulator stdout ---", file=sys.stderr)
                print(out.strip()[-4000:], file=sys.stderr)
        for path in (peer_socket, emulator_socket):
            try:
                os.unlink(path)
            except OSError:
                pass
        shutil.rmtree(directory, ignore_errors=True)
    return 0 if state == "closed" else 1


if __name__ == "__main__":
    sys.exit(main())
