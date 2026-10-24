#!/usr/bin/env python3
"""Send a C6472 boot image as UTOPIA cells (SPRUEC6G section 4.3.4).

The emulator's positional ELF supplies only symbols. This independent ATM master sends the
actual load segments through PHY-addressed, flow-controlled cell writes over a Unix socket.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import sys
import time

if __package__:
    from .emac_boot import BootError, Section, assemble_table, parse_elf, positive_float
else:
    from emac_boot import BootError, Section, assemble_table, parse_elf, positive_float

MAGIC = bytes.fromhex("54492164")


def cell(payload: bytes, width: int, size: int | None = None) -> bytes:
    """A physical cell: 5/6 header bytes plus any user-defined ones up to `size`, 32 boot bytes,
    magic, and unused payload. The payload is always a cell's last 48 bytes."""
    size = size or (53 if width == 8 else 54)
    return bytes(size - 48) + payload.ljust(32, b"\0") + MAGIC + bytes(12)


def exchange(sock: socket.socket, peer: str, request: bytes, deadline: float) -> bytes:
    # Only connection establishment is retried: retrying an unanswered write could duplicate
    # an accepted cell. A negative write reply, unlike a timeout, explicitly means no transfer.
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise BootError("timed out waiting for the UTOPIA master connection")
        sock.settimeout(remaining)
        try:
            sock.sendto(request, peer)
            # The longest reply is a cell read back: at most 64 wire bytes.
            return sock.recv(64)
        except (FileNotFoundError, ConnectionRefusedError):
            time.sleep(min(0.01, remaining))


def send(sock: socket.socket, peer: str, phy: int, data: bytes, deadline: float) -> int:
    refused = 0
    while True:
        reply = exchange(sock, peer, bytes((ord("w"), phy)) + data, deadline)
        if reply == b"\x01":
            return refused
        if reply != b"\x00":
            raise BootError(f"invalid UTOPIA write reply: {reply.hex()}")
        refused += 1
        if time.monotonic() >= deadline:
            raise BootError("timed out waiting for UTOPIA receive capacity/address selection")
        time.sleep(0.001)


def receive(sock: socket.socket, peer: str, phy: int, deadline: float) -> bytes:
    """Poll UXCLAV at `phy` until the slave has a cell, then take it off the wire."""
    while exchange(sock, peer, bytes((ord("x"), phy)), deadline) != b"\x01":
        if time.monotonic() >= deadline:
            raise BootError("timed out waiting for a cell from the UTOPIA slave")
        time.sleep(0.001)
    cell = exchange(sock, peer, bytes((ord("r"), phy)), deadline)
    if len(cell) < 53:
        raise BootError(f"invalid UTOPIA read reply: {cell.hex()}")
    return cell


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True, type=Path)
    parser.add_argument("--peer", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--phy", type=int, choices=range(31), default=0)
    parser.add_argument("--width", type=int, choices=(8, 16), default=8)
    parser.add_argument("--cell-size", type=int, choices=range(53, 65),
                        help="the physical cell size the boot parameters name (default 53/54)")
    parser.add_argument("--timeout", type=positive_float, default=60)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    bound = False
    try:
        elf = parse_elf(args.image)
        image = elf.image(0, zero_fill=True)
        sections = []
        for segment in image.segments:
            end = segment.address + len(segment.data)
            # The ROM uses the addressable half of L1D and its documented L2 scratch.
            for start, stop in ((0x00F00000, 0x00F04000), (0x008969C0, 0x00898000)):
                address = segment.address
                if 0x10000000 <= address < 0x11000000:
                    address -= 0x10000000
                if address < stop and address + len(segment.data) > start:
                    raise BootError("image overlaps core 0's UTOPIA boot workspace")
            if 0x00898000 < end <= 0x00A00000:
                raise BootError("image exceeds C6472 local L2 SRAM")
            sections.append(Section(segment.address, segment.data))
        table = assemble_table(image.entry, sections, elf.endian == ">")
        if elf.endian != ">":
            # The ROM assembles cells big-endian, so on a little-endian device each wire word
            # reaches L1D turned around: the table travels as `hex6x -order M` writes it, every
            # word most significant byte first.
            table = b"".join(table[at:at + 4][::-1] for at in range(0, len(table), 4))
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.bind(str(args.socket))
            bound = True
            deadline = time.monotonic() + args.timeout
            count = refused = 0
            for offset in range(0, len(table), 32):
                refused += send(sock, args.peer, args.phy,
                                cell(table[offset:offset + 32], args.width, args.cell_size),
                                deadline)
                count += 1
        report = {"cells": count, "table_bytes": count * 32,
                  "entry": image.entry, "refused": refused}
        print(json.dumps(report) if args.json else
              f"sent {count} cells; entry 0x{image.entry:08x}; {refused} refused transfers")
        return 0
    except (BootError, OSError) as error:
        print(f"utopia_boot: {error}", file=sys.stderr)
        return 1
    finally:
        if bound:
            args.socket.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
