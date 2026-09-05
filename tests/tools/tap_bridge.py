#!/usr/bin/env python3
"""Relay Ethernet frames between a host TAP interface and the emulator's Unix socket.

The emulator deliberately knows nothing about TAP devices: it exchanges one raw Ethernet frame
per Unix datagram (see src/netio.rs), and this helper owns /dev/net/tun. That keeps the
privileged part - creating or opening the TAP - outside the emulator, and lets a test script take
this program's place to drive a deterministic exchange.

Usage:
    tools/tap_bridge.py --tap dsp0 --socket /tmp/c64xemu.sock [--peer /tmp/tap.sock]

The emulator is then started with:
    c64xemu run IMAGE --net-unix /tmp/c64xemu.sock --net-peer /tmp/tap.sock --max-cycles 0

A persistent TAP owned by your user needs no privileges here; create it once with:
    sudo ip tuntap add dev dsp0 mode tap user "$USER"
    sudo ip addr add 192.168.64.1/24 dev dsp0
    sudo ip link set dsp0 up
"""

from __future__ import annotations

import argparse
import fcntl
import os
import select
import socket
import struct
import sys

TUNSETIFF = 0x400454CA
IFF_TAP = 0x0002
IFF_NO_PI = 0x1000
MAX_FRAME = 9216


def open_tap(name: str) -> int:
    fd = os.open("/dev/net/tun", os.O_RDWR)
    request = struct.pack("16sH22s", name.encode(), IFF_TAP | IFF_NO_PI, b"")
    fcntl.ioctl(fd, TUNSETIFF, request)
    os.set_blocking(fd, False)
    return fd


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tap", required=True, help="TAP interface name, e.g. dsp0")
    parser.add_argument("--socket", required=True, help="the emulator's socket path")
    parser.add_argument("--peer", required=True, help="this helper's own socket path")
    parser.add_argument("--verbose", action="store_true", help="log every frame")
    options = parser.parse_args()

    tap = open_tap(options.tap)
    if os.path.exists(options.peer):
        os.unlink(options.peer)
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(options.peer)
    sock.setblocking(False)

    to_emulator = 0
    to_host = 0
    print(f"bridging {options.tap} <-> {options.socket}", file=sys.stderr)
    try:
        while True:
            readable, _, _ = select.select([tap, sock], [], [], 1.0)
            if tap in readable:
                try:
                    frame = os.read(tap, MAX_FRAME)
                except BlockingIOError:
                    frame = b""
                if frame:
                    to_emulator += 1
                    if options.verbose:
                        print(f"host -> dsp {len(frame)}B", file=sys.stderr)
                    try:
                        sock.sendto(frame, options.socket)
                    except (FileNotFoundError, ConnectionRefusedError):
                        pass
            if sock in readable:
                try:
                    frame, _ = sock.recvfrom(MAX_FRAME)
                except BlockingIOError:
                    frame = b""
                if frame:
                    to_host += 1
                    if options.verbose:
                        print(f"dsp -> host {len(frame)}B", file=sys.stderr)
                    os.write(tap, frame)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"frames: host->dsp {to_emulator}, dsp->host {to_host}", file=sys.stderr)
        sock.close()
        os.close(tap)
        if os.path.exists(options.peer):
            os.unlink(options.peer)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
