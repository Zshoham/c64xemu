#!/usr/bin/env python3
"""Send C6472 ROM Ethernet-boot tables over the emulator's Unix frame transport.

The C6472 ROM first emits the Ethernet-ready BOOTP request described by SPRUEC6G section
4.3.5.2. This helper learns the DSP MAC from that request, converts TI C6000 ELF load segments
into the boot-table format from section 6.2.2, and carries the table in the sequenced UDP frames
specified by section 4.3.5.3. One Unix datagram is one raw Ethernet frame, matching the
emulator's ``--net-unix`` interface without requiring a host TAP device or network privileges.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass

EM_TI_C6000 = 140
PT_LOAD = 1
ETH_IPV4 = 0x0800
IP_UDP = 17
BOOTP_SERVER_PORT = 67
BOOTP_CLIENT_PORT = 68
BOOT_MAGIC = 0x544B
BOOT_OPCODE = 0x01
BOOT_FILE_PREFIX = b"ti-boot-table-"
BOOT_COMPLETE_STAT = 0x02AB_0004
DSP_BOOT_ADDR0 = 0x02AB_0204
MAX_BOOT_PAYLOAD = 1472 - 4
MAX_FRAME = 65535

LOCAL_WINDOWS = (
    (0x0080_0000, 608 * 1024),  # L2
    (0x00E0_0000, 32 * 1024),  # L1P
    (0x00F0_0000, 32 * 1024),  # L1D
)
# The C6455's L2 window is 2 MB and the C6472's is 608 KB, at the same address. A section past
# the end of the C6472's L2 is not a local address at all, so it would be sent to a global
# address every core shares instead of the core it was meant for: refuse it and say why.
C6455_L2_END = 0x00A0_0000
# What core 0's boot code keeps for itself while it runs: the receive ring and its ten packet
# buffers, inside the L2 scratch SPRUEC6G section 4.1 reserves for the boot.
BOOT_SCRATCH = (0x0089_1000, 0x0089_8000)


class BootError(Exception):
    """A user-facing failure whose message is sufficient on stderr."""


class BootArgumentParser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise BootError(message)


@dataclass(frozen=True)
class Segment:
    address: int
    data: bytes


@dataclass(frozen=True)
class Image:
    core: int
    entry: int
    segments: tuple[Segment, ...]


@dataclass(frozen=True)
class Section:
    destination: int
    data: bytes


def checksum(data: bytes) -> int:
    if len(data) % 2:
        data += b"\x00"
    total = 0
    for index in range(0, len(data), 2):
        total += (data[index] << 8) | data[index + 1]
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def mac_text(mac: bytes) -> str:
    return ":".join(f"{byte:02x}" for byte in mac)


def parse_u32(text: str) -> int:
    try:
        value = int(text, 0)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid 32-bit value: {text}") from error
    if not 0 <= value <= 0xFFFF_FFFF:
        raise argparse.ArgumentTypeError(f"32-bit value out of range: {text}")
    return value


def parse_core(text: str) -> tuple[int, str]:
    core_text, separator, path = text.partition(":")
    if not separator or not path:
        raise argparse.ArgumentTypeError("--core must be N:IMAGE")
    try:
        core = int(core_text, 10)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid core index: {core_text}") from error
    if not 0 <= core <= 5:
        raise argparse.ArgumentTypeError("C6472 core index must be between 0 and 5")
    return core, path


def parse_word(text: str) -> tuple[int, int]:
    address_text, separator, value_text = text.partition("=")
    if not separator or not address_text or not value_text:
        raise argparse.ArgumentTypeError("--word must be ADDR=VALUE")
    try:
        return parse_u32(address_text), parse_u32(value_text)
    except argparse.ArgumentTypeError as error:
        raise argparse.ArgumentTypeError(f"invalid --word {text}: {error}") from error


def parse_mac(text: str) -> bytes:
    pieces = text.split(":")
    if len(pieces) != 6 or any(len(piece) != 2 for piece in pieces):
        raise argparse.ArgumentTypeError(f"invalid MAC address: {text}")
    try:
        return bytes(int(piece, 16) for piece in pieces)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid MAC address: {text}") from error


def parse_ip(text: str) -> bytes:
    try:
        packed = socket.inet_pton(socket.AF_INET, text)
    except OSError as error:
        raise argparse.ArgumentTypeError(f"invalid IPv4 address: {text}") from error
    return packed


def parse_port(text: str) -> int:
    try:
        port = int(text, 0)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid UDP port: {text}") from error
    if not 0 <= port <= 0xFFFF:
        raise argparse.ArgumentTypeError(f"UDP port out of range: {text}")
    return port


def positive_multiple_of_four(text: str) -> int:
    try:
        value = int(text, 0)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid payload size: {text}") from error
    if value <= 0 or value % 4:
        raise argparse.ArgumentTypeError("--payload must be a positive multiple of 4")
    return value


def nonnegative_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid nonnegative number: {text}") from error
    if not math.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError(f"value must be finite and nonnegative: {text}")
    return value


def positive_float(text: str) -> float:
    value = nonnegative_float(text)
    if value == 0:
        raise argparse.ArgumentTypeError(f"value must be positive: {text}")
    return value


def load_elf(core: int, path: str) -> Image:
    try:
        with open(path, "rb") as elf_file:
            data = elf_file.read()
    except OSError as error:
        raise BootError(f"{path}: {error}") from error

    def invalid(message: str) -> BootError:
        return BootError(f"{path}: {message}")

    if len(data) < 52 or data[:4] != b"\x7fELF":
        raise invalid("not an ELF file")
    if data[4] != 1:
        raise invalid("only ELF32 images are supported")
    if data[5] == 1:
        endian = "<"
    elif data[5] == 2:
        endian = ">"
    else:
        raise invalid("unsupported ELF byte order")
    if data[6] != 1:
        raise invalid("unsupported ELF identification version")

    header = struct.unpack_from(endian + "HHIIIIIHHHHHH", data, 16)
    machine, version, entry, program_offset = header[1], header[2], header[3], header[4]
    program_size, program_count = header[8], header[9]
    if machine != EM_TI_C6000:
        raise invalid("ELF image is not for TI C6000 (e_machine must be 140)")
    if version != 1:
        raise invalid("unsupported ELF version")
    if program_count and program_size < 32:
        raise invalid("invalid ELF program-header size")

    segments = []
    for index in range(program_count):
        offset = program_offset + index * program_size
        if offset + 32 > len(data):
            raise invalid("program-header table lies outside file")
        fields = struct.unpack_from(endian + "IIIIIIII", data, offset)
        kind, file_offset, virtual_address, physical_address, file_size, memory_size = fields[:6]
        if kind != PT_LOAD or file_size == 0:
            continue
        if file_size > memory_size:
            raise invalid(f"program header {index} has p_filesz greater than p_memsz")
        if file_offset + file_size > len(data):
            raise invalid(f"program header {index} data lies outside file")
        address = physical_address if physical_address else virtual_address
        segments.append(Segment(address, data[file_offset : file_offset + file_size]))
    return Image(core, entry, tuple(segments))


def global_alias(core: int, address: int) -> int:
    for base, size in LOCAL_WINDOWS:
        if base <= address < base + size:
            if core == 0 and BOOT_SCRATCH[0] <= address < BOOT_SCRATCH[1]:
                raise BootError(
                    f"0x{address:08x} is inside the L2 the boot code is using for its receive "
                    "ring; link core 0's image below "
                    f"0x{BOOT_SCRATCH[0]:08x}"
                )
            return 0x1000_0000 + core * 0x0100_0000 + address
    l2_base, l2_size = LOCAL_WINDOWS[0]
    if l2_base + l2_size <= address < C6455_L2_END:
        raise BootError(
            f"0x{address:08x} is past the {l2_size // 1024} KB of local L2 a C6472 core has; "
            "relink the image for this device rather than the C6455's 2 MB window"
        )
    return address


def parse_announcement(frame: bytes) -> tuple[bytes, str] | None:
    if len(frame) < 14 + 20 or struct.unpack("!H", frame[12:14])[0] != ETH_IPV4:
        return None
    ip = frame[14:]
    if ip[0] >> 4 != 4:
        return None
    header_length = (ip[0] & 0x0F) * 4
    if header_length < 20 or len(ip) < header_length or ip[9] != IP_UDP:
        return None
    total_length = struct.unpack("!H", ip[2:4])[0]
    if total_length < header_length + 8 or total_length > len(ip):
        return None
    udp = ip[header_length:total_length]
    source_port, destination_port, udp_length = struct.unpack("!HHH", udp[:6])
    if source_port != BOOTP_CLIENT_PORT or destination_port != BOOTP_SERVER_PORT:
        return None
    if udp_length < 8 or udp_length > len(udp):
        return None
    bootp = udp[8:udp_length]
    if len(bootp) < 236 or bootp[0] != 1:
        return None
    boot_file = bootp[108:236].split(b"\x00", 1)[0]
    if not boot_file.startswith(BOOT_FILE_PREFIX) or len(boot_file) < len(BOOT_FILE_PREFIX) + 4:
        return None
    device_bytes = boot_file[len(BOOT_FILE_PREFIX) : len(BOOT_FILE_PREFIX) + 4]
    try:
        device_id = device_bytes.decode("ascii")
    except UnicodeDecodeError:
        return None
    return bootp[28:34], device_id


def wait_for_announcement(
    sock: socket.socket, wait_seconds: float, overall_deadline: float
) -> tuple[bytes, str]:
    deadline = min(time.monotonic() + wait_seconds, overall_deadline)
    ignored = 0
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise BootError(
                "timed out waiting for the DSP Ethernet-ready announcement "
                f"({ignored} other frame{'s' if ignored != 1 else ''} ignored)"
            )
        readable, _, _ = select.select([sock], [], [], remaining)
        if not readable:
            continue
        try:
            frame = sock.recv(MAX_FRAME)
        except BlockingIOError:
            continue
        announcement = parse_announcement(frame)
        if announcement is not None:
            return announcement
        ignored += 1


def assemble_table(entry: int, sections: list[Section], big_endian: bool) -> bytes:
    word = struct.Struct(">I" if big_endian else "<I")
    table = bytearray(word.pack(entry))
    for section in sections:
        padding = (-len(section.data)) % 4
        data = section.data + b"\x00" * padding
        table += word.pack(len(data))
        table += word.pack(section.destination)
        table += data
    table += word.pack(0)
    return bytes(table)


def build_frame(
    destination_mac: bytes,
    source_mac: bytes,
    source_ip: bytes,
    destination_ip: bytes,
    source_port: int,
    destination_port: int,
    sequence: int,
    payload: bytes,
) -> bytes:
    boot_header = struct.pack("!HH", BOOT_MAGIC, (BOOT_OPCODE << 8) | (sequence & 0xFF))
    udp_payload = boot_header + payload
    udp = struct.pack("!HHHH", source_port, destination_port, 8 + len(udp_payload), 0)
    udp += udp_payload
    ip = struct.pack(
        "!BBHHHBBH4s4s",
        0x45,
        0,
        20 + len(udp),
        sequence & 0xFFFF,
        0,
        64,
        IP_UDP,
        0,
        source_ip,
        destination_ip,
    )
    ip = ip[:10] + struct.pack("!H", checksum(ip)) + ip[12:]
    frame = destination_mac + source_mac + struct.pack("!H", ETH_IPV4) + ip + udp
    if len(frame) < 60:
        frame += b"\x00" * (60 - len(frame))
    return frame


def send_table(
    sock: socket.socket,
    peer: str,
    table: bytes,
    payload_size: int,
    gap: float,
    deadline: float,
    destination_mac: bytes,
    arguments: argparse.Namespace,
) -> int:
    payloads = [table[offset : offset + payload_size] for offset in range(0, len(table), payload_size)]
    for sequence, payload in enumerate(payloads):
        if time.monotonic() >= deadline:
            raise BootError(f"overall timeout expired after sending {sequence} frames")
        frame = build_frame(
            destination_mac,
            arguments.src_mac,
            arguments.src_ip,
            arguments.dst_ip,
            arguments.src_port,
            arguments.dest_port,
            sequence,
            payload,
        )
        try:
            sock.sendto(frame, peer)
        except OSError as error:
            raise BootError(f"could not send frame {sequence} to {peer}: {error}") from error
        if gap and sequence + 1 < len(payloads):
            time.sleep(min(gap, max(0.0, deadline - time.monotonic())))
    return len(payloads)


def build_parser() -> argparse.ArgumentParser:
    parser = BootArgumentParser(description=__doc__)
    parser.add_argument("--socket", required=True, help="Unix datagram socket path to bind")
    parser.add_argument("--peer", required=True, help="emulator --net-unix socket path")
    parser.add_argument(
        "--core",
        action="append",
        required=True,
        type=parse_core,
        metavar="N:IMAGE",
        help="ELF32 TI C6000 image and the core that runs it (repeatable)",
    )
    parser.add_argument("--entry", type=parse_u32, metavar="SPEC", help="boot-table entry override")
    parser.add_argument(
        "--word",
        action="append",
        default=[],
        type=parse_word,
        metavar="ADDR=VALUE",
        help="append a one-word boot-table section (repeatable)",
    )
    parser.add_argument("--release", type=parse_u32, help="BOOT_COMPLETE_STAT value")
    parser.add_argument("--no-release", action="store_true", help="omit BOOT_COMPLETE_STAT section")
    parser.add_argument("--dest-port", type=parse_port, default=9)
    parser.add_argument("--src-port", type=parse_port, default=4096)
    parser.add_argument("--dst-mac", type=parse_mac, help="DSP destination MAC")
    parser.add_argument("--src-mac", type=parse_mac, default=parse_mac("02:00:00:00:00:01"))
    parser.add_argument("--src-ip", type=parse_ip, default=parse_ip("192.168.64.1"))
    parser.add_argument("--dst-ip", type=parse_ip, default=parse_ip("192.168.64.10"))
    parser.add_argument("--payload", type=positive_multiple_of_four, default=1024, metavar="BYTES")
    parser.add_argument("--gap", type=nonnegative_float, default=0.002, metavar="SECONDS")
    parser.add_argument("--wait-ready", type=nonnegative_float, default=20.0, metavar="SECONDS")
    parser.add_argument("--no-wait-ready", action="store_true", help="send without an announcement")
    parser.add_argument("--big-endian", action="store_true", help="emit boot-table words big-endian")
    parser.add_argument("--timeout", type=positive_float, default=60.0, metavar="SECONDS")
    parser.add_argument("--json", action="store_true", help="print one JSON report")
    return parser


def run(arguments: argparse.Namespace) -> dict[str, object]:
    cores = [core for core, _ in arguments.core]
    if len(set(cores)) != len(cores):
        raise BootError("each core may be specified only once")
    payload_size = min(arguments.payload, MAX_BOOT_PAYLOAD)
    deadline = time.monotonic() + arguments.timeout

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    bound = False
    try:
        try:
            os.unlink(arguments.socket)
        except FileNotFoundError:
            pass
        sock.bind(arguments.socket)
        bound = True
        sock.setblocking(False)

        ready = False
        dsp_mac = None
        device_id = None
        if not arguments.no_wait_ready:
            dsp_mac, device_id = wait_for_announcement(sock, arguments.wait_ready, deadline)
            ready = True

        images = []
        for core, path in arguments.core:
            images.append(load_elf(core, path))
            if time.monotonic() >= deadline:
                raise BootError("overall timeout expired while reading ELF images")

        if arguments.entry is not None:
            entry = arguments.entry
        else:
            default_image = next((image for image in images if image.core == 0), None)
            if default_image is None:
                default_image = min(images, key=lambda image: image.core)
            entry = default_image.entry

        word = struct.Struct(">I" if arguments.big_endian else "<I")
        sections = []
        for image in images:
            sections.extend(
                Section(global_alias(image.core, segment.address), segment.data)
                for segment in image.segments
            )
        for image in images:
            if image.core != 0:
                sections.append(
                    Section(DSP_BOOT_ADDR0 + 0x20 * image.core, word.pack(image.entry >> 10))
                )
        sections.extend(Section(address, word.pack(value)) for address, value in arguments.word)

        if arguments.no_release:
            release = None
        else:
            release = arguments.release
            if release is None:
                release = sum(1 << core for core in cores)
            sections.append(Section(BOOT_COMPLETE_STAT, word.pack(release)))

        table = assemble_table(entry, sections, arguments.big_endian)
        destination_mac = arguments.dst_mac or dsp_mac or b"\xff" * 6
        frames = send_table(
            sock,
            arguments.peer,
            table,
            payload_size,
            arguments.gap,
            deadline,
            destination_mac,
            arguments,
        )
    finally:
        sock.close()
        if bound:
            try:
                os.unlink(arguments.socket)
            except FileNotFoundError:
                pass

    return {
        "ready": ready,
        "dsp_mac": mac_text(dsp_mac) if dsp_mac is not None else None,
        "device_id": device_id,
        "entry": entry,
        "sections": len(sections),
        "table_bytes": len(table),
        "frames": frames,
        "release": release,
    }


def print_report(report: dict[str, object], as_json: bool) -> None:
    if as_json:
        print(json.dumps(report))
        return
    print(f"ready: {'yes' if report['ready'] else 'no'}")
    print(f"dsp_mac: {report['dsp_mac'] or 'unknown'}")
    print(f"device_id: {report['device_id'] or 'unknown'}")
    print(f"entry: 0x{report['entry']:08x}")
    print(f"sections: {report['sections']}")
    print(f"table_bytes: {report['table_bytes']}")
    print(f"frames: {report['frames']}")
    release = report["release"]
    print(f"release: {'none' if release is None else f'0x{release:08x}'}")


def main() -> int:
    try:
        parser = build_parser()
        arguments = parser.parse_args()
        report = run(arguments)
        print_report(report, arguments.json)
        return 0
    except (BootError, OSError) as error:
        print(f"emac_boot.py: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
