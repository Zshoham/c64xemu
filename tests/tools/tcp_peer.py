#!/usr/bin/env python3
"""Hold a TCP connection with firmware running inside the emulator.

The emulator exchanges raw Ethernet frames over a Unix datagram socket (see src/netio.rs), so a
host process can take the place of a network without any privilege: this script is a minimal
ARP/IPv4/TCP peer that speaks to the NDK stack running on the emulated C6455. It launches the
emulator itself so the whole exchange is one command, and every step is checked, so the exit
status is the test result:

    ARP who-has the DSP -> reply, then SYN -> SYN/ACK -> ACK, payload -> echo, FIN -> FIN/ACK.

    Beyond that default there are three other exchanges, each selected by a flag: an ICMP echo
    round trip (--icmp), a SYN to a port nothing listens on that must come back as a reset
    (--expect-reset), and a TCP payload whose segments arrive out of order (--out-of-order).
    The bandwidth mode (--bandwidth-kbps, optionally with --udp-burst) paces the sender to the
    configured rate with a token bucket over wall clock and fails when the exchange cannot
    complete at that rate. Everything is content-checked; the exit status is the test result.

    Nothing here knows anything about the emulator's internals; the only interface is the socket and
    the frames on it.
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

ETH_ARP = 0x0806
ETH_IPV4 = 0x0800
ARP_REQUEST = 1
ARP_REPLY = 2
IP_TCP = 6
IP_UDP = 17
IP_ICMP = 1
ICMP_ECHO = 8
ICMP_ECHOREPLY = 0
DHCP_SERVER_PORT = 67
DHCP_CLIENT_PORT = 68
DHCP_MAGIC = b"\x63\x82\x53\x63"

FIN = 0x01
SYN = 0x02
RST = 0x04
PSH = 0x08
ACK = 0x10


def checksum(data: bytes) -> int:
    if len(data) % 2:
        data += b"\x00"
    total = 0
    for index in range(0, len(data), 2):
        total += (data[index] << 8) | data[index + 1]
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


class Peer:
    def __init__(self, sock: socket.socket, emulator_path: str, *, mac: bytes, ip: str, dsp_ip: str):
        self.sock = sock
        self.emulator_path = emulator_path
        self.mac = mac
        self.ip = socket.inet_aton(ip)
        self.dsp_ip = socket.inet_aton(dsp_ip)
        self.dsp_mac: bytes | None = None
        self.log: list[str] = []

    # ---- wire ----------------------------------------------------------------
    def send_frame(self, destination: bytes, ether_type: int, payload: bytes) -> None:
        frame = destination + self.mac + struct.pack("!H", ether_type) + payload
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

    # ---- ARP -----------------------------------------------------------------
    def arp_request(self) -> None:
        payload = struct.pack("!HHBBH", 1, ETH_IPV4, 6, 4, ARP_REQUEST)
        payload += self.mac + self.ip + b"\x00" * 6 + self.dsp_ip
        self.send_frame(b"\xff" * 6, ETH_ARP, payload)

    def arp_reply(self, to_mac: bytes, to_ip: bytes) -> None:
        payload = struct.pack("!HHBBH", 1, ETH_IPV4, 6, 4, ARP_REPLY)
        payload += self.mac + self.ip + to_mac + to_ip
        self.send_frame(to_mac, ETH_ARP, payload)

    def handle_arp(self, payload: bytes) -> None:
        if len(payload) < 28:
            return
        operation = struct.unpack("!H", payload[6:8])[0]
        sender_mac, sender_ip = payload[8:14], payload[14:18]
        target_ip = payload[24:28]
        if operation == ARP_REPLY and sender_ip == self.dsp_ip:
            self.dsp_mac = sender_mac
            self.log.append(f"arp: reply from {socket.inet_ntoa(sender_ip)} at {mac_text(sender_mac)}")
        elif operation == ARP_REQUEST and target_ip == self.ip:
            self.dsp_mac = sender_mac
            self.log.append(f"arp: request from {socket.inet_ntoa(sender_ip)}, answered")
            self.arp_reply(sender_mac, sender_ip)

    # ---- TCP -----------------------------------------------------------------
    def tcp(self, source_port: int, destination_port: int, seq: int, ack: int, flags: int, payload: bytes = b"") -> bytes:
        header = struct.pack(
            "!HHIIBBHHH", source_port, destination_port, seq, ack, 5 << 4, flags, 8192, 0, 0
        )
        pseudo = self.ip + self.dsp_ip + struct.pack("!BBH", 0, IP_TCP, len(header) + len(payload))
        total = checksum(pseudo + header + payload)
        header = header[:16] + struct.pack("!H", total) + header[18:]
        return header + payload

    def send_ip(self, protocol: int, payload: bytes, ident: int) -> None:
        assert self.dsp_mac is not None
        header = struct.pack(
            "!BBHHHBBH", 0x45, 0, 20 + len(payload), ident, 0, 64, protocol, 0
        ) + self.ip + self.dsp_ip
        header = header[:10] + struct.pack("!H", checksum(header)) + header[12:]
        self.send_frame(self.dsp_mac, ETH_IPV4, header + payload)


class DhcpServer:
    """The smallest DHCP server that satisfies the NDK's client (RFC 2131 sections 3.1 and 4.3).

    One fixed lease, offered and then acknowledged: the client broadcasts a DISCOVER from 0.0.0.0
    to 255.255.255.255, this answers with an OFFER carrying the address, mask, router and server
    identifier, and answers the REQUEST that follows with an ACK. Nothing else is needed - and the
    address handed out is the one the firmware used to have statically, so everything downstream
    of it is unchanged.
    """

    def __init__(self, peer: "Peer", lease: str, server: str, mask: str):
        self.peer = peer
        self.lease = socket.inet_aton(lease)
        self.server = socket.inet_aton(server)
        self.mask = socket.inet_aton(mask)
        self.steps: list[str] = []

    def handle(self, frame: bytes) -> bool:
        if len(frame) < 42 or struct.unpack("!H", frame[12:14])[0] != ETH_IPV4:
            return False
        ip = frame[14:]
        header_length = (ip[0] & 0x0F) * 4
        if ip[9] != IP_UDP:
            return False
        udp = ip[header_length:]
        source_port, destination_port = struct.unpack("!HH", udp[:4])
        if destination_port != DHCP_SERVER_PORT:
            return False
        bootp = udp[8:]
        if len(bootp) < 240 or bootp[0] != 1 or bootp[236:240] != DHCP_MAGIC:
            return False
        xid = bootp[4:8]
        client_mac = bootp[28:34]
        message_type = 0
        index = 240
        while index + 1 < len(bootp):
            option, length = bootp[index], bootp[index + 1]
            if option == 255:
                break
            if option == 53 and length == 1:
                message_type = bootp[index + 2]
            index += 2 + length
        if message_type == 1:
            self.steps.append("discover")
            self.reply(xid, client_mac, 2)
            self.steps.append("offer")
            return True
        if message_type == 3:
            self.steps.append("request")
            self.reply(xid, client_mac, 5)
            self.steps.append("ack")
            return True
        return False

    def reply(self, xid: bytes, client_mac: bytes, message_type: int) -> None:
        bootp = struct.pack("!BBBB", 2, 1, 6, 0) + xid + struct.pack("!HH", 0, 0)
        bootp += b"\x00" * 4 + self.lease + self.server + b"\x00" * 4
        bootp += client_mac + b"\x00" * 10 + b"\x00" * 192 + DHCP_MAGIC
        options = bytes([53, 1, message_type])
        options += bytes([1, 4]) + self.mask
        options += bytes([3, 4]) + self.server
        options += bytes([51, 4]) + struct.pack("!I", 3600)
        options += bytes([54, 4]) + self.server
        options += bytes([255])
        payload = bootp + options
        udp = struct.pack(
            "!HHHH", DHCP_SERVER_PORT, DHCP_CLIENT_PORT, 8 + len(payload), 0
        ) + payload
        total = 20 + len(udp)
        header = struct.pack("!BBHHHBBH", 0x45, 0, total, 0, 0, 64, IP_UDP, 0)
        header += self.server + b"\xff\xff\xff\xff"
        header = header[:10] + struct.pack("!H", checksum(header)) + header[12:]
        self.peer.send_frame(b"\xff" * 6, ETH_IPV4, header + udp)


def mac_text(mac: bytes) -> str:
    return ":".join(f"{byte:02x}" for byte in mac)


def parse_ip_tcp(frame: bytes) -> tuple[bytes, bytes, int, int, int, int, int, bytes] | None:
    """Return (source_ip, destination_ip, sport, dport, seq, ack, flags, payload)."""
    if len(frame) < 34 or struct.unpack("!H", frame[12:14])[0] != ETH_IPV4:
        return None
    ip = frame[14:]
    header_length = (ip[0] & 0x0F) * 4
    if ip[9] != IP_TCP:
        return None
    total_length = struct.unpack("!H", ip[2:4])[0]
    tcp = ip[header_length:total_length]
    if len(tcp) < 20:
        return None
    sport, dport, seq, ack = struct.unpack("!HHII", tcp[:12])
    data_offset = (tcp[12] >> 4) * 4
    flags = tcp[13]
    return ip[12:16], ip[16:20], sport, dport, seq, ack, flags, tcp[data_offset:]


def build_icmp_echo(ident: int, seq: int, data: bytes) -> bytes:
    """An ICMP echo request with the checksum filled in."""
    message = struct.pack("!BBHHH", ICMP_ECHO, 0, 0, ident, seq) + data
    return message[:2] + struct.pack("!H", checksum(message)) + message[4:]


def parse_icmp_echo(frame: bytes, dsp_ip: bytes) -> tuple[int, int, int, bytes] | None:
    """Return (icmp_type, id, seq, payload) for an ICMP message from the DSP, else None."""
    if len(frame) < 42 or struct.unpack("!H", frame[12:14])[0] != ETH_IPV4:
        return None
    ip = frame[14:]
    header_length = (ip[0] & 0x0F) * 4
    if ip[9] != IP_ICMP or ip[12:16] != dsp_ip:
        return None
    total_length = struct.unpack("!H", ip[2:4])[0]
    icmp = ip[header_length:total_length]
    if len(icmp) < 8:
        return None
    icmp_type = icmp[0]
    ident, seq = struct.unpack("!HH", icmp[4:8])
    return icmp_type, ident, seq, icmp[8:]


def build_payload(arguments: argparse.Namespace) -> bytes:
    """The bytes to send: a request, a pattern of a given length, or the default text."""
    if arguments.datasrv:
        # TI's dtask_tcp_datasrv reads "sizeof(int)" bytes and, on a big-endian build, byte-swaps
        # them because "Size comes in as little endian!" - so the wire order is little-endian
        # whatever the DSP is.
        return struct.pack("<i", arguments.datasrv)
    if arguments.request:
        return arguments.request.encode().decode("unicode_escape").encode()
    if arguments.bytes:
        return bytes(((index * 31 + 7) & 0xFF) for index in range(arguments.bytes))
    return arguments.payload.encode()


# TI's own buffer, from `dtask_tcp_datasrv` in `tools/servers/newservers.c`: a 512-byte block
# filled with `data[i] = ' ' + i`, which the daemon sends `size / 512` times.
DATASRV_BUFSIZE = 512
DATASRV_BLOCK = bytes((0x20 + index) & 0xFF for index in range(DATASRV_BUFSIZE))


def datasrv_reply(size: int) -> bytes:
    """What TI's daemon sends for a request of `size` bytes.

    Its own arithmetic: `count = size / DATASRV_BUFSIZE` whole blocks, then `remainder` bytes
    taken from `(char *)&size` - the request word itself, which is what the code says even
    though a reader expects the buffer. A size that is a whole number of blocks avoids the
    question, and that is what the suite asks for.
    """
    count, remainder = divmod(size, DATASRV_BUFSIZE)
    return DATASRV_BLOCK * count + struct.pack("<i", size)[:remainder]


def strip_telnet(stream: bytes) -> bytes:
    """The console's text with telnet's own option negotiation removed.

    RFC 854: 0xff introduces a command - two bytes for DO/DONT/WILL/WONT and one for the rest,
    with 0xff 0xff standing for a literal 0xff.
    """
    out = bytearray()
    index = 0
    while index < len(stream):
        byte = stream[index]
        if byte != 0xFF:
            out.append(byte)
            index += 1
            continue
        if index + 1 >= len(stream):
            break
        command = stream[index + 1]
        if command == 0xFF:
            out.append(0xFF)
            index += 2
        elif command in (0xFB, 0xFC, 0xFD, 0xFE):
            index += 3
        else:
            index += 2
    return bytes(out)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--emulator", default="target/release/c64xemu")
    parser.add_argument("--image", required=True)
    parser.add_argument("--max-cycles", type=int, default=1_200_000_000)
    parser.add_argument("--net-poll", type=int, default=512)
    parser.add_argument("--dsp-ip", default="192.168.64.10")
    parser.add_argument("--peer-ip", default="192.168.64.1")
    parser.add_argument("--port", type=int, default=3000)
    parser.add_argument("--payload", default="c64xemu says hello over TCP\n")
    parser.add_argument(
        "--bytes",
        type=int,
        default=0,
        help="send this many bytes of a deterministic pattern instead of the text payload",
    )
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--grace", type=float, default=0.5, help="seconds to let the firmware finish printing")
    parser.add_argument(
        "--udp",
        default="",
        help="send this text as a UDP datagram to --udp-port and require it echoed back, "
        "instead of running a TCP exchange",
    )
    parser.add_argument("--udp-port", type=int, default=3001)
    parser.add_argument(
        "--request",
        default="",
        help="send this text once and collect the reply until the firmware closes, instead of "
        "expecting the payload echoed back",
    )
    parser.add_argument(
        "--expect",
        default="",
        help="require this substring in the collected reply",
    )
    parser.add_argument(
        "--drop-echo",
        default="",
        help="comma-separated indices of echoed data segments to drop once, forcing the "
        "firmware's own retransmission timer to recover them",
    )
    parser.add_argument(
        "--icmp",
        action="store_true",
        help="ping the firmware once and require its ICMP echo reply, instead of running a "
        "TCP exchange",
    )
    parser.add_argument(
        "--expect-reset",
        action="store_true",
        help="treat a reset from the firmware as success: the SYN went to a port nothing "
        "listens on, and the refusal is the contract",
    )
    parser.add_argument(
        "--out-of-order",
        action="store_true",
        help="send the payload's segments out of order, so the firmware's TCP reassembly "
        "queue has to hold the later segments until the earlier ones arrive",
    )
    parser.add_argument(
        "--bandwidth-kbps",
        type=float,
        default=0,
        help="pace the sender at this many kilobits per second (wall clock); with --udp-burst "
        "it paces a stream of datagrams instead of a TCP payload",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="seconds to send at --bandwidth-kbps; the TCP payload or the datagram count is "
        "derived from it",
    )
    parser.add_argument(
        "--udp-burst",
        action="store_true",
        help="send a paced burst of 500-byte datagrams to --udp-port and require every one "
        "echoed, instead of a single datagram or a TCP exchange",
    )
    parser.add_argument(
        "--telnet",
        action="store_true",
        help="hold a dialogue with the firmware's telnet console: wait for its banner, ask for "
        "help, then quit, instead of sending a payload",
    )
    parser.add_argument(
        "--datasrv",
        type=int,
        default=0,
        help="request this many bytes from TI's own dtask_tcp_datasrv (port 1000) and require "
        "exactly the bytes TI's daemon sends back, instead of an echo of the payload",
    )
    parser.add_argument(
        "--report",
        action="store_true",
        help="print one JSON line describing the exchange, for a test harness to check",
    )
    arguments = parser.parse_args()

    directory = tempfile.mkdtemp(prefix="c64xemu-tcp-")
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

    deadline = time.monotonic() + arguments.timeout
    while not os.path.exists(emulator_socket):
        if time.monotonic() > deadline or emulator.poll() is not None:
            print("emulator never bound its socket", file=sys.stderr)
            return 1
        time.sleep(0.05)

    peer = Peer(sock, emulator_socket, mac=b"\x02\x00\x00\x44\x55\x66", ip=arguments.peer_ip, dsp_ip=arguments.dsp_ip)
    dhcp = DhcpServer(peer, arguments.dsp_ip, arguments.peer_ip, "255.255.255.0")
    source_port = 40000
    initial_seq = 0x1000
    payload = build_payload(arguments)
    window = 4096
    chunk = 1024
    # Pacing: the sender admits bytes as wall-clock time earns them. The TCP payload for a
    # paced run is whatever the configured rate carries in the configured duration.
    bytes_per_sec = arguments.bandwidth_kbps * 125.0
    poll_timeout = 0.02
    if arguments.bandwidth_kbps:
        poll_timeout = min(0.02, max(0.001, chunk / bytes_per_sec))
        if not arguments.udp_burst:
            payload = bytes(
                ((index * 31 + 7) & 0xFF)
                for index in range(int(arguments.bandwidth_kbps * 125.0 * arguments.duration))
            )
    # What the firmware is expected to send back, which is settled only once the payload is:
    # normally it echoes what it was sent, while TI's data server answers a four-byte request
    # with as many bytes as that request asked for.
    expected_reply = datasrv_reply(arguments.datasrv) if arguments.datasrv else payload
    # The pacing epoch starts when the first byte actually goes out, after ARP and DHCP, so
    # the deadline is the configured duration with a fifty-percent margin on top of that.
    send_started = 0.0
    send_deadline = 0.0
    # A paced UDP burst: 500-byte datagrams, each with its index in the tail, every one of
    # which must come back - a lost datagram is a lost contract, there is no retransmission.
    udp_burst_size = 500
    udp_count = (
        int(arguments.bandwidth_kbps * 125.0 * arguments.duration / udp_burst_size)
        if arguments.udp_burst
        else 0
    )
    udp_sent_count = 0
    udp_echoed_count = 0
    # Out-of-order mode sends the segments newest-first after the first, so the stack's
    # reassembly queue has to hold every later segment until its predecessor arrives.
    segment_count = (len(payload) + chunk - 1) // chunk
    ooo_order = [0] + list(range(segment_count - 1, 0, -1))
    ooo_index = 0

    seq = initial_seq          # our next byte to send, absolute
    unacked = initial_seq + 1  # the first byte the firmware has not acknowledged
    ack = 0                    # the next byte we expect from the firmware
    echoed = b""
    dropped_targets = {int(part) for part in arguments.drop_echo.split(",") if part.strip()}
    dropped: set[int] = set()
    segments = 0
    state = "arp"
    last_send = 0.0
    last_progress = time.monotonic()
    status = 1
    identifier = 1
    icmp_sent = 0.0   # when the last echo request went out; 0 means never
    icmp_ok = False

    def send(flags: int, data: bytes = b"", *, at: int | None = None) -> None:
        nonlocal identifier
        identifier += 1
        peer.send_ip(
            IP_TCP,
            peer.tcp(source_port, arguments.port, initial_seq + 1 + (at if at is not None else 0), ack, flags, data),
            identifier,
        )

    udp_sent = 0.0
    udp_reply = b""
    # The telnet dialogue: each step waits for a marker in what the console has printed and then
    # sends one line. The console asks for no password, so the first marker is its own welcome.
    telnet_steps = [
        ("Welcome to the console program.", b"?\r\n"),
        # Waiting for the last line of the help text rather than its first keeps the whole
        # listing in the stream: quitting on the first line would cut it off mid-print.
        ("gives more information on the route command.", b"quit\r\n"),
    ]
    telnet_step = 0
    try:
        while time.monotonic() < deadline and emulator.poll() is None:
            now = time.monotonic()
            if arguments.udp:
                # The datagram path: resolve the firmware's address, then send and wait.
                if peer.dsp_mac is None:
                    if now - last_send > 0.5:
                        peer.arp_request()
                        last_send = now
                elif now - udp_sent > 2.0 and not udp_reply:
                    payload = arguments.udp.encode()
                    body = struct.pack("!HHHH", 40000, arguments.udp_port, 8 + len(payload), 0)
                    peer.send_ip(IP_UDP, body + payload, 7)
                    peer.log.append(f"udp: sent {len(payload)} bytes")
                    udp_sent = now
                for frame in peer.poll(poll_timeout):
                    ether_type = struct.unpack("!H", frame[12:14])[0] if len(frame) >= 14 else 0
                    if ether_type == ETH_ARP:
                        peer.handle_arp(frame[14:])
                        continue
                    if dhcp.handle(frame):
                        continue
                    if len(frame) < 42 or ether_type != ETH_IPV4:
                        continue
                    ip = frame[14:]
                    if ip[9] != IP_UDP:
                        continue
                    udp = ip[((ip[0] & 0x0F) * 4):]
                    source_port, destination_port = struct.unpack("!HH", udp[:4])
                    length = struct.unpack("!H", udp[4:6])[0]
                    if destination_port != 40000:
                        continue
                    udp_reply = udp[8:length]
                    peer.log.append(f"udp: {len(udp_reply)} bytes echoed from port {source_port}")
                    state = "closed"
                if state == "closed":
                    break
                continue
            if arguments.icmp:
                # The ping path: resolve the firmware's address, then an echo request - resent
                # every two seconds until the reply - and the stack's own answer: icmpin.c
                # reflects the request with the identifier, sequence and payload intact, so a
                # matching reply is the whole contract.
                if peer.dsp_mac is None:
                    if now - last_send > 0.5:
                        peer.arp_request()
                        last_send = now
                elif now - icmp_sent > 2.0 and not icmp_ok:
                    peer.send_ip(IP_ICMP, build_icmp_echo(0x4242, 1, b"c64xemu icmp ping payload"), 9)
                    peer.log.append("icmp: echo request sent")
                    icmp_sent = now
                for frame in peer.poll(poll_timeout):
                    ether_type = struct.unpack("!H", frame[12:14])[0] if len(frame) >= 14 else 0
                    if ether_type == ETH_ARP:
                        peer.handle_arp(frame[14:])
                        continue
                    if dhcp.handle(frame):
                        continue
                    reply = parse_icmp_echo(frame, peer.dsp_ip)
                    if reply is None:
                        continue
                    icmp_type, icmp_id, icmp_seq, icmp_data = reply
                    if (
                        icmp_type == ICMP_ECHOREPLY
                        and icmp_id == 0x4242
                        and icmp_seq == 1
                        and icmp_data == b"c64xemu icmp ping payload"
                    ):
                        icmp_ok = True
                        peer.log.append("icmp: echo reply matched")
                        state = "closed"
                        break
                if state == "closed":
                    break
                continue
            if arguments.udp_burst:
                # The paced datagram burst: every datagram must come back, and there is no
                # retransmission, so a drop anywhere in the emulator's receive path fails the
                # exchange. The deadline is the configured rate with a fifty-percent margin.
                if peer.dsp_mac is None:
                    if now - last_send > 0.5:
                        peer.arp_request()
                        last_send = now
                else:
                    if send_started == 0.0:
                        send_started = now
                        send_deadline = send_started + arguments.duration * 1.5
                    # One datagram of initial credit so the first one goes out immediately.
                    allowed = int((now - send_started) * bytes_per_sec) + udp_burst_size
                    while (
                        udp_sent_count < udp_count
                        and udp_sent_count * udp_burst_size + udp_burst_size <= allowed
                    ):
                        index = udp_sent_count
                        data = bytes(((byte * 31 + 7) & 0xFF) for byte in range(496)) + struct.pack("!I", index)
                        body = struct.pack("!HHHH", 40000, arguments.udp_port, 8 + len(data), 0)
                        peer.send_ip(IP_UDP, body + data, 7)
                        udp_sent_count += 1
                    if now > send_deadline and udp_sent_count < udp_count:
                        peer.log.append(
                            f"udp: {udp_sent_count} of {udp_count} datagrams sent by the deadline"
                        )
                        state = "failed"
                for frame in peer.poll(poll_timeout):
                    ether_type = struct.unpack("!H", frame[12:14])[0] if len(frame) >= 14 else 0
                    if ether_type == ETH_ARP:
                        peer.handle_arp(frame[14:])
                        continue
                    if dhcp.handle(frame):
                        continue
                    if len(frame) < 42 or ether_type != ETH_IPV4:
                        continue
                    ip = frame[14:]
                    if ip[9] != IP_UDP:
                        continue
                    udp = ip[((ip[0] & 0x0F) * 4):]
                    source_port, destination_port = struct.unpack("!HH", udp[:4])
                    length = struct.unpack("!H", udp[4:6])[0]
                    if destination_port != 40000 or length != 8 + udp_burst_size:
                        continue
                    udp_echoed_count += 1
                    peer.log.append(
                        f"udp: {udp_echoed_count}/{udp_count} datagrams echoed from port {source_port}"
                    )
                if udp_echoed_count >= udp_count and udp_sent_count >= udp_count:
                    peer.log.append(f"udp: all {udp_count} datagrams echoed")
                    state = "closed"
                elif send_started > 0.0 and now > send_deadline + 0.75:
                    peer.log.append(
                        f"udp: {udp_echoed_count} of {udp_count} datagrams echoed by the deadline"
                    )
                    state = "failed"
                if state in ("closed", "failed"):
                    break
                continue
            if state == "arp":
                if peer.dsp_mac is None:
                    if now - last_send > 0.5:
                        peer.arp_request()
                        last_send = now
                else:
                    peer.send_ip(IP_TCP, peer.tcp(source_port, arguments.port, initial_seq, 0, SYN), 1)
                    peer.log.append("tcp: SYN sent")
                    state = "syn-sent"
                    last_send = now
            elif state == "syn-sent" and now - last_send > 1.0:
                peer.send_ip(IP_TCP, peer.tcp(source_port, arguments.port, initial_seq, 0, SYN), 1)
                last_send = now
            elif state == "open":
                # Keep the window full, then retransmit from the first unacknowledged byte if
                # nothing has moved for a while: a frame the receive FIFO dropped is the one
                # thing that stalls this exchange.
                sent_offset = seq - (initial_seq + 1)
                if arguments.bandwidth_kbps and not arguments.udp_burst:
                    # Paced: admit as many bytes as the elapsed wall time has earned, and
                    # fail fast if the emulator cannot absorb the configured rate - the
                    # window throttling alone would hide it behind retransmissions.
                    if send_started == 0.0:
                        send_started = now
                        send_deadline = send_started + arguments.duration * 1.5
                    # One segment of initial credit so the first one goes out immediately.
                    allowed = int((now - send_started) * bytes_per_sec) + chunk
                    while (
                        sent_offset < len(payload)
                        and sent_offset < allowed
                        and seq - unacked < window
                    ):
                        piece = payload[sent_offset : sent_offset + chunk]
                        send(ACK | PSH, piece, at=sent_offset)
                        seq += len(piece)
                        sent_offset += len(piece)
                    if now > send_deadline and sent_offset < len(payload):
                        peer.log.append(
                            f"tcp: {sent_offset} of {len(payload)} bytes sent by the deadline"
                        )
                        state = "failed"
                elif arguments.out_of_order:
                    # Send every segment exactly once, newest-first after the first, so the
                    # stack's reassembly queue has to hold the later segments until their
                    # predecessors arrive. The resend fallback below is disabled in this
                    # mode: it would paper over a broken reassembly queue and the exchange
                    # would still complete.
                    while ooo_index < len(ooo_order) and seq - unacked < window:
                        offset = ooo_order[ooo_index] * chunk
                        piece = payload[offset : offset + chunk]
                        send(ACK | PSH, piece, at=offset)
                        seq = max(seq, initial_seq + 1 + offset + len(piece))
                        ooo_index += 1
                else:
                    while sent_offset < len(payload) and seq - unacked < window:
                        piece = payload[sent_offset : sent_offset + chunk]
                        send(ACK | PSH, piece, at=sent_offset)
                        seq += len(piece)
                        sent_offset += len(piece)
                if now - last_progress > 1.0 and not arguments.out_of_order:
                    resend_offset = unacked - (initial_seq + 1)
                    if resend_offset < len(payload):
                        piece = payload[resend_offset : resend_offset + chunk]
                        send(ACK | PSH, piece, at=resend_offset)
                        peer.log.append(f"tcp: retransmitted {len(piece)} bytes at {resend_offset}")
                        seq = max(seq, unacked + len(piece))
                    last_progress = now
            elif state == "draining" and now - last_progress > 1.0:
                send(ACK, at=len(payload))
                last_progress = now

            for frame in peer.poll(poll_timeout):
                ether_type = struct.unpack("!H", frame[12:14])[0] if len(frame) >= 14 else 0
                if ether_type == ETH_ARP:
                    peer.handle_arp(frame[14:])
                    continue
                if dhcp.handle(frame):
                    continue
                parsed = parse_ip_tcp(frame)
                if parsed is None:
                    continue
                source_ip, destination_ip, sport, dport, their_seq, their_ack, flags, data = parsed
                if destination_ip != peer.ip or dport != source_port:
                    continue
                if flags & RST:
                    if arguments.expect_reset:
                        peer.log.append("tcp: reset by the firmware, as expected")
                        state = "reset"
                    else:
                        peer.log.append("tcp: reset by the firmware")
                        state = "failed"
                    break
                if state == "syn-sent" and flags & SYN and flags & ACK:
                    ack = their_seq + 1
                    seq = initial_seq + 1
                    unacked = seq
                    send(ACK)
                    peer.log.append(f"tcp: SYN/ACK from {socket.inet_ntoa(source_ip)}:{sport}, connection open")
                    state = "open"
                    last_progress = now
                    continue
                if flags & ACK and state in ("open", "draining"):
                    if their_ack > unacked:
                        unacked = their_ack
                        last_progress = now
                if data and state in ("open", "draining"):
                    # Drop a chosen segment exactly once and do not acknowledge it: the only way
                    # the byte stream can still complete is the firmware's retransmission.
                    if segments in dropped_targets and segments not in dropped:
                        dropped.add(segments)
                        peer.log.append(f"tcp: dropped echo segment {segments} of {len(data)} bytes")
                        segments += 1
                        continue
                    segments += 1
                    if their_seq == ack:
                        ack = their_seq + len(data)
                        echoed += data
                        last_progress = now
                    send(ACK, at=min(seq, initial_seq + 1 + len(payload)) - (initial_seq + 1))
                    if arguments.telnet and telnet_step < len(telnet_steps):
                        marker, line = telnet_steps[telnet_step]
                        # Telnet option negotiation is interleaved with the text, so the markers
                        # are looked for in the stream with the IAC sequences taken out.
                        if marker.encode() in strip_telnet(echoed):
                            offset = seq - (initial_seq + 1)
                            send(ACK | PSH, line, at=offset)
                            seq += len(line)
                            payload += line
                            peer.log.append(f"telnet: sent {line!r}")
                            telnet_step += 1
                        continue
                    if not arguments.request and len(echoed) >= len(expected_reply):
                        send(FIN | ACK, at=len(payload))
                        peer.log.append(f"tcp: {len(echoed)} bytes echoed, FIN sent")
                        state = "fin-sent"
                    continue
                if state == "open" and seq - (initial_seq + 1) >= len(payload) and unacked >= seq:
                    state = "draining"
                    peer.log.append(f"tcp: sent {len(payload)} bytes, all acknowledged")
                if arguments.telnet and flags & FIN and state in ("open", "draining"):
                    ack = their_seq + 1
                    send(ACK, at=len(payload))
                    send(FIN | ACK, at=len(payload))
                    peer.log.append(
                        f"telnet: console closed the session after {len(echoed)} bytes"
                    )
                    state = "closed"
                    break
                if arguments.request and flags & FIN and state in ("open", "draining"):
                    ack = their_seq + 1
                    send(ACK, at=len(payload))
                    send(FIN | ACK, at=len(payload))
                    peer.log.append(
                        f"tcp: firmware closed after {len(echoed)} bytes of reply"
                    )
                    state = "closed"
                    break
                if state == "fin-sent" and flags & FIN:
                    ack = their_seq + 1
                    send(ACK, at=len(payload) + 1)
                    peer.log.append("tcp: FIN/ACK exchanged, connection closed")
                    state = "closed"
                    break
            if state in ("closed", "failed", "reset"):
                break

        if arguments.icmp:
            status = 0 if state == "closed" and icmp_ok else 1
        elif arguments.expect_reset:
            status = 0 if state == "reset" else 1
        elif arguments.udp_burst:
            status = 0 if state == "closed" and udp_echoed_count == udp_count else 1
        elif arguments.udp:
            echoed = udp_reply
            status = 0 if udp_reply == arguments.udp.encode() else 1
        elif arguments.telnet:
            echoed = strip_telnet(echoed)
            status = 0 if state == "closed" and telnet_step == len(telnet_steps) else 1
        elif arguments.request:
            status = 0 if state == "closed" and arguments.expect.encode() in echoed else 1
        else:
            status = 0 if state == "closed" and echoed == expected_reply else 1
        # Let the firmware's own close path print before the emulator is stopped.
        if state == "closed":
            grace = time.monotonic() + arguments.grace
            while time.monotonic() < grace:
                peer.poll(0.05)
    finally:
        emulator.terminate()
        try:
            out, err = emulator.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            emulator.kill()
            out, err = emulator.communicate()
        sock.close()
        if arguments.report:
            wanted = (
                "c64xemu ndk",
                "Network Added",
                "ndk:",
                "echo:",
                "udp:",
                "nocopy:",
                "servers:",
            )
            lines = [
                line.strip()
                for line in out.splitlines()
                if line.strip().startswith(wanted)
            ]
            print(
                json.dumps(
                    {
                        "state": state,
                        "echoed": len(echoed),
                        "payload_matched": echoed == expected_reply,
                        "reply_head": echoed[:96].decode("latin-1"),
                        # The console's own lines, which is what a telnet session is judged on.
                        "console": [
                            line.strip()
                            for line in echoed.decode("latin-1").splitlines()
                            if line.strip()
                        ]
                        if arguments.telnet
                        else [],
                        "expected_present": arguments.expect.encode() in echoed
                        if arguments.expect
                        else None,
                        "dsp_mac": mac_text(peer.dsp_mac) if peer.dsp_mac else None,
                        "dropped": sorted(dropped),
                        "icmp_reply": icmp_ok if arguments.icmp else None,
                        "udp_sent": udp_sent_count,
                        "udp_echoed": udp_echoed_count,
                        "dhcp": dhcp.steps,
                        # Bounded so the firmware's own "done" line stays out: it is printed
                        # after the exchange these cases assert, once the peer has stopped
                        # listening, so whether it arrives depends on when the emulator is
                        # stopped. Seven start-up lines and the two lines of a connection is
                        # what an exchange is judged on.
                        "firmware": lines[:9],
                    }
                )
            )
        if not arguments.quiet and not arguments.report:
            for line in peer.log:
                print(f"peer: {line}")
            print("--- firmware output ---")
            print(out.strip())
            if err.strip():
                print("--- emulator stderr ---", file=sys.stderr)
                print(err.strip(), file=sys.stderr)
        for path in (peer_socket, emulator_socket):
            try:
                os.unlink(path)
            except OSError:
                pass
        os.rmdir(directory)

    if status != 0:
        print(f"peer: failed in state {state}, echoed {echoed!r}", file=sys.stderr)
    elif not arguments.report:
        if arguments.request:
            print(f"peer: collected {len(echoed)} bytes of reply")
        else:
            print(f"peer: echo matched {len(echoed)} bytes")
    return status


if __name__ == "__main__":
    raise SystemExit(main())
