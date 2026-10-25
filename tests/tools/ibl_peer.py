#!/usr/bin/env python3
"""Serve TI's IBL its Ethernet boot: the BOOTP server and TFTP server a C6472 EVM boots from.

TI's C6472 EVM configuration (the IBL's ``iblConfig`` ``c6472_ibl_config``) makes boot mode 0
a TFTP boot with BOOTP: the IBL broadcasts a BOOTP request (RFC 951) from MAC port 0, takes its
address, the server's and the file name from the reply, resolves the server with ARP (RFC 826)
and reads the file with TFTP (RFC 1350) into DDR2 as a binary blob, then branches to the blob's
first byte. This script launches the emulator with the IBL's EEPROM and plays every host on
that network over the emulator's Unix datagram frame transport (``--net-unix``), so the whole
boot is one command whose exit status is the result.

The addresses are the EVM configuration's own: the DSP is 192.168.1.3 and the server
192.168.1.2 on a /24. TFTP is lock-step: each DATA block waits for its ACK and is sent again if
none arrives, which the IBL relies on - it does not acknowledge a block its stream buffer has
no room for yet. ACKs are taken from whatever port the IBL sends them from, because after the
first block the IBL moves its end of the transfer to port 1234.
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
IP_UDP = 17
ARP_REQUEST = 1
ARP_REPLY = 2
BOOTP_SERVER_PORT = 67
BOOTP_CLIENT_PORT = 68
TFTP_PORT = 69
TFTP_RRQ, TFTP_DATA, TFTP_ACK, TFTP_ERROR = 1, 3, 4, 5
TFTP_BLOCK = 512
# The port this server answers a read request from: its transfer identifier (RFC 1350 section 4).
TFTP_TID = 50000
VEND_COOKIE = b"\x63\x82\x53\x63"
BROADCAST_MAC = b"\xff" * 6
SERVER_MAC = b"\x02\x00\x00\x44\x55\x66"
SERVER_IP = socket.inet_aton("192.168.1.2")
DSP_IP = socket.inet_aton("192.168.1.3")
NET_MASK = socket.inet_aton("255.255.255.0")
# Wall-clock seconds before an unacknowledged block goes again. The emulator runs the IBL far
# slower than silicon, so this is long in emulated time; it only has to outlast the IBL's own
# drain of its stream buffer.
RETRANSMIT_SECONDS = 1.0


def checksum(data: bytes) -> int:
    if len(data) % 2:
        data += b"\x00"
    total = sum(struct.unpack(f"!{len(data) // 2}H", data))
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def mac_text(mac: bytes) -> str:
    return ":".join(f"{byte:02x}" for byte in mac)


class Network:
    """The hosts on the DSP's wire: one frame per Unix datagram, in each direction."""

    def __init__(self, sock: socket.socket, emulator_socket: str) -> None:
        self.sock = sock
        self.emulator_socket = emulator_socket
        self.dsp_mac: bytes | None = None
        self.steps: list[str] = []
        # Set once the emulator has exited: nothing is bound at its socket any more, so the
        # frames still queued from it are read but answers to them go nowhere.
        self.closed = False

    def send_frame(self, destination: bytes, ether_type: int, payload: bytes) -> None:
        if self.closed:
            return
        frame = destination + SERVER_MAC + struct.pack("!H", ether_type) + payload
        frame += bytes(max(0, 60 - len(frame)))
        self.sock.sendto(frame, self.emulator_socket)

    def send_udp(self, destination_mac: bytes, destination_ip: bytes, source_port: int,
                 destination_port: int, payload: bytes) -> None:
        length = 8 + len(payload)
        pseudo = SERVER_IP + destination_ip + struct.pack("!BBH", 0, IP_UDP, length)
        header = struct.pack("!HHHH", source_port, destination_port, length, 0)
        total = checksum(pseudo + header + payload) or 0xFFFF
        udp = header[:6] + struct.pack("!H", total) + payload
        ip = struct.pack("!BBHHHBBH", 0x45, 0, 20 + len(udp), 0, 0, 64, IP_UDP, 0)
        ip += SERVER_IP + destination_ip
        ip = ip[:10] + struct.pack("!H", checksum(ip)) + ip[12:]
        self.send_frame(destination_mac, ETH_IPV4, ip + udp)

    def poll(self, timeout: float) -> list[bytes]:
        frames = []
        ready, _, _ = select.select([self.sock], [], [], timeout)
        while ready:
            try:
                frames.append(self.sock.recv(65535))
            except BlockingIOError:
                break
            ready, _, _ = select.select([self.sock], [], [], 0)
        return frames


class Server:
    """BOOTP, ARP and TFTP for one DSP and one file."""

    def __init__(self, network: Network, file_name: str, contents: bytes | bytearray) -> None:
        self.network = network
        self.file_name = file_name
        self.contents = contents
        self.client: tuple[bytes, int] | None = None  # the DSP's TFTP port, once it asks
        # The block being sent, counted from 1 without limit: it places the block in the file
        # and says when the file is done. On the wire it is 16 bits and wraps from 65535 to 0
        # (RFC 1350 section 5), which TI's client follows with its own Uint16 block_num.
        self.block = 0
        self.sent_at = 0.0
        self.complete = False
        self.retransmits = 0
        self.errors: list[str] = []

    @property
    def blocks(self) -> int:
        """Blocks in the file: a final short block ends it, so an exact multiple of 512 ends
        with an empty one (RFC 1350 section 6)."""
        return len(self.contents) // TFTP_BLOCK + 1

    def handle(self, frame: bytes) -> None:
        if len(frame) < 14:
            return
        ether_type = struct.unpack("!H", frame[12:14])[0]
        if ether_type == ETH_ARP:
            self.handle_arp(frame[14:])
        elif ether_type == ETH_IPV4:
            self.handle_ip(frame[6:12], frame[14:])

    def handle_arp(self, arp: bytes) -> None:
        if len(arp) < 28 or struct.unpack("!H", arp[6:8])[0] != ARP_REQUEST:
            return
        sender_mac, sender_ip, target_ip = arp[8:14], arp[14:18], arp[24:28]
        if target_ip != SERVER_IP:
            return
        self.network.dsp_mac = sender_mac
        self.network.steps.append(f"arp who-has {socket.inet_ntoa(target_ip)} "
                                  f"from {socket.inet_ntoa(sender_ip)}")
        reply = struct.pack("!HHBBH", 1, ETH_IPV4, 6, 4, ARP_REPLY)
        reply += SERVER_MAC + SERVER_IP + sender_mac + sender_ip
        self.network.send_frame(sender_mac, ETH_ARP, reply)

    def handle_ip(self, source_mac: bytes, ip: bytes) -> None:
        if len(ip) < 28 or ip[9] != IP_UDP:
            return
        header_length = (ip[0] & 0x0F) * 4
        source_ip, destination_ip = ip[12:16], ip[16:20]
        udp = ip[header_length:]
        source_port, destination_port, length = struct.unpack("!HHH", udp[:6])
        payload = udp[8:length]
        if destination_port == BOOTP_SERVER_PORT:
            self.handle_bootp(source_mac, payload)
        elif destination_ip == SERVER_IP and destination_port == TFTP_PORT:
            self.handle_request(source_ip, source_port, payload)
        elif destination_ip == SERVER_IP and destination_port == TFTP_TID:
            self.handle_ack(source_ip, source_port, payload)

    def handle_bootp(self, source_mac: bytes, bootp: bytes) -> None:
        if len(bootp) < 236 or bootp[0] != 1:
            return
        xid, client_mac = bootp[4:8], bootp[28:34]
        self.network.dsp_mac = client_mac
        self.network.steps.append(f"bootp request from {mac_text(client_mac)}")
        if source_mac != client_mac:
            self.errors.append(f"bootp from {mac_text(source_mac)} names {mac_text(client_mac)}")
        name = self.file_name.encode()
        reply = struct.pack("!BBBB", 2, 1, 6, 0) + xid + struct.pack("!HH", 0, 0)
        reply += bytes(4) + DSP_IP + SERVER_IP + bytes(4)
        reply += client_mac + bytes(10) + bytes(64) + name + bytes(128 - len(name))
        reply += (VEND_COOKIE + bytes([1, 4]) + NET_MASK + bytes([255])).ljust(64, b"\x00")
        # The DSP has no address yet, so the reply is broadcast (RFC 951 section 7.5).
        self.network.send_udp(BROADCAST_MAC, b"\xff" * 4, BOOTP_SERVER_PORT, BOOTP_CLIENT_PORT,
                              reply)
        self.network.steps.append(f"bootp reply {socket.inet_ntoa(DSP_IP)} file {self.file_name}")

    def handle_request(self, source_ip: bytes, source_port: int, request: bytes) -> None:
        if len(request) < 4 or struct.unpack("!H", request[:2])[0] != TFTP_RRQ:
            return
        fields = request[2:].split(b"\x00")
        name, mode = fields[0].decode(errors="replace"), fields[1].decode(errors="replace")
        self.network.steps.append(f"tftp read {name} {mode}")
        if self.client is not None:
            return  # a repeated request while the first is already being answered
        if name != self.file_name or mode.lower() != "octet":
            self.errors.append(f"tftp read of {name!r} in {mode!r}")
            error = struct.pack("!HH", TFTP_ERROR, 1) + b"File not found\x00"
            self.network.send_udp(self.require_mac(), source_ip, TFTP_TID, source_port, error)
            return
        self.client = (source_ip, source_port)
        self.block = 1
        self.send_block()

    def handle_ack(self, source_ip: bytes, source_port: int, packet: bytes) -> None:
        if self.client is None or len(packet) < 4:
            return
        opcode, block = struct.unpack("!HH", packet[:4])
        if opcode == TFTP_ERROR:
            self.errors.append(f"tftp error from the DSP: {packet[4:].rstrip(bytes(1))!r}")
            self.client = None
            return
        if opcode != TFTP_ACK or block != self.block & 0xFFFF:
            return
        self.client = (source_ip, source_port)
        if self.block == self.blocks:
            self.complete = True
            self.client = None
            self.network.steps.append(f"tftp sent {len(self.contents)} bytes in {self.blocks} "
                                      f"blocks, {self.retransmits} sent again")
            return
        self.block += 1
        self.send_block()

    def send_block(self) -> None:
        assert self.client is not None
        start = (self.block - 1) * TFTP_BLOCK
        data = struct.pack("!HH", TFTP_DATA, self.block & 0xFFFF)
        data += self.contents[start:start + TFTP_BLOCK]
        self.network.send_udp(self.require_mac(), self.client[0], TFTP_TID, self.client[1], data)
        self.sent_at = time.monotonic()

    def tick(self) -> None:
        if self.client is not None and time.monotonic() - self.sent_at > RETRANSMIT_SECONDS:
            self.retransmits += 1
            self.send_block()

    def require_mac(self) -> bytes:
        if self.network.dsp_mac is None:
            raise RuntimeError("the DSP spoke TFTP before BOOTP or ARP named its MAC address")
        return self.network.dsp_mac


def main() -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").partition("\n")[0])
    parser.add_argument("--emulator", required=True)
    parser.add_argument("--image", required=True,
                        help="the ELF whose symbols the run reports against (the booted program)")
    parser.add_argument("--eeprom", required=True, help="the IBL's I2C EEPROM image")
    parser.add_argument("--blob", required=True, help="the file served over TFTP")
    parser.add_argument("--file-name", required=True, help="the name BOOTP hands the DSP")
    parser.add_argument("--max-cycles", type=int, required=True)
    parser.add_argument("--state-out", required=True)
    parser.add_argument("--timeout", type=float, default=900.0, help="wall-clock seconds")
    arguments = parser.parse_args()

    directory = tempfile.mkdtemp(prefix="c64xemu-ibl-")
    emulator_socket = os.path.join(directory, "emu.sock")
    peer_socket = os.path.join(directory, "peer.sock")
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    sock.bind(peer_socket)
    sock.setblocking(False)
    with open(arguments.blob, "rb") as blob:
        contents = blob.read()

    # A state file left by an earlier run must not stand in for this one's.
    if os.path.exists(arguments.state_out):
        os.remove(arguments.state_out)
    # The emulator writes CIO output synchronously, so a pipe nobody reads until exit would fill
    # and block it mid-boot - and it would then never exit. stdout is discarded (the state file
    # carries the CIO output), and stderr goes to a file, which takes any amount without a
    # reader and is read back once the run is over.
    errors = tempfile.TemporaryFile(mode="w+")
    emulator = subprocess.Popen([
        arguments.emulator, "run", arguments.image, "--device", "c6472", "--boot-mode", "2",
        "--i2c-eeprom", arguments.eeprom, "--max-cycles", str(arguments.max_cycles),
        "--state-out", arguments.state_out, "--net-unix", emulator_socket,
        "--net-peer", peer_socket,
    ], stdout=subprocess.DEVNULL, stderr=errors, text=True)
    network = Network(sock, emulator_socket)
    server = Server(network, arguments.file_name, contents)
    deadline = time.monotonic() + arguments.timeout
    timed_out = False
    while emulator.poll() is None:
        if time.monotonic() > deadline:
            emulator.kill()
            timed_out = True
            break
        if not os.path.exists(emulator_socket):
            time.sleep(0.05)
            continue
        for frame in network.poll(0.05):
            server.handle(frame)
        server.tick()
    emulator.wait()
    errors.seek(0)
    stderr = errors.read()
    errors.close()
    # The emulator can send the last ACK and run the program to its end between two polls, so
    # whatever it sent before exiting is still queued here: read it before deciding.
    network.closed = True
    for frame in network.poll(0):
        server.handle(frame)
    sock.close()
    # TFTP completing says the IBL fetched the file, not that what it fetched ran to its end:
    # the boot succeeded only if the emulator stopped because the program reached its exit.
    state = {}
    if os.path.exists(arguments.state_out):
        with open(arguments.state_out) as state_file:
            state = json.load(state_file)
    halted = state.get("halted") is True and state.get("fault") is None
    print(json.dumps({
        "steps": network.steps,
        "complete": server.complete,
        "retransmits": server.retransmits,
        "errors": server.errors,
        "timed_out": timed_out,
        "halted": halted,
        "emulator_status": emulator.returncode,
        "emulator_stderr": stderr[-2000:],
    }))
    return 0 if server.complete and not server.errors and not timed_out and halted else 1


if __name__ == "__main__":
    sys.exit(main())
