"""The TFTP server TI's IBL boots from: a whole file over the Unix datagram transport, its
16-bit block number wrapping while the file offset keeps counting."""
from __future__ import annotations

from pathlib import Path
import select
import socket
import struct
import tempfile
from types import SimpleNamespace

import pytest

from tools import ibl_peer

DSP_PORT = 1234


@pytest.mark.parametrize(
    "full_blocks, tail",
    [(65535, b""), (131072, b"tail")],
    ids=["empty-final-block-zero", "short-final-block-after-two-wraps"],
)
def test_tftp_rollover_preserves_the_file(
    full_blocks: int, tail: bytes, monkeypatch: pytest.MonkeyPatch,
) -> None:
    # TI's Uint16 block counter wraps, but the file offset must not. Give every full block
    # its own ordinal so reading from the beginning again after a wrap cannot pass.
    # The server only measures and slices the file, so it gets this buffer itself: converting it
    # to bytes first would hold a second copy of the file (64 MB for two wraps) at the peak.
    contents = bytearray(full_blocks * 512)
    for index in range(full_blocks):
        struct.pack_into("!I", contents, index * 512, index + 1)
    contents.extend(tail)
    now = 0.0
    monkeypatch.setattr(ibl_peer, "time", SimpleNamespace(monotonic=lambda: now))

    # Short paths avoid AF_UNIX's path-length limit even under a long pytest --basetemp.
    with (
        tempfile.TemporaryDirectory(prefix="ibl-") as directory,
        socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as host,
        socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client,
    ):
        host_path = str(Path(directory) / "host.sock")
        client_path = str(Path(directory) / "client.sock")
        host.bind(host_path)
        client.bind(client_path)
        host.settimeout(2)
        client.settimeout(2)
        network = ibl_peer.Network(host, client_path)
        dsp_mac = b"\x02\x00\x00\x00\x64\x72"
        network.dsp_mac = dsp_mac
        server = ibl_peer.Server(network, "big.bin", contents)

        def send_tftp(packet: bytes, port: int = ibl_peer.TFTP_TID) -> None:
            udp = struct.pack("!HHHH", DSP_PORT, port, 8 + len(packet), 0) + packet
            ip = struct.pack("!BBHHHBBH4s4s", 0x45, 0, 20 + len(udp), 0, 0, 64, 17, 0,
                             ibl_peer.DSP_IP, ibl_peer.SERVER_IP)
            ip = ip[:10] + struct.pack("!H", ibl_peer.checksum(ip)) + ip[12:]
            frame = ibl_peer.SERVER_MAC + dsp_mac + struct.pack("!H", 0x0800) + ip + udp
            client.sendto(frame, host_path)
            server.handle(host.recv(65535))

        def receive_tftp() -> bytes:
            frame = client.recv(65535)
            udp = frame[34:]  # Ethernet header and the server's fixed 20-byte IPv4 header.
            return udp[8:struct.unpack("!H", udp[4:6])[0]]

        send_tftp(b"\x00\x01big.bin\x00octet\x00", ibl_peer.TFTP_PORT)
        wire_block = 1
        for ordinal in range(1, full_blocks + 2):
            packet = receive_tftp()
            payload = struct.pack("!I", ordinal) + bytes(508) if ordinal <= full_blocks else tail
            assert packet == struct.pack("!HH", 3, wire_block) + payload
            assert not server.complete

            if wire_block == 0:
                # Neither the previous ACK nor an ACK for the next block advances block zero.
                for wrong_ack in (65535, 1):
                    send_tftp(struct.pack("!HH", 4, wrong_ack))
                    assert not server.complete
                    assert not select.select([client], [], [], 0)[0]
                now += 2.0
                server.tick()
                assert receive_tftp() == packet

            send_tftp(struct.pack("!HH", 4, wire_block))
            assert server.complete == (ordinal == full_blocks + 1)
            wire_block = 0 if wire_block == 65535 else wire_block + 1

        assert server.errors == []
        assert not select.select([client], [], [], 0)[0]
