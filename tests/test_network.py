from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import pytest

from common import NDK_IMAGE, ROOT, Case, assert_case, case_id, run_tool, selected_case_ids

TCP_PEER = ROOT / "tests/tools/tcp_peer.py"

# Paced TCP rates at or above this are marked `high_bandwidth`: CI deselects them because a
# shared runner cannot be relied on to sustain the emulator's pace. The local suite keeps them.
HIGH_BANDWIDTH_KBPS = 1024

def network_cases(
    candidate: Path | None, selected_ids: set[str] | None = None
) -> tuple[list[Case], dict[str, Any]]:
    """Hold real TCP connections with TI's NDK stack running as firmware.

    A host process speaks raw Ethernet frames to the emulator over its Unix datagram backend:
    ARP, then a three-way handshake to the echo server on port 3000, a payload, the echo back
    and a FIN exchange. Everything asserted here is content rather than timing - the bytes that
    came back and the lines the firmware itself printed - so the cases do not depend on how many
    cycles the host took to answer.

    Two exchanges: one segment, and 64 KB in both directions at once, which is what makes the
    receive and transmit rings wrap, the window fill and the stack's ACK path work for real.
    Three further contracts sit on top of the same connection machinery: an ICMP echo round
    trip, a SYN to a closed port that must come back as a reset, and a TCP payload whose
    segments arrive out of order so the stack's reassembly queue has to put them back.
    Finally, a bandwidth set paced over the powers of two from 16 to 4096 kbps, one TCP
    stream and one UDP burst per rate: the peer sends exactly one second's worth of traffic
    at the configured rate and requires the whole exchange to complete, so each case is the
    claim that the emulator sustains that rate.
    """
    # The firmware has no static address: its DHCP client asks for one and the peer's DHCP server
    # leases it, so every exchange below starts with the four-step handshake of RFC 2131 section
    # 4.3. The echo task is listening before the address arrives, which is the order these lines
    # come out in.
    dhcp_steps = ["discover", "offer", "request", "ack"]
    echo_lines = [
        "c64xemu ndk firmware starting",
        # The network-open hook registers TI's own daemons before it creates the application's
        # tasks, so its line comes out before theirs.
        "servers: ti daemons on ports 7 and 1000",
        "echo: listening on port 3000",
        "udp: listening on port 3001",
        "nocopy: listening on port 3002",
        "Network Added: If-1:192.168.64.10",
        "ndk: interface 1 up with 192.168.64.10",
    ]

    def echo_expected(length: int, dropped: list[int]) -> dict[str, Any]:
        return {
            "state": "closed",
            "echoed": length,
            "payload_matched": True,
            "dsp_mac": "02:00:00:11:22:33",
            "dropped": dropped,
            "dhcp": dhcp_steps,
            "firmware": echo_lines
            + [
                "echo: connection from 192.168.64.1:40000",
                f"echo: connection closed after {length} bytes",
                # The server's own "done" line is printed after the exchange this case asserts,
                # once the peer has closed and stopped listening for output, so whether it
                # reaches the harness depends on when the emulator is stopped rather than on
                # anything the firmware did. The connection's content is the contract.
            ],
        }

    # The NDK's own HTTP server, serving a page the application put in the embedded file system.
    # The reply is the server's, not an echo, so the exact header it writes is the expectation:
    # sixty-five bytes of header and the seventy-five byte page.
    http_reply = (
        "HTTP/1.0 200 OK\r\nContent-type: text/html \r\nContent-length: 75\r\n\r\n"
        "<html><body><h1>c64xemu</h1><p>"
    )
    # TI's own server daemons out of `tools/servers`, which the `servers_min_ipv4` library this
    # firmware links has always contained and which nothing here had ever started. The
    # expectation for each is TI's code's, not this project's: `dtask_tcp_datasrv` replies to a
    # four-byte little-endian length with that many bytes out of a buffer it fills with
    # `data[i] = ' ' + i`, in 512-byte blocks, and `tests/tools/tcp_peer.py` reconstructs that
    # rule from the daemon's own source. Registering the echo daemon took reading its source
    # too: its header says "(SOCK_STREAMNC, port 7)" because it reads with `recvnc`, and on an
    # ordinary stream socket it accepts the connection and then echoes nothing.
    def daemon_expected(length: int) -> dict[str, Any]:
        return {
            "state": "closed",
            "echoed": length,
            "payload_matched": True,
            "dsp_mac": "02:00:00:11:22:33",
            "dhcp": dhcp_steps,
            "firmware": echo_lines,
        }

    exchanges = (
        ("tcp-echo", [], echo_expected(28, [])),
        # 4096 is eight whole 512-byte blocks, so the daemon's own `remainder` branch - which
        # sends bytes out of the request word rather than the buffer - does not come into it.
        ("ti-datasrv", ["--port", "1000", "--datasrv", "4096"], daemon_expected(4096)),
        ("ti-tcp-echo", ["--port", "7", "--bytes", "4096"], daemon_expected(4096)),
        ("tcp-bulk", ["--bytes", "65536"], echo_expected(65536, [])),
        # One echoed segment is dropped and never acknowledged, so the only way the stream can
        # complete is the firmware's own retransmission timer expiring - which puts the timer,
        # the SYS/BIOS clock it drives and TCP's recovery path in the loop.
        (
            "tcp-retransmit",
            ["--bytes", "4096", "--drop-echo", "2"],
            echo_expected(4096, [2]),
        ),
        # The stack's zero-copy receive path: `recvncfrom` hands the task a pointer into the
        # stack's own buffer and a handle to return with `recvncfree`, so the datagram is echoed
        # without a copy. It is datagram-only - SockRecvNC refuses a socket that is not an atomic
        # reader, which is why the TCP echo above cannot use it.
        (
            "nocopy-datagram",
            ["--udp", "zero copy datagram", "--udp-port", "3002"],
            {
                "state": "closed",
                "echoed": 18,
                "dhcp": dhcp_steps,
                "firmware": echo_lines + ["nocopy: echoed 18 bytes without a copy"],
            },
        ),
        (
            "udp-echo",
            ["--udp", "datagram over the emulated EMAC"],
            {
                "state": "closed",
                "echoed": 31,
                "dhcp": dhcp_steps,
                "firmware": echo_lines + ["udp: echoed 31 bytes"],
            },
        ),
        # One ICMP echo request and the stack's reply: NDK's own icmpin.c reflects the request
        # with the identifier, sequence and payload intact, so a matching reply is the whole
        # contract. This is the raw ICMP receive/reply path through the EMAC, not socket code.
        (
            "icmp-ping",
            ["--icmp"],
            {
                "state": "closed",
                "icmp_reply": True,
                "dhcp": dhcp_steps,
                "firmware": echo_lines,
            },
        ),
        # A SYN to a port nothing is listening on must come back as a reset: NDK's TCP refuses
        # the connection with RST|ACK (tcpin.c's dropwithreset path), and the refusal is the
        # contract - a stack that silently dropped the SYN would hang every client forever.
        (
            "tcp-reset",
            ["--port", "4001", "--expect-reset"],
            {
                "state": "reset",
                "dhcp": dhcp_steps,
                "firmware": echo_lines,
            },
        ),
        # The peer sends the payload's segments newest-first and never retransmits them, so the
        # only way the echo completes is NDK's TCP reassembly queue (tcpin.c's pReasm) holding
        # the later segments until their predecessors arrive.
        (
            "tcp-out-of-order",
            ["--bytes", "4096", "--out-of-order"],
            echo_expected(4096, []),
        ),
        # TI's own telnet daemon with TI's own console behind it: the daemon accepts the
        # connection, calls ConsoleOpen, and the console task talks over a pipe. The dialogue is
        # driven by content, not timing - the peer waits for each marker before typing the next
        # line - so the whole session is exact: the banner the application owns, the welcome the
        # console prints with the peer's address, the help listing its own source spells out, and
        # the goodbye it sends before closing.
        (
            "telnet-console",
            ["--port", "23", "--telnet", "--payload", ""],
            {
                "state": "closed",
                "echoed": 1069,
                "console": [
                    "c64xemu NDK console",
                    "Welcome connection : 192.168.64.1:40000",
                    "Welcome to the console program.",
                    "Enter '?' or 'help' for a list of commands.",
                    ">?",
                    "c64xemu NDK console",
                    "[Help Command]",
                    "The basic commands are:",
                    "acct     - Manage PPP user accounts",
                    "bye      - Logoff the console",
                    "echo     - Perform echo test",
                    "help     - Displays this message",
                    "mem      - Display memory status",
                    "nslookup - Lookup hostname or IP address",
                    "pswd     - Change console password",
                    "ping     - Test echo request",
                    "quit     - Logoff the console",
                    "reboot   - Reboot system (terminates session)",
                    "route    - Maintain route table",
                    "shutdown - Shutdown stack (terminates session)",
                    "socket   - Print socket table",
                    "stat     - Print internal stack statistics",
                    "tftp     - Test TFTP file transfer",
                    "lli      - Test Static LLI (ARP) Configuration",
                    "vlan     - Add/Delete VLAN devices",
                    "ipaddr   - Configuration of IPAddress",
                    "Some commands have additional help information. For example",
                    "entering 'route' gives more information on the route command.",
                    ">quit",
                    "Goodbye",
                ],
                "dhcp": dhcp_steps,
                "firmware": echo_lines,
            },
        ),
        (
            "http-get",
            [
                "--port",
                "80",
                "--request",
                "GET /index.html HTTP/1.0\\r\\n\\r\\n",
                "--expect",
                "c64xemu",
            ],
            {
                "state": "closed",
                "echoed": 140,
                "expected_present": True,
                "reply_head": http_reply,
                "dhcp": dhcp_steps,
                "firmware": echo_lines,
            },
        ),
    )
    cases: list[Case] = []
    results: dict[str, Any] = {}
    # The lease itself is a contract of its own, and the first exchange already performs it, so it
    # is asserted from that run rather than by handshaking again.
    lease_case = Case(
        request={"id": "network:ndk-dhcp-lease"},
        expected={
            "dhcp": dhcp_steps,
            "firmware": echo_lines
            + [
                "echo: connection from 192.168.64.1:40000",
                "echo: connection closed after 28 bytes",
            ],
        },
        weight=100,
        domain="network",
    )
    baseline_failure: str | None = None

    def run_exchange(case_id: str, extra: list[str]) -> dict[str, Any] | None:
        """Run the peer for one case and return its report, or None if it cannot run."""
        if candidate is None or not NDK_IMAGE.is_file() or not TCP_PEER.is_file():
            return None
        executed = run_tool(
            [
                sys.executable,
                str(TCP_PEER),
                "--emulator",
                str(candidate),
                "--image",
                str(NDK_IMAGE),
                "--report",
                "--timeout",
                "180",
                *extra,
            ],
            ROOT,
            timeout=240,
        )
        if executed is None:
            return {"runner_error": "peer process produced no result"}
        try:
            return json.loads(executed.stdout.strip().splitlines()[-1])
        except (json.JSONDecodeError, IndexError):
            return {
                "runner_error": "peer process returned invalid JSON",
                "exit": executed.returncode,
            }

    for name, extra, expected in exchanges:
        case_id = f"network:ndk-{name}"
        cases.append(
            Case(request={"id": case_id}, expected=expected, weight=120, domain="network")
        )
        requested = selected_ids is None or case_id in selected_ids
        if name == "tcp-echo" and lease_case.request["id"] in (selected_ids or set()):
            requested = True
        if requested and baseline_failure is not None:
            results[case_id] = {"blocked_by": baseline_failure}
            continue
        if not requested:
            continue
        report = run_exchange(case_id, extra)
        if report is None:
            continue
        results[case_id] = report
        if name == "tcp-echo":
            results[lease_case.request["id"]] = report
            if (
                report.get("runner_error")
                or report.get("state") != "closed"
                or report.get("payload_matched") is False
            ):
                baseline_failure = case_id
    #
    # TCP keeps the full range through 4096 kbps. UDP is capped at 512 kbps: above that the
    # NDK firmware's UDP socket receive queue overflows once the paced datagram rate outruns
    # the echo task's drain at the emulator's speed, and the stack drops datagrams - faithful
    # modelling, but not a useful regression claim. The rates kept still prove the whole
    # receive-and-echo path end to end. The paced TCP cases at `HIGH_BANDWIDTH_KBPS` and above
    # carry the `high_bandwidth` mark so CI can deselect them.
    tcp_bandwidth_rates = (16, 32, 64, 128, 256, 512, 1024, 2048, 4096)
    udp_bandwidth_rates = (16, 32, 64, 128, 256, 512)
    for rate in tcp_bandwidth_rates:
        tcp_payload = int(rate * 125.0)
        for name, extra, expected in (
            (
                f"bw-tcp-{rate}k",
                ["--bandwidth-kbps", str(rate), "--duration", "1.0"],
                echo_expected(tcp_payload, []),
            ),
        ):
            case_id = f"network:ndk-{name}"
            request = {"id": case_id}
            if rate >= HIGH_BANDWIDTH_KBPS:
                request["high_bandwidth"] = True
            cases.append(
                Case(request=request, expected=expected, weight=120, domain="network")
            )
            if selected_ids is not None and case_id not in selected_ids:
                continue
            if baseline_failure is not None:
                results[case_id] = {"blocked_by": baseline_failure}
                continue
            report = run_exchange(case_id, extra)
            if report is not None:
                results[case_id] = report
    for rate in udp_bandwidth_rates:
        udp_datagrams = int(rate * 125.0 / 500)
        name = f"bw-udp-{rate}k"
        extra = ["--udp-burst", "--bandwidth-kbps", str(rate), "--duration", "1.0"]
        expected = {
            "state": "closed",
            "udp_sent": udp_datagrams,
            "udp_echoed": udp_datagrams,
            "dhcp": dhcp_steps,
            "firmware": echo_lines + ["udp: echoed 500 bytes"],
        }
        case_id = f"network:ndk-{name}"
        cases.append(
            Case(request={"id": case_id}, expected=expected, weight=120, domain="network")
        )
        if selected_ids is not None and case_id not in selected_ids:
            continue
        if baseline_failure is not None:
            results[case_id] = {"blocked_by": baseline_failure}
            continue
        report = run_exchange(case_id, extra)
        if report is not None:
            results[case_id] = report
    cases.append(lease_case)
    return cases, results

NETWORK_CASES = network_cases(None)[0]

@pytest.fixture(scope="session")
def network_results(
    request: pytest.FixtureRequest, candidate_binary: Path
) -> dict[str, Any]:
    selected_ids = selected_case_ids(request, "test_network_case")
    return network_cases(candidate_binary, selected_ids)[1]

@pytest.mark.slow
@pytest.mark.network
@pytest.mark.parametrize(
    "case",
    [
        pytest.param(
            case,
            marks=(pytest.mark.high_bandwidth,) if case.request.get("high_bandwidth") else (),
            id=case_id(case),
        )
        for case in NETWORK_CASES
    ],
)
def test_network_case(case: Case, network_results: dict[str, Any]) -> None:
    assert_case(case, network_results)
