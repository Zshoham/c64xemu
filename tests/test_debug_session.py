"""The debug-session harness behind every front end, exercised through two transports.

One scenario - break at main, continue to the halt, step off it, continue to the exit -
runs once over the console REPL's stdin and once over the GDB stub's TCP socket. Both
front ends are thin adapters onto the same machine, so the halt rule, the stop reasons
and the reported state must agree; the transport abstraction is the point, not the
protocol details.
"""

from __future__ import annotations

import os
import re
import select
import socket
import struct
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import pytest

from common import ROOT, run_tool
from target_programs import build_target_program, load_programs

# The program the session runs: small, deterministic, no peripherals, exits through
# C$$EXIT with A4 = 55 (the hand-derived sum of 1..10).
PROGRAM = "sum_loop"
EXPECTED_EXIT_CODE = 55


def symbol_address(image: Path, names: tuple[str, ...]) -> int | None:
    """The value of the first function symbol in `names`, from a TI EABI ELF32 image.

    The images are linked with `--symdebug:none`, so there is no DWARF; the symbol
    table is the only place a debugger can learn where `main` lives.
    """
    data = image.read_bytes()
    (_, _, _, _, _, _, e_shoff, _, _, _, _, e_shentsize, e_shnum, _) = struct.unpack_from(
        "<16sHHIIIIIHHHHHH", data, 0
    )
    sections = [
        struct.unpack_from("<IIIIIIIIII", data, e_shoff + index * e_shentsize)
        for index in range(e_shnum)
    ]
    strtabs = {
        index: data[sh_offset : sh_offset + sh_size]
        for index, (_, sh_type, _, _, sh_offset, sh_size, _, _, _, _) in enumerate(sections)
        if sh_type == 3  # SHT_STRTAB
    }
    for (_, sh_type, _, _, sh_offset, sh_size, sh_link, _, _, _) in sections:
        if sh_type != 2:  # SHT_SYMTAB
            continue
        strings = strtabs.get(sh_link)
        if strings is None:
            continue
        for entry in range(sh_size // 16):
            st_name, st_value, _, st_info, _, _ = struct.unpack_from(
                "<IIIBBH", data, sh_offset + entry * 16
            )
            if st_info & 0xF != 2:  # STT_FUNC
                continue
            end = strings.find(b"\0", st_name)
            if end < 0:
                continue
            if strings[st_name:end].decode("ascii", "replace") in names:
                return st_value
    return None


@dataclass(frozen=True)
class Stop:
    """Why a run stopped, as both front ends report it."""

    outcome: str  # "breakpoint" | "exit" | "fault" | "budget"
    pc: int | None
    exit_code: int | None


class DebugSession:
    """One debug session over a transport. Subclasses implement the wire details."""

    def __init__(self, candidate: Path, image: Path):
        self.candidate = candidate
        self.image = image

    def __enter__(self) -> "DebugSession":
        self.start()
        return self

    def __exit__(self, *exc: object) -> bool:
        self.close()
        return False

    def start(self) -> None:
        raise NotImplementedError

    def break_at(self, address: int) -> None:
        raise NotImplementedError

    def continue_run(self) -> Stop:
        raise NotImplementedError

    def step(self) -> int:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


class ReplSession(DebugSession):
    """The console REPL: commands on stdin, output up to the next prompt."""

    PROMPT = b"(c64xemu) "

    def start(self) -> None:
        self.process = subprocess.Popen(
            [str(self.candidate), "debug", str(self.image)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=ROOT,
        )
        self._read_until_prompt()

    def _read_until_prompt(self, timeout: float = 60.0) -> str:
        output = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ready, _, _ = select.select([self.process.stdout], [], [], 0.2)
            if ready:
                chunk = os.read(self.process.stdout.fileno(), 4096)
                if not chunk:
                    break
                output += chunk
                if output.endswith(self.PROMPT):
                    return output.decode("utf-8", "replace")
        raise TimeoutError(f"REPL produced no prompt; output so far: {output!r}")

    def command(self, line: str) -> str:
        self.process.stdin.write(line.encode() + b"\n")
        self.process.stdin.flush()
        return self._read_until_prompt()

    def break_at(self, address: int) -> None:
        self.command(f"break 0x{address:x}")

    def _pc(self, output: str) -> int:
        match = re.search(r"pc=0x([0-9a-f]+)", output)
        assert match is not None, f"no pc in REPL output: {output!r}"
        return int(match.group(1), 16)

    def continue_run(self) -> Stop:
        output = self.command("continue")
        exit_match = re.search(r"program exited with code (\d+)", output)
        if exit_match is not None:
            return Stop("exit", pc=None, exit_code=int(exit_match.group(1)))
        return Stop("breakpoint", pc=self._pc(output), exit_code=None)

    def step(self) -> int:
        return self._pc(self.command("stepi"))

    def close(self) -> None:
        try:
            self.process.stdin.close()
        except BrokenPipeError:
            pass
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()


class GdbSession(DebugSession):
    """The GDB remote serial protocol stub over TCP."""

    def start(self) -> None:
        # The stub ships register values in the target's byte order (ELF e_ident[5]).
        self.endian = "big" if self.image.read_bytes()[5] == 2 else "little"
        self.process = subprocess.Popen(
            [str(self.candidate), "debug", "--gdb", "127.0.0.1:0", str(self.image)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=ROOT,
        )
        line = self.process.stdout.readline().decode().strip()
        assert line.startswith("GDB listening on "), f"unexpected stub banner: {line!r}"
        host, port = line[len("GDB listening on ") :].rsplit(":", 1)
        self.socket = socket.create_connection((host, int(port)), timeout=60)
        self.socket.settimeout(60)

    def _packet(self, payload: str) -> str:
        checksum = sum(payload.encode()) & 0xFF
        self.socket.sendall(f"${payload}#{checksum:02x}".encode())
        # The stub acknowledges each packet, then replies as "$payload#checksum".
        while True:
            byte = self.socket.recv(1)
            if not byte:
                raise ConnectionError("GDB stub closed the connection")
            if byte == b"$":
                break
        reply = b""
        while True:
            byte = self.socket.recv(1)
            if not byte:
                raise ConnectionError("GDB stub closed the connection")
            if byte == b"#":
                reply += self.socket.recv(2)
                break
            reply += byte
        return reply[:-2].decode()

    def break_at(self, address: int) -> None:
        reply = self._packet(f"Z0,{address:x},4")
        assert reply == "OK", f"breakpoint insert refused: {reply!r}"

    def _pc(self) -> int:
        # Register numbers are hex on the wire: 0x21 is the PC (33 decimal); the bytes are
        # in the target's byte order.
        reply = self._packet("p21")
        assert len(reply) == 8, f"unexpected pc reply: {reply!r}"
        return int.from_bytes(bytes.fromhex(reply), self.endian)

    def continue_run(self) -> Stop:
        reply = self._packet("c")
        if reply.startswith("W"):
            return Stop("exit", pc=None, exit_code=int(reply[1:], 16))
        assert reply in ("S05", "S02", "S04"), f"unexpected stop reply: {reply!r}"
        return Stop("breakpoint", pc=self._pc(), exit_code=None)

    def step(self) -> int:
        reply = self._packet("s")
        assert reply == "S05", f"unexpected step reply: {reply!r}"
        return self._pc()

    def close(self) -> None:
        self.socket.close()
        self.process.terminate()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()


def make_session(candidate: Path, image: Path, transport: str) -> DebugSession:
    if transport == "repl":
        return ReplSession(candidate, image)
    if transport == "gdb":
        return GdbSession(candidate, image)
    raise ValueError(f"unknown transport {transport}")


@pytest.fixture(scope="module")
def session_image(tmp_path_factory: pytest.TempPathFactory) -> Path:
    workdir = tmp_path_factory.mktemp("debug-session")
    program = next(program for program in load_programs() if program.name == PROGRAM)
    image = build_target_program(program, workdir)
    assert image is not None, f"failed to build {PROGRAM} for the debug session"
    return image


@pytest.mark.parametrize(
    "transport", ["repl", "gdb"], ids=["console-repl-stdin", "gdb-stub-tcp"]
)
def test_debug_session(
    candidate_binary: Path, session_image: Path, transport: str
) -> None:
    """The halt rule, the stop reasons and the reported state agree across transports."""
    main = symbol_address(session_image, ("main", "_c_int00"))
    assert main is not None, "no main symbol in the session image"

    with make_session(candidate_binary, session_image, transport) as session:
        # Break at main; the run halts at the fetch boundary, before main executes.
        session.break_at(main)
        stop = session.continue_run()
        assert stop.outcome == "breakpoint"
        assert stop.pc == main

        # Stepping moves off the breakpoint: the halt rule dropped the current address.
        pc = session.step()
        assert pc != main

        # Continuing reaches the exit point, which the halt rule added automatically.
        stop = session.continue_run()
        assert stop.outcome == "exit"
        assert stop.exit_code == EXPECTED_EXIT_CODE
