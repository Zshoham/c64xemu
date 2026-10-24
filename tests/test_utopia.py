"""A C6455 echoes ATM cells to an external master through both UTOPIA queues."""
from __future__ import annotations

import json
from pathlib import Path
import socket
import subprocess
import time

import pytest

from common import ROOT
from target_programs import Program, build_target_program
from tools.utopia_boot import exchange, receive, send

pytestmark = [pytest.mark.slow, pytest.mark.host]

FIRMWARE = ROOT / "tests/firmware/utopia_echo"
SLAVE = 9
ECHOES = 3


@pytest.fixture(scope="module")
def image(tmp_path_factory: pytest.TempPathFactory) -> Path:
    # The project's minimal start-up (`tests/programs/common/crt.asm`) halts with main's return
    # value in A4, which the state report gives as the exit code.
    program = Program(
        name="utopia_echo", weight=0, sources=(str(FIRMWARE / "main.c"),), includes=(),
        cflags=("-O2",), link_flags=(), entry="_start", max_cycles=200_000_000,
        opt_levels=("-O2",), rts=False, expect={}, cio_output=None, rx_frames=(),
        tx_frames=None, boot_table=None, boot_config=None,
    )
    built = build_target_program(program, tmp_path_factory.mktemp("utopia-echo"))
    assert built is not None, "could not build utopia_echo"
    return built


def test_cells_written_by_the_master_are_read_back_unchanged(
    candidate_binary: Path, image: Path, tmp_path: Path,
) -> None:
    emulator_path = str(tmp_path / "emu.sock")
    state_path = tmp_path / "state.json"
    emulator = subprocess.Popen([
        str(candidate_binary), "run", str(image), "--device", "c6455",
        "--utopia-unix", emulator_path, "--utopia-poll", "256",
        "--max-cycles", "200000000", "--state-out", str(state_path),
    ], cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    # Header bytes, then a payload that differs per cell, so a stale or reordered echo shows.
    cells = [bytes(range(1 + index, 54 + index)) for index in range(ECHOES)]
    echoes = []
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.bind(str(tmp_path / "master.sock"))
            deadline = time.monotonic() + 60
            # The receiver answers its address once the firmware has taken it out of reset.
            while exchange(sock, emulator_path, bytes((ord("p"), SLAVE)), deadline) != b"\x01":
                assert time.monotonic() < deadline, "UTOPIA never enabled"
            # Another address is high-impedance on both sides, and nothing is waiting yet.
            assert exchange(sock, emulator_path, bytes((ord("x"), SLAVE + 1)), deadline) == b""
            assert exchange(sock, emulator_path, bytes((ord("r"), SLAVE)), deadline) == b"\0"
            for cell in cells:
                send(sock, emulator_path, SLAVE, cell, deadline)
                echoes.append(receive(sock, emulator_path, SLAVE, deadline))
            send(sock, emulator_path, SLAVE, bytes(53), deadline)
        stdout, stderr = emulator.communicate(timeout=120)
    finally:
        if emulator.poll() is None:
            emulator.kill()
            emulator.communicate()
    assert state_path.is_file(), f"no state: {stderr}{stdout}"
    state = json.loads(state_path.read_text())
    assert echoes == cells
    assert state["exit_code"] == ECHOES
    assert state["fault"] is None
