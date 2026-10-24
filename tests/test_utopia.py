"""Both devices echo ATM cells to an external master through their UTOPIA queues.

The C6455 firmware copies each cell from the receive queue to the transmit queue with the CPU.
The C6472 has no CPU window onto either queue, so its firmware moves every cell receive queue ->
PDMA -> memory -> PDMA -> transmit queue; what the master reads back with the `x`/`r` exchange
is then the PDMA transmit path's own output.
"""
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

SLAVE = 9
ECHOES = 3
# Per device: the firmware, and the machine options that run it. On the C6472, boot mode 4 (I2C
# slave, which has no modelled boot source) loads the ELF and brings only core 0 out of reset.
DEVICES = {
    "c6455": ("utopia_echo", ()),
    "c6472": ("c6472_utopia_echo", ("--boot-mode", "4")),
}


@pytest.fixture(scope="module", params=sorted(DEVICES))
def device(request: pytest.FixtureRequest, tmp_path_factory: pytest.TempPathFactory) -> tuple[str, Path]:
    name = request.param
    firmware = DEVICES[name][0]
    # The project's minimal start-up (`tests/programs/common/crt.asm`) halts with main's return
    # value in A4, which the state report gives as the exit code.
    program = Program(
        name=firmware, weight=0, sources=(str(ROOT / "tests/firmware" / firmware / "main.c"),),
        includes=(), cflags=("-O2",), link_flags=(), entry="_start", max_cycles=200_000_000,
        opt_levels=("-O2",), rts=False, expect={}, cio_output=None, rx_frames=(),
        tx_frames=None, boot_table=None, boot_config=None,
    )
    built = build_target_program(program, tmp_path_factory.mktemp(firmware))
    assert built is not None, f"could not build {firmware}"
    return name, built


def test_cells_written_by_the_master_are_read_back_unchanged(
    candidate_binary: Path, device: tuple[str, Path], tmp_path: Path,
) -> None:
    name, image = device
    emulator_path = str(tmp_path / "emu.sock")
    state_path = tmp_path / "state.json"
    emulator = subprocess.Popen([
        str(candidate_binary), "run", str(image), "--device", name, *DEVICES[name][1],
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
            assert exchange(sock, emulator_path, bytes((ord("x"), SLAVE)), deadline) == b"\0"
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
