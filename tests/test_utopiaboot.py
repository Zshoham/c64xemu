"""A C6472 executes an image delivered only through PHY-addressed UTOPIA cells."""
from __future__ import annotations

import json
from pathlib import Path
import socket
import subprocess
import sys
import time

import pytest

from common import ROOT
from target_programs import build_target_program, load_programs
from tools.utopia_boot import cell, exchange, send

pytestmark = [pytest.mark.slow, pytest.mark.host]


@pytest.fixture(scope="module", params=["sum_loop", "big_endian"])
def image(request: pytest.FixtureRequest, tmp_path_factory: pytest.TempPathFactory) -> Path:
    program = next(item for item in load_programs() if item.name == request.param)
    built = build_target_program(program, tmp_path_factory.mktemp(f"utopia-{request.param}"))
    assert built is not None, f"could not build {request.param}"
    return built


@pytest.mark.parametrize("mode,width", [(5, 8), (8, 16)])
def test_boot_ignores_other_phys_and_invalid_cells_then_executes(
    candidate_binary: Path, image: Path, tmp_path: Path, mode: int, width: int,
) -> None:
    emulator_path = str(tmp_path / "emu.sock")
    state_path = tmp_path / "state.json"
    emulator = subprocess.Popen([
        str(candidate_binary), "run", str(image), "--device", "c6472", "--boot-mode", str(mode),
        "--utopia-unix", emulator_path, "--utopia-phy", "7", "--utopia-poll", "256",
        "--max-cycles", "100000000", "--state-out", str(state_path),
    ], cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.bind(str(tmp_path / "probe.sock"))
            deadline = time.monotonic() + 60
            # Wait until ROM initialization has enabled this slave, not just the socket.
            while exchange(sock, emulator_path, b"p\x07", deadline) != b"\x01":
                assert time.monotonic() < deadline, "UTOPIA never enabled"
            assert exchange(sock, emulator_path, b"p\x06", deadline) == b""
            assert exchange(sock, emulator_path, b"w\x06" + cell(bytes(32), width), deadline) == b"\0"
            # A correctly addressed cell reaches PDMA but cannot become boot-table bytes
            # without the boot magic. It must not corrupt the entry/section parser.
            send(sock, emulator_path, 7, bytes(53 if width == 8 else 54), deadline)
        sender = subprocess.run([
            sys.executable, str(ROOT / "tests/tools/utopia_boot.py"),
            "--socket", str(tmp_path / "host.sock"), "--peer", emulator_path,
            "--image", str(image), "--phy", "7", "--width", str(width), "--json",
        ], cwd=ROOT, capture_output=True, text=True, timeout=90)
        assert sender.returncode == 0, sender.stderr
        report = json.loads(sender.stdout)
        stdout, stderr = emulator.communicate(timeout=120)
        assert state_path.is_file(), f"no boot state: {stderr}{stdout}"
    finally:
        if emulator.poll() is None:
            emulator.kill()
            emulator.communicate()
    state = json.loads(state_path.read_text())
    boot = state["utopia_boot"]
    assert boot["error"] is None
    assert boot["entry"] == report["entry"]
    assert boot["cells_accepted"] == report["cells"]
    assert boot["cells_discarded"] == 1
    assert boot["table_bytes"] == report["table_bytes"]
    assert state["exit_code"] == 55
    assert state["fault"] is None
    assert state["cores"][0]["held_in_reset"] is False
    assert all(core["held_in_reset"] and core["cycles"] == 0 for core in state["cores"][1:])
