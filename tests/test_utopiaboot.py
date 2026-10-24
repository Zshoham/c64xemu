"""A C6472 executes an image delivered only through PHY-addressed UTOPIA cells."""
from __future__ import annotations

import json
from pathlib import Path
import socket
import subprocess
import sys
import time
from typing import Callable

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


UCR = 0x02B40000
UCR_BEND = 1 << 31
UCR_U16M = 1 << 13
UCR_UREN = 1 << 0


def boot_from_eeprom(
    candidate_binary: Path, image: Path, tmp_path: Path, eeprom_options: list[str],
    phy: int, width: int, cell_size: int, probe: Callable[[socket.socket, str, float], None],
) -> tuple[dict, dict]:
    """Boot a C6472 over I2C from an EEPROM whose parameters hand over to UTOPIA, run `probe`
    once the port listens at `phy`, send the image as `cell_size` cells there, and return the
    emulator's state and the sender's report."""
    eeprom = tmp_path / "eeprom.bin"
    built = subprocess.run([
        sys.executable, str(ROOT / "tests/tools/i2c_eeprom.py"), "--image", str(image),
        "--out", str(eeprom), "--utopia", "--width", str(width), "--cell-size", str(cell_size),
        *eeprom_options,
    ], cwd=ROOT, capture_output=True, text=True, timeout=60)
    assert built.returncode == 0, built.stderr
    emulator_path = str(tmp_path / "emu.sock")
    state_path = tmp_path / "state.json"
    emulator = subprocess.Popen([
        str(candidate_binary), "run", str(image), "--device", "c6472", "--boot-mode", "2",
        "--i2c-eeprom", str(eeprom), "--utopia-unix", emulator_path, "--utopia-poll", "256",
        "--max-cycles", "100000000", "--state-out", str(state_path),
    ], cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sock:
            sock.bind(str(tmp_path / "probe.sock"))
            deadline = time.monotonic() + 60
            while exchange(sock, emulator_path, b"p" + bytes((phy,)), deadline) != b"\x01":
                assert time.monotonic() < deadline, "UTOPIA never enabled"
            probe(sock, emulator_path, deadline)
        sender = subprocess.run([
            sys.executable, str(ROOT / "tests/tools/utopia_boot.py"),
            "--socket", str(tmp_path / "host.sock"), "--peer", emulator_path,
            "--image", str(image), "--phy", str(phy), "--width", str(width),
            "--cell-size", str(cell_size), "--json",
        ], cwd=ROOT, capture_output=True, text=True, timeout=90)
        assert sender.returncode == 0, sender.stderr
        stdout, stderr = emulator.communicate(timeout=120)
        assert state_path.is_file(), f"no boot state: {stderr}{stdout}"
    finally:
        if emulator.poll() is None:
            emulator.kill()
            emulator.communicate()
    return json.loads(state_path.read_text()), json.loads(sender.stdout)


def assert_booted(state: dict, report: dict, discarded: int) -> None:
    boot = state["utopia_boot"]
    assert boot["error"] is None
    assert boot["entry"] == report["entry"]
    assert boot["cells_accepted"] == report["cells"]
    assert boot["cells_discarded"] == discarded
    assert state["i2c_boot"]["error"] is None
    assert state["i2c_boot"]["entry"] is None, "the I2C boot handed over rather than branching"
    assert state["exit_code"] == 55
    assert state["fault"] is None
    assert all(core["held_in_reset"] and core["cycles"] == 0 for core in state["cores"][1:])


# SPRUEC6G Tables 24 and 26: the EEPROM's second parameter block names UTOPIA with a cell size
# other than the standard one, a slave ID, a core frequency, and either multi-PHY on an 8-bit bus
# or single-PHY on a 16-bit one.
@pytest.mark.parametrize("width,cell_size,single_phy", [(8, 64, False), (16, 60, True)])
def test_eeprom_parameters_select_and_configure_the_utopia_boot(
    candidate_binary: Path, image: Path, tmp_path: Path,
    width: int, cell_size: int, single_phy: bool,
) -> None:
    slave = 9

    def probe(sock: socket.socket, peer: str, deadline: float) -> None:
        # Multi-PHY answers its slave ID alone; single-PHY decodes no address at all.
        expected = b"\x01" if single_phy else b""
        assert exchange(sock, peer, b"p\x08", deadline) == expected
        # A standard cell is padded at its end to the configured size, so its magic no longer
        # sits where the ROM looks: the cell size is what the ROM searches by.
        send(sock, peer, slave, cell(bytes(32), width), deadline)

    options = ["--slave-id", str(slave), "--core-mhz", "250"]
    if single_phy:
        options.append("--single-phy")
    state, report = boot_from_eeprom(
        candidate_binary, image, tmp_path, options, slave, width, cell_size, probe,
    )
    assert_booted(state, report, discarded=1)
    assert state["i2c_boot"]["parameter_blocks"] == 2


# Table 26 bit 2 skips port initialisation. The EEPROM's boot configuration table (options 2,
# section 6.2.3) brings the port up at PHY 7 with four user-defined header bytes - a 57-byte cell -
# before the handover, and the UTOPIA parameters' own slave ID (2) is never applied.
def test_a_skipped_port_initialisation_keeps_the_eeprom_configuration(
    candidate_binary: Path, image: Path, tmp_path: Path,
) -> None:
    phy = 7
    set_mask = UCR_BEND | (phy << 24) | (4 << 2) | UCR_UREN

    def probe(sock: socket.socket, peer: str, deadline: float) -> None:
        assert exchange(sock, peer, b"p\x02", deadline) == b""

    state, report = boot_from_eeprom(
        candidate_binary, image, tmp_path,
        ["--skip-port-init", "--slave-id", "2", "--config", f"{UCR},{set_mask},{UCR_U16M}"],
        phy, 8, 57, probe,
    )
    assert_booted(state, report, discarded=0)
    assert state["i2c_boot"]["config_blocks"] == 1
    assert state["i2c_boot"]["parameter_blocks"] == 2
