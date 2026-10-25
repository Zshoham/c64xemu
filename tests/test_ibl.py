"""TI's IBL, built from TI's source with the suite's tools, boots a C6472 end to end.

The ROM's I2C boot loads the IBL's first stage off the EEPROM ``make evm_c6472`` writes; that
stage drives the I2C module itself to read its configuration table and its second stage; the
second stage sets up PLL1 and DDR2 the way TI's C6472 EVM configuration says, takes an address
and a file name over BOOTP and reads the file into DDR2 over TFTP, then branches to it. The
file is a compiled program that returns 55, so the program's exit code says the whole chain
ran - nothing of it is preloaded, and the only bytes that reach the device are the EEPROM and
the network frames.
"""
from __future__ import annotations

import dataclasses
import json
from pathlib import Path
import subprocess
import sys

import pytest

from common import ROOT
from target_programs import build_target_program, load_programs
from tools import ibl_build
from tools.emac_boot import PT_LOAD, parse_elf

pytestmark = [pytest.mark.slow]

# The EVM configuration's TFTP boot loads a binary blob at the base of DDR2 and branches there
# (iblConfig's c6472_ibl_config: startAddress and branchAddress 0xE0000000).
DDR2_BASE = 0xE000_0000
DDR2_LINK = """
MEMORY
{
    DDR2 : origin = 0xE0000000, length = 0x00100000
}

SECTIONS
{
    .entry      > 0xE0000000 { *crt.obj(.text) }
    .text       > DDR2
    .const      > DDR2
    .rodata     > DDR2
    .switch     > DDR2
    .cinit      > DDR2
    .neardata   > DDR2
    .fardata    > DDR2
    .bss        > DDR2
    .far        > DDR2
    .stack      > DDR2
}
"""


@dataclasses.dataclass(frozen=True)
class Board:
    endian: str
    ibl: ibl_build.Build
    program: Path
    blob: Path


def blob(program: Path, out: Path) -> Path:
    """The program's loadable contents as one image from the base of DDR2, the form the IBL's
    binary-blob loader copies in byte for byte."""
    elf = parse_elf(str(program))
    assert elf.entry == DDR2_BASE, f"entry {elf.entry:#x} is not the blob's first byte"
    image = bytearray()
    for segment in elf.programs:
        if segment.kind != PT_LOAD or segment.file_size == 0:
            continue
        offset = segment.address - DDR2_BASE
        assert offset >= 0, f"segment at {segment.address:#x} is below DDR2"
        image.extend(bytes(max(0, offset + segment.file_size - len(image))))
        image[offset:offset + segment.file_size] = elf.data[
            segment.offset:segment.offset + segment.file_size]
    out.write_bytes(bytes(image))
    return out


@pytest.fixture(scope="module", params=[("le", "sum_loop"), ("be", "big_endian")],
                ids=["little-endian", "big-endian"])
def board(request: pytest.FixtureRequest, tmp_path_factory: pytest.TempPathFactory) -> Board:
    endian, name = request.param
    try:
        ibl = ibl_build.build(endian)
    except ibl_build.IblToolError as error:
        pytest.fail(str(error))
    work = tmp_path_factory.mktemp(f"ibl-{endian}")
    link = work / "ddr2.cmd"
    link.write_text(DDR2_LINK)
    base = next(item for item in load_programs() if item.name == name)
    program = build_target_program(
        dataclasses.replace(base, name=f"{name}_ddr2", link_cmd=str(link)), work)
    assert program is not None, f"could not build {name} for DDR2"
    return Board(endian, ibl, program, blob(program, work / f"c6472-{endian}.bin"))


def test_tis_ibl_boots_a_program_over_ethernet(
    candidate_binary: Path, board: Board, tmp_path: Path,
) -> None:
    state_path = tmp_path / "state.json"
    run = subprocess.run([
        sys.executable, str(ROOT / "tests/tools/ibl_peer.py"),
        "--emulator", str(candidate_binary), "--image", str(board.program),
        "--eeprom", str(board.ibl.eeprom), "--blob", str(board.blob),
        "--file-name", board.blob.name, "--max-cycles", "3000000000",
        "--state-out", str(state_path),
    ], cwd=ROOT, capture_output=True, text=True, timeout=1200)
    report = json.loads(run.stdout)
    assert run.returncode == 0, json.dumps(report, indent=2)
    assert report["steps"][:2] == [
        "bootp request from 02:00:00:00:64:72",
        f"bootp reply 192.168.1.3 file {board.blob.name}",
    ]
    assert f"tftp read {board.blob.name} octet" in report["steps"]
    state = json.loads(state_path.read_text())
    # The ROM loaded the first stage and nothing else: one parameter table, no failed block.
    assert state["i2c_boot"]["error"] is None
    assert state["i2c_boot"]["parameter_blocks"] == 1
    assert state["i2c_boot"]["checksum_failures"] == 0
    # exit_code is only A4 when the run ended; halted says the run ended because the program
    # reached its exit, rather than at the cycle limit with 55 happening to be in A4.
    assert state["halted"] is True
    assert state["exit_code"] == 55
    assert state["fault"] is None
