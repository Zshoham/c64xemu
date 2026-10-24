"""A C6472 executes a program that reached it only through its master I2C boot."""
from __future__ import annotations

import dataclasses
import json
from pathlib import Path
import subprocess
import sys

import pytest

from common import ROOT
from target_programs import build_target_program, load_programs
from tools import i2c_eeprom, ibl_tools
from tools.emac_boot import parse_elf

pytestmark = [pytest.mark.slow]


@pytest.fixture(scope="module", params=["sum_loop", "big_endian"])
def image(request: pytest.FixtureRequest, tmp_path_factory: pytest.TempPathFactory) -> Path:
    program = next(item for item in load_programs() if item.name == request.param)
    built = build_target_program(program, tmp_path_factory.mktemp(f"i2c-{request.param}"))
    assert built is not None, f"could not build {request.param}"
    return built


# BOOTMODE 2 reads the EEPROM at 0x50 and 3 the one at 0x51; CFG[3:0] selects the parameter
# block and CFG[4] PLL1's multiplier (SPRUEC6G Table 22).
@pytest.mark.parametrize("mode,slave,index,pll_x19", [(2, 0x50, 0, False), (3, 0x51, 5, True)])
def test_a_c6472_boots_a_program_from_its_eeprom(
    candidate_binary: Path, image: Path, tmp_path: Path,
    mode: int, slave: int, index: int, pll_x19: bool,
) -> None:
    eeprom = tmp_path / "eeprom.bin"
    built = subprocess.run([
        sys.executable, str(ROOT / "tests/tools/i2c_eeprom.py"), "--image", str(image),
        "--out", str(eeprom), "--index", str(index), "--slave", hex(slave), "--json",
    ], cwd=ROOT, capture_output=True, text=True, timeout=60)
    assert built.returncode == 0, built.stderr
    report = json.loads(built.stdout)
    state_path = tmp_path / "state.json"
    command = [
        str(candidate_binary), "run", str(image), "--device", "c6472", "--boot-mode", str(mode),
        "--i2c-eeprom", str(eeprom), "--boot-param-index", str(index),
        "--max-cycles", "50000000", "--state-out", str(state_path),
    ]
    if pll_x19:
        command.append("--i2c-pll-x19")
    run = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=300)
    assert state_path.is_file(), run.stderr + run.stdout
    state = json.loads(state_path.read_text())
    boot = state["i2c_boot"]
    assert boot["error"] is None
    assert boot["parameter_blocks"] == 1
    assert boot["checksum_failures"] == 0
    assert boot["table_bytes"] == report["table_bytes"]
    assert boot["entry"] == report["entry"]
    assert "utopia_boot" not in state
    assert state["exit_code"] == 55
    assert state["fault"] is None
    assert state["cores"][0]["held_in_reset"] is False
    assert all(core["held_in_reset"] and core["cycles"] == 0 for core in state["cores"][1:])


# The EEPROMs TI's own tools make: the IBL's hex6x/bconvert64x/b2i2c/romparse chain
# (tests/tools/ibl_tools.py), the same index, EEPROM address and EEPROM type (options bits 4:2,
# tiboot_c6472.h) as above.
TI_EEPROMS = [(2, 0x50, 0, 0), (3, 0x51, 5, 1)]


def ti_eeprom(image: Path, work: Path, slave: int, index: int, eeprom_type: int) -> bytes:
    elf = parse_elf(str(image))
    try:
        return ibl_tools.eeprom(
            image, elf.entry, elf.endian == ">",
            ibl_tools.Section(param_index=index, options=1 | (eeprom_type << 2),
                              dev_addr_ext=slave),
            work,
        )
    except ibl_tools.IblToolError as error:
        pytest.fail(str(error))


@pytest.mark.parametrize("mode,slave,index,eeprom_type", TI_EEPROMS)
def test_an_eeprom_made_by_tis_tools_boots(
    candidate_binary: Path, image: Path, tmp_path: Path,
    mode: int, slave: int, index: int, eeprom_type: int,
) -> None:
    eeprom = tmp_path / "ti.bin"
    eeprom.write_bytes(ti_eeprom(image, tmp_path, slave, index, eeprom_type))
    state_path = tmp_path / "state.json"
    run = subprocess.run([
        str(candidate_binary), "run", str(image), "--device", "c6472", "--boot-mode", str(mode),
        "--i2c-eeprom", str(eeprom), "--boot-param-index", str(index),
        "--max-cycles", "50000000", "--state-out", str(state_path),
    ], cwd=ROOT, capture_output=True, text=True, timeout=300)
    assert state_path.is_file(), run.stderr + run.stdout
    state = json.loads(state_path.read_text())
    assert state["i2c_boot"]["error"] is None
    assert state["i2c_boot"]["entry"] == parse_elf(str(image)).entry
    assert state["exit_code"] == 55
    assert state["fault"] is None


# The suite's own builder, given the same choices, writes TI's image byte for byte: romparse's
# 30-byte parameter tables with the check disabled, b2i2c's blocks, and table words most
# significant byte first in either byte order. The UTOPIA cases rest on this builder because
# romparse writes no Table 26 block.
@pytest.mark.parametrize("mode,slave,index,eeprom_type", TI_EEPROMS)
def test_the_eeprom_builder_writes_what_tis_tools_write(
    image: Path, tmp_path: Path, mode: int, slave: int, index: int, eeprom_type: int,
) -> None:
    table, _ = i2c_eeprom.boot_table(str(image))
    ours = i2c_eeprom.build(index=index, slave=slave, table=table, utopia=None, config=[],
                            eeprom_type=eeprom_type)
    assert ours == ti_eeprom(image, tmp_path, slave, index, eeprom_type)


L1D_PROGRAM = """
#pragma DATA_SECTION(marker, ".l1d")
volatile unsigned marker = 0x12345678;
int main(void) { return marker == 0x12345678 ? 55 : 1; }
"""


# An I2C-only boot leaves the 16 KB of L1D SRAM below the ROM's 16 KB cache to the image
# (SPRUEC6G Table 21): only UTOPIA boot fills it with its receive buffer. A segment there comes
# off the EEPROM like any other.
def test_an_i2c_boot_loads_a_segment_into_l1d_sram(
    candidate_binary: Path, tmp_path: Path,
) -> None:
    source = tmp_path / "main.c"
    source.write_text(L1D_PROGRAM)
    link = tmp_path / "link.cmd"
    link.write_text((ROOT / "tests/programs/common/link.cmd").read_text().replace(
        "MEMORY\n{\n", "MEMORY\n{\n    L1D : origin = 0x00F00000, length = 0x00004000\n",
    ).replace("SECTIONS\n{\n", "SECTIONS\n{\n    .l1d        > L1D\n"))
    sum_loop = next(item for item in load_programs() if item.name == "sum_loop")
    program = dataclasses.replace(
        sum_loop, name="l1d_marker", sources=(str(source),), link_cmd=str(link),
    )
    image = build_target_program(program, tmp_path)
    assert image is not None, "could not build the L1D program"
    eeprom = tmp_path / "eeprom.bin"
    built = subprocess.run([
        sys.executable, str(ROOT / "tests/tools/i2c_eeprom.py"), "--image", str(image),
        "--out", str(eeprom),
    ], cwd=ROOT, capture_output=True, text=True, timeout=60)
    assert built.returncode == 0, built.stderr
    state_path = tmp_path / "state.json"
    subprocess.run([
        str(candidate_binary), "run", str(image), "--device", "c6472", "--boot-mode", "2",
        "--i2c-eeprom", str(eeprom), "--max-cycles", "50000000",
        "--dump", "0x00F00000:4", "--state-out", str(state_path),
    ], cwd=ROOT, capture_output=True, text=True, timeout=300)
    state = json.loads(state_path.read_text())
    assert state["i2c_boot"]["error"] is None
    assert state["exit_code"] == 55
    assert state["fault"] is None


@pytest.mark.parametrize("image", ["sum_loop"], indirect=True)
@pytest.mark.parametrize("arguments,message", [
    (["--boot-mode", "2"], "give it the EEPROM with --i2c-eeprom"),
    (["--boot-mode", "0", "--i2c-eeprom", "IMAGE"], "--boot-mode 2 (EEPROM at 0x50) or 3"),
    (["--boot-mode", "2", "--i2c-eeprom", "IMAGE", "--boot-param-index", "16"],
     "beyond the 16 parameter blocks"),
])
def test_the_i2c_boot_options_follow_the_c6472_pins(
    candidate_binary: Path, image: Path, arguments: list[str], message: str,
) -> None:
    arguments = [str(image) if argument == "IMAGE" else argument for argument in arguments]
    run = subprocess.run([
        str(candidate_binary), "run", str(image), "--device", "c6472", *arguments,
    ], cwd=ROOT, capture_output=True, text=True, timeout=60)
    assert run.returncode != 0
    assert message in run.stderr
