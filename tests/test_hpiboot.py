"""C6455 host boot: the positional ELF supplies symbols, never executable memory.

The host writes PT_LOAD code/data over --hpi-unix and releases the held CPU with DSPINT.
Unlike Ethernet boot there is no boot table or entry field: execution starts at L2 base.
"""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
import struct
import subprocess
import sys

import pytest

from common import ROOT
from target_programs import build_target_program, load_programs
from tools.hpi_host import L2_BASE, load_boot_image
from tools.emac_boot import BootError, parse_elf

pytestmark = [pytest.mark.slow, pytest.mark.host]

TOOL = ROOT / "tests/tools/hpi_host.py"
LINKER = ROOT / "tests/firmware/hpi_boot/link.cmd"
PROBE_ADDRESS = 0x0086_0001
PROBE = bytes.fromhex("123456789abcdef00123")


def append_probe(image: Path) -> None:
    """Add a final unaligned ten-byte load segment: three words, not a four-word DMA burst."""
    elf = parse_elf(str(image))
    data = bytearray(elf.data)
    endian, size, count = elf.endian, elf.table_entry_size, len(elf.programs)
    table = bytes(data[elf.table_offset : elf.table_offset + size * count])
    probe_offset = len(data)
    data.extend(PROBE)
    table_offset = len(data)
    data.extend(table)
    data.extend(struct.pack(
        endian + "IIIIIIII", 1, probe_offset, PROBE_ADDRESS, PROBE_ADDRESS,
        len(PROBE), len(PROBE), 6, 1,
    ) + bytes(size - 32))
    struct.pack_into(endian + "I", data, 28, table_offset)
    struct.pack_into(endian + "H", data, 44, count + 1)
    image.write_bytes(data)


@pytest.fixture(scope="module", params=["sum_loop", "big_endian"])
def image(request: pytest.FixtureRequest, tmp_path_factory: pytest.TempPathFactory) -> Path:
    program = next((item for item in load_programs() if item.name == request.param), None)
    if program is None:
        pytest.skip(f"no {request.param} target program")
    workdir = tmp_path_factory.mktemp(f"hpiboot-{request.param}")
    built = build_target_program(replace(program, link_cmd=str(LINKER)), workdir)
    assert built is not None, f"could not build the {request.param} target program"
    append_probe(built)
    return built


def hpiboot(candidate: Path, image: Path, workdir: Path, release: bool = True) -> tuple[dict, dict]:
    state = workdir / "state.json"
    result = subprocess.run(
        [
            sys.executable, str(TOOL), "--emulator", str(candidate), "--image", str(image),
            "--boot", "--report", "--hpi-poll", "256", "--max-cycles", "100000000",
            "--state-out", str(state), "--timeout", "120",
            *([] if release else ["--no-dspint"]),
        ],
        cwd=ROOT, capture_output=True, text=True, timeout=180,
    )
    assert result.returncode == 0, f"host failed: {result.stderr}{result.stdout}"
    assert state.is_file(), f"no machine state: {result.stderr}{result.stdout}"
    return json.loads(result.stdout), json.loads(state.read_text())


def test_c6455_boots_code_and_data_from_external_hpi_host(
    candidate_binary: Path, image: Path, tmp_path: Path,
) -> None:
    report, state = hpiboot(candidate_binary, image, tmp_path)
    loaded, order = load_boot_image(str(image))
    assert loaded.entry == L2_BASE
    assert loaded.segments[-1].address == PROBE_ADDRESS
    assert loaded.segments[-1].data == PROBE
    assert report["boot"] == {
        "entry": L2_BASE,
        "bytes": sum(len(segment.data) for segment in loaded.segments),
        "segments": len(loaded.segments),
        "verified": True,
        "released": True,
        "byteorder": order,
    }
    assert state["device"] == "c6455"
    assert state["hpi_boot"]["entry"] == L2_BASE
    assert state["exit_code"] == 55, "the transferred sum_loop computes 1 + ... + 10"
    assert state["fault"] is None
    assert len(state["cores"]) == 1
    assert state["cores"][0]["held_in_reset"] is False
    assert state["cores"][0]["cycles"] > 0


def test_c6455_hpi_load_without_dspint_executes_nothing(
    candidate_binary: Path, image: Path, tmp_path: Path,
) -> None:
    report, state = hpiboot(candidate_binary, image, tmp_path, release=False)
    assert report["boot"]["verified"] is True
    assert report["boot"]["released"] is False
    assert state["hpi_boot"]["entry"] is None
    assert state["fault"] is None
    assert state["cycles"] == 0
    assert state["cores"][0]["held_in_reset"] is True
    assert state["cores"][0]["cycles"] == 0


def test_host_rejects_an_elf_entry_that_hpi_boot_cannot_reach(image: Path, tmp_path: Path) -> None:
    elf = parse_elf(str(image))
    data = bytearray(elf.data)
    struct.pack_into(elf.endian + "I", data, 24, L2_BASE + 32)
    moved = tmp_path / "wrong-entry.out"
    moved.write_bytes(data)
    with pytest.raises(BootError, match="HPI boot requires ELF entry"):
        load_boot_image(str(moved))
