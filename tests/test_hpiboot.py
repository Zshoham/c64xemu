"""Host boot through the HPI: the positional ELF supplies symbols, never executable memory.

On the C6455 the host writes PT_LOAD code/data over --hpi-unix and releases the held CPU with
DSPINT; there is no boot table or entry field, so execution starts at L2 base. On the C6472 all
six cores are held, DSPINT releases none, and the host writes each chosen core's image at its
global alias, DSP_BOOT_ADDRn (entry >> 10) and the BCn bits of BOOT_COMPLETE_STAT.

Every boot runs twice: once with the host a live process on --hpi-unix, and once with the same
host written down by `hpi_host.py --emit-replay` and replayed with --hpi-replay.
"""

from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
import struct
import subprocess
import sys

import pytest

from common import ROOT, RTS_INCLUDE_DIR
from target_programs import Program, build_target_program, load_programs
from tools.hpi_host import L2_BASE, emulator_failure, load_boot_image, parse_dumps, place_segments
from tools.emac_boot import BootError, Image, Segment, parse_elf

pytestmark = [pytest.mark.slow, pytest.mark.host]

TOOL = ROOT / "tests/tools/hpi_host.py"
LINKER = ROOT / "tests/firmware/hpi_boot/link.cmd"
C6472_FIRMWARE = ROOT / "tests/firmware/hpi_boot"
PROBE_ADDRESS = 0x0086_0001
PROBE = bytes.fromhex("123456789abcdef00123")
CORES = 6
RESULTS = 0x0020_0000
# The firmware's per-core result: 1 + 2 + ... + (10 + core), computed on that core.
EXPECTED = [sum(range(1, 11 + core)) for core in range(CORES)]
LOCAL_L2_END = 0x0089_8000


def append_probe(image: Path, address: int = PROBE_ADDRESS, payload: bytes = PROBE) -> None:
    """Add a final load segment, by default an unaligned ten-byte one: three words, not a
    four-word DMA burst."""
    elf = parse_elf(str(image))
    data = bytearray(elf.data)
    endian, size, count = elf.endian, elf.table_entry_size, len(elf.programs)
    table = bytes(data[elf.table_offset : elf.table_offset + size * count])
    probe_offset = len(data)
    data.extend(payload)
    table_offset = len(data)
    data.extend(table)
    data.extend(struct.pack(
        endian + "IIIIIIII", 1, probe_offset, address, address,
        len(payload), len(payload), 6, 1,
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


@pytest.fixture(params=["socket", "file"])
def transport(request: pytest.FixtureRequest) -> str:
    """How the host reaches the emulator: a live process, or a replayed file."""
    return request.param


def emulator_options(extra: tuple[str, ...]) -> tuple[list[str], list[str]]:
    """Split the host tool's options into the host's own and the emulator's `--device` and
    `--dump`, which a replayed boot passes to the emulator itself."""
    host, emulator = [], []
    arguments = iter(extra)
    for argument in arguments:
        if argument in ("--device", "--dump"):
            value = next(arguments)
            emulator += [argument, value]
            if argument == "--device":
                host += [argument, value]
        elif argument == "--cores":
            host += [argument, next(arguments)]
        else:
            host.append(argument)
    return host, emulator


def emit_replay(image: Path, workdir: Path, host: list[str]) -> tuple[Path, dict]:
    """The boot written as an --hpi-replay file, and the boot report the emitter gives."""
    replay = workdir / "boot.csv"
    result = subprocess.run(
        [sys.executable, str(TOOL), "--image", str(image), "--boot", "--report",
         "--emit-replay", str(replay), *host],
        cwd=ROOT, capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, f"emitter failed: {result.stderr}{result.stdout}"
    return replay, json.loads(result.stdout)


def replay_boot(
    candidate: Path, image: Path, replay: Path, state: Path, emulator: list[str],
) -> subprocess.CompletedProcess:
    return subprocess.run(
        [str(candidate), "run", str(image), "--boot-mode", "1", "--hpi-replay", str(replay),
         "--max-cycles", "100000000", "--state-out", str(state), *emulator],
        cwd=ROOT, capture_output=True, text=True, timeout=180,
    )


def hpiboot(
    candidate: Path, image: Path, workdir: Path, release: bool = True, extra: tuple[str, ...] = (),
    transport: str = "socket",
) -> tuple[dict, dict]:
    state = workdir / "state.json"
    options = (*([] if release else ["--no-dspint"]), *extra)
    if transport == "file":
        host, emulator = emulator_options(options)
        replay, report = emit_replay(image, workdir, host)
        result = replay_boot(candidate, image, replay, state, emulator)
        # The emulator's exit status is the application's exit code (sum_loop exits 55), or 1
        # for a held boot that ran out of cycles: the state decides, as it does for the socket.
        failure = emulator_failure(str(state), result.returncode, require_exit=report["boot"]["released"])
        assert failure is None, f"{failure}: {result.stderr[-2000:]}"
        report["dumps"] = parse_dumps(result.stderr)
        state_json = json.loads(state.read_text())
        assert state_json["hpi_replay"]["complete"] is True
        assert state_json["hpi_replay"]["failure"] is None
        return report, state_json
    result = subprocess.run(
        [
            sys.executable, str(TOOL), "--emulator", str(candidate), "--image", str(image),
            "--boot", "--report", "--hpi-poll", "256", "--max-cycles", "100000000",
            "--state-out", str(state), "--timeout", "120", *options,
        ],
        cwd=ROOT, capture_output=True, text=True, timeout=180,
    )
    assert result.returncode == 0, f"host failed: {result.stderr}{result.stdout}"
    assert state.is_file(), f"no machine state: {result.stderr}{result.stdout}"
    return json.loads(result.stdout), json.loads(state.read_text())


def test_c6455_boots_code_and_data_from_external_hpi_host(
    candidate_binary: Path, image: Path, tmp_path: Path, transport: str,
) -> None:
    report, state = hpiboot(candidate_binary, image, tmp_path, transport=transport)
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
    assert state["hpi_boot"]["entries"] == [L2_BASE]
    assert state["exit_code"] == 55, "the transferred sum_loop computes 1 + ... + 10"
    assert state["fault"] is None
    assert len(state["cores"]) == 1
    assert state["cores"][0]["held_in_reset"] is False
    assert state["cores"][0]["cycles"] > 0


def test_c6455_hpi_load_without_dspint_executes_nothing(
    candidate_binary: Path, image: Path, tmp_path: Path, transport: str,
) -> None:
    report, state = hpiboot(candidate_binary, image, tmp_path, release=False, transport=transport)
    assert report["boot"]["verified"] is True
    assert report["boot"]["released"] is False
    assert state["hpi_boot"]["entry"] is None
    assert state["hpi_boot"]["entries"] == [None]
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


# ---- C6472: six cores held, released by BOOT_COMPLETE_STAT ------------------------------------


@pytest.fixture(
    scope="module",
    params=[(0x0080_0000, "little"), (0x0081_0400, "little"), (0x0081_0400, "big")],
    ids=["entry-l2-base", "entry-1kb", "entry-1kb-big-endian"],
)
def c6472_image(request: pytest.FixtureRequest, tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, int]:
    """The result probe linked so its startup, and so the ELF entry, is at the requested address.
    Away from the base of L2, the probe's `guard` goes there, so a core started at the default
    boot address rather than its programmed one publishes the guard's value instead of a sum."""
    entry, order = request.param
    workdir = tmp_path_factory.mktemp(f"hpiboot-c6472-{entry:08x}-{order}")
    linker = (C6472_FIRMWARE / "c6472.cmd").read_text()
    text, guard = ".text 0x00800000", ".guard      > L2"
    assert text in linker and guard in linker
    linker = linker.replace(text, f".text 0x{entry:08x}")
    if entry != L2_BASE:
        linker = linker.replace(guard, f".guard 0x{L2_BASE:08x}")
    link_cmd = workdir / "c6472.cmd"
    link_cmd.write_text(linker)
    endian = ("--big_endian",) if order == "big" else ()
    program = Program(
        name="hpiboot_c6472", weight=0, sources=(str(C6472_FIRMWARE / "c6472.c"),),
        includes=(str(RTS_INCLUDE_DIR),),
        cflags=("-O1", *endian), link_flags=endian, entry="_start", max_cycles=0,
        opt_levels=("-O1",), rts=False, expect={}, cio_output=None, rx_frames=(), tx_frames=None,
        boot_table=None, boot_config=None, link_cmd=str(link_cmd),
    )
    built = build_target_program(program, workdir)
    assert built is not None, "could not build the C6472 host-boot probe"
    if entry != L2_BASE:
        segments = parse_elf(str(built)).image(0).segments
        assert any(segment.address == L2_BASE for segment in segments), "no guard at L2 base"
    return built, entry


def c6472_boot(
    candidate: Path, image: Path, workdir: Path, cores: tuple[int, ...] | None = None,
    extra: tuple[str, ...] = (), transport: str = "socket",
) -> tuple[dict, dict, list[int]]:
    """Boot over the HPI and return the report, the state and the shared mailbox at exit. The
    dump reads memory as little-endian words; the mailbox holds words in the image's order."""
    report, state = hpiboot(candidate, image, workdir, transport=transport, extra=(
        "--device", "c6472", "--dump", f"{RESULTS:#010x}:{4 * CORES}",
        *(["--cores", ",".join(map(str, cores))] if cores else []), *extra,
    ))
    order = report["boot"]["byteorder"]
    mailbox = [
        int.from_bytes(word.to_bytes(4, "little"), order)
        for word in report["dumps"][f"{RESULTS:#010x}"]
    ]
    return report, state, mailbox


def assert_only_these_cores_ran(state: dict, cores: tuple[int, ...], entry: int) -> None:
    assert state["device"] == "c6472"
    assert state["fault"] is None
    assert len(state["cores"]) == CORES
    assert state["hpi_boot"]["entry"] == state["hpi_boot"]["entries"][0]
    assert state["hpi_boot"]["entries"] == [entry if core in cores else None for core in range(CORES)]
    for core in range(CORES):
        snapshot = state["cores"][core]
        assert snapshot["fault"] is None
        assert snapshot["held_in_reset"] is (core not in cores), f"core {core}"
        assert (snapshot["cycles"] > 0) is (core in cores), f"core {core}"


def test_c6472_boots_all_six_cores_independently(
    candidate_binary: Path, c6472_image: tuple[Path, int], tmp_path: Path, transport: str,
) -> None:
    image, entry = c6472_image
    report, state, mailbox = c6472_boot(candidate_binary, image, tmp_path, transport=transport)
    loaded, order = load_boot_image(str(image), "c6472")
    assert loaded.entry == entry
    assert report["boot"]["cores"] == list(range(CORES))
    assert report["boot"]["released"] is True
    assert report["boot"]["verified"] is True
    assert report["boot"]["byteorder"] == order
    assert state["exit_code"] == 0b111111, "the leader saw every core's result before exiting"
    assert mailbox == EXPECTED
    assert_only_these_cores_ran(state, tuple(range(CORES)), entry)


@pytest.mark.parametrize("cores", [(0, 3), (2, 4)])
def test_c6472_releases_only_the_selected_cores(
    candidate_binary: Path, c6472_image: tuple[Path, int], tmp_path: Path, cores: tuple[int, ...],
    transport: str,
) -> None:
    image, entry = c6472_image
    report, state, mailbox = c6472_boot(candidate_binary, image, tmp_path, cores, transport=transport)
    assert report["boot"]["cores"] == list(cores)
    assert state["cores"][min(cores)]["halted"], "the selected leader reached the exit"
    assert mailbox == [EXPECTED[core] if core in cores else 0 for core in range(CORES)]
    assert_only_these_cores_ran(state, cores, entry)


@pytest.mark.parametrize("extra", [(), ("--dspint",)], ids=["loaded", "dspint-only"])
def test_c6472_hpi_load_and_dspint_do_not_release_a_core(
    candidate_binary: Path, c6472_image: tuple[Path, int], tmp_path: Path, extra: tuple[str, ...],
    transport: str,
) -> None:
    image, entry = c6472_image
    report, state, mailbox = c6472_boot(
        candidate_binary, image, tmp_path, extra=("--no-release", *extra), transport=transport,
    )
    assert report["boot"]["verified"] is True
    assert report["boot"]["released"] is False
    assert mailbox == [0] * CORES
    assert state["cycles"] == 0
    assert_only_these_cores_ran(state, (), entry)


def test_c6472_host_rejects_entries_and_segments_that_cannot_boot(
    c6472_image: tuple[Path, int], tmp_path: Path,
) -> None:
    image, entry = c6472_image
    elf = parse_elf(str(image))
    for name, address, message in (
        ("misaligned", entry + 4, "1 KB-aligned"),
        ("outside", LOCAL_L2_END, "1 KB-aligned"),
        ("below", 0x0020_0000, "1 KB-aligned"),
    ):
        data = bytearray(elf.data)
        struct.pack_into(elf.endian + "I", data, 24, address)
        bad = tmp_path / f"{name}.out"
        bad.write_bytes(data)
        with pytest.raises(BootError, match=message):
            load_boot_image(str(bad), "c6472")

    # A segment that starts in L2 and runs past its end would land in another core's memory
    # through the alias arithmetic: its tail must be refused, not wrapped.
    overflowing = tmp_path / "overflow.out"
    overflowing.write_bytes(elf.data)
    append_probe(overflowing, LOCAL_L2_END - 4, bytes(8))
    with pytest.raises(BootError, match="not wholly inside one core-local window"):
        load_boot_image(str(overflowing), "c6472")


def test_c6472_host_replicates_local_segments_and_writes_shared_ones_once() -> None:
    code = Segment(0x0080_0400, b"code")
    shared = Segment(0x0020_0000, b"data")
    placements = place_segments(Image(0, code.address, (code, shared)), (1, 4))
    # Shared L2's 0020 0000h is local-only (SPRS612G Table 2-2); the HPI names it by its alias.
    assert placements == [(0x1180_0400, b"code"), (0x1480_0400, b"code"), (0x1020_0000, b"data")]



# ---- replay files ------------------------------------------------------------------------------


def test_a_replayed_boot_places_the_unaligned_probe_exactly(
    candidate_binary: Path, image: Path, tmp_path: Path,
) -> None:
    """The ten-byte probe at 0x00860001 covers two words partly: the emitter checks each against
    what it knows is there before writing over it, and the bytes around the probe survive."""
    base = PROBE_ADDRESS & ~3
    report, state = hpiboot(
        candidate_binary, image, tmp_path, transport="file", extra=("--dump", f"{base:#010x}:16"),
    )
    memory = b"".join(word.to_bytes(4, "little") for word in report["dumps"][f"{base:#010x}"])
    offset = PROBE_ADDRESS - base
    assert memory[offset : offset + len(PROBE)] == PROBE
    assert memory[:offset] == bytes(offset)
    assert memory[offset + len(PROBE) :] == bytes(len(memory) - offset - len(PROBE))
    rows = (tmp_path / "boot.csv").read_text().splitlines()
    assert f"0,address,write,0x{base >> 2:08x}" in rows
    assert "0,data,read,0x00000000" in rows, "a guard on a partly covered word"


def test_a_corrupted_replay_word_fails_the_boot_at_its_line(
    candidate_binary: Path, image: Path, tmp_path: Path,
) -> None:
    replay, report = emit_replay(image, tmp_path, [])
    rows = replay.read_text().splitlines()
    # The first checked read of transferred data: expect a word the host did not write.
    number = next(index for index, row in enumerate(rows) if ",data-auto,read,0x" in row)
    tick, port, direction, value = rows[number].split(",")
    rows[number] = f"{tick},{port},{direction},0x{int(value, 16) ^ 1:08x}"
    replay.write_text("\n".join(rows) + "\n")
    state = tmp_path / "state.json"
    result = replay_boot(candidate_binary, image, replay, state, [])
    assert result.returncode == 1, result.stderr
    assert state.is_file(), "the state is written on a failure too"
    report = json.loads(state.read_text())
    line = number + 1
    assert f"boot.csv line {line} tick 0: data-auto read {value}, expected" in report["fault"]
    assert report["hpi_replay"]["failure"]["line"] == line
    assert report["hpi_replay"]["complete"] is False
    assert report["cores"][0]["held_in_reset"] is True, "nothing was released"
    assert report["cycles"] == 0
