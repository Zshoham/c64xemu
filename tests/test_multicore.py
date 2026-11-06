"""The six-core device, driven the way a user drives it.

The C6472 is six C64x+ megamodules around one set of peripherals (SPRS612G section 2.3), so the
claims worth making end to end are the ones a single-core device cannot make: that every core out
of reset executes, that the 768 KB shared L2 at 0020 0000h is one memory every core sees at that
address, that each core's own L2 at 0080 0000h is six separate memories, and that a core's memory
also carries the global address (1n80 0000h) another core or an Ethernet MAC has to use. The unit
tests in `src/device.rs` and `src/system.rs` pin the register-level behaviour - boot release, the
inter-processor interrupt, the DMA snoop against a peer's L1D, the per-core Ethernet interrupt;
this file pins what the command line does with it.
"""

from __future__ import annotations

import json
import re
import select
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

from common import ROOT, symbol_address
from target_programs import Program, build_target_program, load_programs

FIRMWARE = ROOT / "tests/firmware/c6472_multicore"
NETECHO_FIRMWARE = ROOT / "tests/firmware/c6472_netecho"
NETECHO_PEER = ROOT / "tests/tools/multicore_echo.py"

# Payload rate per core for the paced echo case, in kilobits a second of wall clock: the same
# powers of two the single-core paced cases use, so the top rate is 4096 kbps on each of six
# cores, 24.6 Mbps through one MAC. The exchange stops completing between 6144 and 7168 kbps a
# core, and at the top rate here the emulator is the thing that is busy - 89% of the run's wall
# clock against the host peer's 34% - so what the case measures is the device model and not the
# harness.
NETECHO_RATES = (16, 32, 64, 128, 256, 512, 1024, 2048, 4096)
# Rates at or above this per core carry the `high_bandwidth` mark, which CI deselects: this is
# where the measured headroom to the boundary above drops below four times, and a shared runner
# cannot be relied on to sustain a pace that close. Everything below it passes with three
# competing CPU hogs on this machine, and so does the top rate - but not with four times less
# host to run on.
HIGH_BANDWIDTH_PER_CORE = 2048
# Half a second of sending per rate. The rate is the claim; the duration only decides how much
# of it is sent, and six cores of emulation is expensive enough that the suite should not spend
# a second on each of nine rates.
NETECHO_DURATION = "0.5"


# A small deterministic program with no peripheral dependencies: it ends at C$$EXIT with a known
# value, which is all these cases need from the code the cores run.
PROGRAM = "sum_loop"
EXPECTED_EXIT_CODE = 55
CORES = 6


@pytest.fixture(scope="module")
def image(tmp_path_factory: pytest.TempPathFactory) -> Path:
    workdir = tmp_path_factory.mktemp("multicore")
    program = next(
        (program for program in load_programs() if program.name == PROGRAM), None
    )
    if program is None:
        pytest.skip(f"no {PROGRAM} target program")
    built = build_target_program(program, workdir)
    if built is None:
        pytest.skip(f"could not build the {PROGRAM} target program")
    return built


def run_device(
    candidate: Path, image: Path, device: str, *extra: str, tag: str = ""
) -> dict:
    state = image.parent / f"state-{device}{tag}.json"
    completed = subprocess.run(
        [
            str(candidate),
            "run",
            str(image),
            "--device",
            device,
            "--max-cycles",
            "2000000",
            "--state-out",
            str(state),
            *extra,
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert state.is_file(), f"no state dump: {completed.stderr}"
    return json.loads(state.read_text())


def debug_session(candidate: Path, image: Path, script: str) -> str:
    completed = subprocess.run(
        [str(candidate), "debug", str(image), "--device", "c6472"],
        cwd=ROOT,
        input=script,
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert completed.returncode == 0, completed.stderr
    return completed.stdout


def dumped_words(output: str, address: int) -> list[str]:
    """The bytes a `mem` command printed for `address`, as lower-case hex pairs."""
    pattern = re.compile(rf"{address:08x} ((?:[0-9a-f]{{2}} )+)")
    match = pattern.search(output)
    assert match is not None, f"no dump of {address:#010x} in:\n{output}"
    return match.group(1).split()


def test_c6472_runs_every_core(candidate_binary: Path, image: Path) -> None:
    """Immediate boot releases all six cores, and each runs the image on its own.

    SPRS612G section 2.4.1: in boot mode 0 every megamodule starts at its DSP_BOOT_ADDR without
    waiting to be released. One image, six cores, six independent cycle counts.
    """
    state = run_device(candidate_binary, image, "c6472")
    assert state["device"] == "c6472"
    cores = state["cores"]
    assert len(cores) == CORES
    for core in cores:
        assert core["held_in_reset"] is False
        assert core["fault"] is None
        assert core["cycles"] > 0, f"core {core['core']} never ran"
    # Every core ran the same program to the same place, so they agree on where they stopped.
    assert len({core["pc"] for core in cores}) == 1
    assert state["exit_code"] == EXPECTED_EXIT_CODE

    # The single-core device reports exactly one core and the same result.
    single = run_device(candidate_binary, image, "c6455")
    assert single["device"] == "c6455"
    assert len(single["cores"]) == 1
    assert single["exit_code"] == EXPECTED_EXIT_CODE


def test_stack_windows_report_per_core_and_change_nothing_else(
    candidate_binary: Path, image: Path
) -> None:
    """`--stack-window-all` adds one entry per core in core order, `--stack-window` one for the
    focused core, and the rest of the report is exactly a plain run's.

    The program's crt0 builds B15 with MVKL then MVKH, so for one cycle B15 holds the stack top's
    low half, far outside the stack: the observer must arm on the completed value without
    calling that an excursion. The stack is the 2 KB `stack_base` reserves, and B15 starts eight
    bytes below its top.
    """
    base = symbol_address(image, ("stack_base",), kinds=(0, 1))
    assert base is not None, "crt.asm's stack_base is not in the symbol table"
    window = f"{base:#x}:{base + 2048:#x}"
    for device, cores in (("c6455", 1), ("c6472", CORES)):
        plain = run_device(candidate_binary, image, device)
        every = run_device(
            candidate_binary, image, device, "--stack-window-all", window, tag="-all"
        )
        one = run_device(
            candidate_binary, image, device, "--stack-window", window, tag="-one"
        )
        observations = every.pop("stack_observations")
        assert "stack_observation" not in every
        assert [entry["core"] for entry in observations] == list(range(cores))
        single = one.pop("stack_observation")
        assert "stack_observations" not in one
        assert single == observations[0]
        assert every == plain and one == plain, device
        for entry in observations:
            assert entry["start"] == base and entry["end"] == base + 2048
            assert entry["sp_written"] and entry["armed"], entry
            assert entry["min_sp"] == base + 2040, entry
            assert entry["min_sp_cycle"] == 1, entry  # the MVKH, one cycle after the MVKL
            assert not entry["out_of_bounds_sp"] and not entry["out_of_bounds_write"], entry

    # The two modes exclude each other, and a window must be non-empty and word aligned.
    for arguments in (
        ["--stack-window", window, "--stack-window-all", window],
        ["--stack-window", f"{base + 2048:#x}:{base:#x}"],
        ["--stack-window-all", f"{base + 2:#x}:{base + 2048:#x}"],
    ):
        completed = subprocess.run(
            [str(candidate_binary), "run", str(image), *arguments],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=60,
        )
        assert completed.returncode == 2, (arguments, completed.stderr)


def test_c6472_shared_memory_is_shared_and_local_memory_is_not(
    candidate_binary: Path, image: Path
) -> None:
    """One shared L2, six local L2s, and a global alias that reaches another core's.

    Table 2-2: shared RAM at 0020 0000h in every core's local map, local L2 at 0080 0000h per
    megamodule, and core *n*'s L2 also at 1n80 0000h. The debugger's memory commands act through
    the focused core, which is what makes the difference observable from outside.
    """
    output = debug_session(
        candidate_binary,
        image,
        "\n".join(
            [
                # Core 0 publishes a word in the shared RAM.
                "write 0x00200040 deadbeef",
                "core 3",
                # Core 3 sees it at the same local address, and writes its own L2.
                "mem 0x00200040 4",
                "write 0x00800080 11223344",
                "core 0",
                # Core 0's own L2 at that address is a different memory ...
                "mem 0x00800080 4",
                # ... but core 3's L2 is reachable at its global alias.
                "mem 0x13800080 4",
                "quit",
            ]
        )
        + "\n",
    )
    assert dumped_words(output, 0x0020_0040)[:4] == ["de", "ad", "be", "ef"]
    assert dumped_words(output, 0x1380_0080)[:4] == ["11", "22", "33", "44"]
    assert dumped_words(output, 0x0080_0080)[:4] != ["11", "22", "33", "44"]


def test_c6472_debugger_lists_and_switches_cores(
    candidate_binary: Path, image: Path
) -> None:
    """`core` names every core of the device, and `core N` moves what the session drives."""
    output = debug_session(candidate_binary, image, "core\nstepc 8\ncore 4\ncore\nquit\n")
    listed = [
        line.strip()
        for line in output.splitlines()
        if re.search(r"(\*|\s)\s*core \d ", line)
    ]
    assert len(listed) >= 2 * CORES, output
    first, last = listed[:CORES], listed[-CORES:]
    assert first[0].lstrip("(c64xemu) ").startswith("* core 0"), first
    assert last[4].startswith("* core 4"), last
    # Stepping advanced every core, not just the focused one, and they stay in lockstep: the
    # cores run one cycle each per device cycle (SPRS612G section 2.3 - one clock, six cores).
    counts = set(re.findall(r"running, (\d+) cycles", output))
    assert counts == {"0", "8"}, output
    stepped = [line for line in listed if "running, 8 cycles" in line]
    assert len(stepped) == 2 * CORES, output
    addresses = {re.search(r"pc (0x[0-9a-f]+)", line).group(1) for line in stepped}
    assert len(addresses) == 1, addresses


def gdb_packet(body: str) -> bytes:
    checksum = sum(body.encode()) & 0xFF
    return f"${body}#{checksum:02x}".encode()


def gdb_exchange(candidate: Path, image: Path, requests: list[str]) -> list[str]:
    """Send packets to the GDB stub and collect one reply each, acknowledgements aside."""
    process = subprocess.Popen(
        [str(candidate), "debug", "--gdb", "127.0.0.1:0", str(image), "--device", "c6472"],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        port = None
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and port is None:
            line = process.stdout.readline() if process.stdout else ""
            match = re.search(r"127\.0\.0\.1:(\d+)", line)
            if match:
                port = int(match.group(1))
        assert port is not None, "the stub never reported its port"
        replies: list[str] = []
        with socket.create_connection(("127.0.0.1", port), timeout=30) as link:
            for request in requests:
                link.sendall(gdb_packet(request))
                buffer = ""
                while "#" not in buffer:
                    ready, _, _ = select.select([link], [], [], 30)
                    assert ready, f"no reply to {request}"
                    chunk = link.recv(4096).decode()
                    assert chunk, f"stub closed while answering {request}"
                    buffer += chunk.replace("+", "", 1) if buffer == "" else chunk
                replies.append(buffer.split("$", 1)[-1].split("#", 1)[0])
                link.sendall(b"+")
        return replies
    finally:
        process.kill()
        process.wait(timeout=30)


def test_c6472_gdb_stub_reports_a_thread_per_core(
    candidate_binary: Path, image: Path
) -> None:
    """`info threads` in gdb lists the device's cores, and selecting one moves the session.

    The stub numbers threads from one and cores from zero, so thread 5 is core 4; a register
    read after `Hg5` is core 4's, which is what makes a six-core device debuggable at all.
    """
    replies = gdb_exchange(
        candidate_binary,
        image,
        ["qfThreadInfo", "qsThreadInfo", "qC", "Hg5", "qC", "qThreadExtraInfo,5", "T6", "Hg9"],
    )
    threads, more, first_focus, selected, second_focus, extra, alive, missing = replies
    assert threads == "m1,2,3,4,5,6", threads
    assert more == "l"
    assert first_focus == "QC1"
    assert selected == "OK"
    assert second_focus == "QC5", "the focused core followed Hg"
    assert bytes.fromhex(extra).decode() == "core 4"
    assert alive == "OK", "thread 6 is core 5, the last one"
    assert missing == "E01", "there is no core 8"


@pytest.fixture(scope="module")
def multicore_firmware(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """The C6472 rendezvous firmware, built with the checked-in TI toolchain.

    It needs its own linker command file: 608 KB of local L2 rather than the C6455's 2 MB, a
    1 KB-aligned window for the entry the released cores are given, and the shared L2 for the
    rendezvous buffer.
    """
    program = Program(
        name="c6472_multicore",
        weight=0,
        sources=(str(FIRMWARE / "main.c"),),
        includes=(),
        cflags=("-O1",),
        link_flags=(),
        entry="_c_int00",
        max_cycles=5_000_000,
        opt_levels=("-O1",),
        rts=True,
        expect={},
        cio_output=None,
        rx_frames=(),
        tx_frames=None,
        boot_table=None,
        boot_config=None,
        link_cmd=str(FIRMWARE / "link.cmd"),
    )
    workdir = tmp_path_factory.mktemp("c6472-firmware")
    image = build_target_program(program, workdir)
    if image is None:
        pytest.skip("could not build the C6472 multicore firmware")
    return image


def dumped(stderr: str, address: int) -> list[int]:
    """The words a `--dump` printed for one address."""
    words: list[int] = []
    collecting = False
    for line in stderr.splitlines():
        if line.startswith("dump "):
            collecting = line.startswith(f"dump {address:#010x}")
            continue
        if collecting and line.strip():
            try:
                words.extend(int(word, 16) for word in line.split()[1:])
            except ValueError:
                # The run's own closing line (a cycle limit, say) is not part of the dump.
                collecting = False
    return words


# STT_OBJECT and STT_COMMON: how TI's linker types an initialised and an uninitialised global.
VARIABLE_SYMBOLS = (1, 5)


def result_addresses(image: Path) -> tuple[int, int]:
    """Where the firmware left `answered` and `total`, which the compiler's layout decides."""
    answered = symbol_address(image, ("answered",), kinds=VARIABLE_SYMBOLS)
    total = symbol_address(image, ("total",), kinds=VARIABLE_SYMBOLS)
    assert answered is not None and total is not None, "result variables missing from the image"
    return answered, total


def test_c6472_firmware_releases_the_other_cores_and_meets_them(
    candidate_binary: Path, multicore_firmware: Path
) -> None:
    """The real multicore boot flow, run as firmware on the emulated device.

    In a ROM boot mode only core 0 leaves reset (SPRS612G section 2.4.1). It gives cores 1-5 a
    start address in DSP_BOOT_ADDRn, releases them through BOOT_COMPLETE_STAT and announces
    itself to each with an inter-processor interrupt; every released core writes its signature
    into the shared L2 and reads the source flag out of its own IPCGR. Core 0 waits for all six
    and tallies the result: 0+1+2+3+4+5 = 15, plus 0x100 for each of the five cores that saw
    the interrupt source, is 0x50F.
    """
    answered_at, total_at = result_addresses(multicore_firmware)
    completed = subprocess.run(
        [
            str(candidate_binary),
            "run",
            str(multicore_firmware),
            "--device",
            "c6472",
            # I2C slave boot has no modelled source: the ELF loads and cores 1-5 start held.
            "--boot-mode",
            "4",
            "--max-cycles",
            "5000000",
            "--dump",
            f"{answered_at:#010x}:4",
            "--dump",
            f"{total_at:#010x}:4",
            "--dump",
            "0x00200000:24",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=300,
    )
    # The firmware returns its tally from main, so the process exit status is the program's;
    # the values it left in memory are the result.
    (answered,) = dumped(completed.stderr, answered_at)
    (total,) = dumped(completed.stderr, total_at)
    assert answered == CORES, "every core reported in"
    assert total == 0x50F, f"tally {total:#x}"
    signatures = dumped(completed.stderr, 0x0020_0000)
    assert signatures == [0xC0DE0000 | core for core in range(CORES)]


def test_c6472_firmware_leaves_cores_held_without_a_release(
    candidate_binary: Path, multicore_firmware: Path
) -> None:
    """Nothing but BOOT_COMPLETE_STAT starts the other cores.

    Run the same firmware on a single-core C6455, where the boot controller does not exist and
    the writes go nowhere: only one core runs, so the rendezvous never completes and the tally
    counts one core. That is the control for the case above - the six-core result comes from
    the boot controller doing its job, not from the cores having been running all along.
    """
    answered_at, total_at = result_addresses(multicore_firmware)
    completed = subprocess.run(
        [
            str(candidate_binary),
            "run",
            str(multicore_firmware),
            "--device",
            "c6455",
            "--max-cycles",
            "20000000",
            "--dump",
            f"{answered_at:#010x}:4",
            "--dump",
            f"{total_at:#010x}:4",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=300,
    )
    (answered,) = dumped(completed.stderr, answered_at)
    (total,) = dumped(completed.stderr, total_at)
    assert answered == 1, "only core 0 ever reported"
    assert total == 0, "no other core's index was added"


@pytest.fixture(scope="module")
def netecho_firmware(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """The six-core Ethernet echo firmware, built with the checked-in TI toolchain.

    Its linker command file puts code, data and stack in each core's own 608 KB L2 and the
    descriptor rings and frame buffers in the shared L2 window, which is the only memory both
    every core and the MAC's DMA can reach.
    """
    program = Program(
        name="c6472_netecho",
        weight=0,
        sources=(str(NETECHO_FIRMWARE / "main.c"),),
        includes=(),
        cflags=("-O2",),
        link_flags=(),
        entry="_c_int00",
        max_cycles=5_000_000,
        opt_levels=("-O2",),
        rts=True,
        expect={},
        cio_output=None,
        rx_frames=(),
        tx_frames=None,
        boot_table=None,
        boot_config=None,
        link_cmd=str(NETECHO_FIRMWARE / "link.cmd"),
    )
    workdir = tmp_path_factory.mktemp("c6472-netecho")
    image = build_target_program(program, workdir)
    if image is None:
        pytest.skip("could not build the C6472 echo firmware")
    return image


@pytest.mark.parametrize(
    "kbps",
    [
        pytest.param(
            rate,
            marks=(pytest.mark.high_bandwidth,) if rate >= HIGH_BANDWIDTH_PER_CORE else (),
            id=f"{rate}k",
        )
        for rate in NETECHO_RATES
    ],
)
def test_c6472_echoes_paced_traffic_on_all_six_cores_at_once(
    candidate_binary: Path, netecho_firmware: Path, kbps: int
) -> None:
    """Six cores, one MAC, one paced stream per core, every byte back from the right core.

    The device gives all six megamodules one Ethernet MAC (SPRS612G Table 7-105), so the claim
    worth making is the one a single core cannot: that six cores each own a receive channel of
    the same MAC, each answer their own unicast address out of the shared address RAM, and each
    keep up with a rate paced against wall clock at the same time. The peer sends a deterministic
    stream to 02:00:00:64:72:0N for every N and requires it echoed by the core that owns that
    address, in order, with the whole exchange inside one and a half times the sending time -
    the same deadline the single-core bandwidth cases use.

    The ladder runs to 4096 kbps a core, which is 24.6 Mbps of payload through one MAC and
    within one and a half times the rate at which the exchange stops completing at all, so the
    top rates are a real measurement of how fast six cores can move frames rather than a
    formality.

    A core that never started, a core polling the wrong channel, an address slot bound to the
    wrong channel, or a descriptor given to the MAC at a local rather than a global address all
    show up the same way: that core's bytes never come back.
    """
    completed = subprocess.run(
        [
            sys.executable,
            str(NETECHO_PEER),
            "--emulator",
            str(candidate_binary),
            "--image",
            str(netecho_firmware),
            "--cores",
            str(CORES),
            "--bandwidth-kbps",
            str(kbps),
            "--duration",
            NETECHO_DURATION,
            "--report",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=300,
    )
    lines = [line for line in completed.stdout.splitlines() if line.startswith("{")]
    assert lines, f"the peer printed no report: {completed.stdout}{completed.stderr}"
    report = json.loads(lines[-1])
    expected_per_core = int(kbps * 125.0 * float(NETECHO_DURATION))
    assert report["state"] == "closed", f"{report}"
    assert len(report["cores"]) == CORES
    for core in report["cores"]:
        assert core["matched"], f"core {core['core']}: {core['mismatch']}"
        assert core["echoed"] == expected_per_core, f"core {core['core']} echoed {core['echoed']}"
    assert report["echoed"] == expected_per_core * CORES


def test_c6472_echo_needs_every_core_out_of_reset(
    candidate_binary: Path, netecho_firmware: Path
) -> None:
    """The control for the case above: in a ROM boot mode only core 0 runs.

    Nothing in this firmware writes BOOT_COMPLETE_STAT, so cores 1-5 stay held (SPRS612G
    section 2.4.1) and never arm a channel. Core 0 still brings the MAC up and announces
    itself, which is what makes the difference legible: the six-core result comes from six
    cores each answering their own address, not from one core answering all six.
    """
    completed = subprocess.run(
        [
            sys.executable,
            str(NETECHO_PEER),
            "--emulator",
            str(candidate_binary),
            "--image",
            str(netecho_firmware),
            "--cores",
            str(CORES),
            "--bandwidth-kbps",
            "64",
            "--duration",
            "0.2",
            # A boot mode with no modelled source: core 0 leaves reset and the rest wait for a
            # release that never comes.
            "--boot-mode",
            "4",
            # Core 0 announces itself in a few hundred thousand cycles; a wait an order of
            # magnitude longer than that is enough to call the other five held.
            "--ready-timeout",
            "2",
            "--report",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=300,
    )
    lines = [line for line in completed.stdout.splitlines() if line.startswith("{")]
    assert lines, f"the peer printed no report: {completed.stdout}{completed.stderr}"
    report = json.loads(lines[-1])
    assert report["state"] == "cores-never-ready", f"{report}"
    ready = [core["ready"] for core in report["cores"]]
    assert ready == [True] + [False] * (CORES - 1), f"{ready}"

TIMERS_FIRMWARE = ROOT / "tests/firmware/c6472_timers"


@pytest.fixture(scope="module")
def timers_firmware(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """The twelve-timer firmware, built with the checked-in TI toolchain.

    It needs the C6472's memory map: code and data in one megamodule's 608 KB local L2, and the
    counters it reports in the shared 768 KB window, at a fixed address so the dump does not
    depend on how the linker orders the sections.
    """
    program = Program(
        name="c6472_timers",
        weight=0,
        sources=(str(TIMERS_FIRMWARE / "main.c"),),
        includes=(),
        cflags=("-O1",),
        link_flags=(),
        entry="_c_int00",
        max_cycles=5_000_000,
        opt_levels=("-O1",),
        rts=True,
        expect={},
        cio_output=None,
        rx_frames=(),
        tx_frames=None,
        boot_table=None,
        boot_config=None,
        link_cmd=str(TIMERS_FIRMWARE / "link.cmd"),
    )
    workdir = tmp_path_factory.mktemp("c6472-timers")
    image = build_target_program(program, workdir)
    if image is None:
        pytest.skip("could not build the C6472 timer firmware")
    return image


def test_c6472_timers_all_count_alike(
    candidate_binary: Path, timers_firmware: Path
) -> None:
    """Twelve register blocks, one clock: every timer counts the internal CPU/6 clock, and which
    core programmed the block makes no difference.

    SPRS612G section 7.16: the device has "six localized timers (Timer 0 - Timer 5) and six
    shared timers (Timer 6 - Timer 11)" on 64 KB boundaries from 025E 0000h, and the localized
    ones are "clocked with an internal clock with a CPU/6 frequency". SPRU818B gives the 64-bit
    general-purpose mode no prescaler (section 3.9.2) and one register layout for the family
    (section 5), so instances brought up the same way must count at the same rate.

    The firmware brings all twelve up from core 0 - five of them belonging to megamodules whose
    cores are not running it, so those writes cross the switch fabric - spins 24 million CPU
    cycles on the time-stamp counter, and leaves each counter's delta plus its TGCR in the shared
    L2. A per-instance clock, a mis-addressed block, a dropped write, or a counter that stopped
    would all show here as a delta that does not match its neighbours'.
    """
    completed = subprocess.run(
        [
            str(candidate_binary),
            "run",
            str(timers_firmware),
            "--device",
            "c6472",
            "--boot-mode",
            "4",
            "--max-cycles",
            "120000000",
            "--dump",
            "0x00200000:104",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=300,
    )
    words = dumped(completed.stderr, 0x0020_0000)
    assert len(words) == 26, f"expected twelve deltas, twelve TGCRs, the spin and the verdict, got {words}"
    deltas, tgcr, spin, worst = words[:12], words[12:24], words[24], words[25]

    assert tgcr == [3] * 12, f"every timer in 64-bit general-purpose mode, got {tgcr}"
    assert spin > 20_000_000, "the spin was measured on the time-stamp counter"

    # The internal clock is CPU/6, so a million counts per six million CPU cycles: the timers
    # start a moment apart, which is why this is a tolerance and not an equality.
    expected = spin // 6
    for index, delta in enumerate(deltas):
        assert abs(delta - expected) <= expected // 100, (
            f"timer {index} counted {delta}, not about {expected} (CPU/6 of {spin} CPU cycles)"
        )

    spread = max(deltas) - min(deltas)
    assert spread <= expected // 100, (
        f"the twelve timers disagree by {spread} counts: {deltas}"
    )
    assert worst == 0, f"the firmware's own check found a deviation of {worst} thousandths"


SMC_FIRMWARE = ROOT / "tests/firmware/c6472_smc"


@pytest.fixture(scope="module")
def smc_firmware(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """The shared-memory controller firmware: C for the experiments, assembly for the three atomic
    instructions and the User-mode store, which cl6x has no way to say.

    It uses the C6472 rendezvous firmware's memory map - code and data in the local 608 KB L2, the
    results in the shared window at a fixed address - and leaves every cache off, as reset does.
    """
    program = Program(
        name="c6472_smc",
        weight=0,
        sources=(str(SMC_FIRMWARE / "main.c"), str(SMC_FIRMWARE / "atomics.asm")),
        includes=(),
        cflags=("-O1",),
        link_flags=(),
        entry="_c_int00",
        max_cycles=5_000_000,
        opt_levels=("-O1",),
        rts=True,
        expect={},
        cio_output=None,
        rx_frames=(),
        tx_frames=None,
        boot_table=None,
        boot_config=None,
        link_cmd=str(SMC_FIRMWARE / "link.cmd"),
    )
    workdir = tmp_path_factory.mktemp("c6472-smc")
    image = build_target_program(program, workdir)
    if image is None:
        pytest.skip("could not build the C6472 shared-memory controller firmware")
    return image


# `struct results` of tests/firmware/c6472_smc/main.c, in word order.
SMC_RESULT_FIELDS = (
    "word",
    "loaded",
    "contended_cmtl",
    "memory_after",
    "link_data",
    "link_status",
    "control_cmtl",
    "control_memory",
    "supervisor_flag",
    "prefetch_enable",
    "fault_status",
    "fault_address",
) + tuple(f"seen{core}" for core in range(CORES)) + tuple(
    f"done{core}" for core in range(CORES)
)


@pytest.fixture(scope="module")
def smc_results(candidate_binary: Path, smc_firmware: Path) -> dict[str, int]:
    """Run the firmware on the six-core device, cores 1-5 held until core 0 releases them."""
    completed = subprocess.run(
        [
            str(candidate_binary),
            "run",
            str(smc_firmware),
            "--device",
            "c6472",
            "--boot-mode",
            "4",
            "--max-cycles",
            "5000000",
            "--dump",
            f"0x00200000:{4 * len(SMC_RESULT_FIELDS)}",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=300,
    )
    words = dumped(completed.stderr, 0x0020_0000)
    assert len(words) == len(SMC_RESULT_FIELDS), completed.stderr
    return dict(zip(SMC_RESULT_FIELDS, words))


def test_c6472_a_second_store_link_breaks_the_link_and_the_commit_changes_nothing(
    smc_results: dict[str, int],
) -> None:
    """LL, SL(A), SL(B), CMTL on a non-prefetchable word of the shared L2, caches off.

    SPRUEG5D section 3.1.2 Table 3: a matching SL that meets LinkdtV already set must "Invalidate
    the Link (LinkV = 0)", and section 3.1.3 Table 4: a commit against LinkV = 0 must "Discard the
    request and return 0". The monitor keeps the first datum - SL2AMLDATA still reports A, and
    SL2AMLSTAT has LinkdtV set and LinkV clear (the register's CPUID field is core 0's) - and the
    word keeps what it held. A single SL is the control: it commits, returns 1 and stores.
    """
    results = smc_results
    assert results["loaded"] == 0x1111_1111, "LL is a plain load of the word"
    assert results["contended_cmtl"] == 0
    assert results["memory_after"] == 0x1111_1111, "the discarded commit wrote nothing"
    assert results["link_data"] == 0xAAAA_0001, "LinkData is still A"
    status = results["link_status"]
    assert status & 1 == 1, "LinkdtV is still set"
    assert status & 2 == 0, "LinkV was cleared by the second SL"
    assert (status >> 2) & 7 == 0, "the monitor belongs to core 0"
    assert (status & 0x00FF_FFE0) == 0, "the word is the first granule of the shared L2"
    assert results["control_cmtl"] == 1
    assert results["control_memory"] == 0xC0DE_0003


def test_c6472_a_user_mode_write_to_an_smc_register_raises_event_63_on_all_six_cores(
    smc_results: dict[str, int],
) -> None:
    """A User-mode store to SL2PFCPEN is refused, recorded and raised everywhere.

    SPRUEG5D section 5.1: the SMC "generates an exception back to that C64x+ megamodule as well as
    to other C64x+ megamodules connected to it", and section 5.3 puts the accessing megamodule's
    id and mode in SL2MPFSR and the address in SL2MPFAR (MODE reads 1, nonsecure: a User-mode
    write). SPRS612G Table 7-10 makes that exception system event 63, latched in every core's
    EVTFLAG1 bit 31. A Supervisor-mode write of the same register, made first, raises nothing.
    """
    results = smc_results
    assert results["supervisor_flag"] == 0, "a Supervisor write is not a fault"
    assert results["prefetch_enable"] == 0, "the refused write did not reach the register"
    assert results["fault_status"] == (0 << 2) | (1 << 1), "core 0, nonsecure"
    assert results["fault_address"] == 0x02AA_8000
    assert [results[f"done{core}"] for core in range(CORES)] == [1] * CORES
    assert [results[f"seen{core}"] for core in range(CORES)] == [1] * CORES
