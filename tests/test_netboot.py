"""Booting the six-core device over Ethernet, the way a board does.

With BOOTMODE[3:0] strapped to 9 or 10 the C6472's on-chip ROM runs on core 0 while the other
five cores stay held in reset, announces itself with a BOOTP-shaped frame, and then accepts UDP
frames carrying a boot table (SPRUEC6G section 4.3.5). The table is what places every core's
image - each core's memory is reachable at its global alias - and what starts the cores, by
writing DSP_BOOT_ADDRn and BOOT_COMPLETE_STAT as ordinary sections.

Nothing here is a shortcut: `tests/tools/emac_boot.py` is a host program speaking that protocol
over the emulator's Unix frame transport, one datagram per Ethernet frame, and the emulator is
given no image to load - the positional ELF supplies only symbols, so a program that runs at all
can only have arrived over the wire. The frame-level checks live in `src/romboot.rs`; this file
pins what the command line does with them.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from common import ROOT
from target_programs import build_target_program, load_programs

TOOL = ROOT / "tests/tools/emac_boot.py"

# The same small deterministic program the immediate-boot cases use: it ends at C$$EXIT with a
# known value, which is all a boot case needs from the code the cores run.
PROGRAM = "sum_loop"
EXPECTED_EXIT_CODE = 55
CORES = 6
# The address the emulator fuses into the MAC ID words unless `--mac-id` says otherwise.
DEFAULT_MAC = "02:00:00:00:64:72"


@pytest.fixture(scope="module")
def image(tmp_path_factory: pytest.TempPathFactory) -> Path:
    workdir = tmp_path_factory.mktemp("netboot")
    program = next((program for program in load_programs() if program.name == PROGRAM), None)
    if program is None:
        pytest.skip(f"no {PROGRAM} target program")
    built = build_target_program(program, workdir)
    if built is None:
        pytest.skip(f"could not build the {PROGRAM} target program")
    return built


def netboot(
    candidate: Path,
    image: Path,
    workdir: Path,
    cores: list[int],
    release: str | None = None,
    extra: list[str] | None = None,
    boot_mode: str = "9",
    emulator_extra: list[str] | None = None,
) -> tuple[dict, dict]:
    """Boot `cores` over Ethernet and return the sender's report and the emulator's state.

    The sender binds first: the announcement is sent once, a few cycles into the boot, and is
    never retransmitted, so a host that is not listening yet has missed the boot.
    """
    emulator_socket = workdir / "emu.sock"
    host_socket = workdir / "host.sock"
    state = workdir / "state.json"
    arguments = [
        sys.executable,
        str(TOOL),
        "--socket",
        str(host_socket),
        "--peer",
        str(emulator_socket),
        "--json",
    ]
    for core in cores:
        arguments += ["--core", f"{core}:{image}"]
    if release is not None:
        arguments += ["--release", release]
    arguments += extra or []
    sender = subprocess.Popen(arguments, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + 30
        while not os.path.exists(host_socket):
            if time.monotonic() > deadline or sender.poll() is not None:
                out, err = sender.communicate(timeout=10)
                pytest.fail(f"the sender never bound its socket: {err}{out}")
            time.sleep(0.02)
        emulator = subprocess.run(
            [
                str(candidate),
                "run",
                str(image),
                "--device",
                "c6472",
                "--boot-mode",
                boot_mode,
                *(emulator_extra or []),
                "--net-unix",
                str(emulator_socket),
                "--net-peer",
                str(host_socket),
                "--max-cycles",
                "100000000",
                "--state-out",
                str(state),
            ],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=300,
        )
        sender_out, sender_err = sender.communicate(timeout=60)
    finally:
        if sender.poll() is None:
            sender.kill()
    assert sender.returncode == 0, f"sender failed: {sender_err}{emulator.stderr}"
    assert state.is_file(), f"no state dump: {emulator.stderr}"
    return json.loads(sender_out), json.loads(state.read_text())


def test_c6472_boots_every_core_from_udp_frames(
    candidate_binary: Path, image: Path, tmp_path: Path
) -> None:
    """One image per core over the wire, and all six cores run it.

    The sender learns the DSP's address from the announcement, sends the table, and the boot
    ends where the table's entry point says. Every core runs because the table gave cores 1-5 a
    DSP_BOOT_ADDR and set all six BOOT_COMPLETE_STAT bits; nothing was preloaded, so the code
    they execute can only be the code the frames carried.
    """
    report, state = netboot(candidate_binary, image, tmp_path, list(range(CORES)))

    assert report["ready"] is True, "the announcement was seen"
    assert report["dsp_mac"] == DEFAULT_MAC
    assert report["device_id"] == "0000"
    assert report["release"] == 0b111111
    assert report["frames"] > 0
    boot = state["ethernet_boot"]
    assert boot["frames_accepted"] == report["frames"]
    assert boot["frames_discarded"] == 0
    assert boot["table_bytes"] == report["table_bytes"], "every table byte arrived"
    assert boot["entry"] == report["entry"]

    assert state["device"] == "c6472"
    assert state["exit_code"] == EXPECTED_EXIT_CODE
    cores = state["cores"]
    assert len(cores) == CORES
    for core in cores:
        assert core["held_in_reset"] is False, f"core {core['core']} never started"
        assert core["fault"] is None, f"core {core['core']} faulted"
        assert core["cycles"] > 0, f"core {core['core']} never ran"


def test_c6472_netboot_starts_only_the_cores_the_table_names(
    candidate_binary: Path, image: Path, tmp_path: Path
) -> None:
    """A table that carries two images and releases two cores leaves the other four held.

    Placement is per core: each image goes to its own core's memory through that core's global
    alias, and a core with no image is never given a boot address and never released.
    """
    report, state = netboot(candidate_binary, image, tmp_path, [0, 3])

    assert report["release"] == 0b1001
    boot = state["ethernet_boot"]
    assert boot["frames_discarded"] == 0
    assert boot["entry"] == report["entry"]

    running = {core["core"] for core in state["cores"] if not core["held_in_reset"]}
    assert running == {0, 3}
    for core in state["cores"]:
        assert core["fault"] is None, f"core {core['core']} faulted"
        if core["core"] in running:
            assert core["cycles"] > 0, f"core {core['core']} never ran"
        else:
            assert core["cycles"] == 0, f"core {core['core']} ran while held"
    assert state["exit_code"] == EXPECTED_EXIT_CODE


def test_c6472_netboot_streams_a_table_split_across_hundreds_of_frames(
    candidate_binary: Path, image: Path, tmp_path: Path
) -> None:
    """The same table cut into one-word frames, with nothing released until it ends.

    Every record then straddles a frame boundary, the sequence numbers have to be consecutive
    over hundreds of frames, and the ten receive descriptors the boot code posts are recycled
    twenty times over. The table names no BOOT_COMPLETE_STAT bits, so the only core that ever
    runs is the one the ROM itself starts when the table's terminator arrives - which is also
    what makes the case a claim about the whole stream rather than about how far it got before
    a released core reached the exit point.
    """
    report, state = netboot(
        candidate_binary,
        image,
        tmp_path,
        list(range(CORES)),
        extra=["--payload", "4", "--gap", "0.001", "--no-release"],
    )

    assert report["release"] is None
    assert report["frames"] == report["table_bytes"] // 4
    assert report["frames"] > 100, "the table really is split into hundreds of frames"
    boot = state["ethernet_boot"]
    assert boot["frames_accepted"] == report["frames"]
    assert boot["frames_discarded"] == 0
    assert boot["table_bytes"] == report["table_bytes"]
    assert boot["entry"] == report["entry"]

    # Core 0 is started by the ROM, not by the table, so it runs its image and exits; the rest
    # were never released and never fetched an instruction.
    assert state["exit_code"] == EXPECTED_EXIT_CODE
    for core in state["cores"]:
        held = core["core"] != 0
        assert core["held_in_reset"] is held, f"core {core['core']} is in the wrong state"
        assert core["fault"] is None, f"core {core['core']} faulted"
        assert (core["cycles"] == 0) is held, f"core {core['core']} ran when it should not"


def test_c6472_boots_over_the_second_mac_with_its_own_address_and_port(
    candidate_binary: Path, image: Path, tmp_path: Path
) -> None:
    """Boot mode 10 is the same boot over MAC port 1, with the board's own address and port.

    SPRS612G Table 2-4 gives the two MAC ports their own boot modes, so the port the wire is on
    is a strap rather than a constant; the fused MAC ID and the UDP port the boot listens on are
    likewise the board's, and the host has to name the same ones for a single frame to be
    accepted at all.
    """
    mac = "aa:bb:cc:dd:ee:01"
    report, state = netboot(
        candidate_binary,
        image,
        tmp_path,
        [0, 5],
        extra=["--dest-port", "5000"],
        boot_mode="10",
        emulator_extra=["--mac-id", mac, "--boot-udp-port", "5000"],
    )

    assert report["dsp_mac"] == mac, "the announcement carries the fused address"
    boot = state["ethernet_boot"]
    assert boot["frames_accepted"] == report["frames"]
    assert boot["frames_discarded"] == 0
    assert boot["entry"] == report["entry"]
    assert {core["core"] for core in state["cores"] if not core["held_in_reset"]} == {0, 5}
    assert state["exit_code"] == EXPECTED_EXIT_CODE
