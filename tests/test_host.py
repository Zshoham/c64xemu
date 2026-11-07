from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest

from common import CL6X, ROOT, Case, assert_case, case_id, run_tool
from target_programs import Program, build_target_program

HPI_FIRMWARE = ROOT / "tests/firmware/hpi_mailbox"
HPI_HOST = ROOT / "tests/tools/hpi_host.py"
HPI_EXCHANGES = 4

def hpi_expected_answers() -> list[int]:
    """What the firmware must answer, derived here rather than taken from the tool.

    Exchange *i* sends four words, `i * 0x01010101 + step * 7 + 1`, and asks for their sum on even
    rounds and for the first word's bytes reversed on odd ones.
    """
    answers = []
    for index in range(HPI_EXCHANGES):
        payload = [(index * 0x0101_0101 + step * 7 + 1) & 0xFFFF_FFFF for step in range(4)]
        if index % 2 == 0:
            answers.append(sum(payload) & 0xFFFF_FFFF)
        else:
            answers.append(int.from_bytes(payload[0].to_bytes(4, "little"), "big"))
    return answers

def host_cases(candidate: Path | None, workroot: Path) -> tuple[list[Case], dict[str, Any]]:
    """A process on the host exchanging a mailbox with firmware through the HPI.

    The host writes a request block into DSP memory through HPID, raises DSPINT, and waits for
    HINT; the firmware takes DSPINT as a real CPU interrupt through INTMUX, answers in memory and
    raises HINT, acknowledging DSPINT the way SPRU969B section 5.1 requires. Nothing here is
    timing-dependent: the answers and the firmware's own count are what is asserted.
    """
    answers = hpi_expected_answers()
    groups: tuple[tuple[str, dict[str, Any], int], ...] = (
        (
            "hpi-mailbox-every-round-answered",
            {"answered": [True] * HPI_EXCHANGES},
            30,
        ),
        ("hpi-mailbox-answers", {"answers": answers}, 40),
        (
            "hpi-mailbox-sequence",
            {"sequence": list(range(1, HPI_EXCHANGES + 1))},
            30,
        ),
        (
            "hpi-mailbox-firmware-count",
            {"firmware": ["hpi mailbox ready", f"hpi mailbox served {HPI_EXCHANGES} last 2"]},
            30,
        ),
    )
    cases = [
        Case(request={"id": f"host:{name}"}, expected=expected, weight=weight, domain="host")
        for name, expected, weight in groups
    ]
    if candidate is None or not HPI_HOST.is_file() or not CL6X.is_file():
        return cases, {}
    workdir = workroot / "hpi_mailbox"
    workdir.mkdir(parents=True, exist_ok=True)
    image = build_mailbox(workdir)
    if image is None:
        return cases, {}
    executed = run_tool(
        [
            sys.executable,
            str(HPI_HOST),
            "--emulator",
            str(candidate),
            "--image",
            str(image),
            "--exchanges",
            str(HPI_EXCHANGES),
            "--report",
            "--timeout",
            "120",
        ],
        ROOT,
        timeout=180,
    )
    if executed is None or executed.returncode != 0:
        return cases, {}
    try:
        report = json.loads(executed.stdout.strip().splitlines()[-1])
    except (json.JSONDecodeError, IndexError):
        return cases, {}
    exchanges = report.get("exchanges", [])
    observed = {
        "answered": [bool(exchange.get("answered")) for exchange in exchanges],
        "answers": [exchange.get("answer") for exchange in exchanges],
        "sequence": [exchange.get("sequence") for exchange in exchanges],
        "firmware": report.get("firmware"),
    }
    return cases, {case.request["id"]: observed for case in cases}

def build_mailbox(workdir: Path) -> Path | None:
    """The mailbox firmware, built with the TI toolchain."""
    program = Program(
        name="hpi_mailbox",
        weight=0,
        sources=(str(HPI_FIRMWARE / "main.c"), str(HPI_FIRMWARE / "vectors.asm")),
        cflags=("-O2",),
        includes=(),
        max_cycles=200_000_000,
        opt_levels=("-O2",),
        rts=True,
        expect={},
        cio_output=None,
        rx_frames=(),
        tx_frames=None,
        entry="_c_int00",
        link_flags=(),
        boot_table=None,
        boot_config=None,
    )
    return build_target_program(program, workdir)

HOST_CASES = host_cases(None, Path())[0]

@pytest.fixture(scope="session")
def host_results(
    candidate_binary: Path, tmp_path_factory: pytest.TempPathFactory
) -> dict[str, Any]:
    return host_cases(candidate_binary, tmp_path_factory.mktemp("host"))[1]

@pytest.mark.slow
@pytest.mark.host
@pytest.mark.parametrize("case", HOST_CASES, ids=case_id)
def test_host_case(case: Case, host_results: dict[str, Any]) -> None:
    assert_case(case, host_results)


# ---- the same host, replayed from a file -------------------------------------------------------


@pytest.fixture(scope="module")
def mailbox_replay(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, Path]:
    """The mailbox firmware, and its host written as an --hpi-replay file with every answer
    checked: no host process, so nothing about the run depends on wall-clock time."""
    if not CL6X.is_file():
        pytest.skip("no TI toolchain")
    workdir = tmp_path_factory.mktemp("hpi_mailbox_replay")
    image = build_mailbox(workdir)
    assert image is not None, "could not build the mailbox firmware"
    replay = workdir / "mailbox.csv"
    subprocess.run(
        [sys.executable, str(HPI_HOST), "--image", str(image), "--exchanges", str(HPI_EXCHANGES),
         "--emit-replay", str(replay)],
        cwd=ROOT, check=True, timeout=60,
    )
    return image, replay


def replay_mailbox(
    candidate: Path, image: Path, replay: Path, state: Path, *extra: str,
) -> subprocess.CompletedProcess:
    return subprocess.run(
        [str(candidate), "run", str(image), "--hpi-replay", str(replay),
         "--max-cycles", "200000000", "--state-out", str(state), *extra],
        cwd=ROOT, capture_output=True, text=True, timeout=180,
    )


@pytest.mark.slow
@pytest.mark.host
def test_a_replayed_mailbox_host_is_answered_and_independent_of_batching(
    candidate_binary: Path, mailbox_replay: tuple[Path, Path], tmp_path: Path,
) -> None:
    image, replay = mailbox_replay
    states = []
    for name, extra in (("batched", ("--cache",)), ("stepped", ("--cache", "--pc-trail", "1"))):
        state = tmp_path / f"{name}.json"
        result = replay_mailbox(candidate_binary, image, replay, state, *extra)
        assert result.returncode == 0, result.stderr[-2000:]
        states.append(json.loads(state.read_text()))
    batched, stepped = states
    assert batched["fault"] is None
    assert batched["halted"] is True
    assert batched["cio_output"] == (
        f"hpi mailbox ready\nhpi mailbox served {HPI_EXCHANGES} last 2\n"
    )
    assert batched["hpi_replay"]["complete"] is True
    assert batched["hpi_replay"]["failure"] is None
    rows = replay.read_text().splitlines()
    answers = [int(row.split(",")[3], 16) for row in rows if ",data-auto,read," in row][1::2]
    assert answers == hpi_expected_answers(), "the file checks the answers derived here"
    assert batched == stepped, "the run does not depend on how the run loop batches"


@pytest.mark.slow
@pytest.mark.host
def test_a_program_that_exits_before_the_replay_ends_fails_the_run(
    candidate_binary: Path, mailbox_replay: tuple[Path, Path], tmp_path: Path,
) -> None:
    image, replay = mailbox_replay
    longer = tmp_path / "longer.csv"
    text = replay.read_text()
    longer.write_text(text + "+1000000000,control,read,-\n")
    line = len(text.splitlines()) + 1
    state = tmp_path / "state.json"
    result = replay_mailbox(candidate_binary, image, longer, state)
    assert result.returncode == 1
    assert f"hpi replay: program exited before line {line}" in result.stderr
    report = json.loads(state.read_text())
    assert report["halted"] is True
    assert report["fault"] is None
    assert report["hpi_replay"]["complete"] is False
    assert report["hpi_replay"]["next_line"] == line
