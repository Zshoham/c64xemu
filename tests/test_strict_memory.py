"""`C64XEMU_STRICT_PHYSICAL_MEMORY=1` end to end, through the batch runner.

A program that loads from reserved memory runs to its exit without the diagnostic - the
emulator serves the hole sparsely - and with it stops at the access as a fault: the normal state
report on stdout naming the violation, the core and where it stopped, the fault on stderr, and
the runner's fault exit status rather than an abort. 00A0 0000h is reserved on both devices:
past the C6455's 2 MB of L2 (SPRS276M Table 2-2) and past the C6472's 608 KB (SPRS612G
Table 2-2).
"""

from __future__ import annotations

import json
import os
import subprocess
from dataclasses import replace
from pathlib import Path

import pytest

from target_programs import build_target_program, load_programs

HOLE = 0x00A0_0000

SOURCE = f"""
int main(void)
{{
    volatile unsigned int *hole = (volatile unsigned int *)0x{HOLE:08X}u;
    return (int)(*hole + 7u);
}}
"""

STRICT_MESSAGE = (
    f"strict physical-memory: read of 4 bytes by Core(0) at 0x{HOLE:08x} enters a reserved, "
    f"unsupported, or cache-converted range at 0x{HOLE:08x}"
)


@pytest.fixture(scope="module")
def hole_reader(tmp_path_factory: pytest.TempPathFactory) -> Path:
    workdir = tmp_path_factory.mktemp("strict-memory")
    source = workdir / "hole_reader.c"
    source.write_text(SOURCE, encoding="utf-8")
    # The plain program shape every non-RTS case uses: crt.asm's _start and C$$EXIT, and the
    # shared link command file, which places everything at the base of L2 on either device.
    template = next(program for program in load_programs() if program.name == "sum_loop")
    program = replace(template, name="hole_reader", sources=(str(source),), cflags=("-O2",))
    image = build_target_program(program, workdir)
    assert image is not None, "failed to build the hole reader"
    return image


def run(candidate: Path, image: Path, device: str, strict: bool) -> subprocess.CompletedProcess[str]:
    environment = {
        name: value
        for name, value in os.environ.items()
        if name != "C64XEMU_STRICT_PHYSICAL_MEMORY"
    }
    if strict:
        environment["C64XEMU_STRICT_PHYSICAL_MEMORY"] = "1"
    return subprocess.run(
        [str(candidate), "run", "--device", device, "--max-cycles", "200000", str(image)],
        capture_output=True,
        text=True,
        timeout=120,
        env=environment,
        check=False,
    )


@pytest.mark.parametrize("device", ["c6455", "c6472"])
def test_a_reserved_read_runs_through_without_the_diagnostic(
    candidate_binary: Path, hole_reader: Path, device: str
) -> None:
    completed = run(candidate_binary, hole_reader, device, strict=False)
    assert completed.returncode == 7, completed.stderr
    state = json.loads(completed.stdout)
    assert state["fault"] is None
    assert "strict_violation" not in state


@pytest.mark.parametrize("device", ["c6455", "c6472"])
def test_a_strict_violation_is_a_reported_fault_not_an_abort(
    candidate_binary: Path, hole_reader: Path, device: str
) -> None:
    completed = run(candidate_binary, hole_reader, device, strict=True)
    # The runner's fault status: the state report is written first, then the error exits 1.
    # An abort would be 134 (SIGABRT) with nothing on stdout.
    assert completed.returncode == 1, completed.stderr
    state = json.loads(completed.stdout)
    violation = state["strict_violation"]
    assert violation["message"] == STRICT_MESSAGE
    assert violation["operation"] == "read"
    assert violation["length"] == 4
    assert violation["requester"] == "Core(0)"
    assert violation["start"] == violation["first"] == f"0x{HOLE:08x}"
    # On the six-core device every core runs the same program and makes the same load in the
    # same cycle; core 0 is first, and the first violation is the one kept.
    assert violation["core"] == 0
    assert violation["role"] == "the core that made the access"
    assert violation["cycles"] > 0
    assert violation["cycles"] == state["cycles"]
    assert violation["pc"] == state["pc"]
    assert state["fault"] == (
        f"{STRICT_MESSAGE} (reported on core 0, the core that made the access; stopped at PC "
        f"0x{state['pc']:08x}, cycle {state['cycles']})"
    )
    assert state["halted"] is False
    assert state["cores"][0]["fault"] == state["fault"]
    assert len(state["registers"]) == 64
    assert f"fault: {state['fault']}" in completed.stderr
    assert "panicked" not in completed.stderr
