from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest

from common import BIOS_IMAGE, ROOT, Case, assert_case, case_id, run_tool

def rtos_cases(
    candidate: Path | None, state_path: Path | None = None
) -> tuple[list[Case], dict[str, Any]]:
    """Run TI's kernel, not a peripheral: tasks, semaphores, a Swi, a Clock and nested interrupts.

    One firmware run, several independent statements. Every count it prints is fixed by the
    program rather than by a timer, so each group below is exact on its own: sixty-four hand-offs
    down a chain of three tasks; sixteen rounds of an interrupt that posts a Swi, wakes a task and
    is itself preempted by a higher-priority interrupt raised from inside its own service routine;
    a seventeenth round that shows an interrupt raised while its enable bit is clear is still
    taken once the bit returns; four periodic Clock callbacks and two Task_sleeps, both of which
    only timer 0 can end; and sixteen messages through a Mailbox. The checksum folds all of them
    plus the two event flags, which their service routines clear. Two more statements ride the
    same run: an Event whose and-mask needs one half posted by a task and the other from
    interrupt context, and a nested Hwi_disable where the inner restore must not let the pending
    interrupt in because the outer disable is still in force.
    """
    groups: tuple[tuple[str, dict[str, Any], int], ...] = (
        ("task-handoffs", {"consumed": 64, "relayed": 64}, 20),
        (
            "interrupt-nesting",
            {"outer": 22, "inner": 22, "nested": 22, "failures": 0},
            25,
        ),
        ("swi-and-wake-from-interrupt", {"swi": 22, "woken": 16}, 20),
        ("masked-interrupt-still-delivered", {"masked": 1}, 15),
        ("clock-tick-and-sleep", {"ticks": 4, "slept": 2}, 20),
        ("mailbox", {"letters": 16}, 15),
        # Event's and-mask: neither half alone releases the waiting task, and one of the two
        # halves is posted from interrupt context.
        ("event-both-halves", {"events": 4}, 20),
        # Nested Hwi_disable: the inner restore leaves interrupts off because the outer disable
        # still stands, and only undoing that one delivers the interrupt already pending.
        ("nested-hwi-disable", {"held": 1, "released": 1}, 20),
        (
            "checksum-and-exit",
            {"checksum": 3410629677, "halted": True, "fault": None},
            25,
        ),
    )
    cases = [
        Case(
            request={"id": f"rtos:{name}"},
            expected=expected,
            weight=weight,
            domain="rtos",
        )
        for name, expected, weight in groups
    ]
    if candidate is None or not BIOS_IMAGE.is_file():
        return cases, {}
    state_path = state_path or ROOT / "target" / "bios-kernel-state.json"
    executed = run_tool(
        [
            str(candidate),
            "run",
            str(BIOS_IMAGE),
            "--max-cycles",
            "20000000",
            "--state-out",
            str(state_path),
        ],
        ROOT,
        timeout=300,
    )
    if executed is None or not state_path.is_file():
        return cases, {}
    try:
        state = json.loads(state_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return cases, {}
    # "bios kernel consumed 64 relayed 64 ..." - name and value alternating after the prefix.
    words = (state.get("cio_output") or "").split()
    counts: dict[str, Any] = {}
    for index in range(2, len(words) - 1, 2):
        try:
            counts[words[index]] = int(words[index + 1])
        except ValueError:
            continue
    counts["halted"] = bool(state.get("halted"))
    counts["fault"] = state.get("fault")
    results = {case.request["id"]: counts for case in cases}
    return cases, results

RTOS_CASES = rtos_cases(None)[0]

@pytest.fixture(scope="session")
def rtos_results(
    candidate_binary: Path, tmp_path_factory: pytest.TempPathFactory
) -> dict[str, Any]:
    state_path = tmp_path_factory.mktemp("rtos") / "state.json"
    return rtos_cases(candidate_binary, state_path)[1]

@pytest.mark.slow
@pytest.mark.rtos
@pytest.mark.parametrize("case", RTOS_CASES, ids=case_id)
def test_rtos_case(case: Case, rtos_results: dict[str, Any]) -> None:
    assert_case(case, rtos_results)
