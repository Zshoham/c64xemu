from __future__ import annotations

import re
import sys
from pathlib import Path
from typing import Any

import pytest

from common import BIOS_IMAGE, NDK_IMAGE, ROOT, Case, assert_case, case_id, run_tool

def differential_cases(
    candidate: Path | None, program_images: list[Path] | None = None
) -> tuple[list[Case], dict[str, Any]]:
    """Run the two decoder differentials that use TI's own tools as the oracle.

    `tools/decode_sweep.py` takes every instruction word in binutils' tic6x vectors, including the
    files assembled for the C674x superset, and requires each mnemonic to match binutils' own
    rendering while the C67x floating-point words decode to nothing. `tools/dis6x_diff.py`
    disassembles the firmware images with TI's `dis6x` and compares every instruction, compact ones
    against their fetch packet's header. Both are checks rather than reports: they exit non-zero on
    any disagreement, and the counts below are what a clean run prints.
    """
    sweep_case = Case(
        request={"id": "decoder:binutils-vector-sweep"},
        expected={"exit": 0, "words": 2195, "matched": 2032, "skipped": 163},
        weight=150,
        domain="decoder",
    )
    # The counts float with the firmware and the programs, so what is asserted is that every
    # image was disassembled and that nothing disagreed.
    dis6x_case = Case(
        request={"id": "decoder:dis6x-image-differential"},
        expected={"exit": 0, "images": 2, "disagreeing": 0},
        weight=150,
        domain="decoder",
    )
    programs_case = Case(
        request={"id": "decoder:dis6x-program-differential"},
        expected={"exit": 0, "disagreeing": 0},
        weight=150,
        domain="decoder",
    )
    cases = [sweep_case, dis6x_case, programs_case]
    if candidate is None:
        return cases, {}
    results: dict[str, Any] = {}
    sweep = run_tool([sys.executable, str(ROOT / "tests/tools/decode_sweep.py")], ROOT, timeout=120)
    if sweep is not None:
        numbers = re.search(
            r"words (\d+)\s+matched (\d+)\s+floating-point skipped (\d+)", sweep.stdout
        )
        if numbers:
            results[sweep_case.request["id"]] = {
                "exit": sweep.returncode,
                "words": int(numbers.group(1)),
                "matched": int(numbers.group(2)),
                "skipped": int(numbers.group(3)),
            }
    if NDK_IMAGE.is_file() and BIOS_IMAGE.is_file():
        differential = run_tool(
            [sys.executable, str(ROOT / "tests/tools/dis6x_diff.py")], ROOT, timeout=300
        )
        if differential is not None:
            numbers = re.search(r"total (\d+), disagreeing (\d+)", differential.stdout)
            if numbers:
                results[dis6x_case.request["id"]] = {
                    "exit": differential.returncode,
                    "images": differential.stdout.count(" instructions ("),
                    "disagreeing": int(numbers.group(2)),
                }
    if program_images:
        differential = run_tool(
            [sys.executable, str(ROOT / "tests/tools/dis6x_diff.py"), *map(str, program_images)],
            ROOT,
            timeout=600,
        )
        if differential is not None:
            numbers = re.search(r"total (\d+), disagreeing (\d+)", differential.stdout)
            if numbers:
                results[programs_case.request["id"]] = {
                    "exit": differential.returncode,
                    "disagreeing": int(numbers.group(2)),
                }
    return cases, results

DIFFERENTIAL_CASES = differential_cases(None)[0]

@pytest.fixture(scope="session")
def differential_results(
    candidate_binary: Path,
    program_execution: tuple[list[Case], dict[str, Any], list[Path]],
) -> dict[str, Any]:
    program_images = program_execution[2]
    return differential_cases(candidate_binary, program_images)[1]

@pytest.mark.slow
@pytest.mark.differential
@pytest.mark.parametrize("case", DIFFERENTIAL_CASES, ids=case_id)
def test_differential_case(case: Case, differential_results: dict[str, Any]) -> None:
    assert_case(case, differential_results)
