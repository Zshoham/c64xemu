from __future__ import annotations

import os
import re
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path
from typing import Any

import pytest

from common import CL6X, Case, assert_case, case_id
from target_programs import (
    DIFFERENTIAL_PROGRAMS,
    Program,
    build_target_program,
    load_programs,
    program_expected_result,
    run_target_program,
    split_weight,
)

OPT_FLAG = re.compile(r"-O[0-3s]")

def program_cases(
    candidate: Path | None,
    workroot: Path,
    selected_ids: set[str] | None = None,
) -> tuple[list[Case], dict[str, Any], list[Path]]:
    """Compile, run and check every program under tests/programs."""
    if candidate is not None and not CL6X.is_file():
        raise RuntimeError(f"TI toolchain missing: {CL6X}")
    cases: list[Case] = []
    results: dict[str, Any] = {}
    variants: list[tuple[Program, str, list[str], list[int], int | None, Path]] = []
    for program in load_programs():
        expected_result = program_expected_result(program)
        checks = ["built", "no-fault", "halted"]
        if expected_result is not None:
            checks.append("result")
        if program.cio_output is not None:
            checks.append("cio")
        if program.tx_frames is not None:
            checks.append("frames")
        # The same program again with L1P, L1D and L2 configured as caches, required to give
        # the same answer: allocation, eviction, the write buffer and the snoop protocol all at
        # once, over every program the suite already builds. A program whose expectation only
        # holds for one cache configuration opts out through its spec.
        if program.cache_rerun:
            checks.append("cached")
        level_weights = split_weight(program.weight, len(program.opt_levels))
        for level, level_weight in zip(program.opt_levels, level_weights):
            # The level under test replaces any -O a spec carries; left in place the compiler
            # honoured the spec's and ignored the level, so every variant was the same build.
            flags = tuple(flag for flag in program.cflags if not OPT_FLAG.fullmatch(flag))
            variant = replace(program, cflags=flags + (level,))
            workdir = workroot / program.name / level.lstrip("-")
            if candidate is not None:
                workdir.mkdir(parents=True, exist_ok=True)
            check_weights = list(zip(checks, split_weight(level_weight, len(checks))))
            if selected_ids is not None:
                check_weights = [
                    (check, weight)
                    for check, weight in check_weights
                    if f"program:{program.name}:{level.lstrip('-')}:{check}" in selected_ids
                ]
            if not check_weights:
                continue
            selected_checks = [check for check, _ in check_weights]
            selected_weights = [weight for _, weight in check_weights]
            variants.append(
                (
                    variant,
                    level,
                    selected_checks,
                    selected_weights,
                    expected_result,
                    workdir,
                )
            )

    if candidate and variants:
        worker_count = min(len(variants), os.cpu_count() or 1)
        with ThreadPoolExecutor(max_workers=worker_count) as executor:
            images = list(
                executor.map(
                    lambda item: build_target_program(item[0], item[5]),
                    variants,
                )
            )
    else:
        images = [None] * len(variants)

    built: list[Path] = []
    for (variant, level, checks, weights, expected_result, workdir), image in zip(
        variants, images
    ):
        state = (
            run_target_program(candidate, variant, image, workdir)
            if (candidate and image)
            else None
        )
        cached_state = (
            run_target_program(candidate, variant, image, workdir, cache=True)
            if (candidate and image and "cached" in checks)
            else None
        )
        # Only the two levels that generate the packed and software-pipelined code worth
        # differentiating: the lower levels repeat encodings these already cover.
        if (
            image is not None
            and level in ("-O3", "-Os")
            and (variant.name in DIFFERENTIAL_PROGRAMS or variant.name.startswith("dsplib_"))
        ):
            built.append(image)
        cases.extend(
            program_variant_cases(
                variant,
                level,
                checks,
                weights,
                expected_result,
                image,
                state,
                cached_state,
                results,
            )
        )
    return cases, results, built

def program_variant_cases(
    program: Program,
    level: str,
    checks: list[str],
    weights: list[int],
    expected_result: int | None,
    image: Path | None,
    state: dict[str, Any] | None,
    cached_state: dict[str, Any] | None,
    results: dict[str, Any],
) -> list[Case]:
    """Cases for one program built at one optimisation level."""
    cases: list[Case] = []
    for check, weight in zip(checks, weights):
        case_id = f"program:{program.name}:{level.lstrip('-')}:{check}"
        if check == "built":
            expected: Any = {"built": True}
            actual: Any = {"built": image is not None}
        elif check == "no-fault":
            expected = {"fault": None}
            actual = {"fault": state.get("fault")} if state else None
        elif check == "halted":
            expected = {"halted": True}
            actual = {"halted": bool(state.get("halted"))} if state else None
        elif check == "result":
            expected = {"A4": expected_result}
            actual = {"A4": int(state.get("registers", {}).get("A4", -1))} if state else None
        elif check == "cio":
            expected = {"cio_output": program.cio_output}
            actual = {"cio_output": state.get("cio_output")} if state else None
        elif check == "cached":
            # One case for the whole answer, because what the cached rerun asserts is that
            # every observable the uncached run is checked for came out the same.
            expected = {"fault": None, "halted": True}
            if expected_result is not None:
                expected["A4"] = expected_result
            if program.cio_output is not None:
                expected["cio_output"] = program.cio_output
            if program.tx_frames is not None:
                expected["tx_frames"] = list(program.tx_frames)
            actual = (
                {
                    "fault": cached_state.get("fault"),
                    "halted": bool(cached_state.get("halted")),
                    "A4": int(cached_state.get("registers", {}).get("A4", -1)),
                    "cio_output": cached_state.get("cio_output"),
                    "tx_frames": cached_state.get("tx_frames"),
                }
                if cached_state
                else None
            )
        else:
            expected = {"tx_frames": list(program.tx_frames or ())}
            actual = {"tx_frames": state.get("tx_frames")} if state else None
        cases.append(
            Case(
                request={"id": case_id, "action": "program"},
                expected=expected,
                weight=weight,
                domain="programs",
            )
        )
        if actual is not None:
            results[case_id] = actual
    return cases

PROGRAM_CASES = program_cases(None, Path())[0]

@pytest.mark.slow
@pytest.mark.programs
@pytest.mark.parametrize("case", PROGRAM_CASES, ids=case_id)
def test_program_case(
    case: Case,
    program_execution: tuple[list[Case], dict[str, Any], list[Path]],
) -> None:
    assert_case(case, program_execution[1])
