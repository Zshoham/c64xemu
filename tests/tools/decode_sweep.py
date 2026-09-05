#!/usr/bin/env python3
"""Decode every instruction word in binutils' tic6x test vectors and compare mnemonics.

The conformance suite samples this corpus; this tool runs all of it, including the vectors
assembled for `c674x`, which the suite skips because that architecture is a superset: it adds the
C67x floating-point instructions, which a C64x+ does not have. Those mnemonics are listed below
and expected to decode to nothing at all; everything else must match binutils' own rendering.

Exit status is non-zero when any non-floating-point word disagrees, so this is a check and not a
report.
"""
from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
VECTORS = ROOT / "tests/build/binutils-tic6x-vectors"
EMULATOR = ROOT / "target/release/c64xemu"

# The C67x floating-point instruction set, which the C674x vectors exercise and a C64x+ lacks
# (SPRUFE8B chapter 3 marks each of these C67x, and none of them appears in the C64x+ tables).
FLOATING_POINT = {
    "absdp",
    "abssp",
    "adddp",
    "addsp",
    "cmpeqdp",
    "cmpeqsp",
    "cmpgtdp",
    "cmpgtsp",
    "cmpltdp",
    "cmpltsp",
    "dpint",
    "dpsp",
    "dptrunc",
    "intdp",
    "intdpu",
    "intsp",
    "intspu",
    "mpydp",
    "mpyi",
    "mpyid",
    "mpyspdp",
    "mpysp2dp",
    "mpysp",
    "rcpdp",
    "rcpsp",
    "rsqrdp",
    "rsqrsp",
    "spdp",
    "spint",
    "sptrunc",
    "subdp",
    "subsp",
}

# Where binutils renders the underlying operation and TI's tools render the pseudo-operation TI's
# assembler emits it for. `cl6x -mv6400+` assembles `MV .S1 A2,A3` to the very word binutils
# prints as `or .S1 0,a2,a3`, and `dis6x` prints MV for it, so either name is right.
ALIASES = {("or", "mv")}

LINE = re.compile(r"\s([0-9a-fA-F]{8})\[ \\t\]\+(.+)$")
MNEMONIC = re.compile(r"^([A-Za-z][A-Za-z0-9_]*)")


def collect() -> tuple[list[dict[str, object]], dict[str, tuple[str, str]]]:
    if not VECTORS.is_dir():
        # The corpus is provisioned, not checked in, so an unprovisioned tree would otherwise
        # sweep nothing and report a clean run over zero words.
        raise SystemExit(f"missing {VECTORS} - run tests/tools/fetch_deps.sh first")
    requests: list[dict[str, object]] = []
    expected: dict[str, tuple[str, str]] = {}
    for path in sorted(VECTORS.glob("*.d")):
        if "bad" in path.stem:
            continue
        text = path.read_text(errors="replace")
        header = "\n".join(text.splitlines()[:8]).lower()
        for line_no, line in enumerate(text.splitlines(), 1):
            match = LINE.search(line)
            if not match:
                continue
            rendered = match.group(2).replace("\\", "").strip()
            if rendered.startswith("<"):
                continue
            rendered = re.sub(r"^\|\|\^?\s*", "", rendered)
            rendered = re.sub(r"^\[[^]]+\]\s*", "", rendered)
            name = MNEMONIC.match(rendered)
            if not name:
                continue
            case_id = f"{path.stem}:{line_no}"
            requests.append({"id": case_id, "word": int(match.group(1), 16)})
            expected[case_id] = (name.group(1).lower(), rendered)
    return requests, expected


def decode(requests: list[dict[str, object]]) -> dict[str, object]:
    results: dict[str, object] = {}
    for start in range(0, len(requests), 512):
        chunk = requests[start : start + 512]
        completed = subprocess.run(
            [str(EMULATOR), "decode", *[f"{request['word']:08x}" for request in chunk]],
            capture_output=True,
            text=True,
            check=False,
        )
        if completed.returncode != 0:
            raise RuntimeError(completed.stderr.strip()[:500])
        rows = completed.stdout.splitlines()
        if len(rows) != len(chunk):
            raise RuntimeError(
                f"decoder returned {len(rows)} rows for {len(chunk)} instructions"
            )
        for request, row in zip(chunk, rows):
            results[str(request["id"])] = json.loads(row)
    return results


def main() -> int:
    requests, expected = collect()
    if not requests:
        print("no vectors found", file=sys.stderr)
        return 1
    try:
        results = decode(requests)
    except (OSError, RuntimeError, json.JSONDecodeError) as error:
        print(f"decoder failed: {error}", file=sys.stderr)
        return 1
    matched = 0
    skipped = 0
    failures: list[str] = []
    for case_id, (want, rendered) in expected.items():
        got = (results.get(case_id) or {}).get("mnemonic")
        if want in FLOATING_POINT:
            skipped += 1
            if got is not None:
                failures.append(f"{case_id}: {rendered} -> decoded as {got}, but it is C67x only")
            continue
        if got == want or (want, got) in ALIASES:
            matched += 1
        else:
            failures.append(f"{case_id}: {rendered} -> expected {want}, got {got}")
    print(f"words {len(expected)}  matched {matched}  floating-point skipped {skipped}")
    for failure in failures[:40]:
        print(f"  {failure}")
    if failures:
        print(f"{len(failures)} disagreements")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
