from __future__ import annotations

import os
import shlex
import struct
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pytest

ROOT = Path(__file__).resolve().parent.parent
CL6X = ROOT / "tests/build/ti-cgt-c6000_8.3.15/bin/cl6x"
HEX6X = ROOT / "tests/build/ti-cgt-c6000_8.3.15/bin/hex6x"
RTS_LIB_DIR = ROOT / "tests/build/ti-cgt-c6000_8.3.15/lib"
RTS_INCLUDE_DIR = ROOT / "tests/build/ti-cgt-c6000_8.3.15/include"
DEFAULT_OPT_LEVELS = ("-O0", "-O1", "-O2", "-O3", "-Os")
NDK_IMAGE = ROOT / "tests/firmware/ndk_tcp/build/ndk_tcp.out"
BIOS_IMAGE = ROOT / "tests/firmware/bios_kernel/build/bios_kernel.out"

@dataclass(frozen=True)
class Case:
    request: dict[str, Any]
    expected: Any
    weight: int
    domain: str

def run_tool(
    command: list[str], cwd: Path, timeout: int = 300
) -> subprocess.CompletedProcess[str] | None:
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired:
        print(
            f"command timed out after {timeout}s: {shlex.join(command)}",
            file=sys.stderr,
        )
        return None
    except OSError as error:
        print(f"could not run {shlex.join(command)}: {error}", file=sys.stderr)
        return None
    if completed.returncode != 0:
        detail = f"\n{completed.stderr.rstrip()}" if completed.stderr else ""
        print(
            f"command exited {completed.returncode}: {shlex.join(command)}{detail}",
            file=sys.stderr,
        )
    return completed

def symbol_address(
    image: Path, names: tuple[str, ...], kinds: tuple[int, ...] = (2,)
) -> int | None:
    """The value of the first symbol in `names` whose ELF type is in `kinds`, from a TI EABI
    ELF32 image.

    `kinds` holds STT_ values: the default 2 is STT_FUNC; TI's linker types a global variable
    1 (STT_OBJECT) or, when it is uninitialised, 5 (STT_COMMON). The images are
    linked with `--symdebug:none`, so there is no DWARF; the symbol table is the only place
    to learn where `main` or a result variable lives, which moves with the compiler's layout.
    """
    data = image.read_bytes()
    (_, _, _, _, _, _, e_shoff, _, _, _, _, e_shentsize, e_shnum, _) = struct.unpack_from(
        "<16sHHIIIIIHHHHHH", data, 0
    )
    sections = [
        struct.unpack_from("<IIIIIIIIII", data, e_shoff + index * e_shentsize)
        for index in range(e_shnum)
    ]
    strtabs = {
        index: data[sh_offset : sh_offset + sh_size]
        for index, (_, sh_type, _, _, sh_offset, sh_size, _, _, _, _) in enumerate(sections)
        if sh_type == 3  # SHT_STRTAB
    }
    for (_, sh_type, _, _, sh_offset, sh_size, sh_link, _, _, _) in sections:
        if sh_type != 2:  # SHT_SYMTAB
            continue
        strings = strtabs.get(sh_link)
        if strings is None:
            continue
        for entry in range(sh_size // 16):
            st_name, st_value, _, st_info, _, _ = struct.unpack_from(
                "<IIIBBH", data, sh_offset + entry * 16
            )
            if st_info & 0xF not in kinds:
                continue
            end = strings.find(b"\0", st_name)
            if end < 0:
                continue
            if strings[st_name:end].decode("ascii", "replace") in names:
                return st_value
    return None

def matches(expected: Any, actual: Any) -> bool:
    if isinstance(expected, dict):
        return isinstance(actual, dict) and all(key in actual and matches(value, actual[key]) for key, value in expected.items())
    if isinstance(expected, list):
        return isinstance(actual, list) and len(expected) == len(actual) and all(matches(e, a) for e, a in zip(expected, actual))
    return expected == actual

def locate_candidate() -> Path | None:
    override = os.environ.get("C64XEMU_BIN")
    if override:
        candidate = Path(override)
        return candidate if candidate.is_file() else None
    manifest = ROOT / "Cargo.toml"
    if not manifest.is_file():
        return None
    env = os.environ.copy()
    try:
        built = subprocess.run(
            ["cargo", "build", "--release", "--locked", "--bin", "c64xemu"],
            cwd=ROOT,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=300,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        print(f"candidate build unavailable: {error}", file=sys.stderr)
        return None
    if built.returncode != 0:
        print(f"candidate build failed:\n{built.stderr}", file=sys.stderr)
        return None
    candidate = ROOT / "target/release/c64xemu"
    return candidate if candidate.is_file() else None

def case_id(case: Case) -> str:
    return case.request["id"]

def assert_case(case: Case, results: dict[str, Any]) -> None:
    identifier = case_id(case)
    assert identifier in results, f"{identifier} produced no result"
    actual = results[identifier]
    assert matches(case.expected, actual), (
        f"{identifier} did not match\nexpected: {case.expected!r}\nactual:   {actual!r}"
    )

def selected_case_ids(request: pytest.FixtureRequest, test_name: str) -> set[str]:
    selected: set[str] = set()
    for item in request.session.items:
        if getattr(item, "originalname", None) != test_name:
            continue
        callspec = getattr(item, "callspec", None)
        case = callspec.params.get("case") if callspec is not None else None
        if isinstance(case, Case):
            selected.add(case_id(case))
    return selected
