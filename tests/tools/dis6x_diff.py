#!/usr/bin/env python3
"""Compare the decoder against TI's own disassembler over linked images.

Compact 16-bit instructions are compared too. One cannot be decoded on its own - its fetch
packet's header word supplies the data size, saturation and branch fields its meaning depends on -
so the header word `dis6x` prints as `.fphead` is passed with each compact word. That word is the
last of its 32-byte fetch packet, so it is looked up by packet rather than carried forward: the
compact instructions of a packet are printed before their own header.

`dis6x` is the disassembler that ships with the checked-in code generation tools, so it is an
oracle for real compiled code rather than for hand-written vectors: every instruction in the
firmware images this repository builds passes through it. The emulator's `decode` command names
each word independently, and this tool compares that output to the vendor result.

Aliases are listed rather than hidden, because TI's assembler emits one encoding for several
mnemonics: MV is an OR with zero, NOT is an XOR with -1, NEG is a SUB from zero, and ZERO is a
subtraction of a register from itself. dis6x prints the pseudo-operation, the decoder prints the
operation that is encoded, and both are right.

Exit status is non-zero when any instruction disagrees.
"""
from __future__ import annotations

import collections
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
DIS6X = ROOT / "tests/build/ti-cgt-c6000_8.5.0.LTS/bin/dis6x"
EMULATOR = ROOT / "target/release/c64xemu"

# dis6x prints the pseudo-operation TI's assembler accepted; the decoder prints what the bits
# encode. Each pair here is one such pseudo-operation.
# Mnemonics whose unit dis6x prints but the model deliberately does not claim. The 32-bit
# immediate BNOP with its unit bit clear is the C64x+ macro form that uses no functional unit,
# which is how TI's own compiler pairs it with an S-unit instruction in the same execute packet -
# packets those images contain, and which would not validate otherwise. dis6x prints the side
# regardless.
UNIT_NOT_CLAIMED = {"bnop"}

ALIASES = {
    ("mv", "or"),
    ("mv", "add"),
    ("mvk", "add"),
    ("not", "xor"),
    ("neg", "sub"),
    ("zero", "sub"),
    ("zero", "mvk"),
    # A subtraction of a five-bit constant is an addition of its negation: `SUB .L2 B0,1,B13`
    # and `ADD .L2 -1,B0,B13` assemble to the same word, which dis6x prints as the subtraction.
    ("sub", "add"),
    # SWAP2 is PACKLH2 with one source used twice: `SWAP2 .L1 A1,A2` and
    # `PACKLH2 .L1 A1,A1,A2` assemble to the same word.
    ("swap2", "packlh2"),
    ("nop", "nop"),
    ("idle", "idle"),
}

LINE = re.compile(
    r"^([0-9a-f]{8})\s+([0-9a-f]{4}|[0-9a-f]{8})\s+(?:\|\|\s+)?(?:\[\s*(!?[A-Za-z][0-9]+)\s*\]\s+)?"
    r"([.A-Za-z][A-Za-z0-9_]*)(\.[A-Za-z0-9]+)?(.*)$"
)
# The address operand TI's tools print for a memory access: `*+B15[134]`, `*A4++[1]`, `*--B6[2]`.
ADDRESS = re.compile(r"(\*[-+]*[A-Za-z][0-9]+[-+]*\[[^]]*\])")


def normalise_operands(text: str) -> str:
    """dis6x's operand list in one form: lowercase, unspaced, and numbers in decimal.

    Constants are printed in hex or in decimal depending on the instruction, so both sides are
    reduced to decimal before they are compared.
    """
    parts = []
    for token in text.strip().split(","):
        token = token.strip().lower()
        try:
            parts.append(str(int(token, 0)))
        except ValueError:
            parts.append(token)
    return ",".join(parts)


def disassemble(
    image: Path,
) -> list[tuple[int, int, int, str, int, str, str, str, str, str]]:
    completed = subprocess.run(
        [str(DIS6X), "-q", str(image)],
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        print(completed.stderr.strip()[:400], file=sys.stderr)
        return []
    instructions: list[tuple[int, int, int, str, int, str, str, str, str, str]] = []
    headers: dict[int, int] = {}
    parsed: list[tuple[int, int, int, str, str, str, str, str, str]] = []
    for line in completed.stdout.splitlines():
        match = LINE.match(line.strip())
        if not match:
            continue
        address = int(match.group(1), 16)
        word_text = match.group(2)
        predicate = (match.group(3) or "").lower()
        name = match.group(4).lower()
        # The suffix dis6x prints after the mnemonic: the unit, its side, an X for a cross-path
        # read and, on the D unit, the T1/T2 data path. Only the unit, side and cross matter here.
        suffix = (match.group(5) or "").lower().lstrip(".")
        operands = match.group(6) or ""
        address_text = ""
        found = ADDRESS.search(operands)
        if found:
            address_text = found.group(1).lower()
        operand_text = "" if found else normalise_operands(operands)
        unit = ""
        data_path = ""
        if len(suffix) >= 2 and suffix[0] in "lsmd" and suffix[1] in "12":
            unit = suffix[:2] + ("x" if "x" in suffix[2:] else "")
            if "t1" in suffix[2:]:
                data_path = "t1"
            elif "t2" in suffix[2:]:
                data_path = "t2"
        word = int(word_text, 16)
        if name == ".word":
            # Data placed in a code section - string literals and tables - which dis6x prints
            # as words and which no decoder should be asked about.
            continue
        if name == ".fphead":
            # The header of this fetch packet, which its compact instructions decode against.
            headers[address & ~0x1F] = word
            continue
        parsed.append(
            (
                address,
                word,
                len(word_text) * 4,
                name,
                unit,
                predicate,
                data_path,
                address_text,
                operand_text,
            )
        )
    for (
        pc,
        word,
        width,
        name,
        unit,
        predicate,
        data_path,
        address_text,
        operand_text,
    ) in parsed:
        instructions.append(
            (
                pc,
                word,
                width,
                name,
                headers.get(pc & ~0x1F, 0),
                unit,
                predicate,
                data_path,
                address_text,
                operand_text,
            )
        )
    return instructions


def decode(
    instructions: list[tuple[int, int, int, str, int, str, str, str, str, str]],
) -> dict[int, object]:
    results: dict[int, object] = {}
    for start in range(0, len(instructions), 512):
        chunk = instructions[start : start + 512]
        arguments = []
        for _, word, width, _, header, _, _, _, _, _ in chunk:
            arguments.append(f"{word:x}/{header:x}" if width == 16 else f"{word:x}")
        completed = subprocess.run(
            [str(EMULATOR), "decode", *arguments],
            capture_output=True,
            text=True,
            check=False,
        )
        if completed.returncode != 0:
            raise RuntimeError(completed.stderr.strip()[:400])
        rows = completed.stdout.splitlines()
        if len(rows) != len(chunk):
            raise RuntimeError(
                f"decoder returned {len(rows)} rows for {len(chunk)} instructions"
            )
        for index, row in enumerate(rows, start):
            results[index] = json.loads(row)
    return results


def main() -> int:
    images = [Path(argument) for argument in sys.argv[1:]]
    if not images:
        images = sorted(ROOT.glob("tests/firmware/*/build/*.out"))
    total = 0
    operands_compared = 0
    disagreements: collections.Counter[tuple[str, str]] = collections.Counter()
    examples: dict[tuple[str, str], str] = {}
    for image in images:
        instructions = disassemble(image)
        if not instructions:
            print(f"{image}: nothing disassembled", file=sys.stderr)
            return 1
        try:
            results = decode(instructions)
        except (OSError, RuntimeError, json.JSONDecodeError) as error:
            print(f"{image}: decoder failed: {error}", file=sys.stderr)
            return 1
        compact = sum(1 for entry in instructions if entry[2] == 16)
        matched = 0
        for index, (
            address,
            word,
            width,
            want,
            _,
            want_unit,
            want_predicate,
            want_path,
            want_address,
            want_operands,
        ) in enumerate(instructions):
            value = results.get(index) or {}
            got = value.get("mnemonic")
            got_unit = value.get("unit")
            got_predicate = value.get("predicate", "")
            got_path = value.get("data_path", "")
            got_address = value.get("address", "")
            # Both sides go through the same normalisation: a constant printed in hex by one
            # tool and in decimal by the other is not a disagreement about the encoding.
            got_operands = normalise_operands(value.get("operands", ""))
            total += 1
            named = got == want or (want, got) in ALIASES
            # The unit is compared when dis6x printed one; the compact formats it prints without
            # a unit are the ones that claim none.
            unit_agrees = (
                not want_unit
                or got_unit == want_unit
                or (got_unit is None and want in UNIT_NOT_CLAIMED)
            )
            # dis6x prints the predicate for 32-bit instructions only, and the data path only
            # on the D unit's memory accesses.
            predicate_agrees = want_predicate == got_predicate
            path_agrees = not want_path or want_path == got_path
            address_agrees = not want_address or want_address == got_address
            # Operands are compared for every family whose fields the model reads through a
            # shared accessor - the ALU forms, the multiplies, the shifts, the bit-field
            # operations, the constant and control-register moves, the D unit's address
            # arithmetic and the compact families; the rest render nothing and are not
            # compared. An aliased mnemonic is skipped: dis6x prints the pseudo-operation's own
            # operand list, so `MV a10,a5` stands where the encoding is `OR 0,a10,a5` and a
            # subtraction of a constant reorders its sources.
            compared = bool(got_operands) and want == got
            operands_compared += compared
            operands_agree = not compared or got_operands == want_operands
            if not operands_agree and want in ("cmpgtu", "cmpltu"):
                # CMPGTU and CMPLTU take a five-bit unsigned constant on C64x and above -
                # binutils' opcode table says so explicitly, noting that SPRUFE8 does not
                # mention it, and `cl6x` encodes 4 and 20 as different words. `dis6x` prints
                # that constant masked to four bits, so its rendering cannot show the top bit;
                # the comparison drops it too rather than calling the model wrong.
                mine = got_operands.split(",")
                theirs = want_operands.split(",")
                if len(mine) == len(theirs) and mine[1:] == theirs[1:]:
                    try:
                        operands_agree = int(mine[0]) % 16 == int(theirs[0]) % 16
                    except ValueError:
                        operands_agree = False
            if (
                named
                and unit_agrees
                and predicate_agrees
                and path_agrees
                and address_agrees
                and operands_agree
            ):
                matched += 1
                continue
            if not named:
                key = (want, str(got))
            elif not unit_agrees:
                key = (f"{want}.{want_unit}", f"{got}.{got_unit}")
            elif not predicate_agrees:
                key = (f"[{want_predicate}]{want}", f"[{got_predicate}]{got}")
            elif not path_agrees:
                key = (f"{want}.{want_path}", f"{got}.{got_path}")
            elif not address_agrees:
                key = (f"{want} {want_address}", f"{got} {got_address}")
            else:
                key = (f"{want} {want_operands}", f"{got} {got_operands}")
            disagreements[key] += 1
            examples.setdefault(key, f"{image.name} {address:#010x} {word:#010x}")
        print(
            f"{image.name}: {len(instructions)} instructions ({compact} compact), "
            f"{matched} matched"
        )
    print(
        f"total {total}, disagreeing {sum(disagreements.values())}, "
        f"operand lists compared {operands_compared}"
    )
    for (want, got), count in disagreements.most_common(30):
        print(f"  dis6x {want:12s} decoder {got:12s} x{count:<5d} {examples[(want, got)]}")
    return 1 if disagreements else 0


if __name__ == "__main__":
    raise SystemExit(main())
