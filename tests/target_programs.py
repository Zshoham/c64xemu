from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from common import (
    DEFAULT_OPT_LEVELS,
    CL6X,
    HEX6X,
    ROOT,
    RTS_INCLUDE_DIR,
    RTS_LIB_DIR,
    run_tool,
)

PROGRAMS = ROOT / "tests/programs"

# Integration cases below build target firmware and execute it through `c64xemu run`.

@dataclass(frozen=True)
class Program:
    name: str
    weight: int
    sources: tuple[str, ...]
    includes: tuple[str, ...]
    cflags: tuple[str, ...]
    link_flags: tuple[str, ...]
    entry: str
    max_cycles: int
    opt_levels: tuple[str, ...]
    rts: bool
    expect: dict[str, int]
    cio_output: str | None
    rx_frames: tuple[str, ...]
    tx_frames: tuple[str, ...] | None
    boot_table: int | None
    # A boot configuration table: where it sits, and its address/set-mask/clear-mask entries.
    boot_config: tuple[int, tuple[tuple[int, int, int], ...]] | None
    # Whether the program is rerun with the caches configured (`--cache`) and required to give
    # the same answer. A program whose expectation is tied to one cache configuration says no:
    # a coherence test that writes back L1D and nothing else is right on a device whose L2 is
    # all SRAM and wrong on one where L2 is a cache, and both are the hardware's behaviour.
    cache_rerun: bool = True
    # The linker command file, when the program needs a memory map other than the C6455's one
    # L2 window - a C6472 program has 608 KB of local L2 and the shared L2 to place.
    link_cmd: str = str(PROGRAMS / "common/link.cmd")
    # Whether the boot table reaches memory through the ROM's I2C EEPROM boot instead of being
    # placed in memory for it: the same table, read off a serial EEPROM on the I2C bus one
    # block at a time (SPRUEC6G section 2.3.4). Needs `boot_table` for the table itself.
    i2c_boot: bool = False

def load_boot_config(
    spec: dict[str, Any] | None,
) -> tuple[int, tuple[tuple[int, int, int], ...]] | None:
    if spec is None:
        return None
    entries = tuple(
        (int(target, 16), int(set_mask, 16), int(clear_mask, 16))
        for target, set_mask, clear_mask in spec["entries"]
    )
    return int(spec["address"], 16), entries

def load_programs() -> list[Program]:
    programs: list[Program] = []
    for spec_path in sorted(PROGRAMS.glob("*/spec.json")):
        spec = json.loads(spec_path.read_text(encoding="utf-8"))
        directory = spec_path.parent
        sources = []
        for source in spec["sources"]:
            candidate = directory / source
            sources.append(str(candidate if candidate.is_file() else ROOT / source))
        programs.append(
            Program(
                name=directory.name,
                weight=int(spec["weight"]),
                sources=tuple(sources),
                includes=tuple(str(ROOT / path) for path in spec.get("includes", ())),
                cflags=tuple(spec.get("cflags", ())),
                link_flags=tuple(spec.get("link_flags", ())),
                entry=spec.get("entry", "_start"),
                max_cycles=int(spec.get("max_cycles", 100000)),
                opt_levels=tuple(spec.get("opt_levels", DEFAULT_OPT_LEVELS)),
                rts=bool(spec.get("rts", False)),
                expect={key: int(value) for key, value in spec.get("expect", {}).items()},
                cio_output=spec.get("cio_output"),
                rx_frames=tuple(spec.get("rx_frames", ())),
                tx_frames=tuple(spec["tx_frames"]) if "tx_frames" in spec else None,
                boot_table=int(spec["boot_table"], 16) if "boot_table" in spec else None,
                boot_config=load_boot_config(spec.get("boot_config")),
                cache_rerun=bool(spec.get("cache_rerun", True)),
                link_cmd=str(directory / spec["link_cmd"])
                if "link_cmd" in spec
                else str(PROGRAMS / "common/link.cmd"),
                i2c_boot=bool(spec.get("i2c_boot", False)),
            )
        )
    return programs

def build_target_program(program: Program, workdir: Path) -> Path | None:
    """Compile and link a program with the checked-in TI toolchain."""
    objects: list[str] = []
    sources = list(program.sources)
    if not program.rts:
        sources.append(str(PROGRAMS / "common/crt.asm"))
    compile_command = [
        str(CL6X),
        "-mv6400+",
        "--abi=eabi",
        "-q",
        "--symdebug:none",
        *program.cflags,
    ]
    for include in program.includes:
        compile_command.append(f"-I{include}")
    if program.rts:
        compile_command.append(f"-I{RTS_INCLUDE_DIR}")
    compile_command += ["-c", *sources, f"-fr={workdir}", f"-fs={workdir}"]
    compiled = run_tool(compile_command, ROOT)
    if compiled is None or compiled.returncode != 0:
        return None
    for source in sources:
        objects.append(str(workdir / (Path(source).stem + ".obj")))
    image = workdir / f"{program.name}.out"
    link_command = [
        str(CL6X),
        "-mv6400+",
        "--abi=eabi",
        "-q",
        *program.link_flags,
        "-z",
        *objects,
        program.link_cmd,
        f"--entry_point={program.entry}",
        "-o",
        str(image),
    ]
    if program.rts:
        link_command += [
            f"-i{RTS_LIB_DIR}",
            "-llibc.a",
            "--heap_size=0x800",
            "--stack_size=0x1000",
        ]
    linked = run_tool(link_command, ROOT)
    if linked is None or linked.returncode != 0 or not image.is_file():
        return None
    if program.boot_table is not None and not build_boot_table(program, image, workdir):
        return None
    if program.boot_config is not None:
        write_boot_config(program, workdir)
    return image

def boot_table_path(program: Program, workdir: Path) -> Path:
    return workdir / f"{program.name}.btbl.bin"

def boot_config_path(program: Program, workdir: Path) -> Path:
    return workdir / f"{program.name}.bcfg.bin"

def write_boot_config(program: Program, workdir: Path) -> None:
    """Lay out the documented boot configuration table: three words per entry, all-zero end.

    SPRUEC6G section 6.2.3 fixes the layout - address, set mask, clear mask - and the table
    ends when all three words are zero.
    """
    _, entries = program.boot_config
    order = "big" if "--big_endian" in program.link_flags else "little"
    payload = bytearray()
    for entry in entries:
        for word in entry:
            payload += word.to_bytes(4, order)
    payload += bytes(12)
    boot_config_path(program, workdir).write_bytes(bytes(payload))

def build_boot_table(program: Program, image: Path, workdir: Path) -> bool:
    """Turn the linked image into the ROM boot table the bootloader reads.

    The table is produced by the checked-in TI hex conversion utility, so the emulator's
    loader is checked against the toolchain's own boot-table format (SPRUEC6G section 6.2.2)
    rather than against a table this harness invented.
    """
    if not HEX6X.is_file():
        return False
    listing = workdir / f"{program.name}.btbl.txt"
    order = "M" if "--big_endian" in program.link_flags else "L"
    converted = run_tool(
        [
            str(HEX6X),
            "-q",
            "-boot",
            f"-bootorg=0x{program.boot_table:08X}",
            "-memwidth",
            "32",
            "-romwidth",
            "32",
            "-order",
            order,
            "-e",
            program.entry,
            "--ti_txt",
            "-o",
            str(listing),
            str(image),
        ],
        ROOT,
    )
    if converted is None or converted.returncode != 0 or not listing.is_file():
        return False
    # TI-TXT is an "@address" line followed by space-separated hex bytes, ending with "q".
    payload = bytearray()
    start: int | None = None
    for line in listing.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line in ("q", "Q"):
            continue
        if line.startswith("@"):
            address = int(line[1:], 16)
            if start is None:
                start = address
            elif address != start + len(payload):
                return False
            continue
        payload += bytes(int(byte, 16) for byte in line.split())
    if start != program.boot_table or not payload:
        return False
    boot_table_path(program, workdir).write_bytes(bytes(payload))
    return True

# The EEPROM block format the ROM's I2C boot reads, from SPRUEC6G section 2.3.4.1 and Table 5:
# "Each block has a maximum length of 128 bytes, including the 4 byte block header", the header
# being a 16-bit block size that counts itself and a 16-bit ones-complement checksum "including
# the block size and checksum fields", for which "Valid checksum values are 0 and -0".
EEPROM_BLOCK_BYTES = 128
EEPROM_BLOCK_HEADER = 4
# The boot parameter structure of Table 6: 2-byte big-endian fields at these offsets into the
# block, "regardless of the endianness setting of the processor".
EEPROM_PARAM_FIELDS = {
    4: 5,  # Boot mode: 0101b is master I2C boot (Table 3)
    12: 0,  # Broadcast address, unused
    14: 0x04,  # The address the DSP would answer to as a slave
    16: 720,  # CPU frequency in MHz, which the ROM reprograms the bus divider from
    18: 100,  # Desired I2C bus frequency in kHz
    20: 0,  # Next device address, only used for a boot configuration table
    22: 0,
    24: 0,  # Address delay
}
# Options bits 1:0 = 1: "A value of 1 indicates a boot table" (section 2.3.4.2).
EEPROM_OPTION_BOOT_TABLE = 1

def eeprom_checksum(body: bytes) -> int:
    """The checksum value that makes the block's ones-complement sum come out to -0."""
    total = 0
    for offset in range(0, len(body), 2):
        total += int.from_bytes(body[offset : offset + 2].ljust(2, b"\0"), "big")
        while total > 0xFFFF:
            total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF

def eeprom_block(payload: bytes) -> bytes:
    """One block: the size, the checksum computed over the whole block, then the payload."""
    size = EEPROM_BLOCK_HEADER + len(payload)
    assert size <= EEPROM_BLOCK_BYTES
    header = size.to_bytes(2, "big") + bytes(2)
    return size.to_bytes(2, "big") + eeprom_checksum(header + payload).to_bytes(
        2, "big"
    ) + payload

def build_eeprom_image(table: bytes, big_endian: bool) -> bytes:
    """A serial EEPROM's contents: the boot parameter block the ROM reads first, then the boot
    table split across as many blocks as it needs.

    The parameter block sits at address 0, which is where the ROM looks when the configuration
    pins select index 0, and it names the boot table's address as the block right after it.
    `table` is in the build's byte order, as the TI-TXT output holds it; on the EEPROM every
    32-bit word goes most significant byte first, as SPRUEC6G section 6.2.2.3.1's ``hex6x -a``
    writes it and TI's IBL ``bconvert64x`` states ("The C6x boot loader expects all 32 bit
    values to be in big endian").
    """
    if not big_endian:
        table = b"".join(table[at : at + 4][::-1] for at in range(0, len(table), 4))
    payload = bytearray(EEPROM_BLOCK_BYTES - EEPROM_BLOCK_HEADER)
    fields = dict(EEPROM_PARAM_FIELDS)
    fields[6] = EEPROM_OPTION_BOOT_TABLE
    fields[8] = EEPROM_BLOCK_BYTES  # Device address (LSW): where the boot table starts
    fields[10] = 0x50  # Device address (MSW), "used as the I2C EEPROM slave address"
    for offset, value in fields.items():
        at = offset - EEPROM_BLOCK_HEADER
        payload[at : at + 2] = value.to_bytes(2, "big")
    image = bytearray(eeprom_block(bytes(payload)))
    assert len(image) == EEPROM_BLOCK_BYTES
    step = EEPROM_BLOCK_BYTES - EEPROM_BLOCK_HEADER
    for offset in range(0, len(table), step):
        image += eeprom_block(table[offset : offset + step])
    return bytes(image)

def eeprom_path(program: Program, workdir: Path) -> Path:
    return workdir / f"{program.name}.eeprom.bin"

def program_expected_result(program: Program) -> int | None:
    """The expected A4 value at exit, when the program's exit path defines one.

    Expected values are frozen constants derived independently of the emulator
    (see the `expected_source` field of each spec), so a mismatch is always an
    emulator defect rather than a moving target. Programs that terminate through
    the TI RTS have no defined final A4: `exit()` ends in `abort()`, which takes
    no argument (compiler/lib/src/exit.c), so those specs check the CIO text
    instead and omit `expect.A4`.
    """
    if "A4" not in program.expect:
        if program.cio_output is None:
            raise RuntimeError(f"program {program.name} checks neither A4 nor CIO output")
        return None
    return program.expect["A4"] & 0xFFFFFFFF

def write_pcap(path: Path, frames: tuple[str, ...]) -> None:
    """A little-endian pcap of Ethernet frames, which is what the emulator's --pcap-in reads."""
    payload = bytearray()
    payload += (0xA1B2C3D4).to_bytes(4, "little")
    payload += (2).to_bytes(2, "little") + (4).to_bytes(2, "little")
    payload += (0).to_bytes(4, "little") + (0).to_bytes(4, "little")
    payload += (65535).to_bytes(4, "little") + (1).to_bytes(4, "little")
    for frame in frames:
        raw = bytes.fromhex(frame)
        payload += (0).to_bytes(4, "little") + (0).to_bytes(4, "little")
        payload += len(raw).to_bytes(4, "little") + len(raw).to_bytes(4, "little")
        payload += raw
    path.write_bytes(bytes(payload))

# Cold caches turn every first access into a miss, so the same program needs more cycles to
# reach the same answer with L1P, L1D and L2 configured as caches. The cached rerun scales the
# program's budget instead of carrying a second per-program number, because what it asserts is
# the answer rather than the time; the timing cases measure cycles against the manuals. The
# widest inflation across the suite is 1.9x - on the shortest programs, where the cold cache is
# most of the run - and the median is 1.05x, so four times the budget is headroom rather than a
# licence to run away.
CACHED_CYCLE_FACTOR = 4

def run_target_program(
    candidate: Path,
    program: Program,
    image: Path,
    workdir: Path,
    cache: bool = False,
) -> dict[str, Any] | None:
    suffix = "-cached" if cache else ""
    state_path = workdir / f"{program.name}-state{suffix}.json"
    budget = program.max_cycles * (CACHED_CYCLE_FACTOR if cache else 1)
    command = [
        str(candidate),
        "run",
        str(image),
        "--max-cycles",
        str(budget),
        "--state-out",
        str(state_path),
    ]
    if cache:
        command.append("--cache")
    if program.i2c_boot:
        # The same boot table, delivered the way the ROM's I2C boot reads it: a serial EEPROM
        # on the bus with the boot parameter block first and the table blocked behind it. The
        # ELF is only there for its symbols; every byte of memory comes off the bus.
        table = boot_table_path(program, workdir).read_bytes()
        eeprom = eeprom_path(program, workdir)
        eeprom.write_bytes(build_eeprom_image(table, "--big_endian" in program.link_flags))
        command += [
            "--boot-mode",
            "5",
            "--i2c-eeprom",
            str(eeprom),
        ]
    elif program.boot_table is not None:
        # The ELF is only there for its symbols: every byte of memory comes from the table.
        table = boot_table_path(program, workdir)
        command += ["--boot-table", f"{table}:0x{program.boot_table:08X}"]
    if program.boot_config is not None:
        address, _ = program.boot_config
        config = boot_config_path(program, workdir)
        command += ["--boot-config", f"{config}:0x{address:08X}"]
    if program.rx_frames:
        pcap_path = workdir / f"{program.name}-rx.pcap"
        write_pcap(pcap_path, program.rx_frames)
        command += ["--pcap-in", str(pcap_path)]
    executed = run_tool(command, ROOT, timeout=300)
    if executed is None or not state_path.is_file():
        return None
    try:
        return json.loads(state_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None

def split_weight(total: int, parts: int) -> list[int]:
    base = total // parts
    weights = [base] * parts
    weights[-1] = total - base * (parts - 1)
    return weights

# The programs whose images the decoder differential runs over: the ones whose generated code is
# encoding-rich, which is where the compact saturating-subtract defect was hiding. Every DSPLIB
# program qualifies by name.
DIFFERENTIAL_PROGRAMS = frozenset(
    {
        "packed_simd",
        "complex_simd",
        "string_ops",
        "float_math",
        "saturation_flags",
        "bit_twiddle",
        "memory_intrinsics",
        "int64_math",
        "long_arith",
        "memcpy_stride",
    }
)
