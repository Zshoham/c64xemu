#!/usr/bin/env python3
"""Build the serial EEPROM a C6472 boots from over I2C (SPRUEC6G section 4.3.3).

The image is what a board's EEPROM would hold, laid out from the manual and from TI's own C6472
EEPROM tools (the IBL's ``romparse`` and ``b2i2c``) rather than from the emulator: 128-byte blocks
with Table 5's 16-bit size and ones-complement checksum. Parameter blocks carry Table 23's
ten-byte common header and Table 25's I2C or Table 26's UTOPIA fields, all 16-bit, most
significant byte first; boot-table and configuration-table blocks are ``b2i2c``'s, four header
bytes and up to 124 bytes of data. The parameter block the pins select sits at 0x80 * index.

Without ``--utopia`` the block names a boot table (options 1) built from the ELF's initialized
load segments, every word most significant byte first, at the next 128-byte boundary. With ``--utopia`` the block names
another parameter block there (options 0), and that one is extended boot mode 0x104: the device
goes on to a UTOPIA boot configured by its fields, and the program arrives as cells instead
(``utopia_boot.py``). ``--config ADDR,SET,CLEAR`` puts a boot configuration table (options 2,
section 6.2.3) in front of the next parameter block, which is how an EEPROM configures what a
later boot skips.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import struct
import sys

if __package__:
    from .emac_boot import BootError, Section, assemble_table, parse_elf
else:
    from emac_boot import BootError, Section, assemble_table, parse_elf

BLOCK_BYTES = 128
BLOCK_HEADER = 4
# Table 24's extended boot modes, and Table 25's options.
EXTENDED_I2C_MASTER = 0x101
EXTENDED_UTOPIA = 0x104
OPTION_PARAMETERS = 0
OPTION_BOOT_TABLE = 1
OPTION_CONFIGURATION = 2
# Table 26's options bits.
UTOPIA_SINGLE_PHY = 1 << 0
UTOPIA_WIDE = 1 << 1
UTOPIA_SKIP_PORT_INIT = 1 << 2
# What core 0's ROM keeps for itself during a boot (SPRUEC6G Table 20).
ROM_SCRATCH = (0x008969C0, 0x00898000)
C6472_L2_END = 0x00898000


def checksum(body: bytes) -> int:
    """The checksum value that makes the block's ones-complement sum come out to -0."""
    total = 0
    for at in range(0, len(body), 2):
        total += int.from_bytes(body[at:at + 2].ljust(2, b"\0"), "big")
        total = (total & 0xFFFF) + (total >> 16)
    return (~total) & 0xFFFF


def block(payload: bytes) -> bytes:
    size = BLOCK_HEADER + len(payload)
    assert size <= BLOCK_BYTES
    header = size.to_bytes(2, "big") + bytes(2)
    return size.to_bytes(2, "big") + checksum(header + payload).to_bytes(2, "big") + payload


def blocks(data: bytes) -> bytes:
    """Table data cut into blocks as TI's ``b2i2c`` cuts them: 124 bytes behind each header."""
    step = BLOCK_BYTES - BLOCK_HEADER
    return b"".join(block(data[at:at + step]) for at in range(0, len(data), step))


def parameter_block(boot_mode: int, length: int, fields: dict[int, int]) -> bytes:
    """A boot parameter table in its 128-byte slot, as TI's ``romparse`` writes one: Table 23's
    header with the size of the structure (30 bytes for Table 25, 20 for Table 26), a zero
    checksum - which disables the check - and `fields` at their offsets, zeros after."""
    table = bytearray(BLOCK_BYTES)
    for offset, value in {0: length, 4: boot_mode, **fields}.items():
        table[offset:offset + 2] = value.to_bytes(2, "big")
    return bytes(table)


def i2c_parameters(options: int, address: int, slave: int, following: int = 0) -> bytes:
    """Table 25: options, device address low and high, Table 25's default core and bus
    frequencies, and the next device address a configuration table re-enters at."""
    return parameter_block(EXTENDED_I2C_MASTER, 30, {
        10: options, 12: address, 14: slave, 20: 50, 22: 10,
        24: following, 26: slave if following else 0,
    })


def utopia_parameters(options: int, cell: int, width: int, slave: int, mhz: int) -> bytes:
    """Table 26: options, physical cell size, bus width, slave ID and core frequency."""
    return parameter_block(EXTENDED_UTOPIA, 20,
                           {10: options, 12: cell, 14: width, 16: slave, 18: mhz})


def wire_order(table: bytes, big_endian: bool) -> bytes:
    """A table in the device's byte order as the EEPROM carries it: every 32-bit word most
    significant byte first, which is what TI's flow - ``hex6x -a`` then ``bconvert64x`` -
    writes for either build."""
    if big_endian:
        return table
    return b"".join(table[at:at + 4][::-1] for at in range(0, len(table), 4))


def boot_table(path: str) -> tuple[bytes, int]:
    """The ELF's initialized load segments as a boot table, words most significant byte first
    (section 6.2.2). Like ``hex6x -boot`` it carries only what has contents: the program clears
    its own .bss. An I2C-only boot leaves L1D alone, so only the ROM's L2 scratch is refused."""
    elf = parse_elf(path)
    image = elf.image(0)
    sections = []
    for segment in image.segments:
        start, end = segment.address, segment.address + len(segment.data)
        low, high = ROM_SCRATCH
        if start < high and end > low:
            raise BootError(f"segment at 0x{start:08x} overlaps core 0's ROM boot scratch")
        if C6472_L2_END < end <= 0x00A00000:
            raise BootError(f"segment at 0x{start:08x} runs past the C6472's local L2")
        sections.append(Section(segment.address, segment.data))
    big_endian = elf.endian == ">"
    return wire_order(assemble_table(image.entry, sections, big_endian), big_endian), image.entry


def build(
    *,
    index: int,
    slave: int,
    table: bytes | None,
    utopia: tuple[int, int, int, int, int] | None,
    config: list[tuple[int, int, int]],
    eeprom_type: int = 0,
) -> bytes:
    """The EEPROM contents: zeros up to the selected parameter block, that block, then what it
    names. `table` is the boot table for an I2C-only boot; `utopia` is Table 26's fields for a
    boot that hands over to UTOPIA. `eeprom_type` goes in every I2C block's options bits 4:2.
    The layout is ``romparse -compact``'s: the data follows the selected block's 128-byte slot."""
    assert (table is None) != (utopia is None)
    kind = eeprom_type << 2
    base = index * BLOCK_BYTES
    after = base + BLOCK_BYTES
    tail = b""
    if config:
        words = [value for entry in config for value in entry] + [0, 0, 0]
        tail = blocks(b"".join(struct.pack(">I", value) for value in words))
        head = i2c_parameters(kind | OPTION_CONFIGURATION, after, slave, after + len(tail))
    elif utopia is not None:
        head = i2c_parameters(kind | OPTION_PARAMETERS, after, slave)
    else:
        head = i2c_parameters(kind | OPTION_BOOT_TABLE, after, slave)
    if utopia is not None:
        tail += utopia_parameters(*utopia)
    else:
        if config:
            # The parameter block the configuration table re-enters at names the table.
            tail += i2c_parameters(kind | OPTION_BOOT_TABLE, after + len(tail) + BLOCK_BYTES,
                                   slave)
        tail += blocks(table)
    image = bytes(base) + head + tail
    if len(image) > 0x10000:
        raise BootError(f"{len(image)} bytes do not fit a two-byte EEPROM address")
    return image


def parse_entry(text: str) -> tuple[int, int, int]:
    try:
        address, set_mask, clear_mask = (int(part, 0) for part in text.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError("--config must be ADDR,SET,CLEAR") from error
    if set_mask == 0 and clear_mask == 0:
        raise argparse.ArgumentTypeError("an entry with no masks is a function call")
    return address, set_mask, clear_mask


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, help="the C6000 ELF the program comes from")
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--index", type=int, choices=range(16), default=0,
                        help="the parameter block CFG[3:0] selects")
    parser.add_argument("--slave", type=lambda text: int(text, 0), choices=(0x50, 0x51),
                        default=0x50, help="the EEPROM's address: 0x50 for mode 2, 0x51 for 3")
    parser.add_argument("--config", type=parse_entry, action="append", default=[],
                        metavar="ADDR,SET,CLEAR")
    parser.add_argument("--utopia", action="store_true",
                        help="hand over to UTOPIA boot with the fields below")
    parser.add_argument("--single-phy", action="store_true")
    parser.add_argument("--width", type=int, choices=(8, 16), default=8)
    parser.add_argument("--skip-port-init", action="store_true")
    parser.add_argument("--cell-size", type=int)
    parser.add_argument("--slave-id", type=int, choices=range(32), default=0)
    parser.add_argument("--core-mhz", type=int, default=500)
    parser.add_argument("--eeprom-type", type=int, choices=range(8), default=0,
                        help="the I2C options' EEPROM type, bits 4:2 (tiboot_c6472.h)")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    try:
        if args.utopia:
            options = ((UTOPIA_SINGLE_PHY if args.single_phy else 0)
                       | (UTOPIA_WIDE if args.width == 16 else 0)
                       | (UTOPIA_SKIP_PORT_INIT if args.skip_port_init else 0))
            cell = args.cell_size or (54 if args.width == 16 else 53)
            utopia = (options, cell, args.width, args.slave_id, args.core_mhz)
            table, entry = None, None
        else:
            utopia = None
            table, entry = boot_table(args.image)
        image = build(index=args.index, slave=args.slave, table=table, utopia=utopia,
                      config=args.config, eeprom_type=args.eeprom_type)
        args.out.write_bytes(image)
        report = {"bytes": len(image), "table_bytes": len(table) if table else 0, "entry": entry}
        print(json.dumps(report) if args.json else
              f"wrote {len(image)} bytes to {args.out}")
        return 0
    except (BootError, OSError) as error:
        print(f"i2c_eeprom: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
