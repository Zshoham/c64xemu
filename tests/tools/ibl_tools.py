"""TI's own C6472 boot EEPROM tools, built from the IBL source and used as an oracle.

The Intermediate Boot Loader (git.ti.com/keystone-rtos/ibl, provisioned into
tests/build/ibl by tools/fetch_deps.sh) is how TI built the C6472 EVM's boot EEPROM. Its
makefiles turn an image into an EEPROM in four host-tool steps, and this module runs exactly
those, so an EEPROM produced here has no byte from this project's own builder in it:

1. ``hex6x -a -boot`` - the boot table as ASCII hex, with ``-order L|M`` for the build's byte
   order, memwidth and romwidth 32, as SPRUEC6G section 6.2.2.3.1 sets it out;
2. ``bconvert64x -le|-be`` - TI's fix-up of sections whose length is not a multiple of 4;
3. ``b2i2c`` then ``b2ccs`` - the table cut into 128-byte I2C blocks, written as CCS data;
4. ``romparse`` built for the C6472 against the ROM's ``tiboot_c6472.h`` - the boot parameter
   tables and the image layout, from a map in the form of the IBL's ``ibl_c6472/i2crom.map.pre``.

The host tools are 2000s C: they are built with ``-std=gnu89`` and implicit declarations
allowed, and ``romparse`` needs ``bison`` and ``flex``.
"""
from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parent.parent.parent
IBL = ROOT / "tests/build/ibl"
TOOLS = ROOT / "tests/build/ibl-tools"
HEX6X = ROOT / "tests/build/ti-cgt-c6000_8.3.15/bin/hex6x"
LEGACY_C = ["-std=gnu89", "-w", "-include", "stdlib.h", "-include", "string.h"]


class IblToolError(Exception):
    pass


def _run(command: list[str], cwd: Path) -> None:
    done = subprocess.run(command, cwd=cwd, capture_output=True, text=True, timeout=120)
    if done.returncode != 0:
        raise IblToolError(f"{' '.join(command)} failed:\n{done.stdout}{done.stderr}")


def build_tools() -> Path:
    """Build b2i2c, b2ccs, bconvert64x and the C6472 romparse into tests/build/ibl-tools."""
    names = ("b2i2c", "b2ccs", "bconvert64x", "romparse")
    if all((TOOLS / name).is_file() for name in names):
        return TOOLS
    if not IBL.is_dir():
        raise IblToolError(f"{IBL} is missing: run tests/tools/fetch_deps.sh")
    compiler = os.environ.get("CC") or shutil.which("cc") or shutil.which("gcc")
    for tool in ("bison", "flex"):
        if shutil.which(tool) is None:
            raise IblToolError(f"{tool} is required to build TI's romparse")
    if compiler is None:
        raise IblToolError("a host C compiler is required to build TI's IBL tools")
    TOOLS.mkdir(parents=True, exist_ok=True)
    util = IBL / "src/util"
    for name, source in (("b2i2c", "btoccs/b2i2c.c"), ("b2ccs", "btoccs/b2ccs.c"),
                         ("bconvert64x", "bconvert/bconvert64x.c")):
        _run([compiler, *LEGACY_C, "-o", str(TOOLS / name), str(util / source)], TOOLS)
    romparse = util / "romparse"
    _run(["bison", "-d", "-o", str(TOOLS / "rparse.tab.c"), str(romparse / "rparse.y")], TOOLS)
    # romparse.c defines yywrap as `void`, so a lexer that called it would read garbage at end of
    # file and wait for more input forever; TI's own builds evidently linked a lexer that never
    # asked. Generating the lexer with --noyywrap is that lexer.
    _run(["flex", "--noyywrap", "-o", str(TOOLS / "lex.yy.c"), str(romparse / "rparse.flex")],
         TOOLS)
    _run([
        compiler, *LEGACY_C, "-Dc6472", "-DIBL_CFG_I2C_MAP_TABLE_DATA_BUS_ADDR=0x50",
        f"-I{TOOLS}", f"-I{romparse}", f"-I{IBL / 'src'}", f"-I{IBL / 'src/device/c6472'}",
        "-o", str(TOOLS / "romparse"), str(romparse / "romparse.c"),
        str(TOOLS / "rparse.tab.c"), str(TOOLS / "lex.yy.c"),
    ], TOOLS)
    return TOOLS


@dataclass(frozen=True)
class Section:
    """One ``romparse`` boot parameter table, in the map's own field names. The defaults are
    Table 25's for the fields ``tests/tools/i2c_eeprom.py`` writes, and zero for the rest."""
    param_index: int
    options: int
    dev_addr_ext: int
    core_freq_mhz: int = 50
    i2c_clk_freq_khz: int = 10
    sw_pll: int = 0
    my_i2c_id: int = 0


def boot_table(image: Path, entry: int, big_endian: bool, work: Path) -> Path:
    """Steps 1 and 2: the image's boot table as TI's IBL build makes it, a ``.b`` file. The
    command file is the IBL's ``ibl_c6472/ibl_init.rmd`` with this image's entry point: its
    ROMS directive is what puts the ``$A`` address line the later tools skip."""
    raw = work / "app_raw.b"
    (work / "app.rmd").write_text(
        f"-a\n-boot\n-e 0x{entry:08x}\n\nROMS\n{{\n"
        "\tROM1:  org = 0x0400, length = 0x20000, memwidth = 32, romwidth = 32\n"
        f"\tfiles = {{ {raw.name} }}\n}}\n"
    )
    _run([str(HEX6X), "-q", "-order", "M" if big_endian else "L", "app.rmd", str(image)], work)
    converted = work / "app.b"
    _run([str(build_tools() / "bconvert64x"), "-be" if big_endian else "-le",
          str(raw), str(converted)], work)
    return converted


def eeprom(image: Path, entry: int, big_endian: bool, section: Section, work: Path) -> bytes:
    """Steps 1 to 4: the whole EEPROM, as bytes from address 0 of the device the section's
    ``dev_addr_ext`` names. ``romparse -compact`` puts the boot table in the 128-byte slot after
    the selected parameter table, as the IBL's EVM build does with ``COMPACT_I2C=yes``."""
    tools = build_tools()
    table = boot_table(image, entry, big_endian, work)
    blocked = work / "app.i2c.b"
    _run([str(tools / "b2i2c"), str(table), str(blocked)], work)
    ccs = work / "app.i2c.ccs"
    _run([str(tools / "b2ccs"), str(blocked), str(ccs)], work)
    (work / "i2crom.map").write_text(
        "section\n{\n"
        f"  param_index      = {section.param_index}\n"
        "  boot_mode        = 257\n"
        f"  sw_pll           = {section.sw_pll}\n"
        f"  options          = {section.options}\n"
        f"  core_freq_mhz    = {section.core_freq_mhz}\n"
        f"  i2c_clk_freq_khz = {section.i2c_clk_freq_khz}\n"
        f"  dev_addr_ext     = 0x{section.dev_addr_ext:x}\n"
        "  multi_i2c_id     = 0\n"
        f"  my_i2c_id        = {section.my_i2c_id}\n"
        "  address_delay    = 0\n"
        f"  exe_file         = \"{ccs.name}\"\n"
        "}\n"
    )
    _run([str(tools / "romparse"), "-compact", "-rom_base", f"0x{section.dev_addr_ext:x}",
          "i2crom.map"], work)
    lines = (work / "i2crom.ccs").read_text().split()
    words = [int(word, 16) for word in lines[5:]]
    return b"".join(word.to_bytes(4, "big") for word in words)
