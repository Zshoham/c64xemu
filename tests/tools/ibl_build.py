"""TI's Intermediate Boot Loader for the C6472 EVM, built the way ``make evm_c6472`` builds it.

The IBL (git.ti.com/keystone-rtos/ibl, provisioned into tests/build/ibl by tools/fetch_deps.sh)
is a two-stage loader: the ROM's I2C boot loads the first stage, which drives the I2C module
itself to read the IBL's configuration table and the second stage off the same EEPROM. This
module runs the recipe of ``src/make/Makefile``'s ``evm_c6472`` target and ``makestg1`` /
``makestg2`` step by step, with the suite's code generation tools in place of the COFF tools
TI built it with:

1. every object the stage templates (``ibl_c6472/ibl_init_objs_template.inc`` and
   ``ibl_objs_template.inc``) name, compiled with ``make/c64x/makedefs.mk``'s flags and each
   module makefile's include path, for each byte order;
2. the first stage linked with ``ibl_init.cmd``, its symbols the second stage calls extracted
   from the map by TI's ``symExtract``, and the second stage linked with ``ibl.cmd``;
3. each stage turned into I2C blocks by ``hex6x``, ``bconvert64x``, ``b2i2c`` and ``b2ccs``;
4. ``i2crom.map.pre`` preprocessed and laid out by ``romparse -compact``, ``ccs2bin -swap``
   to a binary, and ``iblConfig`` writing the C6472 EVM's configuration table into it.

Two things change with the toolset, both because the 8.5 tools only build EABI ELF:

- EABI names sections COFF did not have (``.fardata``, ``.neardata``, ``.rodata``,
  ``.init_array``) and TI's command files place none of them. ``EABI_SECTIONS`` puts each in the
  region its COFF counterpart goes to, so the second stage still never lands on the first.
- EABI's start-up zero-fills uninitialised data, which COFF's did not. The second stage's
  ``_c_int00`` would then wipe the configuration and status tables the first stage filled in
  for it, so both stages link with ``--zero_init=off``, which is COFF's behaviour.
"""
from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import shutil
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ibl_tools import IBL, LEGACY_C, IblToolError, _run, build_tools  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent.parent
CGT = ROOT / "tests/build/ti-cgt-c6000_8.3.15"
OUT = ROOT / "tests/build/ibl-c6472"
SRC = IBL / "src"

# The evm_c6472 target's CEXCLUDES, and the five makestg1 adds for every c64x device.
EXCLUDES = ("ELF", "COFF", "BIS", "MULTI_BOOT", "SPI", "NOR_SPI", "NOR_EMIF", "NAND_SPI",
            "NAND_EMIF")
I2C_BUS_ADDR = 0x50
I2C_MAP_ADDR = 0x500
# The top-level Makefile's SPI defaults, which reach the main and device modules as SPI_CFG.
SPI_DEFS = ("SPI_MODE=1", "SPI_ADDR_WIDTH=24", "SPI_NPIN=5", "SPI_CSEL=2", "SPI_C2TDEL=4",
            "SPI_CLKDIV=8", "SPI_ROM=1")
MAINDEFS = [f"-DIBL_CFG_I2C_MAP_TABLE_DATA_BUS_ADDR=0x{I2C_BUS_ADDR:x}",
            *(f"-DEXCLUDE_{name}" for name in EXCLUDES)]
# makedefs.mk's non-debug CFLAGS and DCFLAGS.
CFLAGS = ["-k", "-mi200", "-ml0", "-ms3", "-o2", "-pm", "-pds1111", "-pds827", "-pds824",
          "-pds837", "-pds1037", "-pds195", "-pds97", "-pden", "-pdr", "-mv6400+", "-as", "-ss"]
COMMON_SYMBOLS = ("iblBootBtbl", "iblMalloc", "iblFree", "iblMemset", "iblMemcpy")


@dataclass(frozen=True)
class Module:
    """One IBL module makefile: where its sources are found (``vpath``), its ``C6X_C_DIR``, and
    whether the stage-2 make passes it ``CDEFS`` (only ``CFG_MODULES`` get them)."""
    sources: tuple[str, ...]
    includes: tuple[str, ...]
    configured: bool = False


# Each module's makefile, with $(TARGET)=c6472 and $(ARCH)=c64x substituted.
MODULES = {
    "main": Module(
        ("main",),
        ("main", ".", "interp/bis", "interp/coff", "interp/btbl", "interp/blob", "interp/elf",
         "arch/c64x", "device", "device/c6472", "ethboot", "nandboot", "norboot",
         "driver/timer", "hw/i2c", "hw/uart", "hw/spi", "cfg/c6472"),
        configured=True,
    ),
    "device": Module(
        ("device/c6472", "device/c64x"),
        ("device/c6472", "cfg/c6472", "device", ".", "arch/c64x", "hw/plls", "hw/pscs",
         "hw/gpio", "hw/ddrs/emif31", "hw/qm", "hw/cpdma", "hw/pa", "hw/serdes", "hw/nands",
         "hw/nors", "driver/eth", "hw/spi", "hw/uart"),
        configured=True,
    ),
    "hw": Module(
        ("hw/timer/timer64", "hw/macs/cpmac", "hw/plls/pllxx1p8", "hw/pscs/psc2",
         "hw/ddrs/emif31", "hw/mdio", "hw/gpio", "hw/nands/gpio", "hw/i2c",
         "hw/uart/null_uart"),
        (".", "arch/c64x", "hw", "cfg/c6472", "hw/timer", "hw/macs/cpmac", "hw/macs",
         "hw/macs/cpmacsl", "driver/eth", "device", "device/c6472", "hw/plls", "hw/pscs",
         "hw/mdio", "hw/gpio", "hw/nands", "hw/nands/gpio", "hw/nands/emif25", "hw/nands/spi",
         "hw/i2c", "hw/sgmii", "hw/cpsw", "hw/cpdma", "hw/pa", "hw/spi", "hw/nors",
         "hw/emif25", "hw/uart", "ecc"),
    ),
    "interp": Module(
        ("interp/bis", "interp/coff", "interp/btbl", "interp/blob", "interp/elf"),
        (".", "arch/c64x", "interp", "cfg/c6472", "hw/uart"),
    ),
    "driver": Module(
        ("driver/eth", "driver/stream", "driver/timer", "driver/nand", "driver/nor"),
        (".", "arch/c64x", "driver", "driver/timer", "driver/stream", "driver/nand",
         "driver/nor", "cfg/c6472", "hw/timer", "hw/nands", "device", "device/c6472"),
    ),
    "ethboot": Module(
        ("ethboot",),
        (".", "arch/c64x", "device", "device/c6472", "ethboot", "driver/eth", "hw/macs",
         "hw/mdio", "hw/sgmii", "hw/cpsw", "hw/qm", "hw/cpdma", "hw/pa"),
    ),
    "nandboot": Module(
        ("nandboot",),
        (".", "arch/c64x", "device", "device/c6472", "nandboot", "driver/nand"),
    ),
    "ecc": Module(("ecc/3byte",), (".", "arch/c64x", "ecc")),
}

# The EABI sections TI's COFF-era command files do not name, in the region the COFF section
# they replace goes to. Each stage's own SECTIONS block is unchanged.
EABI_SECTIONS = {
    "ibl_init.cmd": ("TEXT_INIT", "DATA_INIT"),
    "ibl.cmd": ("TEXT", "DATA"),
}


@dataclass(frozen=True)
class Build:
    """What one ``make evm_c6472 ENDIAN=...`` produces: the EEPROM image with the configuration
    table written in, and the two linked stages, whose symbols name what is running."""
    eeprom: Path
    init: Path
    main: Path


def _cl6x(*arguments: str | Path, cwd: Path) -> None:
    _run([str(CGT / "bin/cl6x"), *(str(argument) for argument in arguments)], cwd)


def _preprocess(source: Path, defines: list[str], work: Path, includes: tuple[Path, ...] = ()
                ) -> Path:
    """``$(CC) -ppo``: the C preprocessor over a template, the output next to the input as
    ``.pp``."""
    _cl6x("-ppo", *defines, *(f"-I{path}" for path in includes), source, cwd=work)
    return source.with_suffix(".pp")


def _locate(module: Module, stem: str) -> Path:
    for directory in module.sources:
        candidate = SRC / directory / f"{stem}.c"
        if candidate.is_file():
            return candidate
    raise IblToolError(f"no source for {stem}.c in {module.sources}")


def _compile(work: Path, object_path: str, endian: str) -> None:
    """One ``../<module>/c64x/make/<stem>.<le|be>.oc`` entry of a stage template, compiled as
    makeeco.mk's GGCCOMPILE compiles it."""
    target = (work / "make" / object_path).resolve()
    if target.is_file():
        return
    module_name = Path(object_path).parts[1]
    module = MODULES[module_name]
    stem = target.name.split(".")[0]
    source = _locate(module, stem)
    target.parent.mkdir(parents=True, exist_ok=True)
    flags = [*CFLAGS, *(["-me"] if endian == "be" else [])]
    defines = [*MAINDEFS, *(f"-D{define}" for define in SPI_DEFS)] if module.configured else []
    includes = [f"-I{SRC / directory}" for directory in module.includes]
    _cl6x("-c", *flags, *defines, f"-dTNFNAME={source.name}", *includes,
          f"-I{CGT / 'include'}", f"-I{CGT / 'lib'}", f"-fr={target.parent}",
          f"-fs={target.parent}", f"--output_file={target}", source, cwd=target.parent)


def _objects(work: Path, template: str, endian: str) -> list[str]:
    """The template with ENDIAN_TAG replaced and run through the preprocessor with MAINDEFS,
    as makestg2's iblInit and iblMain do; the include file the command file pulls in."""
    stage = work / "make/ibl_c6472"
    name = template.replace("_template.inc", "")
    tmp = stage / f"{name}.tmp"
    tmp.write_text((SRC / "make/ibl_c6472" / template).read_text().replace("ENDIAN_TAG", endian))
    pp = _preprocess(tmp, [*MAINDEFS, f"-DENDIAN_TAG={endian}"], stage)
    shutil.copyfile(pp, stage / f"{name}.inc")
    return [line.strip() for line in pp.read_text().splitlines()
            if line.strip().endswith(".oc")]


def _eabi_sections(stage: Path, command_file: str) -> Path:
    text_region, data_region = EABI_SECTIONS[command_file]
    extra = stage / f"eabi_{command_file}"
    extra.write_text(
        "SECTIONS\n{\n"
        f"\t.rodata     > {text_region}\n"
        f"\t.init_array > {text_region}\n"
        f"\t.fardata    > {data_region}\n"
        f"\t.neardata   > {data_region}\n"
        "}\n"
    )
    return extra


def _link(work: Path, command_file: str, out: str, map_file: str, endian: str) -> Path:
    make = work / "make"
    library = CGT / "lib" / ("rts64pluse_elf.lib" if endian == "be" else "rts64plus_elf.lib")
    extra = _eabi_sections(make / "ibl_c6472", command_file)
    _cl6x("-z", "--zero_init=off", "--warn_sections", "-o", f"ibl_c6472/{out}",
          "-m", f"ibl_c6472/{map_file}", f"ibl_c6472/{command_file}", extra, library, cwd=make)
    return make / "ibl_c6472" / out


def _blocks(work: Path, rmd: str, image: Path, endian: str, ccs: str) -> Path:
    """hex6x with the stage's ROMS file, then bconvert64x, b2i2c and b2ccs, as makestg2 does."""
    make = work / "make"
    tools = build_tools()
    _run([str(CGT / "bin/hex6x"), "-q", "-order", "L" if endian == "le" else "M",
          f"ibl_c6472/{rmd}", str(image)], make)
    _run([str(tools / "bconvert64x"), f"-{endian}", "ibl_le.b", "ibl.b"], make)
    shutil.copyfile(make / "ibl.b", make / "ibl_c6472/ibl.b")
    _run([str(tools / "b2i2c"), "ibl_c6472/ibl.b", "ibl_c6472/ibl.i2c.b"], make)
    _run([str(tools / "b2ccs"), "ibl_c6472/ibl.i2c.b", f"ibl_c6472/{ccs}"], make)
    return make / "ibl_c6472" / ccs


def _stage(work: Path, endian: str) -> None:
    """makestg2's iblInit then iblMain for one byte order: each stage's objects compiled,
    linked and blocked; the first stage's map supplies the symbols the second links against."""
    stage = work / "make/ibl_c6472"
    for object_path in _objects(work, "ibl_init_objs_template.inc", endian):
        _compile(work, object_path, endian)
    init = _link(work, "ibl_init.cmd", "ibl_c6472_init.out", "ibl_c6472_init.map", endian)
    shutil.copyfile(init, stage / f"ibl_c6472_init.{endian}.out")
    _blocks(work, "ibl_init.rmd", init, endian, f"ibl_init.i2c.{endian}.ccs")
    # symExtract matches each symbol at the end of a map line; the 8.5 linker pads its symbol
    # columns with trailing blanks, which the COFF linker's maps did not have.
    plain_map = stage / "ibl_c6472_init.symbols.map"
    plain_map.write_text("".join(
        line.rstrip() + "\n" for line in (stage / "ibl_c6472_init.map").read_text().splitlines()))
    script = (SRC / "util/symExtract/symExtract").read_text().replace("\r", "")
    _run(["bash", "-c", script, "symExtract", str(plain_map),
          str(stage / "ibl_init_symbols.inc"), *COMMON_SYMBOLS], work / "make")
    for object_path in _objects(work, "ibl_objs_template.inc", endian):
        _compile(work, object_path, endian)
    main = _link(work, "ibl.cmd", "ibl_c6472.out", f"ibl_c6472.{endian}.map", endian)
    shutil.copyfile(main, stage / f"ibl_c6472.{endian}.out")
    _blocks(work, "ibl.rmd", main, endian, f"ibl.i2c.{endian}.ccs")


def _host_tool(name: str, sources: list[Path], defines: list[str], includes: list[Path]) -> Path:
    tools = build_tools()
    target = tools / name
    if not target.is_file():
        compiler = os.environ.get("CC") or shutil.which("cc") or shutil.which("gcc")
        if compiler is None:
            raise IblToolError("a host C compiler is required to build TI's IBL tools")
        _run([compiler, *LEGACY_C, *defines, *(f"-I{path}" for path in includes), "-o",
              str(target), *(str(source) for source in sources)], tools)
    return target


def build(endian: str) -> Build:
    """``make evm_c6472 ENDIAN=little`` (``endian`` "le") or ``ENDIAN=big`` ("be"), the C6472
    build TI's doc/build_instructions.txt gives, into tests/build/ibl-c6472/<endian>, once.
    Each is a single-endian EEPROM: one first stage, one second stage, one configuration
    table at I2C_MAP_ADDR."""
    if endian not in ("le", "be"):
        raise ValueError(f"byte order {endian!r} is le or be")
    work = OUT / endian
    stage_dir = work / "make/ibl_c6472"
    # The evm_c6472 target's own name for the image, copied to bin/ last, so its presence
    # means the whole build finished.
    built = Build(work / f"bin/i2crom_0x{I2C_BUS_ADDR:x}_c6472_{endian}.bin",
                  stage_dir / f"ibl_c6472_init.{endian}.out", stage_dir / f"ibl_c6472.{endian}.out")
    if built.eeprom.is_file():
        return built
    if not SRC.is_dir():
        raise IblToolError(f"{SRC} is missing: run tests/tools/fetch_deps.sh")
    if work.exists():
        shutil.rmtree(work)
    stage_dir.mkdir(parents=True)
    for item in (SRC / "make/ibl_c6472").iterdir():
        shutil.copyfile(item, stage_dir / item.name)
    _stage(work, endian)
    make = work / "make"
    tools = build_tools()
    # makestg2's i2crom for I2CROM=little_endian or big_endian: one stage-2 file, the other
    # name empty, and both pad ids 1, so one configuration table.
    files = [f'"ibl_c6472/ibl.i2c.{endian}.ccs"', '""']
    i2c_defs = [f"-DI2C_BUS_ADDR=0x{I2C_BUS_ADDR:x}", f"-DI2C_MAP_ADDR=0x{I2C_MAP_ADDR:x}",
                "-DEVM=c6472", "-DPAD_FILE_ID_1=1", "-DPAD_FILE_ID_2=1",
                f'-DINIT_EXE_FILE="ibl_c6472/ibl_init.i2c.{endian}.ccs"',
                f"-DEXE_FILE_1={files[0] if endian == 'le' else files[1]}",
                f"-DEXE_FILE_2={files[0] if endian == 'be' else files[1]}"]
    pp = _preprocess(stage_dir / "i2crom.map.pre", i2c_defs, make, (SRC / "cfg/c6472",))
    _run([str(tools / "romparse"), "-compact", "-rom_base", f"0x{I2C_BUS_ADDR:x}", str(pp)], make)
    shutil.copyfile(make / "i2crom.ccs", stage_dir / "i2crom.ccs")
    ccs2bin = _host_tool("ccs2bin", [SRC / "util/btoccs/ccs2bin.c"], [], [])
    _run([str(ccs2bin), "-swap", "ibl_c6472/i2crom.ccs", "ibl_c6472/i2crom.bin"], make)
    # iblConfig, with the input file its makefile's gen_input writes for EVM=c6472. Its main
    # returns nothing on its error paths, which current compilers refuse unless told not to.
    config_src = SRC / "util/iblConfig/src"
    ibl_config = _host_tool(
        "iblConfig", [config_src / "iblConfig.c", config_src / "device.c"],
        ["-Wno-return-mismatch", "-DEVM=c6472", f"-DI2C_MAP_ADDR=0x{I2C_MAP_ADDR:x}"],
        [config_src])
    config_dir = work / "iblConfig"
    config_dir.mkdir()
    shutil.copyfile(stage_dir / "i2crom.bin", config_dir / "ibl.bin")
    (config_dir / "input.txt").write_text(
        f"file_name = ibl.bin\ndevice = 5\noffset = 0x{I2C_MAP_ADDR:x}\n"
        "ethBoot-doBootp = \nethBoot-bootFormat = \nethBoot-ipAddr = \nethBoot-serverIp = \n"
        "ethBoot-gatewayIp = \nethBoot-netmask = \nethBoot-fileName = \n"
    )
    _run([str(ibl_config)], config_dir)
    built.eeprom.parent.mkdir()
    shutil.copyfile(config_dir / "ibl.bin", built.eeprom)
    return built


if __name__ == "__main__":
    for byte_order in sys.argv[1:] or ["le", "be"]:
        print(build(byte_order))
