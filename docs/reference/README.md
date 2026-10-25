# Reference documents

The TI manuals, application reports and papers this project is derived from, plus the reference
source its claims cite. The project's own documentation is one level up:
[docs/README.md](../README.md).

Target as scoped 2026-08-21: **standalone C64x+ DSP (C645x-class, TMS320C6455)**,
functional / instruction-accurate CPU, with real models for **cache, interrupt
controller and Ethernet (EMAC/MDIO)**. The emulated core talks to the outside
world only over Ethernet.

Every PDF is stored locally; the documents need no network access to re-read. The reference
source the skills cite is not vendored — it is linked at a pinned upstream revision, listed
under [Reference source](#reference-source--upstream-pinned) below.

**Layout:** every document lives directly in `docs/reference/`; the sections below are a
reading order, not directories.

**Naming convention:** every PDF has a descriptive filename, plus a symlink named after
its TI literature number and revision (e.g. `spru871k.pdf -> c64xplus_megamodule_reference.pdf`).
TI documents cross-reference each other only by literature number, so both names resolve.

---

## Toolchain, ABI and assembler

| File | Lit# | Why it matters |
|---|---|---|
| `c64x_dsp_and_cpu_instruction_set_reference.pdf` | SPRU732J | **Primary ISA spec.** Registers, functional units, delay slots, opcode maps, SPLOOP, exceptions. |
| `c6000_embedded_application_binary_interface.pdf` | SPRAB89B | ELF/EABI: relocations, section layout, calling convention, DP-relative addressing. |
| `tms320c6000_assembly_language_tools.pdf` | SPRUI03F | Assembler/linker semantics, linker command files, object format. |
| `tms320c6000_optimizing_c_cpp_compiler.pdf` | SPRUI04G | Intrinsics list, ABI details, SPLOOP generation. |

## CPU core & architecture

| File | Lit# | Why it matters |
|---|---|---|
| `c64xplus_megamodule_reference.pdf` | SPRU871K | **Megamodule Reference.** INTC (interrupt controller), exceptions, cache control regs, IDMA, power-down, memory protection, bandwidth manager. Second-most important doc after SPRU732. |
| `c64xplus_cache_users_guide.pdf` | SPRU862B | C64x+ Cache User's Guide. L1P/L1D/L2 geometry, coherence rules, writeback/invalidate operations. |
| `c64x_to_c64xplus_cpu_migration.pdf` | SPRAA84A | C64x → C64x+ delta. Fast list of what is new in `+`. |
| `c674x_cpu_and_instruction_set_reference.pdf` | SPRUFE8B | C674x CPU & ISA — superset text; often clearer wording for shared instructions. |
| `c6000_cpu_and_instruction_set_reference.pdf` | SPRU189F | Original C62x/C67x CPU & ISA guide (685 pp). Base-ISA pipeline description. |
| `c6000_programmers_guide.pdf` | SPRU198K | C6000 Programmer's Guide. Software pipelining, functional-unit constraints. |
| `c6000_instruction_set_simulator_techref.pdf` | SPRU600I | **TI's own simulator technical reference** — what a C6000 ISS models and what it deliberately doesn't. |
| `c6000_peripherals_overview.pdf` | SPRU190Q | C6000 peripherals overview (family-wide). |
| `c64x_technical_overview.pdf` | SPRU395B | C64x technical overview. |
| `c6000_designing_for_jtag_emulation.pdf` | SPRU641 | JTAG/emulation design — relevant if a debug stub is added. |
| `c6000_dsp_optimization_intro.pdf` | SPRABF2 | C6000 optimization intro — shows the code shapes an emulator will actually run. |
| `c6000_architecture_whitepaper.pdf` | SPRY127 | TI white paper on C6000 architecture rationale. |

## Device / SoC — TMS320C6455 & family (C645x)

| File | Lit# | Module |
|---|---|---|
| `c6455_data_manual.pdf` | SPRS276M | **C6455 data manual** — memory map, interrupt/event map, boot config, pin muxing. |
| `c6455_c6454_silicon_errata.pdf` | SPRZ234R | Silicon errata — behaviours real firmware works around. |
| `c645x_peripherals_overview.pdf` | SPRUE52 | Peripherals overview / doc map for the family. |
| `c6455_technical_reference.pdf` | SPRU965A | C6455 technical reference. |
| `c645x_emac_mdio_users_guide.pdf` | SPRU975 | **EMAC / MDIO User's Guide** — CPPI 3.0 descriptors, TX/RX channels, interrupt/pacing, statistics. Core of the Ethernet model. |
| `c645x_gigabit_emac_software_operation.pdf` | SPRAA90 | Software operation of the gigabit EMAC — driver-level flow. |
| `emac_implementation_guide_c6472.pdf` | SPRAAU2A | EMAC implementation guide (C6472/TCI6486) — practical driver behaviour. |
| `c645x_edma3_controller_users_guide.pdf` | SPRU966 | EDMA3 controller — PaRAM sets, channel controller/transfer controller, chaining. |
| `edma_v2_to_v3_migration.pdf` | SPRAAB9 | EDMA v2 → v3 migration; explains the EDMA3 model. |
| `c645x_64bit_timer_users_guide.pdf` | SPRU968 | 64-bit timers. |
| `c6455_ddr2_memory_controller_users_guide.pdf` | SPRU970 | DDR2 memory controller. |
| `c645x_emif_users_guide.pdf` | SPRU971 | EMIF (async external memory). |
| `c645x_i2c_users_guide.pdf` | SPRU974 | I2C. |
| `c645x_hpi_users_guide.pdf` | SPRU969 | HPI (host port interface). |
| `c645x_gpio_users_guide.pdf` | SPRU724 | GPIO — also an external interrupt source. |
| `c645x_pll_controller_users_guide.pdf` | SPRUE56 | PLL controller — clock config, affects timers. |
| `c645x_utopia2_users_guide.pdf` | SPRUE48 | UTOPIA2 (low priority for this project). |
| `c645x_c647x_bootloader_users_guide.pdf` | SPRUEC6G | **Bootloader** — boot modes, what state the core is in at entry. Covers C645x *and* C647x. |

## Device / SoC — TMS320C6472 & TCI6486 (C647x)

Six-core C64x+ device: same megamodule as the C6455, six of them, plus shared
peripherals (2× EMAC on one shared MDIO, shared L2/SMC, PSC, IPC registers).
Where a module has both a C645x and a C647x guide, the C647x one is authoritative
for the six-core device — the register sets differ.

| File | Lit# | Module |
|---|---|---|
| `c6472_data_manual.pdf` | SPRS612G | **C6472 data manual** — full memory map (per-core and global aliases), device config (DEVSTAT/DEVCTL), IPCGR/IPCAR inter-DSP interrupt registers, boot modes, per-core interrupt/event map, all peripheral register address tables. |
| `c6472_silicon_errata.pdf` | SPRZ300F | Silicon errata, revisions 1.0/1.1/1.2/2.1 — also pins down MM_REVID / CPU REVISION_ID values per silicon rev. |
| `c6472_shared_memory_controller_users_guide.pdf` | SPRUEG5D | **Shared-Memory Controller (SMC)** — 768KB SL2, per-core page ownership/arbitration, atomic-monitor (hardware semaphore) support, shared counters, SL2 power-down. C6455 has no equivalent. |
| `c6472_power_sleep_controller_users_guide.pdf` | SPRUEG3B | **Power/Sleep Controller (GPSC/LPSC)** — per-core clock/reset domains; this is how cores 1–5 are held in reset and released. C6455 has no PSC. |
| `c647x_emac_mdio_users_guide.pdf` | SPRUEF8F | **EMAC / MDIO User's Guide (C6472)** — two EMACs (EMAC0 GMII/RGMII/MII/RMII/SS-SMII, EMAC1 RGMII/RMII/SS-SMII), 8 TX + 8 RX channels each, one shared MDIO. Delta vs. SPRU975 (C6455, single EMAC). |
| `c647x_edma3_controller_users_guide.pdf` | SPRU727E | EDMA3 revision cited by SPRS612 — 64 channels, per-core region/shadow access, queue-to-TC mapping. |
| `c647x_64bit_timer_users_guide.pdf` | SPRU818B | 64-bit timers — six shared GP timers + one GP/watchdog; timer guide cited by SPRS612. |
| `c647x_ddr2_memory_controller_users_guide.pdf` | SPRU894J | DDR2 memory controller (32-bit, 2×256MB address windows). |
| `c647x_pll_controller_users_guide.pdf` | SPRU806A | PLL controllers — PLL1 (core/system), PLL2 (Ethernet), PLL3 (DDR2). No hardware CLKMODE pins; multiplier set in software. |
| `c647x_hpi_users_guide.pdf` | SPRUEG1B | HPI16 — host access into any core's memory; HRDY handshake not supported on C6472. |
| `c647x_i2c_users_guide.pdf` | SPRUE11C | I2C — also the master boot source. |
| `c647x_gpio_users_guide.pdf` | SPRU725A | GPIO (16 pins) — external interrupt source. |
| `c647x_tsip_users_guide.pdf` | SPRUEG4H | Telecom Serial Interface Port ×3 (TDMU/DMATCU register blocks in SPRS612). No C6455 equivalent. |
| `c647x_utopia2_users_guide.pdf` | SPRUEG2E | UTOPIA2 slave ATM controller (low priority). |
| `c647x_srio_users_guide.pdf` | SPRUE13J | Serial RapidIO 2×1x, v1.2 (low priority). |
| `c645x_c647x_bootloader_users_guide.pdf` | SPRUEC6G | **Bootloader** — the same guide covers C645x *and* C647x; documents six-core boot, BOOT_COMPLETE_STAT, BOOTACTIVE, per-core local boot-mode selection. |
| `emac_implementation_guide_c6472.pdf` | SPRAAU2A | EMAC implementation guide for C6472/TCI6486 (listed above with the C645x set). |
| `multicore_programming_guide.pdf` | SPRAB27B | **Multicore Programming Guide** — atomic arbitration via SL2 hardware monitors on C6472/TCI6486, IPC/notify patterns, shared-memory data flow. |
| `c6472_tci6486_hardware_design_guide.pdf` | SPRAAQ4B | Hardware design guide — power sequencing, reset topology, boot strapping at board level. |
| `c6472_tci6486_throughput.pdf` | SPRAAY0A | Throughput — measured latency/bandwidth of SL2, DDR2, EDMA3, SCR paths; the numbers to calibrate any timing model against. |
| `c6472_tci6486_ddr2_implementation_guidelines.pdf` | SPRAAT7E | DDR2 board-level solution (SPRS612 defers all DDR2 timing to this report). |
| `c6472_tci6486_srio_implementation_guidelines.pdf` | SPRAAT9A | SRIO board-level solution (SPRS612 defers SRIO timing to this report). |

Not obtainable: **SPRS300** (TMS320TCI6486 data manual) is not served by `ti.com/lit/pdf`
(404 for `sprs300` and revisions a–d); it appears to be a restricted comms-infrastructure
document. SPRS612G covers the same silicon. There is **no** separate semaphore-module
guide for this device — unlike C6474/C6678, C6472 does atomic arbitration through the SMC
(SPRUEG5D), and inter-core interrupts through IPCGR0–5/IPCAR0–5/IPCGRH/IPCARH
(0x02A8 0540 / 0x02A8 0580) documented directly in SPRS612 §3.7.2, with NMIGR0–5
(0x02A8 0500) in §3.7.1. `SPRM384` (BSDL model) is a BSDL text file, not a PDF.

## Software stack that will run on the emulator

| File | Lit# | Why |
|---|---|---|
| `ndk_users_guide.pdf` | SPRU523K | TI NDK (Network Developer's Kit) User's Guide — the TCP/IP stack firmware likely uses. |
| `ndk_api_reference.pdf` | SPRU524K | NDK API reference. |
| `ndk_ethernet_driver_design_guide.pdf` | SPRUFP2 | NDK Ethernet driver design (NIMU) — how the stack drives the EMAC. |
| `ndk_getting_started.pdf` | SPRAAX4 | Getting started with the C6000 NDK. |
| `dsp_bios_5x_users_guide.pdf` | SPRU423I | DSP/BIOS 5.x user's guide — scheduler, HWI dispatch, what it expects of the INTC. |
| `dsp_bios_5x_api_reference.pdf` | SPRU403 | DSP/BIOS 5.x API reference. |
| `c6000_chip_support_library_api_reference.pdf` | SPRU401J | Chip Support Library API reference — canonical register/field naming. |

## Papers

| File | Why |
|---|---|
| `c6xsim_cycle_accurate_simulator_c62x.pdf` | CMU cycle-accurate C62x simulator write-up — structure of a C6x ISS. |
| `intro_c6x_vliw_dsp_utexas.pdf` | Compact architecture overview, good for orientation. |
| `cycle_accurate_binary_translation_soc.pdf` | DBT techniques if the interpreter ever needs a JIT. |
| `chassaing_dsp_applications_c6x_dsk.pdf` | Practical C6x programming; useful source of test programs. |

## Reference source — upstream, pinned

The machine-readable specs and reference implementations every skill cites. Each was verified
byte-identical at the revision below before the local copies were dropped, so a permalink and a
line number still mean what the text says.

| Source | Pinned at | Why it matters |
|---|---|---|
| [binutils tic6x](https://sourceware.org/git/?p=binutils-gdb.git;a=tree;f=opcodes;hb=refs/tags/binutils-2_44) — [`tic6x-opcode-table.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/opcode/tic6x-opcode-table.h;hb=refs/tags/binutils-2_44), [`tic6x-insn-formats.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/opcode/tic6x-insn-formats.h;hb=refs/tags/binutils-2_44), [`tic6x-control-registers.h`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=include/opcode/tic6x-control-registers.h;hb=refs/tags/binutils-2_44) | tag `binutils-2_44` | **The encoding spec in machine-readable form**: 589 `INSN()` entries over 235 mnemonics, 127 `FMT()` bit-field formats, control-register numbers and permissions. |
| [`opcodes/tic6x-dis.c`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=opcodes/tic6x-dis.c;hb=refs/tags/binutils-2_44), [`gas/config/tc-tic6x.c`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gas/config/tc-tic6x.c;hb=refs/tags/binutils-2_44), [`bfd/elf32-tic6x.c`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=bfd/elf32-tic6x.c;hb=refs/tags/binutils-2_44) | tag `binutils-2_44` | Reference decoder (fetch packets, p-bits, compact headers), the encoder that resolves what the tables leave ambiguous, and every TIC6X ELF relocation implemented. |
| [GDB tic6x support](https://sourceware.org/git/?p=binutils-gdb.git;a=tree;f=gdb;hb=refs/tags/binutils-2_47) — [`tic6x-tdep.c`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/tic6x-tdep.c;hb=refs/tags/binutils-2_47), [`tic6x-core.xml`](https://sourceware.org/git/?p=binutils-gdb.git;a=blob;f=gdb/features/tic6x-core.xml;hb=refs/tags/binutils-2_47) | tag `binutils-2_47` | Register numbering and XML target descriptions — the exact layout the gdb stub exposes. |
| [Linux `arch/c6x`](https://github.com/torvalds/linux/tree/v5.10/arch/c6x) | tag `v5.10` (removed upstream in v5.12) | Entry/exception paths, cache ops, ELF loading, syscall ABI on real C64x+ hardware. |
| [`drivers/net/ethernet/ti/davinci_emac.c`](https://github.com/torvalds/linux/blob/v5.10/drivers/net/ethernet/ti/davinci_emac.c), [`davinci_cpdma.c`](https://github.com/torvalds/linux/blob/v5.10/drivers/net/ethernet/ti/davinci_cpdma.c), [`davinci_mdio.c`](https://github.com/torvalds/linux/blob/v5.10/drivers/net/ethernet/ti/davinci_mdio.c) | tag `v5.10` | The same EMAC IP as the C6455, fully implemented and readable. |
| U-Boot [`drivers/net/ti/davinci_emac.c`](https://github.com/u-boot/u-boot/blob/v2025.01/drivers/net/ti/davinci_emac.c), [`emac_defs.h`](https://github.com/u-boot/u-boot/blob/v2025.01/arch/arm/mach-davinci/include/mach/emac_defs.h) | tag `v2025.01` | A much smaller driver over the same registers; the easier one to read first. |
| GCC backend [`gcc/config/c6x/`](https://github.com/gcc-mirror/gcc/tree/29a97120429f2255311285cdff86b81ba3ad8e63/gcc/config/c6x) — [`c6x.md`](https://github.com/gcc-mirror/gcc/blob/29a97120429f2255311285cdff86b81ba3ad8e63/gcc/config/c6x/c6x.md), [`c6x-sched.md`](https://github.com/gcc-mirror/gcc/blob/29a97120429f2255311285cdff86b81ba3ad8e63/gcc/config/c6x/c6x-sched.md) | commit `29a9712` (2026-06-01; no release tag matches) | Semantics as RTL, unit reservations and latencies, `c6x_intrinsics.h`. |
| [capstone TMS320C64x](https://github.com/capstone-engine/capstone/tree/4.0.2/arch/TMS320C64x) | tag `4.0.2` | LLVM-TableGen-derived decoder tables + operand model; a second independent decoder to cross-check against. |
| [LLVM-TMS320C64X](https://github.com/alexjordan/LLVM-TMS320C64X/tree/e17e694eb6896d9221f617bc04619bbc51c80e14/lib/Target/TMS320C64X) | commit `e17e694` (2012-01-10, repo HEAD) | `TMS320C64XInstrFormats.td`, `InstrInfo.td`, `Schedule.td` — instruction semantics, formats, itineraries, functional-unit/cluster model. |
| TI C6472 Chip Support Library 3.00.07.01 — [`csl_c6472_src_03_00_07_01.zip`](https://software-dl.ti.com/sdoemb/sdoemb_public_sw/csl/CSL_C6472/03_00_07_01/exports/csl_c6472_src_03_00_07_01.zip) (sha256 `aa9e827666b56ef4aebec4142190c5f3ebe1bc5446dda7e70567d10255acf8f0`), [`csl_c6472_03_00_07_01.zip`](https://software-dl.ti.com/sdoemb/sdoemb_public_sw/csl/CSL_C6472/03_00_07_01/exports/csl_c6472_03_00_07_01.zip) (sha256 `a4ac770f64b5760d6bbbe311810306e040cc57e5e5a69dec21179495b450925a`), [user guide](https://software-dl.ti.com/sdoemb/sdoemb_public_sw/csl/CSL_C6472/03_00_07_01/exports/UserGuide_C6472.html) | release `03_00_07_01`, BSD-style licence in each file header | TI's own software view of the C6472 peripherals. For the UTOPIA PDMA: `inc/csl_pdma.h`, `inc/cslr_pim.h` (register layout and reset values), `src/pdma/csl_pdmaHwSetup.c`, `example/utopia2/src/Utopia2_example.c` (the setup SPRUEG2E Examples 1 and 2 print). **Read for the PDMA's open forms and found nothing that settles them**: `TX_EVENT`/`RX_EVENT` in the example are the *event-group* indices 0 and 1 that select PCR0/PCR1, not a UXEVT event ID, and the CSL has no code for SYNC = 0, PE = 3, unequal burst lengths or an incrementing peripheral address. It does establish that TCR bit 17 FF is the *FIFO full flag*, that GCSR resets to 3 (STRT 0, SOFT 1, FREE 1), and that PCR bit 24 is a super-frame-sync select (`SFRM`) the manual calls reserved. Not vendored, like the other upstream sources; the example's prebuilt `tb_prog*` test-bench images are not source and say nothing. |
| TI Intermediate Boot Loader — [`keystone-rtos/ibl`](https://git.ti.com/cgit/keystone-rtos/ibl/) | commit `ed3f0989d8cdc813b7ca1ed899c4d84b63de3a75` (2019-09-16), BSD-style licence in each file header | TI's own C6472 ROM boot EEPROM tooling, the oracle for `i2cboot.rs`: `src/device/c6472/tiboot_c6472.h` is the ROM's boot parameter structures (I2C and UTOPIA offsets match SPRUEC6G Tables 23, 25 and 26; the I2C options' table type is bits 1:0, EEPROM type 4:2); `src/util/bconvert/bconvert64x.c` states that the boot loader reads every 32-bit table word most significant byte first; `src/util/btoccs/b2i2c.c` cuts a boot table into 4-byte-header, 124-byte data blocks; `src/make/ibl_c6472/i2crom.map.pre` and `src/util/romparse/` build the C6472 EVM EEPROM behind boot mode 257 parameter tables. The IBL is also firmware the emulator runs (`tests/test_ibl.py`), and four of its sources settled three of the defects that boot found (the fourth, big-endian doubleword pairing, surfaced in the RTS heap it links): `src/hw/i2c/i2c.c` reads in repeat mode and ends with STP alone (the I2C STOP), `src/device/c6472/c6472init.c` reads CSR.EN, `src/hw/macs/cpmac/cpmacdrv.c` writes MACADDRHI/LO and `src/device/c6472/c6472.c::deviceLoadDefaultEthAddress` reads the MAC ID fuse words, whose byte split the model follows; `src/util/iblConfig/src/device.c::c6472_ibl_config` is the EVM configuration the boot runs with. Provisioned into `tests/build/ibl`, its host tools built by `tests/tools/ibl_tools.py` and the IBL itself by `tests/tools/ibl_build.py`; see below. |

Also read and **not** added, for the PDMA's open forms: SPRS300N, the TMS320TCI6486 data manual (the
`datasheets.pl` mirror, sha256 `19606ccd…8527e`). Its event tables (PXINT0-5 as EDMA3 channels 22-27,
PRINT/PXINT as events 30/31) and UTOPIA/PDMA register tables are the ones SPRS612G already gives, and
its only PDMA text is that the UTOPIA's two-cell FIFOs signal the PDMA to enable servicing. It has no
UXEVT event ID, PE-mode, SYNC = 0 or channel-buffer text; SPRAAY0A (throughput) and the silicon
errata, both already here, have none either.

## tests/build/ — provisioned by tests/tools/fetch_deps.sh, not checked in

`tests/conftest.py` runs that script when any of these trees is missing, so a fresh checkout
provisions itself before the suites that need them run.

### TI C6000 code generation tools

- `tests/build/ti-cgt-c6000_8.5.0.LTS/bin/` — **`cl6x`, `asm6x`, `dis6x`, `ofd6x`, `hex6x`, `nm6x`.** `dis6x` prints
  `address  hexword  mnemonic` and is the ideal oracle for differential decoder testing.
  Verified working: `cl6x -mv6400+ --abi=eabi -O2 -c t.c` then `dis6x t.obj`.
- `tests/build/ti-cgt-c6000_8.5.0.LTS/lib/src/` — RTS sources. `trgcio.h`, `trgmsg.c`, `host*.c` fully specify the
  **CIO host-I/O protocol** used by `printf` in compiled test programs.
- `tests/build/ti-cgt-c6000_8.5.0.LTS/include/c6x.h` — intrinsic → instruction mapping, useful as a semantics cross-check.

### Third-party trees the suite reads as input

| Path | What | Consumed by |
|---|---|---|
| `tests/build/binutils-tic6x-vectors/` | 231 files from binutils' `gas/testsuite/gas/tic6x`, taken out of the 2.44 source release: 90 `.s`, 111 `.d`, 29 `.l` | `tests/tools/decode_sweep.py` decodes every instruction word in them and compares mnemonics with binutils' own rendering |
| `tests/build/dsplib-c64plus/` | DSPLIB 3.4.0.0 for C64x+. Each kernel ships an intrinsics/optimized version, a natural-C reference (`*_cn.c`) and a driver (`*_d.c`) | 23 `tests/programs/*/spec.json` compile sources from `packages/ti/dsplib/src/` and run them in the emulator |
| `tests/build/ibl/` | TI's Intermediate Boot Loader at commit `ed3f098`, a depth-1 git fetch of that commit | `tests/tools/ibl_tools.py` builds `bconvert64x`, `b2i2c`, `b2ccs` and the C6472 `romparse` from it into `tests/build/ibl-tools/` (`bison` and `flex` required); `tests/test_i2cboot.py` boots the EEPROMs they make and holds `tests/tools/i2c_eeprom.py` byte-identical to them. `tests/tools/ibl_build.py` builds the IBL itself, `make evm_c6472` per byte order, into `tests/build/ibl-c6472/` (also adding `ccs2bin` and `iblConfig` to `tests/build/ibl-tools/`), and `tests/test_ibl.py` boots it |

## Notable negative result

There is **no existing open-source C64x/C64x+ emulator**. Decoders exist (binutils,
capstone, LLVM backend) but no execution engine. This project starts from the decoders
and writes the semantics layer itself.

---

## Derived skills

`skills/` holds nine skills distilled from everything above — the knowledge needed to
build the emulator, with page-level references back into these documents. See
[`skills/README.md`](../../skills/README.md) for the list and a suggested build order, and
[docs/README.md](../README.md) for the emulator's own documentation.
