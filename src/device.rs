//! Which device is being emulated: how many megamodules it has, where its memories are, which
//! peripherals it carries and how its events are numbered.
//!
//! The emulator models two members of the C64x+ family. The **C6455** is one megamodule with
//! 2 MB of L2 SRAM and one Ethernet MAC. The **C6472** is six megamodules of 608 KB each,
//! 768 KB of shared L2, and two Ethernet MACs sharing one MDIO, with a boot controller that
//! holds five of the six cores in reset until core 0 releases them (SPRS612G sections 2.3, 2.4
//! and 7.15). Everything that differs between them is a field of a `DeviceProfile`, so the
//! fabric, the memory hierarchy and the front ends have one device model rather than two.

use crate::board::{Board, Boot};
use crate::cache::CacheView;
use crate::cpu::Cpu;
use crate::ddr2::Ddr2Profile;
use crate::events::{EventMap, C6455_EVENTS, C6472_EVENTS};
use crate::hpi::HPI_BOOT_ENTRY;
use crate::memory::{CoherenceHazard, StrictViolation};
use crate::space::{AddressMap, C6455_MAP, C6472_MAP};
use crate::system::{Endian, Fabric};
use crate::utopia::{UtopiaProfile, C6455_UTOPIA, C6472_UTOPIA};
use std::ops::Range;

/// One memory protection region: the local window a unit protects, the size of a page, and the
/// range of MPPA register indices that cover it (SPRU871K chapter 8; the page geometry is
/// per-device, SPRS612G section 5.2 for the C6472).
#[derive(Clone, Copy)]
pub(crate) struct MpuRegion {
    /// 0 = L2, 1 = L1P, 2 = L1D, matching the order of the fault register blocks.
    pub(crate) unit: usize,
    pub(crate) base: u32,
    pub(crate) page_bytes: u32,
    pub(crate) first_index: usize,
    pub(crate) pages: usize,
}

pub(crate) struct DeviceProfile {
    pub(crate) name: &'static str,
    pub(crate) map: &'static AddressMap,
    pub(crate) events: &'static EventMap,
    /// The Ethernet MAC port register bases, and the interrupt-control module base of each. The
    /// C6455 has one MAC whose interrupts are gated by a single EWCTL; the C6472 has two, each
    /// with a per-core interrupt-control module (SPRUEF8F section 3).
    pub(crate) emac_bases: &'static [u32],
    pub(crate) emic_bases: &'static [u32],
    /// The 64-bit timers, in device order. On the C6472 the first six are one per megamodule
    /// and the last six are shared (SPRS612G section 7.16).
    pub(crate) timer_bases: &'static [u32],
    /// How many of `timer_bases`, from the front, belong to a megamodule rather than to the
    /// device. A megamodule's local timer is clocked by that megamodule - "C64x+ Megamodule local
    /// timers are clock gated as well when the corresponding C64x+ Megamodule is clock gated"
    /// (SPRUEG3B section 4.1) - while the C6455's two timers sit on the device's own clock and no
    /// PSC module gates them.
    pub(crate) megamodule_timers: usize,
    /// The memory protection page geometry of one megamodule.
    pub(crate) mpu_regions: &'static [MpuRegion],
    /// Peripherals a device either has or does not.
    pub(crate) has_mcbsp: bool,
    /// DDR2 register contract: SPRU970G (C6455), SPRU894J (C6472).
    pub(crate) ddr2: &'static Ddr2Profile,
    /// The UTOPIA 2 port and what it has: CPU queue windows, or the SPRUEG2E PDMA.
    pub(crate) utopia: Option<&'static UtopiaProfile>,
    pub(crate) has_emifa: bool,
    /// The C6455's device configuration block at 0x02AC_0000.
    pub(crate) has_device_config: bool,
    /// The C6472's boot controller, chip-level registers, power/sleep controller and
    /// shared-memory controller.
    ///
    /// One field because the hardware gives one fact: the C64x+ devices that carry a boot
    /// controller are exactly those that carry the rest of that block set, so a second flag
    /// would be a distinction the silicon does not make. Anything that varies *independently*
    /// of it - the megamodule revision below, for one - is its own field rather than something
    /// derived from this one.
    pub(crate) has_boot_controller: bool,
    /// Configuration windows, reserved tails included, that an external host (HPI) cannot
    /// reach; accesses there never arrive at a register.
    pub(crate) host_excluded: &'static [Range<u32>],
    /// Which core runs at reset with no boot controller involved.
    pub(crate) reset_core: usize,
    /// The JTAG/device identification register value.
    pub(crate) jtag_id: u32,
    /// MM_REVID, the megamodule revision: VERSION 1 in the upper half and the silicon's
    /// revision in the lower (SPRS612G Table 2-1 gives 0003h for the C6472).
    pub(crate) megamodule_revision: u32,
    /// MDIOVER, the version register of the shared MDIO module.
    pub(crate) mdio_version: u32,
}

const C6455_MPU_REGIONS: &[MpuRegion] = &[
    // 2 MB of L2 SRAM in 32 pages of 64 KB, and 32 KB of each L1 SRAM in 32 pages of 1 KB
    // (SPRU871K chapter 8 as the C6455 configures it).
    MpuRegion {
        unit: 0,
        base: 0x0080_0000,
        page_bytes: 64 * 1024,
        first_index: 0,
        pages: 32,
    },
    MpuRegion {
        unit: 1,
        base: 0x00e0_0000,
        page_bytes: 1024,
        first_index: 0,
        pages: 32,
    },
    MpuRegion {
        unit: 2,
        base: 0x00f0_0000,
        page_bytes: 1024,
        first_index: 0,
        pages: 32,
    },
];

/// The C6472's geometry (SPRS612G section 5.2): 608 KB of local L2 in 19 pages of 32 KB
/// (L2MPPA0-18), the 768 KB of shared L2 in 12 pages of 64 KB reached through the same unit at
/// L2MPPA32-43, and 32 KB of each L1 SRAM in 16 pages of 2 KB at MPPA16-31.
const C6472_MPU_REGIONS: &[MpuRegion] = &[
    MpuRegion {
        unit: 0,
        base: 0x0080_0000,
        page_bytes: 32 * 1024,
        first_index: 0,
        pages: 19,
    },
    MpuRegion {
        unit: 0,
        base: 0x0020_0000,
        page_bytes: 64 * 1024,
        first_index: 32,
        pages: 12,
    },
    MpuRegion {
        unit: 1,
        base: 0x00e0_0000,
        page_bytes: 2048,
        first_index: 16,
        pages: 16,
    },
    MpuRegion {
        unit: 2,
        base: 0x00f0_0000,
        page_bytes: 2048,
        first_index: 16,
        pages: 16,
    },
];

/// Timer0 at 0x0294 0000 and timer1 at 0x0298 0000 (SPRS276M Table 2-3).
const C6455_TIMERS: &[u32] = &[0x0294_0000, 0x0298_0000];

/// Twelve timers on 64 KB boundaries from 0x025E 0000: timers 0-5 belong to megamodules 0-5,
/// timers 6-11 are shared (SPRS612G section 7.16.2).
const C6472_TIMERS: &[u32] = &[
    0x025e_0000,
    0x025f_0000,
    0x0260_0000,
    0x0261_0000,
    0x0262_0000,
    0x0263_0000,
    0x0264_0000,
    0x0265_0000,
    0x0266_0000,
    0x0267_0000,
    0x0268_0000,
    0x0269_0000,
];

/// The C6455 host cannot reach the chip-level registers (Table 5-11) or Device State Control
/// (SPRS276M sections 2.4.1 and 4.3), nor the PLL1 and PLL2 controllers (sections 7.7.3 and
/// 7.8.3: "can only be accessed using the CPU or the emulator").
const C6455_HOST_EXCLUDED: &[Range<u32>] = &[
    0x029a_0000..0x029c_0000,
    0x029c_0000..0x029d_0000,
    0x02a8_0000..0x02b0_0000,
];

/// The C6472 restricts only PLL3 (SPRS612G section 7.10.3). Its PLL1/PLL2 controllers and boot
/// controller are host-programmable before a core is released (SPRUEC6G section 4.3.2).
#[allow(clippy::single_range_in_vec_init)] // A list of windows that has one entry.
const C6472_HOST_EXCLUDED: &[Range<u32>] = &[0x029c_0400..0x029c_0800];

pub(crate) const C6455: DeviceProfile = DeviceProfile {
    name: "c6455",
    map: &C6455_MAP,
    events: &C6455_EVENTS,
    emac_bases: &[0x02c8_0000],
    emic_bases: &[],
    timer_bases: C6455_TIMERS,
    megamodule_timers: 0,
    mpu_regions: C6455_MPU_REGIONS,
    has_mcbsp: true,
    ddr2: &crate::ddr2::C6455_DDR2,
    utopia: Some(&C6455_UTOPIA),
    has_emifa: true,
    has_device_config: true,
    has_boot_controller: false,
    host_excluded: C6455_HOST_EXCLUDED,
    reset_core: 0,
    jtag_id: 0x0009_202f,
    megamodule_revision: 0x0001_0000,
    mdio_version: 0x0007_0102,
};

pub(crate) const C6472: DeviceProfile = DeviceProfile {
    name: "c6472",
    map: &C6472_MAP,
    events: &C6472_EVENTS,
    // EMAC0 at 0x02C8 0000 with EMIC0 at 0x02C8 1000; EMAC1 0x40000 higher (SPRS612G
    // Tables 7-101 and 7-103).
    emac_bases: &[0x02c8_0000, 0x02cc_0000],
    emic_bases: &[0x02c8_1000, 0x02cc_1000],
    timer_bases: C6472_TIMERS,
    megamodule_timers: 6,
    mpu_regions: C6472_MPU_REGIONS,
    // Three TSIPs replace the McBSPs, and the UTOPIA is a slave port with its own PDMA.
    has_mcbsp: false,
    ddr2: &crate::ddr2::C6472_DDR2,
    utopia: Some(&C6472_UTOPIA),
    // The C6472 has no asynchronous EMIF: DDR2 is its only external memory.
    has_emifa: false,
    has_device_config: false,
    has_boot_controller: true,
    host_excluded: C6472_HOST_EXCLUDED,
    reset_core: 0,
    jtag_id: 0x0009_102f,
    megamodule_revision: 0x0001_0003,
    mdio_version: 0x0007_0103,
};

impl DeviceProfile {
    pub(crate) fn cores(&self) -> usize {
        self.map.cores
    }

    /// Whether the UTOPIA's queues are serviced by the SPRUEG2E PDMA.
    pub(crate) fn has_utopia_pdma(&self) -> bool {
        self.utopia.is_some_and(|utopia| utopia.pdma)
    }

    /// The profile a `--device` value names.
    pub(crate) fn named(name: &str) -> Option<&'static DeviceProfile> {
        match name {
            "c6455" => Some(&C6455),
            "c6472" => Some(&C6472),
            _ => None,
        }
    }
}

/// The device as a whole: its cores and the fabric they share.
///
/// The fabric is lent to exactly one core at a time. At rest it stays with the **focused**
/// core, which is the one every front end inspects and drives, so `device.cpu()` behaves the way
/// a single-core `Cpu` always did. While the device runs cycles it holds the fabric itself and
/// gives each core that executes a bus onto it; `with_core` lends it to another core for the
/// length of one closure. Neither is observable from outside, so wherever a caller can look,
/// the focused core is the one holding it.
pub struct Device {
    cpus: Vec<Cpu>,
    focus: usize,
    cycle: u64,
    /// The boot the board strapped, which outlives what it performed: what it did is what a
    /// front end reports afterwards.
    boot: Boot,
}

/// The two boundaries a batch of device cycles ends at, kept as bit masks over the cores and
/// brought up to date as each core steps rather than rescanned across every core each cycle.
///
/// Both predicates read only a core's own state - its PC, its stall counters, its loop buffer,
/// its fault and its halt list - and nothing another core or the fabric can reach. Within a
/// batch that state changes only when the core itself steps, or when the device moves its PC
/// (the ROM boot's entry, a boot-controller release); the device observes the core again at
/// each of those points, so the masks always equal a fresh scan. Whether a core is held in
/// reset is fabric state, which can change on any cycle, so it is not cached: the halt test
/// reads it at the moment it asks, as the scan did.
struct Boundaries {
    /// The CIO breakpoint, when the machine has the handshake armed.
    cio: Option<u32>,
    /// Cores for which `Cpu::halted` holds, held in reset or not.
    halted: u32,
    /// Cores for which `Cpu::ready_to_fetch_at(cio)` holds.
    at_cio: u32,
}

impl Boundaries {
    fn scan(cpus: &[Cpu], cio: Option<u32>) -> Self {
        let mut boundaries = Self {
            cio,
            halted: 0,
            at_cio: 0,
        };
        for (core, cpu) in cpus.iter().enumerate() {
            boundaries.observe(core, cpu);
        }
        boundaries
    }

    #[inline]
    fn observe(&mut self, core: usize, cpu: &Cpu) {
        let bit = 1u32 << core;
        self.halted = (self.halted & !bit) | (u32::from(cpu.halted()) << core);
        let at_cio = self
            .cio
            .is_some_and(|address| cpu.ready_to_fetch_at(address));
        self.at_cio = (self.at_cio & !bit) | (u32::from(at_cio) << core);
    }

    /// Whether a core that is out of reset has halted. A held core does not fetch, so an armed
    /// address its reset PC happens to match is not a halt.
    #[inline]
    fn any_halted(&self, fabric: &Fabric) -> bool {
        let mut halted = self.halted;
        while halted != 0 {
            let core = halted.trailing_zeros() as usize;
            if !fabric.held_in_reset(core) {
                return true;
            }
            halted &= halted - 1;
        }
        false
    }
}

impl Device {
    /// A device whose cores all start at `entry`. A C6472's boot controller then decides which
    /// of them actually run: in immediate boot every core starts at once, and in every other
    /// boot mode core 0 runs and releases the rest (SPRS612G section 2.4.1).
    #[cfg(test)]
    pub fn new(profile: &'static DeviceProfile, entry: u32, endian: Endian) -> Self {
        Self::with_boot_mode(profile, entry, endian, 0)
    }

    /// A device on a board that straps `boot_mode` and supplies nothing else.
    #[cfg(test)]
    pub fn with_boot_mode(
        profile: &'static DeviceProfile,
        entry: u32,
        endian: Endian,
        boot_mode: u8,
    ) -> Self {
        Self::on_board(&Board::pins(profile, boot_mode), entry, endian)
    }

    /// The device a board carries, strapped and fused as the board has it and with the boot
    /// it straps started: mode 0 releases every core, HPI boot holds every core for the host,
    /// and a ROM boot runs in place of core 0 while the others wait. The C6455's HPI entry is
    /// fixed at L2 base; C6472 cores start at their programmed boot addresses.
    pub(crate) fn on_board(board: &Board, entry: u32, endian: Endian) -> Self {
        let profile = board.profile();
        let mut fabric = Box::new(Fabric::with_boot_mode(profile, board.mode()));
        fabric.apply_reset_holds();
        // A DSPINT-released core starts at the base of L2 wherever `entry` says. A C6472 core's
        // start address is not set until the host releases it, from its DSP_BOOT_ADDR.
        let entry = if fabric.hpi_boot_pending() {
            HPI_BOOT_ENTRY
        } else {
            entry
        };
        let mut cpus: Vec<Cpu> = Vec::with_capacity(profile.cores());
        for core in (0..profile.cores()).rev() {
            // Each core in turn takes the fabric to be built with, and the last one built -
            // core 0 - keeps it, which is where the focus starts.
            let mut cpu = Cpu::new_on_device(entry, endian, core, fabric);
            cpu.set_core_number(core);
            fabric = cpu.detach();
            cpus.push(cpu);
        }
        cpus.reverse();
        fabric.set_strict_focus(profile.reset_core);
        cpus[profile.reset_core].attach(fabric);
        let boot = board.start(cpus[profile.reset_core].fabric_mut());
        Self {
            cpus,
            focus: profile.reset_core,
            cycle: 0,
            boot,
        }
    }

    /// The boot the board strapped: whether it holds core 0, and what it has done.
    pub(crate) fn boot(&self) -> &Boot {
        &self.boot
    }

    /// Whether a core is held in reset, without borrowing the fabric mutably: the halt rule
    /// asks on every pass and only needs to read.
    pub fn held_in_reset(&self, core: usize) -> bool {
        self.cpus[self.focus].fabric_ref().held_in_reset(core)
    }

    /// The core that has halted on an armed address, if any. Held cores do not fetch, and
    /// simultaneous hits are attributed deterministically to the lowest core index.
    pub fn halted_core(&self) -> Option<usize> {
        (0..self.cpus.len()).find(|&core| !self.held_in_reset(core) && self.cpus[core].halted())
    }

    /// Program every core's caches through the configuration registers firmware uses (SPRU871K
    /// L1PCFG, L1DCFG and L2CFG), rather than by reaching into the model.
    pub(crate) fn configure_caches(&mut self, l1p: u32, l1d: u32, l2: u32) {
        self.fabric().configure_caches(l1p, l1d, l2);
    }

    /// Check DMA masters' traffic against every core's caches, reporting through
    /// `take_coherence_hazards`.
    pub(crate) fn set_coherence_check(&mut self, on: bool) {
        self.fabric().set_coherence_check(on);
    }

    pub(crate) fn take_coherence_hazards(&mut self) -> Vec<(usize, CoherenceHazard)> {
        self.fabric().take_coherence_hazards()
    }

    /// Place image bytes in memory the way the ELF loader does: behind the caches, once.
    pub(crate) fn load_image(&mut self, address: u32, bytes: &[u8]) {
        self.fabric().load_image_bytes(address, bytes);
    }

    /// Place a ROM boot table at `address` and process it as the bootloader would on the
    /// focused core, answering the entry point it names (SPRUEC6G section 6.2.2).
    pub(crate) fn load_boot_table(&mut self, address: u32, table: &[u8]) -> Result<u32, String> {
        let cpu = self.cpu_mut();
        cpu.write_bytes(address, table);
        crate::boot::load_boot_table(&mut cpu.bus_mut(), address)
    }

    /// Place a boot configuration table at `address` and apply it as the bootloader would on
    /// the focused core, answering the functions it asks to call (SPRUEC6G section 6.2.3).
    pub(crate) fn apply_boot_configuration(
        &mut self,
        address: u32,
        table: &[u8],
    ) -> Result<Vec<u32>, String> {
        let cpu = self.cpu_mut();
        cpu.write_bytes(address, table);
        crate::boot::apply_boot_configuration(&mut cpu.bus_mut(), address)
    }

    /// The focused megamodule's cache state, for a front end that draws it.
    pub(crate) fn cache_views(&self) -> [CacheView; 3] {
        self.focused_megamodule().cache_views()
    }

    /// One resident line of the focused megamodule's caches, or nothing if that level does not
    /// hold the address. Passive: reading it moves no replacement clock.
    pub(crate) fn cache_line_data(&self, level: usize, address: u32) -> Option<&[u8]> {
        self.focused_megamodule().cache_line_data(level, address)
    }

    /// The focused megamodule's interrupt selectors, event flags and cacheability registers.
    pub(crate) fn interrupt_selectors(&self) -> [u32; 3] {
        self.focused_megamodule().interrupt_selectors()
    }

    pub(crate) fn event_flag_registers(&self) -> [u32; 4] {
        self.focused_megamodule().event_flag_registers()
    }

    pub(crate) fn mar_registers(&self) -> [u32; 256] {
        self.focused_megamodule().mar_registers()
    }

    fn focused_megamodule(&self) -> &crate::system::Megamodule {
        self.cpus[self.focus].fabric_ref().megamodule(self.focus)
    }

    pub fn cores(&self) -> usize {
        self.cpus.len()
    }

    /// The core every front end is looking at.
    pub fn focus(&self) -> usize {
        self.focus
    }

    /// Look at a different core. The fabric follows the focus, so the newly focused core can
    /// be inspected and driven exactly like the old one.
    pub fn set_focus(&mut self, core: usize) {
        assert!(core < self.cpus.len(), "no such core");
        if core == self.focus {
            return;
        }
        let mut fabric = self.cpus[self.focus].detach();
        // A system master's strict violation is reported on the core focused when it is made.
        fabric.set_strict_focus(core);
        self.cpus[core].attach(fabric);
        self.focus = core;
    }

    pub fn cpu(&self) -> &Cpu {
        &self.cpus[self.focus]
    }

    pub fn cpu_mut(&mut self) -> &mut Cpu {
        &mut self.cpus[self.focus]
    }

    pub fn core(&self, core: usize) -> &Cpu {
        &self.cpus[core]
    }

    /// Drive one core with the fabric attached to it, whichever core is focused.
    pub fn with_core<R>(&mut self, core: usize, action: impl FnOnce(&mut Cpu) -> R) -> R {
        let focus = self.focus;
        if focus != core {
            let fabric = self.cpus[focus].detach();
            self.cpus[core].attach(fabric);
        }
        let result = action(&mut self.cpus[core]);
        if focus != core {
            let fabric = self.cpus[core].detach();
            self.cpus[focus].attach(fabric);
        }
        result
    }

    /// The shared fabric, for the device-level operations no single core owns.
    pub fn fabric(&mut self) -> &mut Fabric {
        self.cpus[self.focus].fabric_mut()
    }

    pub fn cycle(&self) -> u64 {
        self.cycle
    }

    /// One device cycle: every core that is out of reset executes, and the shared peripherals
    /// advance once.
    ///
    /// Once strict physical-memory mode has refused an access, no further cycle runs.
    pub fn step_cycle(&mut self) {
        let mut fabric = self.cpus[self.focus].detach();
        if !fabric.strict_stopped() {
            self.cycle_on(&mut fabric, None);
        }
        self.latch_strict_violation_from(&mut fabric);
        self.cpus[self.focus].attach(fabric);
    }

    /// The first access strict physical-memory mode refused. The core it is reported on is
    /// fixed in it (`StrictViolation::reported_core`), whatever the focus is now.
    pub(crate) fn strict_violation(&self) -> Option<&StrictViolation> {
        self.cpus[self.focus].fabric_ref().strict_violation()
    }

    /// Hand a strict physical-memory violation to the core it is reported on, as that core's
    /// fault, so every front end stops and reports it the way it does any fault. Idempotent,
    /// and it keeps whatever fault the core already had.
    pub(crate) fn latch_strict_violation(&mut self) {
        let mut fabric = self.cpus[self.focus].detach();
        self.latch_strict_violation_from(&mut fabric);
        self.cpus[self.focus].attach(fabric);
    }

    fn latch_strict_violation_from(&mut self, fabric: &mut Fabric) {
        let Some(core) = fabric.strict_violation().map(|v| v.reported_core) else {
            return;
        };
        let cpu = &mut self.cpus[core];
        if let Some(violation) = fabric.latch_strict_stop(core, cpu.pc(), cpu.cycles()) {
            cpu.latch_strict_violation(violation);
        }
    }

    /// Run up to `cycles` cycles while no machine-level service is due, answering how many
    /// ran. The batch ends early, before the cycle that would cross it, when a core out of
    /// reset has halted on an armed address or a core is ready to fetch the CIO breakpoint -
    /// except on the first cycle when `ignore_cio_once` says the machine has just answered
    /// that handshake and the core is still sitting on it.
    ///
    /// For a nonzero request, zero means a boundary is already due (or the single core
    /// already has a fault). Otherwise the device advances without caller-side stepping.
    /// CIO includes held cores, matching the machine's handshake scan; its first-cycle
    /// exemption still advances a device cycle even when the core at CIO cannot run.
    ///
    /// A single core with no boot holding it runs itself: `Cpu::run_cycles` is the fast path,
    /// it ticks the device from inside its own cycle and counts in the core's cycles, and it
    /// leaves the device's cycle counter alone. Six cores, or a boot that holds core 0 - device
    /// state the core cannot step past on its own - advance whole device cycles and count in
    /// those.
    pub(crate) fn run_cycles(
        &mut self,
        cycles: u64,
        cio: Option<u32>,
        ignore_cio_once: bool,
    ) -> u64 {
        if self.cpus.len() == 1 && !self.boot.holds_core_zero() {
            return self.cpus[0].run_cycles(cycles, cio, ignore_cio_once);
        }
        // The device keeps the fabric for the whole batch and gives it back to the focused
        // core at the end, which is before any caller can look.
        let mut fabric = self.cpus[self.focus].detach();
        let mut boundaries = Boundaries::scan(&self.cpus, cio);
        let start = self.cycle;
        // A strict physical-memory violation ends the batch after the device cycle that made it,
        // as a fault does. Whether the diagnostic is on is asked once per batch.
        let strict = fabric.strict_on();
        for step in 0..cycles {
            if (strict && fabric.strict_stopped()) || boundaries.any_halted(&fabric) {
                break;
            }
            if boundaries.at_cio != 0 && !(step == 0 && ignore_cio_once) {
                break;
            }
            self.cycle_on(&mut fabric, Some(&mut boundaries));
        }
        if fabric.strict_stopped() {
            self.latch_strict_violation_from(&mut fabric);
        }
        self.cpus[self.focus].attach(fabric);
        self.cycle - start
    }

    /// The one implementation of a device cycle, against the fabric the device is holding.
    /// `boundaries`, when a batch is watching for them, is kept current for every core whose
    /// state the cycle touched.
    ///
    /// Cores run in index order and the peripherals tick inside the first core's cycle, which
    /// is what the single-core device has always done; the cores that follow it in the same
    /// cycle therefore see the peripheral state that core has just produced. That is a choice
    /// the hardware does not make for us - six cores are concurrent - but it is deterministic,
    /// which is worth more to a debugger than an invented interleaving. Whether a core runs is
    /// asked when its turn comes, so an earlier core's access this cycle can hold, release or
    /// gate a later one.
    fn cycle_on(&mut self, fabric: &mut Fabric, mut boundaries: Option<&mut Boundaries>) {
        for core in 0..self.cpus.len() {
            if fabric.held_in_reset(core) {
                fabric.drop_held_core_events(core);
                continue;
            }
            if !fabric.core_clocked(core) {
                continue;
            }
            let cpu = &mut self.cpus[core];
            cpu.step_on(fabric);
            if let Some(boundaries) = boundaries.as_deref_mut() {
                boundaries.observe(core, cpu);
            }
        }
        // With every core held in reset the peripherals still run: that is how a host boots the
        // device through the HPI while the cores wait (SPRUEC6G section 4.3.2).
        fabric.tick(self.cycle);
        self.cycle += 1;
        if let Some(entry) = self.boot.tick(fabric) {
            // The boot table has ended: the ROM branches core 0 to the entry point it named,
            // and the cores the table released are already starting from their own boot
            // addresses (SPRUEC6G section 4.3.5).
            self.cpus[0].set_pc(entry);
            fabric.set_held_in_reset(0, false);
            if let Some(boundaries) = boundaries.as_deref_mut() {
                boundaries.observe(0, &self.cpus[0]);
            }
        }
        self.apply_device_requests(fabric, boundaries);
    }

    /// The device-level effects a cycle produced: cores released from reset, and cores whose
    /// NMI pin another core drove (SPRS612G sections 2.4.1 and 3.7.1).
    fn apply_device_requests(
        &mut self,
        fabric: &mut Fabric,
        mut boundaries: Option<&mut Boundaries>,
    ) {
        let released = fabric.take_release_requests();
        for core in 0..self.cpus.len() {
            if released & (1 << core) == 0 {
                continue;
            }
            if self.boot.owns_release(core) {
                continue;
            }
            let held = fabric.held_in_reset(core);
            let address = fabric.boot_address(core);
            if let Some(address) = address {
                self.cpus[core].set_pc(address);
                if let Some(boundaries) = boundaries.as_deref_mut() {
                    boundaries.observe(core, &self.cpus[core]);
                }
            }
            self.boot.released(core, held, address);
            fabric.set_held_in_reset(core, false);
        }
        // Driving the NMI pin latches an edge and moves nothing a boundary reads.
        let nmi = fabric.take_nmi_requests();
        for core in 0..self.cpus.len() {
            if nmi & (1 << core) != 0 {
                // NMIGR is a pulse: the pin goes high and back down, which is one edge.
                self.cpus[core].set_nmi_line(true);
                self.cpus[core].set_nmi_line(false);
            }
        }
    }

    /// A device built around a CPU a test made itself, which is always single-core.
    #[cfg(test)]
    pub fn from_cpu(cpu: Cpu) -> Self {
        Self {
            cpus: vec![cpu],
            focus: 0,
            cycle: 0,
            boot: Boot::none(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::{L1DCFG, L1PCFG, L2CFG};
    use crate::system::{Endian, MasterAccess, Privilege};

    /// Boot controller registers (SPRS612G section 3.9) and the chip-level inter-processor
    /// interrupt registers (section 3.7.2).
    const BOOT_COMPLETE_STAT: u32 = 0x02ab_0004;
    const DSP_BOOT_ADDR0: u32 = 0x02ab_0204;
    const IPCGR0: u32 = 0x02a8_0540;
    const IPCAR0: u32 = 0x02a8_0580;
    /// EVTFLAG2 covers events 64-95, so IPC_LOCAL (84) is bit 20 of it.
    const EVTFLAG2: u32 = 0x0180_0008;

    fn c6472() -> Device {
        Device::new(&C6472, 0x0080_0000, Endian::Little)
    }

    #[test]
    fn hpi_boot_holds_core_zero_in_reset_until_the_hosts_dspint() {
        use crate::hpi::{HostPort, BOOTMODE_HPI};
        const HPIC: u32 = 0x0288_0030;
        const HPIC_DSPINT: u32 = 1 << 1;
        const HPIC_HRDY: u32 = 1 << 3;
        const HPIC_HPIRST: u32 = 1 << 7;
        const PERCFG0: u32 = 0x02ac_0008;
        const PERSTAT0: u32 = 0x02ac_0014;
        const CSR: u8 = 0x01;
        const IFR: u8 = 0x02;
        const INT15: u32 = 1 << 15;

        for endian in [Endian::Little, Endian::Big] {
            let mut device = Device::with_boot_mode(&C6455, 0x0080_1000, endian, BOOTMODE_HPI);
            // Strapped to the boot, the HPI is enabled and out of its software reset before
            // any cycle has run: nothing on the DSP side runs to open it for the host.
            assert_eq!(
                device.fabric().hpi_host_read(HostPort::Control),
                Some(HPIC_HRDY)
            );
            let mut bus = device.cpu_mut().bus_mut();
            assert_eq!(bus.read32(HPIC) & HPIC_HPIRST, 0);
            assert_ne!(bus.read32(PERCFG0) & (1 << 18), 0);
            assert_eq!((bus.read32(PERSTAT0) >> 27) & 7, 1, "HPI enabled");
            assert_eq!(bus.read32(L1PCFG) & 7, 4);
            assert_eq!(bus.read32(L1DCFG) & 7, 4);
            assert_eq!(bus.read32(L2CFG) & 7, 0);
            assert!(device.held_in_reset(0));
            assert_eq!(device.boot().hpi_entry(0), None);

            // Three posted words cannot complete a four-word DMA burst. The host must flush
            // its final short burst; the boot ending does not do it.
            let address = 0x0080_2000;
            device
                .fabric()
                .hpi_host_write(HostPort::Address, address >> 2);
            for word in [0x1234_5678, 0x89ab_cdef, 0xdead_beef] {
                device
                    .fabric()
                    .hpi_host_write(HostPort::DataAutoIncrement, word);
            }
            device.step_cycle();
            assert_eq!(device.cpu_mut().bus_mut().read32(address), 0);
            device
                .fabric()
                .hpi_host_write(HostPort::Address, address >> 2);
            assert_eq!(device.cpu_mut().bus_mut().read32(address + 8), 0xdead_beef);
            device.step_cycle();
            assert!(
                device.held_in_reset(0),
                "loading memory is not boot completion"
            );
            assert_eq!(device.core(0).cycles(), 0);

            device
                .fabric()
                .hpi_host_write(HostPort::Control, HPIC_DSPINT);
            device.step_cycle();
            assert!(!device.held_in_reset(0));
            assert_eq!(device.core(0).pc(), 0x0080_0000, "not the ELF entry");
            assert_eq!(
                device.core(0).cycles(),
                0,
                "release does not execute a packet"
            );
            assert_eq!(device.boot().hpi_entry(0), Some(0x0080_0000));
            // The CPU leaves reset with interrupts disabled, and DSPINT stays set until the
            // application clears it.
            assert_eq!(device.core(0).read_control(CSR) & 1, 0, "GIE");
            assert_eq!(
                device.fabric().hpi_host_read(HostPort::Control),
                Some(HPIC_HRDY | HPIC_DSPINT)
            );
            // The DSPINT arrived while the core was in reset, so it never reaches IFR.
            for _ in 0..20 {
                device.step_cycle();
            }
            assert_eq!(device.core(0).read_control(IFR) & INT15, 0);

            // While DSPINT stands the host cannot raise another; once the application clears
            // it, the next one is an ordinary interrupt and the core keeps running.
            device
                .fabric()
                .hpi_host_write(HostPort::Control, HPIC_DSPINT);
            for _ in 0..20 {
                device.step_cycle();
            }
            assert_eq!(device.core(0).read_control(IFR) & INT15, 0);
            device.cpu_mut().bus_mut().write32(HPIC, HPIC_DSPINT);
            assert_eq!(
                device.fabric().hpi_host_read(HostPort::Control),
                Some(HPIC_HRDY)
            );
            device
                .fabric()
                .hpi_host_write(HostPort::Control, HPIC_DSPINT);
            let cycles = device.core(0).cycles();
            for _ in 0..20 {
                device.step_cycle();
            }
            assert_ne!(device.core(0).read_control(IFR) & INT15, 0);
            assert!(!device.held_in_reset(0));
            assert!(device.core(0).cycles() > cycles, "the core was not reset");
        }
    }

    /// SPRS276M section 7.4.1 and Table 7-3 note 2: the C6455's DSPINT is EDMA3 event DSP_EVT,
    /// so the one ending an HPI boot is waiting in ER bit 0 when the firmware starts, and an
    /// enabled channel 0 transfers on it - the stale boot event as soon as the channel is
    /// enabled, and every later host DSPINT after that, alongside the CPU interrupt.
    #[test]
    fn c6455_dspint_is_edma_event_zero_from_the_boot_on() {
        use crate::hpi::{HostPort, BOOTMODE_HPI};
        const HPIC: u32 = 0x0288_0030;
        const HPIC_DSPINT: u32 = 1 << 1;
        const ER: u32 = 0x02a0_1000;
        const ECR: u32 = 0x02a0_1008;
        const EER: u32 = 0x02a0_1020;
        const EECR: u32 = 0x02a0_1028;
        const EESR: u32 = 0x02a0_1030;
        const EMR: u32 = 0x02a0_0300;
        const IPR: u32 = 0x02a0_1068;
        const ICR: u32 = 0x02a0_1070;
        const PARAM0: u32 = 0x02a0_4000;
        const EVTFLAG0: u32 = 0x0180_0000;
        const EVTCLR0: u32 = 0x0180_0040;
        /// TCINTEN with TCC 0: completion sets IPR bit 0.
        const OPT_TCINTEN: u32 = 1 << 20;
        const SOURCE: u32 = 0x0080_8000;
        const FIRST: u32 = 0x0080_9000;
        const SECOND: u32 = 0x0080_a000;
        let words = [0x1111_1111u32, 0x2222_2222, 0x3333_3333, 0x4444_4444];

        // One A-synchronised 16-byte copy on PaRAM set 0, which channel 0 maps to at reset.
        fn program(device: &mut Device, source: u32, destination: u32) {
            let mut bus = device.cpu_mut().bus_mut();
            let set = [
                OPT_TCINTEN,
                source,
                (1 << 16) | 16,
                destination,
                0,
                0xffff,
                0,
                1,
            ];
            for (index, word) in set.into_iter().enumerate() {
                bus.write32(PARAM0 + 4 * index as u32, word);
            }
        }
        let copied = |device: &mut Device, at: u32| {
            let mut bus = device.cpu_mut().bus_mut();
            (0..4).map(|i| bus.read32(at + 4 * i)).collect::<Vec<_>>()
        };

        let mut device = Device::with_boot_mode(&C6455, 0x0080_0000, Endian::Little, BOOTMODE_HPI);
        host_load(&mut device, SOURCE, &words);
        assert_eq!(host_get(&mut device, ER), 0, "loading is no event");
        device
            .fabric()
            .hpi_host_write(HostPort::Control, HPIC_DSPINT);
        device.step_cycle();
        assert!(!device.held_in_reset(0));

        // What the firmware finds: the boot's event pending, nothing enabled, nothing missed,
        // and no transfer.
        run(&mut device, 20);
        {
            let mut bus = device.cpu_mut().bus_mut();
            assert_eq!(bus.read32(ER), 1);
            assert_eq!(bus.read32(EER), 0);
            assert_eq!(bus.read32(EMR), 0);
            assert_eq!(bus.read32(IPR), 0);
        }

        // Enabling channel 0 without clearing ER first runs the transfer on the stale event.
        program(&mut device, SOURCE, FIRST);
        device.cpu_mut().bus_mut().write32(EESR, 1);
        run(&mut device, 200);
        assert_eq!(copied(&mut device, FIRST), words);
        {
            let mut bus = device.cpu_mut().bus_mut();
            assert_eq!(bus.read32(ER), 0, "serviced");
            assert_eq!(bus.read32(IPR), 1);
            assert_eq!(bus.read32(EMR), 0);
            bus.write32(ICR, 1);
            // Acknowledge the boot DSPINT, without which the host cannot raise another.
            bus.write32(HPIC, HPIC_DSPINT);
            // Forget the boot's system event so the next one is seen fresh.
            bus.write32(EVTCLR0, u32::MAX);
        }
        program(&mut device, SOURCE, SECOND);
        // Let the channel controller go dormant: nothing but the host's DSPINT can wake it.
        run(&mut device, 50);

        // The next host DSPINT is the system event and the channel 0 trigger at once.
        device
            .fabric()
            .hpi_host_write(HostPort::Control, HPIC_DSPINT);
        run(&mut device, 200);
        assert_eq!(copied(&mut device, SECOND), words);
        {
            let mut bus = device.cpu_mut().bus_mut();
            assert_eq!(bus.read32(IPR), 1);
            assert_eq!(bus.read32(EMR), 0);
            assert_ne!(bus.read32(EVTFLAG0) & (1 << 15), 0, "DSPINT system event");

            // With the channel disabled the event is only latched, and ECR clears it.
            bus.write32(ICR, 1);
            bus.write32(HPIC, HPIC_DSPINT);
            bus.write32(EECR, 1);
        }
        device
            .fabric()
            .hpi_host_write(HostPort::Control, HPIC_DSPINT);
        run(&mut device, 50);
        let mut bus = device.cpu_mut().bus_mut();
        assert_eq!(bus.read32(ER), 1);
        assert_eq!(bus.read32(IPR), 0);
        bus.write32(ECR, 1);
        assert_eq!(bus.read32(ER), 0);
    }

    /// SPRS276M sections 2.4.1, 4.3, 7.7.3 and 7.8.3: Device Configuration and PLL1/PLL2 are
    /// the C64x+ megamodule's alone, so the host is shut out while the CPU and IDMA are not.
    #[test]
    fn c6455_hpi_cannot_change_state_or_pll_registers() {
        use crate::hpi::{HostPort, BOOTMODE_HPI};

        for endian in [Endian::Little, Endian::Big] {
            for boot_mode in [0, BOOTMODE_HPI] {
                let mut device = Device::with_boot_mode(&C6455, 0x0080_0000, endian, boot_mode);
                // Normal mailbox operation opens the HPI from the CPU; boot opens it at reset.
                if boot_mode == 0 {
                    device.cpu_mut().bus_mut().write32(0x0288_0030, 0);
                }
                for (address, initial, attempted) in [
                    (0x02a8_0004, 0x311, 0x100),             // PRI_ALLOC
                    (0x02ac_0004, 0xf0f0_f0f0, 0x0f0a_0b00), // PERLOCK
                    (0x02ac_0054, 1, 0),                     // EMUBUFPD
                    (0x029a_0110, 9, 7),                     // PLL1 PLLM
                    (0x029c_0118, 0x8001, 0x8004),           // PLL2 PLLDIV1
                ] {
                    device.cpu_mut().bus_mut().write32(address, initial);
                    assert_eq!(device.cpu_mut().bus_mut().read32(address), initial);
                    // Forbidden reads must not expose CPU-visible register state.
                    // Zero is the emulator's explicit response policy, not a TI reset value.
                    assert_eq!(host_get(&mut device, address), 0);
                    device
                        .fabric()
                        .hpi_host_write(HostPort::Address, address >> 2);
                    assert_eq!(
                        device.fabric().hpi_host_read(HostPort::DataAutoIncrement),
                        Some(0)
                    );
                    host_set(&mut device, address, attempted);
                    assert_eq!(
                        device.cpu_mut().bus_mut().read32(address),
                        initial,
                        "non-incrementing host write reached {address:#x}"
                    );
                    // Both a full burst and a short burst flushed by HPIA must be excluded.
                    for count in [4, 1] {
                        host_load(&mut device, address, &[attempted; 4][..count]);
                        assert_eq!(
                            device.cpu_mut().bus_mut().read32(address),
                            initial,
                            "posted host write reached {address:#x}"
                        );
                    }
                    // The host filter must not leak into the IDMA's word-access path.
                    let idma = Privilege::ID0_SUPERVISOR;
                    device.fabric().write_word(idma, address, attempted);
                    assert_eq!(device.cpu_mut().bus_mut().read32(address), attempted);
                    assert_eq!(device.fabric().read_word(idma, address), Some(attempted));
                }
                // The port remains usable for boot-image memory after denied accesses.
                host_set(&mut device, 0x0080_1000, 0x1234_5678);
                assert_eq!(host_get(&mut device, 0x0080_1000), 0x1234_5678);
                assert_eq!(device.cpu_mut().bus_mut().read32(0x0080_1000), 0x1234_5678);
            }
        }
    }

    /// The host's side of a C6472 HPI boot, one word-addressed access at a time. Rewriting the
    /// address register at the end flushes whatever short burst is still posted.
    fn host_load(device: &mut Device, address: u32, words: &[u32]) {
        use crate::hpi::HostPort;
        device
            .fabric()
            .hpi_host_write(HostPort::Address, address >> 2);
        for &word in words {
            device
                .fabric()
                .hpi_host_write(HostPort::DataAutoIncrement, word);
        }
        device
            .fabric()
            .hpi_host_write(HostPort::Address, address >> 2);
    }

    fn host_set(device: &mut Device, address: u32, value: u32) {
        use crate::hpi::HostPort;
        device
            .fabric()
            .hpi_host_write(HostPort::Address, address >> 2);
        device.fabric().hpi_host_write(HostPort::Data, value);
    }

    fn host_get(device: &mut Device, address: u32) -> u32 {
        use crate::hpi::HostPort;
        device
            .fabric()
            .hpi_host_write(HostPort::Address, address >> 2);
        device.fabric().hpi_host_read(HostPort::Data).unwrap()
    }

    fn run(device: &mut Device, cycles: u32) {
        for _ in 0..cycles {
            device.step_cycle();
        }
    }

    /// SPRUEC6G section 4.3.2: with HPI boot every C6472 core is held in reset while the host
    /// initialises memory and configuration registers, and only a 0 to 1 transition of a BCn bit
    /// in BOOT_COMPLETE_STAT releases core n, at the address in its DSP_BOOT_ADDRn. DSPINT is an
    /// ordinary interrupt here (the C6455's DSPINT release does not exist on this device).
    #[test]
    fn c6472_hpi_boot_releases_only_the_cores_the_host_names() {
        use crate::hpi::{HostPort, BOOTMODE_HPI};
        const HPIC_DSPINT: u32 = 1 << 1;
        const HPIC_HRDY: u32 = 1 << 3;
        /// MVK .L1 0,A4 with its constant in bits 22-18, and ADD .L1 1,A4,A4.
        const MVK_A4: u32 = 0x0200_a358;
        const ADD_A4: u32 = 0x0210_2058;

        // Core n's image leaves 2 * (n + 1) in A4 and then runs on through the zeroed memory
        // after it, which is NOPs, so a wrong core, a wrong address or a restart is told apart.
        // It is n + 2 words long, so most images end in a short burst the host has to flush.
        let image = |core: usize| {
            let mut words = vec![ADD_A4; core + 2];
            words[0] = MVK_A4 | ((core as u32 + 1) << 18);
            words
        };
        let result = |core: usize| 2 * (core as u32 + 1);
        let global = |core: usize, local: u32| 0x1000_0000 + ((core as u32) << 24) + local;
        // DSP_BOOT_ADDRn is the start address shifted right by ten, so the start has to be
        // aligned to 1 KB (section 3.9.5): cores 0 and 3 take two such addresses that are not
        // the default, the rest keep the base of L2.
        let entry = |core: usize| match core {
            0 => 0x0080_4000,
            3 => 0x0080_8400,
            _ => 0x0080_0000,
        };

        for endian in [Endian::Little, Endian::Big] {
            let mut device = Device::with_boot_mode(&C6472, 0x0080_0000, endian, BOOTMODE_HPI);
            // The port is open before any cycle has run, and every core is held with the cache
            // settings of Table 21: 32 KB of L1P and L1D, and no L2 cache.
            assert_eq!(
                device.fabric().hpi_host_read(HostPort::Control),
                Some(HPIC_HRDY)
            );
            for core in 0..6 {
                assert!(device.held_in_reset(core), "core {core}");
                assert_eq!(device.boot().hpi_entry(core), None);
                device.with_core(core, |cpu| {
                    let mut bus = cpu.bus_mut();
                    assert_eq!(bus.read32(L1PCFG) & 7, 4, "core {core}");
                    assert_eq!(bus.read32(L1DCFG) & 7, 4, "core {core}");
                    assert_eq!(bus.read32(L2CFG) & 7, 0, "core {core}");
                });
            }
            // Unlike the C6455, this host can configure PLL1 before releasing a core
            // (SPRUEC6G section 4.3.2). Do not apply the C6455 exclusion to this chip.
            host_set(&mut device, 0x029a_0110, 7);
            assert_eq!(host_get(&mut device, 0x029a_0110), 7);
            device.with_core(0, |cpu| {
                assert_eq!(cpu.bus_mut().read32(0x029a_0110), 7);
            });
            // SPRS612G section 7.10.3 separately excludes PLL3. It is not yet a
            // register model, but the host must not reach its backing storage either.
            device.with_core(0, |cpu| cpu.bus_mut().write32(0x029c_0500, 0x40));
            host_set(&mut device, 0x029c_0500, 1);
            assert_eq!(host_get(&mut device, 0x029c_0500), 0);
            device.with_core(0, |cpu| {
                assert_eq!(cpu.bus_mut().read32(0x029c_0500), 0x40);
            });

            // Loading all six images through the global aliases runs no core.
            for core in 0..6 {
                host_load(&mut device, global(core, entry(core)), &image(core));
            }
            run(&mut device, 100);
            for core in 0..6 {
                assert!(device.held_in_reset(core), "loading released core {core}");
                assert_eq!(device.core(core).cycles(), 0);
            }
            // What the host wrote is what it reads back, registers included.
            assert_eq!(host_get(&mut device, global(3, 0x0080_8400)), image(3)[0]);
            assert_eq!(host_get(&mut device, BOOT_COMPLETE_STAT), 0);
            // The megamodule's own registers are the CPU's alone (SPRU871K section 5.3.1): the
            // host neither reads core 0's cache configuration nor changes it.
            assert_eq!(host_get(&mut device, L1DCFG), 0);
            host_set(&mut device, L1DCFG, 0);
            device.with_core(0, |cpu| assert_eq!(cpu.bus_mut().read32(L1DCFG) & 7, 4));

            // DSPINT is not a release.
            device
                .fabric()
                .hpi_host_write(HostPort::Control, HPIC_DSPINT);
            run(&mut device, 50);
            assert_eq!(
                device.fabric().hpi_host_read(HostPort::Control),
                Some(HPIC_HRDY | HPIC_DSPINT)
            );
            for core in 0..6 {
                assert!(device.held_in_reset(core), "DSPINT released core {core}");
                assert_eq!(device.boot().hpi_entry(core), None);
                assert_eq!(device.core(core).cycles(), 0);
            }
            // Nor an EDMA3 event: channel 0 is GPINT0 here, and no channel is the HPI's.
            device.with_core(0, |cpu| {
                assert_eq!(cpu.bus_mut().read32(0x02a0_1000), 0, "ER")
            });

            // Address registers alone release nothing either; BC0 and BC3 release those two.
            host_set(&mut device, DSP_BOOT_ADDR0, entry(0) >> 10);
            host_set(&mut device, DSP_BOOT_ADDR0 + 0x20 * 3, entry(3) >> 10);
            assert_eq!(host_get(&mut device, DSP_BOOT_ADDR0), entry(0) >> 10);
            run(&mut device, 10);
            assert!((0..6).all(|core| device.held_in_reset(core)));
            host_set(&mut device, BOOT_COMPLETE_STAT, 0b001001);
            assert_eq!(host_get(&mut device, BOOT_COMPLETE_STAT), 0b001001);
            device.step_cycle();
            for core in [0, 3] {
                assert!(!device.held_in_reset(core), "core {core}");
                assert_eq!(device.core(core).pc(), entry(core), "core {core}");
                assert_eq!(device.core(core).cycles(), 0);
                assert_eq!(device.boot().hpi_entry(core), Some(entry(core)));
            }
            for core in [1, 2, 4, 5] {
                assert!(device.held_in_reset(core), "core {core}");
                assert_eq!(device.boot().hpi_entry(core), None);
            }
            run(&mut device, 300);
            for core in [0, 3] {
                assert_eq!(
                    device.core(core).register('A', 4),
                    result(core),
                    "core {core}"
                );
                assert!(device.core(core).pc() > entry(core) + 32);
            }
            for core in [1, 2, 4, 5] {
                assert!(device.held_in_reset(core), "core {core}");
                assert_eq!(device.core(core).cycles(), 0);
                assert_eq!(device.core(core).register('A', 4), 0);
            }

            // BCn is set-only: writing the same bits again, writing zero, or moving the boot
            // address afterwards is no transition and restarts nothing.
            let progress = [device.core(0).pc(), device.core(3).pc()];
            host_set(&mut device, BOOT_COMPLETE_STAT, 0b001001);
            host_set(&mut device, BOOT_COMPLETE_STAT, 0);
            host_set(&mut device, DSP_BOOT_ADDR0, 0x0080_0000 >> 10);
            host_set(&mut device, BOOT_COMPLETE_STAT, 0b000001);
            device.step_cycle();
            assert!(device.core(0).pc() >= progress[0], "core 0 restarted");
            assert!(device.core(3).pc() >= progress[1], "core 3 restarted");
            assert_eq!(device.boot().hpi_entry(0), Some(entry(0)));
            assert_eq!(device.boot().hpi_entry(3), Some(entry(3)));
            assert_eq!(host_get(&mut device, BOOT_COMPLETE_STAT), 0b001001);

            // The rest go at the default start, and only the edges release anything.
            host_set(&mut device, DSP_BOOT_ADDR0, entry(0) >> 10);
            host_set(&mut device, BOOT_COMPLETE_STAT, 0b111111);
            device.step_cycle();
            for core in [1, 2, 4, 5] {
                assert!(!device.held_in_reset(core), "core {core}");
                assert_eq!(device.core(core).pc(), 0x0080_0000, "core {core}");
                assert_eq!(device.boot().hpi_entry(core), Some(0x0080_0000));
            }
            assert!(device.core(0).pc() >= progress[0], "core 0 restarted");
            assert!(device.core(3).pc() >= progress[1], "core 3 restarted");
            run(&mut device, 300);
            for core in 0..6 {
                assert_eq!(
                    device.core(core).register('A', 4),
                    result(core),
                    "core {core}"
                );
            }
            // The clock stays the device's for as long as the device booted this way.
            assert!(device.boot().holds_core_zero());
        }
    }

    #[test]
    fn a_c6472_has_six_cores_each_reporting_its_own_number() {
        let device = c6472();
        assert_eq!(device.cores(), 6);
        for core in 0..6 {
            // DNUM is control register 0x11 (SPRU871K section 2.9.4).
            assert_eq!(device.core(core).read_control(0x11), core as u32);
        }
        assert_eq!(Device::new(&C6455, 0, Endian::Little).cores(), 1);
    }

    /// Immediate boot releases every core at once; every other boot mode runs core 0 and holds
    /// cores 1-5 until core 0 sets their bit in BOOT_COMPLETE_STAT (SPRS612G section 2.4.1).
    #[test]
    fn immediate_boot_runs_every_core() {
        let mut device = c6472();
        for core in 0..6 {
            assert!(!device.fabric().held_in_reset(core), "core {core} held");
        }
    }

    /// Core 0 gives another core a start address and releases it; the released core begins
    /// executing there, and not before.
    #[test]
    fn core_zero_releases_a_held_core_at_the_address_it_chose() {
        let mut device = c6472();
        // Put cores 1-5 back in reset, which is where a ROM boot mode leaves them.
        for core in 1..6 {
            device.fabric().set_held_in_reset(core, true);
        }
        // A NOP-only program at 0x0080_2000 for core 3 to run, and the address it starts at.
        let start = 0x0080_2000u32;
        device.with_core(3, |cpu| cpu.write_bytes(start, &[0u8; 32]));
        device.with_core(0, |cpu| {
            let mut bus = cpu.bus_mut();
            // DSP_BOOT_ADDR carries the address shifted right by ten (SPRS612G section 3.9.5).
            bus.write32(DSP_BOOT_ADDR0 + 0x20 * 3, start >> 10);
            bus.write32(BOOT_COMPLETE_STAT, 1 << 3);
        });
        let before = device.core(3).cycles();
        device.step_cycle();
        assert!(!device.fabric().held_in_reset(3), "core 3 is still held");
        assert_eq!(
            device.core(3).pc(),
            start,
            "core 3 starts where it was told"
        );
        assert_eq!(
            device.core(3).cycles(),
            before,
            "a core released this cycle has not run yet"
        );
        device.step_cycle();
        assert!(device.core(3).cycles() > before, "core 3 is running");
        // A core that was never released stays put.
        assert!(device.fabric().held_in_reset(4));
    }

    /// A batch of device cycles ends exactly where stepping one device cycle at a time and
    /// asking every core before each cycle would: before the first cycle on which a core out of
    /// reset has halted or any core is ready to fetch the CIO breakpoint, with the handshake's
    /// first-cycle exemption. The cases include a core the boot controller moves onto an armed
    /// address mid-batch, and a held core sitting on one.
    #[test]
    fn a_batch_of_device_cycles_stops_where_a_per_cycle_scan_would() {
        let entry = 0x0080_0000u32;
        let start = 0x0080_2000u32;
        // Cores 1-5 held, core 3 about to be released onto `start`, and each core's halt list.
        let prepare = |armed: &[(usize, u32)]| {
            let mut device = c6472();
            for core in 1..6 {
                device.fabric().set_held_in_reset(core, true);
            }
            device.with_core(3, |cpu| cpu.write_bytes(start, &[0u8; 32]));
            device.with_core(0, |cpu| {
                let mut bus = cpu.bus_mut();
                bus.write32(DSP_BOOT_ADDR0 + 0x20 * 3, start >> 10);
                bus.write32(BOOT_COMPLETE_STAT, 1 << 3);
            });
            for &(core, address) in armed {
                device.with_core(core, |cpu| cpu.set_stop_on_fetch(vec![address]));
            }
            device
        };
        let scanned = |device: &mut Device, cycles: u64, cio: Option<u32>, ignore: bool| {
            let first = device.cycle();
            for step in 0..cycles {
                if device.halted_core().is_some() {
                    break;
                }
                let due = cio.is_some_and(|address| {
                    (0..device.cores()).any(|core| device.core(core).ready_to_fetch_at(address))
                });
                if due && !(step == 0 && ignore) {
                    break;
                }
                device.step_cycle();
            }
            device.cycle() - first
        };
        // Each case: the halt lists armed, the CIO breakpoint, the handshake's first-cycle
        // exemption, and where the batch ends.
        type Case<'a> = (&'a [(usize, u32)], Option<u32>, bool, u64);
        let cases: &[Case] = &[
            // An already-halted running core prevents any cycle, even with CIO exempted.
            (&[(0, entry)], Some(entry), true, 0),
            // Core 3 is released onto its armed address after the first cycle.
            (&[(3, start)], None, false, 1),
            // A held core on an armed address is not a halt.
            (&[(4, entry)], None, false, 40),
            // Held cores sit on the entry point, so the CIO boundary is due at once...
            (&[], Some(entry), false, 0),
            // ...and still due after the exempt first cycle.
            (&[], Some(entry), true, 1),
            // The release moves core 3 onto the CIO breakpoint.
            (&[], Some(start), false, 1),
        ];
        for &(armed, cio, ignore, expected) in cases {
            let mut batched = prepare(armed);
            let mut stepped = prepare(armed);
            let advanced = batched.run_cycles(40, cio, ignore);
            assert_eq!(advanced, scanned(&mut stepped, 40, cio, ignore));
            assert_eq!(advanced, expected, "{armed:?} {cio:?} {ignore}");
            for core in 0..6 {
                assert_eq!(batched.core(core).pc(), stepped.core(core).pc());
                assert_eq!(batched.core(core).cycles(), stepped.core(core).cycles());
            }
            assert_eq!(batched.halted_core(), stepped.halted_core());
        }
    }

    #[test]
    fn single_core_batches_report_existing_boundaries_without_advancing() {
        let entry = 0x0080_0000;
        let mut device = Device::new(&C6455, entry, Endian::Little);
        device.cpu_mut().write_words(entry, &[0, 0x5645_4314]);
        assert_eq!(device.run_cycles(0, None, false), 0);
        assert_eq!(device.run_cycles(4, Some(entry), false), 0);
        assert_eq!(device.cpu().cycles(), 0);
        assert_eq!(device.cpu().pc(), entry);

        device.cpu_mut().set_stop_on_fetch(vec![entry]);
        assert_eq!(device.run_cycles(4, Some(entry), true), 0);
        assert!(device.halted_core().is_some());
        assert_eq!(device.cpu().cycles(), 0);

        device.cpu_mut().set_stop_on_fetch(Vec::new());
        assert_eq!(device.run_cycles(1, Some(entry), true), 1);
        assert_eq!(device.cpu().pc(), entry + 4);
        assert!(device.run_cycles(4, None, false) > 0);
        assert!(device.cpu().fault().is_some());
        let cycles = device.cpu().cycles();
        let pc = device.cpu().pc();
        assert_eq!(device.run_cycles(4, None, false), 0);
        assert_eq!(device.cpu().cycles(), cycles);
        assert_eq!(device.cpu().pc(), pc);
    }

    /// `LDW *A0, A1` and then nothing but NOPs, in every core's L2, with strict physical-memory
    /// checks on and A0 pointing each core at its own L2 - except core `bad`, which reads
    /// `address`.
    fn strict_c6472_loading(bad: usize, address: u32) -> Device {
        let mut device = c6472();
        device.fabric().set_strict_physical_memory(true);
        let mut code = 0x0080_0264u32.to_le_bytes().to_vec();
        code.resize(1024, 0);
        device.load_image(0x0080_0000, &code);
        for core in 0..6 {
            let pointer = if core == bad { address } else { 0x0080_1000 };
            device.with_core(core, |cpu| cpu.set_register("A0", pointer).unwrap());
        }
        device
    }

    /// A strict violation by one core's load ends the batch after the device cycle it was made
    /// in - every core finishes that cycle, none starts another - and becomes that core's
    /// fault, naming the access and where the core stopped. The load never writes back.
    #[test]
    fn a_strict_violation_stops_the_device_on_the_core_that_made_it() {
        let mut device = strict_c6472_loading(3, 0x0089_8000);
        assert_eq!(device.run_cycles(100, None, false), 1);
        let violation = device.strict_violation().unwrap();
        assert_eq!((violation.core(), violation.reported_core), (Some(3), 3));
        for core in 0..6 {
            assert_eq!(device.core(core).cycles(), 1, "core {core}");
            if core != 3 {
                assert_eq!(device.core(core).fault(), None, "core {core}");
            }
        }
        assert_eq!(
            device.core(3).fault(),
            Some(
                "strict physical-memory: read of 4 bytes by Core(3) at 0x00898000 enters a \
                 reserved, unsupported, or cache-converted range at 0x00898000 (reported on \
                 core 3, the core that made the access; stopped at PC 0x00800004, cycle 1)"
            )
        );
        // The focused core is still core 0; it does not have the fault.
        assert_eq!(device.focus(), 0);
        assert_eq!(device.cpu().fault(), None);
        // Nothing runs after it, however the device is driven.
        assert_eq!(device.run_cycles(100, None, false), 0);
        device.step_cycle();
        for core in 0..6 {
            assert_eq!(device.core(core).cycles(), 1, "core {core}");
        }
        assert_eq!(device.core(3).register('A', 1), 0);
        // No exception and no IERR bit: the diagnostic is the emulator's.
        assert_eq!(device.core(3).snapshot().control[0x1f], 0);
    }

    /// The first violation is the one reported, even when a later core makes another in the
    /// same cycle.
    #[test]
    fn the_first_strict_violation_is_the_one_reported() {
        let mut device = strict_c6472_loading(1, 0x0000_0000);
        device.with_core(4, |cpu| cpu.set_register("A0", 0x0089_8000).unwrap());
        device.run_cycles(100, None, false);
        let violation = device.strict_violation().unwrap();
        assert_eq!(violation.reported_core, 1);
        assert_eq!(violation.start, 0);
        assert!(device.core(1).fault().is_some());
        assert_eq!(device.core(4).fault(), None);
    }

    /// A system master has no core of its own, so its violation is reported on the focused
    /// core, saying that this core is only where the program was.
    #[test]
    fn a_masters_strict_violation_is_reported_on_the_focused_core() {
        let mut device = strict_c6472_loading(usize::MAX, 0);
        device.set_focus(2);
        assert_eq!(
            device
                .fabric()
                .read(Privilege::ID0_SUPERVISOR, 0x0080_0000, 4),
            Some(vec![0; 4])
        );
        assert_eq!(device.run_cycles(100, None, false), 0);
        assert_eq!(device.strict_violation().map(|v| v.reported_core), Some(2));
        assert_eq!(device.core(2).fault(), Some(MASTER_FAULT_ON_CORE_2));
    }

    const MASTER_FAULT_ON_CORE_2: &str = "strict physical-memory: DMA read of 4 bytes by Dma at \
        0x00800000 enters a reserved, unsupported, or cache-converted range at 0x00800000 \
        (reported on core 2, the focused core; a system bus master made the access, not this \
        core; stopped at PC 0x00800000, cycle 0)";

    /// "Focused" means focused when the master made the access: moving the focus between the
    /// access and the run loop noticing it, or after the fault is latched, moves nothing, and
    /// latching again never hands the violation to a second core.
    #[test]
    fn a_masters_strict_violation_stays_on_the_core_focused_at_the_access() {
        for refocus_before_latch in [false, true] {
            let mut device = strict_c6472_loading(usize::MAX, 0);
            device.set_focus(2);
            device
                .fabric()
                .read(Privilege::ID0_SUPERVISOR, 0x0080_0000, 4);
            if refocus_before_latch {
                device.set_focus(4);
            }
            assert_eq!(device.run_cycles(100, None, false), 0);
            device.set_focus(4);
            device.latch_strict_violation();
            device.step_cycle();
            device.set_focus(0);
            device.latch_strict_violation();
            let context = format!("refocus before latch {refocus_before_latch}");
            let violation = device.strict_violation().unwrap();
            assert_eq!(violation.reported_core, 2, "{context}");
            assert_eq!(violation.stopped, Some((0x0080_0000, 0)), "{context}");
            assert_eq!(
                device.core(2).fault(),
                Some(MASTER_FAULT_ON_CORE_2),
                "{context}"
            );
            for core in [0, 1, 3, 4, 5] {
                assert_eq!(device.core(core).fault(), None, "{context} core {core}");
            }
        }
    }

    /// A single core runs its own batches; a violation stops it the same way.
    #[test]
    fn a_strict_violation_stops_a_single_core_batch() {
        let entry = 0x0080_0000;
        let mut device = Device::new(&C6455, entry, Endian::Little);
        device.fabric().set_strict_physical_memory(true);
        let mut code = 0x0080_0274u32.to_le_bytes().to_vec();
        code.resize(1024, 0);
        device.load_image(entry, &code);
        device.cpu_mut().set_register("A0", 0x00a0_0000).unwrap();
        device.cpu_mut().set_register("A1", 0x1234_5678).unwrap();
        let ran = device.run_cycles(100, None, false);
        let fault = device.cpu().fault().expect("a strict fault").to_owned();
        assert!(
            fault.starts_with(
                "strict physical-memory: write of 4 bytes by Core(0) at 0x00a00000 enters"
            ),
            "{fault}"
        );
        assert_eq!(device.run_cycles(100, None, false), 0);
        assert_eq!(device.cpu().cycles(), ran);
        assert_eq!(device.fabric().host_read_bytes(0, 0x00a0_0000, 4), [0; 4]);
    }

    /// A held core executes nothing, but its memories still answer: that is how a host, or
    /// core 0, loads a program for it (SPRUEC6G section 4.3.2).
    #[test]
    fn a_held_core_does_not_execute_but_its_memory_is_reachable() {
        let mut device = c6472();
        device.fabric().set_held_in_reset(2, true);
        let before = device.core(2).cycles();
        for _ in 0..8 {
            device.step_cycle();
        }
        assert_eq!(device.core(2).cycles(), before);
        // Core 0 writes into core 2's L2 SRAM through the global alias 1280 0000h.
        device.with_core(0, |cpu| {
            cpu.bus_mut().write32(0x1280_0100, 0xfeed_face);
        });
        device.with_core(2, |cpu| {
            assert_eq!(cpu.bus_mut().read32(0x0080_0100), 0xfeed_face);
        });
    }

    /// An inter-processor interrupt is delivered to the core the register names, and to no
    /// other core (SPRS612G section 3.7.2, Table 7-10 event 84).
    #[test]
    fn an_ipc_interrupt_reaches_only_its_destination_core() {
        let mut device = c6472();
        device.with_core(0, |cpu| {
            // Source flag 0 plus the generate bit, addressed at core 3.
            cpu.bus_mut().write32(IPCGR0 + 4 * 3, (1 << 4) | 1);
        });
        device.step_cycle();
        for core in 0..6 {
            let flags = device.with_core(core, |cpu| cpu.bus_mut().read32(EVTFLAG2));
            let raised = flags & (1 << (84 - 64)) != 0;
            assert_eq!(raised, core == 3, "event 84 on core {core}");
        }
        // The destination reads which source interrupted it and acknowledges that source.
        device.with_core(3, |cpu| {
            let mut bus = cpu.bus_mut();
            assert_eq!(bus.read32(IPCGR0 + 4 * 3), 1 << 4);
            bus.write32(IPCAR0 + 4 * 3, 1 << 4);
            assert_eq!(bus.read32(IPCGR0 + 4 * 3), 0);
        });
    }

    /// The shared L2 is one memory every core reaches through its own local window, while each
    /// core's own L2 is private until another core uses the global alias (SPRS612G Table 2-2).
    #[test]
    fn shared_memory_is_common_and_local_memory_is_not() {
        let mut device = c6472();
        device.with_core(1, |cpu| {
            cpu.bus_mut().write32(0x0020_0040, 0x1234_5678); // shared L2 RAM
            cpu.bus_mut().write32(0x0080_0040, 0x1111_1111); // core 1's own L2
        });
        device.with_core(5, |cpu| {
            let mut bus = cpu.bus_mut();
            assert_eq!(bus.read32(0x0020_0040), 0x1234_5678, "shared L2 is shared");
            assert_eq!(bus.read32(0x0080_0040), 0, "local L2 is private");
            assert_eq!(
                bus.read32(0x1180_0040),
                0x1111_1111,
                "core 1's L2 through its global alias"
            );
            // Every core's alias of the shared RAM is the same memory.
            assert_eq!(bus.read32(0x1520_0040), 0x1234_5678);
            assert_eq!(bus.read32(0x1020_0040), 0x1234_5678);
        });
    }

    /// The C6455 keeps exactly one core and no global aliases.
    #[test]
    fn the_single_core_device_is_unchanged() {
        let mut device = Device::new(&C6455, 0x0080_0000, Endian::Little);
        assert_eq!(device.cores(), 1);
        device.with_core(0, |cpu| {
            cpu.bus_mut().write32(0x0080_0040, 0xabcd_ef01);
            assert_eq!(cpu.bus_mut().read32(0x0080_0040), 0xabcd_ef01);
            // 2 MB of L2 SRAM, which the C6472 does not have.
            cpu.bus_mut().write32(0x009f_0000, 0x5555_aaaa);
            assert_eq!(cpu.bus_mut().read32(0x009f_0000), 0x5555_aaaa);
        });
    }
    /// A megamodule's local timer runs on that megamodule's clock, so gating the megamodule stops
    /// the timer with it - while a shared timer, which belongs to no megamodule, keeps counting.
    ///
    /// SPRUEG3B section 4.1: "C64x+ Megamodule local timers are clock gated as well when the
    /// corresponding C64x+ Megamodule is clock gated"; section 4.1.2 has a module outside Enable
    /// gate its functional clocks. SPRS612G section 7.16: timers 0-5 are the megamodules' and
    /// timers 6-11 are shared, all of them counting the internal clock at CPU/6.
    #[test]
    fn gating_a_megamodule_stops_its_local_timer_and_not_a_shared_one() {
        const TIMER0: u32 = 0x025e_0000;
        const TIMER1: u32 = 0x025f_0000;
        const TIMER6: u32 = 0x0264_0000;
        const CNTLO: u32 = 0x10;
        const PRDLO: u32 = 0x18;
        const PRDHI: u32 = 0x1c;
        const TCR: u32 = 0x20;
        const TGCR: u32 = 0x24;
        // MDCTL1 is the second megamodule's module register (SPRS612G Table 7-3); LRSTZ is bit 8
        // and NEXT is 4:0, so 103h is Enable and 102h is Disable with reset left de-asserted
        // (SPRUEG3B Tables 5 and 13, section 4.1.2).
        const MDCTL1: u32 = 0x02ae_0a00 + 4;
        const PTCMD: u32 = 0x02ae_0120;
        const ENABLE: u32 = 0x103;
        const DISABLE: u32 = 0x102;
        let mut device = c6472();

        // Three free-running 64-bit counters: TIMMODE 00b with both halves out of reset (SPRU818B
        // Table 13), period out of reach, and ENAMODE_LO 10b to count continuously (Table 12).
        for base in [TIMER0, TIMER1, TIMER6] {
            device.with_core(0, |cpu| {
                let mut bus = cpu.bus_mut();
                bus.write32(base + CNTLO, 0);
                bus.write32(base + 0x14, 0);
                bus.write32(base + PRDHI, 0xffff_ffff);
                bus.write32(base + PRDLO, 0xffff_ffff);
                bus.write32(base + PRDHI, 0xffff_ffff);
                bus.write32(base + TGCR, 0x3);
                bus.write32(base + TCR, 0x80);
            });
        }

        let count = |device: &mut Device, base: u32| {
            device.with_core(0, |cpu| cpu.bus_mut().read32(base + CNTLO))
        };
        for _ in 0..6 * 32 {
            device.step_cycle();
        }
        let (t0, t1, t6) = (
            count(&mut device, TIMER0),
            count(&mut device, TIMER1),
            count(&mut device, TIMER6),
        );
        assert!(
            t0 > 0 && t1 > 0 && t6 > 0,
            "the three timers started: {t0} {t1} {t6}"
        );

        // Gate megamodule 1's clock without asserting its local reset, and let the domain's GO
        // complete: the controller acknowledges one device cycle after the GO. The baseline is
        // taken after that, so what the assertion below measures is the gate and not the
        // transition's own cycle.
        device.with_core(0, |cpu| {
            let mut bus = cpu.bus_mut();
            bus.write32(MDCTL1, DISABLE);
            bus.write32(PTCMD, 1);
        });
        device.step_cycle();
        device.step_cycle();
        assert!(!device.fabric().core_clocked(1), "megamodule 1 is gated");
        let t1 = count(&mut device, TIMER1);

        for _ in 0..6 * 32 {
            device.step_cycle();
        }
        assert_eq!(
            count(&mut device, TIMER1),
            t1,
            "the gated megamodule's timer stopped"
        );
        assert!(
            count(&mut device, TIMER0) > t0,
            "the boot core's timer kept counting"
        );
        assert!(
            count(&mut device, TIMER6) > t6,
            "a shared timer kept counting"
        );

        // And it is a gate, not a stop: enabling the module again lets the timer count on.
        let gated = count(&mut device, TIMER1);

        device.with_core(0, |cpu| {
            let mut bus = cpu.bus_mut();
            bus.write32(MDCTL1, ENABLE);
            bus.write32(PTCMD, 1);
        });
        for _ in 0..6 * 32 {
            device.step_cycle();
        }
        assert!(count(&mut device, TIMER1) > gated, "the timer counts again");
    }

    /// SPRUEC6G section 4.3.4: the UTOPIA boot of the C6472, driven the way an external ATM
    /// master drives it - through the receive queue, one cell at a time, with the ROM taking the
    /// cells off the PDMA's buffer when an interrupt wakes it.
    mod utopia_boot {
        use super::*;
        use crate::utopiaboot::Progress as UtopiaProgress;

        const PHY: u8 = 0x15;
        const DEVSTAT: u32 = 0x02a8_0000;
        const UCR: u32 = 0x02b4_0000;
        const UCR_BEND: u32 = 1 << 31;
        const UCR_SLID: u32 = 0x1f << 24;
        const UCR_MPHY: u32 = 1 << 14;
        const UCR_U16M: u32 = 1 << 13;
        const UCR_UREN: u32 = 1 << 0;
        const PDMA_RXC0: u32 = 0x02bc_0180;
        const PDMA_TCR: u32 = 0xc;
        const PLLM: u32 = 0x029a_0110;
        const TIMER0: u32 = 0x025e_0000;
        const DSP_BOOT_ADDR1: u32 = DSP_BOOT_ADDR0 + 0x20;
        /// MVK .L1 k,A4 with k in bits 22-18, and ADD .L1 1,A4,A4.
        const MVK_A4: u32 = 0x0200_a358;
        const ADD_A4: u32 = 0x0210_2058;
        const MAGIC: [u8; 4] = [0x54, 0x49, 0x21, 0x64];
        /// Mode, 16-bit port, PLL multiplier (SPRUEC6G Table 22).
        const MODES: [(u8, bool, u32); 4] =
            [(5, false, 10), (6, false, 20), (7, true, 10), (8, true, 20)];
        const CODE: u32 = 0x0080_1000;
        const DATA: u32 = 0x0081_0000;
        const CORE1_CODE: u32 = 0x0080_2400;

        fn label(endian: Endian, mode: u8) -> String {
            let order = if endian == Endian::Big {
                "big"
            } else {
                "little"
            };
            format!("{order}-endian mode {mode}")
        }

        fn boot_device(endian: Endian, mode: u8) -> Device {
            Device::on_board(
                &Board::pins(&C6472, mode).with_utopia_phy(PHY),
                0x0080_0000,
                endian,
            )
        }

        /// Table words as the hex conversion utility's `-order M` writes them, most significant
        /// byte first: with the ROM's big-endian cell assembly that is how a table reaches L1D
        /// in the device's own byte order, whichever that is.
        fn wire(words: &[u32]) -> Vec<u8> {
            words.iter().flat_map(|word| word.to_be_bytes()).collect()
        }

        /// A boot table that loads core 0's three-instruction program, a 40-byte data section,
        /// core 1's two-instruction program through its global alias, and then names core 1's
        /// start address and sets its boot-complete bit, which is what starts it. Core 0 ends
        /// with 7 in A4 and core 1 with 8.
        fn full_table() -> Vec<u8> {
            let data: Vec<u32> = (0..10).map(|index| 0x0101_0101u32 * (index + 1)).collect();
            let mut words = vec![CODE, 12, CODE, MVK_A4 | (5 << 18), ADD_A4, ADD_A4];
            words.extend([40, DATA]);
            words.extend(&data);
            words.extend([8, 0x1000_0000 + (1 << 24) + CORE1_CODE]);
            words.extend([MVK_A4 | (7 << 18), ADD_A4]);
            words.extend([4, DSP_BOOT_ADDR1, CORE1_CODE >> 10]);
            words.extend([4, BOOT_COMPLETE_STAT, 0b11]);
            words.push(0);
            wire(&words)
        }

        /// A table that only loads core 0's program, which fits in one cell.
        fn short_table() -> Vec<u8> {
            wire(&[CODE, 12, CODE, MVK_A4 | (5 << 18), ADD_A4, ADD_A4, 0])
        }

        /// What an ATM master puts on the wire: the header, 32 bytes of table, the magic and
        /// the 12 bytes of payload after it that complete the 48 (SPRUEG2E section 2).
        fn cell(wide: bool, table: &[u8], magic: [u8; 4]) -> Vec<u8> {
            sized_cell(if wide { 54 } else { 53 }, table, magic)
        }

        /// A cell of `bytes` on the wire: everything in front of the 48-byte payload is header,
        /// the standard cell's and any user-defined bytes.
        fn sized_cell(bytes: usize, table: &[u8], magic: [u8; 4]) -> Vec<u8> {
            let mut cell: Vec<u8> = (1..=(bytes - 48) as u8).collect();
            let mut chunk = table.to_vec();
            chunk.resize(32, 0);
            cell.extend(chunk);
            cell.extend(magic);
            cell.extend([0x5a; 12]);
            assert_eq!(cell.len(), bytes);
            cell
        }

        fn table_cells(wide: bool, table: &[u8]) -> Vec<Vec<u8>> {
            table
                .chunks(32)
                .map(|chunk| cell(wide, chunk, MAGIC))
                .collect()
        }

        /// Cells that are not boot cells: no magic, a different magic, and the magic the wrong
        /// way round.
        fn junk_cell(wide: bool, kind: usize) -> Vec<u8> {
            let magic = [[0; 4], [0x54, 0x49, 0x21, 0x65], [0x64, 0x21, 0x49, 0x54]][kind % 3];
            cell(wide, &[0xee; 32], magic)
        }

        fn send(device: &mut Device, cell: &[u8]) {
            send_to(device, PHY, cell);
        }

        fn send_to(device: &mut Device, phy: u8, cell: &[u8]) {
            for _ in 0..200_000 {
                if device.fabric().utopia_master_poll_receive(phy) == Some(true) {
                    assert!(device.fabric().utopia_master_send_cell(phy, cell));
                    return;
                }
                device.step_cycle();
            }
            panic!("the receive queue never had room for a cell");
        }

        fn boot_until_done(device: &mut Device) -> UtopiaProgress {
            for _ in 0..600_000 {
                if let Some(progress) = device
                    .boot()
                    .utopia()
                    .filter(|progress| progress.entry.is_some() || progress.error.is_some())
                {
                    return progress;
                }
                device.step_cycle();
            }
            panic!("the boot never ended: {:?}", device.boot().utopia());
        }

        fn read(device: &mut Device, address: u32) -> u32 {
            device.with_core(0, |cpu| cpu.bus_mut().read32(address))
        }

        /// Whether a receive channel of the PDMA is enabled, read back through its proxy block:
        /// a read of the switch address register loads the proxies from the live context.
        fn channel_enabled(device: &mut Device, channel: u32) -> bool {
            let proxy = PDMA_RXC0 + 0x10 * channel;
            device.with_core(0, |cpu| {
                let mut bus = cpu.bus_mut();
                bus.read32(proxy);
                bus.read32(proxy + PDMA_TCR) >> 31 != 0
            })
        }

        #[test]
        fn a_table_spanning_cells_boots_and_runs_in_every_mode_and_byte_order() {
            for endian in [Endian::Little, Endian::Big] {
                for (mode, wide, multiplier) in MODES {
                    let label = label(endian, mode);
                    let mut device = boot_device(endian, mode);
                    run(&mut device, 200);

                    // The ROM holds core 0 while it runs in its place, and the other cores
                    // wait for the table to release them. The pins say which boot this is.
                    for core in 0..6 {
                        assert!(device.held_in_reset(core), "{label}: core {core}");
                    }
                    let devstat = read(&mut device, DEVSTAT);
                    assert_eq!(devstat & 0xf, u32::from(mode), "{label}: BOOTMODE");
                    assert_eq!((devstat >> 17) & 0x1f, u32::from(PHY), "{label}: CFGGP");

                    // What the ROM programmed, read back through the registers it used: Table
                    // 21's cache settings, the mode's PLL multiplier, a big-endian multi-PHY
                    // receiver at the PHY ID and the width the mode names, the PDMA channel
                    // and the timer.
                    assert_eq!(read(&mut device, L1PCFG) & 7, 4, "{label}: L1P");
                    assert_eq!(read(&mut device, L1DCFG) & 7, 3, "{label}: L1D");
                    assert_eq!(read(&mut device, L2CFG) & 7, 0, "{label}: L2 cache is off");
                    assert_eq!(read(&mut device, PLLM) & 0x3f, multiplier - 1, "{label}");
                    let mask = UCR_BEND | UCR_SLID | UCR_MPHY | UCR_U16M | UCR_UREN;
                    let expected = UCR_BEND
                        | (u32::from(PHY) << 24)
                        | UCR_MPHY
                        | UCR_UREN
                        | if wide { UCR_U16M } else { 0 };
                    assert_eq!(read(&mut device, UCR) & mask, expected, "{label}: UCR");
                    assert!(channel_enabled(&mut device, 0), "{label}: PDMA channel 8");
                    let period = if wide { 4166 } else { 8333 };
                    assert_eq!(read(&mut device, TIMER0 + 0x18) + 1, period, "{label}");

                    // Only the PHY the pins name answers, and a cell sent to another is lost.
                    assert_eq!(
                        device.fabric().utopia_master_poll_receive(PHY),
                        Some(true),
                        "{label}"
                    );
                    for other in [PHY ^ 1, 0x1f] {
                        assert_eq!(
                            device.fabric().utopia_master_poll_receive(other),
                            None,
                            "{label}: PHY {other}"
                        );
                        let stray = cell(wide, &[0; 32], MAGIC);
                        assert!(!device.fabric().utopia_master_send_cell(other, &stray));
                    }

                    // The table is 116 bytes: four cells, with sections that straddle them.
                    let table = full_table();
                    assert_eq!(table.len(), 116);
                    for cell in table_cells(wide, &table) {
                        send(&mut device, &cell);
                    }
                    // The ROM is asleep until something wakes it: four cells reach neither half
                    // of the buffer, so it is the timer at its documented cadence.
                    let sent = device.cycle();
                    assert!(device.held_in_reset(0), "{label}: core 0 runs the ROM");
                    let progress = boot_until_done(&mut device);
                    assert_eq!(progress.error, None, "{label}");
                    assert_eq!(progress.entry, Some(CODE), "{label}");
                    assert_eq!(progress.accepted, 4, "{label}");
                    assert_eq!(progress.discarded, 0, "{label}");
                    assert_eq!(progress.table_bytes, 128, "{label}");
                    let ended = device.cycle();
                    // The counter starts from zero and the first match comes PRD timer clocks
                    // later; after that it restarts from zero the clock after each match, so the
                    // steady period is PRD + 1 clocks, the 100 us (50 us) the manual asks for
                    // (SPRU968 Table 12). Each timer clock is six device cycles.
                    let first_match = 6 * u64::from(period - 1);
                    assert!(sent < first_match, "{label}: the cells arrived in time");
                    assert!(ended >= first_match, "{label}: woke after {ended}");
                    assert!(ended < first_match + 600, "{label}: woke after {ended}");

                    // The ROM ended the way the manual says: every receive channel and the
                    // timer disabled, and core 0 released at the entry point.
                    for channel in 0..6 {
                        assert!(
                            !channel_enabled(&mut device, channel),
                            "{label}: PDMA channel {}",
                            8 + channel
                        );
                    }
                    assert_eq!(read(&mut device, TIMER0 + 0x20), 0, "{label}: TCR");
                    assert_eq!(read(&mut device, TIMER0 + 0x24), 0, "{label}: TGCR");
                    assert!(!device.held_in_reset(0), "{label}");
                    assert_eq!(device.core(0).pc(), CODE, "{label}");

                    // The table's sections landed, and its release started core 1 alone.
                    for index in 0..10u32 {
                        assert_eq!(
                            read(&mut device, DATA + 4 * index),
                            0x0101_0101 * (index + 1),
                            "{label}: data word {index}"
                        );
                    }
                    run(&mut device, 400);
                    assert_eq!(device.core(0).register('A', 4), 7, "{label}: core 0");
                    assert!(device.core(0).pc() >= CODE + 12, "{label}");
                    assert!(!device.held_in_reset(1), "{label}: core 1");
                    assert_eq!(device.core(1).register('A', 4), 8, "{label}: core 1");
                    for core in 2..6 {
                        assert!(device.held_in_reset(core), "{label}: core {core}");
                        assert_eq!(device.core(core).cycles(), 0, "{label}: core {core}");
                    }
                    // Nothing is left latched for the application to find as an interrupt.
                    assert_eq!(read(&mut device, 0x0180_0000) & ((1 << 16) | (1 << 30)), 0);
                }
            }
        }

        #[test]
        fn cells_without_the_magic_are_discarded_and_counted_wherever_they_fall() {
            for endian in [Endian::Little, Endian::Big] {
                for (mode, wide) in [(5, false), (8, true)] {
                    let label = label(endian, mode);
                    let mut device = boot_device(endian, mode);
                    let cells = table_cells(wide, &full_table());
                    assert_eq!(cells.len(), 4);
                    // Junk before the first table cell, between the others and at the head of
                    // the last, in every flavour of wrong.
                    let mut junk = 0;
                    for (index, table_cell) in cells.iter().enumerate() {
                        if index != 2 {
                            send(&mut device, &junk_cell(wide, junk));
                            junk += 1;
                        }
                        send(&mut device, table_cell);
                    }
                    assert_eq!(junk, 3);
                    let progress = boot_until_done(&mut device);
                    assert_eq!(progress.error, None, "{label}");
                    assert_eq!(progress.accepted, 4, "{label}");
                    assert_eq!(progress.discarded, 3, "{label}");
                    assert_eq!(progress.table_bytes, 128, "{label}");
                    assert_eq!(progress.entry, Some(CODE), "{label}");
                    // Junk contributed nothing to the table, so the program is the one sent.
                    run(&mut device, 400);
                    assert_eq!(device.core(0).register('A', 4), 7, "{label}");
                    assert_eq!(device.core(1).register('A', 4), 8, "{label}");
                }
            }
        }

        #[test]
        fn the_half_buffer_interrupt_wakes_the_rom_and_the_buffer_wraps() {
            for endian in [Endian::Little, Endian::Big] {
                let label = label(endian, 7);
                let mut device = boot_device(endian, 7);
                // 292 cells fit in the 16 KB buffer, and the PDMA interrupts at 146 and 292.
                // 300 cells that are not boot cells push the write position through both marks
                // and past the wrap, and the boot cell after them lands near the buffer's start.
                for index in 0..300 {
                    send(&mut device, &junk_cell(true, index));
                }
                for cell in table_cells(true, &short_table()) {
                    send(&mut device, &cell);
                }
                let progress = boot_until_done(&mut device);
                assert_eq!(progress.error, None, "{label}");
                assert_eq!(progress.discarded, 300, "{label}");
                assert_eq!(progress.accepted, 1, "{label}");
                assert_eq!(progress.entry, Some(CODE), "{label}");
                run(&mut device, 400);
                assert_eq!(device.core(0).register('A', 4), 7, "{label}");
                assert!(device.held_in_reset(1), "{label}: no table released core 1");
            }
        }

        #[test]
        fn the_full_buffer_interrupt_wakes_the_rom_before_the_timer() {
            // The boot cell is the 292nd packet, the one that fills the buffer: the PDMA's
            // end-of-buffer interrupt has to wake the ROM, because it is sent just after a
            // timer match and the next one is almost 50 us away.
            let mut device = boot_device(Endian::Little, 7);
            for index in 0..291 {
                send(&mut device, &junk_cell(true, index));
            }
            let period = read(&mut device, TIMER0 + 0x18) + 1;
            for _ in 0..200_000 {
                if read(&mut device, TIMER0 + 0x10) < 8 {
                    break;
                }
                device.step_cycle();
            }
            assert!(
                read(&mut device, TIMER0 + 0x10) < 8,
                "the timer never restarted"
            );
            let cells = table_cells(true, &short_table());
            assert_eq!(cells.len(), 1);
            send(&mut device, &cells[0]);
            let sent = device.cycle();
            let progress = boot_until_done(&mut device);
            assert_eq!(progress.entry, Some(CODE));
            assert_eq!(progress.discarded, 291);
            let woke = device.cycle() - sent;
            assert!(
                woke < 6 * u64::from(period) / 4,
                "woke {woke} cycles after the last cell, the timer's turn"
            );
        }

        #[test]
        fn the_boot_stops_without_an_entry_point_when_the_table_never_ends() {
            let mut device = boot_device(Endian::Little, 5);
            // An entry point and then sections that never end: the stream refuses the table
            // once it has more sections than any real image, and the core stays held.
            let mut table = wire(&[CODE]);
            for _ in 0..4097 {
                table.extend(wire(&[4, DATA, 0xa5a5_a5a5]));
            }
            for cell in table_cells(false, &table) {
                send(&mut device, &cell);
            }
            let progress = boot_until_done(&mut device);
            assert_eq!(progress.entry, None);
            assert!(progress.error.is_some());
            assert!(
                device.held_in_reset(0),
                "the core is not given a bogus entry"
            );
        }

        /// The handover of SPRUEC6G section 4.3.3 and Tables 23, 24 and 26: an I2C boot (mode
        /// 2, the EEPROM at 0x50) reads a parameter block that names another, and that one names
        /// UTOPIA (extended boot mode 0x104) with its own fields.
        mod from_i2c {
            use super::*;
            use crate::i2cboot::tests::{blocks, c6472_i2c_parameters, c6472_parameters, words};

            const UCR_RUDC: u32 = 0xf << 2;
            const PDMA_BSR: u32 = 0x8;
            const OPTION_SINGLE_PHY: u16 = 1;
            const OPTION_WIDE: u16 = 2;
            const OPTION_SKIP_PORT_INIT: u16 = 4;

            /// Table 26: options, cell size, bus width, slave ID, core frequency.
            fn utopia_block(options: u16, cell: u16, width: u16, slave: u16, mhz: u16) -> Vec<u8> {
                c6472_parameters(
                    0x104,
                    &[
                        (10, options),
                        (12, cell),
                        (14, width),
                        (16, slave),
                        (18, mhz),
                    ],
                )
            }

            /// A C6472 strapped to I2C boot from 0x50 with CFG[4] high, so PLL1 is x19, booting
            /// the EEPROM `image`.
            fn i2c_device(endian: Endian, image: Vec<u8>) -> Device {
                Device::on_board(
                    &Board::pins(&C6472, 2).with_i2c_eeprom(0x10, image),
                    0x0080_0000,
                    endian,
                )
            }

            fn until_listening(device: &mut Device, phy: u8) {
                for _ in 0..600_000 {
                    if device.fabric().utopia_master_poll_receive(phy) == Some(true) {
                        return;
                    }
                    device.step_cycle();
                }
                panic!(
                    "the port never listened at PHY {phy}: {:?}",
                    device.boot().utopia()
                );
            }

            /// The table's five fields each reach the hardware: the PHY mode and slave ID, the
            /// bus width, a cell size other than the standard one - which changes the packet,
            /// the PDMA burst and how many cells the 16 KB buffer holds - and the core frequency
            /// the timer is counted from. PLL1 stays at the I2C boot's x19. Enough junk cells go
            /// first to carry the PDMA's write position round the shorter buffer, so the ROM
            /// only finds the table if it and the PDMA agree where every packet starts.
            #[test]
            fn the_parameter_table_configures_the_utopia_boot_it_hands_over_to() {
                // Options, cell, width, slave, MHz, buffered cells.
                let cases: [(u16, u16, u16, u8, u32, u32); 2] = [
                    (0, 64, 8, 0x0b, 250, 256),
                    (OPTION_SINGLE_PHY | OPTION_WIDE, 60, 16, 0x03, 1000, 273),
                ];
                for endian in [Endian::Little, Endian::Big] {
                    for (options, cell_bytes, width, slave, mhz, cells) in cases {
                        let wide = width == 16;
                        let single = options & OPTION_SINGLE_PHY != 0;
                        let label = format!(
                            "big-endian {} {width}-bit {cell_bytes}-byte cells",
                            endian == Endian::Big
                        );
                        let mut image = c6472_i2c_parameters(0, 0x80, 0x50, (0, 0));
                        image.extend(utopia_block(
                            options,
                            cell_bytes,
                            width,
                            u16::from(slave),
                            mhz as u16,
                        ));
                        let mut device = i2c_device(endian, image);
                        until_listening(&mut device, slave);

                        let i2c = device.boot().i2c().unwrap();
                        assert_eq!(i2c.error, None, "{label}");
                        assert_eq!(i2c.parameter_blocks, 2, "{label}");
                        assert_eq!(i2c.entry, None, "{label}: UTOPIA's boot, not I2C's");
                        let extra = u32::from(cell_bytes) - if wide { 54 } else { 53 };
                        let mask = UCR_BEND | UCR_SLID | UCR_MPHY | UCR_U16M | UCR_RUDC | UCR_UREN;
                        let expected = UCR_BEND
                            | (u32::from(slave) << 24)
                            | if single { 0 } else { UCR_MPHY }
                            | if wide { UCR_U16M } else { 0 }
                            | (extra << 2)
                            | UCR_UREN;
                        assert_eq!(read(&mut device, UCR) & mask, expected, "{label}: UCR");
                        let packet = 4 * u32::from(cell_bytes).div_ceil(4);
                        let bsr = device.with_core(0, |cpu| {
                            let mut bus = cpu.bus_mut();
                            bus.read32(PDMA_RXC0);
                            bus.read32(PDMA_RXC0 + PDMA_BSR)
                        });
                        assert_eq!(bsr, (packet << 16) | (cells * packet), "{label}: BSR");
                        let microseconds = if wide { 50 } else { 100 };
                        assert_eq!(
                            read(&mut device, TIMER0 + 0x18) + 1,
                            mhz * microseconds / 6,
                            "{label}: timer period"
                        );
                        assert_eq!(read(&mut device, PLLM) & 0x3f, 18, "{label}: PLL1 x19");

                        // Multi-PHY answers its slave ID alone; single-PHY decodes no address.
                        let other = slave ^ 1;
                        let answer = device.fabric().utopia_master_poll_receive(other);
                        assert_eq!(answer, single.then_some(true), "{label}: PHY {other}");
                        let address = if single { other } else { slave };
                        let junk = sized_cell(usize::from(cell_bytes), &[0xee; 32], [0; 4]);
                        for _ in 0..280 {
                            send_to(&mut device, address, &junk);
                        }
                        for chunk in full_table().chunks(32) {
                            let cell = sized_cell(usize::from(cell_bytes), chunk, MAGIC);
                            send_to(&mut device, address, &cell);
                        }
                        let progress = boot_until_done(&mut device);
                        assert_eq!(progress.error, None, "{label}");
                        assert_eq!(progress.discarded, 280, "{label}");
                        assert_eq!(progress.accepted, 4, "{label}");
                        assert_eq!(progress.entry, Some(CODE), "{label}");
                        run(&mut device, 400);
                        assert_eq!(device.core(0).register('A', 4), 7, "{label}: core 0");
                        assert_eq!(device.core(1).register('A', 4), 8, "{label}: core 1");
                    }
                }
            }

            /// Options bit 2 skips port initialisation. The ROM still sets up its own receive
            /// path - PDMA channel 8 and the timer - but UCR is whatever came before: here a boot
            /// configuration table on the same EEPROM, run by the I2C boot first, which brings
            /// the port up at a PHY and a cell size of its own. The parameter table's slave ID
            /// names another PHY and is not applied. Without that table the port stays in
            /// reset and no cell can reach the device.
            #[test]
            fn skipping_port_initialisation_leaves_the_port_as_the_eeprom_configured_it() {
                const RUDC_57: u32 = 4 << 2;
                for endian in [Endian::Little, Endian::Big] {
                    let label = format!("big-endian {}", endian == Endian::Big);
                    let set = UCR_BEND | (u32::from(PHY) << 24) | RUDC_57 | UCR_UREN;
                    let config = blocks(&words(&[UCR, set, UCR_U16M, 0, 0, 0]));
                    let next = 0x80 + config.len() as u16;
                    let mut image = c6472_i2c_parameters(2, 0x80, 0x50, (next, 0x50));
                    image.extend(config);
                    image.extend(utopia_block(OPTION_SKIP_PORT_INIT, 57, 8, 0x02, 500));
                    let mut device = i2c_device(endian, image);
                    // The configuration table brings the port up before the I2C boot has read
                    // the parameter block after it; cells sent now wait in the queue for the
                    // PDMA channel the UTOPIA boot programs.
                    until_listening(&mut device, PHY);
                    assert_eq!(device.fabric().utopia_master_poll_receive(0x02), None);
                    for chunk in full_table().chunks(32) {
                        send(&mut device, &sized_cell(57, chunk, MAGIC));
                    }
                    let progress = boot_until_done(&mut device);
                    assert_eq!(progress.error, None, "{label}");
                    assert_eq!(progress.entry, Some(CODE), "{label}");
                    let i2c = device.boot().i2c().unwrap();
                    assert_eq!(i2c.config_blocks, 1, "{label}");
                    assert_eq!(i2c.parameter_blocks, 2, "{label}");
                    // UCR is still the table's: the handover wrote neither SLID nor anything else.
                    let mask = UCR_BEND | UCR_SLID | UCR_MPHY | UCR_U16M | UCR_RUDC | UCR_UREN;
                    assert_eq!(read(&mut device, UCR) & mask, set | UCR_MPHY, "{label}");
                    run(&mut device, 400);
                    assert_eq!(device.core(0).register('A', 4), 7, "{label}");
                }

                // The same skip with nothing to configure the port: the ROM's own path is up,
                // the port is not, and the boot waits with nothing able to reach it.
                let mut image = c6472_i2c_parameters(0, 0x80, 0x50, (0, 0));
                image.extend(utopia_block(OPTION_SKIP_PORT_INIT, 53, 8, PHY.into(), 500));
                let mut device = i2c_device(Endian::Little, image);
                for _ in 0..300_000 {
                    device.step_cycle();
                }
                let progress = device.boot().utopia().expect("handed over to UTOPIA");
                assert!(channel_enabled(&mut device, 0), "PDMA channel 8");
                assert_eq!(read(&mut device, TIMER0 + 0x18) + 1, 8333);
                // SPRUEG2E Figure 9: the reserved one, MPHY and U16M, with UREN clear.
                assert_eq!(read(&mut device, UCR), 0x2000_6000, "UCR as reset left it");
                for phy in [PHY, 0x02, 0x1f] {
                    assert_eq!(device.fabric().utopia_master_poll_receive(phy), None);
                }
                assert_eq!(progress.accepted + progress.discarded, 0);
                assert_eq!(progress.entry, None);
                assert!(device.held_in_reset(0));
            }
        }
    }
}
