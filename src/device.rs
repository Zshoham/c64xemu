//! Which device is being emulated: how many megamodules it has, where its memories are, which
//! peripherals it carries and how its events are numbered.
//!
//! The emulator models two members of the C64x+ family. The **C6455** is one megamodule with
//! 2 MB of L2 SRAM and one Ethernet MAC. The **C6472** is six megamodules of 608 KB each,
//! 768 KB of shared L2, and two Ethernet MACs sharing one MDIO, with a boot controller that
//! holds five of the six cores in reset until core 0 releases them (SPRS612G sections 2.3, 2.4
//! and 7.15). Everything that differs between them is a field of a `DeviceProfile`, so the
//! fabric, the memory hierarchy and the front ends have one device model rather than two.

use crate::cpu::Cpu;
use crate::events::{EventMap, C6455_EVENTS, C6472_EVENTS};
use crate::i2cboot::{I2cBoot, Progress as I2cProgress};
use crate::romboot::{BootParameters, EmacBoot, Progress};
use crate::space::{AddressMap, C6455_MAP, C6472_MAP};
use crate::system::{Endian, Fabric};

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
    pub(crate) has_utopia: bool,
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
    has_utopia: true,
    has_emifa: true,
    has_device_config: true,
    has_boot_controller: false,
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
    has_utopia: true,
    // The C6472 has no asynchronous EMIF: DDR2 is its only external memory.
    has_emifa: false,
    has_device_config: false,
    has_boot_controller: true,
    reset_core: 0,
    jtag_id: 0x0009_102f,
    megamodule_revision: 0x0001_0003,
    mdio_version: 0x0007_0103,
};

impl DeviceProfile {
    pub(crate) fn cores(&self) -> usize {
        self.map.cores
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

/// Which of the ROM's boot modes is running in place of core 0. The two are separate state
/// machines because they share nothing but the boot table they end in and the core they hold.
enum RomBoot {
    Emac(EmacBoot),
    I2c(I2cBoot),
}

impl RomBoot {
    /// One cycle of whichever boot this is, answering the entry point when it ends.
    fn tick(&mut self, fabric: &mut Fabric) -> Option<u32> {
        match self {
            Self::Emac(boot) => boot.tick(fabric),
            Self::I2c(boot) => boot.tick(fabric),
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
    /// The ROM bootloader's boot, once a front end has strapped the device to one. It outlives
    /// the boot it performed: what it did is what a front end reports afterwards.
    rom_boot: Option<RomBoot>,
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

    /// A device strapped to a boot mode: 0 starts every core at once, and any other mode runs
    /// core 0 and holds the rest until it releases them (SPRS612G section 2.4.1).
    pub fn with_boot_mode(
        profile: &'static DeviceProfile,
        entry: u32,
        endian: Endian,
        boot_mode: u8,
    ) -> Self {
        let mut fabric = Box::new(Fabric::with_boot_mode(profile, boot_mode));
        fabric.apply_reset_holds();
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
        cpus[profile.reset_core].attach(fabric);
        Self {
            cpus,
            focus: profile.reset_core,
            cycle: 0,
            rom_boot: None,
        }
    }

    /// Strap the device to the ROM bootloader's Ethernet boot on the MAC port at `base`
    /// (SPRUEC6G section 4.3.5). Core 0 is held while the boot code runs in its place, and the
    /// other cores stay held until the boot table releases them.
    pub fn begin_ethernet_boot(&mut self, base: u32, parameters: BootParameters) {
        let endian = self.fabric().endian();
        self.fabric().set_held_in_reset(0, true);
        self.rom_boot = Some(RomBoot::Emac(EmacBoot::new(base, endian, parameters)));
    }

    /// Strap the device to the ROM bootloader's I2C EEPROM boot (SPRUEC6G section 2.3.4), with
    /// the EEPROM image on the bus at the address the ROM requires and the boot parameter block
    /// at `0x80 * index`. Core 0 is held while the boot code runs in its place.
    pub fn begin_i2c_boot(&mut self, index: u32, contents: Vec<u8>) {
        let endian = self.fabric().endian();
        self.fabric()
            .attach_i2c_eeprom(crate::i2cboot::EEPROM_ADDRESS, contents);
        self.fabric().set_held_in_reset(0, true);
        self.rom_boot = Some(RomBoot::I2c(I2cBoot::new(index, endian)));
    }

    /// What the I2C boot has done, when the device was strapped to one.
    pub fn i2c_boot(&self) -> Option<I2cProgress> {
        match self.rom_boot.as_ref()? {
            RomBoot::I2c(boot) => Some(boot.progress()),
            RomBoot::Emac(_) => None,
        }
    }

    /// Whether the ROM bootloader is running in place of core 0. A run loop has to advance the
    /// device rather than the core while it is, because the boot is device state and the core
    /// it is booting is held.
    pub fn rom_booting(&self) -> bool {
        self.rom_boot.is_some()
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

    /// What the Ethernet boot has done, when the device was strapped to one.
    pub fn ethernet_boot(&self) -> Option<Progress> {
        match self.rom_boot.as_ref()? {
            RomBoot::Emac(boot) => Some(boot.progress()),
            RomBoot::I2c(_) => None,
        }
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
        let fabric = self.cpus[self.focus].detach();
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
    pub fn step_cycle(&mut self) {
        let mut fabric = self.cpus[self.focus].detach();
        self.cycle_on(&mut fabric, None);
        self.cpus[self.focus].attach(fabric);
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
    /// A single core with no ROM boot runs itself: `Cpu::run_cycles` is the fast path, it ticks
    /// the device from inside its own cycle and counts in the core's cycles, and it leaves the
    /// device's cycle counter alone. Six cores, or a ROM boot - which is device state, and holds
    /// the core it boots - advance whole device cycles and count in those.
    pub(crate) fn run_cycles(
        &mut self,
        cycles: u64,
        cio: Option<u32>,
        ignore_cio_once: bool,
    ) -> u64 {
        if self.cpus.len() == 1 && self.rom_boot.is_none() {
            return self.cpus[0].run_cycles(cycles, cio, ignore_cio_once);
        }
        // The device keeps the fabric for the whole batch and gives it back to the focused
        // core at the end, which is before any caller can look.
        let mut fabric = self.cpus[self.focus].detach();
        let mut boundaries = Boundaries::scan(&self.cpus, cio);
        let start = self.cycle;
        for step in 0..cycles {
            if boundaries.any_halted(&fabric) {
                break;
            }
            if boundaries.at_cio != 0 && !(step == 0 && ignore_cio_once) {
                break;
            }
            self.cycle_on(&mut fabric, Some(&mut boundaries));
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
            if fabric.held_in_reset(core) || !fabric.core_clocked(core) {
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
        if let Some(boot) = self.rom_boot.as_mut() {
            if let Some(entry) = boot.tick(fabric) {
                // The boot table has ended: the ROM branches core 0 to the entry point it named,
                // and the cores the table released are already starting from their own boot
                // addresses (SPRUEC6G section 4.3.5).
                self.cpus[0].set_pc(entry);
                fabric.set_held_in_reset(0, false);
                if let Some(boundaries) = boundaries.as_deref_mut() {
                    boundaries.observe(0, &self.cpus[0]);
                }
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
            // BC0 never starts core 0 during a ROM boot. Core 0 is the core that runs the boot
            // code, so it is out of reset from power-up and its bit only signals that the boot
            // finished (SPRUEC6G section 4.1); where core 0 goes is the boot table's entry
            // point, which the boot itself applies.
            if core == 0 && self.rom_boot.is_some() {
                continue;
            }
            if let Some(address) = fabric.boot_address(core) {
                self.cpus[core].set_pc(address);
                if let Some(boundaries) = boundaries.as_deref_mut() {
                    boundaries.observe(core, &self.cpus[core]);
                }
            }
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
            rom_boot: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::system::Endian;

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
}
