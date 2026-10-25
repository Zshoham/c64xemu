//! The peripheral fabric: the megamodules, the shared peripherals, and the routing between
//! them.
//!
//! A C64x+ device is a set of megamodules around one set of peripherals. Everything private to
//! a megamodule - its caches, its interrupt controller, its memory protection units, its
//! bandwidth manager, its IDMA - is a [`Megamodule`]; everything the device shares - the
//! Ethernet MACs, the EDMA3, the timers, the memory behind the caches - is [`Shared`]; and the
//! [`Fabric`] holds both and answers every access, from a core or from a DMA master, by
//! deciding which of the two it belongs to.
//!
//! That split is what makes six cores expressible. On a C6472 an access from core 3 to
//! 0080 0000h is core 3's own L2 SRAM, an access from the EMAC to 1380 0000h is the same
//! bytes, and both have to be snooped against core 3's L1D (SPRS612G Table 2-2, SPRU862B
//! chapter 2). The address map in `space.rs` resolves the aliases and the fabric applies the
//! coherence rules on whichever megamodule owns the memory - which on a single-core device is
//! always the only one there is, so the C6455 behaves exactly as it did before.

use crate::boot::BootMemory;
use crate::bootctl::BootController;
use crate::cache::CacheView;
use crate::chipcfg::{ChipConfig, DEVSTAT_LITTLE_ENDIAN};
use crate::ddr2::Ddr2;
use crate::devcfg::DeviceConfig;
use crate::device::{DeviceProfile, MpuRegion};
use crate::edma::{Edma, TransferMemory};
use crate::edma_tc::TransferController;
use crate::emac::{DmaMemory as EmacDmaMemory, Emac};
use crate::emic::{EmacInterruptControl, Sources as EmacSources};
use crate::emifa::Emifa;
use crate::events::Destination;
use crate::gpio::Gpio;
use crate::hpi::{HostPort, Hpi, HpiBoot};
use crate::i2c::I2c;
use crate::idma::{Idma, Source as IdmaSource};
use crate::mcbsp::Mcbsp;
use crate::mdio::Mdio;
use crate::memory::{is_memory, AccessTiming, Backing, BoundHierarchy, LineFill, MemoryHierarchy};
use crate::pdma::{Pdma, MAX_BURST_WORDS};
use crate::pll::{Pll, Pll2};
use crate::port::{PortContext, PortWrite, SlavePort};
use crate::psc::PowerSleep;
use crate::smc::{SharedMemory, SharedMemoryProfiler};
use crate::space::{Placement, Requester};
use crate::sysclk3::Clocked;
use crate::timer::Timer;
use crate::utopia::{Utopia, WireCell};
#[cfg(test)]
use serde_json::{json, Value};

const EVTFLAG0: u32 = 0x0180_0000;
const EVTSET0: u32 = 0x0180_0020;
const EVTCLR0: u32 = 0x0180_0040;
const EVTMASK0: u32 = 0x0180_0080;
const MEVTFLAG0: u32 = 0x0180_00a0;
const EXPMASK0: u32 = 0x0180_00c0;
const MEXPFLAG0: u32 = 0x0180_00e0;
const INTMUX1: u32 = 0x0180_0104;
const INTXSTAT: u32 = 0x0180_0180;
const INTXCLR: u32 = 0x0180_0184;
const INTDMASK: u32 = 0x0180_0188;
const AEGMUX0: u32 = 0x0181_0140;
const AEGMUX1: u32 = 0x0181_0144;
const PDCCMD: u32 = 0x0181_0000;
/// MM_REVID, the megamodule revision (SPRS612G section 5.6).
const MM_REVID: u32 = 0x0181_2000;
const L2PDWAKE0: u32 = 0x0184_c040;
const L2PDWAKE1: u32 = 0x0184_c044;
const L2PDSLEEP0: u32 = 0x0184_c050;
const L2PDSLEEP1: u32 = 0x0184_c054;
const L2PDSTAT0: u32 = 0x0184_c060;
const L2PDSTAT1: u32 = 0x0184_c064;
/// Arbitration register address, reset value and writable field mask (SPRU871K Table 6-2).
const BWM_REGISTERS: [(u32, u32, u32); 12] = [
    (0x0184_1000, 0x0001_0010, 0x0007_003f), // CPUARBU
    (0x0184_1004, 0x0000_0010, 0x0000_003f), // IDMAARBU
    (0x0184_1008, 0x0000_0001, 0x0000_003f), // SDMAARBU
    (0x0184_100c, 0x0000_0020, 0x0000_003f), // UCARBU
    (0x0184_1040, 0x0001_0010, 0x0007_003f), // CPUARBD
    (0x0184_1044, 0x0000_0010, 0x0000_003f), // IDMAARBD
    (0x0184_1048, 0x0000_0001, 0x0000_003f), // SDMAARBD
    (0x0184_104c, 0x0000_0020, 0x0000_003f), // UCARBD
    (0x0182_0200, 0x0001_0010, 0x0007_003f), // CPUARBE
    (0x0182_0204, 0x0000_0010, 0x0000_003f), // IDMAARBE
    (0x0182_0208, 0x0000_0001, 0x0000_003f), // SDMAARBE
    (0x0182_020c, 0x0007_0000, 0x0007_0000), // MDMAARBE
];

/// The memory protection units, one per local memory. Register offsets come from
/// `linux-c6x/arch/c6x/platforms/cache.c`: fault address, fault status and fault clear at the
/// base, and the per-page attribute registers 0x200 further on.
const MPU_BASES: [(u32, u32); 3] = [
    (0x0184_a000, 0x0184_a200),
    (0x0184_a400, 0x0184_a600),
    (0x0184_ac00, 0x0184_ae00),
];
/// `MPFSR` bit 8: the fault was a local (CPU) request rather than a requestor's
/// (SPRU871K Table 4-39 and section 8.4.2.1.3).
const MPFSR_LOCAL: u32 = 1 << 8;

/// Whether a unit's fault registers let a local (CPU) fault replace the remote (DMA or IDMA)
/// one they are already holding. L1P and L2 say they do - SPRU871K 2.9.2.3.3: "The L1P allows
/// a local fault to replace a remote fault and generate a new exception", and 4.6.2.4.3 states
/// it as a rule: "If the LOCAL field of the MPFSR register = 0, and the pending exception sets
/// it to 1, the hardware records the new fault and signals the new exception". L1D's own
/// section carries neither sentence, only the general rule of 3.7.2.3.3: "The hardware records
/// the information about the first fault and generates an exception only for that fault." The
/// units are L2, L1P and L1D, in `MPU_BASES` order.
const fn local_replaces_remote(unit: usize) -> bool {
    unit != 2
}

/// How many MPPA registers a unit can have. The C6455 uses 32 pages per unit; the C6472 uses
/// index 43 for the last shared-L2 page (SPRS612G section 5.2), so the file is 64 wide and the
/// device profile says which entries mean anything.
const MPU_PAGES: usize = 64;

/// `MPPA`: bit 8 LOCAL, bits 5-0 SR SW SX UR UW UX. A page that grants everything a CPU can
/// ask for needs all seven, which is the reset state - firmware that never programs the unit
/// must see no faults.
const MPPA_CPU_MASK: u32 = 0x0000_013f;
/// The bits a DMA master's access is checked against: the six AID enables (SPRU871K 8.2.2)
/// and the same six permission bits.
const MPPA_DMA_MASK: u32 = 0x0000_fe3f;
/// The single `EXCEP` input the exception combiner presents, which the dropped-interrupt
/// reporting also uses. Every C64x+ megamodule numbers it 96 (SPRU871K 7.2.1).
const EXCEP_EVENT: usize = 96;
const SYSCLK3_DIVIDER: u8 = 6;
const ACTIVE_IDMA: u8 = 1 << 0;
const ACTIVE_EMAC: u8 = 1 << 1;
const ACTIVE_EDMA: u8 = 1 << 2;
const ACTIVE_I2C: u8 = 1 << 3;
const ACTIVE_MCBSP: u8 = 1 << 4;
const ACTIVE_MDIO: u8 = 1 << 5;
/// Every bit the summary defines: a wake-everything write sets all of them and the tick
/// settles each one back, so nothing outside this mask may survive a tick.
const ACTIVE_ALL: u8 =
    ACTIVE_IDMA | ACTIVE_EMAC | ACTIVE_EDMA | ACTIVE_I2C | ACTIVE_MCBSP | ACTIVE_MDIO;

#[inline]
fn directly_routed_memory(address: u32) -> bool {
    !(0x0180_0000..0xa000_0000).contains(&address)
}

/// The megamodule's internal configuration space, "C64x+ Megamodule Registers" at 0180 0000h -
/// 01BF FFFFh (SPRS276M Table 2-2): its interrupt controller, cache, memory protection and
/// power-down registers.
const MEGAMODULE_REGISTERS: std::ops::Range<u32> = 0x0180_0000..0x01c0_0000;

fn mux_uses_combiner(value: u32) -> bool {
    (0..4).any(|slot| ((value >> (slot * 8)) & 0x7f) < 4)
}

/// Events 0-3 are the interrupt combiner's own outputs: they never latch in EVTFLAG0 and so
/// cannot reach the exception combiner (SPRU871K 7.2.1).
const fn reserved_events(bank: usize) -> u32 {
    if bank == 0 {
        0xf
    } else {
        0
    }
}

/// What a CPU access asks a page for.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Permission {
    Read,
    Write,
    Execute,
}

/// Whom a master's access is made for: the PrivID and privilege mode memory protection judges
/// it by. Not to be confused with `Requester`, which only picks the address view an access
/// resolves through. A core's PrivID is its index - AID0 to AID5 on the C6472 (SPRS612G Table
/// 5-3), 0 on the one-core C6455 (SPRS276M section 5.2) - and an EDMA3 transfer carries the
/// one stamped on its PaRAM set (SPRU966C section 2.10.2).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Privilege {
    pub id: u8,
    pub user: bool,
}

impl Privilege {
    /// The HPI's DMA and the IDMA: PrivID 0, supervisor (SPRS276M section 5.2, SPRS612G
    /// Table 5-3).
    pub const ID0_SUPERVISOR: Self = Self { id: 0, user: false };
    /// The EMACs and the UTOPIA PDMA: PrivID 0, always user (SPRS612G section 5.2).
    pub const ID0_USER: Self = Self { id: 0, user: true };

    /// The bit of a page attribute register that admits this PrivID: AID0 to AID5 at bits 10
    /// to 15, and every id above them with AIDX at bit 9 (SPRU966C Figure 4-28; the
    /// megamodule's MPPA registers share the layout).
    pub fn allowed_id_bit(self) -> u32 {
        if self.id < 6 {
            1 << (10 + u32::from(self.id))
        } else {
            1 << 9
        }
    }
}

#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub(crate) enum Endian {
    #[default]
    Little,
    Big,
}

/// One megamodule: the caches and everything in its 0180 0000h configuration space.
pub(crate) struct Megamodule {
    /// The caches, the MAR bits and the cache configuration of this megamodule.
    memory: MemoryHierarchy,
    event_flags: [u32; 4],
    /// Which events are asserted this cycle. The interrupt selector routes these raw lines to
    /// the CPU, while the latched flags above feed the event combiner (SPRU871K Figure 7-9), so
    /// a level source keeps its line high until it is quieted and a pulse source raises it for
    /// one cycle.
    event_lines: [u32; 4],
    event_masks: [u32; 4],
    /// The exception combiner's mask: a set bit keeps that event out of the single `EXCEP`
    /// input, the same polarity as `EVTMASK` (SPRU871K 7.2.4).
    exception_masks: [u32; 4],
    /// Whether any latched flag reaches `EXCEP` through those masks. The CPU asks once per
    /// cycle and only a flag or mask write can change the answer, so it is maintained on
    /// those writes rather than recomputed over the four banks every cycle.
    exception_pending: bool,
    interrupt_mux: [u32; 3],
    selector_uses_combiner: bool,
    interrupt_exception: u32,
    dropped_interrupt_mask: u16,
    aeg_mux: [u32; 2],
    bandwidth: [u32; 12],
    pdccmd: u32,
    l2_power_status: [u32; 2],
    /// Per-page attributes for L2, L1P and L1D, with the fault address and status each unit
    /// latches. `restrictive` is false while every page still grants a CPU everything, which
    /// is the reset state and lets the access path skip the check entirely.
    mpu_pages: [[u32; MPU_PAGES]; 3],
    mpu_fault_address: [u32; 3],
    mpu_fault_status: [u32; 3],
    mpu_restrictive: bool,
    /// Whether any page has stopped granting every DMA requester id everything. The DMA check
    /// runs per byte, so the reset state has to cost nothing.
    mpu_dma_restrictive: bool,
    mpu_lock_key: [[u32; 4]; 3],
    mpu_lock_input: [[u32; 4]; 3],
    mpu_lock_written: [u8; 3],
    mpu_lock_sequence: [bool; 3],
    mpu_locked: [bool; 3],
    idma: Idma,
    /// Level-triggered events known at the last device tick, and the CPU lines they select,
    /// used only by the dormant short path.
    idle_level_events: [u32; 4],
    idle_cpu_lines: u16,
    /// Whether the boot controller is holding this core in reset. Its memories still answer -
    /// a host or core 0 initialises them while the core waits (SPRUEC6G section 4.3.2).
    held_in_reset: bool,
}

impl Megamodule {
    fn new(core: usize, profile: &'static DeviceProfile) -> Self {
        Self {
            memory: MemoryHierarchy::new(core, profile.map),
            event_flags: [0; 4],
            event_lines: [0; 4],
            event_masks: [0; 4],
            exception_masks: [u32::MAX; 4],
            exception_pending: false,
            interrupt_mux: [0x0706_0504, 0x0b0a_0908, 0x0f0e_0d0c],
            selector_uses_combiner: false,
            interrupt_exception: 0,
            dropped_interrupt_mask: 0,
            aeg_mux: [0x0302_0100, 0x0706_0504],
            bandwidth: BWM_REGISTERS.map(|(_, reset, _)| reset),
            pdccmd: 0,
            l2_power_status: [0; 2],
            mpu_pages: [[0x0000_ffff; MPU_PAGES]; 3],
            mpu_fault_address: [0; 3],
            mpu_fault_status: [0; 3],
            mpu_restrictive: false,
            mpu_dma_restrictive: false,
            mpu_lock_key: [[0; 4]; 3],
            mpu_lock_input: [[0; 4]; 3],
            mpu_lock_written: [0; 3],
            mpu_lock_sequence: [false; 3],
            mpu_locked: [false; 3],
            idma: Idma::default(),
            idle_level_events: [0; 4],
            idle_cpu_lines: 0,
            held_in_reset: false,
        }
    }

    /// Live view of the three cache levels, for the debugger and the dashboard.
    pub(crate) fn cache_views(&self) -> [CacheView; 3] {
        self.memory.cache_views()
    }

    pub(crate) fn cache_line_data(&self, level: usize, address: u32) -> Option<&[u8]> {
        self.memory.cache_line_data(level, address)
    }

    /// The 256 MAR registers; bit 0 of each enables caching for its 16 MiB segment.
    pub(crate) fn mar_registers(&self) -> [u32; 256] {
        self.memory.mar_registers()
    }

    /// INTMUX1-3: the selector byte per CPU interrupt INT4-INT15.
    pub(crate) fn interrupt_selectors(&self) -> [u32; 3] {
        self.interrupt_mux
    }

    /// EVTFLAG0-3: the latched flag for each of the 128 chip-level events.
    pub(crate) fn event_flag_registers(&self) -> [u32; 4] {
        self.event_flags
    }

    /// Assert a system event for this cycle: its flag latches until software clears it, and
    /// its line is what the interrupt selector routes to the CPU.
    pub(crate) fn raise_event(&mut self, event: usize) {
        let bank = event / 32;
        let bit = 1 << (event % 32);
        self.event_flags[bank] |= bit;
        self.event_lines[bank] |= bit;
        self.exception_pending |= bit & !self.exception_masks[bank] & !reserved_events(bank) != 0;
    }

    /// The interrupt lines the selector presents to the CPU this cycle, consuming the event
    /// receptions that produced them.
    ///
    /// SPRU871K Figure 7-9 feeds the interrupt selector from the raw system events while the
    /// latched flags feed the event combiner, and section 7.4.2 states that a service routine
    /// for a directly selected single event needs neither to read nor to clear EVTFLAG. So a
    /// directly routed event asserts its CPU interrupt when it is *received*, not for as long as
    /// its flag stays set: an unacknowledged flag cannot re-enter the handler. The four combiner
    /// outputs are different - they are the level of their group's unmasked flags (7.2.2), which
    /// is why a combined-event routine has to clear the flags before it returns.
    pub(crate) fn take_pending_cpu_interrupts(&mut self, short_tick: bool) -> u16 {
        if short_tick {
            return self.idle_cpu_lines;
        }
        if self.event_lines == [0; 4] && !self.selector_uses_combiner {
            return 0;
        }
        let pending = self.selected_cpu_interrupts(&self.event_lines);
        self.event_lines = [0; 4];
        pending
    }

    /// Drop the event receptions waiting for the selector without presenting them to the CPU.
    /// A core held in reset has no IFR to latch them into, so an event received while it is
    /// held is gone when it comes out. The flags still latch: EVTFLAG and the combiner outputs
    /// belong to the interrupt controller, not to the CPU in reset.
    pub(crate) fn drop_event_receptions(&mut self) {
        self.event_lines = [0; 4];
    }

    fn selected_cpu_interrupts(&self, event_lines: &[u32; 4]) -> u16 {
        let mut pending = 0u16;
        for interrupt in 4..16 {
            let mux = self.interrupt_mux[(interrupt - 4) / 4];
            let event = ((mux >> (((interrupt - 4) % 4) * 8)) & 0x7f) as usize;
            let asserted = if event < 4 {
                self.event_flags[event] & !self.event_masks[event] != 0
            } else {
                event_lines[event / 32] & (1 << (event % 32)) != 0
            };
            if asserted {
                pending |= 1 << interrupt;
            }
        }
        pending
    }

    pub(crate) fn record_dropped_interrupt(&mut self, interrupt: u8) {
        let bit = 1u16 << interrupt;
        if self.interrupt_exception != 0 || self.dropped_interrupt_mask & bit != 0 {
            return;
        }
        let mux = self.interrupt_mux[(interrupt as usize - 4) / 4];
        let event = (mux >> (((interrupt as usize - 4) % 4) * 8)) & 0x7f;
        self.interrupt_exception = (event << 24) | (u32::from(interrupt) << 16) | 1;
        self.raise_event(EXCEP_EVENT);
    }

    /// The single `EXCEP` input the CPU sees: the OR of every latched event the exception
    /// combiner does not mask out. Events 0-3 are the interrupt combiner's own outputs and
    /// never appear in EVTFLAG0, so they cannot reach it (SPRU871K 7.2.1, 7.2.4).
    pub(crate) fn pending_exception_event(&self) -> bool {
        self.exception_pending
    }

    fn masked_exception_flags(&self, bank: usize) -> u32 {
        self.event_flags[bank] & !self.exception_masks[bank] & !reserved_events(bank)
    }

    /// Recompute the cached `EXCEP` summary after a write to the flags or the mask.
    fn refresh_exception_pending(&mut self) {
        self.exception_pending = (0..4).any(|bank| self.masked_exception_flags(bank) != 0);
    }

    /// Record one memory protection fault in a unit's MPFAR and MPFSR and raise the exception
    /// that belongs to its source.
    ///
    /// SPRU871K 8.4.2.1.3: "The MPFAR and MPFSR registers only store information for one
    /// fault... each individual memory generates no more than one exception until the CPU
    /// clears that memory's MPFAR and MPFSR registers." The single exception to that is a
    /// local fault arriving at a unit holding a remote one, which `local_replaces_remote`
    /// quotes.
    pub(crate) fn record_mpu_fault(
        &mut self,
        unit: usize,
        address: u32,
        status: u32,
        events: &crate::events::EventMap,
    ) {
        let recorded = self.mpu_fault_status[unit];
        let local = status & MPFSR_LOCAL != 0;
        if recorded != 0 && !(local && recorded & MPFSR_LOCAL == 0 && local_replaces_remote(unit)) {
            return;
        }
        self.mpu_fault_address[unit] = address;
        self.mpu_fault_status[unit] = status;
        let event = if local {
            events.mpu_cpu[unit]
        } else {
            events.mpu_dma[unit]
        };
        self.raise_event(event);
    }

    fn record_lock_fault(&mut self, unit: usize, address: u32, events: &crate::events::EventMap) {
        // A refused lock command is the supervisor's own write, so it is a local fault with SW.
        self.record_mpu_fault(unit, address, MPFSR_LOCAL | (1 << 4), events);
        self.mpu_lock_sequence[unit] = false;
    }

    /// The megamodule's own configuration registers, except the ones the fabric has to see:
    /// `None` when the address is not one of them.
    fn cfg_read(
        &mut self,
        address: u32,
        profile: &'static DeviceProfile,
        context: PortContext,
    ) -> Option<u32> {
        if let Some(value) = self.idma.read(address, context) {
            return Some(value);
        }
        if let Some((bank, kind)) = intc_register(address) {
            return Some(match kind {
                IntcRegister::Flag => self.event_flags[bank],
                IntcRegister::Mask => self.event_masks[bank],
                IntcRegister::MaskedFlag => self.event_flags[bank] & !self.event_masks[bank],
                IntcRegister::ExceptionMask => self.exception_masks[bank],
                IntcRegister::MaskedExceptionFlag => self.masked_exception_flags(bank),
            });
        }
        match address {
            INTXSTAT => return Some(self.interrupt_exception),
            INTXCLR => return Some(0),
            INTDMASK => return Some(u32::from(self.dropped_interrupt_mask)),
            AEGMUX0 => return Some(self.aeg_mux[0]),
            AEGMUX1 => return Some(self.aeg_mux[1]),
            MM_REVID => return Some(profile.megamodule_revision),
            _ => {}
        }
        if let Some(value) = self.mpu_register(address, None, profile) {
            return Some(value);
        }
        if let Some(index) = bank_index(address, INTMUX1) {
            if index < 3 {
                return Some(self.interrupt_mux[index]);
            }
        }
        if let Some(index) = bandwidth_register(address) {
            return Some(self.bandwidth[index]);
        }
        match address {
            PDCCMD => return Some(self.pdccmd | 0xffff),
            L2PDWAKE0 | L2PDWAKE1 | L2PDSLEEP0 | L2PDSLEEP1 => return Some(0),
            L2PDSTAT0 => return Some(self.l2_power_status[0]),
            L2PDSTAT1 => return Some(self.l2_power_status[1]),
            _ => {}
        }
        if let Some(value) = self.memory.cache_register_read(address) {
            return Some(value);
        }
        self.memory.mar_read(address)
    }

    /// Returns the event an IDMA completion raised, if the write started one.
    fn cfg_write(
        &mut self,
        backing: &mut Backing,
        address: u32,
        value: u32,
        profile: &'static DeviceProfile,
        context: PortContext,
    ) -> Option<CfgWrite> {
        match self.idma.write(address, value, context) {
            PortWrite::Unhandled => {}
            PortWrite::Done => return Some(CfgWrite::Done),
            PortWrite::Event(event) => return Some(CfgWrite::Event(event)),
        }
        if let Some(index) = bandwidth_register(address) {
            self.bandwidth[index] = value & BWM_REGISTERS[index].2;
            return Some(CfgWrite::Done);
        }
        match address {
            INTXSTAT | MM_REVID => return Some(CfgWrite::Done),
            INTXCLR => {
                if value & 1 != 0 {
                    self.interrupt_exception = 0;
                }
                return Some(CfgWrite::Done);
            }
            INTDMASK => {
                self.dropped_interrupt_mask = (value & 0xfff0) as u16;
                return Some(CfgWrite::Done);
            }
            AEGMUX0 => {
                self.aeg_mux[0] = value;
                return Some(CfgWrite::Done);
            }
            AEGMUX1 => {
                self.aeg_mux[1] = value;
                return Some(CfgWrite::Done);
            }
            PDCCMD => {
                self.pdccmd = value & (1 << 16);
                return Some(CfgWrite::Done);
            }
            L2PDWAKE0 => {
                self.l2_power_status[0] &= !(value & 3);
                return Some(CfgWrite::Done);
            }
            L2PDWAKE1 => {
                self.l2_power_status[1] &= !(value & 3);
                return Some(CfgWrite::Done);
            }
            L2PDSLEEP0 => {
                self.l2_power_status[0] |= value & 3;
                return Some(CfgWrite::Done);
            }
            L2PDSLEEP1 => {
                self.l2_power_status[1] |= value & 3;
                return Some(CfgWrite::Done);
            }
            L2PDSTAT0 | L2PDSTAT1 => return Some(CfgWrite::Done),
            _ => {}
        }
        if let Some(index) = bank_index(address, EVTSET0) {
            let writable = if index == 0 { value & !0xf } else { value };
            self.event_flags[index] |= writable;
            self.event_lines[index] |= writable;
            self.exception_pending |=
                writable & !self.exception_masks[index] & !reserved_events(index) != 0;
            return Some(CfgWrite::Done);
        }
        if let Some(index) = bank_index(address, EVTCLR0) {
            self.event_flags[index] &= !value;
            self.refresh_exception_pending();
            return Some(CfgWrite::Done);
        }
        if let Some(index) = bank_index(address, EVTMASK0) {
            self.event_masks[index] = value;
            return Some(CfgWrite::Done);
        }
        if let Some(index) = bank_index(address, EXPMASK0) {
            self.exception_masks[index] = value;
            self.refresh_exception_pending();
            return Some(CfgWrite::Done);
        }
        if self.mpu_register(address, Some(value), profile).is_some() {
            return Some(CfgWrite::Done);
        }
        if let Some(index) = bank_index(address, INTMUX1) {
            if index < 3 {
                self.interrupt_mux[index] = value;
                self.selector_uses_combiner =
                    self.interrupt_mux.iter().copied().any(mux_uses_combiner);
                return Some(CfgWrite::Done);
            }
        }
        if self
            .memory
            .bind(backing)
            .cache_register_write(address, value)
        {
            return Some(CfgWrite::Done);
        }
        if self.memory.mar_write(address, value) {
            return Some(CfgWrite::Done);
        }
        None
    }

    fn mpu_register(
        &mut self,
        address: u32,
        value: Option<u32>,
        profile: &'static DeviceProfile,
    ) -> Option<u32> {
        for (unit, (fault_base, page_base)) in MPU_BASES.iter().copied().enumerate() {
            if let Some(index) = page_index(address, page_base) {
                if let Some(value) = value {
                    if !self.mpu_locked[unit] {
                        self.mpu_pages[unit][index] = value & 0xffff;
                        self.mpu_restrictive = self
                            .mpu_pages
                            .iter()
                            .flatten()
                            .any(|page| page & MPPA_CPU_MASK != MPPA_CPU_MASK);
                        self.mpu_dma_restrictive = self
                            .mpu_pages
                            .iter()
                            .flatten()
                            .any(|page| page & MPPA_DMA_MASK != MPPA_DMA_MASK);
                    }
                }
                return Some(self.mpu_pages[unit][index]);
            }
            let lock_base = fault_base + 0x100;
            if let Some(offset) = address.checked_sub(lock_base) {
                match offset {
                    0x00 | 0x04 | 0x08 | 0x0c => {
                        if let Some(value) = value {
                            let index = (offset / 4) as usize;
                            let bit = 1 << index;
                            if !self.mpu_lock_sequence[unit]
                                || self.mpu_lock_written[unit] & bit != 0
                            {
                                self.record_lock_fault(unit, address, profile.events);
                            } else {
                                self.mpu_lock_input[unit][index] = value;
                                self.mpu_lock_written[unit] |= bit;
                            }
                        }
                        return Some(0);
                    }
                    0x10 => {
                        if let Some(command) = value {
                            if command & !7 != 0 || command.count_ones() > 1 {
                                self.record_lock_fault(unit, address, profile.events);
                            } else if command & 4 != 0 {
                                self.mpu_lock_sequence[unit] = true;
                                self.mpu_lock_written[unit] = 0;
                                self.mpu_lock_input[unit] = [0; 4];
                            } else if command & 2 != 0 {
                                if !self.mpu_locked[unit]
                                    && self.mpu_lock_sequence[unit]
                                    && self.mpu_lock_written[unit] == 0x0f
                                {
                                    self.mpu_lock_key[unit] = self.mpu_lock_input[unit];
                                    self.mpu_locked[unit] = true;
                                    self.mpu_lock_sequence[unit] = false;
                                } else {
                                    self.record_lock_fault(unit, address, profile.events);
                                }
                            } else if command & 1 != 0 {
                                if self.mpu_locked[unit]
                                    && self.mpu_lock_sequence[unit]
                                    && self.mpu_lock_written[unit] == 0x0f
                                    && self.mpu_lock_input[unit] == self.mpu_lock_key[unit]
                                {
                                    self.mpu_locked[unit] = false;
                                    self.mpu_lock_sequence[unit] = false;
                                } else {
                                    self.record_lock_fault(unit, address, profile.events);
                                }
                            }
                        }
                        return Some(0);
                    }
                    0x14 => return Some(u32::from(self.mpu_locked[unit])),
                    _ => {}
                }
            }
            match address.checked_sub(fault_base) {
                Some(0) => return Some(self.mpu_fault_address[unit]),
                Some(4) => return Some(self.mpu_fault_status[unit]),
                // MPFCLR: bit 0 is a command and nothing else in the register means anything.
                // SPRU871K Table 4-40 gives bit 0 as "Clear the L2MPFAR and the L2MPFCR
                // registers" and 8.4.2.1.3 adds that "Writing a 1 to any other bit of the
                // MPFCR register has no effect", so a write that misses the command bit leaves
                // the recorded fault standing.
                Some(8) => {
                    if value.is_some_and(|value| value & 1 != 0) {
                        self.mpu_fault_address[unit] = 0;
                        self.mpu_fault_status[unit] = 0;
                    }
                    return Some(0);
                }
                _ => {}
            }
        }
        None
    }
}

/// What a megamodule configuration write turned out to be.
enum CfgWrite {
    Done,
    /// An IDMA transfer completed and raised this event in the local core.
    Event(usize),
}

/// Names one shared peripheral port.
///
/// Declaration order is consult order, and it is load-bearing: the chip-level registers claim a
/// whole window, so they must answer after every port whose registers sit inside it. Nothing
/// else in the list overlaps, which is what `no_two_slave_ports_claim_one_address` proves - over
/// this same list, so the proof cannot drift from the dispatch.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum Port {
    Emac(usize),
    Emic(usize),
    Edma,
    Timer(usize),
    Gpio,
    I2c,
    Pll,
    Hpi,
    Ddr2,
    Emifa,
    Mcbsp(usize),
    Utopia,
    Pdma,
    Devcfg,
    Pll2,
    TransferController(usize),
    Mdio,
    BootController,
    Psc,
    Smc,
    SmcProfiler(usize),
    ChipConfig,
}

/// What the router owes the megamodules once a port has handled an access.
///
/// CONTEXT.md keeps the interrupt latchers in the router rather than in a port, because a
/// follow-up the megamodules are owed is not part of a port's shape. This names the follow-ups
/// so they belong to the port rather than to a position in a hand-written chain.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Follow {
    Nothing,
    EmacInterrupts,
    EdmaInterrupt,
    I2cInterrupt,
    UtopiaInterrupt,
    TransferControllerErrors,
    ReleaseBootedCores,
    PowerTransitions,
    PrefetchFlush,
    InterProcessorInterrupts,
}

impl Port {
    /// A read's follow-up. Only the three ports whose reads can raise something answer with
    /// one: UTOPIA's empty receive queue sets its starvation flag, and the EDMA3 and its
    /// transfer controllers report a refused or misaligned access by asserting an error. The
    /// rest are level sources the idle refresh recomputes, so a read that only clears a flag
    /// needs nothing here.
    fn after_read(self) -> Follow {
        match self {
            Port::Edma => Follow::EdmaInterrupt,
            Port::Utopia => Follow::UtopiaInterrupt,
            Port::TransferController(_) => Follow::TransferControllerErrors,
            _ => Follow::Nothing,
        }
    }

    /// A write's follow-up: the interrupt a register change may have asserted, or the
    /// device-level effect the write asked for.
    fn after_write(self) -> Follow {
        match self {
            Port::Emac(_) | Port::Emic(_) => Follow::EmacInterrupts,
            Port::Edma => Follow::EdmaInterrupt,
            Port::I2c => Follow::I2cInterrupt,
            Port::Utopia => Follow::UtopiaInterrupt,
            Port::TransferController(_) => Follow::TransferControllerErrors,
            Port::BootController => Follow::ReleaseBootedCores,
            Port::Psc => Follow::PowerTransitions,
            Port::Smc => Follow::PrefetchFlush,
            Port::ChipConfig => Follow::InterProcessorInterrupts,
            Port::Timer(_)
            | Port::Gpio
            | Port::Pll
            | Port::Hpi
            | Port::Ddr2
            | Port::Emifa
            | Port::Mcbsp(_)
            | Port::Pdma
            | Port::Devcfg
            | Port::Pll2
            | Port::Mdio
            | Port::SmcProfiler(_) => Follow::Nothing,
        }
    }
}

/// The peripherals and memory the device shares between its megamodules.
pub(crate) struct Shared {
    /// Every megamodule's SRAM, the shared L2, the DDR2 window and everything sparse.
    backing: Backing,
    mdio: Mdio,
    // Boxes make the temporary ownership transfer pointer-sized while the rest of the fabric
    // serves DMA accesses.
    emac: Vec<Option<Box<Emac>>>,
    /// The per-core interrupt-control module in front of each MAC, on a device that has them.
    emic: Vec<EmacInterruptControl>,
    edma: Option<Box<Edma>>,
    /// The SYSCLK3 blocks, each with its place in the SYSCLK3 schedule (`sysclk3.rs`).
    timers: Vec<Clocked<Timer>>,
    gpio: Clocked<Gpio>,
    i2c: I2c,
    pll: Pll,
    transfer_controllers: [TransferController; 4],
    hpi: Hpi,
    ddr2: Ddr2,
    emifa: Option<Emifa>,
    utopia: Option<Clocked<Utopia>>,
    /// The UTOPIA's PDMA, on a device whose UTOPIA has one (`pdma.rs`).
    pdma: Option<Pdma>,
    mcbsp: Vec<Mcbsp>,
    devcfg: Option<Clocked<DeviceConfig>>,
    pll2: Pll2,
    /// The C6472's device-level blocks.
    chipcfg: Option<ChipConfig>,
    bootctl: Option<BootController>,
    psc: Option<PowerSleep>,
    smc: Option<SharedMemory>,
    smc_profilers: Vec<SharedMemoryProfiler>,
    /// Whether the last ask of the device was a short tick: the peripherals did not move, so
    /// a core reads the interrupt lines the last full tick selected. Recorded rather than
    /// answered per core, because every core asks once per cycle of its own and the first ask
    /// of a cycle is the one that decides.
    last_tick_short: bool,
    /// What this device is made of, decided once. The tick runs every emulated cycle, so it
    /// must not re-derive the device's shape from its `Option`s and `Vec`s each time.
    emacs: usize,
    /// Whether this device has the device-level blocks `tick_device_blocks` advances - the
    /// interrupt-control modules in front of the MACs, the power/sleep controller, the
    /// shared-memory controller and its profilers. Cached from the profile rather than read
    /// through it, because the tick runs every emulated cycle and each block is behind an
    /// `Option` or an empty `Vec` that it must not interrogate to find out.
    device_level_blocks: bool,
    sysclk3_divide: u8,
    /// How many SYSCLK3 edges the device has had, counted where the timers step within the
    /// tick, so an access earlier in an edge's cycle sees the blocks as they were before it.
    sysclk3_edge: u64,
    /// No SYSCLK3 block is due before this edge: the schedule steps none of them until then.
    /// Never later than the earliest block's due edge, and brought forward by every access.
    sysclk3_deadline: u64,
    /// How many of the timers, from the front, belong to a megamodule and run on its clock.
    megamodule_timers: usize,
    /// No cycle-driven peripheral can change state before the next SYSCLK3 edge or host/MMIO
    /// action. Level interrupt sources are still relatched on the short path.
    dynamic_idle: bool,
    /// Devices that were active after the previous full tick. MMIO wakes all bits once.
    dynamic_active: u8,
    /// Whether anything since the last full tick could have armed a transfer or changed a
    /// level-triggered interrupt source: an access that left the memory windows, a frame
    /// arriving, or a peripheral that actually ticked. While it is clear the tick replays the
    /// two cached answers below instead of interrogating every peripheral again.
    peripheral_dirty: bool,
    /// Whether any level-triggered source was asserting at the last interrogation. A level
    /// has to be relatched every cycle, so the cached "nothing is asserted" is the only
    /// answer the tick may act on without asking again.
    levels_asserted: bool,
    /// The identity the EDMA3 was last told its current bus master carried, so the seam every
    /// load and store crosses does not restate it. A register access the EDMA3 sees is made
    /// for this identity, so every path that reaches its slave port must present its own
    /// first: the CPU through `check_access`, the system masters through
    /// `master_register_read`/`master_register_write`.
    edma_privilege: Privilege,
    /// Whether any megamodule's interrupt selector reads a combiner output, which the latched
    /// flags a pulse sets can change without any level source moving.
    combiner_in_use: bool,
    /// The ports this device has, in consult order, decided once from the profile.
    ///
    /// This is the device's peripheral address ownership: which ports exist, and who gets asked
    /// first. It used to be three hand-written lists - the read chain, the write chain and the
    /// no-two-ports-claim-one-address proof - that could only agree by someone remembering to
    /// edit all three.
    ports: Vec<Port>,
}

impl Shared {
    /// Which ports a device with this shape has, in consult order.
    fn port_order(profile: &'static DeviceProfile) -> Vec<Port> {
        let mut ports = Vec::with_capacity(32);
        ports.extend((0..profile.emac_bases.len()).map(Port::Emac));
        ports.extend((0..profile.emic_bases.len()).map(Port::Emic));
        ports.push(Port::Edma);
        ports.extend((0..profile.timer_bases.len()).map(Port::Timer));
        ports.extend([Port::Gpio, Port::I2c, Port::Pll, Port::Hpi, Port::Ddr2]);
        if profile.has_emifa {
            ports.push(Port::Emifa);
        }
        if profile.has_mcbsp {
            ports.extend((0..2).map(Port::Mcbsp));
        }
        if profile.utopia.is_some() {
            ports.push(Port::Utopia);
        }
        if profile.has_utopia_pdma() {
            ports.push(Port::Pdma);
        }
        if profile.has_device_config {
            ports.push(Port::Devcfg);
        }
        ports.push(Port::Pll2);
        ports.extend((0..4).map(Port::TransferController));
        ports.push(Port::Mdio);
        if profile.has_boot_controller {
            ports.extend([Port::BootController, Port::Psc, Port::Smc]);
            ports.extend((0..profile.cores()).map(Port::SmcProfiler));
            // The chip-level registers claim a whole window, so they answer last.
            ports.push(Port::ChipConfig);
        }
        ports
    }

    /// The port a name refers to, or `None` when the device does not have it - or has it but
    /// the fabric has lent it out: the EMAC and the EDMA3 are moved out of their slots for the
    /// duration of a DMA callback, and an access arriving then finds the slot empty.
    ///
    /// A trait object here rather than a match returning values, which is not the obvious
    /// choice and was measured both ways: a 21-arm match inlined into each walk costs 2.2% of
    /// the emulator's throughput against 1.0% for the virtual call, inlined or outlined. The
    /// register path probes ports in a loop, so the match's code size hurts more than the
    /// indirect call does.
    ///
    /// A SYSCLK3 block is handed out caught up through the edges the device has had, so the
    /// access sees and changes the state the per-edge model would have; once a port claims the
    /// access, `reschedule` plans the block again from what the access left.
    fn port(&mut self, which: Port) -> Option<&mut dyn SlavePort> {
        let edge = self.sysclk3_edge;
        Some(match which {
            Port::Emac(index) => self.emac[index].as_deref_mut()? as &mut dyn SlavePort,
            Port::Emic(index) => &mut self.emic[index],
            Port::Edma => self.edma.as_deref_mut()?,
            Port::Timer(index) => {
                let clocked = self.timer_clocked(index);
                self.timers[index].at(edge, clocked)
            }
            Port::Gpio => self.gpio.at(edge, true),
            Port::I2c => &mut self.i2c,
            Port::Pll => &mut self.pll,
            Port::Hpi => &mut self.hpi,
            Port::Ddr2 => &mut self.ddr2,
            Port::Emifa => self.emifa.as_mut()?,
            Port::Mcbsp(index) => &mut self.mcbsp[index],
            Port::Utopia => self.utopia.as_mut()?.at(edge, true),
            Port::Pdma => self.pdma.as_mut()?,
            Port::Devcfg => self.devcfg.as_mut()?.at(edge, true),
            Port::Pll2 => &mut self.pll2,
            Port::TransferController(index) => &mut self.transfer_controllers[index],
            Port::Mdio => &mut self.mdio,
            Port::BootController => self.bootctl.as_mut()?,
            Port::Psc => self.psc.as_mut()?,
            Port::Smc => self.smc.as_mut()?,
            Port::SmcProfiler(index) => &mut self.smc_profilers[index],
            Port::ChipConfig => self.chipcfg.as_mut()?,
        })
    }

    /// A port claimed an access. A SYSCLK3 block's register write - or a UTOPIA queue read -
    /// may have changed when it next does something, so it is planned again from the state the
    /// access left, and the deadline brought forward if that is sooner. The device
    /// configuration block is never due, whatever it holds.
    fn reschedule(&mut self, which: Port) {
        let due = match which {
            Port::Timer(index) => {
                let clocked = self.timer_clocked(index);
                let timer = &mut self.timers[index];
                timer.plan(clocked);
                timer.due()
            }
            Port::Gpio => {
                self.gpio.plan(true);
                self.gpio.due()
            }
            Port::Utopia => {
                let Some(utopia) = self.utopia.as_mut() else {
                    return;
                };
                utopia.plan(true);
                utopia.due()
            }
            _ => return,
        };
        self.sysclk3_deadline = self.sysclk3_deadline.min(due);
    }

    /// The earliest edge any stepped SYSCLK3 block is due on.
    fn next_sysclk3_due(&self) -> u64 {
        let timers = self.timers.iter().map(Clocked::due).min();
        let utopia = self.utopia.as_ref().map(Clocked::due);
        [timers, utopia]
            .into_iter()
            .flatten()
            .fold(self.gpio.due(), u64::min)
    }

    /// Whether a megamodule's functional clock runs: its module is in Enable (SPRUEG3B section
    /// 4.1.2); a device without a PSC always runs.
    fn core_clocked(&self, core: usize) -> bool {
        match self.psc.as_ref() {
            Some(psc) => psc.module_enabled(crate::psc::MODULE_GEM0 + core),
            None => true,
        }
    }

    /// Whether a timer is clocked. A megamodule's local timer runs on that megamodule's clock:
    /// "C64x+ Megamodule local timers are clock gated as well when the corresponding C64x+
    /// Megamodule is clock gated" (SPRUEG3B section 4.1), and a module outside Enable has its
    /// functional clocks gated (section 4.1.2). The shared timers, and every timer on a device
    /// with no megamodule to belong to, keep counting.
    fn timer_clocked(&self, index: usize) -> bool {
        index >= self.megamodule_timers || self.core_clocked(index)
    }

    /// Bring the megamodule timers through the edges they have had under the gating they had it
    /// with, before a PSC transition can change it.
    fn sync_megamodule_timers(&mut self) {
        for index in 0..self.megamodule_timers {
            let clocked = self.timer_clocked(index);
            self.timers[index].sync(self.sysclk3_edge, clocked);
        }
    }

    /// Plan the megamodule timers under the gating a PSC transition left.
    fn plan_megamodule_timers(&mut self) {
        for index in 0..self.megamodule_timers {
            let clocked = self.timer_clocked(index);
            self.timers[index].plan(clocked);
            self.sysclk3_deadline = self.sysclk3_deadline.min(self.timers[index].due());
        }
    }

    /// UINT, from a UTOPIA brought up to date first.
    fn utopia_interrupt_pending(&mut self) -> bool {
        let edge = self.sysclk3_edge;
        self.utopia
            .as_mut()
            .is_some_and(|utopia| utopia.at(edge, true).interrupt_pending())
    }

    /// An access left the memory windows, or a host delivered something: the next device tick
    /// re-derives the activity summary and the level sources instead of trusting the cache.
    #[inline(always)]
    fn touched(&mut self) {
        self.dynamic_idle = false;
        self.dynamic_active = u8::MAX;
        self.peripheral_dirty = true;
    }

    fn new(profile: &'static DeviceProfile, endian: Endian, boot_mode: u8) -> Self {
        // HPI boot is mode 1 on both devices. Whether the host's DSPINT ends it is the C6455's
        // rule (SPRS276M section 2.4); the C6472's host releases each core through the boot
        // controller instead (SPRUEC6G section 4.3.2).
        let hpi_boot = if boot_mode != crate::hpi::BOOTMODE_HPI {
            HpiBoot::Off
        } else if profile.has_boot_controller {
            HpiBoot::BootController
        } else {
            HpiBoot::DspintRelease
        };
        Self {
            backing: Backing::new(profile.map, endian),
            mdio: Mdio::with_version(profile.mdio_version),
            emac: profile
                .emac_bases
                .iter()
                .map(|base| Some(Box::new(Emac::new(*base, profile.emic_bases.is_empty()))))
                .collect(),
            emic: profile
                .emic_bases
                .iter()
                .map(|base| EmacInterruptControl::new(*base, profile.cores()))
                .collect(),
            edma: Some(Box::new(Edma::default())),
            timers: profile
                .timer_bases
                .iter()
                .map(|base| Clocked::new(Timer::new(*base)))
                .collect(),
            gpio: Clocked::new(Gpio::default()),
            i2c: I2c::default(),
            pll: Pll::default(),
            transfer_controllers: [
                TransferController::new(0, 0x02a2_0000),
                TransferController::new(1, 0x02a2_8000),
                TransferController::new(2, 0x02a3_0000),
                TransferController::new(3, 0x02a3_8000),
            ],
            hpi: Hpi::new(hpi_boot),
            ddr2: Ddr2::default(),
            emifa: profile.has_emifa.then(Emifa::default),
            utopia: profile
                .utopia
                .map(|utopia| Clocked::new(Utopia::new(utopia))),
            pdma: profile.has_utopia_pdma().then(Pdma::default),
            mcbsp: if profile.has_mcbsp {
                vec![
                    Mcbsp::new(0x028c_0000, 0x3000_0000, 0x3000_0010),
                    Mcbsp::new(0x0290_0000, 0x3400_0000, 0x3400_0010),
                ]
            } else {
                Vec::new()
            },
            devcfg: profile
                .has_device_config
                .then(|| Clocked::new(DeviceConfig::new(hpi_boot != HpiBoot::Off))),
            pll2: Pll2::default(),
            chipcfg: profile.has_boot_controller.then(|| {
                ChipConfig::new(
                    match endian {
                        Endian::Little => DEVSTAT_LITTLE_ENDIAN,
                        Endian::Big => 0,
                    } | u32::from(boot_mode & 0xf),
                    profile.jtag_id,
                )
            }),
            // The BOOTMODE pins decide which cores start: immediate boot releases them all,
            // and every ROM boot mode runs core 0 and leaves cores 1-5 for it to release
            // (SPRS612G section 2.4.1).
            bootctl: profile
                .has_boot_controller
                .then(|| BootController::new(boot_mode, profile.cores())),
            psc: profile.has_boot_controller.then(PowerSleep::booted),
            smc: profile.has_boot_controller.then(SharedMemory::default),
            smc_profilers: if profile.has_boot_controller {
                (0..profile.cores())
                    .map(SharedMemoryProfiler::new)
                    .collect()
            } else {
                Vec::new()
            },
            emacs: profile.emac_bases.len(),
            device_level_blocks: profile.has_boot_controller,
            sysclk3_divide: 0,
            sysclk3_edge: 0,
            // Every block starts due on the first edge (`Clocked::new`).
            sysclk3_deadline: 1,
            megamodule_timers: profile.megamodule_timers,
            last_tick_short: false,
            dynamic_idle: false,
            dynamic_active: u8::MAX,
            peripheral_dirty: true,
            levels_asserted: true,
            edma_privilege: Privilege::ID0_SUPERVISOR,
            combiner_in_use: true,
            ports: Self::port_order(profile),
        }
    }
}

/// The whole device: its megamodules and the peripherals they share.
pub struct Fabric {
    profile: &'static DeviceProfile,
    /// One box per megamodule: indexing a slice of the struct itself makes every access
    /// multiply by its size, and the access path runs several times per emulated cycle.
    mm: Box<[Box<Megamodule>]>,
    /// How many megamodules there are, so the per-cycle paths index rather than iterate.
    cores: usize,
    shared: Shared,
    /// The device cycle the shared peripherals have been advanced to. Each core asks the
    /// fabric to tick as it finishes its own cycle, and the first ask of a cycle is the one
    /// that runs: peripherals advance once per device cycle however many cores are running.
    ticked_cycle: Option<u64>,
    /// Cores whose NMI pin the chip-level registers have driven, and cores the boot controller
    /// or the power/sleep controller has released, each consumed once by the device that owns
    /// the CPUs (SPRS612G sections 3.7.1 and 2.4.1).
    pending_nmi: u32,
    pending_release: u32,
}

impl Default for Fabric {
    fn default() -> Self {
        Self::new(&crate::device::C6455)
    }
}

impl Fabric {
    pub(crate) fn new(profile: &'static DeviceProfile) -> Self {
        Self::with_boot_mode(profile, 0)
    }

    /// A device strapped to a boot mode, which is what decides how many cores start.
    pub(crate) fn with_boot_mode(profile: &'static DeviceProfile, boot_mode: u8) -> Self {
        Self {
            profile,
            mm: (0..profile.cores())
                .map(|core| Box::new(Megamodule::new(core, profile)))
                .collect::<Vec<_>>()
                .into_boxed_slice(),
            cores: profile.cores(),
            shared: Shared::new(profile, Endian::Little, boot_mode),
            ticked_cycle: None,
            pending_nmi: 0,
            pending_release: 0,
        }
    }

    #[cfg(test)]
    pub(crate) fn megamodule(&self, core: usize) -> &Megamodule {
        &self.mm[core]
    }

    /// A view of the fabric from one core, which is what the CPU holds while it runs.
    #[cfg(test)]
    pub(crate) fn core(&mut self, core: usize) -> CoreBus<'_> {
        CoreBus { core, fabric: self }
    }

    pub(crate) fn set_endian(&mut self, endian: Endian) {
        self.shared.backing.set_endian(endian);
    }

    pub(crate) fn endian(&self) -> Endian {
        self.shared.backing.endian()
    }

    pub(crate) fn profile(&self) -> &'static DeviceProfile {
        self.profile
    }

    /// Which cores the boot controller is holding in reset, as a bitmask. A held core does not
    /// execute, but every one of its memories still answers (SPRUEC6G section 4.3.2).
    pub(crate) fn held_in_reset(&self, core: usize) -> bool {
        self.mm[core].held_in_reset
    }

    pub(crate) fn set_held_in_reset(&mut self, core: usize, held: bool) {
        self.mm[core].held_in_reset = held;
    }

    /// A held core does not receive the events raised while it waits; see
    /// `drop_event_receptions`.
    pub(crate) fn drop_held_core_events(&mut self, core: usize) {
        self.mm[core].drop_event_receptions();
    }

    /// The address the boot controller starts a core at, when the device has one.
    pub(crate) fn boot_address(&self, core: usize) -> Option<u32> {
        Some(self.shared.bootctl.as_ref()?.boot_address(core))
    }

    // ------------------------------------------------------------------
    // The routing surface: a CPU access from one core.
    // ------------------------------------------------------------------

    #[inline(always)]
    fn port_context(&self) -> PortContext {
        PortContext {
            endian: if self.shared.backing.endian_big() {
                Endian::Big
            } else {
                Endian::Little
            },
        }
    }

    /// One core's memory hierarchy bound to the storage every master shares.
    ///
    /// This is where the device's ownership split is expressed: the caches belong to the
    /// megamodule, the bytes belong to the device, and the fabric is the only thing that holds
    /// both. Every access below binds them here instead of naming the store again.
    #[inline(always)]
    fn memory(&mut self, core: usize) -> BoundHierarchy<'_> {
        self.mm[core].memory.bind(&mut self.shared.backing)
    }

    #[inline(always)]
    pub(crate) fn read32(&mut self, core: usize, address: u32) -> u32 {
        if directly_routed_memory(address) {
            return self.memory(core).read_u32(address);
        }
        self.shared.touched();
        let context = self.port_context();
        if let Some(value) = self.peripheral_read(address, context) {
            return value;
        }
        if let Some(value) = self.mm[core].cfg_read(address, self.profile, context) {
            return value;
        }
        self.memory(core).read_u32(address)
    }

    #[inline(always)]
    pub(crate) fn write32(&mut self, core: usize, address: u32, value: u32) {
        if directly_routed_memory(address) {
            self.memory(core).write_u32(address, value);
            return;
        }
        self.shared.touched();
        let context = self.port_context();
        if self.peripheral_write(address, value, context) {
            return;
        }
        let written = {
            let (mm, shared) = (&mut self.mm[core], &mut self.shared);
            mm.cfg_write(&mut shared.backing, address, value, self.profile, context)
        };
        match written {
            Some(CfgWrite::Done) => return,
            Some(CfgWrite::Event(event)) => {
                self.mm[core].raise_event(event);
                return;
            }
            None => {}
        }
        self.memory(core).write_u32(address, value);
    }

    /// The MDIO interrupts an EMAC's MACINVECTOR reports alongside its own.
    ///
    /// The one shared register the router decodes itself, because it is the one no port can
    /// answer: it sits in the EMAC's register map but reports the MDIO's interrupts too, since
    /// only the control module sees both (SPRU975E section 2.16.4).
    fn mac_input_vector(&mut self, address: u32) -> Option<u32> {
        let index = self
            .profile
            .emac_bases
            .iter()
            .position(|base| address == base + 0x90)?;
        let mut vector = self.shared.emac[index]
            .as_mut()
            .map_or(0, |emac| emac.input_vector());
        if self.shared.mdio.link_interrupt_pending() {
            vector |= 1 << 30;
        }
        if self.shared.mdio.user_interrupt_pending() {
            vector |= 1 << 31;
        }
        Some(vector)
    }

    /// The shared peripherals' slave ports, in the order the router consults them.
    ///
    /// One walk of one list. Which ports exist and who answers first is the device's, decided
    /// once from its profile; what the router owes the megamodules afterwards belongs to the
    /// port that was hit rather than to a position in a chain.
    fn peripheral_read(&mut self, address: u32, context: PortContext) -> Option<u32> {
        if let Some(vector) = self.mac_input_vector(address) {
            return Some(vector);
        }
        for index in 0..self.shared.ports.len() {
            let which = self.shared.ports[index];
            let Some(port) = self.shared.port(which) else {
                continue;
            };
            if let Some(value) = port.read(address, context) {
                self.shared.reschedule(which);
                self.follow(which.after_read());
                return Some(value);
            }
        }
        None
    }

    /// Which port answers for this address, without performing the access's follow-up.
    ///
    /// Peripheral address ownership is a device property, and this is how a case asks about it:
    /// what a C6472 puts at an address is not what a C6455 puts there, and before the ports
    /// were one list the only way to find out was to boot a device and infer it from what the
    /// register did.
    #[cfg(test)]
    fn owner_of(&mut self, address: u32) -> Option<Port> {
        let context = self.port_context();
        for index in 0..self.shared.ports.len() {
            let which = self.shared.ports[index];
            let Some(port) = self.shared.port(which) else {
                continue;
            };
            if port.read(address, context).is_some() {
                self.shared.reschedule(which);
                return Some(which);
            }
        }
        None
    }

    fn peripheral_write(&mut self, address: u32, value: u32, context: PortContext) -> bool {
        for index in 0..self.shared.ports.len() {
            let which = self.shared.ports[index];
            let Some(port) = self.shared.port(which) else {
                continue;
            };
            if port.write(address, value, context) != PortWrite::Unhandled {
                self.shared.reschedule(which);
                self.follow(which.after_write());
                return true;
            }
        }
        false
    }

    /// Perform what a port's access left for the router.
    fn follow(&mut self, follow: Follow) {
        match follow {
            Follow::Nothing => {}
            Follow::EmacInterrupts => {
                self.latch_emac_interrupts();
            }
            Follow::EdmaInterrupt => {
                self.latch_edma_interrupt();
            }
            Follow::I2cInterrupt => self.latch_i2c_interrupt(),
            Follow::UtopiaInterrupt => {
                self.latch_utopia_interrupt();
            }
            Follow::TransferControllerErrors => self.latch_transfer_controller_errors(),
            Follow::ReleaseBootedCores => self.release_booted_cores(),
            Follow::PowerTransitions => self.apply_power_transitions(),
            // The model holds no prefetched data, so a flush has nothing to discard; consuming
            // the edge keeps the register's write-one-shot semantics honest (SPRUEG5D 2.8).
            Follow::PrefetchFlush => {
                if let Some(smc) = self.shared.smc.as_mut() {
                    let _ = smc.take_prefetch_flush();
                }
            }
            Follow::InterProcessorInterrupts => self.deliver_inter_processor_interrupts(),
        }
    }

    // ------------------------------------------------------------------
    // Device-level effects of a peripheral write.
    // ------------------------------------------------------------------

    /// An IPCGR write raises system event 84 in the core it names, and an NMIGR write drives
    /// that core's NMI pin (SPRS612G section 3.7).
    fn deliver_inter_processor_interrupts(&mut self) {
        let Some(chip) = self.shared.chipcfg.as_mut() else {
            return;
        };
        let pulses = chip.take_ipc_pulse();
        let nmi = chip.take_nmi_pulse();
        if pulses == 0 && nmi == 0 {
            return;
        }
        let Some(event) = self.profile.events.ipc else {
            return;
        };
        for core in 0..self.mm.len() {
            if pulses & (1 << core) != 0 {
                self.mm[core].raise_event(event);
            }
        }
        // The NMI is a pin, not an event: the device raises it on the core's CPU, which the
        // machine reads back through `take_nmi_requests`.
        self.pending_nmi |= nmi;
    }

    /// Cores the boot controller has just released start executing at their DSP_BOOT_ADDR.
    fn release_booted_cores(&mut self) {
        let released = self
            .shared
            .bootctl
            .as_mut()
            .map_or(0, BootController::take_released_cores);
        self.pending_release |= released;
    }

    /// A local reset the power/sleep controller asserted or released.
    fn apply_power_transitions(&mut self) {
        let Some(psc) = self.shared.psc.as_mut() else {
            return;
        };
        for (module, asserted) in psc.take_local_reset_edges() {
            if module < self.mm.len() {
                self.mm[module].held_in_reset = asserted;
                if !asserted {
                    self.pending_release |= 1 << module;
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // The device tick.
    // ------------------------------------------------------------------

    /// Advance every cycle-driven peripheral by one CPU cycle, once per device cycle however
    /// many cores are running.
    ///
    /// Each core asks as it finishes a cycle of its own, stalls included, and the first ask of
    /// a cycle is the one that runs. Cores that stall drift behind the others and simply skip
    /// their ask until they catch up, so the peripherals advance at the rate of the fastest
    /// core rather than six times per cycle.
    /// Answers whether this was a *short* tick: the peripherals did not move, so the asking
    /// core reads the interrupt lines the last full tick selected instead of fresh ones.
    #[inline(always)]
    pub(crate) fn tick(&mut self, cycle: u64) -> bool {
        if self.ticked_cycle.is_some_and(|last| cycle <= last) {
            // Another core already advanced the device this cycle; this core sees whatever
            // that ask decided.
            return self.shared.last_tick_short;
        }
        self.ticked_cycle = Some(cycle);
        self.shared.sysclk3_divide += 1;
        let sysclk3 = self.shared.sysclk3_divide == SYSCLK3_DIVIDER;
        if !sysclk3 && self.shared.dynamic_idle {
            self.shared.last_tick_short = true;
            return true;
        }
        if sysclk3 {
            self.shared.sysclk3_divide = 0;
        }
        self.shared.last_tick_short = false;
        self.tick_device(sysclk3);
        false
    }

    /// Advance the device by one cycle for a driver that has no CPU of its own - the
    /// conformance vectors and the unit tests - which therefore has no cycle number to ask
    /// with.
    #[cfg(test)]
    pub(crate) fn tick_once(&mut self) {
        let next = self.ticked_cycle.map_or(0, |last| last.wrapping_add(1));
        self.tick(next);
    }

    /// Lend a bus master out for the duration of one of its own transfers.
    ///
    /// A master that moves data - either Ethernet MAC, the EDMA3, the HPI's host side - reaches
    /// memory *and registers* through the whole fabric, because a DMA access inside a peripheral
    /// aperture goes to that peripheral's slave port with every side effect a CPU access would
    /// have. So the callback needs `&mut Fabric`, which cannot be held while a borrow of the
    /// master living inside it is also held. The master is moved out of its slot for the call
    /// and moved back after.
    ///
    /// It is pointer-sized while it travels, which is the whole reason for the box: `Emac` is
    /// 840 bytes and `Edma` a thousand. The protocol costs two pointer writes per active master
    /// per device cycle.
    ///
    /// The consequence worth knowing is that the slot is empty while the callback runs, so a
    /// master is not addressable by its own transfer: an access to its register block finds no
    /// port and falls through to the memory behind the aperture.
    /// `a_lent_out_master_does_not_answer_its_own_transfer` pins that.
    ///
    /// Splitting the borrow instead - handing the callback only the parts it needs - would mean
    /// moving the whole router onto a view over borrowed parts: both dispatch walks, the port
    /// map, the follow-ups and the latchers, because Rust cannot express "all of the fabric
    /// except this one slot". That is a larger change than the protocol it would remove, it
    /// would cost the router the locality it has, and it would recover two pointer writes.
    #[inline(always)]
    fn lend_emac<T>(&mut self, index: usize, call: impl FnOnce(&mut Emac, &mut Self) -> T) -> T {
        let mut emac = self.shared.emac[index].take().unwrap();
        let result = call(&mut emac, self);
        self.shared.emac[index] = Some(emac);
        result
    }

    /// Lend the EDMA3 out for one of its transfers; see `lend_emac` for why.
    #[inline(always)]
    fn lend_edma<T>(&mut self, call: impl FnOnce(&mut Edma, &mut Self) -> T) -> T {
        let mut edma = self.shared.edma.take().unwrap();
        let result = call(&mut edma, self);
        self.shared.edma = Some(edma);
        result
    }

    /// Lend the HPI's host side out for one host cycle; see `lend_emac` for why. The HPI is not
    /// boxed because a host cycle is polled rather than per-cycle work.
    fn lend_hpi<T>(&mut self, call: impl FnOnce(&mut Hpi, &mut Self) -> T) -> T {
        let mut hpi = std::mem::take(&mut self.shared.hpi);
        let result = call(&mut hpi, self);
        self.shared.hpi = hpi;
        result
    }

    /// One full device cycle: everything a peripheral does, and the two summaries the short
    /// tick above trusts. Deliberately out of line, so the dormant path does not build its
    /// frame.
    #[inline(never)]
    fn tick_device(&mut self, sysclk3: bool) {
        // Whether any cycle-driven peripheral actually moved. Together with `peripheral_dirty`
        // - set by every access that left the memory windows and by every frame a host
        // delivered - it decides whether the level-triggered sources have to be interrogated
        // again, or whether last cycle's answer still holds. A dormant device answers "nothing
        // asserted" for millions of cycles, and asking six peripherals to confirm it costs
        // more than the tick it guards.
        //
        // The activity summary is maintained the same way, one bit at a time: a peripheral
        // whose bit is clear was inactive and only a register write can arm it, and a write
        // sets every bit through `Shared::touched`. So each branch below recomputes its own
        // bit whenever it was considered, and nothing scans the peripherals it did not touch.
        let entered_dirty = std::mem::take(&mut self.shared.peripheral_dirty);
        let previous = self.shared.dynamic_active;
        let mut active = previous & ACTIVE_ALL;
        let mut moved = false;
        if previous & ACTIVE_IDMA != 0 {
            self.tick_idma();
            let mut idma = false;
            for core in 0..self.cores {
                idma |= self.mm[core].idma.active();
            }
            active = (active & !ACTIVE_IDMA) | (u8::from(idma) * ACTIVE_IDMA);
            moved = true;
        }
        if previous & ACTIVE_EMAC != 0 {
            let mut emac_active = false;
            for index in 0..self.shared.emacs {
                if self.shared.emac[index].as_ref().unwrap().active() {
                    self.lend_emac(index, |emac, fabric| emac.tick(&mut EmacMaster(fabric)));
                    moved = true;
                }
                emac_active |= self.shared.emac[index].as_ref().unwrap().active();
            }
            active = (active & !ACTIVE_EMAC) | (u8::from(emac_active) * ACTIVE_EMAC);
        }
        if previous & ACTIVE_EDMA != 0 {
            if self.shared.edma.as_ref().unwrap().active() {
                self.lend_edma(|edma, fabric| edma.tick(fabric));
                self.latch_transfer_controller_errors();
                moved = true;
            }
            let edma_active = self.shared.edma.as_ref().unwrap().active();
            active = (active & !ACTIVE_EDMA) | (u8::from(edma_active) * ACTIVE_EDMA);
        }
        // The SYSCLK3 blocks step only on the edges the schedule has them due: on the others
        // they would raise nothing, trigger nothing and add nothing to `moved`, and whatever
        // they count is caught up when something reads it (`sysclk3.rs`). The edge is counted
        // here, where the timers step, so the masters that ran above saw the edge before.
        let mut scheduled = false;
        if sysclk3 {
            self.shared.sysclk3_edge += 1;
            scheduled = self.shared.sysclk3_edge >= self.shared.sysclk3_deadline;
            if scheduled {
                moved |= self.tick_timers();
                moved |= self.tick_gpio();
            }
        }
        if previous & ACTIVE_I2C != 0 {
            if self.shared.i2c.active() {
                self.tick_i2c();
                moved = true;
            }
            let i2c_active = self.shared.i2c.transfer_active();
            active = (active & !ACTIVE_I2C) | (u8::from(i2c_active) * ACTIVE_I2C);
        }
        if scheduled {
            if self.tick_utopia() {
                moved = true;
            }
            self.shared.sysclk3_deadline = self.shared.next_sysclk3_due();
        }
        if previous & ACTIVE_MCBSP != 0 {
            if self.shared.mcbsp.iter().any(Mcbsp::active) {
                self.tick_mcbsp(sysclk3);
                moved = true;
            }
            let mcbsp_active = self.shared.mcbsp.iter().any(Mcbsp::active);
            active = (active & !ACTIVE_MCBSP) | (u8::from(mcbsp_active) * ACTIVE_MCBSP);
        }
        if previous & ACTIVE_MDIO != 0 {
            if self.shared.mdio.active() {
                self.shared.mdio.tick();
                moved = true;
            }
            let mdio_active = self.shared.mdio.active();
            active = (active & !ACTIVE_MDIO) | (u8::from(mdio_active) * ACTIVE_MDIO);
        }
        if self.shared.device_level_blocks {
            self.tick_device_blocks();
            moved = true;
        }
        // A timer, the GPIO pins, the I2C, a McBSP or the UTOPIA port arms an EDMA channel by
        // triggering its synchronisation event, so a tick that moved anything has to ask the
        // channel controller again - it is the one peripheral another peripheral can wake.
        if moved {
            active |= u8::from(self.shared.edma.as_ref().unwrap().active()) * ACTIVE_EDMA;
        }
        // A transfer this tick performed may itself have reached a peripheral register, which
        // wakes every bit through `Shared::touched`. The flag was cleared before the branches
        // ran, so finding it set again means the per-branch answers are stale and everything
        // has to be considered next cycle.
        let woken = self.shared.peripheral_dirty;
        self.shared.peripheral_dirty = false;
        self.shared.dynamic_active = if woken { ACTIVE_ALL } else { active };
        self.shared.dynamic_idle = self.shared.dynamic_active == 0;
        let dirty = moved || entered_dirty || woken;
        // A level has to be relatched every cycle it asserts, and a combiner-routed selector
        // reads the latched flags a pulse may have just set, so both keep the interrogation
        // running even when nothing moved. Each latcher reports whether it raised anything,
        // which is the same question as "must this run again next cycle" and costs nothing
        // beyond the interrogation it already did.
        if dirty || self.shared.levels_asserted || self.shared.combiner_in_use {
            let mut asserted = self.latch_utopia_interrupt();
            asserted |= self.latch_edma_interrupt();
            asserted |= self.latch_emac_interrupts();
            asserted |= self.refresh_idle_level_events();
            self.shared.levels_asserted = asserted;
            self.shared.combiner_in_use =
                (0..self.cores).any(|core| self.mm[core].selector_uses_combiner);
        }
    }

    /// The blocks only the C6472 has: the interrupt-control modules, the power/sleep
    /// controller and the shared-memory controller with its per-core profilers.
    fn tick_device_blocks(&mut self) {
        for index in 0..self.shared.emic.len() {
            self.shared.emic[index].tick();
        }
        if let Some(psc) = self.shared.psc.as_ref() {
            // A GO this tick completes can take a megamodule in or out of Enable, which gates
            // its local timer: the schedule brings those timers through the edges they have had
            // under the old gating first, and plans them under the new one after.
            let regating = psc.transition_pending();
            if regating {
                self.shared.sync_megamodule_timers();
            }
            let psc = self.shared.psc.as_mut().unwrap();
            psc.tick();
            let interrupt = psc.interrupt_pending();
            if regating {
                self.shared.plan_megamodule_timers();
            }
            if interrupt {
                if let Some(event) = self.profile.events.psc {
                    self.raise(Destination::All, event);
                }
            }
            self.apply_power_transitions();
        }
        if self
            .shared
            .smc
            .as_ref()
            .is_some_and(SharedMemory::fault_pending)
        {
            if let Some(event) = self.profile.events.smc_fault {
                self.raise(Destination::All, event);
            }
        }
        for core in 0..self.shared.smc_profilers.len() {
            if self.shared.smc_profilers[core].take_event() {
                if let Some(event) = self.profile.events.smc_event {
                    self.raise(Destination::Core(core), event);
                }
            }
        }
    }

    /// Each megamodule's own IDMA moves data between its local memories.
    fn tick_idma(&mut self) {
        for core in 0..self.cores {
            if !self.mm[core].idma.active() {
                continue;
            }
            let steps = self.mm[core].idma.tick();
            for step in steps.into_iter().flatten() {
                let value = match step.source {
                    Some(IdmaSource::Address(address)) => self.master_read32(address),
                    Some(IdmaSource::Fill(value)) => Some(value),
                    None => None,
                };
                if let Some(value) = value {
                    self.master_write32(step.destination, value);
                }
                if let Some(event) = step.completion_event {
                    self.mm[core].raise_event(event);
                }
            }
        }
    }

    /// Raise a system event in whichever megamodules the device map says can see it.
    fn raise(&mut self, destination: Destination, event: usize) {
        match destination {
            Destination::Core(core) => {
                if let Some(mm) = self.mm.get_mut(core) {
                    mm.raise_event(event);
                }
            }
            Destination::All => {
                for core in 0..self.cores {
                    self.mm[core].raise_event(event);
                }
            }
        }
    }

    /// Recompute the level-triggered event set the dormant short tick replays, and answer
    /// whether anything is in it.
    fn refresh_idle_level_events(&mut self) -> bool {
        let mut shared_events = [0u32; 4];
        let mut steered: Vec<(usize, usize)> = Vec::new();
        {
            let mut note = |event: usize| shared_events[event / 32] |= 1 << (event % 32);
            for index in 0..self.shared.emac.len() {
                let emac = self.shared.emac[index].as_ref().unwrap();
                if emac.combined_interrupt_pending(self.shared.mdio.interrupt_pending()) {
                    note(self.profile.events.emac[index].common);
                }
            }
            if self.shared.edma.as_ref().unwrap().interrupt_pending() {
                note(self.profile.events.edma_global);
            }
            if self.shared.utopia_interrupt_pending() {
                note(self.profile.events.utopia);
            }
            if self.shared.i2c.interrupt_pending() {
                note(self.profile.events.i2c);
            }
        }
        if self.shared.edma.as_ref().unwrap().interrupt_pending() {
            for region in 0..8 {
                if !self
                    .shared
                    .edma
                    .as_mut()
                    .unwrap()
                    .region_interrupt_pending(region)
                {
                    continue;
                }
                match self.profile.events.edma_region_completion(region) {
                    Some((Destination::All, event)) => {
                        shared_events[event / 32] |= 1 << (event % 32);
                    }
                    Some((Destination::Core(core), event)) => steered.push((core, event)),
                    None => {}
                }
            }
        }
        let asserted = shared_events != [0; 4] || !steered.is_empty();
        if self.cores == 1 && steered.is_empty() {
            let mm = &mut self.mm[0];
            mm.idle_level_events = shared_events;
            mm.idle_cpu_lines = if shared_events == [0; 4] && !mm.selector_uses_combiner {
                0
            } else {
                mm.selected_cpu_interrupts(&shared_events)
            };
            return asserted;
        }
        for core in 0..self.cores {
            let mut events = shared_events;
            if !steered.is_empty() {
                for (target, event) in &steered {
                    if *target == core {
                        events[event / 32] |= 1 << (event % 32);
                    }
                }
            }
            let mm = &mut self.mm[core];
            mm.idle_level_events = events;
            // With no level events and no combiner-routed event, the selected line set is empty
            // without walking the twelve mux slots (SPRU871K 7.2.2, 7.4.2).
            mm.idle_cpu_lines = if events == [0; 4] && !mm.selector_uses_combiner {
                0
            } else {
                mm.selected_cpu_interrupts(&events)
            };
        }
        asserted
    }

    /// The Ethernet interrupts.
    ///
    /// On a device with no interrupt-control module the MAC and the MDIO share one event, gated
    /// by EWCTL inside the MAC itself (SPRU975E section 2.16.4). On the C6472 each MAC has an
    /// EMIC that gates its eighteen sources per core and drives three events - receive,
    /// transmit and common (SPRUEF8F section 3.1) - so the same sources reach different cores
    /// depending on what each core enabled.
    /// Answers whether it asserted anything, which is the same question as whether it has to
    /// run again next cycle.
    fn latch_emac_interrupts(&mut self) -> bool {
        if self.shared.emic.is_empty() {
            // One MAC, one event, gated inside the MAC itself.
            let pending = self.shared.emac[0]
                .as_ref()
                .unwrap()
                .combined_interrupt_pending(self.shared.mdio.interrupt_pending());
            if pending {
                self.raise(Destination::All, self.profile.events.emac[0].common);
            }
            return pending;
        }
        let mdio_link = self.shared.mdio.link_interrupt_pending();
        let mdio_user = self.shared.mdio.user_interrupt_pending();
        let mut asserted = false;
        for index in 0..self.shared.emacs {
            let events = self.profile.events.emac[index];
            let (rx, tx, stat, host) = self.shared.emac[index].as_ref().unwrap().masked_sources();
            let sources = EmacSources {
                rx,
                tx,
                stat,
                host,
                mdio_link,
                mdio_user,
            };
            self.shared.emic[index].update(sources);
            for core in 0..self.mm.len() {
                let lines = self.shared.emic[index].lines(core);
                if lines.rx {
                    asserted = true;
                    if let Some(event) = events.receive {
                        self.mm[core].raise_event(event);
                    }
                }
                if lines.tx {
                    asserted = true;
                    if let Some(event) = events.transmit {
                        self.mm[core].raise_event(event);
                    }
                }
                if lines.common {
                    asserted = true;
                    self.mm[core].raise_event(events.common);
                }
            }
        }
        asserted
    }

    /// The EDMA3 channel controller's global completion, its per-region completions, its error
    /// interrupt and its memory protection interrupt (SPRU966C Tables 2-10 and 2-11; SPRS612G
    /// Table 7-10 for the C6472's numbers and per-core steering).
    ///
    /// Answers whether the level-triggered half of that - the completions - is still asserting.
    fn latch_edma_interrupt(&mut self) -> bool {
        let pending = self.shared.edma.as_mut().unwrap().interrupt_pending();
        if pending {
            self.raise(Destination::All, self.profile.events.edma_global);
            for region in 0..8 {
                if self
                    .shared
                    .edma
                    .as_mut()
                    .unwrap()
                    .region_interrupt_pending(region)
                {
                    if let Some((destination, event)) =
                        self.profile.events.edma_region_completion(region)
                    {
                        self.raise(destination, event);
                    }
                }
            }
        }
        if self.shared.edma.as_mut().unwrap().take_error_pulse() {
            self.raise(Destination::All, self.profile.events.edma_error);
        }
        if self.shared.edma.as_mut().unwrap().take_fault_pulse() {
            self.raise(Destination::All, self.profile.events.edma_mp);
        }
        pending
    }

    /// Step the timers due on this SYSCLK3 edge, in device order, reporting whether any period
    /// match fired: a match raises a CPU event and may arm an EDMA channel, so the tick has to
    /// look at the device again afterwards. A timer that is not due would raise nothing here.
    fn tick_timers(&mut self) -> bool {
        let edge = self.shared.sysclk3_edge;
        let mut fired = false;
        for index in 0..self.shared.timers.len() {
            if self.shared.timers[index].due() > edge {
                continue;
            }
            // A gated megamodule timer lets the edge pass without counting (`timer_clocked`).
            let clocked = self.shared.timer_clocked(index);
            let raised = self.shared.timers[index].step(edge, clocked, Timer::tick_sysclk3);
            fired |= raised[0] || raised[1];
            for (half, raised) in raised.into_iter().enumerate() {
                if !raised {
                    continue;
                }
                let Some((destination, event, channel)) = self.profile.events.timer(index, half)
                else {
                    continue;
                };
                self.raise(destination, event);
                if let Some(channel) = channel {
                    self.shared.edma.as_mut().unwrap().trigger_event(channel);
                }
            }
        }
        fired
    }

    /// Sample the GPIO pins if they changed since the detector last looked, reporting whether
    /// any edge was detected.
    fn tick_gpio(&mut self) -> bool {
        let edge = self.shared.sysclk3_edge;
        if self.shared.gpio.due() > edge {
            return false;
        }
        let raised = self.shared.gpio.step(edge, true, Gpio::tick_sysclk3);
        if raised == 0 {
            return false;
        }
        for pin in 0..16 {
            if raised & (1 << pin) == 0 {
                continue;
            }
            let Some((destination, event, channel)) = self.profile.events.gpio(pin) else {
                continue;
            };
            self.raise(destination, event);
            if let Some(channel) = channel {
                self.shared.edma.as_mut().unwrap().trigger_event(channel);
            }
        }
        true
    }

    /// Put a serial EEPROM on the I2C bus, which is what a board does for the ROM's I2C boot
    /// (SPRUEC6G section 2.3.4).
    pub(crate) fn attach_i2c_eeprom(&mut self, address: u32, contents: Vec<u8>) {
        self.shared.i2c.attach_eeprom(address, contents);
    }

    fn tick_i2c(&mut self) {
        let events = self.shared.i2c.tick();
        let (receive, transmit) = self.profile.events.i2c_channels;
        if events.receive {
            self.shared.edma.as_mut().unwrap().trigger_event(receive);
        }
        if events.transmit {
            self.shared.edma.as_mut().unwrap().trigger_event(transmit);
        }
        self.latch_i2c_interrupt();
    }

    /// One CPU cycle of both ports; their sample rate generators count SYSCLK3 edges, the
    /// CPU/6 clock SPRU580G section 4.3 gives the C6455's McBSPs.
    fn tick_mcbsp(&mut self, sysclk3: bool) {
        for index in 0..self.shared.mcbsp.len() {
            let events = self.shared.mcbsp[index].tick(sysclk3);
            let Some([receive, transmit]) = self.profile.events.mcbsp(index) else {
                continue;
            };
            if events.receive {
                self.shared.edma.as_mut().unwrap().trigger_event(receive.1);
            }
            if events.transmit {
                self.shared.edma.as_mut().unwrap().trigger_event(transmit.1);
            }
            if events.receive_interrupt {
                self.raise(Destination::All, receive.0);
            }
            if events.transmit_interrupt {
                self.raise(Destination::All, transmit.0);
            }
        }
    }

    /// One SYSCLK3 edge of the UTOPIA port if it is due, reporting whether it moved: a
    /// clock-detect section that failed latches an error the interrupt is a level on, and a
    /// cell either way arms an EDMA channel - or, on a device whose UTOPIA has a PDMA, is
    /// moved by it. A port with both directions disabled cannot do either, and a port that is
    /// not due does neither and has no interrupt asserted.
    fn tick_utopia(&mut self) -> bool {
        let edge = self.shared.sysclk3_edge;
        let Some(utopia) = self.shared.utopia.as_mut() else {
            return false;
        };
        if utopia.due() > edge {
            return false;
        }
        let events = utopia.step(edge, true, Utopia::tick_sysclk3);
        let moved = events.receive || events.transmit || utopia.at(edge, true).interrupt_pending();
        if let Some((receive, transmit)) = self.profile.events.utopia_pdma {
            if events.receive {
                self.service_utopia_receive(receive);
            }
            if events.transmit {
                self.service_utopia_transmit(transmit);
            }
            return moved;
        }
        let Some((receive, transmit)) = self.profile.events.utopia_channels else {
            return moved;
        };
        if events.receive {
            self.shared.edma.as_mut().unwrap().trigger_event(receive);
        }
        if events.transmit {
            self.shared.edma.as_mut().unwrap().trigger_event(transmit);
        }
        moved
    }

    /// UREVT is asserted: the PDMA takes the event if a receive channel is set up for it, reads
    /// one burst out of the receive queue, writes it through the switch into memory the way any
    /// DMA master does, and raises `interrupt` in the core the channel's SINT names when the
    /// transfer control asks for one (SPRUEG2E sections 4.3 and 6.2). Memory the DMA
    /// protection refuses is not written. The UTOPIA is planned again from the queue the burst
    /// left, because the read cleared the event.
    fn service_utopia_receive(&mut self, interrupt: usize) {
        let edge = self.shared.sysclk3_edge;
        let (Some(utopia), Some(pdma)) = (self.shared.utopia.as_mut(), self.shared.pdma.as_mut())
        else {
            return;
        };
        let utopia_block = utopia.at(edge, true);
        let event = utopia_block
            .receive_event_route()
            .map(|id| crate::pdma::ReceiveEvent {
                id,
                generation: utopia_block.receive_generation(),
            });
        let Some(burst) = pdma.receive_event(event) else {
            return;
        };
        let mut words = [0u32; MAX_BURST_WORDS];
        for word in &mut words[..burst.words] {
            *word = utopia_block.read_receive_queue();
        }
        utopia.plan(true);
        let switch = pdma.advance(burst);
        // The PDMA is a switch master like any other: memory under protection and the snoop
        // protocol, a register through its port. UTOPIA transfers are always user mode, with
        // PrivID 0 (SPRS612G section 5.2: "For most peripheral masters (EMAC0, EMAC1, UTOPIA,
        // TSIP0, TSIP1, and TSIP2), the privilege mode is always user mode").
        for (&address, &word) in switch.addresses().iter().zip(&words) {
            self.pdma_write_element(address, word, switch.element);
        }
        if let Some(core) = switch.interrupt {
            self.raise(Destination::Core(core), interrupt);
        }
    }

    /// UXEVT is asserted: the PDMA takes the event if a transmit channel is set up for it and
    /// the queue can take the whole burst, reads the burst through the switch out of memory -
    /// as PrivID 0 in user mode, like the receive side, so a word protection refuses reads as
    /// zero and records the fault - writes it into the transmit queue, and raises `interrupt` in
    /// the core the channel's SINT names when the transfer control asks for one (SPRUEG2E
    /// sections 3.3, 4.4 and 6.2). That interrupt output is also the EDMA3 synchronisation
    /// event of its channel (SPRS612G Table 7-4). The UTOPIA is planned again, because the
    /// first write of a cell cleared the event.
    fn service_utopia_transmit(&mut self, interrupt: usize) {
        let edge = self.shared.sysclk3_edge;
        let (Some(utopia), Some(pdma)) = (self.shared.utopia.as_mut(), self.shared.pdma.as_mut())
        else {
            return;
        };
        let Some(burst) = pdma.transmit_event(utopia.at(edge, true).transmit_event()) else {
            return;
        };
        let switch = pdma.advance(burst);
        let mut words = [0u32; MAX_BURST_WORDS];
        for (word, &address) in words.iter_mut().zip(switch.addresses()) {
            *word = self.pdma_read_element(address, switch.element);
        }
        let utopia = self.shared.utopia.as_mut().unwrap();
        let utopia_block = utopia.at(edge, true);
        for &word in &words[..burst.words] {
            utopia_block.write_transmit_queue(word);
        }
        utopia.plan(true);
        if let Some(core) = switch.interrupt {
            self.raise(Destination::Core(core), interrupt);
            if let Some(channel) = self.profile.events.pdma_transmit_channel(core) {
                self.shared.edma.as_mut().unwrap().trigger_event(channel);
            }
        }
    }

    /// One switch element of a PDMA burst, a transfer of PrivID 0 in user mode. A 32-bit element
    /// is the word access every other master makes; a narrower one is the low bytes of the
    /// word - right justified, as Table 32 puts it - stored or fetched in the device's byte
    /// order, with a register reached the way a byte-moving master reaches it.
    fn pdma_write_element(&mut self, address: u32, word: u32, element: u32) {
        if element == 4 {
            self.master_write32_as(address, word, true);
            return;
        }
        let element = element as usize;
        let (big, little) = (word.to_be_bytes(), word.to_le_bytes());
        let bytes = if self.shared.backing.endian_big() {
            &big[4 - element..]
        } else {
            &little[..element]
        };
        self.requestor_write_bytes(address, bytes, Privilege::ID0_USER);
    }

    /// The element `pdma_write_element` stores, zero-extended into a word; a refused read is
    /// zero.
    fn pdma_read_element(&mut self, address: u32, element: u32) -> u32 {
        if element == 4 {
            return self.master_read32_as(address, true).unwrap_or(0);
        }
        let bytes = self.requestor_read_bytes(address, element as usize, Privilege::ID0_USER);
        let mut word = [0; 4];
        if self.shared.backing.endian_big() {
            word[4 - bytes.len()..].copy_from_slice(&bytes);
            u32::from_be_bytes(word)
        } else {
            word[..bytes.len()].copy_from_slice(&bytes);
            u32::from_le_bytes(word)
        }
    }

    /// The UTOPIA error interrupt, which is a level, so it answers whether it is asserting.
    fn latch_utopia_interrupt(&mut self) -> bool {
        let pending = self.shared.utopia_interrupt_pending();
        if pending {
            self.raise(Destination::All, self.profile.events.utopia);
        }
        pending
    }

    fn latch_transfer_controller_errors(&mut self) {
        if let Some((controller, detail)) = self.shared.edma.as_mut().unwrap().take_transfer_error()
        {
            if let Some(controller) = self.shared.transfer_controllers.get_mut(controller) {
                controller.report_transfer_error(detail);
            }
        }
        // A serviced request stays visible in the controller's debug windows.
        if let Some((controller, request)) =
            self.shared.edma.as_mut().unwrap().take_serviced_request()
        {
            if let Some(controller) = self.shared.transfer_controllers.get_mut(controller) {
                controller.record_request(request);
            }
        }
        for index in 0..self.shared.transfer_controllers.len() {
            if self.shared.transfer_controllers[index].take_error_pulse() {
                let (destination, event) = self.profile.events.transfer_controller_error(index);
                self.raise(destination, event);
            }
        }
    }

    fn latch_i2c_interrupt(&mut self) {
        if self.shared.i2c.interrupt_pending() {
            self.raise(Destination::All, self.profile.events.i2c);
        }
    }

    // ------------------------------------------------------------------
    // The host side.
    // ------------------------------------------------------------------

    /// Whether an external host's access can reach `address` (`DeviceProfile::host_excluded`).
    fn hpi_accessible(&self, address: u32) -> bool {
        !self
            .profile
            .host_excluded
            .iter()
            .any(|window| window.contains(&address))
    }

    /// A denied host read does not reach a peripheral. HPI completes it with zero:
    /// a deterministic emulator policy, not a hardware read value specified by TI.
    pub(crate) fn hpi_read32(&mut self, address: u32) -> Option<u32> {
        if !self.hpi_accessible(address) {
            return None;
        }
        self.master_read32(address)
    }

    pub(crate) fn hpi_write32(&mut self, address: u32, value: u32) {
        if self.hpi_accessible(address) {
            self.master_write32(address, value);
        }
    }

    /// An external host's read cycle on the HPI. Outside the profile's host-excluded windows the
    /// transfer goes through the same DMA path every other master uses, so the snoop rules apply.
    pub fn hpi_host_read(&mut self, port: HostPort) -> Option<u32> {
        self.lend_hpi(|hpi, fabric| hpi.host_read(port, fabric))
    }

    /// One halfword of a 16-bit multiplexed host cycle.
    #[cfg(test)]
    pub fn hpi_host_read_halfword(&mut self, port: HostPort, second: bool) -> Option<u16> {
        self.lend_hpi(|hpi, fabric| hpi.host_read_halfword(port, second, fabric))
    }

    #[cfg(test)]
    pub fn hpi_host_write_halfword(&mut self, port: HostPort, value: u16, second: bool) {
        let interrupt =
            self.lend_hpi(|hpi, fabric| hpi.host_write_halfword(port, value, second, fabric));
        if interrupt {
            self.host_interrupt();
        }
    }

    /// An external host's write cycle. A zero-to-one transition on DSPINT is the host-to-CPU
    /// interrupt (SPRS276M Table 7-9, SPRS612G Table 7-10 for the C6472's event number).
    pub fn hpi_host_write(&mut self, port: HostPort, value: u32) {
        let interrupt = self.lend_hpi(|hpi, fabric| hpi.host_write(port, value, fabric));
        if interrupt {
            self.host_interrupt();
        }
    }

    /// The host has raised DSPINT. On a C6455 the first one after an HPI boot reset is what
    /// "causes boot configuration logic to bring the CPU out of the 'stalled' state" (SPRS276M
    /// section 2.4); on a C6472 it releases nothing, the host's BOOT_COMPLETE_STAT write does
    /// (SPRUEC6G section 4.3.2). Either way it is the same system event as every later one,
    /// which a held core does not receive. On a C6455 every DSPINT is also EDMA3 event
    /// DSP_EVT, so the boot leaves ER bit 0 set for the firmware to clear (SPRS276M section
    /// 7.4.1); the channel controller is not held with the core.
    fn host_interrupt(&mut self) {
        self.raise(Destination::All, self.profile.events.hpi);
        if let Some(channel) = self.profile.events.hpi_channel {
            self.shared.edma.as_mut().unwrap().trigger_event(channel);
            // The event may have queued a transfer; wake the channel controller.
            self.shared.touched();
        }
        if self.shared.hpi.take_boot_release() {
            self.pending_release |= 1 << self.profile.reset_core;
        }
    }

    /// Deliver a received frame to one of the device's MACs. A C6472 module has one Ethernet
    /// connection, which is EMAC0 unless a front end says otherwise.
    pub(crate) fn inject_frame_on(&mut self, port: usize, frame: Vec<u8>) {
        self.shared.touched();
        if let Some(Some(emac)) = self.shared.emac.get_mut(port) {
            emac.inject_frame(frame);
        }
    }

    /// Whether the named MAC's receive FIFO has room for a frame of this length, which a host
    /// backend asks before delivering one.
    pub(crate) fn emac_rx_has_room(&self, port: usize, length: usize) -> bool {
        match self.shared.emac.get(port) {
            Some(Some(emac)) => emac.rx_fifo_has_room(length),
            _ => true,
        }
    }

    /// Drive the PHY's link state, which latches the MDIO link-change interrupt.
    #[cfg(test)]
    pub(crate) fn mdio_set_link(&mut self, up: bool) {
        self.shared.mdio.set_link(up);
    }

    /// Fuse the device's Ethernet address, which the ROM bootloader's Ethernet boot reads out of
    /// the MAC ID words. A device without a chip-level register file has nowhere to hold one.
    pub(crate) fn fuse_mac_id(&mut self, mac: [u8; 6]) {
        if let Some(chip) = self.shared.chipcfg.as_mut() {
            chip.fuse_mac_id(mac);
        }
    }

    /// Strap the pins a UTOPIA boot reads: the PHY ID on CFGGP[4:0] and the UTOPIA_EN pins for
    /// an 8- or 16-bit port. A device without a chip-level register file has no such pins.
    pub(crate) fn strap_utopia(&mut self, phy: u8, wide: bool) {
        if let Some(chip) = self.shared.chipcfg.as_mut() {
            chip.strap_utopia(phy, wide);
        }
    }

    /// Strap CFGGP[4:0], the configuration pins a C6472 boot mode reads (SPRUEC6G Table 22).
    /// A device without a chip-level register file has no such pins.
    pub(crate) fn strap_cfggp(&mut self, value: u8) {
        if let Some(chip) = self.shared.chipcfg.as_mut() {
            chip.strap_cfggp(value);
        }
    }

    /// URCLAV as an external ATM master sees it after polling `address` on URADDR: `None` while
    /// the pin is high-impedance, otherwise whether the receive queue has room for a whole cell.
    pub fn utopia_master_poll_receive(&mut self, address: u8) -> Option<bool> {
        let edge = self.shared.sysclk3_edge;
        self.shared
            .utopia
            .as_mut()?
            .at(edge, true)
            .master_poll_receive(address)
    }

    /// The master selects `address` and sends one cell's wire bytes into the receive queue. The
    /// cell is taken only if the slave answered that address with room for it; where the PDMA
    /// delivers it, and the interrupt that tells the CPU, follow on the SYSCLK3 edges the device
    /// ticks through.
    pub fn utopia_master_send_cell(&mut self, address: u8, bytes: &[u8]) -> bool {
        let edge = self.shared.sysclk3_edge;
        let Some(utopia) = self.shared.utopia.as_mut() else {
            return false;
        };
        let accepted = utopia.at(edge, true).master_send_cell(address, bytes);
        self.shared.touched();
        self.shared.reschedule(Port::Utopia);
        accepted
    }

    /// UXCLAV as an external ATM master sees it after polling `address` on UXADDR: `None` while
    /// the pin is high-impedance, otherwise whether a complete cell is waiting to be sent.
    pub fn utopia_master_poll_transmit(&mut self, address: u8) -> Option<bool> {
        let edge = self.shared.sysclk3_edge;
        self.shared
            .utopia
            .as_mut()?
            .at(edge, true)
            .master_poll_transmit(address)
    }

    /// The master selects `address` and takes the cell the transmit queue holds, if the slave
    /// answered that address with one. A write the full queue had stalled lands in the space it
    /// frees, and the transmit event that follows is the device's to raise on its next edge.
    pub fn utopia_master_take_cell(&mut self, address: u8) -> Option<WireCell> {
        let edge = self.shared.sysclk3_edge;
        let cell = self
            .shared
            .utopia
            .as_mut()?
            .at(edge, true)
            .master_take_cell(address);
        self.shared.touched();
        self.shared.reschedule(Port::Utopia);
        cell
    }

    pub(crate) fn take_tx_frames(&mut self) -> Vec<Vec<u8>> {
        let mut frames = Vec::new();
        for emac in self.shared.emac.iter_mut().flatten() {
            frames.extend(emac.take_transmitted());
        }
        frames
    }

    // ------------------------------------------------------------------
    // Memory protection.
    // ------------------------------------------------------------------

    /// Check a CPU access against the page attributes of whichever local memory holds it
    /// (SPRU871K chapter 8). A violation latches the address and the attempted permission in
    /// that unit's MPFAR and MPFSR, raises the unit's system event so the exception combiner
    /// can turn it into `EXCEP`, and denies the access.
    ///
    /// The AID fields are not consulted: they gate DMA masters by requester id, and the CPU
    /// is the local requester, which `LOCAL` covers.
    #[inline(always)]
    pub(crate) fn check_access(
        &mut self,
        core: usize,
        address: u32,
        permission: Permission,
        user: bool,
    ) -> bool {
        // The bus carries the requesting core's PrivID and privilege, which the EDMA3's active
        // memory protection filters on and which it stamps on every PaRAM set this access
        // writes (SPRU966C sections 2.10.1 and 2.10.2). Every load and store crosses this seam,
        // and the channel controller is a cold allocation away, so the comparison stays here,
        // inline, and the call is only made when the identity changes.
        let privilege = Privilege {
            id: core as u8,
            user,
        };
        if self.shared.edma_privilege != privilege {
            self.present_to_edma(privilege);
        }
        // Every page grants the CPU everything until software narrows one, which is the reset
        // state and the state the whole of this project's firmware runs in.
        if !self.mm[core].mpu_restrictive {
            return true;
        }
        self.check_restricted_access(core, address, permission, user)
    }

    /// Tell the EDMA3 which master its next register access is made for. The system masters'
    /// paths call this directly, so it repeats the comparison `check_access` makes inline. The
    /// channel controller is out on loan while it services a request, and nothing reads its
    /// privilege then.
    #[inline(never)]
    fn present_to_edma(&mut self, privilege: Privilege) {
        if self.shared.edma_privilege == privilege {
            return;
        }
        if let Some(edma) = self.shared.edma.as_mut() {
            edma.set_privilege(privilege);
            self.shared.edma_privilege = privilege;
        }
    }

    /// The check itself, once some page has stopped granting everything.
    #[inline(never)]
    fn check_restricted_access(
        &mut self,
        core: usize,
        address: u32,
        permission: Permission,
        user: bool,
    ) -> bool {
        let Some((unit, page)) = mpu_page(self.profile.mpu_regions, address) else {
            return true;
        };
        let attributes = self.mm[core].mpu_pages[unit][page];
        let bit = permission_bit(permission, user);
        if attributes & bit != 0 && attributes & 0x100 != 0 {
            return true;
        }
        self.mm[core].record_mpu_fault(unit, address, MPFSR_LOCAL | bit, self.profile.events);
        false
    }

    /// Whether any megamodule restricts DMA masters at all; until one does, no DMA access is
    /// checked.
    fn dma_protection_active(&self) -> bool {
        (0..self.cores).any(|core| self.mm[core].mpu_dma_restrictive)
    }

    /// DMA accesses carry a requestor PrivID rather than LOCAL, and the privilege mode the
    /// master transfers in. IDMA, the HPI's DMA, the UTOPIA PDMA and the EMACs use ID 0; an
    /// EDMA3 transfer carries the identity stamped on its PaRAM set. The unit consulted belongs
    /// to whichever megamodule owns the memory, which on a multicore device is decided by the
    /// global address the master used.
    fn check_dma_access(
        &mut self,
        address: u32,
        permission: Permission,
        privilege: Privilege,
    ) -> bool {
        let Some((core, unit, page)) = self.dma_mpu_page(address) else {
            return true;
        };
        let attributes = self.mm[core].mpu_pages[unit][page];
        let permission_bit = permission_bit(permission, privilege.user);
        if attributes & privilege.allowed_id_bit() != 0 && attributes & permission_bit != 0 {
            return true;
        }
        let status = (u32::from(privilege.id) << 9) | permission_bit;
        self.mm[core].record_mpu_fault(unit, address, status, self.profile.events);
        false
    }

    /// The megamodule, unit and page a DMA address protects, resolving a global alias back to
    /// the local address the protection registers are written in terms of.
    fn dma_mpu_page(&self, address: u32) -> Option<(usize, usize, usize)> {
        let map = self.profile.map;
        let (core, local) = match map.local_of_global(address) {
            Some(pair) => pair,
            // Without global aliases a DMA master addresses the only megamodule's local
            // windows directly.
            None if !map.has_global_aliases() => (0, address),
            None => return None,
        };
        let (unit, page) = mpu_page(self.profile.mpu_regions, local)?;
        (core < self.mm.len()).then_some((core, unit, page))
    }

    // ------------------------------------------------------------------
    // The memory surface.
    // ------------------------------------------------------------------

    #[inline(always)]
    pub(crate) fn read_u8(&mut self, core: usize, address: u32) -> u8 {
        self.memory(core).read_u8(address)
    }

    #[inline(always)]
    pub(crate) fn write_u8(&mut self, core: usize, address: u32, value: u8) {
        self.memory(core).write_u8(address, value);
    }

    #[inline(always)]
    pub(crate) fn read_u16(&mut self, core: usize, address: u32) -> u16 {
        self.memory(core).read_u16(address)
    }

    #[inline(always)]
    pub(crate) fn write_u16(&mut self, core: usize, address: u32, value: u16) {
        self.memory(core).write_u16(address, value);
    }

    #[cfg(test)]
    pub(crate) fn fetch32(&mut self, core: usize, address: u32) -> u32 {
        self.memory(core).fetch32(address)
    }

    #[inline(always)]
    pub(crate) fn fetch_packet(&mut self, core: usize, address: u32) -> [u8; 32] {
        self.memory(core).fetch_packet(address)
    }

    pub(crate) fn read_bytes(&mut self, core: usize, address: u32, length: usize) -> Vec<u8> {
        self.memory(core).read_bytes(address, length)
    }

    /// Load an image segment the way a host or the boot ROM does.
    ///
    /// A segment that lands in a megamodule's local window is loaded into *every* core's copy of
    /// that memory: the local addresses name six different memories on a C6472, each core fetches
    /// its reset vector from its own L2 (SPRS612G Table 2-4), and a host boot writes the same
    /// program into each of them (SPRUEC6G section 4.3.2). Anything else - shared L2, DDR2,
    /// external memory - is one memory and is written once.
    pub(crate) fn load_image_bytes(&mut self, address: u32, bytes: &[u8]) {
        let local = matches!(
            self.profile.map.placement(Requester::Core(0), address),
            Placement::CoreL2 { .. } | Placement::CoreL1P { .. } | Placement::CoreL1D { .. }
        );
        if local {
            for core in 0..self.cores {
                self.load_bytes(core, address, bytes);
            }
        } else {
            self.load_bytes(self.profile.reset_core, address, bytes);
        }
    }

    /// Load memory the way an image loader or a boot table does: straight into the storage,
    /// behind every cache.
    pub(crate) fn load_bytes(&mut self, core: usize, address: u32, bytes: &[u8]) {
        self.shared.backing.note_memory_write(address, bytes.len());
        self.shared
            .backing
            .write_bytes(Requester::Core(core), address, bytes);
    }

    /// A host read is the debug port, and the debug port sees what the CPU wrote: a line still
    /// held dirty in L1D or in the L2 cache has not reached memory yet.
    pub(crate) fn host_read_bytes(&self, core: usize, address: u32, length: usize) -> Vec<u8> {
        let mut data = self
            .shared
            .backing
            .read_bytes(Requester::Core(core), address, length);
        self.mm[core].memory.host_snoop_read(address, &mut data);
        // A line another core holds dirty is just as unwritten, and on a multicore device the
        // address the debugger is looking at may be that core's memory.
        if let Some((owner, local)) = self.owning_core(Requester::Core(core), address) {
            if owner != core {
                self.mm[owner].memory.host_snoop_read(local, &mut data);
                self.mm[owner].memory.host_snoop_read(address, &mut data);
            }
        }
        data
    }

    pub(crate) fn host_write_bytes(&mut self, core: usize, address: u32, bytes: &[u8]) {
        let length = bytes.len().min(u32::MAX as usize) as u32;
        let owner = self.owning_core(Requester::Core(core), address);
        {
            let (mm, backing) = (&mut self.mm[core], &mut self.shared.backing);
            mm.memory.bind(backing).host_invalidate(address, length);
        }
        if let Some((owner, local)) = owner {
            if owner != core {
                let (mm, backing) = (&mut self.mm[owner], &mut self.shared.backing);
                mm.memory.bind(backing).host_invalidate(local, length);
                mm.memory.bind(backing).host_invalidate(address, length);
            }
        }
        self.load_bytes(core, address, bytes);
    }

    /// Which megamodule's memory an address belongs to, and what that core calls it locally.
    /// Resolved through the storage rather than the static map, so memory a core has converted
    /// into cache belongs to nobody.
    fn owning_core(&self, requester: Requester, address: u32) -> Option<(usize, u32)> {
        let map = self.profile.map;
        match self.shared.backing.placement(requester, address) {
            Placement::CoreL2 { core, offset } => Some((core, map.local_l2().0 + offset)),
            Placement::CoreL1P { core, offset } => Some((core, 0x00e0_0000 + offset)),
            Placement::CoreL1D { core, offset } => Some((core, 0x00f0_0000 + offset)),
            _ => None,
        }
    }

    // ------------------------------------------------------------------
    // DMA masters.
    // ------------------------------------------------------------------

    /// One word for a system master that moves a word at a time - the HPI's DMA logic, and the
    /// IDMA the CPU programs. Both carry privilege ID 0 (SPRS276M section 5.2, SPRS612G Table
    /// 5-3), which is what memory protection judges them by; memory follows the snoop protocol
    /// the hierarchy owns, and the configuration space answers as `master_register_read` says.
    /// `None` when protection refuses the read.
    pub(crate) fn master_read32(&mut self, address: u32) -> Option<u32> {
        self.master_read32_as(address, false)
    }

    /// `master_read32` for a master whose transfers carry `user` privilege rather than the
    /// supervisor's, with PrivID 0.
    fn master_read32_as(&mut self, address: u32, user: bool) -> Option<u32> {
        let privilege = Privilege { id: 0, user };
        if !is_memory(address) {
            return Some(self.master_register_read(address, privilege));
        }
        self.checked_dma_word(address, privilege)
    }

    pub(crate) fn master_write32(&mut self, address: u32, value: u32) {
        self.master_write32_as(address, value, false);
    }

    /// `master_write32` for a master whose transfers carry `user` privilege rather than the
    /// supervisor's, with PrivID 0.
    fn master_write32_as(&mut self, address: u32, value: u32, user: bool) {
        let privilege = Privilege { id: 0, user };
        if !is_memory(address) {
            self.master_register_write(address, value, privilege);
            return;
        }
        self.checked_dma_store_word(address, value, privilege);
    }

    /// A system master's word in the configuration space. A chip-level peripheral answers
    /// through its slave port, with every effect a CPU access would have. The megamodules' own
    /// registers - interrupt controller, cache and protection configuration - are "only
    /// accessible by the CPU" (SPRU871K section 5.3.1): a master reads zero there and its
    /// writes go nowhere. Whatever else the configuration space holds that no port claims is
    /// device memory, reached as any other memory is.
    fn master_register_read(&mut self, address: u32, privilege: Privilege) -> u32 {
        self.present_to_edma(privilege);
        self.shared.touched();
        let context = self.port_context();
        if let Some(value) = self.peripheral_read(address, context) {
            return value;
        }
        if MEGAMODULE_REGISTERS.contains(&address) {
            return 0;
        }
        self.dma_word(address)
    }

    fn master_register_write(&mut self, address: u32, value: u32, privilege: Privilege) {
        self.present_to_edma(privilege);
        self.shared.touched();
        let context = self.port_context();
        if self.peripheral_write(address, value, context) || MEGAMODULE_REGISTERS.contains(&address)
        {
            return;
        }
        self.dma_store_word(address, value);
    }

    /// A memory word in the device's byte order, as a DMA master sees it.
    fn dma_word(&mut self, address: u32) -> u32 {
        let bytes: [u8; 4] = self.dma_bytes(address, 4).try_into().unwrap();
        match self.shared.backing.endian() {
            Endian::Little => u32::from_le_bytes(bytes),
            Endian::Big => u32::from_be_bytes(bytes),
        }
    }

    fn dma_store_word(&mut self, address: u32, value: u32) {
        let bytes = match self.shared.backing.endian() {
            Endian::Little => value.to_le_bytes(),
            Endian::Big => value.to_be_bytes(),
        };
        self.dma_store(address, &bytes);
    }

    /// A DMA read of memory: the stored bytes with the L2 SRAM snoop applied.
    ///
    /// The L2 controller serves a DMA read of L2 SRAM from L1D when the CPU holds that line
    /// dirty, so a buffer the CPU has only written into its cache still goes out correctly
    /// (SPRU862B section 2.4.2). External memory is not snooped at all. On a multicore device
    /// the line may be tagged with either the local or the global address, depending on which
    /// the core used, so both are looked for.
    fn dma_bytes(&mut self, address: u32, length: usize) -> Vec<u8> {
        let mut data = self
            .shared
            .backing
            .read_bytes(Requester::Dma, address, length);
        if let Some((core, local)) = self.snooped_l2(address) {
            self.mm[core].memory.snoop_read(local, &mut data);
            if local != address {
                self.mm[core].memory.snoop_read(address, &mut data);
            }
        }
        data
    }

    /// A DMA write to L2 SRAM is snoop-written into L1D, so the CPU sees it without any cache
    /// operation. Outside L2 SRAM the CPU keeps its stale line, and maintaining that is the
    /// firmware's job (SPRU862B sections 2.4 and 2.6).
    fn dma_store(&mut self, address: u32, bytes: &[u8]) {
        self.shared.backing.note_memory_write(address, bytes.len());
        self.shared
            .backing
            .write_bytes(Requester::Dma, address, bytes);
        if let Some((core, local)) = self.snooped_l2(address) {
            self.mm[core].memory.snoop_write(local, bytes);
            if local != address {
                self.mm[core].memory.snoop_write(address, bytes);
            }
        }
    }

    /// The megamodule whose L2 SRAM a DMA address lands in, and the local address that core
    /// would have used - the only memory the coherence protocol snoops. Memory the core has
    /// converted into cache is not L2 SRAM and is not snooped.
    fn snooped_l2(&self, address: u32) -> Option<(usize, u32)> {
        let map = self.profile.map;
        match self.shared.backing.placement(Requester::Dma, address) {
            Placement::CoreL2 { core, offset } => Some((core, map.local_l2().0 + offset)),
            _ => None,
        }
    }

    fn dma_register_read(&mut self, address: u32, length: usize, privilege: Privilege) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(length);
        while bytes.len() < length {
            let byte_address = address.wrapping_add(bytes.len() as u32);
            let word = self.master_register_read(byte_address & !3, privilege);
            let word = if self.shared.backing.endian_big() {
                word.to_be_bytes()
            } else {
                word.to_le_bytes()
            };
            for byte in word.into_iter().skip((byte_address & 3) as usize) {
                if bytes.len() == length {
                    break;
                }
                bytes.push(byte);
            }
        }
        bytes
    }

    /// Write the configuration space for a DMA master, assembling whole words around the bytes
    /// the transfer supplies.
    fn dma_register_write(&mut self, address: u32, bytes: &[u8], privilege: Privilege) {
        let mut written = 0usize;
        while written < bytes.len() {
            let byte_address = address.wrapping_add(written as u32);
            let word_address = byte_address & !3;
            let current = self.master_register_read(word_address, privilege);
            let mut word = if self.shared.backing.endian_big() {
                current.to_be_bytes()
            } else {
                current.to_le_bytes()
            };
            for slot in word.iter_mut().skip((byte_address & 3) as usize) {
                if written == bytes.len() {
                    break;
                }
                *slot = bytes[written];
                written += 1;
            }
            let value = if self.shared.backing.endian_big() {
                u32::from_be_bytes(word)
            } else {
                u32::from_le_bytes(word)
            };
            self.master_register_write(word_address, value, privilege);
        }
    }

    /// Whether protection lets a master with `privilege` make `permission` accesses to all of
    /// `length` bytes at `address`; the first protected byte latches the fault and refuses the
    /// whole access.
    fn dma_access_allowed(
        &mut self,
        address: u32,
        length: usize,
        permission: Permission,
        privilege: Privilege,
    ) -> bool {
        if !self.dma_protection_active() {
            return true;
        }
        (0..length).all(|offset| {
            let byte_address = address.wrapping_add(offset as u32);
            self.check_dma_access(byte_address, permission, privilege)
        })
    }

    /// A memory word for a master with `privilege`, its four bytes judged as
    /// `dma_access_allowed` says; `None` when protection refuses it.
    fn checked_dma_word(&mut self, address: u32, privilege: Privilege) -> Option<u32> {
        if !self.dma_access_allowed(address, 4, Permission::Read, privilege) {
            return None;
        }
        Some(self.dma_word(address))
    }

    /// `checked_dma_word` for a store: a refused word is discarded.
    fn checked_dma_store_word(&mut self, address: u32, value: u32, privilege: Privilege) {
        if self.dma_access_allowed(address, 4, Permission::Write, privilege) {
            self.dma_store_word(address, value);
        }
    }

    /// `length` bytes for a byte-moving system master with `privilege`. A DMA access inside a
    /// peripheral aperture reaches that peripheral's register through its slave port, with all
    /// the side effects a CPU read would have, rather than the memory map behind it; a
    /// protected byte refuses the whole access.
    fn requestor_read_bytes(
        &mut self,
        address: u32,
        length: usize,
        privilege: Privilege,
    ) -> Vec<u8> {
        if !is_memory(address) {
            return self.dma_register_read(address, length, privilege);
        }
        if !self.dma_access_allowed(address, length, Permission::Read, privilege) {
            return vec![0; length];
        }
        self.dma_bytes(address, length)
    }

    fn requestor_write_bytes(&mut self, address: u32, bytes: &[u8], privilege: Privilege) {
        if !is_memory(address) {
            self.dma_register_write(address, bytes, privilege);
            return;
        }
        if !self.dma_access_allowed(address, bytes.len(), Permission::Write, privilege) {
            return;
        }
        self.dma_store(address, bytes);
    }
}

/// The EDMA3's transfers, each made with the privilege stamped on its PaRAM set: that of the
/// master that programmed it (SPRU966C section 2.10.2, SPRS612G Table 5-3). The checks and the
/// L2 snoop are the ones every other DMA master gets.
impl TransferMemory for Fabric {
    fn read(&mut self, privilege: Privilege, address: u32, length: usize) -> Vec<u8> {
        self.requestor_read_bytes(address, length, privilege)
    }

    fn write(&mut self, privilege: Privilege, address: u32, bytes: &[u8]) {
        self.requestor_write_bytes(address, bytes, privilege);
    }
}

/// The fabric as an EMAC's DMA sees it. Every peripheral master but the HPI and SRIO carries
/// PrivID 0 (SPRS612G Table 5-3; on the C6455 every system master does, SPRS276M section 5.2),
/// and the EMACs always transfer in user mode (SPRS612G section 5.2: "For most peripheral
/// masters (EMAC0, EMAC1, UTOPIA, TSIP0, TSIP1, and TSIP2), the privilege mode is always user
/// mode").
struct EmacMaster<'a>(&'a mut Fabric);

impl EmacDmaMemory for EmacMaster<'_> {
    fn dma_read_bytes(&mut self, address: u32, length: usize) -> Vec<u8> {
        self.0
            .requestor_read_bytes(address, length, Privilege::ID0_USER)
    }

    fn dma_write_bytes(&mut self, address: u32, bytes: &[u8]) {
        self.0
            .requestor_write_bytes(address, bytes, Privilege::ID0_USER);
    }

    /// A descriptor word in memory is judged like the packet buffers, and a refused one reads
    /// as zero. The EMAC's own descriptor RAM sits in its peripheral aperture, outside memory
    /// protection.
    fn dma_read_u32(&mut self, address: u32) -> u32 {
        if !is_memory(address) {
            return self.0.dma_word(address);
        }
        self.0
            .checked_dma_word(address, Privilege::ID0_USER)
            .unwrap_or(0)
    }

    fn dma_write_u32(&mut self, address: u32, value: u32) {
        if !is_memory(address) {
            self.0.dma_store_word(address, value);
            return;
        }
        self.0
            .checked_dma_store_word(address, value, Privilege::ID0_USER);
    }
}

/// The fabric as one core sees it: the seam the CPU crosses for every access it makes.
pub(crate) struct CoreBus<'a> {
    core: usize,
    fabric: &'a mut Fabric,
}

impl<'a> CoreBus<'a> {
    pub(crate) fn new(core: usize, fabric: &'a mut Fabric) -> Self {
        Self { core, fabric }
    }

    pub(crate) fn endian(&self) -> Endian {
        self.fabric.endian()
    }

    #[inline(always)]
    pub(crate) fn read32(&mut self, address: u32) -> u32 {
        self.fabric.read32(self.core, address)
    }

    #[inline(always)]
    pub(crate) fn write32(&mut self, address: u32, value: u32) {
        self.fabric.write32(self.core, address, value);
    }

    #[inline(always)]
    pub(crate) fn read_u8(&mut self, address: u32) -> u8 {
        self.fabric.read_u8(self.core, address)
    }

    #[inline(always)]
    pub(crate) fn write_u8(&mut self, address: u32, value: u8) {
        self.fabric.write_u8(self.core, address, value);
    }

    #[inline(always)]
    pub(crate) fn read_u16(&mut self, address: u32) -> u16 {
        self.fabric.read_u16(self.core, address)
    }

    #[inline(always)]
    pub(crate) fn write_u16(&mut self, address: u32, value: u16) {
        self.fabric.write_u16(self.core, address, value);
    }

    pub(crate) fn read_bytes(&mut self, address: u32, length: usize) -> Vec<u8> {
        self.fabric.read_bytes(self.core, address, length)
    }

    pub(crate) fn load_bytes(&mut self, address: u32, bytes: &[u8]) {
        self.fabric.load_bytes(self.core, address, bytes);
    }

    pub(crate) fn host_read_bytes(&self, address: u32, length: usize) -> Vec<u8> {
        self.fabric.host_read_bytes(self.core, address, length)
    }

    pub(crate) fn host_write_bytes(&mut self, address: u32, bytes: &[u8]) {
        self.fabric.host_write_bytes(self.core, address, bytes);
    }

    #[inline(always)]
    pub(crate) fn fetch_packet(&mut self, address: u32) -> [u8; 32] {
        self.fabric.fetch_packet(self.core, address)
    }

    #[inline(always)]
    pub(crate) fn fetch_reusable(&self, address: u32) -> bool {
        self.fabric.mm[self.core].memory.fetch_reusable(address)
    }

    #[inline(always)]
    pub(crate) fn l1p_epoch(&self) -> u64 {
        self.fabric.mm[self.core].memory.l1p_epoch()
    }

    pub(crate) fn memory_version(&self, address: u32) -> u64 {
        self.fabric.shared.backing.memory_version(address)
    }

    pub(crate) fn track_memory_version(&mut self, address: u32) -> u64 {
        self.fabric.shared.backing.track_memory_version(address)
    }

    /// Everything one data access tells the timing model - residency, whether a read would need
    /// a fill, and where the line comes from - in one answer, because the data access path
    /// charges all three of every access.
    #[inline(always)]
    pub(crate) fn access_timing(&self, address: u32) -> AccessTiming {
        self.fabric.mm[self.core].memory.access_timing(address)
    }

    /// The interrupt lines to take this cycle and whether the exception combiner is asserting,
    /// which `finish_cycle` needs together. `short_tick` is what this core's own ask of the
    /// device answered: on a cycle the peripherals did not move, the lines the last full tick
    /// selected are the ones still asserted.
    #[inline(always)]
    pub(crate) fn take_interrupt_state(&mut self, short_tick: bool) -> (u16, bool) {
        let mm = &mut self.fabric.mm[self.core];
        (
            mm.take_pending_cpu_interrupts(short_tick),
            mm.pending_exception_event(),
        )
    }

    #[inline(always)]
    pub(crate) fn take_l1d_victim_flush(&mut self) -> bool {
        self.fabric.mm[self.core].memory.take_l1d_victim_flush()
    }

    pub(crate) fn drain_l1p_fills(&mut self) -> std::vec::Drain<'_, LineFill> {
        self.fabric.mm[self.core].memory.drain_l1p_fills()
    }

    pub(crate) fn check_access(
        &mut self,
        address: u32,
        permission: Permission,
        user: bool,
    ) -> bool {
        self.fabric
            .check_access(self.core, address, permission, user)
    }

    /// Advance the shared peripherals, once per device cycle, and answer whether this ask was
    /// a short tick.
    #[inline(always)]
    pub(crate) fn tick(&mut self, cycle: u64) -> bool {
        self.fabric.tick(cycle)
    }

    pub(crate) fn record_dropped_interrupt(&mut self, interrupt: u8) {
        self.fabric.mm[self.core].record_dropped_interrupt(interrupt);
    }

    pub(crate) fn cache_views(&self) -> [CacheView; 3] {
        self.fabric.mm[self.core].cache_views()
    }

    pub(crate) fn cache_line_data(&self, level: usize, address: u32) -> Option<&[u8]> {
        self.fabric.mm[self.core].cache_line_data(level, address)
    }

    pub(crate) fn event_flag_registers(&self) -> [u32; 4] {
        self.fabric.mm[self.core].event_flag_registers()
    }

    pub(crate) fn interrupt_selectors(&self) -> [u32; 3] {
        self.fabric.mm[self.core].interrupt_selectors()
    }

    /// The host port belongs to the device, not this core, but a front end reaches it through
    /// whichever core it is driving.
    pub(crate) fn hpi_host_read(&mut self, port: HostPort) -> Option<u32> {
        self.fabric.hpi_host_read(port)
    }

    pub(crate) fn hpi_host_write(&mut self, port: HostPort, value: u32) {
        self.fabric.hpi_host_write(port, value);
    }

    /// Drive one conformance-vector operation at the device through this core.
    #[cfg(test)]
    pub(crate) fn apply(&mut self, operation: &Value) -> Result<Option<Value>, String> {
        self.fabric.apply(operation)
    }

    pub(crate) fn mar_registers(&self) -> [u32; 256] {
        self.fabric.mm[self.core].mar_registers()
    }
}

/// The ROM bootloader runs on the CPU, so its reads and writes are ordinary CPU accesses and
/// reach peripheral registers the way a boot configuration table expects (SPRUEC6G section 6.2.3).
impl BootMemory for CoreBus<'_> {
    fn boot_read_word(&mut self, address: u32) -> u32 {
        self.read32(address)
    }

    fn boot_read_bytes(&mut self, address: u32, length: usize) -> Vec<u8> {
        self.read_bytes(address, length)
    }

    fn boot_write_word(&mut self, address: u32, value: u32) {
        self.write32(address, value);
    }

    /// Section data is copied a word at a time wherever the destination allows, with only a
    /// ragged head or tail written a byte at a time. That is what the ROM's own copy loop does,
    /// and it is what makes a section whose destination is a register work - which SPRUEC6G
    /// section 4.3.3 relies on when it sets DSP_BOOT_ADDR*n* and BOOT_COMPLETE_STAT from a
    /// standard boot table - because a byte access in this model reaches memory only.
    fn boot_write_bytes(&mut self, address: u32, bytes: &[u8]) {
        let endian = self.endian();
        let mut cursor = address;
        let mut rest = bytes;
        while let Some(byte) = rest.first().copied() {
            if cursor % 4 == 0 && rest.len() >= 4 {
                let word: [u8; 4] = rest[..4].try_into().expect("four bytes");
                self.write32(
                    cursor,
                    match endian {
                        Endian::Little => u32::from_le_bytes(word),
                        Endian::Big => u32::from_be_bytes(word),
                    },
                );
                cursor = cursor.wrapping_add(4);
                rest = &rest[4..];
            } else {
                self.write_u8(cursor, byte);
                cursor = cursor.wrapping_add(1);
                rest = &rest[1..];
            }
        }
    }
}

/// The megamodule's four flag/mask register kinds, by base address.
#[derive(Clone, Copy)]
enum IntcRegister {
    Flag,
    Mask,
    MaskedFlag,
    ExceptionMask,
    MaskedExceptionFlag,
}

fn mpu_page(regions: &'static [MpuRegion], address: u32) -> Option<(usize, usize)> {
    regions.iter().find_map(|region| {
        let offset = address.checked_sub(region.base)?;
        let page = (offset / region.page_bytes) as usize;
        (page < region.pages).then_some((region.unit, region.first_index + page))
    })
}

/// The MPPA bit an access needs: SR SW SX for Supervisor, UR UW UX for User (SPRU871K 8.2).
fn permission_bit(permission: Permission, user: bool) -> u32 {
    match (permission, user) {
        (Permission::Read, false) => 1 << 5,
        (Permission::Write, false) => 1 << 4,
        (Permission::Execute, false) => 1 << 3,
        (Permission::Read, true) => 1 << 2,
        (Permission::Write, true) => 1 << 1,
        (Permission::Execute, true) => 1 << 0,
    }
}

/// One of the per-page attribute registers of a unit.
fn page_index(address: u32, base: u32) -> Option<usize> {
    let offset = address.checked_sub(base)?;
    (offset < MPU_PAGES as u32 * 4 && offset % 4 == 0).then_some((offset / 4) as usize)
}

fn intc_register(address: u32) -> Option<(usize, IntcRegister)> {
    bank_index(address, EVTFLAG0)
        .map(|bank| (bank, IntcRegister::Flag))
        .or_else(|| bank_index(address, EVTMASK0).map(|bank| (bank, IntcRegister::Mask)))
        .or_else(|| bank_index(address, MEVTFLAG0).map(|bank| (bank, IntcRegister::MaskedFlag)))
        .or_else(|| bank_index(address, EXPMASK0).map(|bank| (bank, IntcRegister::ExceptionMask)))
        .or_else(|| {
            bank_index(address, MEXPFLAG0).map(|bank| (bank, IntcRegister::MaskedExceptionFlag))
        })
}

fn bandwidth_register(address: u32) -> Option<usize> {
    BWM_REGISTERS
        .iter()
        .position(|(register, _, _)| *register == address)
}

fn bank_index(address: u32, base: u32) -> Option<usize> {
    let offset = address.checked_sub(base)?;
    (offset < 16 && offset % 4 == 0).then_some((offset / 4) as usize)
}

impl Fabric {
    /// Cores whose NMI pin the chip-level registers drove since this was last called
    /// (SPRS612G section 3.7.1). The device delivers them to the matching CPUs.
    pub(crate) fn take_nmi_requests(&mut self) -> u32 {
        std::mem::take(&mut self.pending_nmi)
    }

    /// Cores released from reset since this was last called, by the boot controller or by the
    /// power/sleep controller.
    pub(crate) fn take_release_requests(&mut self) -> u32 {
        std::mem::take(&mut self.pending_release)
    }

    /// Whether this core's megamodule has a running clock. A core whose LPSC module the
    /// power/sleep controller has taken out of Enable does not execute (SPRUEG3B section
    /// 4.1.2); a device without a PSC always runs.
    pub(crate) fn core_clocked(&self, core: usize) -> bool {
        self.shared.core_clocked(core)
    }

    /// Whether a core is held in reset at power-up: with a boot controller, every core but the
    /// one the boot mode starts (SPRS612G section 2.4.1; a C6472 strapped to HPI boot starts
    /// none, SPRUEC6G section 4.3.2); a C6455 strapped to HPI boot, the one core there is,
    /// until the host's DSPINT (SPRS276M section 2.4: "the CPU is actually held in reset until
    /// a DSP interrupt is generated by the host").
    pub(crate) fn apply_reset_holds(&mut self) {
        if self.shared.hpi.boot_pending() {
            self.mm[self.profile.reset_core].held_in_reset = true;
        }
        let Some(boot) = self.shared.bootctl.as_ref() else {
            return;
        };
        let held: Vec<bool> = (0..self.mm.len())
            .map(|core| boot.held_in_reset(core))
            .collect();
        for (core, held) in held.into_iter().enumerate() {
            self.mm[core].held_in_reset = held;
        }
    }

    /// Whether the device is strapped to an HPI boot the host has not yet ended with DSPINT.
    pub(crate) fn hpi_boot_pending(&self) -> bool {
        self.shared.hpi.boot_pending()
    }

    /// Whether the device was strapped to HPI boot, which holds for the device's lifetime: the
    /// C6455's DSPINT ends the hold on its core and the C6472's host ends each core's own, but
    /// neither makes the device any less one that booted this way.
    pub(crate) fn hpi_boot_strapped(&self) -> bool {
        self.shared.hpi.strapped()
    }

    /// The shared L2 ROM image, which is where the boot controller starts core 0 for every ROM
    /// boot mode (SPRS612G Table 2-4).
    #[cfg(test)]
    pub fn apply(&mut self, operation: &Value) -> Result<Option<Value>, String> {
        self.shared.touched();
        // Every operation acts for one core; a vector that says nothing means core 0, which is
        // the only core a single-core device has.
        let core = operation
            .get("core")
            .and_then(Value::as_u64)
            .unwrap_or(0)
            .min(self.mm.len() as u64 - 1) as usize;
        let op = string_field(operation, "op")?;
        match op {
            "write32" => {
                let address = u32_field(operation, "address")?;
                let value = u32_field(operation, "value")?;
                self.write32(core, address, value);
                Ok(None)
            }
            "read32" => {
                let address = u32_field(operation, "address")?;
                Ok(Some(json!(self.read32(core, address))))
            }
            "write_bytes" => {
                let address = u32_field(operation, "address")?;
                let bytes = decode_hex(string_field(operation, "hex")?)?;
                self.load_bytes(core, address, &bytes);
                Ok(None)
            }
            "read_bytes" => {
                let address = u32_field(operation, "address")?;
                let length = usize_field(operation, "length")?;
                Ok(Some(json!(encode_hex(
                    &self.read_bytes(core, address, length)
                ))))
            }
            // A load, store or fetch as the core issues it: checked against the page the
            // address falls in, where `write32` and `read32` above are the debug port and are
            // never checked. It answers whether the access was granted.
            "cpu_access" => {
                let address = u32_field(operation, "address")?;
                let permission = match string_field(operation, "permission")? {
                    "read" => Permission::Read,
                    "write" => Permission::Write,
                    "execute" => Permission::Execute,
                    other => return Err(format!("unsupported permission {other}")),
                };
                let user = operation
                    .get("user")
                    .and_then(Value::as_bool)
                    .unwrap_or(false);
                Ok(Some(json!(
                    self.check_access(core, address, permission, user)
                )))
            }
            "tick" => {
                let cycles = usize_field(operation, "cycles")?;
                for _ in 0..cycles {
                    self.tick_once();
                }
                Ok(None)
            }
            // The ROM bootloader's two table formats (SPRUEC6G sections 6.2.2 and 6.2.3).
            "boot_table" => {
                let address = u32_field(operation, "address")?;
                let entry = crate::boot::load_boot_table(&mut self.core(core), address)?;
                Ok(Some(json!(entry)))
            }
            "boot_config" => {
                let address = u32_field(operation, "address")?;
                let calls = crate::boot::apply_boot_configuration(&mut self.core(core), address)?;
                Ok(Some(json!(calls)))
            }
            "take_tx_frames" => {
                let frames = self
                    .take_tx_frames()
                    .into_iter()
                    .map(|frame| encode_hex(&frame))
                    .collect::<Vec<_>>();
                Ok(Some(json!(frames)))
            }
            "set_link" => {
                // Drive the PHY's link state, which latches the MDIO link-change interrupt.
                let value = u32_field(operation, "value")?;
                self.mdio_set_link(value != 0);
                Ok(None)
            }
            // The four host access types the HCNTL pins select (SPRU969B Table 4).
            "hpi_read" => {
                let port = host_port(string_field(operation, "port")?)?;
                Ok(Some(json!(self.hpi_host_read(port))))
            }
            // The 16-bit multiplexed mode: one datagram per halfword, HHWIL as `second`.
            "hpi_read16" => {
                let port = host_port(string_field(operation, "port")?)?;
                let second = operation
                    .get("second")
                    .and_then(Value::as_bool)
                    .unwrap_or(false);
                Ok(Some(json!(self.hpi_host_read_halfword(port, second))))
            }
            "hpi_write16" => {
                let port = host_port(string_field(operation, "port")?)?;
                let value = u32_field(operation, "value")? as u16;
                let second = operation
                    .get("second")
                    .and_then(Value::as_bool)
                    .unwrap_or(false);
                self.hpi_host_write_halfword(port, value, second);
                Ok(None)
            }
            "hpi_write" => {
                let port = host_port(string_field(operation, "port")?)?;
                let value = u32_field(operation, "value")?;
                self.hpi_host_write(port, value);
                Ok(None)
            }
            // The external ATM master on the UTOPIA pins. It drives a PHY address onto the
            // address bus - the pulled-up null PHY 1Fh when the case names none - and polls the
            // addressed slave's cell-available pin, or selects that address and takes a cell from
            // the transmit queue or offers one to the receive queue. Like a register access, it
            // acts on the port as of the current edge and leaves it to be planned again.
            "utopia_master_poll" => {
                let address = utopia_address(operation)?;
                let transmit = match string_field(operation, "direction")? {
                    "transmit" => true,
                    "receive" => false,
                    other => return Err(format!("unknown UTOPIA direction {other:?}")),
                };
                let edge = self.shared.sysclk3_edge;
                let clav = self.shared.utopia.as_mut().and_then(|utopia| {
                    let utopia = utopia.at(edge, true);
                    if transmit {
                        utopia.master_poll_transmit(address)
                    } else {
                        utopia.master_poll_receive(address)
                    }
                });
                self.shared.reschedule(Port::Utopia);
                Ok(Some(clav.map_or(Value::Null, Value::Bool)))
            }
            "utopia_master_read" => {
                let address = utopia_address(operation)?;
                let edge = self.shared.sysclk3_edge;
                let cell = self
                    .shared
                    .utopia
                    .as_mut()
                    .and_then(|utopia| utopia.at(edge, true).master_take_cell(address));
                self.shared.reschedule(Port::Utopia);
                self.latch_utopia_interrupt();
                Ok(Some(match cell {
                    Some(cell) => json!(encode_hex(cell.bytes())),
                    None => Value::Null,
                }))
            }
            "utopia_master_write" => {
                let address = utopia_address(operation)?;
                let bytes = decode_hex(string_field(operation, "hex")?)?;
                let edge = self.shared.sysclk3_edge;
                let accepted =
                    self.shared.utopia.as_mut().is_some_and(|utopia| {
                        utopia.at(edge, true).master_send_cell(address, &bytes)
                    });
                self.shared.reschedule(Port::Utopia);
                self.latch_utopia_interrupt();
                Ok(Some(json!(accepted)))
            }
            // A device on the I2C bus at one address, with the bytes it answers a read with.
            "i2c_slave" => {
                let address = u32_field(operation, "address")?;
                let bytes = match operation.get("hex") {
                    Some(_) => decode_hex(string_field(operation, "hex")?)?,
                    None => Vec::new(),
                };
                self.shared.i2c.attach_slave(address, bytes);
                Ok(None)
            }
            // What that device has been sent.
            "i2c_slave_received" => Ok(Some(json!(encode_hex(&self.shared.i2c.slave_received())))),
            // An external driver on the GPIO input pins, which nothing on the device model
            // would otherwise hold at any level.
            "gpio_input" => {
                let value = u32_field(operation, "value")?;
                let edge = self.shared.sysclk3_edge;
                self.shared.gpio.at(edge, true).drive_inputs(value);
                self.shared.reschedule(Port::Gpio);
                Ok(None)
            }
            "inject_frame" => {
                let frame = decode_hex(string_field(operation, "hex")?)?;
                let port = operation.get("port").and_then(Value::as_u64).unwrap_or(0) as usize;
                // A frame may arrive with the wire errors the receiver cannot make itself.
                let mut errors = 0;
                if let Some(names) = operation.get("errors").and_then(Value::as_array) {
                    for name in names {
                        errors |= match name.as_str().unwrap_or_default() {
                            "crc" => crate::emac::RX_ERROR_CRC,
                            "align" => crate::emac::RX_ERROR_ALIGN,
                            "code" => crate::emac::RX_ERROR_CODE,
                            other => return Err(format!("unknown frame error {other:?}")),
                        };
                    }
                }
                self.shared.touched();
                if let Some(Some(emac)) = self.shared.emac.get_mut(port) {
                    emac.inject_frame_with_errors(frame, errors);
                }
                Ok(None)
            }
            other => Err(format!("unsupported system operation {other}")),
        }
    }
}

/// The host access type a case names, matching the HCNTL encoding in SPRU969B Table 4.
#[cfg(test)]
fn host_port(name: &str) -> Result<HostPort, String> {
    match name {
        "hpic" => Ok(HostPort::Control),
        "hpia" => Ok(HostPort::Address),
        "hpid" => Ok(HostPort::Data),
        "hpid_auto" => Ok(HostPort::DataAutoIncrement),
        other => Err(format!("unknown HPI host port {other}")),
    }
}

/// The PHY address a UTOPIA master operation drives: five bits, the null PHY when unnamed.
#[cfg(test)]
fn utopia_address(operation: &Value) -> Result<u8, String> {
    match operation.get("address") {
        None => Ok(crate::utopia::NULL_PHY),
        Some(_) => match u32_field(operation, "address")? {
            address @ 0..=0x1f => Ok(address as u8),
            address => Err(format!("UTOPIA address {address} is wider than five bits")),
        },
    }
}

#[cfg(test)]
fn string_field<'a>(value: &'a Value, key: &str) -> Result<&'a str, String> {
    value
        .get(key)
        .and_then(Value::as_str)
        .ok_or_else(|| format!("missing string field {key}"))
}

#[cfg(test)]
fn u32_field(value: &Value, key: &str) -> Result<u32, String> {
    value
        .get(key)
        .and_then(Value::as_u64)
        .and_then(|number| u32::try_from(number).ok())
        .ok_or_else(|| format!("missing u32 field {key}"))
}

#[cfg(test)]
fn usize_field(value: &Value, key: &str) -> Result<usize, String> {
    value
        .get(key)
        .and_then(Value::as_u64)
        .and_then(|number| usize::try_from(number).ok())
        .ok_or_else(|| format!("missing usize field {key}"))
}

#[cfg(test)]
fn decode_hex(text: &str) -> Result<Vec<u8>, String> {
    if text.len() % 2 != 0 {
        return Err("hex string has odd length".into());
    }
    text.as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let text = std::str::from_utf8(pair).map_err(|_| "invalid hex")?;
            u8::from_str_radix(text, 16).map_err(|_| "invalid hex".into())
        })
        .collect()
}

#[cfg(test)]
fn encode_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0xf) as usize] as char);
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::emac::DmaMemory;
    use crate::memory::{L1DCFG, L1DINV, L1PCFG, L2CFG, L2WB, L2WBAR, L2WWC, MAR_BASE};
    use crate::port::SlavePort;

    #[test]
    fn cache_size_registers_follow_the_c6455_encodings() {
        let mut fabric = Fabric::default();
        for (mode, l1_bytes, l2_bytes) in [
            (0, 0, 0),
            (1, 4 * 1024, 32 * 1024),
            (2, 8 * 1024, 64 * 1024),
            (3, 16 * 1024, 128 * 1024),
            (4, 32 * 1024, 256 * 1024),
            // Encodings above 4 select the maximum; TI's kernel writes 7 to all three registers.
            (7, 32 * 1024, 256 * 1024),
        ] {
            fabric.write32(0, L1PCFG, mode);
            fabric.write32(0, L1DCFG, mode);
            fabric.write32(0, L2CFG, mode);
            let [l1p, l1d, l2] = fabric.megamodule(0).cache_views();
            assert_eq!(l1p.capacity_lines * l1p.line_size as usize, l1_bytes);
            assert_eq!(l1d.capacity_lines * l1d.line_size as usize, l1_bytes);
            assert_eq!(l2.capacity_lines * l2.line_size as usize, l2_bytes);
        }
    }

    /// The other half of what a cache size decides: how much SRAM is left, and where.
    ///
    /// SPRU862B Figure 2-1 draws it for the C6455's 2048 KB L2 - the four cache sizes sit at
    /// 009F 8000h, 009F 0000h, 009E 0000h and 009C 0000h - and the C6472's 608 KB L2 follows
    /// the same rule from a different top (SPRS612G Table 2-2 ends its local L2 at 0089 7FFFh).
    /// L1P is here rather than in a suite case because no case can read it: "the CPU cannot
    /// read from L1P, even if L1P is memory-mapped" (SPRU871K section 2.3.1.2), so a DMA master
    /// is the only requester that can ask, which is how firmware writes L1P too.
    #[test]
    fn a_cache_size_moves_the_end_of_the_sram_it_came_out_of() {
        // The four documented sizes, then the "maximal cache" encoding that repeats the last
        // (SPRU871K Tables 2-2, 3-2 and 4-4).
        const MODES: [u32; 6] = [0, 1, 2, 3, 4, 7];
        const L2_SIZES: [u32; 6] = [0, 32 << 10, 64 << 10, 128 << 10, 256 << 10, 256 << 10];
        const L1_SIZES: [u32; 6] = [0, 4 << 10, 8 << 10, 16 << 10, 32 << 10, 32 << 10];
        for profile in [&crate::device::C6455, &crate::device::C6472] {
            let map = profile.map;
            // A DMA master's address is global wherever the device has global addresses
            // (SPRS612G Table 2-2), and a DMA master is the only requester that can ask about
            // L1P at all.
            let alias = map.global_alias_base(0).unwrap_or(0);
            let [l1p, l1d] = map.local_l1();
            let windows = [
                (L2CFG, alias + map.local_l2().0, map.l2_bytes(), L2_SIZES),
                (L1PCFG, alias + l1p.0, l1p.1, L1_SIZES),
                (L1DCFG, alias + l1d.0, l1d.1, L1_SIZES),
            ];
            for (register, base, bytes, sizes) in windows {
                for (index, mode) in MODES.into_iter().enumerate() {
                    let cache = sizes[index];
                    let mut fabric = Fabric::new(profile);
                    // Where the SRAM ends once this much of the window has become cache.
                    let split = base + bytes - cache;
                    if cache > 0 {
                        fabric.master_write32(split, 0xfeed_face);
                    }
                    if cache < bytes {
                        fabric.master_write32(split - 4, 0xfeed_face);
                    }
                    fabric.write32(0, register, mode);
                    if cache > 0 {
                        // The first word the cache took holds nothing and keeps nothing.
                        assert_eq!(fabric.master_read32(split), Some(0), "{register:#x} {mode}");
                        fabric.master_write32(split, 0x0bad_c0de);
                        assert_eq!(fabric.master_read32(split), Some(0), "{register:#x} {mode}");
                    }
                    if cache < bytes {
                        // The last word below it is still the SRAM it always was.
                        assert_eq!(
                            fabric.master_read32(split - 4),
                            Some(0xfeed_face),
                            "{register:#x} {mode}"
                        );
                    }
                }
            }
        }
    }

    /// The same policy where it actually bites: a dirty L1D line whose L2 line has already been
    /// evicted, so the victim arrives at an L2 that no longer has anywhere to put it.
    ///
    /// Two rules meet here and each is right on its own. L1D evicts dirty lines and writes them
    /// back; L2 allocates a line on a write that misses it. Composing them made a late victim
    /// allocate an L2 line the hardware never allocates - SPRU871K section 4.3.8.3: "L1D victim
    /// writebacks do not trigger line allocations in L2. L1D victims are written directly to
    /// external memory if they miss L2."
    ///
    /// Reaching that state takes the geometry rather than a big loop, because a linear sweep
    /// never gets there: the L1D victim trails the read pointer by an L1D capacity, which is far
    /// less than L2's, so its line is always still resident. What separates the two caches is
    /// that an L2 set spans two L1D sets - the L2 set field is address bits 12:7 and the L1D set
    /// field is bits 10:6, so within one L2 set only bit 6 is free, and the two halves of a
    /// 128-byte L2 line fall in different L1D sets. Conflicting in L2 through the *far* half of
    /// each line therefore evicts the victim's L2 line while leaving its L1D line alone.
    #[test]
    fn an_l1d_victim_that_misses_l2_does_not_allocate_a_line_there() {
        const DDR2: u32 = 0xe000_0000;
        const MAR224: u32 = MAR_BASE + 224 * 4;
        /// 32 KB of L2 cache over four ways: the stride that indexes one set.
        const L2_WAY_STRIDE: u32 = 8 * 1024;
        /// 4 KB of L1D cache over two ways.
        const L1D_WAY_STRIDE: u32 = 2 * 1024;
        const L2_LINE: u32 = 128;

        let mut fabric = Fabric::default();
        // Without the MAR's permit-copies bit nothing external is cached at all (SPRU871K
        // section 4.3.7.1), so there would be neither a fill nor a victim.
        fabric.write32(0, MAR224, 1);
        fabric.write32(0, L1DCFG, 1);
        fabric.write32(0, L2CFG, 1);

        // The victim: read to allocate the L1D line, then write to dirty it. L1D is
        // read-allocate, so this is the only way a line becomes dirty.
        let victim = DDR2;
        let seen = fabric.read32(0, victim);
        fabric.write32(0, victim, seen ^ 0xa5a5_5a5a);

        // Evict the victim's L2 line: four more lines in the same L2 set, addressed through the
        // far half of each so they land in the other L1D set and cannot touch the victim's L1D
        // line. Four is the associativity, so the set is turned over completely.
        for way in 1..=4 {
            fabric.read32(0, victim + way * L2_WAY_STRIDE + L2_LINE / 2);
        }
        let [_, _, l2] = fabric.megamodule(0).cache_views();
        assert!(
            !l2.lines.iter().any(|line| line.address == victim),
            "the conflicting reads have to evict the victim's own L2 line first"
        );

        // Now evict the victim from L1D, which is what sends it down. Two more lines in its L1D
        // set fill both ways; they sit in other L2 sets, so nothing else moves.
        for way in 1..=2 {
            fabric.read32(0, victim + way * L1D_WAY_STRIDE);
        }

        let [_, l1d, l2] = fabric.megamodule(0).cache_views();
        assert!(
            l1d.stats.dirty_evictions > 0,
            "the victim has to have left L1D dirty for there to be a writeback at all"
        );
        assert!(
            !l1d.lines.iter().any(|line| line.address == victim),
            "the victim is out of L1D"
        );
        // The whole point: the victim missed L2 and went past it to memory, rather than
        // allocating a line there and evicting something the hardware would have kept.
        assert!(
            !l2.lines.iter().any(|line| line.address == victim),
            "an L1D victim that misses L2 must not allocate a line in it"
        );
        // And the data reached memory: reading it back refills from the backing store.
        assert_eq!(fabric.read32(0, victim), 0xa5a5_5a5a);
    }

    /// Which ports a device has comes from its profile and nothing else, so the two devices
    /// differ by exactly the blocks the data manuals say they differ by: the C6472 adds an
    /// interrupt-control module in front of each of its two MACs, a boot controller, a
    /// power/sleep controller, a shared-memory controller with a profiler per core, and the
    /// chip-level registers; the C6455 adds the EMIFA, UTOPIA, the two McBSPs and the device
    /// configuration block.
    #[test]
    fn the_profile_decides_which_ports_a_device_has() {
        let ports = |profile| Fabric::new(profile).shared.ports.clone();
        let c6455 = ports(&crate::device::C6455);
        let c6472 = ports(&crate::device::C6472);

        // Every device has one EDMA3 with four transfer controllers, both PLLs, and the
        // peripherals that are not device-specific.
        for list in [&c6455, &c6472] {
            assert!(list.contains(&Port::Edma));
            assert!(list.contains(&Port::Mdio));
            assert!(list.contains(&Port::Pll));
            assert!(list.contains(&Port::Pll2));
            for index in 0..4 {
                assert!(list.contains(&Port::TransferController(index)));
            }
            // The chip-level registers claim a whole window, so nothing may be asked after
            // them.
            if let Some(position) = list.iter().position(|port| *port == Port::ChipConfig) {
                assert_eq!(position, list.len() - 1);
            }
        }

        assert_eq!(
            c6455
                .iter()
                .filter(|port| matches!(port, Port::Emac(_)))
                .count(),
            1
        );
        assert_eq!(
            c6472
                .iter()
                .filter(|port| matches!(port, Port::Emac(_)))
                .count(),
            2
        );

        for only_c6472 in [
            Port::Emic(0),
            Port::BootController,
            Port::Psc,
            Port::Smc,
            Port::SmcProfiler(0),
            Port::ChipConfig,
        ] {
            assert!(!c6455.contains(&only_c6472), "C6455 has {only_c6472:?}");
            assert!(c6472.contains(&only_c6472), "C6472 lacks {only_c6472:?}");
        }
        for only_c6455 in [Port::Emifa, Port::Mcbsp(0), Port::Devcfg] {
            assert!(c6455.contains(&only_c6455), "C6455 lacks {only_c6455:?}");
            assert!(!c6472.contains(&only_c6455), "C6472 has {only_c6455:?}");
        }
        // UTOPIA is on both: the C6472 carries the same ATM interface (SPRS612G Table 2-2).
        assert!(c6455.contains(&Port::Utopia) && c6472.contains(&Port::Utopia));
        // One profiler per core.
        assert_eq!(
            c6472
                .iter()
                .filter(|port| matches!(port, Port::SmcProfiler(_)))
                .count(),
            6
        );
    }

    /// A master lent out for its own transfer is not addressable by that transfer.
    ///
    /// This is a consequence of how the fabric hands a master to itself rather than a rule any
    /// document states: the slot is empty for the duration, so no port claims the master's
    /// register block and the access falls through to the memory behind the aperture. Nothing
    /// asserted it before, which meant a change to the lending protocol could alter it silently.
    #[test]
    fn a_lent_out_master_does_not_answer_its_own_transfer() {
        const TXIDVER: u32 = 0x02c8_0000;
        let mut fabric = Fabric::default();

        // In its slot, the MAC owns its register block and identifies itself.
        assert_eq!(fabric.owner_of(TXIDVER), Some(Port::Emac(0)));
        assert_eq!(fabric.read32(0, TXIDVER), 0x000c_0a08);

        // Lent out, nothing claims it.
        fabric.lend_emac(0, |_emac, fabric| {
            assert_eq!(fabric.owner_of(TXIDVER), None);
            assert_eq!(fabric.read32(0, TXIDVER), 0);
        });

        // Back in its slot, it answers again.
        assert_eq!(fabric.owner_of(TXIDVER), Some(Port::Emac(0)));
        assert_eq!(fabric.read32(0, TXIDVER), 0x000c_0a08);

        // The other masters stay reachable while one is lent out, which is what lets the EDMA3
        // transfer to and from a peripheral register during its own tick.
        fabric.lend_emac(0, |_emac, fabric| {
            assert_eq!(fabric.owner_of(0x02a0_0000), Some(Port::Edma));
        });
        fabric.lend_edma(|_edma, fabric| {
            assert_eq!(fabric.owner_of(TXIDVER), Some(Port::Emac(0)));
            assert_eq!(
                fabric.owner_of(0x02a0_0000),
                None,
                "the EDMA3 itself is out"
            );
        });
    }

    /// Every shared port answers at the base its data manual gives it, on the device that has
    /// it, and the lowest address in its window that is a register rather than a hole.
    ///
    /// Asserting *which* port owns an address, not only that some port does, is what catches a
    /// window that has drifted onto a neighbour's: `no_two_slave_ports_claim_one_address` would
    /// still pass if two ports swapped windows. Addresses come from SPRS276M Tables 7-13 to
    /// 7-38 for the C6455 and SPRS612G Tables 7-98 to 7-116 for the C6472.
    #[test]
    fn each_port_answers_at_its_documented_base() {
        let mut c6455 = Fabric::new(&crate::device::C6455);
        for (address, expected) in [
            (0x02a0_0000, Port::Edma),
            (0x02a2_0000, Port::TransferController(0)),
            (0x02a2_8000, Port::TransferController(1)),
            (0x02a3_0000, Port::TransferController(2)),
            (0x02a3_8000, Port::TransferController(3)),
            // The C6455's two timers sit 0x40000 apart (SPRS276M section 7.16).
            (0x0294_0000, Port::Timer(0)),
            (0x0298_0000, Port::Timer(1)),
            (0x02b0_0008, Port::Gpio),
            (0x02b0_4000, Port::I2c),
            // Both PLL controllers keep their registers well into their windows.
            (0x029a_00e4, Port::Pll),
            (0x029c_0118, Port::Pll2),
            (0x0288_0004, Port::Hpi),
            (0x7800_0000, Port::Ddr2),
            (0x7000_0000, Port::Emifa),
            (0x02b4_0000, Port::Utopia),
            (0x028c_0000, Port::Mcbsp(0)),
            (0x0290_0000, Port::Mcbsp(1)),
            // The device configuration block owns two windows.
            (0x02a8_0000, Port::Devcfg),
            (0x02ac_0004, Port::Devcfg),
            (0x02c8_0000, Port::Emac(0)),
            (0x02c8_1800, Port::Mdio),
        ] {
            assert_eq!(
                c6455.owner_of(address),
                Some(expected),
                "C6455 {address:#010x}"
            );
        }

        let mut c6472 = Fabric::new(&crate::device::C6472);
        for (address, expected) in [
            (0x02a0_0000, Port::Edma),
            (0x02a2_0000, Port::TransferController(0)),
            (0x02a3_8000, Port::TransferController(3)),
            (0x02b0_0008, Port::Gpio),
            (0x02b0_4000, Port::I2c),
            (0x029a_00e4, Port::Pll),
            (0x029c_0118, Port::Pll2),
            (0x0288_0004, Port::Hpi),
            (0x7800_0000, Port::Ddr2),
            (0x02b4_0000, Port::Utopia),
            // Two MACs 0x40000 apart, each with its own interrupt-control module in front, and
            // one MDIO they share (SPRS612G Tables 7-101 and 7-103).
            (0x02c8_0000, Port::Emac(0)),
            (0x02cc_0000, Port::Emac(1)),
            (0x02c8_1000, Port::Emic(0)),
            (0x02cc_1000, Port::Emic(1)),
            // The UTOPIA's configuration and PDMA windows (SPRS612G Tables 7-142 and 7-143).
            (0x02b4_0000, Port::Utopia),
            (0x02b4_0020, Port::Utopia),
            (0x02b4_0104, Port::Utopia),
            (0x02bc_0000, Port::Pdma),
            (0x02bc_0180, Port::Pdma),
            (0x02c8_1800, Port::Mdio),
            // Twelve timers on 64 KB boundaries from 0x025E 0000: 0-5 belong to the
            // megamodules, 6-11 are shared (SPRS612G section 7.16.2).
            (0x025e_0000, Port::Timer(0)),
            (0x0263_0000, Port::Timer(5)),
            (0x0264_0000, Port::Timer(6)),
            (0x0269_0000, Port::Timer(11)),
            // The device-level blocks a C6455 does not have.
            (0x02ab_0000, Port::BootController),
            (0x02ae_0000, Port::Psc),
            (0x02aa_8000, Port::Smc),
            (0x02a8_0000, Port::ChipConfig),
        ] {
            assert_eq!(
                c6472.owner_of(address),
                Some(expected),
                "C6472 {address:#010x}"
            );
        }
    }

    /// The identification registers are profile data, not something derived from a capability.
    ///
    /// MM_REVID used to pick its value from `has_boot_controller`, which happened to give the
    /// right answer for the two modelled devices and would have given a wrong one for any
    /// device whose revision moved without its block set changing. It is a field now, and
    /// nothing pinned it before this case.
    #[test]
    fn each_device_reports_its_own_identification_registers() {
        // MM_REVID: VERSION 1 in the upper half, the silicon revision in the lower. SPRS612G
        // Table 2-1 gives 0003h for the C6472; the C6455's megamodule is revision 0.
        let mut c6455 = Fabric::new(&crate::device::C6455);
        assert_eq!(c6455.read32(0, MM_REVID), 0x0001_0000);
        let mut c6472 = Fabric::new(&crate::device::C6472);
        assert_eq!(c6472.read32(0, MM_REVID), 0x0001_0003);

        // Every megamodule of a multicore device reports the same revision: it is the core's
        // design, not its instance.
        for core in 0..6 {
            assert_eq!(c6472.read32(core, MM_REVID), 0x0001_0003, "core {core}");
        }

        // The register is read-only: a write is accepted and discarded rather than refused.
        c6472.write32(0, MM_REVID, 0xffff_ffff);
        assert_eq!(c6472.read32(0, MM_REVID), 0x0001_0003);
    }

    /// The same address names a different block on each device, which is why peripheral
    /// ownership has to come from the profile rather than from one table for the family.
    ///
    /// 0x02A8 0000 is the C6455's device configuration block and the C6472's chip-level
    /// registers; 0x02C8 1004 is EWCTL inside the C6455's own MAC and EMIC0's PSCFG on the
    /// C6472 (SPRS276M sections 7.15 and 7.19; SPRS612G Tables 7-98 and 7-105).
    #[test]
    fn one_address_names_a_different_block_on_each_device() {
        let mut c6455 = Fabric::new(&crate::device::C6455);
        let mut c6472 = Fabric::new(&crate::device::C6472);
        assert_eq!(c6455.owner_of(0x02a8_0000), Some(Port::Devcfg));
        assert_eq!(c6472.owner_of(0x02a8_0000), Some(Port::ChipConfig));
        assert_eq!(c6455.owner_of(0x02c8_1004), Some(Port::Emac(0)));
        assert_eq!(c6472.owner_of(0x02c8_1004), Some(Port::Emic(0)));
    }

    /// A window belonging to a block the device does not have is claimed by nobody, so the
    /// access falls through to memory instead of being answered by whichever port sits nearby.
    /// The timers are the sharpest case: both devices have them, at different addresses.
    #[test]
    fn a_block_a_device_lacks_claims_nothing() {
        let mut c6455 = Fabric::new(&crate::device::C6455);
        for absent in [
            0x02cc_0000, // the second MAC
            0x02cc_1000, // its interrupt-control module
            0x02ab_0000, // the boot controller
            0x02ae_0000, // the power/sleep controller
            0x02aa_8000, // the shared-memory controller
            0x02c2_0000, // its per-core profilers
            0x025e_0000, // the C6472's timer 0
        ] {
            assert_eq!(c6455.owner_of(absent), None, "C6455 {absent:#010x}");
        }
        let mut c6472 = Fabric::new(&crate::device::C6472);
        for absent in [
            0x7000_0000, // the EMIFA
            0x028c_0000, // McBSP 0
            0x0290_0000, // McBSP 1
            0x02ac_0004, // the device configuration block's second window
            0x0294_0000, // the C6455's timer 0
            0x0298_0000, // the C6455's timer 1
        ] {
            assert_eq!(c6472.owner_of(absent), None, "C6472 {absent:#010x}");
        }
    }

    /// The C6472 gives every core its own shared-memory profiler, 16 KB apart from
    /// 0x02C2 0000 (SPRUEG5D section 3). Each answers for its own core and no other.
    #[test]
    fn each_core_has_its_own_shared_memory_profiler() {
        let mut fabric = Fabric::new(&crate::device::C6472);
        for core in 0..6usize {
            let base = 0x02c2_0000 + 0x4000 * core as u32;
            assert_eq!(
                fabric.owner_of(base),
                Some(Port::SmcProfiler(core)),
                "profiler {core}"
            );
        }
        // The window stops after the sixth: a seventh core's address belongs to nobody.
        assert_eq!(fabric.owner_of(0x02c2_0000 + 0x4000 * 6), None);
    }

    #[test]
    fn c6472_emac_softreset_preserves_emic_mdio_and_descriptor_ram() {
        // SPRUEF8F §2.15.1, p.64: unlike EMAC, "the MDIO, EMIC modules,
        // and CPPI buffer managers cannot be placed in reset" by a register.
        let mut fabric = Fabric::new(&crate::device::C6472);
        fabric.write32(0, 0x02c8_1828, 1);
        for base in [0x02c8_0000, 0x02cc_0000] {
            fabric.write32(0, base + 0x1004, 17);
            for core in 0..6 {
                fabric.write32(0, base + 0x1020 + 4 * core, 0x00ff_ff1e);
            }
            for channel in 0..8 {
                for offset in [0x1100, 0x1140] {
                    fabric.write32(0, base + offset + 4 * channel, 0x0005_030c);
                }
            }
            fabric.write32(0, base + 0x2000, 0x1234_5678);
            fabric.write32(0, base + 0x10c, 64);
            fabric.write32(0, base + 0x174, 1);
            assert_eq!(fabric.read32(0, base + 0x10c), 1518);
            assert_eq!(fabric.read32(0, base + 0x1004), 17);
            for core in 0..6 {
                assert_eq!(fabric.read32(0, base + 0x1020 + 4 * core), 0x00ff_ff1e);
            }
            for channel in 0..8 {
                for offset in [0x1100, 0x1140] {
                    assert_eq!(fabric.read32(0, base + offset + 4 * channel), 0x0005_0300);
                }
            }
            assert_eq!(fabric.read32(0, base + 0x2000), 0x1234_5678);
            assert_eq!(fabric.read32(0, 0x02c8_1828), 1);
        }
    }

    #[test]
    fn c6455_emac_softreset_preserves_the_separate_control_module() {
        // SPRU975E §2.14.1, p.53: "the MDIO and EMAC control modules cannot be
        // placed in reset from a register inside their memory map"; §3.1 Table 11
        // places EWCTL in that separate control module.
        let mut fabric = Fabric::new(&crate::device::C6455);
        fabric.write32(0, 0x02c8_1004, 1);
        fabric.write32(0, 0x02c8_1804, 1 << 30);
        fabric.write32(0, 0x02c8_1828, 1);
        fabric.write32(0, 0x02c8_010c, 64);
        fabric.write32(0, 0x02c8_0174, 1);
        assert_eq!(fabric.read32(0, 0x02c8_010c), 1518);
        assert_eq!(fabric.read32(0, 0x02c8_0174), 0);
        assert_eq!(fabric.read32(0, 0x02c8_1004), 1);
        assert_eq!(fabric.read32(0, 0x02c8_1804) & (1 << 30), 1 << 30);
        assert_eq!(fabric.read32(0, 0x02c8_1828), 1);
        // The surviving gate must still deliver a newly enabled MAC source.
        fabric.write32(0, 0x02c8_0200, 0x8000_0000);
        fabric.write32(0, 0x02c8_00b8, 1);
        fabric.tick_once();
        assert_ne!(fabric.read32(0, 0x0180_0000) & (1 << 17), 0);
    }

    #[test]
    fn c6455_ewctl_gates_mdio_completion_before_the_cpu_event() {
        // SPRU975E §3.2 Table 12, p.61: with INTEN=0, EMAC_MDIO_INT is
        // "kept de-asserted", including USERINT from §2.16.2.2.
        const EWCTL: u32 = 0x02c8_1004;
        const EVTFLAG0: u32 = 0x0180_0000;
        const EVTCLR0: u32 = 0x0180_0040;
        const MACINT: u32 = 1 << 17;
        let mut fabric = Fabric::new(&crate::device::C6455);
        fabric.write32(0, 0x02c8_1804, 1 << 30);
        fabric.write32(0, 0x02c8_1828, 1);
        fabric.write32(0, 0x02c8_1880, (1 << 31) | (1 << 16));
        for _ in 0..32 {
            fabric.tick_once();
        }
        assert_eq!(fabric.read32(0, 0x02c8_1824), 1);
        assert_ne!(fabric.read32(0, 0x02c8_0090) & (1 << 31), 0);
        assert_eq!(fabric.read32(0, EVTFLAG0) & MACINT, 0);
        fabric.write32(0, EWCTL, 1);
        fabric.tick_once();
        assert_eq!(fabric.read32(0, EVTFLAG0) & MACINT, MACINT);
        fabric.write32(0, EWCTL, 0);
        fabric.write32(0, EVTCLR0, MACINT);
        for _ in 0..32 {
            fabric.tick_once();
        }
        assert_eq!(fabric.read32(0, EVTFLAG0) & MACINT, 0);
        fabric.write32(0, EWCTL, 1);
        fabric.tick_once();
        assert_eq!(fabric.read32(0, EVTFLAG0) & MACINT, MACINT);
    }

    /// MACINVECTOR is the one shared register the router answers itself, because it reports the
    /// MDIO's interrupts from inside an EMAC's register map and only the control module sees
    /// both (SPRU975E section 2.16.4). On a C6472 there are two of them and one MDIO, so each
    /// MAC's vector must carry that MAC's own interrupts and the shared MDIO's.
    #[test]
    fn each_mac_input_vector_composes_its_own_mac_with_the_shared_mdio() {
        const MDIO_CONTROL: u32 = 0x02c8_1804;
        const MDIO_USERINTMASKSET: u32 = 0x02c8_1828;
        const MDIO_USERACCESS0: u32 = 0x02c8_1880;
        const MDIO_ENABLE: u32 = 1 << 30;
        const USER_BIT: u32 = 1 << 31;
        const LINK_BIT: u32 = 1 << 30;

        let mut fabric = Fabric::new(&crate::device::C6472);
        for base in [0x02c8_0000u32, 0x02cc_0000] {
            assert_eq!(fabric.read32(0, base + 0x90) & (LINK_BIT | USER_BIT), 0);
        }

        // Complete a user access on channel 0, which is what raises the interrupt: enable the
        // module, unmask the channel, issue a read of the PHY, and let the transaction finish.
        fabric.write32(0, MDIO_CONTROL, MDIO_ENABLE);
        fabric.write32(0, MDIO_USERINTMASKSET, 1);
        fabric.write32(0, MDIO_USERACCESS0, (1 << 31) | (1 << 16));
        for _ in 0..32 {
            fabric.tick_once();
        }

        // One module, two MACs: both vectors report the shared MDIO's interrupt.
        for base in [0x02c8_0000u32, 0x02cc_0000] {
            assert_eq!(
                fabric.read32(0, base + 0x90) & USER_BIT,
                USER_BIT,
                "MACINVECTOR at {base:#010x} lost the shared MDIO's interrupt"
            );
        }
        // No port owns the composed register: the router decodes it before the walk.
        assert_eq!(fabric.owner_of(0x02c8_0090), None);
    }

    /// Reads are the only ownership probe that can be swept over a live device. Several ports
    /// have write-only registers, while PSC reset requests and boot-release writes are destructive,
    /// so probing write ownership would require rebuilding the device for every address.
    #[test]
    fn no_two_slave_ports_claim_one_address() {
        for profile in [&crate::device::C6455, &crate::device::C6472] {
            let mut fabric = Fabric::new(profile);
            let context = fabric.port_context();
            let mut windows = vec![
                // The megamodule-local IDMA port is included once: every core has the same private
                // address, and the fabric selects the requesting core before this dispatch.
                (0x0182_0000, 0x114),
                (0x02a0_0000, 0x3000),
                (0x02a0_4000, 0x2000),
                (0x02a2_0000, 0x8000),
                (0x02a2_8000, 0x8000),
                (0x02a3_0000, 0x8000),
                (0x02a3_8000, 0x8000),
                (0x02b0_0000, 0x34),
                (0x02b0_4000, 0x50),
                (0x029a_0000, 0x168),
                (0x029c_0000, 0x154),
                (0x0288_0000, 0x3c),
                (0x7800_0000, 0xe8),
                (0x02c8_1800, 0x90),
            ];
            windows.extend(
                profile
                    .emac_bases
                    .iter()
                    .copied()
                    .map(|base| (base, 0x1000)),
            );
            windows.extend(profile.emic_bases.iter().copied().map(|base| (base, 0x800)));
            windows.extend(profile.timer_bases.iter().copied().map(|base| (base, 0x2c)));
            if profile.has_emifa {
                windows.push((0x7000_0000, 0x100));
            }
            if profile.has_mcbsp {
                windows.extend([
                    (0x028c_0000, 0x100),
                    (0x0290_0000, 0x100),
                    (0x3000_0000, 4),
                    (0x3000_0010, 4),
                    (0x3400_0000, 4),
                    (0x3400_0010, 4),
                ]);
            }
            if profile.utopia.is_some() {
                windows.extend([
                    (0x02b4_0000, 0x20),
                    (0x3c00_0000, 0x400),
                    (0x3c00_0400, 0x400),
                ]);
            }
            if profile.has_utopia_pdma() {
                windows.extend([(0x02b4_0020, 0x24), (0x02b4_0100, 8), (0x02bc_0000, 0x200)]);
            }
            if profile.has_device_config {
                windows.extend([(0x02a8_0000, 0x0c), (0x02ac_0000, 0x58)]);
            }
            if profile.has_boot_controller {
                windows.extend([
                    (0x02a8_0000, 0x0002_0000),
                    (0x02aa_8000, 0x8000),
                    (0x02ab_0000, 0x8000),
                    (0x02ae_0000, 0x1_0000),
                ]);
                windows.extend(
                    (0..profile.cores()).map(|core| (0x02c2_0000 + 0x4000 * core as u32, 0x4000)),
                );
            }

            // Walk the same list the router walks, so the proof cannot name a set of ports the
            // dispatch does not have. The megamodule-local IDMA port is checked alongside them:
            // every core has it at the same private address, and the fabric selects the
            // requesting core before this dispatch.
            for (base, size) in windows {
                for address in (base..base + size).step_by(4) {
                    let mut claimed_by: Option<String> = None;
                    let mut claim = |name: String, taken: bool| {
                        if !taken {
                            return;
                        }
                        if let Some(first) = &claimed_by {
                            panic!(
                                "{} address {address:#010x} is claimed by {first} and {name}",
                                profile.name
                            );
                        }
                        claimed_by = Some(name);
                    };
                    let taken = SlavePort::read(&mut fabric.mm[0].idma, address, context).is_some();
                    claim("IDMA".to_owned(), taken);
                    for index in 0..fabric.shared.ports.len() {
                        let which = fabric.shared.ports[index];
                        let Some(port) = fabric.shared.port(which) else {
                            continue;
                        };
                        let taken = port.read(address, context).is_some();
                        claim(format!("{which:?}"), taken);
                    }
                }
            }
        }
    }

    /// The C6455's EWCTL and the C6472's EMIC0 PSCFG share 0x02C8 1004, but they belong to
    /// different device-specific control blocks (SPRS276M section 7.15; SPRS612G Table 7-105).
    #[test]
    fn the_c6472_emac_does_not_claim_the_interrupt_controller_window() {
        let mut c6472 = Fabric::new(&crate::device::C6472);
        c6472.write32(0, 0x02c8_1004, 0x1234);
        assert_eq!(c6472.read32(0, 0x02c8_1004), 0x1234);

        let mut c6455 = Fabric::new(&crate::device::C6455);
        c6455.write32(0, 0x02c8_1004, 1);
        assert_eq!(c6455.read32(0, 0x02c8_1004), 1);
    }

    #[test]
    fn l2_writeback_controls_dma_visibility() {
        const DDR: u32 = 0xe000_1000;
        let mut fabric = Fabric::default();
        fabric.load_bytes(0, DDR, &[0]);
        fabric.write32(0, MAR_BASE + 224 * 4, 1);
        fabric.write32(0, L2CFG, 1);
        fabric.write_u8(0, DDR, 0xaa);
        assert_eq!(
            fabric.shared.backing.read_byte(Requester::Dma, DDR),
            Some(0)
        );
        fabric.write32(0, L2WB, 1);
        assert_eq!(
            fabric.shared.backing.read_byte(Requester::Dma, DDR),
            Some(0xaa)
        );
    }

    #[test]
    fn partial_host_write_preserves_dirty_neighbors() {
        const DDR: u32 = 0xe000_1000;
        let mut fabric = Fabric::default();
        fabric.load_bytes(0, DDR, &[0; 8]);
        fabric.write32(0, MAR_BASE + 224 * 4, 1);
        fabric.write32(0, L2CFG, 1);

        fabric.write_u8(0, DDR, 0xaa);
        fabric.host_write_bytes(0, DDR + 4, &[0xbb]);

        assert_eq!(fabric.read_u8(0, DDR), 0xaa);
        assert_eq!(fabric.read_u8(0, DDR + 4), 0xbb);
    }

    #[test]
    fn l1d_invalidate_discards_stale_dma_view() {
        const BUFFER: u32 = 0x0080_1000;
        let mut fabric = Fabric::default();
        fabric.load_bytes(0, BUFFER, &[1]);
        fabric.write32(0, L1DCFG, 1);
        assert_eq!(fabric.read_u8(0, BUFFER), 1);
        fabric.shared.backing.write_byte(Requester::Dma, BUFFER, 2);
        assert_eq!(fabric.read_u8(0, BUFFER), 1);
        fabric.write32(0, L1DINV, 1);
        assert_eq!(fabric.read_u8(0, BUFFER), 2);
    }

    #[test]
    fn l2_range_writeback_flushes_l1d_and_l2_without_invalidating() {
        const DDR: u32 = 0xe000_1000;
        let mut fabric = Fabric::default();
        fabric.load_bytes(0, DDR, &[0]);
        fabric.write32(0, MAR_BASE + 224 * 4, 1);
        fabric.write32(0, L2CFG, 1);
        fabric.write32(0, L1DCFG, 1);

        assert_eq!(fabric.read_u8(0, DDR), 0);
        fabric.write_u8(0, DDR, 0xaa);
        assert_eq!(
            fabric.shared.backing.read_byte(Requester::Dma, DDR),
            Some(0)
        );

        fabric.write32(0, L2WBAR, DDR);
        fabric.write32(0, L2WWC, 1);
        assert_eq!(
            fabric.shared.backing.read_byte(Requester::Dma, DDR),
            Some(0xaa)
        );

        fabric.shared.backing.write_byte(Requester::Dma, DDR, 0xbb);
        assert_eq!(fabric.read_u8(0, DDR), 0xaa);
    }

    /// A page that grants nothing to User mode refuses a User read, latches the address and
    /// the permission it refused, and raises the L1D unit's event - while a Supervisor read
    /// of the same page still goes through (SPRU871K chapter 8).
    #[test]
    fn a_page_refuses_the_permission_it_does_not_grant_and_reports_where() {
        let mut fabric = Fabric::default();
        // L1DMPPA0 covers the first kilobyte of L1D SRAM: supervisor everything, user nothing.
        fabric.write32(0, 0x0184_ae00, 0x0000_0138);
        let address = 0x00f0_0010;
        assert!(fabric.check_access(0, address, Permission::Read, false));
        assert!(!fabric.check_access(0, address, Permission::Read, true));
        assert_eq!(fabric.read32(0, 0x0184_ac00), address); // L1DMPFAR
        assert_eq!(fabric.read32(0, 0x0184_ac04), 0x104); // L1DMPFSR: LOCAL and UR
        assert_eq!(
            fabric.megamodule(0).event_flag_registers()[3] & (1 << 26),
            1 << 26
        ); // event 122

        // The neighbouring page is untouched, and a fully open page never even looks.
        assert!(fabric.check_access(0, 0x00f0_0400, Permission::Read, true));
        // A write that misses the command bit leaves the fault standing; the command bit
        // clears the address as well as the status.
        fabric.write32(0, 0x0184_ac08, 0x104);
        assert_eq!(fabric.read32(0, 0x0184_ac04), 0x104);
        fabric.write32(0, 0x0184_ac08, 1);
        assert_eq!(fabric.read32(0, 0x0184_ac04), 0);
        assert_eq!(fabric.read32(0, 0x0184_ac00), 0);
    }

    /// Denying execute makes the fetch path deliver nothing, so an address in that page reads
    /// as a NOP rather than as whatever the page happens to hold.
    #[test]
    fn a_page_without_execute_permission_yields_no_instruction() {
        let mut fabric = Fabric::default();
        fabric.load_bytes(0, 0x0080_0000, &0x0210_a358u32.to_le_bytes());
        assert_eq!(fabric.fetch32(0, 0x0080_0000), 0x0210_a358);
        // L2MPPA0 covers the first 64 KB of L2 SRAM: read and write, no execute.
        fabric.write32(0, 0x0184_a200, 0x0000_0130);
        assert!(fabric.check_access(0, 0x0080_0000, Permission::Read, false));
        assert!(!fabric.check_access(0, 0x0080_0000, Permission::Execute, false));
        assert_eq!(
            fabric.megamodule(0).event_flag_registers()[3] & (1 << 28),
            1 << 28
        ); // event 124
    }

    /// A DMA master reading another core's L2 SRAM is served from that core's L1D when the CPU
    /// holds the line dirty, whichever of the two addresses the CPU used for it (SPRU862B
    /// section 2.4.2, SPRS612G Table 2-2).
    #[test]
    fn a_dma_read_of_a_peer_core_l2_is_snooped_against_that_core_l1d() {
        let mut fabric = Fabric::new(&crate::device::C6472);
        // Core 3 caches its own L2 in L1D and dirties a line through its local address.
        fabric.write32(3, crate::memory::L1DCFG, 4);
        fabric.load_bytes(3, 0x0080_2000, &[0; 64]);
        // L1D allocates on a read miss, so the line has to be read before a store leaves it
        // dirty in the cache (SPRU862B section 3.4).
        assert_eq!(fabric.read32(3, 0x0080_2000), 0);
        fabric.write32(3, 0x0080_2000, 0xdead_beef);
        // The line is dirty in core 3's L1D and has not reached the SRAM.
        assert_eq!(
            fabric
                .shared
                .backing
                .read_bytes(Requester::Dma, 0x1380_2000, 4),
            vec![0, 0, 0, 0]
        );
        // The EMAC reads the buffer through the global alias and sees the CPU's data.
        assert_eq!(
            EmacMaster(&mut fabric).dma_read_u32(0x1380_2000),
            0xdead_beef
        );
        // And a DMA write through the alias is snooped into that core's L1D, so the CPU sees
        // it without any cache operation.
        EmacMaster(&mut fabric).dma_write_u32(0x1380_2000, 0x0bad_c0de);
        assert_eq!(fabric.read32(3, 0x0080_2000), 0x0bad_c0de);
        // No other core's memory moved.
        assert_eq!(fabric.read32(2, 0x0080_2000), 0);
    }

    /// The C6472 has two Ethernet MACs whose register blocks are 0x40000 apart, and one MDIO
    /// they share (SPRS612G Tables 7-101 and 7-103).
    #[test]
    fn a_c6472_answers_for_both_macs_and_one_mdio() {
        let mut fabric = Fabric::new(&crate::device::C6472);
        // TXIDVER identifies each port.
        assert_eq!(fabric.read32(0, 0x02c8_0000), 0x000c_0a08);
        assert_eq!(fabric.read32(0, 0x02cc_0000), 0x000c_0a08);
        // Each port keeps its own state: enabling receive on EMAC1 leaves EMAC0 alone.
        fabric.write32(0, 0x02cc_0014, 1);
        assert_eq!(fabric.read32(0, 0x02cc_0014), 1);
        assert_eq!(fabric.read32(0, 0x02c8_0014), 0);
        // The MDIO answers once, in EMAC0's control window (SPRUEF8F section 4.2).
        assert_eq!(fabric.read32(0, 0x02c8_1800), 0x0007_0103);
        // The C6455 has neither a second MAC nor an interrupt-control module.
        let mut single = Fabric::default();
        assert_eq!(single.read32(0, 0x02cc_0000), 0);
    }

    /// The whole point of a C6472 module: six cores, one Ethernet connection.
    ///
    /// One MAC receives a frame into a buffer that belongs to one particular core - reached by
    /// the global address, because that is the only address the EMAC's DMA can use (SPRS612G
    /// Table 2-2) - and the receive interrupt is delivered only to the core whose EW_INTCTL
    /// enabled that channel (SPRUEF8F section 3.1). The other five cores see neither the data
    /// nor the interrupt.
    #[test]
    fn one_ethernet_connection_delivers_a_frame_to_one_core_of_six() {
        const EMAC0: u32 = 0x02c8_0000;
        const RXCONTROL: u32 = EMAC0 + 0x14;
        const RXINTMASKSET: u32 = EMAC0 + 0xa8;
        const MACCONTROL: u32 = EMAC0 + 0x160;
        const RXMBPENABLE: u32 = EMAC0 + 0x100;
        const RX0HDP: u32 = EMAC0 + 0x620;
        const RX0CP: u32 = EMAC0 + 0x660;
        const EW_INTCTL0: u32 = 0x02c8_1020;
        /// A descriptor and a packet buffer in core 4's L2 SRAM, named globally.
        const DESCRIPTOR: u32 = 0x1480_1000;
        const BUFFER: u32 = 0x1480_2000;
        const OWNER: u32 = 1 << 29;

        let mut fabric = Fabric::new(&crate::device::C6472);
        // Core 4 owns the receive ring: it enables channel 0's interrupt in its own EW_INTCTL,
        // and no other core enables anything.
        fabric.write32(4, EW_INTCTL0 + 4 * 4, 1 << 16);
        // The driver builds one empty receive descriptor: no next, a 256-byte buffer, owned by
        // the MAC (SPRU975E section 2.5.5).
        fabric.write32(4, DESCRIPTOR, 0);
        fabric.write32(4, DESCRIPTOR + 4, BUFFER);
        fabric.write32(4, DESCRIPTOR + 8, 256);
        fabric.write32(4, DESCRIPTOR + 12, OWNER);
        fabric.write32(4, RXINTMASKSET, 1);
        // Copy every frame to channel 0, enable the receive DMA, and take the interface out of
        // reset - the last one is GMIIEN, which TI's sequence writes after the DMA enables
        // (SPRU975E section 2.15.4 step 17).
        fabric.write32(4, RXMBPENABLE, 1 << 21);
        fabric.write32(4, RXCONTROL, 1);
        fabric.write32(4, MACCONTROL, 1 << 5);
        fabric.write32(4, RX0HDP, DESCRIPTOR);

        let mut frame = vec![0u8; 64];
        frame[..6].copy_from_slice(&[0x02, 0x00, 0x00, 0x11, 0x22, 0x33]);
        frame[12] = 0x08;
        frame[14] = 0xc6;
        frame[15] = 0x47;
        fabric.inject_frame_on(0, frame.clone());
        for _ in 0..256 {
            fabric.tick_once();
        }

        // The frame landed in core 4's own L2 SRAM, reachable at its local address.
        let received = fabric.read_bytes(4, 0x0080_2000, 16);
        assert_eq!(received[..6], frame[..6]);
        assert_eq!(&received[12..16], &[0x08, 0x00, 0xc6, 0x47]);
        // The descriptor came back to software with the packet length recorded.
        let flags_and_length = fabric.read32(4, 0x0080_100c);
        assert_eq!(flags_and_length & 0xffff, 64, "packet length");
        assert_eq!(flags_and_length & OWNER, 0, "the MAC released the buffer");

        // MACRXINT0 is event 5, and only core 4 asked for it.
        let flagged: Vec<usize> = (0..6)
            .filter(|core| fabric.megamodule(*core).event_flag_registers()[0] & (1 << 5) != 0)
            .collect();
        assert_eq!(flagged, vec![4], "only the core that enabled the channel");
        // No other core's memory was touched.
        for core in [0, 3, 5] {
            assert_eq!(fabric.read_bytes(core, 0x0080_2000, 4), vec![0, 0, 0, 0]);
        }

        // Another core can take the next frame by owning the ring itself: core 1 enables the
        // channel, core 4 disables it, and the interrupt follows.
        fabric.write32(4, EW_INTCTL0 + 4 * 4, 0);
        fabric.write32(1, EW_INTCTL0 + 4, 1 << 16);
        fabric.write32(4, RX0CP, DESCRIPTOR);
        fabric.write32(1, 0x1180_1000, 0);
        fabric.write32(1, 0x1180_1004, 0x1180_2000);
        fabric.write32(1, 0x1180_1008, 256);
        fabric.write32(1, 0x1180_100c, OWNER);
        fabric.write32(1, RX0HDP, 0x1180_1000);
        for core in 0..6 {
            fabric.write32(core, 0x0180_0040, u32::MAX); // EVTCLR0
        }
        fabric.inject_frame_on(0, frame);
        for _ in 0..256 {
            fabric.tick_once();
        }
        let flagged: Vec<usize> = (0..6)
            .filter(|core| fabric.megamodule(*core).event_flag_registers()[0] & (1 << 5) != 0)
            .collect();
        assert_eq!(flagged, vec![1], "the interrupt follows the enable");
        assert_eq!(
            fabric.read_bytes(1, 0x0080_2000, 6),
            vec![0x02, 0, 0, 0x11, 0x22, 0x33]
        );
    }

    /// The SYSCLK3 schedule is invisible: a C6455 timer the fabric only steps on its due edges
    /// raises its events on the same cycles, and answers every register read the same, as a
    /// timer stepped on every edge - whatever the CPU writes to it and whenever it looks.
    #[test]
    fn a_scheduled_timer_matches_one_stepped_on_every_edge() {
        use crate::sysclk3::TestRng;
        const BASE: u32 = 0x0294_0000;
        // CNTLO, CNTHI, PRDLO, PRDHI, TCR, TGCR and WDTCR (SPRU968 Table 6).
        const REGISTERS: [u32; 7] = [0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x28];
        // The watchdog's key sequences (SPRU968 Figure 12): arm it, service it, or time it out
        // with a wrong key, after which it stays out of the way until reset.
        const WATCHDOG: [&[u32]; 3] = [
            &[0xa5c6_4000, 0xda7e_4000],
            &[0xa5c6_0000, 0xda7e_0000],
            &[0x1234_0000],
        ];
        let mut fabric = Fabric::new(&crate::device::C6455);
        let context = fabric.port_context();
        let mut stepped = Timer::new(BASE);
        let events = [0, 1].map(|half| fabric.profile.events.timer(0, half).unwrap().1);
        let mut rng = TestRng::new(0x5c3d_ed9e);
        let mut raised = [0; 2];
        let write = |fabric: &mut Fabric, stepped: &mut Timer, offset: u32, value: u32| {
            fabric.write32(0, BASE + offset, value);
            stepped.write(BASE + offset, value, context);
        };
        for _ in 0..2_000_000 {
            match rng.below(1500) {
                0 => {
                    let offset = REGISTERS[rng.below(6) as usize];
                    let value = match offset {
                        // Counts below periods of a few hundred edges, and a high half that
                        // is often zero so the 64-bit and watchdog modes match too.
                        0x10 => rng.below(40) as u32,
                        0x18 => 40 + rng.below(360) as u32,
                        0x14 | 0x1c if rng.below(2) == 0 => 0,
                        0x14 => rng.below(40) as u32,
                        0x1c => 40 + rng.below(360) as u32,
                        // Any pulse width and clock mode, and mostly continuous halves.
                        0x20 => {
                            let enable = [2, 2, 2, 1][rng.below(4) as usize] * 0x0040_0040;
                            (rng.u32() & 0x0039_0039) | enable
                        }
                        // Any mode and prescaler, mostly with both halves out of reset.
                        _ => (rng.u32() & 0xff0c) | if rng.below(4) == 0 { 0 } else { 3 },
                    };
                    write(&mut fabric, &mut stepped, offset, value);
                }
                1 if rng.below(20) == 0 => {
                    for key in WATCHDOG[rng.below(3) as usize] {
                        write(&mut fabric, &mut stepped, 0x28, *key);
                    }
                }
                2..=9 => {
                    let offset = REGISTERS[rng.below(REGISTERS.len() as u64) as usize];
                    assert_eq!(
                        fabric.read32(0, BASE + offset),
                        stepped.read(BASE + offset, context).unwrap(),
                        "register {offset:#x}"
                    );
                }
                _ => {}
            }
            fabric.tick_once();
            let expected = stepped.tick();
            let flags = fabric.megamodule(0).event_flag_registers();
            for half in 0..2 {
                let (bank, bit) = (events[half] / 32, 1 << (events[half] % 32));
                assert_eq!(flags[bank] & bit != 0, expected[half], "half {half}");
                if expected[half] {
                    fabric.write32(0, EVTCLR0 + 4 * bank as u32, bit);
                    raised[half] += 1;
                }
            }
        }
        assert!(
            raised[0] > 200 && raised[1] > 20,
            "matches seen: {raised:?}"
        );
    }

    mod utopia_pdma {
        use super::*;

        const UTOPIA: u32 = 0x02b4_0000;
        const PDMA: u32 = 0x02bc_0000;
        const RXC0: u32 = PDMA + 0x180;
        const L1D: u32 = 0x10f0_0000;
        const SLAVE: u8 = 3;
        const PACKET_BYTES: u32 = 56;
        const PRINT: usize = 30;

        /// A C6472 with its UTOPIA enabled as slave 3 in 8-bit, multi-PHY mode (little-endian
        /// assembly, as the device is) and channel 8 programmed as the boot ROM does: ABU into a
        /// four-cell buffer in core 0's L1D, 32-bit elements, interrupt at half and end.
        fn receiver(enable_channel: bool) -> Fabric {
            let mut fabric = Fabric::new(&crate::device::C6472);
            fabric.master_write32(UTOPIA, (u32::from(SLAVE) << 24) | (1 << 14) | 1);
            let group = (2 << 22) | (1 << 20) | (1 << 17) | (1 << 16) | (13 << 12) | (1 << 11);
            fabric.master_write32(PDMA + 0x64, group | (13 << 4));
            fabric.master_write32(RXC0, L1D);
            fabric.master_write32(RXC0 + 4, crate::pdma::RECEIVE_QUEUE_PORT);
            fabric.master_write32(RXC0 + 8, 4 * PACKET_BYTES);
            fabric.master_write32(PDMA + 4, 1 << 31);
            if enable_channel {
                enable(&mut fabric);
            }
            fabric
        }

        fn enable(fabric: &mut Fabric) {
            let control = (1 << 31) | (1 << 28) | (1 << 27) | (2 << 22) | (2 << 20);
            fabric.master_write32(RXC0 + 12, control);
        }

        /// A standard cell whose payload words are recognisable: header bytes 0xa0+cell, then
        /// bytes counting up from the cell number.
        fn cell(number: u8) -> Vec<u8> {
            let mut cell = vec![0xa0 + number; 5];
            cell.extend((0..48).map(|i| number.wrapping_mul(0x10).wrapping_add(i)));
            cell
        }

        fn run(fabric: &mut Fabric, cycles: u32) {
            for _ in 0..cycles {
                fabric.tick_once();
            }
        }

        fn print_flagged(fabric: &Fabric) -> bool {
            fabric.megamodule(0).event_flag_registers()[PRINT / 32] & (1 << (PRINT % 32)) != 0
        }

        fn clear_print(fabric: &mut Fabric) {
            fabric.write32(0, EVTCLR0, 1 << PRINT);
        }

        fn word(fabric: &mut Fabric, address: u32) -> u32 {
            fabric.master_read32(address).unwrap()
        }

        #[test]
        fn cells_reach_l1d_through_channel_eight_and_interrupt_at_half_and_full() {
            let mut fabric = receiver(true);
            for number in 0..4u8 {
                assert_eq!(fabric.utopia_master_poll_receive(SLAVE), Some(true));
                assert!(fabric.utopia_master_send_cell(SLAVE, &cell(number)));
                run(&mut fabric, 60);
                let at = L1D + u32::from(number) * PACKET_BYTES;
                // Three dummy bytes, Header 1, then the payload at byte 8 of the packet.
                assert_eq!(word(&mut fabric, at), u32::from(0xa0 + number) << 24);
                assert_eq!(
                    word(&mut fabric, at + 8),
                    u32::from_le_bytes(std::array::from_fn(|i| {
                        number.wrapping_mul(0x10).wrapping_add(i as u8)
                    }))
                );
                // The write pointer reads back through the proxy, and a cell that has been
                // moved leaves room for the next one.
                let next = (u32::from(number) + 1) % 4;
                assert_eq!(word(&mut fabric, RXC0), L1D + next * PACKET_BYTES);
                assert_eq!(fabric.utopia_master_poll_receive(SLAVE), Some(true));
                // Half way through (the second cell) and at the end (the fourth).
                let interrupted = number == 1 || number == 3;
                assert_eq!(print_flagged(&fabric), interrupted, "cell {number}");
                clear_print(&mut fabric);
            }
        }

        #[test]
        fn the_pdma_writes_with_user_privilege() {
            // SPRS612G section 5.2: UTOPIA transfers are always user mode, PrivID 0. L1DMPPA16
            // guards core 0's first 2 KB of L1D, where the buffer starts; every attribute keeps
            // AID0, the local bit and supervisor read, so the result can be read back.
            const L1DMPPA16: u32 = 0x0184_ae40;
            const L1DMPFAR: u32 = 0x0184_ac00;
            const L1DMPFSR: u32 = 0x0184_ac04;
            const AID0: u32 = 1 << 10;
            const LOCAL: u32 = 1 << 8;
            const SR: u32 = 1 << 5;
            const SW: u32 = 1 << 4;
            const UW: u32 = 1 << 1;
            for (attributes, lands) in [
                (AID0 | LOCAL | SR | SW, false),
                (AID0 | LOCAL | SR | UW, true),
            ] {
                let mut fabric = receiver(true);
                fabric.write32(0, L1DMPPA16, attributes);
                assert!(fabric.utopia_master_send_cell(SLAVE, &cell(1)));
                run(&mut fabric, 60);
                let first = word(&mut fabric, L1D);
                let status = fabric.read32(0, L1DMPFSR);
                if lands {
                    assert_eq!(first, 0xa1 << 24, "user write granted");
                    assert_eq!(status, 0, "no fault");
                } else {
                    assert_eq!(first, 0, "user write refused although supervisor may write");
                    assert_eq!(fabric.read32(0, L1DMPFAR), L1D);
                    assert_eq!(status, UW, "PrivID 0, user write");
                }
            }
        }

        #[test]
        fn a_disabled_channel_backs_two_cells_up_and_then_takes_them() {
            let mut fabric = receiver(false);
            assert!(fabric.utopia_master_send_cell(SLAVE, &cell(0)));
            assert!(fabric.utopia_master_send_cell(SLAVE, &cell(1)));
            assert_eq!(fabric.utopia_master_poll_receive(SLAVE), Some(false));
            assert!(!fabric.utopia_master_send_cell(SLAVE, &cell(2)));
            run(&mut fabric, 120);
            // Nothing moved: the cells are still in the queue, and the PDMA counted the event.
            assert_eq!(fabric.utopia_master_poll_receive(SLAVE), Some(false));
            assert_eq!(word(&mut fabric, L1D), 0);
            assert!(!print_flagged(&fabric));
            assert_eq!(word(&mut fabric, PDMA + 0x24) & 0xffff, 1);
            // Programming the channel releases the event the peripheral was holding, cell by cell.
            enable(&mut fabric);
            run(&mut fabric, 120);
            assert_eq!(word(&mut fabric, L1D) >> 24, 0xa0);
            assert_eq!(word(&mut fabric, L1D + PACKET_BYTES) >> 24, 0xa1);
            assert_eq!(fabric.utopia_master_poll_receive(SLAVE), Some(true));
            assert!(
                print_flagged(&fabric),
                "the second cell reaches the half-buffer mark"
            );
        }

        #[test]
        fn disabling_the_channel_stops_delivery_and_a_wrong_phy_is_not_heard() {
            let mut fabric = receiver(true);
            assert_eq!(fabric.utopia_master_poll_receive(SLAVE + 1), None);
            assert!(!fabric.utopia_master_send_cell(SLAVE + 1, &cell(5)));
            run(&mut fabric, 60);
            assert_eq!(word(&mut fabric, L1D), 0);
            assert_eq!(fabric.utopia_master_poll_receive(SLAVE), Some(true));
            // The ROM's last step: a TCR write with STRT clear.
            fabric.master_write32(RXC0 + 12, 0);
            assert!(fabric.utopia_master_send_cell(SLAVE, &cell(1)));
            run(&mut fabric, 60);
            assert_eq!(word(&mut fabric, L1D), 0);
            assert_eq!(word(&mut fabric, RXC0), L1D);
            assert_eq!(word(&mut fabric, RXC0 + 12), 0);
            // The cell is still waiting, and nothing is flagged.
            assert!(!print_flagged(&fabric));
            assert!(fabric.utopia_master_send_cell(SLAVE, &cell(2)));
            assert_eq!(fabric.utopia_master_poll_receive(SLAVE), Some(false));
        }

        const TXC0: u32 = PDMA + 0x100;
        const PXINT: usize = 31;
        /// Where the transmit cells wait: core 0's L1D, past the receive buffer.
        const SOURCE: u32 = L1D + 0x400;

        /// A C6472 with its transmitter enabled as slave 3 in 8-bit, multi-PHY mode, its transmit
        /// group set up as SPRUEG2E Example 1 does - PE = 1, block mode, 14-word bursts from an
        /// incrementing switch address into the constant UXQ port - and `cells` cells waiting at
        /// `SOURCE`, each laid out as the cell packet the queue holds.
        fn transmitter(cells: u8) -> Fabric {
            let mut fabric = Fabric::new(&crate::device::C6472);
            fabric.master_write32(UTOPIA, (u32::from(SLAVE) << 24) | (1 << 16) | (1 << 14));
            let group = (1 << 22) | (1 << 20) | (13 << 12) | (1 << 11) | (13 << 4);
            fabric.master_write32(PDMA + 0x60, group);
            fabric.master_write32(PDMA + 4, 1 << 31);
            for number in 0..cells {
                let at = SOURCE + u32::from(number) * PACKET_BYTES;
                for (index, word) in packet(&cell(number)).into_iter().enumerate() {
                    fabric.master_write32(at + 4 * index as u32, word);
                }
            }
            fabric
        }

        /// The cell packet for an 8-bit standard cell on a little-endian device: three dummy
        /// bytes, then the wire bytes (SPRUEG2E Table 6).
        fn packet(cell: &[u8]) -> Vec<u32> {
            let mut bytes = vec![0; 3];
            bytes.extend_from_slice(cell);
            bytes
                .chunks_exact(4)
                .map(|chunk| u32::from_le_bytes(chunk.try_into().unwrap()))
                .collect()
        }

        fn enable_transmit(fabric: &mut Fabric, cells: u32) {
            start_transmit(fabric, cells, 1 << 27);
        }

        /// Program transmit channel 0 for `cells` cells from `SOURCE` with 32-bit elements, and
        /// `interrupt` as its IE and SINT fields.
        fn start_transmit(fabric: &mut Fabric, cells: u32, interrupt: u32) {
            start_transmit_block(fabric, cells * PACKET_BYTES, 2, interrupt);
        }

        /// Program transmit channel 0 for a block of `bytes` bytes from `SOURCE` in switch
        /// elements of `ssiz` (SSIZ, Table 39) and 32-bit peripheral elements.
        fn start_transmit_block(fabric: &mut Fabric, bytes: u32, ssiz: u32, interrupt: u32) {
            fabric.master_write32(TXC0, SOURCE);
            fabric.master_write32(TXC0 + 4, crate::pdma::TRANSMIT_QUEUE_PORT);
            fabric.master_write32(TXC0 + 8, bytes);
            let control = (1 << 31) | interrupt | (2 << 22) | (ssiz << 20);
            fabric.master_write32(TXC0 + 12, control);
        }

        fn pxint_flagged(fabric: &Fabric) -> bool {
            pxint_flagged_in(fabric, 0)
        }

        fn pxint_flagged_in(fabric: &Fabric, core: usize) -> bool {
            fabric.megamodule(core).event_flag_registers()[PXINT / 32] & (1 << (PXINT % 32)) != 0
        }

        #[test]
        fn cells_leave_l1d_through_channel_zero_and_reach_the_master_exactly() {
            let mut fabric = transmitter(2);
            run(&mut fabric, 60);
            assert_eq!(fabric.utopia_master_poll_transmit(SLAVE), Some(false));
            assert_eq!(fabric.utopia_master_poll_transmit(SLAVE + 1), None);
            enable_transmit(&mut fabric, 2);
            run(&mut fabric, 60);
            // UXCLAV says a cell is ready, and the wire carries the cell without its dummies.
            assert_eq!(fabric.utopia_master_poll_transmit(SLAVE), Some(true));
            let first = fabric.utopia_master_take_cell(SLAVE).unwrap();
            assert_eq!(first.bytes(), cell(0).as_slice());
            let second = fabric.utopia_master_take_cell(SLAVE).unwrap();
            assert_eq!(second.bytes(), cell(1).as_slice());
            assert_eq!(fabric.utopia_master_poll_transmit(SLAVE), Some(false));
            // The block is done: PXINT in the core SINT names (Table 5), STRT clear and BCZ set,
            // and the switch address past both cells (Table 39).
            assert!(pxint_flagged(&fabric));
            assert!(!print_flagged(&fabric));
            assert_eq!(word(&mut fabric, TXC0), SOURCE + 2 * PACKET_BYTES);
            assert_eq!(word(&mut fabric, TXC0 + 8), 0);
            assert_eq!(
                word(&mut fabric, TXC0 + 12) & ((1 << 31) | (1 << 16)),
                1 << 16
            );
            // Nothing more leaves once the block has completed.
            run(&mut fabric, 60);
            assert_eq!(fabric.utopia_master_poll_transmit(SLAVE), Some(false));
        }

        #[test]
        fn a_full_transmit_queue_holds_the_block_back_until_the_master_drains_it() {
            let mut fabric = transmitter(3);
            // Before the channel exists, UXEVT is an error, counted once however long it waits.
            run(&mut fabric, 120);
            assert_eq!(word(&mut fabric, PDMA + 0x20), 1);
            enable_transmit(&mut fabric, 3);
            run(&mut fabric, 120);
            // The queue holds two cells (SPRUEG2E section 2), so the third stays in memory and
            // the block is not done.
            assert_eq!(word(&mut fabric, TXC0), SOURCE + 2 * PACKET_BYTES);
            assert_eq!(word(&mut fabric, TXC0 + 8), PACKET_BYTES);
            assert!(!pxint_flagged(&fabric));
            assert_eq!(
                fabric.utopia_master_take_cell(SLAVE).unwrap().bytes(),
                cell(0).as_slice()
            );
            run(&mut fabric, 60);
            assert!(pxint_flagged(&fabric));
            for number in 1..3 {
                assert_eq!(
                    fabric.utopia_master_take_cell(SLAVE).unwrap().bytes(),
                    cell(number).as_slice()
                );
            }
            // Waiting for the master was never a stall: UXEVT only asks when there is room.
            assert_eq!(word(&mut fabric, PDMA + 0x20), 1);
        }

        #[test]
        fn the_pdma_reads_its_source_with_user_privilege() {
            // SPRS612G section 5.2 again, on the read side: L1DMPPA16 guards the first 2 KB of
            // core 0's L1D, where the cells wait. A page without UR refuses the PDMA's reads -
            // the words go out as zeros and the fault is a user read - and one with UR passes.
            const L1DMPPA16: u32 = 0x0184_ae40;
            const L1DMPFAR: u32 = 0x0184_ac00;
            const L1DMPFSR: u32 = 0x0184_ac04;
            const AID0: u32 = 1 << 10;
            const LOCAL: u32 = 1 << 8;
            const SR: u32 = 1 << 5;
            const SW: u32 = 1 << 4;
            const UR: u32 = 1 << 2;
            for (attributes, granted) in [
                (AID0 | LOCAL | SR | SW, false),
                (AID0 | LOCAL | SR | UR, true),
            ] {
                let mut fabric = transmitter(1);
                fabric.write32(0, L1DMPPA16, attributes);
                enable_transmit(&mut fabric, 1);
                run(&mut fabric, 60);
                let sent = fabric.utopia_master_take_cell(SLAVE).unwrap();
                let status = fabric.read32(0, L1DMPFSR);
                if granted {
                    assert_eq!(sent.bytes(), cell(0).as_slice(), "user read granted");
                    assert_eq!(status, 0, "no fault");
                } else {
                    assert_eq!(
                        sent.bytes(),
                        [0; 53],
                        "user read refused although supervisor may read"
                    );
                    assert_eq!(fabric.read32(0, L1DMPFAR), SOURCE);
                    assert_eq!(status, UR, "PrivID 0, user read");
                }
            }
        }

        const EDMA: u32 = 0x02a0_0000;
        const EDMA_ER: u32 = EDMA + 0x1000;
        const EDMA_ERH: u32 = EDMA + 0x1004;
        const EDMA_EESR: u32 = EDMA + 0x1030;
        const EDMA_PARAM: u32 = EDMA + 0x4000;
        /// The words each armed channel copies, in core 0's L1D past the transmit cells.
        const EDMA_SOURCE: u32 = L1D + 0x800;
        const EDMA_DATA: [u32; 3] = [0xd0d0_0001, 0xd0d0_0002, 0xd0d0_0003];

        /// Where channel `channel` of PXINT0-5 (EDMA3 channels 22-27) writes.
        fn edma_destination(channel: u32) -> u32 {
            L1D + 0x900 + 0x10 * (channel - 22)
        }

        fn load_edma_source(fabric: &mut Fabric) {
            for (index, word) in EDMA_DATA.into_iter().enumerate() {
                fabric.master_write32(EDMA_SOURCE + 4 * index as u32, word);
            }
        }

        /// Map `channel` to the PaRAM set of the same number - a C6472 resets every DMA channel
        /// to set 0 (SPRU727E section 4.2.1.3) - and enable it for its synchronisation event
        /// with an A-synchronised set that moves one word of `EDMA_DATA` per event: `arrays`
        /// events complete it.
        fn arm_edma(fabric: &mut Fabric, channel: u32, arrays: u32) {
            fabric.master_write32(EDMA + 0x100 + 4 * channel, channel << 5);
            let set = [
                0,
                EDMA_SOURCE,
                (arrays << 16) | 4,
                edma_destination(channel),
                (4 << 16) | 4,
                0xffff,
                0,
                1,
            ];
            for (index, word) in set.into_iter().enumerate() {
                fabric.master_write32(EDMA_PARAM + 32 * channel + 4 * index as u32, word);
            }
            fabric.master_write32(EDMA_EESR, 1 << channel);
        }

        /// What the six PXINT destinations hold, in channel order.
        fn edma_results(fabric: &mut Fabric) -> Vec<u32> {
            (22..28)
                .map(|channel| word(fabric, edma_destination(channel)))
                .collect()
        }

        #[test]
        fn a_completed_transmit_block_drives_only_the_edma_channel_its_sint_selects() {
            // SINT n asserts PXINTn, EDMA3 channel 22 + n, and is event 31 in core n; SINT 6 and
            // 7 select no connected output. Every one of the six channels is armed, so a
            // transfer on the wrong channel shows up as a word where there should be none.
            for sint in 0..8u32 {
                let mut fabric = transmitter(1);
                load_edma_source(&mut fabric);
                for channel in 22..28 {
                    arm_edma(&mut fabric, channel, 1);
                }
                start_transmit(&mut fabric, 1, (1 << 27) | (sint << 24));
                run(&mut fabric, 400);
                let mut expected = [0; 6];
                if let Some(slot) = expected.get_mut(sint as usize) {
                    *slot = EDMA_DATA[0];
                }
                assert_eq!(edma_results(&mut fabric), expected, "SINT {sint}");
                // The CPU interrupt is delivered exactly as before, to the core SINT names.
                for core in 0..6 {
                    assert_eq!(
                        pxint_flagged_in(&fabric, core),
                        core == sint as usize,
                        "SINT {sint}, core {core}"
                    );
                }
                // The serviced event is not left latched.
                assert_eq!(word(&mut fabric, EDMA_ER), 0, "SINT {sint}");
            }
        }

        #[test]
        fn a_transmit_block_without_interrupt_enable_raises_no_edma_event() {
            let mut fabric = transmitter(1);
            load_edma_source(&mut fabric);
            arm_edma(&mut fabric, 22, 1);
            start_transmit(&mut fabric, 1, 0);
            run(&mut fabric, 400);
            // The block completed - the cell is on the wire and BCZ is set - so nothing but IE
            // kept the output quiet.
            assert_eq!(
                fabric.utopia_master_take_cell(SLAVE).unwrap().bytes(),
                cell(0).as_slice()
            );
            // Reading SAR snapshots the channel's context into the proxies (section 5); the
            // proxy TCR read after it then shows what the state machine set.
            assert_eq!(word(&mut fabric, TXC0), SOURCE + PACKET_BYTES);
            assert_eq!(word(&mut fabric, TXC0 + 12) & (1 << 16), 1 << 16);
            assert!(!pxint_flagged(&fabric));
            assert_eq!(word(&mut fabric, EDMA_ER), 0);
            assert_eq!(edma_results(&mut fabric), [0; 6]);
        }

        #[test]
        fn each_transmit_completion_runs_the_next_array_of_the_edma_transfer() {
            let mut fabric = transmitter(1);
            load_edma_source(&mut fabric);
            arm_edma(&mut fabric, 22, 3);
            for completed in 0..3 {
                start_transmit(&mut fabric, 1, 1 << 27);
                run(&mut fabric, 400);
                assert_eq!(
                    fabric.utopia_master_take_cell(SLAVE).unwrap().bytes(),
                    cell(0).as_slice()
                );
                // The CPU saw this completion too: clear the flag so the next one is visible.
                assert!(pxint_flagged(&fabric), "completion {completed}");
                fabric.write32(0, EVTCLR0, 1 << PXINT);
                // One array per event, so the words arrive one completion at a time.
                for (index, expected) in EDMA_DATA.into_iter().enumerate() {
                    let expected = if index <= completed { expected } else { 0 };
                    assert_eq!(
                        word(&mut fabric, edma_destination(22) + 4 * index as u32),
                        expected,
                        "completion {completed}, word {index}"
                    );
                }
            }
        }

        #[test]
        fn receive_completions_raise_no_edma_event() {
            // PRINT has no synchronisation channel in Table 7-4.
            let mut fabric = receiver(true);
            for number in 0..4u8 {
                assert!(fabric.utopia_master_send_cell(SLAVE, &cell(number)));
                run(&mut fabric, 60);
            }
            assert!(print_flagged(&fabric));
            assert_eq!(word(&mut fabric, EDMA_ER), 0);
            assert_eq!(word(&mut fabric, EDMA_ERH), 0);
        }

        /// Channel 8 in block mode for `bytes` bytes at `L1D` in switch elements of `ssiz`, with
        /// the interrupt enabled.
        fn block_receiver(ssiz: u32, bytes: u32) -> Fabric {
            let mut fabric = Fabric::new(&crate::device::C6472);
            fabric.master_write32(UTOPIA, (u32::from(SLAVE) << 24) | (1 << 14) | 1);
            let group = (2 << 22) | (1 << 20) | (1 << 16) | (13 << 12) | (1 << 11) | (13 << 4);
            fabric.master_write32(PDMA + 0x64, group);
            fabric.master_write32(RXC0, L1D);
            fabric.master_write32(RXC0 + 4, crate::pdma::RECEIVE_QUEUE_PORT);
            fabric.master_write32(RXC0 + 8, bytes);
            fabric.master_write32(PDMA + 4, 1 << 31);
            fabric.master_write32(RXC0 + 12, (1 << 31) | (1 << 27) | (2 << 22) | (ssiz << 20));
            fabric
        }

        /// `count` bytes of memory at `address` in storage order, whatever the device's byte
        /// order: a word read gives the bytes in that order, so this undoes it.
        fn memory_bytes(fabric: &mut Fabric, address: u32, count: usize, big: bool) -> Vec<u8> {
            (0..count.div_ceil(4) as u32)
                .flat_map(|index| {
                    let value = word(fabric, address + 4 * index);
                    if big {
                        value.to_be_bytes()
                    } else {
                        value.to_le_bytes()
                    }
                })
                .take(count)
                .collect()
        }

        /// Put `bytes` in memory at `address` in storage order.
        fn store_bytes(fabric: &mut Fabric, address: u32, bytes: &[u8], big: bool) {
            for (index, chunk) in bytes.chunks(4).enumerate() {
                let mut padded = [0; 4];
                padded[..chunk.len()].copy_from_slice(chunk);
                let value = if big {
                    u32::from_be_bytes(padded)
                } else {
                    u32::from_le_bytes(padded)
                };
                fabric.master_write32(address + 4 * index as u32, value);
            }
        }

        /// An element of `size` bytes holding `value`, in storage order.
        fn element(value: u32, size: usize, big: bool) -> Vec<u8> {
            if big {
                value.to_be_bytes()[4 - size..].to_vec()
            } else {
                value.to_le_bytes()[..size].to_vec()
            }
        }

        #[test]
        fn a_block_receive_into_narrow_elements_keeps_the_low_bits_of_each_word() {
            // SPRUEG2E Table 32: data is right justified, so each 32-bit word of the cell
            // packet becomes one byte or halfword, in the device's byte order.
            let words = packet(&cell(1));
            for (big, ssiz, size) in [(false, 0, 1), (false, 1, 2), (true, 0, 1), (true, 1, 2)] {
                let mut fabric = block_receiver(ssiz, 14 * size as u32);
                if big {
                    fabric.set_endian(Endian::Big);
                }
                assert!(fabric.utopia_master_send_cell(SLAVE, &cell(1)));
                run(&mut fabric, 60);
                let expected: Vec<u8> = words[..14]
                    .iter()
                    .flat_map(|&value| element(value, size, big))
                    .collect();
                let landed = memory_bytes(&mut fabric, L1D, expected.len() + 4, big);
                assert_eq!(landed[..expected.len()], expected, "big {big}, size {size}");
                assert_eq!(landed[expected.len()..], [0; 4], "nothing past the block");
                // The block is spent: the address moved by the element size, and the
                // completion interrupt arrived.
                assert_eq!(word(&mut fabric, RXC0), L1D + 14 * size as u32);
                assert_eq!(word(&mut fabric, RXC0 + 8), 0);
                assert!(print_flagged(&fabric));
            }
        }

        #[test]
        fn a_block_transmit_from_narrow_elements_widens_each_to_a_word() {
            // The same right justification on the way out: a byte or halfword of memory is the
            // low part of the 32-bit word the queue receives, the rest zero.
            for (big, ssiz, size) in [(false, 0, 1), (false, 1, 2), (true, 0, 1), (true, 1, 2)] {
                let mut fabric = transmitter(0);
                if big {
                    fabric.set_endian(Endian::Big);
                }
                let values: Vec<u32> = (1..=14)
                    .map(|k| k * if size == 1 { 0x11 } else { 0x1111 })
                    .collect();
                let bytes: Vec<u8> = values
                    .iter()
                    .flat_map(|&value| element(value, size, big))
                    .collect();
                store_bytes(&mut fabric, SOURCE, &bytes, big);
                start_transmit_block(&mut fabric, 14 * size as u32, ssiz, 1 << 27);
                run(&mut fabric, 60);
                // Fourteen words, one per element, are the packet whose wire cell skips the
                // three leading dummy bytes.
                let wire: Vec<u8> = values
                    .iter()
                    .flat_map(|value| value.to_le_bytes())
                    .skip(3)
                    .take(53)
                    .collect();
                assert_eq!(
                    fabric.utopia_master_take_cell(SLAVE).unwrap().bytes(),
                    wire.as_slice(),
                    "big {big}, size {size}"
                );
                assert!(pxint_flagged(&fabric));
            }
        }
    }

    /// An EDMA3 transfer is made for the core that programmed it: that core's PrivID and
    /// privilege mode, captured when it wrote the PaRAM set (SPRU966C section 2.10.2, SPRS612G
    /// Table 5-3), and judged by the memory protection of the page it touches.
    mod edma_privilege {
        use super::*;

        const PARAM: u32 = 0x02a0_4000;
        const ESR: u32 = 0x02a0_1010;
        const L2MPFAR: u32 = 0x0184_a000;
        const L2MPFSR: u32 = 0x0184_a004;
        const L2MPPA0: u32 = 0x0184_a200;
        const AIDX: u32 = 1 << 9;
        const LOCAL: u32 = 1 << 8;
        const SR: u32 = 1 << 5;
        const SW: u32 = 1 << 4;
        const UR: u32 = 1 << 2;
        const UW: u32 = 1 << 1;
        const SOURCE: u32 = 0x0080_0100;
        const DESTINATION: u32 = 0x0080_0200;
        const DATA: [u8; 4] = [0xde, 0xad, 0xbe, 0xef];

        fn aid(id: u32) -> u32 {
            1 << (10 + id)
        }

        /// The address a DMA master uses for `local` in `core`'s memory: the C6472 gives each
        /// core's L2 a global alias, the C6455 has none.
        fn dma_address(fabric: &Fabric, core: usize, local: u32) -> u32 {
            if fabric.profile.map.has_global_aliases() {
                local + 0x1000_0000 + 0x0100_0000 * core as u32
            } else {
                local
            }
        }

        /// One CPU store as the core issues it: the access check precedes it, and carries the
        /// core's privilege to the EDMA3.
        fn store(fabric: &mut Fabric, core: usize, user: bool, address: u32, value: u32) {
            assert!(fabric.check_access(core, address, Permission::Write, user));
            fabric.write32(core, address, value);
        }

        /// `core` programs channel 0 to copy `DATA` across a protected page, in `user` or
        /// supervisor mode, and raises its event; the CPU then runs on in `then_user` mode before
        /// the controller services the request.
        fn transfer(
            profile: &'static DeviceProfile,
            core: usize,
            user: bool,
            attributes: u32,
            then_user: bool,
        ) -> (Vec<u8>, u32, u32) {
            let mut fabric = Fabric::new(profile);
            fabric.load_bytes(core, SOURCE, &DATA);
            fabric.write32(core, L2MPPA0, attributes);
            let source = dma_address(&fabric, core, SOURCE);
            let destination = dma_address(&fabric, core, DESTINATION);
            let set = [0, source, (1 << 16) | 4, destination, 0, 0xffff, 0, 1];
            for (index, word) in set.into_iter().enumerate() {
                store(&mut fabric, core, user, PARAM + 4 * index as u32, word);
            }
            store(&mut fabric, core, user, ESR, 1);
            // The CPU changes privilege - an exception, or a return from one - while the request
            // waits; a lookup at execution time would see this and not the submitter.
            assert!(fabric.check_access(core, SOURCE, Permission::Read, then_user));
            for _ in 0..16 {
                fabric.tick_once();
            }
            (
                fabric.read_bytes(core, DESTINATION, 4),
                fabric.read32(core, L2MPFAR),
                fabric.read32(core, L2MPFSR),
            )
        }

        #[test]
        fn a_user_mode_submitter_is_refused_by_a_page_granting_only_supervisor_writes() {
            let attributes = aid(0) | LOCAL | SR | SW | UR;
            let (destination, address, status) =
                transfer(&crate::device::C6455, 0, true, attributes, true);
            assert_eq!(destination, [0; 4], "the user write is refused");
            assert_eq!(address, DESTINATION);
            assert_eq!(status, UW, "PrivID 0, user write");
        }

        #[test]
        fn a_supervisor_mode_submitter_completes_the_same_transfer() {
            let attributes = aid(0) | LOCAL | SR | SW | UR;
            let (destination, _, status) =
                transfer(&crate::device::C6455, 0, false, attributes, false);
            assert_eq!(destination, DATA);
            assert_eq!(status, 0);
        }

        #[test]
        fn a_c6472_cpu_submitter_carries_its_core_id_not_aidx() {
            // A page granting AIDX alone: core 3 is AID3, so its transfer is refused - at the
            // first read, as a supervisor with every permission the page can give.
            let attributes = AIDX | LOCAL | SR | SW | UR | UW;
            let (destination, _, status) =
                transfer(&crate::device::C6472, 3, false, attributes, false);
            assert_eq!(destination, [0; 4]);
            assert_eq!(status, (3 << 9) | SR, "PrivID 3, supervisor read");
            // Granting that core's own AID lets the same transfer through.
            let attributes = aid(3) | LOCAL | SR | SW | UR | UW;
            let (destination, _, status) =
                transfer(&crate::device::C6472, 3, false, attributes, false);
            assert_eq!(destination, DATA);
            assert_eq!(status, 0);
            // And the fault names the submitter whose privilege failed.
            let attributes = aid(3) | LOCAL | SR | SW | UR;
            let (destination, address, status) =
                transfer(&crate::device::C6472, 3, true, attributes, true);
            assert_eq!(destination, [0; 4]);
            assert_eq!(
                address,
                dma_address(&Fabric::new(&crate::device::C6472), 3, DESTINATION)
            );
            assert_eq!(status, (3 << 9) | UW, "PrivID 3, user write");
        }

        #[test]
        fn the_cpu_changing_privilege_before_the_transfer_runs_does_not_change_its_judge() {
            let attributes = aid(0) | LOCAL | SR | SW | UR;
            // Submitted in user mode, serviced after the CPU is back in supervisor mode: still a
            // user write, still refused.
            let (destination, _, status) =
                transfer(&crate::device::C6455, 0, true, attributes, false);
            assert_eq!(destination, [0; 4]);
            assert_eq!(status, UW);
            // Submitted in supervisor mode, serviced while the CPU is in user mode: still a
            // supervisor write, still granted.
            let (destination, _, status) =
                transfer(&crate::device::C6455, 0, false, attributes, true);
            assert_eq!(destination, DATA);
            assert_eq!(status, 0);
        }

        #[test]
        fn a_dma_master_writing_a_set_stamps_its_own_identity_not_the_last_cpu_access() {
            let mut fabric = Fabric::new(&crate::device::C6472);
            // Core 3 in user mode is the last thing to have touched the bus.
            store(&mut fabric, 3, true, ESR, 0);
            fabric.master_write32(PARAM + 4, 0x1234);
            // PRIV (bit 31) says supervisor and PRIVID (bits 27-24) says 0: the HPI's identity.
            assert_eq!(fabric.read32(3, PARAM) >> 24, 0x80);
            // A CPU write stamps its own.
            store(&mut fabric, 3, true, PARAM + 4, 0x1234);
            assert_eq!(fabric.read32(3, PARAM) >> 24, 0x03);
        }
    }
}
