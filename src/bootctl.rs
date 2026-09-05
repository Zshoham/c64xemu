//! The C6472 boot controller (SPRS612G section 3.9, memory map Table 2-2: 32K at 02AB 0000).
//!
//! The boot controller is the block that decides, after a device-level reset, which of the six
//! C64x+ megamodule cores start fetching and from where. It owns four things:
//!
//! * RESET_STAT - why each core was last locally reset, and whether a global reset happened.
//! * BOOT_COMPLETE_STAT - the six set-only BCn bits. A 0 to 1 transition on BCn takes core n out
//!   of the internally "held in reset" state (SPRS612G sections 2.4.1 and 2.4.2).
//! * BOOTPROGRESS - a plain 32-bit read/write scratch word. SPRS612G section 3.9.3 gives it no
//!   fields at all: "The ROM boot code periodically writes values to this register to indicate
//!   progress. This can also be used by other software as a debugging tool." So it is modelled as
//!   free-form software state with no hardware side effect.
//! * BOOTMODEn and DSP_BOOT_ADDRn, one pair per core, holding the local boot option and the 22
//!   most significant bits of the core's initial fetch address.
//!
//! This module is a passive register file. It never resets a core and never raises an event; the
//! device model asks it for state (`held_in_reset`, `boot_active`, `boot_address`) and consumes
//! the release edge exactly once with `take_released_cores`. The boot controller drives no entry
//! in the SPRS612G Table 7-10 event map, so there is no interrupt accessor here, and it has no
//! time-dependent state, so there is no `tick`.
//!
//! Not modelled here: the ROM bootloader itself (SPRUEC6), the device configuration registers at
//! 02A8 0000, and the power/sleep controller.
use crate::port::{PortContext, PortWrite, SlavePort};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

/// SPRS612G Table 2-2: the boot controller occupies 02AB 0000 - 02AB 7FFF.
const BASE: u32 = 0x02ab_0000;
const WINDOW: u32 = 0x8000;

const RESET_STAT: u32 = BASE;
const BOOT_COMPLETE_STAT: u32 = BASE + 0x004;
const BOOTPROGRESS: u32 = BASE + 0x008;
/// BOOTMODEn is at 02AB 0200 + 20h*n and DSP_BOOT_ADDRn four bytes above it (SPRS612G Table 3-14).
const CORE_BASE: u32 = BASE + 0x200;
const CORE_STRIDE: u32 = 0x20;

/// The register has six LRn fields plus GR; every one of those bits is write-one-to-clear
/// (SPRS612G Figure 3-15, Table 3-20).
const RESET_STAT_GR: u32 = 1 << 31;
const RESET_STAT_LR_MASK: u32 = 0x0fff;
const RESET_STAT_CLEARABLE: u32 = RESET_STAT_GR | RESET_STAT_LR_MASK;

/// LRn encodings (SPRS612G Table 3-20).
#[cfg(test)]
const LR_NONE: u32 = 0b00;
#[cfg(test)]
const LR_IN: u32 = 0b01;
#[cfg(test)]
const LR_OUT: u32 = 0b11;

/// UNCERTAIN: SPRS612G Figure 3-15 annotates every LRn field "RC-10", i.e. reset value 10b, while
/// Table 3-20 lists 10b as Reserved for the same field and gives 00b the meaning "core n has not
/// received a local reset". The two statements cannot both be right. The figure is the normative
/// source for reset values, so 10b is used here, and a device model that wants the field to read
/// as a meaningful encoding out of reset must clear it (write ones) first. Nothing in this module
/// treats 10b as anything but an opaque power-up pattern.
const LR_RESET: u32 = 0b10;
const RESET_STAT_RESET: u32 = (LR_RESET << 10)
    | (LR_RESET << 8)
    | (LR_RESET << 6)
    | (LR_RESET << 4)
    | (LR_RESET << 2)
    | LR_RESET;

/// BOOTMODEn: 0 immediate (reset default), 1 host, 2-15 reserved (SPRS612G Table 3-22).
const BOOTMODE_MASK: u32 = 0xf;
const BOOTMODE_IMMEDIATE: u32 = 0;
#[cfg(test)]
const BOOTMODE_HOST: u32 = 1;

/// DSP_BOOT_ADDRn holds the 22 MSBs of the fetch address (SPRS612G section 3.9.5, Figure 3-19).
const BOOT_ADDR_MASK: u32 = 0x003f_ffff;
/// 32 - 22: the field is shifted left by ten to form the address. The documented defaults confirm
/// the shift - 00 2000h gives 0080 0000h (L2 RAM base) and 00 0400h gives 0010 0000h (shared L2
/// ROM base), exactly the two values SPRS612G Table 2-4 lists as default start addresses.
const BOOT_ADDR_SHIFT: u32 = 10;
/// L2 RAM base tie-off, the default for boot modes 0-1 on every core and for cores 1-5 always.
const BOOT_ADDR_L2RAM: u32 = 0x0000_2000;
/// Shared L2 ROM base tie-off, the default of DSP_BOOT_ADDR0 in boot modes 2-15.
const BOOT_ADDR_ROM: u32 = 0x0000_0400;

/// The device supports six megamodules; BOOT_COMPLETE_STAT has exactly six BC bits.
const MAX_CORES: usize = 6;

/// The C6472 boot controller register file.
///
/// `BootController::new` needs the strapped BOOTMODE[3:0] value because both the initial reset
/// state of the cores and the DSP_BOOT_ADDRn defaults depend on it (SPRS612G Table 2-4,
/// section 3.9.5).
pub(crate) struct BootController {
    /// Number of megamodules the device model instantiates, at most six.
    cores: usize,
    /// The BOOTMODE[3:0] pin value latched at device reset (SPRS612G Table 2-3).
    /// RESET_STAT as read: GR in bit 31, LR0-LR5 in bits 11:0.
    reset_stat: u32,
    /// BOOT_COMPLETE_STAT bits 5:0.
    boot_complete: u32,
    /// BOOTPROGRESS, free-form software scratch (SPRS612G section 3.9.3).
    boot_progress: u32,
    /// BOOTMODE0-5, four bits each.
    local_boot_mode: [u32; MAX_CORES],
    /// DSP_BOOT_ADDR0-5, 22 bits each.
    boot_addr: [u32; MAX_CORES],
    /// Cores whose BCn bit went 0 to 1 since `take_released_cores` was last called.
    released: u32,
    /// Cores the controller is currently holding in reset.
    held: [bool; MAX_CORES],
}

impl BootController {
    /// Build the block in its device-reset state for the given strapped boot mode.
    ///
    /// Reset state of the cores, from SPRS612G section 2.4.1 ("Boot Modes Supported") and
    /// section 2.4.2 ("BOOTACTIVE"):
    ///
    /// * mode 0, immediate boot: "all cores are released from reset immediately".
    /// * mode 1, host boot: "all C64x+ megamodule cores are internally 'held in reset' while the
    ///   remainder of the device ... is released from reset". Section 2.4.2 repeats it: the host
    ///   "needs to write to the boot complete bit ... corresponding to each C64x+ megamodule that
    ///   is to be taken out of reset". Core 0 is held too - host boot has no ROM code to run.
    /// * modes 2-15, the ROM boot modes: "core 0 comes out of RESET immediately, but all other
    ///   cores are still in RESET", and core 0's ROM code releases cores 1-5 by writing BC1-BC5.
    ///
    /// DSP_BOOT_ADDRn defaults, from SPRS612G section 3.9.5 and Table 2-4: boot modes 0-1 give
    /// every core 00 2000h (0080 0000h), boot modes 2-15 give core 0 00 0400h (0010 0000h, the
    /// shared L2 ROM) and cores 1-5 00 2000h.
    ///
    /// BOOTMODE0-5 always reset to 0, immediate boot, whatever the pins say (SPRS612G section
    /// 3.9.4); they select what happens on a *local* reset, not on this device reset.
    pub(crate) fn new(bootmode: u8, cores: usize) -> Self {
        let device_boot_mode = bootmode & 0xf;
        let cores = cores.min(MAX_CORES);
        let rom_mode = device_boot_mode >= 2;

        let mut boot_addr = [BOOT_ADDR_L2RAM; MAX_CORES];
        if rom_mode {
            boot_addr[0] = BOOT_ADDR_ROM;
        }

        let mut held = [false; MAX_CORES];
        for (core, hold) in held.iter_mut().enumerate().take(cores) {
            *hold = match device_boot_mode {
                0 => false,
                1 => true,
                _ => core != 0,
            };
        }

        Self {
            cores,
            reset_stat: RESET_STAT_RESET,
            boot_complete: 0,
            boot_progress: 0,
            local_boot_mode: [BOOTMODE_IMMEDIATE; MAX_CORES],
            boot_addr,
            released: 0,
            held,
        }
    }
}

impl SlavePort for BootController {
    /// None when the address is not ours.
    ///
    /// Every 32-bit word of the 32K window belongs to the boot controller; the words SPRS612G
    /// Table 3-14 marks Reserved read as zero rather than reporting "not mine", so a router that
    /// hands the whole window over does not turn a reserved access into a bus fault. Registers
    /// answer only at their own word.
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        if !Self::in_window(address) {
            return None;
        }
        let word = address & !3;
        let value = match word {
            RESET_STAT => self.reset_stat,
            BOOT_COMPLETE_STAT => self.boot_complete,
            BOOTPROGRESS => self.boot_progress,
            _ => match Self::core_register(word) {
                Some((core, 0)) if core < self.cores => self.local_boot_mode[core],
                Some((core, 4)) if core < self.cores => self.boot_addr[core],
                _ => 0,
            },
        };
        Some(value)
    }

    /// `PortWrite::Unhandled` when the address is not ours.
    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        if !Self::in_window(address) {
            return PortWrite::Unhandled;
        }
        let word = address & !3;
        match word {
            // GR and every LRn bit are write-one-to-clear; writing a zero has no effect
            // (SPRS612G Table 3-20).
            RESET_STAT => self.reset_stat &= !(value & RESET_STAT_CLEARABLE),
            BOOT_COMPLETE_STAT => self.set_boot_complete(value),
            BOOTPROGRESS => self.boot_progress = value,
            _ => match Self::core_register(word) {
                Some((core, 0)) if core < self.cores => {
                    self.local_boot_mode[core] = value & BOOTMODE_MASK;
                }
                Some((core, 4)) if core < self.cores => {
                    self.boot_addr[core] = value & BOOT_ADDR_MASK;
                }
                // Reserved words absorb the write.
                _ => {}
            },
        }
        PortWrite::Done
    }
}

impl BootController {
    /// Cores whose BCn bit went 0 to 1 since this was last called, as a bitmask.
    ///
    /// Consumed by the device model exactly once per edge: for a core that was held in reset this
    /// is the instant it must start fetching from `boot_address(core)` (SPRS612G section 2.4.1).
    /// The edge is also reported for a core that was already running - "any active core can set
    /// bits in BOOT_COMPLETE_STAT at any time" (section 2.4.2) - and the device model is expected
    /// to ignore an edge for a core whose `held_in_reset` was already false rather than restart
    /// it.
    pub(crate) fn take_released_cores(&mut self) -> u32 {
        core::mem::take(&mut self.released)
    }

    /// True while the boot controller is holding this core in reset (SPRS612G sections 2.4.1,
    /// 2.4.2). Out of range cores read false.
    pub(crate) fn held_in_reset(&self, core: usize) -> bool {
        core < self.cores && self.held[core]
    }

    /// The BOOTACTIVE output pin: "asserted upon reset and de-asserted on boot complete ...
    /// BOOTACTIVE will be high if any cores are held in reset" (SPRS612G section 2.4.2). In boot
    /// mode 0 nothing is ever held, so it reads low immediately, which matches the same section:
    /// "all cores are released from reset immediately. BOOTACTIVE also goes low".
    #[cfg(test)]
    pub(crate) fn boot_active(&self) -> bool {
        self.held[..self.cores].iter().any(|held| *held)
    }

    /// The core's initial fetch address: DSP_BOOT_ADDRn is the 22 MSBs of it, so the register
    /// value is shifted left by ten (SPRS612G section 3.9.5, Table 2-4). Out of range cores read
    /// zero.
    pub(crate) fn boot_address(&self, core: usize) -> u32 {
        if core >= self.cores {
            return 0;
        }
        self.boot_addr[core] << BOOT_ADDR_SHIFT
    }

    /// Record in RESET_STAT that core n saw a local reset.
    ///
    /// `from_pin` selects between the two documented LRn encodings (SPRS612G Table 3-20):
    /// `true` records 01b, "a local reset (lreset_in) has been asserted to core n", and `false`
    /// records 11b, "core n has responded with lreset_out". The reset controller normally calls
    /// this twice for one reset - once with `true` when it drives lreset_in, once with `false`
    /// when the megamodule acknowledges - and software clears the field by writing ones.
    ///
    /// UNCERTAIN: SPRS612G names three sources of a local reset in section 2.4.1 (the external
    /// LRESET pin, the on-chip reset controller, and the power/sleep controller) but never says
    /// whether an on-chip source shows up as lreset_in in this field or only as lreset_out. This
    /// model treats every asserted local reset, whatever its source, as lreset_in, and reserves
    /// 11b for the megamodule's acknowledge.
    #[cfg(test)]
    pub(crate) fn note_local_reset(&mut self, core: usize, from_pin: bool) {
        if core >= self.cores {
            return;
        }
        let field = if from_pin { LR_IN } else { LR_OUT };
        let shift = 2 * core as u32;
        self.reset_stat = (self.reset_stat & !(0b11 << shift)) | (field << shift);
    }

    /// The LRn field of RESET_STAT for one core: 00b none, 01b lreset_in asserted, 11b lreset_out
    /// responded, 10b the power-up pattern (SPRS612G Table 3-20, Figure 3-15).
    #[cfg(test)]
    pub(crate) fn local_reset_state(&self, core: usize) -> u32 {
        if core >= self.cores {
            return LR_NONE;
        }
        (self.reset_stat >> (2 * core as u32)) & 0b11
    }

    /// Record in RESET_STAT that a global (device-level) reset happened: GR, bit 31.
    ///
    /// UNCERTAIN: SPRS612G Table 3-20 documents GR only as "Global Reset" with a write-one-to-
    /// clear rule, and Figure 3-15 gives it reset value 0, so the device reset that this bit
    /// reports cannot be the same reset that clears it. The manual never names which of the three
    /// device-level resets of section 2.4.1 - power-on, warm/RESET pin, system reset - sets it.
    /// The bit is therefore left entirely to the device model: nothing in this module sets it.
    #[cfg(test)]
    pub(crate) fn note_global_reset(&mut self) {
        self.reset_stat |= RESET_STAT_GR;
    }

    /// True when a *local* reset of this core will leave it held in reset, i.e. its BOOTMODEn
    /// register selects host boot: "the C64x+ megamodule core (x) is internally 'held in reset'
    /// while the remainder of the C64x+ megamodule is released from reset upon being given a
    /// local reset" (SPRS612G section 2.4.1, local boot modes). With BOOTMODEn = 0, immediate
    /// boot, the core restarts straight away from `boot_address(core)`.
    ///
    /// The device model consults this when it locally resets a core; this module does not hold a
    /// core on its own, because it is not told about local resets until after the fact.
    ///
    /// UNCERTAIN: BCn is set-only (RS in SPRS612G Figure 3-16) and SPRS612G documents no way to
    /// clear it, so after the first boot completes there is no register state left that a second,
    /// local host boot could gate on. A device model that re-holds a core after a local reset has
    /// to track that hold itself and decide what a further write of one to an already-set BCn
    /// means; this module reports only genuine 0 to 1 edges.
    #[cfg(test)]
    pub(crate) fn held_after_local_reset(&self, core: usize) -> bool {
        core < self.cores && self.local_boot_mode[core] == BOOTMODE_HOST
    }

    /// BOOTMODEn, the core's local boot option (SPRS612G Table 3-22).
    #[cfg(test)]
    pub(crate) fn local_boot_mode(&self, core: usize) -> u32 {
        if core >= self.cores {
            return BOOTMODE_IMMEDIATE;
        }
        self.local_boot_mode[core]
    }

    fn in_window(address: u32) -> bool {
        address.wrapping_sub(BASE) < WINDOW
    }

    /// Split a word address in the per-core region into (core, offset within the 20h slot), or
    /// None when the word is outside 02AB 0200 - 02AB 02BF.
    fn core_register(word: u32) -> Option<(usize, u32)> {
        let offset = word.checked_sub(CORE_BASE)?;
        let core = (offset / CORE_STRIDE) as usize;
        if core >= MAX_CORES {
            return None;
        }
        Some((core, offset % CORE_STRIDE))
    }

    /// BCn is set-only: "write 1 to set", a written zero leaves the bit alone (SPRS612G
    /// Figure 3-16, RS-0). A 0 to 1 transition releases a core the controller was holding.
    fn set_boot_complete(&mut self, value: u32) {
        let present = if self.cores == 0 {
            0
        } else {
            (1u32 << self.cores) - 1
        };
        let setting = value & present & !self.boot_complete;
        self.boot_complete |= setting;
        self.released |= setting;
        for core in 0..self.cores {
            if setting & (1 << core) != 0 {
                self.held[core] = false;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    fn immediate() -> BootController {
        BootController::new(0, MAX_CORES)
    }

    fn host() -> BootController {
        BootController::new(1, MAX_CORES)
    }

    fn rom() -> BootController {
        BootController::new(4, MAX_CORES)
    }

    #[test]
    fn immediate_boot_releases_every_core_at_once() {
        // SPRS612G section 2.4.2: "In the case of BOOTMODE 0, all cores are released from reset
        // immediately."
        let boot = immediate();
        for core in 0..MAX_CORES {
            assert!(!boot.held_in_reset(core));
        }
        assert!(!boot.boot_active());
    }

    #[test]
    fn host_boot_holds_all_six_cores() {
        // SPRS612G section 2.4.1, host boot: "all C64x+ megamodule cores are internally 'held in
        // reset'", and section 2.4.2: the host writes the boot complete bit "corresponding to
        // each C64x+ megamodule that is to be taken out of reset".
        let boot = host();
        for core in 0..MAX_CORES {
            assert!(boot.held_in_reset(core));
        }
        assert!(boot.boot_active());
    }

    #[test]
    fn rom_boot_releases_core_zero_only() {
        // SPRS612G section 2.4.2: "In the case of any other boot, core 0 comes out of RESET
        // immediately, but all other cores are still in RESET, so BOOTACTIVE will be high."
        let boot = rom();
        assert!(!boot.held_in_reset(0));
        for core in 1..MAX_CORES {
            assert!(boot.held_in_reset(core));
        }
        assert!(boot.boot_active());
    }

    #[test]
    fn boot_complete_release_clears_bootactive_when_the_last_core_starts() {
        // SPRS612G section 2.4.2: "BOOTACTIVE will go low after the boot complete bit (BCx) ...
        // is set for all six cores."
        let mut boot = rom();
        for core in 1..MAX_CORES {
            assert_ne!(
                boot.write(BOOT_COMPLETE_STAT, 1 << core, TEST_CONTEXT),
                PortWrite::Unhandled
            );
            assert!(!boot.held_in_reset(core));
            assert_eq!(boot.boot_active(), core != MAX_CORES - 1);
        }
    }

    #[test]
    fn boot_complete_is_set_only() {
        // SPRS612G Figure 3-16: BCn is RS-0, so a written zero cannot clear a set bit.
        let mut boot = host();
        assert_ne!(
            boot.write(BOOT_COMPLETE_STAT, 0b010101, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(boot.read(BOOT_COMPLETE_STAT, TEST_CONTEXT), Some(0b010101));
        assert_ne!(
            boot.write(BOOT_COMPLETE_STAT, 0, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(boot.read(BOOT_COMPLETE_STAT, TEST_CONTEXT), Some(0b010101));
        // Reserved bits 31:6 stay zero.
        assert_ne!(
            boot.write(BOOT_COMPLETE_STAT, 0xffff_ffff, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(boot.read(BOOT_COMPLETE_STAT, TEST_CONTEXT), Some(0b111111));
    }

    #[test]
    fn released_cores_edge_is_consumed_exactly_once() {
        // SPRS612G section 2.4.1: the 0 to 1 "transition causes the Boot Controller to bring the
        // C64x+ megamodule core out of the 'held-in-reset' state" - one release per transition.
        let mut boot = host();
        assert_eq!(boot.take_released_cores(), 0);
        boot.write(BOOT_COMPLETE_STAT, 0b000110, TEST_CONTEXT);
        assert_eq!(boot.take_released_cores(), 0b000110);
        assert_eq!(boot.take_released_cores(), 0);
        // Writing the same ones again is not a transition.
        boot.write(BOOT_COMPLETE_STAT, 0b000110, TEST_CONTEXT);
        assert_eq!(boot.take_released_cores(), 0);
        // A new bit is.
        boot.write(BOOT_COMPLETE_STAT, 0b001110, TEST_CONTEXT);
        assert_eq!(boot.take_released_cores(), 0b001000);
    }

    #[test]
    fn boot_address_defaults_and_shift_match_table_2_4() {
        // SPRS612G section 3.9.5: boot modes 0-1 give all six registers 00 2000h "which
        // corresponds to 0080 0000h"; boot modes 2-15 give DSP_BOOT_ADDR0 00 0400h, the shared L2
        // ROM base, i.e. 0010 0000h in Table 2-4.
        let boot = immediate();
        for core in 0..MAX_CORES {
            assert_eq!(boot.boot_address(core), 0x0080_0000);
        }
        let mut boot = rom();
        assert_eq!(boot.read(BOOTPROGRESS, TEST_CONTEXT), Some(0));
        assert_eq!(boot.boot_address(0), 0x0010_0000);
        assert_eq!(boot.read(CORE_BASE + 4, TEST_CONTEXT), Some(BOOT_ADDR_ROM));
        for core in 1..MAX_CORES {
            assert_eq!(boot.boot_address(core), 0x0080_0000);
            assert_eq!(
                boot.read(CORE_BASE + CORE_STRIDE * core as u32 + 4, TEST_CONTEXT),
                Some(BOOT_ADDR_L2RAM)
            );
        }
        // Field is the 22 MSBs: a written value appears shifted left by ten, and bits above 21
        // are not writable.
        assert_ne!(
            boot.write(CORE_BASE + CORE_STRIDE * 3 + 4, 0xffff_ffff, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(
            boot.read(CORE_BASE + CORE_STRIDE * 3 + 4, TEST_CONTEXT),
            Some(0x003f_ffff)
        );
        assert_eq!(boot.boot_address(3), 0xffff_fc00);
        assert_ne!(
            boot.write(CORE_BASE + CORE_STRIDE * 2 + 4, 0x0000_0001, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(boot.boot_address(2), 0x0000_0400);
    }

    #[test]
    fn bootmode_registers_default_to_immediate_and_keep_four_bits() {
        // SPRS612G section 3.9.4 and Table 3-22: "The default values of these registers are set
        // to immediate boot mode", field is bits 3:0, 31:4 reserved.
        let mut boot = rom();
        for core in 0..MAX_CORES {
            let address = CORE_BASE + CORE_STRIDE * core as u32;
            assert_eq!(boot.read(address, TEST_CONTEXT), Some(0));
            assert!(!boot.held_after_local_reset(core));
            assert_ne!(
                boot.write(address, 0xffff_fff1, TEST_CONTEXT),
                PortWrite::Unhandled
            );
            assert_eq!(boot.read(address, TEST_CONTEXT), Some(1));
            // BOOTMODEn = 1 is host boot: a local reset leaves the core held.
            assert!(boot.held_after_local_reset(core));
            assert_eq!(boot.local_boot_mode(core), BOOTMODE_HOST);
        }
    }

    #[test]
    fn reset_stat_reset_value_and_write_one_to_clear() {
        // SPRS612G Figure 3-15: GR is RC-0 and each LRn is RC-10. Table 3-20: "Writing a 1 to
        // LRn clears the bit; writing a 0 has no effect."
        let mut boot = immediate();
        assert_eq!(boot.read(RESET_STAT, TEST_CONTEXT), Some(0b1010_1010_1010));
        // A zero write changes nothing.
        assert_ne!(
            boot.write(RESET_STAT, 0, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(boot.read(RESET_STAT, TEST_CONTEXT), Some(0b1010_1010_1010));
        // Clearing LR2 only touches bits 5:4.
        assert_ne!(
            boot.write(RESET_STAT, 0b11 << 4, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(boot.read(RESET_STAT, TEST_CONTEXT), Some(0b1010_1000_1010));
        assert_eq!(boot.local_reset_state(2), LR_NONE);
        assert_eq!(boot.local_reset_state(3), LR_RESET);
        // GR is write-one-to-clear as well, and reserved bits 30:12 never latch.
        boot.note_global_reset();
        assert_eq!(boot.read(RESET_STAT, TEST_CONTEXT), Some(0x8000_0a8a));
        assert_ne!(
            boot.write(RESET_STAT, 0xffff_ffff, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(boot.read(RESET_STAT, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn local_reset_bookkeeping_records_in_and_out() {
        // SPRS612G Table 3-20: LRn 01b "a local reset (lreset_in) has been asserted to core n",
        // 11b "core n has responded with lreset_out".
        let mut boot = immediate();
        boot.write(RESET_STAT, RESET_STAT_LR_MASK, TEST_CONTEXT);
        boot.note_local_reset(4, true);
        assert_eq!(boot.local_reset_state(4), LR_IN);
        assert_eq!(boot.read(RESET_STAT, TEST_CONTEXT), Some(LR_IN << 8));
        boot.note_local_reset(4, false);
        assert_eq!(boot.local_reset_state(4), LR_OUT);
        assert_eq!(boot.read(RESET_STAT, TEST_CONTEXT), Some(LR_OUT << 8));
        // Other cores are untouched.
        assert_eq!(boot.local_reset_state(5), LR_NONE);
    }

    #[test]
    fn bootprogress_is_a_plain_scratch_word() {
        // SPRS612G section 3.9.3 and Figure 3-17: R/W-0, all 32 bits, no fields.
        let mut boot = immediate();
        assert_eq!(boot.read(BOOTPROGRESS, TEST_CONTEXT), Some(0));
        assert_ne!(
            boot.write(BOOTPROGRESS, 0xdead_beef, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(boot.read(BOOTPROGRESS, TEST_CONTEXT), Some(0xdead_beef));
    }

    #[test]
    fn reserved_words_inside_the_window_read_zero_and_absorb_writes() {
        // SPRS612G Table 3-14 lists 02AB 000C - 02AB 01FC and 02AB 02A8 - 02AB 7FFC as Reserved,
        // but they are still inside the block's 32K window (Table 2-2).
        let mut boot = immediate();
        for address in [BASE + 0x00c, BASE + 0x100, BASE + 0x2a8, BASE + 0x7ffc] {
            assert_eq!(boot.read(address, TEST_CONTEXT), Some(0));
            assert_ne!(
                boot.write(address, 0xffff_ffff, TEST_CONTEXT),
                PortWrite::Unhandled
            );
            assert_eq!(boot.read(address, TEST_CONTEXT), Some(0));
        }
        // Registers answer at their own word only: RESET_STAT does not alias 02AB 0100.
        assert_ne!(
            boot.write(BASE + 0x100, RESET_STAT_LR_MASK, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(boot.read(RESET_STAT, TEST_CONTEXT), Some(RESET_STAT_RESET));
    }

    #[test]
    fn addresses_outside_the_block_are_not_ours() {
        // SPRS612G Table 2-2: the boot controller is 02AB 0000 - 02AB 7FFF; 02AB 8000 onwards is
        // the next (reserved) region and 02AA FFFF belongs to the shared memory controller.
        let mut boot = immediate();
        for address in [BASE - 4, BASE + WINDOW, 0x02a8_0000, 0x0000_0000] {
            assert_eq!(boot.read(address, TEST_CONTEXT), None);
            assert_eq!(
                boot.write(address, 0xffff_ffff, TEST_CONTEXT),
                PortWrite::Unhandled
            );
        }
    }

    #[test]
    fn a_device_with_fewer_cores_ignores_the_missing_ones() {
        // Not a documented C6472 configuration - the device always has six megamodules - but the
        // model is built with a core count, so the extra BC bits must not release phantom cores.
        let mut boot = BootController::new(1, 2);
        assert_ne!(
            boot.write(BOOT_COMPLETE_STAT, 0b111111, TEST_CONTEXT),
            PortWrite::Unhandled
        );
        assert_eq!(boot.read(BOOT_COMPLETE_STAT, TEST_CONTEXT), Some(0b11));
        assert_eq!(boot.take_released_cores(), 0b11);
        assert!(!boot.boot_active());
        assert!(!boot.held_in_reset(2));
        assert_eq!(boot.boot_address(2), 0);
    }
}
