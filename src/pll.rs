//! The C6455's PLL1 controller, its reset controller register, and the PLL2 controller.
//!
//! SPRUE56 with the device-specific register set in SPRS276M section 7.7.3: a multiplier, a
//! pre-divider, the two programmable system-clock dividers this device exposes, and the GO
//! operation that moves a newly written divider ratio into effect. Writing a different ratio
//! flags the divider in DCHANGE; the GO operation applies exactly those dividers and clears the
//! flags. Dividers D2 and D3, which clock the megamodule and the peripherals, are fixed at /3
//! and /6 and have no registers.
//!
//! The reset controller's only register, RSTYPE, shares the PLL1 address range and latches which
//! reset last happened (SPRS276M section 7.6.7). This model always starts from a power-on reset.
//!
//! The PLL2 controller (SPRS276M section 7.8) has a fixed x20 multiplier and one programmable
//! divider, D1, which generates SYSCLK1 for the EMAC. Only PLLDIV1, the command and status
//! registers, the alignment control, the ratio-change status and the SYSCLK status register exist
//! on this device, and the same GO protocol applies.
use crate::port::{PortContext, PortWrite, SlavePort};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const BASE: u32 = 0x029a_0000;
const PLLCTL: u32 = BASE + 0x0100;
const PLLM: u32 = BASE + 0x0110;
const PREDIV: u32 = BASE + 0x0114;
const PLLCMD: u32 = BASE + 0x0138;
const PLLSTAT: u32 = BASE + 0x013c;
const ALNCTL: u32 = BASE + 0x0140;
const DCHANGE: u32 = BASE + 0x0144;
const SYSTAT: u32 = BASE + 0x0150;
const PLLDIV4: u32 = BASE + 0x0160;
const PLLDIV5: u32 = BASE + 0x0164;
/// The reset controller's only register lives in the PLL1 range (SPRS276M section 7.6.7).
const RSTYPE: u32 = BASE + 0x00e4;
/// This model comes out of a power-on reset, which RSTYPE latches in bit 0 (Table 7-13).
const RSTYPE_VALUE: u32 = 1 << 0;

/// PLLCTL: bit 6 always reads one, PLL reset is asserted out of reset, and only the reset,
/// power-down and enable bits are writable (SPRS276M Table 7-19).
const PLLCTL_ONES: u32 = 1 << 6;
const PLLCTL_WRITABLE: u32 = (1 << 3) | (1 << 1) | (1 << 0);
/// Each divider register holds an enable bit and a five-bit ratio, both reset to a running
/// divider (SPRS276M Figures 7-14 and 7-15).
const DIVIDER_MASK: u32 = (1 << 15) | 0x1f;
const DIVIDER_RESET: u32 = (1 << 15) | 3;
/// ALNCTL and SYSTAT are read-only on this device: SYSCLK4 and SYSCLK5 are always aligned, and
/// every system clock is on (SPRS276M Tables 7-26 and 7-28).
const ALNCTL_VALUE: u32 = (1 << 4) | (1 << 3);
const SYSTAT_VALUE: u32 = 0x1f;
/// DCHANGE reports SYSCLK4 in bit 3 and SYSCLK5 in bit 4.
const DCHANGE_SYS4: u32 = 1 << 3;
const DCHANGE_SYS5: u32 = 1 << 4;

pub struct Pll {
    control: u32,
    multiplier: u32,
    prediv: u32,
    command: u32,
    dchange: u32,
    divider: [u32; 2],
    /// The ratio each divider is actually running at, which only a GO operation updates.
    active: [u32; 2],
}

impl Default for Pll {
    fn default() -> Self {
        Self {
            control: PLLCTL_ONES | (1 << 3),
            multiplier: 0,
            prediv: 0,
            command: 0,
            dchange: 0,
            divider: [DIVIDER_RESET; 2],
            active: [DIVIDER_RESET & 0x1f; 2],
        }
    }
}

impl SlavePort for Pll {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let value = match address {
            PLLCTL => self.control,
            PLLM => self.multiplier,
            PREDIV => self.prediv,
            PLLCMD => self.command,
            // A GO operation completes within the window in which the caution in SPRUE56
            // section 3.2.1.1 requires all chip operation to be stopped, so it is never seen
            // in progress from software.
            PLLSTAT => 0,
            ALNCTL => ALNCTL_VALUE,
            DCHANGE => self.dchange,
            SYSTAT => SYSTAT_VALUE,
            PLLDIV4 => self.divider[0],
            PLLDIV5 => self.divider[1],
            RSTYPE => RSTYPE_VALUE,
            _ => return None,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        match address {
            PLLCTL => self.control = PLLCTL_ONES | (value & PLLCTL_WRITABLE),
            PLLM => self.multiplier = value & 0x3f,
            PREDIV => self.prediv = value & DIVIDER_MASK,
            PLLCMD => {
                // GOSET stays set once written, and each write of one starts a GO operation.
                self.command = value & 1;
                if value & 1 != 0 {
                    self.go();
                }
            }
            PLLSTAT | ALNCTL | SYSTAT | RSTYPE => {}
            PLLDIV4 => self.write_divider(0, DCHANGE_SYS4, value),
            PLLDIV5 => self.write_divider(1, DCHANGE_SYS5, value),
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

impl Pll {
    /// Writing a ratio different from the one the divider is running at flags it in DCHANGE
    /// (SPRS276M Table 7-27).
    fn write_divider(&mut self, index: usize, flag: u32, value: u32) {
        self.divider[index] = value & DIVIDER_MASK;
        if self.divider[index] & 0x1f != self.active[index] {
            self.dchange |= flag;
        } else {
            self.dchange &= !flag;
        }
    }

    /// A GO operation moves the flagged dividers to their new ratios and clears the flags.
    fn go(&mut self) {
        for (index, flag) in [(0, DCHANGE_SYS4), (1, DCHANGE_SYS5)] {
            if self.dchange & flag != 0 {
                self.active[index] = self.divider[index] & 0x1f;
            }
        }
        self.dchange = 0;
    }
}

const PLL2_BASE: u32 = 0x029c_0000;
const PLL2_PLLDIV1: u32 = PLL2_BASE + 0x0118;
const PLL2_PLLCMD: u32 = PLL2_BASE + 0x0138;
const PLL2_PLLSTAT: u32 = PLL2_BASE + 0x013c;
const PLL2_ALNCTL: u32 = PLL2_BASE + 0x0140;
const PLL2_DCHANGE: u32 = PLL2_BASE + 0x0144;
const PLL2_SYSTAT: u32 = PLL2_BASE + 0x0150;

/// D1 comes up enabled dividing by two, which is the only other legal ratio besides five
/// (SPRS276M Table 7-33).
const PLL2_DIVIDER_RESET: u32 = (1 << 15) | 1;
/// ALN1 resets set, so SYSCLK1 is aligned on a GO operation (SPRS276M Table 7-36).
const PLL2_ALNCTL_ALN1: u32 = 1 << 0;
/// SYSTAT reports SYSCLK1 on and nothing else (SPRS276M Table 7-38).
const PLL2_SYSTAT_VALUE: u32 = 1 << 0;
/// DCHANGE reports the one divider this controller has, SYSCLK1, in bit 0.
const PLL2_DCHANGE_SYS1: u32 = 1 << 0;

pub struct Pll2 {
    divider: u32,
    active: u32,
    command: u32,
    align: u32,
    dchange: u32,
}

impl Default for Pll2 {
    fn default() -> Self {
        Self {
            divider: PLL2_DIVIDER_RESET,
            active: PLL2_DIVIDER_RESET & 0x1f,
            command: 0,
            align: PLL2_ALNCTL_ALN1,
            dchange: 0,
        }
    }
}

impl SlavePort for Pll2 {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let value = match address {
            PLL2_PLLDIV1 => self.divider,
            PLL2_PLLCMD => self.command,
            // As with PLL1, the GO operation finishes inside the window software is required to
            // keep quiet, so it is never observed in progress.
            PLL2_PLLSTAT => 0,
            PLL2_ALNCTL => self.align,
            PLL2_DCHANGE => self.dchange,
            PLL2_SYSTAT => PLL2_SYSTAT_VALUE,
            _ => return None,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        match address {
            PLL2_PLLDIV1 => {
                self.divider = value & DIVIDER_MASK;
                if self.divider & 0x1f != self.active {
                    self.dchange |= PLL2_DCHANGE_SYS1;
                } else {
                    self.dchange &= !PLL2_DCHANGE_SYS1;
                }
            }
            PLL2_PLLCMD => {
                self.command = value & 1;
                if value & 1 != 0 {
                    if self.dchange & PLL2_DCHANGE_SYS1 != 0 {
                        self.active = self.divider & 0x1f;
                    }
                    self.dchange = 0;
                }
            }
            PLL2_ALNCTL => self.align = value & PLL2_ALNCTL_ALN1,
            PLL2_PLLSTAT | PLL2_DCHANGE | PLL2_SYSTAT => {}
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    #[test]
    fn the_reset_values_are_the_ones_the_device_manual_gives() {
        // SPRS276M section 7.7.3: PLLCTL comes up with the PLL in reset and bit 6 reading one,
        // both dividers enabled at their reset ratio, every system clock on, and both
        // alignments fixed.
        let mut pll = Pll::default();
        assert_eq!(pll.read(PLLCTL, TEST_CONTEXT), Some((1 << 6) | (1 << 3)));
        assert_eq!(pll.read(PLLDIV4, TEST_CONTEXT), Some(0x8003));
        assert_eq!(pll.read(PLLDIV5, TEST_CONTEXT), Some(0x8003));
        assert_eq!(pll.read(ALNCTL, TEST_CONTEXT), Some(0x18));
        assert_eq!(pll.read(SYSTAT, TEST_CONTEXT), Some(0x1f));
        assert_eq!(pll.read(DCHANGE, TEST_CONTEXT), Some(0));
        assert_eq!(pll.read(PLLSTAT, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn a_changed_ratio_is_flagged_until_the_go_operation() {
        // SPRUE56 section 3.2.1.2: program the ratio, see it flagged in DCHANGE, then the GO
        // operation applies it and clears the flag.
        let mut pll = Pll::default();
        pll.write(PLLDIV4, 0x8005, TEST_CONTEXT);
        assert_eq!(pll.read(DCHANGE, TEST_CONTEXT), Some(DCHANGE_SYS4));
        // Writing the ratio the divider already runs at leaves nothing flagged.
        pll.write(PLLDIV5, 0x8003, TEST_CONTEXT);
        assert_eq!(pll.read(DCHANGE, TEST_CONTEXT), Some(DCHANGE_SYS4));
        pll.write(PLLCMD, 1, TEST_CONTEXT);
        assert_eq!(pll.read(DCHANGE, TEST_CONTEXT), Some(0));
        assert_eq!(pll.read(PLLCMD, TEST_CONTEXT), Some(1));
        assert_eq!(pll.read(PLLDIV4, TEST_CONTEXT), Some(0x8005));
        // The applied ratio is now the running one, so rewriting it flags nothing.
        pll.write(PLLDIV4, 0x8005, TEST_CONTEXT);
        assert_eq!(pll.read(DCHANGE, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn only_the_documented_control_bits_are_writable() {
        // SPRS276M Table 7-19: the reserved bits are fixed, and PLLM and PREDIV keep only
        // their own fields.
        let mut pll = Pll::default();
        pll.write(PLLCTL, 0xffff_ffff, TEST_CONTEXT);
        assert_eq!(
            pll.read(PLLCTL, TEST_CONTEXT),
            Some((1 << 6) | (1 << 3) | (1 << 1) | 1)
        );
        pll.write(PLLCTL, 0, TEST_CONTEXT);
        assert_eq!(pll.read(PLLCTL, TEST_CONTEXT), Some(1 << 6));
        pll.write(PLLM, 0xffff, TEST_CONTEXT);
        assert_eq!(pll.read(PLLM, TEST_CONTEXT), Some(0x3f));
        pll.write(PREDIV, 0xffff, TEST_CONTEXT);
        assert_eq!(pll.read(PREDIV, TEST_CONTEXT), Some(0x801f));
    }

    #[test]
    fn the_reset_controller_reports_a_power_on_reset() {
        // SPRS276M Table 7-13: RSTYPE is read-only and latches the last reset, which for this
        // model is always the power-on reset.
        let mut pll = Pll::default();
        assert_eq!(pll.read(RSTYPE, TEST_CONTEXT), Some(1));
        pll.write(RSTYPE, 0, TEST_CONTEXT);
        assert_eq!(pll.read(RSTYPE, TEST_CONTEXT), Some(1));
    }

    #[test]
    fn the_pll2_controller_moves_its_one_divider_on_a_go_operation() {
        // SPRS276M section 7.8.3: D1 comes up enabled at /2, a write of /5 is flagged in
        // DCHANGE, and the GO operation applies it and clears the flag.
        let mut pll2 = Pll2::default();
        assert_eq!(pll2.read(PLL2_PLLDIV1, TEST_CONTEXT), Some(0x8001));
        assert_eq!(pll2.read(PLL2_ALNCTL, TEST_CONTEXT), Some(1));
        assert_eq!(pll2.read(PLL2_SYSTAT, TEST_CONTEXT), Some(1));
        assert_eq!(pll2.read(PLL2_DCHANGE, TEST_CONTEXT), Some(0));
        assert_eq!(pll2.read(PLL2_PLLSTAT, TEST_CONTEXT), Some(0));
        pll2.write(PLL2_PLLDIV1, (1 << 15) | 4, TEST_CONTEXT);
        assert_eq!(pll2.read(PLL2_DCHANGE, TEST_CONTEXT), Some(1));
        pll2.write(PLL2_PLLCMD, 1, TEST_CONTEXT);
        assert_eq!(pll2.read(PLL2_DCHANGE, TEST_CONTEXT), Some(0));
        assert_eq!(pll2.read(PLL2_PLLCMD, TEST_CONTEXT), Some(1));
        assert_eq!(pll2.read(PLL2_PLLDIV1, TEST_CONTEXT), Some((1 << 15) | 4));
        // Rewriting the running ratio flags nothing, and the reserved middle field is dropped.
        pll2.write(PLL2_PLLDIV1, 0xffff_ffff, TEST_CONTEXT);
        assert_eq!(pll2.read(PLL2_PLLDIV1, TEST_CONTEXT), Some(0x801f));
        assert_eq!(pll2.read(PLL2_DCHANGE, TEST_CONTEXT), Some(1));
    }
}
