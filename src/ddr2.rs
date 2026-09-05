//! The C6455's DDR2 memory controller registers.
//!
//! SPRU970G section 4: the module identifies itself, reports whether its interface logic is out
//! of reset and which endianness it inherited, and holds the SDRAM configuration, refresh and
//! timing values. The data path itself is plain memory in this model - DDR2 latency and refresh
//! are deliberately not simulated - but the register contract firmware programs at boot is here,
//! including the locks that guard the timing registers and the refresh rate the controller
//! substitutes when software programs one that is too small.
use crate::port::{PortContext, PortWrite, SlavePort};
use crate::system::Endian;
#[cfg(test)]
const LITTLE_CONTEXT: PortContext = PortContext {
    endian: Endian::Little,
};
#[cfg(test)]
const BIG_CONTEXT: PortContext = PortContext {
    endian: Endian::Big,
};

const BASE: u32 = 0x7800_0000;
const MIDR: u32 = BASE;
const DMCSTAT: u32 = BASE + 0x04;
const SDCFG: u32 = BASE + 0x08;
const SDRFC: u32 = BASE + 0x0c;
const SDTIM1: u32 = BASE + 0x10;
const SDTIM2: u32 = BASE + 0x14;
const BPRIO: u32 = BASE + 0x20;
const DMCCTL: u32 = BASE + 0xe4;

/// Module id 31h, major revision 3, minor revision Fh (SPRU970G Figure 19).
const MIDR_VALUE: u32 = (0x0031 << 16) | (0x03 << 8) | 0x0f;
/// DMCSTAT bit 30 always reads one, bit 31 carries the endianness and bit 2 the interface-ready
/// status (SPRU970G Table 19).
const DMCSTAT_ONES: u32 = 1 << 30;
const DMCSTAT_BIG_ENDIAN: u32 = 1 << 31;
const DMCSTAT_IFRDY: u32 = 1 << 2;
/// Reset values from SPRU970G Figures 21 to 26.
const SDCFG_RESET: u32 = (0xa << 19) | (0x3 << 16) | (0x5 << 9) | (0x2 << 4);
const SDRFC_RESET: u32 = 0x0753;
const SDTIM1_RESET: u32 = (0x3f << 25)
    | (0x7 << 22)
    | (0x7 << 19)
    | (0x7 << 16)
    | (0x1f << 11)
    | (0x1f << 6)
    | (0x7 << 3)
    | 0x3;
const SDTIM2_RESET: u32 = (0x3 << 23) | (0x7f << 16) | (0xff << 8) | (0x7 << 5) | 0x1f;
const BPRIO_RESET: u32 = 0xff;
const DMCCTL_RESET: u32 = (0x5000 << 16) | (0x190 << 6) | DMCCTL_IFRESET | 0x7;
const DMCCTL_IFRESET: u32 = 1 << 5;
/// SDCFG locks: BOOT_UNLOCK guards bits 22-16 and TIMUNLOCK the timing registers.
const SDCFG_BOOT_UNLOCK: u32 = 1 << 23;
const SDCFG_TIMUNLOCK: u32 = 1 << 15;
/// A refresh rate below 100h is replaced by twice the refresh cycle time SDTIM1 holds
/// (SPRU970G Table 21).
const REFRESH_FLOOR: u32 = 0x0100;

pub struct Ddr2 {
    config: u32,
    refresh: u32,
    timing1: u32,
    timing2: u32,
    priority: u32,
    control: u32,
}

impl Default for Ddr2 {
    fn default() -> Self {
        Self {
            config: SDCFG_RESET,
            refresh: SDRFC_RESET,
            timing1: SDTIM1_RESET,
            timing2: SDTIM2_RESET,
            priority: BPRIO_RESET,
            control: DMCCTL_RESET,
        }
    }
}

impl SlavePort for Ddr2 {
    fn read(&mut self, address: u32, context: PortContext) -> Option<u32> {
        let value = match address {
            MIDR => MIDR_VALUE,
            DMCSTAT => {
                // The interface logic is ready once software has released its reset.
                DMCSTAT_ONES
                    | (u32::from(context.endian == Endian::Big) * DMCSTAT_BIG_ENDIAN)
                    | (u32::from(self.control & DMCCTL_IFRESET == 0) * DMCSTAT_IFRDY)
            }
            SDCFG => self.config,
            SDRFC => self.refresh,
            SDTIM1 => self.timing1,
            SDTIM2 => self.timing2,
            BPRIO => self.priority,
            DMCCTL => self.control,
            _ => return None,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        match address {
            MIDR | DMCSTAT => {}
            SDCFG => {
                // The writable fields are BOOT_UNLOCK, DDR_DRIVE, TIMUNLOCK, NM, CL, IBANK and
                // PAGESIZE; the reserved fields keep their reset values, and DDR_DRIVE only
                // moves when the same write unlocks it (SPRU970G Table 20).
                let mut writable = SDCFG_BOOT_UNLOCK | SDCFG_TIMUNLOCK | (1 << 14) | 0xe77;
                if value & SDCFG_BOOT_UNLOCK != 0 {
                    writable |= 1 << 18;
                }
                self.config = (self.config & !writable) | (value & writable);
            }
            SDRFC => {
                let rate = value & 0xffff;
                let rate = if rate < REFRESH_FLOOR {
                    2 * ((self.timing1 >> 25) & 0x7f)
                } else {
                    rate
                };
                self.refresh = (value & (1 << 31)) | rate;
            }
            // The timing registers only accept writes while TIMUNLOCK is set.
            SDTIM1 => {
                if self.config & SDCFG_TIMUNLOCK != 0 {
                    self.timing1 = value & !(1 << 2);
                }
            }
            SDTIM2 => {
                if self.config & SDCFG_TIMUNLOCK != 0 {
                    self.timing2 = value & 0x01ff_ffff;
                }
            }
            BPRIO => self.priority = value & 0xff,
            DMCCTL => self.control = (DMCCTL_RESET & 0xffff_0000) | (value & 0xffff),
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
    fn the_reset_values_are_the_ones_the_manual_gives() {
        // SPRU970G Figures 19 to 26, and Table 19: the interface logic comes up in reset, so it
        // is not ready, and bit 30 of the status register always reads one.
        let mut ddr2 = Ddr2::default();
        assert_eq!(ddr2.read(MIDR, LITTLE_CONTEXT), Some(0x0031_030f));
        assert_eq!(ddr2.read(DMCSTAT, LITTLE_CONTEXT), Some(1 << 30));
        assert_eq!(ddr2.read(DMCSTAT, BIG_CONTEXT), Some((1 << 31) | (1 << 30)));
        assert_eq!(ddr2.read(SDCFG, LITTLE_CONTEXT), Some(0x0053_0a20));
        assert_eq!(ddr2.read(SDRFC, LITTLE_CONTEXT), Some(0x0753));
        assert_eq!(ddr2.read(SDTIM1, LITTLE_CONTEXT), Some(0x7fff_fffb));
        assert_eq!(ddr2.read(SDTIM2, LITTLE_CONTEXT), Some(0x01ff_ffff));
        assert_eq!(ddr2.read(BPRIO, LITTLE_CONTEXT), Some(0xff));
        assert_eq!(ddr2.read(DMCCTL, LITTLE_CONTEXT), Some(0x5000_6427));
    }

    #[test]
    fn releasing_the_interface_reset_makes_it_ready() {
        // SPRU970G Table 25 and Table 19: IFRESET drives the ready status the CPU polls.
        let mut ddr2 = Ddr2::default();
        ddr2.write(DMCCTL, 0x6407, LITTLE_CONTEXT);
        assert_eq!(ddr2.read(DMCCTL, LITTLE_CONTEXT), Some(0x5000_6407));
        assert_eq!(
            ddr2.read(DMCSTAT, LITTLE_CONTEXT),
            Some((1 << 30) | (1 << 2))
        );
    }

    #[test]
    fn the_timing_registers_and_refresh_rate_follow_their_locks() {
        // SPRU970G Table 20 and Table 21: the timing registers move only while TIMUNLOCK is
        // set, and a refresh rate below 100h is replaced by twice the refresh cycle time.
        let mut ddr2 = Ddr2::default();
        ddr2.write(SDTIM1, 0, LITTLE_CONTEXT);
        assert_eq!(ddr2.read(SDTIM1, LITTLE_CONTEXT), Some(0x7fff_fffb));
        ddr2.write(SDCFG, SDCFG_RESET | SDCFG_TIMUNLOCK, LITTLE_CONTEXT);
        ddr2.write(SDTIM1, (0x1f << 25) | 3, LITTLE_CONTEXT);
        assert_eq!(ddr2.read(SDTIM1, LITTLE_CONTEXT), Some((0x1f << 25) | 3));
        ddr2.write(SDTIM2, 0x0100_0000, LITTLE_CONTEXT);
        assert_eq!(ddr2.read(SDTIM2, LITTLE_CONTEXT), Some(0x0100_0000));
        // A rate of 0x80 is too small, so the controller loads 2 x T_RFC, which is 0x1F here.
        ddr2.write(SDRFC, 0x80, LITTLE_CONTEXT);
        assert_eq!(ddr2.read(SDRFC, LITTLE_CONTEXT), Some(0x3e));
        ddr2.write(SDRFC, (1 << 31) | 0x79e, LITTLE_CONTEXT);
        assert_eq!(ddr2.read(SDRFC, LITTLE_CONTEXT), Some((1 << 31) | 0x79e));
        // The boot-locked fields need BOOT_UNLOCK in the same write.
        ddr2.write(SDCFG, SDCFG_RESET | (1 << 18), LITTLE_CONTEXT);
        assert_eq!(ddr2.read(SDCFG, LITTLE_CONTEXT).unwrap() & (1 << 18), 0);
        ddr2.write(
            SDCFG,
            SDCFG_RESET | SDCFG_BOOT_UNLOCK | (1 << 18),
            LITTLE_CONTEXT,
        );
        assert_eq!(
            ddr2.read(SDCFG, LITTLE_CONTEXT).unwrap() & (1 << 18),
            1 << 18
        );
    }
}
