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
/// BOOT_UNLOCK guards each profile's `boot_locked` SDCFG bits; TIMUNLOCK guards SDTIM1/SDTIM2.
const SDCFG_BOOT_UNLOCK: u32 = 1 << 23;
const SDCFG_TIMUNLOCK: u32 = 1 << 15;
/// A refresh rate below 100h is replaced by twice the refresh cycle time SDTIM1 holds
/// (SPRU970G Table 21).
const REFRESH_FLOOR: u32 = 0x0100;

/// A device's DDR2 register contract (SPRU970G §4 for the C6455).
#[derive(Debug)]
pub(crate) struct Ddr2Profile {
    /// Module identification (SPRU970G Figure 19).
    midr: u32,
    /// SDCFG reset (SPRU970G Figure 21).
    sdcfg_reset: u32,
    /// SDRFC reset (SPRU970G Figure 22).
    sdrfc_reset: u32,
    /// SDTIM1 reset (SPRU970G Figure 23).
    sdtim1_reset: u32,
    /// SDTIM2 reset (SPRU970G Figure 24).
    sdtim2_reset: u32,
    /// SDCFG bits writable without BOOT_UNLOCK (SPRU970G Table 20).
    writable: u32,
    /// SDCFG bits requiring BOOT_UNLOCK (SPRU970G Table 20).
    boot_locked: u32,
    /// DMCCTL writable bits (SPRU970G Table 25).
    control_mask: u32,
}
/// C6455 DDR2 register contract (SPRU970G §4).
pub(crate) static C6455_DDR2: Ddr2Profile = Ddr2Profile {
    midr: MIDR_VALUE,
    sdcfg_reset: SDCFG_RESET,
    sdrfc_reset: SDRFC_RESET,
    sdtim1_reset: SDTIM1_RESET,
    sdtim2_reset: SDTIM2_RESET,
    writable: SDCFG_BOOT_UNLOCK | SDCFG_TIMUNLOCK | (1 << 14) | 0xe77,
    boot_locked: 1 << 18,
    control_mask: 0xffff,
};

pub struct Ddr2 {
    profile: &'static Ddr2Profile,
    config: u32,
    refresh: u32,
    timing1: u32,
    timing2: u32,
    priority: u32,
    control: u32,
}

impl Default for Ddr2 {
    fn default() -> Self {
        Self::new(&C6455_DDR2)
    }
}

impl Ddr2 {
    pub(crate) fn new(profile: &'static Ddr2Profile) -> Self {
        Self {
            profile,
            config: profile.sdcfg_reset,
            refresh: profile.sdrfc_reset,
            timing1: profile.sdtim1_reset,
            timing2: profile.sdtim2_reset,
            priority: BPRIO_RESET,
            control: DMCCTL_RESET,
        }
    }
}

impl SlavePort for Ddr2 {
    fn read(&mut self, address: u32, context: PortContext) -> Option<u32> {
        let value = match address {
            MIDR => self.profile.midr,
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
                // Profile masks select writable fields; reserved fields retain reset values.
                let mut writable = self.profile.writable;
                // Accept prior or incoming BOOT_UNLOCK: SPRU970G §2.11.3 sets it and then programs
                // DDR_DRIVE in the write that clears it, while existing callers and TI IBL also
                // program fields in the unlock write.
                let boot_unlocked = (self.config | value) & SDCFG_BOOT_UNLOCK != 0;
                if boot_unlocked {
                    writable |= self.profile.boot_locked;
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
            DMCCTL => {
                let writable = self.profile.control_mask;
                self.control = (DMCCTL_RESET & !writable) | (value & writable);
            }
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
        // A locked field stays fixed until a prior or incoming write unlocks it.
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

    #[test]
    fn boot_unlock_accepts_prior_and_incoming_unlocks() {
        // SPRU970G §2.11.3 and Table 20: accept prior and incoming unlocks.
        for prior in [false, true] {
            let mut d = Ddr2::default();
            let initial = d.config;
            if prior {
                d.write(SDCFG, initial | SDCFG_BOOT_UNLOCK, LITTLE_CONTEXT);
            }
            d.write(
                SDCFG,
                initial | (1 << 18) | if prior { 0 } else { SDCFG_BOOT_UNLOCK },
                LITTLE_CONTEXT,
            );
            assert_ne!(d.config & (1 << 18), 0);
        }
    }

    #[test]
    fn c6455_reserved_sdcfg_fields_keep_figure_21_defaults() {
        // SPRU970G Figure 21: reserved 31:24, 22:19=A, 17:16=3, 13:12, 8:7, 3.
        const RESERVED: u32 = 0xff00_0000 | (0xf << 19) | (3 << 16) | 0x3000 | 0x180 | 8;
        for prior in [false, true] {
            for value in [0, u32::MAX] {
                let mut d = Ddr2::default();
                if prior {
                    d.write(SDCFG, SDCFG_BOOT_UNLOCK, LITTLE_CONTEXT);
                }
                let value = value & !SDCFG_BOOT_UNLOCK;
                d.write(
                    SDCFG,
                    value | if prior { 0 } else { SDCFG_BOOT_UNLOCK },
                    LITTLE_CONTEXT,
                );
                assert_eq!(d.config & RESERVED, 0x0053_0000);
                // Only DDR_DRIVE is boot-locked; the ordinary fields remain writable.
                assert_eq!(d.config & 0x004e77, value & 0x004e77);
                assert_eq!(d.config & (1 << 18), value & (1 << 18));
            }
        }
    }
}
