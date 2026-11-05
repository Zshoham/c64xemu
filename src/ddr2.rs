//! DDR2 memory controller register contracts for the C6455 and C6472.
//!
//! SPRU970G section 4 (C6455) and SPRU894J section 4 (C6472): the module identifies itself,
//! reports whether its interface logic is out of reset and which endianness it inherited,
//! and holds the SDRAM configuration, refresh and
//! timing values. The data path itself is plain memory in this model - DDR2 latency and refresh
//! are deliberately not simulated - but the register contract firmware programs at boot is here.
//! These devices share a register layout, not reset values or all writable fields.
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

/// C6455 module id 31h, major revision 3, minor revision Fh (SPRU970G Figure 19).
const MIDR_VALUE: u32 = (0x0031 << 16) | (0x03 << 8) | 0x0f;
/// C6472 module id 034h, major revision 0Bh, minor revision 1Fh (SPRU894J Figure 19).
const C6472_MIDR_VALUE: u32 = (0x0034 << 16) | (0x0b << 8) | 0x1f;
/// DMCSTAT bit 30 reads one on the C6455 (SPRU970G Table 19); on the C6472 it is the
/// DUAL_CLK_MODE input and has reset value one (SPRU894J Figure 20). Bit 31 is endianness and
/// bit 2 is interface-ready on both devices.
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

/// C6472/TCI648x reset values from SPRU894J Figures 19-26. Its reset configuration differs
/// materially from the C6455 values, including the NM/CL/IBANK/PAGESIZE fields and EBANK support.
const C6472_SDCFG_RESET: u32 = (1 << 21) | (2 << 9) | (2 << 4);
const C6472_SDRFC_RESET: u32 = 0x1388;
const C6472_SDTIM1_RESET: u32 = (0x2a << 25)
    | (0x4 << 22)
    | (0x4 << 19)
    | (0x4 << 16)
    | (0xe << 11)
    | (0xb << 6)
    | (0x3 << 3)
    | 0x2;
const C6472_SDTIM2_RESET: u32 = (0x1 << 23) | (0x2d << 16) | (0xc7 << 8) | (0x2 << 5) | 0x2;

/// Register contract: SPRU970G §4 (C6455), SPRU894J §4 (C6472).
#[derive(Debug)]
pub(crate) struct Ddr2Profile {
    /// Module identification (SPRU970G/SPRU894J Figure 19).
    midr: u32,
    /// SDCFG reset (SPRU970G and SPRU894J Figure 21).
    sdcfg_reset: u32,
    /// SDRFC reset (SPRU970G and SPRU894J Figure 22).
    sdrfc_reset: u32,
    /// SDTIM1 reset (SPRU970G and SPRU894J Figure 23).
    sdtim1_reset: u32,
    /// SDTIM2 reset (SPRU970G and SPRU894J Figure 24).
    sdtim2_reset: u32,
    /// SDCFG bits writable without BOOT_UNLOCK (SPRU970G/SPRU894J Table 20).
    writable: u32,
    /// SDCFG bits requiring BOOT_UNLOCK (SPRU970G/SPRU894J Table 20).
    boot_locked: u32,
    /// DMCCTL writable bits (SPRU970G/SPRU894J Table 25).
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
/// C6472 DDR2 register contract (SPRU894J §4).
pub(crate) static C6472_DDR2: Ddr2Profile = Ddr2Profile {
    midr: C6472_MIDR_VALUE,
    sdcfg_reset: C6472_SDCFG_RESET,
    sdrfc_reset: C6472_SDRFC_RESET,
    sdtim1_reset: C6472_SDTIM1_RESET,
    sdtim2_reset: C6472_SDTIM2_RESET,
    writable: SDCFG_BOOT_UNLOCK | SDCFG_TIMUNLOCK | (1 << 14) | 0xe7f,
    boot_locked: (1 << 27) | (0x7f << 16),
    control_mask: 7,
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
    /// The C6472 DDR controller uses SPRU894J's contract; `Default` remains the historical
    /// C6455/SPRU970G controller so existing C6455 callers retain their register behavior.
    #[cfg(test)]
    pub(crate) fn c6472() -> Self {
        Self::new(&C6472_DDR2)
    }

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
                // The interface logic is ready once software has released IFRESET. The C6472
                // reserves that DMCCTL bit, so it does not report ready through it.
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
                // Accept prior or incoming BOOT_UNLOCK: both manuals describe set-then-program,
                // while existing callers and TI IBL also program fields in the unlock write.
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
        // SPRU894J/SPRU970G §2.11.3 and Table 20: accept prior and incoming unlocks.
        for profile in [&C6455_DDR2, &C6472_DDR2] {
            for prior in [false, true] {
                let mut d = Ddr2::new(profile);
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
    }

    #[test]
    fn c6472_boot_locked_and_reserved_sdcfg_bits() {
        // SPRU894J Table 20: boot-locked fields and reserved bits.
        for bit in [27, 22, 21, 20, 19, 18, 17, 16] {
            let mut d = Ddr2::c6472();
            let changed = 0x00200420 ^ (1 << bit);
            d.write(SDCFG, changed, LITTLE_CONTEXT);
            assert_eq!(d.config, 0x00200420);
            d.write(SDCFG, changed | SDCFG_BOOT_UNLOCK, LITTLE_CONTEXT);
            assert_eq!(d.config, changed | SDCFG_BOOT_UNLOCK);
        }
        let mut d = Ddr2::c6472();
        d.write(SDCFG, 0xffffffff, LITTLE_CONTEXT);
        assert_eq!(d.config & 0xf7003180, 0);
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

    #[test]
    fn c6472_has_its_own_spru894j_reset_contract() {
        // SPRU894J Figures 19-26 and Tables 18-25, C6472 row where the guide distinguishes
        // device defaults. Keep this separate from the C6455/SPRU970G reset fixture above.
        let mut ddr2 = Ddr2::c6472();
        assert_eq!(ddr2.read(MIDR, LITTLE_CONTEXT), Some(0x0034_0b1f));
        assert_eq!(ddr2.read(DMCSTAT, LITTLE_CONTEXT), Some(1 << 30));
        assert_eq!(ddr2.read(SDCFG, LITTLE_CONTEXT), Some(0x0020_0420));
        assert_eq!(ddr2.read(SDRFC, LITTLE_CONTEXT), Some(0x1388));
        assert_eq!(ddr2.read(SDTIM1, LITTLE_CONTEXT), Some(0x5524_72da));
        assert_eq!(ddr2.read(SDTIM2, LITTLE_CONTEXT), Some(0x00ad_c742));
        assert_eq!(ddr2.read(BPRIO, LITTLE_CONTEXT), Some(0xff));
        assert_eq!(ddr2.read(DMCCTL, LITTLE_CONTEXT), Some(0x5000_6427));
    }

    #[test]
    fn c6472_sdcfg_masks_reserved_fields_but_exposes_ebank_and_boot_fields() {
        let mut ddr2 = Ddr2::c6472();
        let initial = ddr2.read(SDCFG, LITTLE_CONTEXT).unwrap();
        // EBANK is a C6472 field; reserved bit 7 and bits 13:12 remain at their reset values.
        ddr2.write(
            SDCFG,
            initial | (1 << 3) | (1 << 7) | (3 << 12),
            LITTLE_CONTEXT,
        );
        assert_eq!(ddr2.read(SDCFG, LITTLE_CONTEXT), Some(initial | (1 << 3)));

        // Set-then-program is one accepted BOOT_UNLOCK sequence; configuration fields are then
        // accepted by the following write that clears BOOT_UNLOCK (SPRU894J section 2.11.3).
        ddr2.write(SDCFG, initial | SDCFG_BOOT_UNLOCK, LITTLE_CONTEXT);
        ddr2.write(SDCFG, initial | (1 << 27) | (1 << 18), LITTLE_CONTEXT);
        assert_eq!(
            ddr2.read(SDCFG, LITTLE_CONTEXT),
            Some(initial | (1 << 27) | (1 << 18))
        );
    }

    #[test]
    fn c6472_dmcctl_only_changes_read_latency() {
        let mut ddr2 = Ddr2::c6472();
        ddr2.write(DMCCTL, 0, LITTLE_CONTEXT);
        assert_eq!(ddr2.read(DMCCTL, LITTLE_CONTEXT), Some(0x5000_6420));
        // The C6455-only IFRESET bit cannot be used to alter C6472's reserved control fields.
        assert_eq!(ddr2.read(DMCSTAT, LITTLE_CONTEXT), Some(1 << 30));
    }

    #[test]
    fn c6472_timing_register_writes_follow_timunlock() {
        let mut ddr2 = Ddr2::c6472();
        let timing1 = ddr2.read(SDTIM1, LITTLE_CONTEXT).unwrap();
        let timing2 = ddr2.read(SDTIM2, LITTLE_CONTEXT).unwrap();
        ddr2.write(SDTIM1, 0, LITTLE_CONTEXT);
        ddr2.write(SDTIM2, 0, LITTLE_CONTEXT);
        assert_eq!(ddr2.read(SDTIM1, LITTLE_CONTEXT), Some(timing1));
        assert_eq!(ddr2.read(SDTIM2, LITTLE_CONTEXT), Some(timing2));

        ddr2.write(SDCFG, C6472_SDCFG_RESET | SDCFG_TIMUNLOCK, LITTLE_CONTEXT);
        ddr2.write(SDTIM1, 0x1234_5678, LITTLE_CONTEXT);
        ddr2.write(SDTIM2, 0x7654_3210, LITTLE_CONTEXT);
        assert_eq!(
            ddr2.read(SDTIM1, LITTLE_CONTEXT),
            Some(0x1234_5678 & !(1 << 2))
        );
        assert_eq!(
            ddr2.read(SDTIM2, LITTLE_CONTEXT),
            Some(0x7654_3210 & 0x01ff_ffff)
        );
    }

    #[test]
    fn c6472_cas_latency_survives_timunlock_read_modify_write() {
        // SPRU894J §2.11.3/Table 20: CL changes even when TIMUNLOCK is set in this write.
        for unlock in [0, SDCFG_TIMUNLOCK] {
            let mut ddr2 = Ddr2::c6472();
            ddr2.write(
                SDCFG,
                (0x00200420 & !0xe00) | unlock | (4 << 9),
                LITTLE_CONTEXT,
            );
            let value = ddr2.read(SDCFG, LITTLE_CONTEXT).unwrap();
            ddr2.write(SDCFG, value & !SDCFG_TIMUNLOCK, LITTLE_CONTEXT);
            assert_eq!(ddr2.read(SDCFG, LITTLE_CONTEXT), Some(0x00200820));
        }
    }
}
