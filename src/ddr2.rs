//! DDR2 memory controller register contracts for the C6455 and C6472.
//!
//! SPRU970G section 4: the module identifies itself, reports whether its interface logic is out
//! of reset and which endianness it inherited, and holds the SDRAM configuration, refresh and
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
/// SDCFG locks: BOOT_UNLOCK guards bits 22-16 and TIMUNLOCK the timing registers.
const SDCFG_BOOT_UNLOCK: u32 = 1 << 23;
const SDCFG_TIMUNLOCK: u32 = 1 << 15;
const SDCFG_CL: u32 = 0x7 << 9;
const C6472_SDCFG_ENABLES: u32 = (1 << 20) | (1 << 17) | (1 << 16);
/// A refresh rate below 100h is replaced by twice the refresh cycle time SDTIM1 holds
/// (SPRU970G Table 21).
const REFRESH_FLOOR: u32 = 0x0100;

/// C6472/TCI648x reset values from SPRU894J Figures 19-26. Its reset configuration differs
/// materially from the C6455 values, including the NM/CL/IBANK/PAGESIZE fields and EBANK support.
const C6472_SDCFG_RESET: u32 = (1 << 21) | (1 << 19) | (2 << 9) | (2 << 4);
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DeviceKind {
    C6455,
    C6472,
}

pub struct Ddr2 {
    device: DeviceKind,
    /// C6472 IFRDY is independent of DMCCTL.RL. The untimed model marks the interface ready
    /// once software enables SDRAM, DDR and DDR2 in SDCFG; the register write stands in for the
    /// completion of the documented initialization sequence.
    interface_ready: bool,
    config: u32,
    refresh: u32,
    timing1: u32,
    timing2: u32,
    priority: u32,
    control: u32,
}

impl Default for Ddr2 {
    fn default() -> Self {
        Self::new(DeviceKind::C6455)
    }
}

impl Ddr2 {
    /// The C6472 DDR controller uses SPRU894J's contract; `Default` remains the historical
    /// C6455/SPRU970G controller so existing C6455 callers retain their register behavior.
    pub(crate) fn c6472() -> Self {
        Self::new(DeviceKind::C6472)
    }

    fn new(device: DeviceKind) -> Self {
        let (config, refresh, timing1, timing2) = match device {
            DeviceKind::C6455 => (SDCFG_RESET, SDRFC_RESET, SDTIM1_RESET, SDTIM2_RESET),
            DeviceKind::C6472 => (
                C6472_SDCFG_RESET,
                C6472_SDRFC_RESET,
                C6472_SDTIM1_RESET,
                C6472_SDTIM2_RESET,
            ),
        };
        Self {
            device,
            interface_ready: false,
            config,
            refresh,
            timing1,
            timing2,
            priority: BPRIO_RESET,
            control: DMCCTL_RESET,
        }
    }
}

impl SlavePort for Ddr2 {
    fn read(&mut self, address: u32, context: PortContext) -> Option<u32> {
        let value = match address {
            MIDR => match self.device {
                DeviceKind::C6455 => MIDR_VALUE,
                DeviceKind::C6472 => C6472_MIDR_VALUE,
            },
            DMCSTAT => {
                // C6455 exposes IFRESET in DMCCTL; C6472 reserves those DMCCTL bits and reports
                // IFRDY separately after its SDCFG-controlled initialization.
                DMCSTAT_ONES
                    | (u32::from(context.endian == Endian::Big) * DMCSTAT_BIG_ENDIAN)
                    | (u32::from(match self.device {
                        DeviceKind::C6455 => self.control & DMCCTL_IFRESET == 0,
                        DeviceKind::C6472 => self.interface_ready,
                    }) * DMCSTAT_IFRDY)
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
                if self.device == DeviceKind::C6472 && self.config & SDCFG_TIMUNLOCK == 0 {
                    // SPRU894J Table 20 explicitly gates CL with TIMUNLOCK. The prose above
                    // Table 20 says TIMUNLOCK=0 is unlocked, but the field description and
                    // initialization sequence both define 1 as unlocked; follow those specifics.
                    writable &= !SDCFG_CL;
                }
                if self.device == DeviceKind::C6472 {
                    // SPRU894J Table 20 adds EBANK at bit 3 and DDR2_TERM[1] at bit 27.
                    writable |= 1 << 3;
                }
                if value & SDCFG_BOOT_UNLOCK != 0 {
                    writable |= match self.device {
                        DeviceKind::C6455 => 1 << 18,
                        DeviceKind::C6472 => (1 << 27) | (0x7f << 16),
                    };
                }
                self.config = (self.config & !writable) | (value & writable);
                if self.device == DeviceKind::C6472 {
                    // With no DDR clock/sequencer timing in this model, complete the documented
                    // reset/configuration initialization at the enabling SDCFG write.
                    self.interface_ready = self.config & C6472_SDCFG_ENABLES == C6472_SDCFG_ENABLES;
                }
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
                let writable = match self.device {
                    DeviceKind::C6455 => 0xffff,
                    // SPRU894J Table 25: C6472 exposes RL in bits 2:0; all other bits retain
                    // the reset value and must not be treated as a software reset control.
                    DeviceKind::C6472 => 0x7,
                };
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

    #[test]
    fn c6472_has_its_own_spru894j_reset_contract() {
        // SPRU894J Figures 19-26 and Tables 18-25, C6472 row where the guide distinguishes
        // device defaults. Keep this separate from the C6455/SPRU970G reset fixture above.
        let mut ddr2 = Ddr2::c6472();
        assert_eq!(ddr2.read(MIDR, LITTLE_CONTEXT), Some(0x0034_0b1f));
        assert_eq!(ddr2.read(DMCSTAT, LITTLE_CONTEXT), Some(1 << 30));
        assert_eq!(ddr2.read(SDCFG, LITTLE_CONTEXT), Some(C6472_SDCFG_RESET));
        assert_eq!(ddr2.read(SDRFC, LITTLE_CONTEXT), Some(0x1388));
        assert_eq!(ddr2.read(SDTIM1, LITTLE_CONTEXT), Some(C6472_SDTIM1_RESET));
        assert_eq!(ddr2.read(SDTIM2, LITTLE_CONTEXT), Some(C6472_SDTIM2_RESET));
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

        // BOOT_UNLOCK gates bits 27 and 22:16, including the DDR enable and drive fields.
        ddr2.write(SDCFG, initial | (1 << 27) | (1 << 18), LITTLE_CONTEXT);
        assert_eq!(ddr2.read(SDCFG, LITTLE_CONTEXT), Some(initial));
        ddr2.write(
            SDCFG,
            initial | SDCFG_BOOT_UNLOCK | (1 << 27) | (1 << 18),
            LITTLE_CONTEXT,
        );
        assert_eq!(
            ddr2.read(SDCFG, LITTLE_CONTEXT),
            Some(initial | SDCFG_BOOT_UNLOCK | (1 << 27) | (1 << 18))
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
    fn c6472_cas_latency_is_locked_until_timunlock_was_set() {
        let mut ddr2 = Ddr2::c6472();
        let initial = ddr2.read(SDCFG, LITTLE_CONTEXT).unwrap();
        ddr2.write(SDCFG, (initial & !SDCFG_CL) | (5 << 9), LITTLE_CONTEXT);
        assert_eq!(ddr2.read(SDCFG, LITTLE_CONTEXT), Some(initial));

        ddr2.write(SDCFG, initial | SDCFG_TIMUNLOCK, LITTLE_CONTEXT);
        ddr2.write(
            SDCFG,
            (initial & !SDCFG_CL) | SDCFG_TIMUNLOCK | (5 << 9),
            LITTLE_CONTEXT,
        );
        assert_eq!(
            ddr2.read(SDCFG, LITTLE_CONTEXT),
            Some((initial & !SDCFG_CL) | SDCFG_TIMUNLOCK | (5 << 9))
        );
    }

    #[test]
    fn c6472_ifrdy_follows_the_untimed_sdcfg_initialization() {
        let mut ddr2 = Ddr2::c6472();
        assert_eq!(ddr2.read(DMCSTAT, LITTLE_CONTEXT), Some(1 << 30));

        let partial = C6472_SDCFG_RESET | SDCFG_BOOT_UNLOCK | (1 << 17) | (1 << 16);
        ddr2.write(SDCFG, partial, LITTLE_CONTEXT);
        assert_eq!(ddr2.read(DMCSTAT, LITTLE_CONTEXT), Some(1 << 30));

        ddr2.write(SDCFG, partial | (1 << 20), LITTLE_CONTEXT);
        assert_eq!(
            ddr2.read(DMCSTAT, LITTLE_CONTEXT),
            Some((1 << 30) | DMCSTAT_IFRDY)
        );

        // DMCCTL.RL is writable on C6472, but reserved bit 5 does not release IFRDY.
        ddr2.write(DMCCTL, 0, LITTLE_CONTEXT);
        assert_eq!(
            ddr2.read(DMCSTAT, LITTLE_CONTEXT),
            Some((1 << 30) | DMCSTAT_IFRDY)
        );
    }
}
