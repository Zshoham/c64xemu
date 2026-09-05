//! The C6455's chip-level and device-state control registers.
//!
//! SPRS276M sections 3.4 to 3.6 and 4.2: DEVSTAT reports what the configuration pins latched at
//! reset, JTAGID identifies the silicon, PRI_ALLOC sets the bus priority of the masters that do
//! not own their own priority register, and the device-state block turns peripherals on and off
//! through PERLOCK/PERCFG0 with PERSTAT0/PERSTAT1 reporting the transition.
//!
//! Two device-level choices this model makes, both documented rather than invented: the silicon
//! is revision 3.1, the last one the errata lists (SPRZ234R Table 3), and the configuration pins
//! are the ones a board that populates the memory map this model backs would strap - EMIFA and
//! the DDR2 controller enabled, everything else at its internal-resistor default (SPRS276M Figure
//! 3-10). LENDIAN is not a strap here: it follows the endianness of the loaded image, which is
//! real state the model has.
//!
//! Enabling or disabling a peripheral does not gate its registers here: PERCFG0 and the status
//! registers report the configuration, and every modelled peripheral answers regardless.
//! SPRS276M Table 3-4 says what the disabled state is - "Peripheral is held in reset and clock is
//! turned off" - and section 3.2 that "All peripherals on the C6455 device are disabled by
//! default, except when used for boot, and must be enabled through software before being used",
//! but the chapter never says what an access to a disabled peripheral's registers does, and
//! gating on it would mean every target in the suite growing the enable sequence first.
//! EMACCFG's RMII_RST reaches nothing for a different reason: it resets "the RMII logic of the
//! EMAC" (section 3.4.5), and MACSEL[1:0] straps this device to the MII interface (Table 3-1),
//! so there is no RMII logic to hold. The enable that does gate this device's wire is
//! MACCONTROL.GMIIEN, in `emac.rs`.
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

const CHIP_BASE: u32 = 0x02a8_0000;
const DEVSTAT: u32 = CHIP_BASE;
const PRI_ALLOC: u32 = CHIP_BASE + 0x04;
const JTAGID: u32 = CHIP_BASE + 0x08;

const STATE_BASE: u32 = 0x02ac_0000;
const PERLOCK: u32 = STATE_BASE + 0x04;
const PERCFG0: u32 = STATE_BASE + 0x08;
const PERSTAT0: u32 = STATE_BASE + 0x14;
const PERSTAT1: u32 = STATE_BASE + 0x18;
const EMACCFG: u32 = STATE_BASE + 0x20;
const PERCFG1: u32 = STATE_BASE + 0x2c;
const EMUBUFPD: u32 = STATE_BASE + 0x54;

/// EMIFA_EN and DDR2_EN strapped high, the two reserved bits that always read one, and the
/// little-endian default (SPRS276M Figure 3-10).
const DEVSTAT_FIXED: u32 = (1 << 22) | (1 << 21) | (1 << 16) | (1 << 8);
const DEVSTAT_LENDIAN: u32 = 1 << 6;
/// Silicon revision 3.1: variant 0100b over the C6455 part number and manufacturer code
/// (SPRS276M Figure 3-11, SPRZ234R Table 3).
const JTAGID_VALUE: u32 = 0x4008_a02f;

/// SRIO 001, HOST 010 and EMAC 001, with the read-only 100 the reserved field keeps
/// (SPRS276M Figure 4-3).
const PRI_ALLOC_RESERVED: u32 = 4 << 6;
const PRI_ALLOC_WRITABLE: u32 = (0x7 << 9) | (0x7 << 3) | 0x7;
const PRI_ALLOC_RESET: u32 = PRI_ALLOC_RESERVED | (1 << 9) | (2 << 3) | 1;

const PERLOCK_RESET: u32 = 0xf0f0_f0f0;
/// The one value that opens the PERCFG0 write window (SPRS276M Table 3-6).
const PERLOCK_KEY: u32 = 0x0f0a_0b00;
/// The window, and the time a peripheral takes to change state, are both 16 SYSCLK3 cycles
/// (SPRS276M sections 3.4.1 to 3.4.3).
const LOCK_WINDOW: u32 = 16;
const TRANSITION: u32 = 16;
#[cfg(test)]
const CLOCK_DIVIDER: u32 = 6;

/// The mode control bits PERCFG0 actually owns (SPRS276M Figure 3-4).
const PERCFG0_WRITABLE: u32 = (0x3 << 30)
    | (1 << 22)
    | (1 << 20)
    | (1 << 18)
    | (1 << 16)
    | (1 << 14)
    | (1 << 12)
    | (1 << 10)
    | (1 << 8)
    | (1 << 6)
    | (1 << 4)
    | (1 << 2)
    | 1;
/// EMIFACTL and DDR2CTL, both set-only (SPRS276M Table 3-8).
const PERCFG1_WRITABLE: u32 = 0x3;

/// The reserved field above RMII_RST reads 0001b and the reset holds RMII in reset
/// (SPRS276M Figure 3-8).
const EMACCFG_RESET: u32 = (1 << 19) | (1 << 18);
const EMACCFG_WRITABLE: u32 = 1 << 18;

/// Each status field is three bits: 000 disabled, 001 enabled, 100 disabling, 101 enabling
/// (SPRS276M Tables 3-9 and 3-10).
const STATUS_ENABLED: u32 = 0b001;
const STATUS_DISABLING: u32 = 0b100;
const STATUS_ENABLING: u32 = 0b101;

/// PERSTAT0 field shift paired with the PERCFG0 control bit it follows.
const PERSTAT0_FIELDS: [(u32, u32); 10] = [
    (27, 18), // HPI
    (24, 16), // McBSP1
    (21, 14), // McBSP0
    (18, 12), // I2C
    (15, 10), // GPIO
    (12, 8),  // Timer 1
    (9, 6),   // Timer 0
    (6, 4),   // EMAC/MDIO
    (3, 2),   // VCP
    (0, 0),   // TCP
];
/// PERSTAT1 carries UTOPIA and PCI only.
const PERSTAT1_FIELDS: [(u32, u32); 2] = [(3, 22), (0, 20)];

pub struct DeviceConfig {
    priority: u32,
    lock: u32,
    /// The requested configuration, and the one the peripherals have reached.
    config: u32,
    settled: u32,
    config1: u32,
    emac: u32,
    emulator_buffers: u32,
    /// SYSCLK3 cycles left in the PERLOCK window, and whether it still holds a write.
    lock_window: u32,
    lock_armed: bool,
    /// SYSCLK3 cycles left before the requested configuration takes effect.
    transition: u32,
    #[cfg(test)]
    divide: u32,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            priority: PRI_ALLOC_RESET,
            lock: PERLOCK_RESET,
            config: 0,
            settled: 0,
            config1: 0,
            emac: EMACCFG_RESET,
            emulator_buffers: 0,
            lock_window: 0,
            lock_armed: false,
            transition: 0,
            #[cfg(test)]
            divide: 0,
        }
    }
}

impl SlavePort for DeviceConfig {
    fn read(&mut self, address: u32, context: PortContext) -> Option<u32> {
        let value = match address {
            DEVSTAT => {
                DEVSTAT_FIXED
                    | if context.endian == Endian::Big {
                        0
                    } else {
                        DEVSTAT_LENDIAN
                    }
            }
            PRI_ALLOC => self.priority,
            JTAGID => JTAGID_VALUE,
            PERLOCK => self.lock,
            PERCFG0 => self.config,
            PERCFG1 => self.config1,
            PERSTAT0 => self.status(&PERSTAT0_FIELDS),
            PERSTAT1 => self.status(&PERSTAT1_FIELDS),
            EMACCFG => self.emac,
            EMUBUFPD => self.emulator_buffers,
            _ => return None,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        match address {
            DEVSTAT | JTAGID => {}
            PRI_ALLOC => {
                self.priority = PRI_ALLOC_RESERVED | (value & PRI_ALLOC_WRITABLE);
            }
            PERLOCK => {
                self.lock = value;
                if value == PERLOCK_KEY {
                    self.lock_window = LOCK_WINDOW;
                    self.lock_armed = true;
                }
            }
            PERCFG0 => {
                // One write is allowed per key, and only inside the window it opened.
                if !self.lock_armed {
                    return PortWrite::Done;
                }
                self.lock_armed = false;
                let requested = value & PERCFG0_WRITABLE;
                if requested != self.config {
                    self.config = requested;
                    self.transition = TRANSITION;
                }
            }
            // EMIFACTL and DDR2CTL latch high and cannot be cleared again.
            PERCFG1 => self.config1 |= value & PERCFG1_WRITABLE,
            PERSTAT0 | PERSTAT1 => {}
            EMACCFG => self.emac = (EMACCFG_RESET & !EMACCFG_WRITABLE) | (value & EMACCFG_WRITABLE),
            EMUBUFPD => self.emulator_buffers = value & 1,
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

impl DeviceConfig {
    #[cfg(test)]
    pub fn tick(&mut self) {
        self.divide += 1;
        if self.divide == CLOCK_DIVIDER {
            self.divide = 0;
            self.tick_sysclk3();
        }
    }

    pub(crate) fn tick_sysclk3(&mut self) {
        if self.lock_window > 0 {
            self.lock_window -= 1;
            if self.lock_window == 0 {
                self.lock_armed = false;
            }
        }
        if self.transition > 0 {
            self.transition -= 1;
            if self.transition == 0 {
                self.settled = self.config;
            }
        }
    }

    fn status(&self, fields: &[(u32, u32)]) -> u32 {
        let mut value = 0;
        for &(shift, control) in fields {
            let requested = self.config & (1 << control) != 0;
            let reached = self.settled & (1 << control) != 0;
            let code = match (requested, reached) {
                (true, true) => STATUS_ENABLED,
                (true, false) => STATUS_ENABLING,
                (false, true) => STATUS_DISABLING,
                (false, false) => continue,
            };
            value |= code << shift;
        }
        value
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    fn sysclk3(config: &mut DeviceConfig, cycles: u32) {
        for _ in 0..cycles * CLOCK_DIVIDER {
            config.tick();
        }
    }

    #[test]
    fn the_identity_registers_are_the_documented_ones() {
        // SPRS276M Figures 3-10, 3-11 and 4-3 with SPRZ234R Table 3 for the variant.
        let mut config = DeviceConfig::default();
        assert_eq!(config.read(DEVSTAT, LITTLE_CONTEXT), Some(0x0061_0140));
        assert_eq!(config.read(DEVSTAT, BIG_CONTEXT), Some(0x0061_0100));
        assert_eq!(config.read(JTAGID, LITTLE_CONTEXT), Some(0x4008_a02f));
        assert_eq!(config.read(PRI_ALLOC, LITTLE_CONTEXT), Some(0x0000_0311));
        assert_eq!(config.read(PERLOCK, LITTLE_CONTEXT), Some(0xf0f0_f0f0));
        assert_eq!(config.read(EMACCFG, LITTLE_CONTEXT), Some(0x000c_0000));
    }

    #[test]
    fn percfg0_needs_the_key_and_takes_one_write() {
        // SPRS276M Table 3-6: the key opens a single write inside sixteen SYSCLK3 cycles.
        let mut config = DeviceConfig::default();
        config.write(PERCFG0, 1 << 10, LITTLE_CONTEXT);
        assert_eq!(config.read(PERCFG0, LITTLE_CONTEXT), Some(0));
        config.write(PERLOCK, PERLOCK_KEY, LITTLE_CONTEXT);
        config.write(PERCFG0, (1 << 10) | (1 << 6), LITTLE_CONTEXT);
        assert_eq!(
            config.read(PERCFG0, LITTLE_CONTEXT),
            Some((1 << 10) | (1 << 6))
        );
        // The second write finds the window already spent.
        config.write(PERCFG0, 1 << 12, LITTLE_CONTEXT);
        assert_eq!(
            config.read(PERCFG0, LITTLE_CONTEXT),
            Some((1 << 10) | (1 << 6))
        );
        // A key whose window elapses without a write leaves PERCFG0 locked again.
        config.write(PERLOCK, PERLOCK_KEY, LITTLE_CONTEXT);
        sysclk3(&mut config, LOCK_WINDOW);
        config.write(PERCFG0, 1 << 12, LITTLE_CONTEXT);
        assert_eq!(
            config.read(PERCFG0, LITTLE_CONTEXT),
            Some((1 << 10) | (1 << 6))
        );
    }

    #[test]
    fn the_status_registers_show_the_transition() {
        // SPRS276M Tables 3-9 and 3-10: GPIO and Timer 0 report enabling, then enabled once the
        // sixteen SYSCLK3 cycles have passed, and disabling on the way back.
        let mut config = DeviceConfig::default();
        config.write(PERLOCK, PERLOCK_KEY, LITTLE_CONTEXT);
        config.write(PERCFG0, (1 << 10) | (1 << 6) | (1 << 22), LITTLE_CONTEXT);
        assert_eq!(
            config.read(PERSTAT0, LITTLE_CONTEXT),
            Some((STATUS_ENABLING << 15) | (STATUS_ENABLING << 9))
        );
        assert_eq!(
            config.read(PERSTAT1, LITTLE_CONTEXT),
            Some(STATUS_ENABLING << 3)
        );
        sysclk3(&mut config, TRANSITION - 1);
        assert_eq!(
            config.read(PERSTAT0, LITTLE_CONTEXT),
            Some((STATUS_ENABLING << 15) | (STATUS_ENABLING << 9))
        );
        sysclk3(&mut config, 1);
        assert_eq!(
            config.read(PERSTAT0, LITTLE_CONTEXT),
            Some((STATUS_ENABLED << 15) | (STATUS_ENABLED << 9))
        );
        config.write(PERLOCK, PERLOCK_KEY, LITTLE_CONTEXT);
        config.write(PERCFG0, 0, LITTLE_CONTEXT);
        assert_eq!(
            config.read(PERSTAT0, LITTLE_CONTEXT),
            Some((STATUS_DISABLING << 15) | (STATUS_DISABLING << 9))
        );
        sysclk3(&mut config, TRANSITION);
        assert_eq!(config.read(PERSTAT0, LITTLE_CONTEXT), Some(0));
        assert_eq!(config.read(PERSTAT1, LITTLE_CONTEXT), Some(0));
    }

    #[test]
    fn percfg1_and_the_small_registers_latch_the_documented_way() {
        // SPRS276M Table 3-8: EMIFACTL and DDR2CTL cannot be cleared once set.
        let mut config = DeviceConfig::default();
        config.write(PERCFG1, 1, LITTLE_CONTEXT);
        assert_eq!(config.read(PERCFG1, LITTLE_CONTEXT), Some(1));
        config.write(PERCFG1, 0, LITTLE_CONTEXT);
        assert_eq!(config.read(PERCFG1, LITTLE_CONTEXT), Some(1));
        config.write(PERCFG1, 0xffff_ffff, LITTLE_CONTEXT);
        assert_eq!(config.read(PERCFG1, LITTLE_CONTEXT), Some(3));
        // EMACCFG only owns RMII_RST; PRI_ALLOC keeps its read-only reserved field.
        config.write(EMACCFG, 0, LITTLE_CONTEXT);
        assert_eq!(config.read(EMACCFG, LITTLE_CONTEXT), Some(1 << 19));
        config.write(PRI_ALLOC, 0xffff_ffff, LITTLE_CONTEXT);
        assert_eq!(config.read(PRI_ALLOC, LITTLE_CONTEXT), Some(0x0000_0f3f));
        config.write(EMUBUFPD, 0xffff_ffff, LITTLE_CONTEXT);
        assert_eq!(config.read(EMUBUFPD, LITTLE_CONTEXT), Some(1));
    }
}
