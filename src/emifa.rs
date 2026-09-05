//! The C6455's asynchronous external memory interface registers.
//!
//! SPRU971E section 12: the module identifies itself, reports the endianness it inherited, and
//! holds the burst priority, the four chip-enable configuration registers, the asynchronous wait
//! cycle configuration and the asynchronous time-out interrupt. The data path is plain memory in
//! this model - EMIFA strobe timing is not simulated - and there is no external device to hold
//! ARDY, so the time-out the interrupt reports has no injectable source; the mask registers and
//! their write-one-to-clear pairing are still exactly as documented.
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

const BASE: u32 = 0x7000_0000;
const MIDR: u32 = BASE;
const STAT: u32 = BASE + 0x04;
const BPRIO: u32 = BASE + 0x20;
const CECFG_BASE: u32 = BASE + 0x80;
const AWCC: u32 = BASE + 0xa0;
const INTRAW: u32 = BASE + 0xc0;
const INTMSK: u32 = BASE + 0xc4;
const INTMSKSET: u32 = BASE + 0xc8;
const INTMSKCLR: u32 = BASE + 0xcc;

const CHIP_ENABLES: usize = 4;
/// Module id 32h, major revision 3, minor revision 11h (SPRU971E Figure 26).
const MIDR_VALUE: u32 = (0x0032 << 16) | (0x03 << 8) | 0x11;
/// STAT carries the endianness in bit 31 and reads one in bit 30 (SPRU971E Figure 27).
const STAT_ONES: u32 = 1 << 30;
const STAT_BIG_ENDIAN: u32 = 1 << 31;
/// The asynchronous chip-enable configuration resets to the slowest strobes (Figure 29).
const CECFG_RESET: u32 =
    (0xf << 24) | (0x3f << 18) | (0x7 << 15) | (0xf << 11) | (0x3f << 5) | (0x7 << 2);
/// AWCC resets with an active-high ready pin, three turnaround cycles and 0x80 wait periods.
const AWCC_RESET: u32 = (1 << 30) | (0x3 << 8) | 0x80;
const AWCC_MASK: u32 = (1 << 30) | (0x7 << 8) | 0xff;
/// The one interrupt this interface raises: an asynchronous access that timed out.
const INT_AT: u32 = 1 << 0;

pub struct Emifa {
    priority: u32,
    chip_enable: [u32; CHIP_ENABLES],
    wait_config: u32,
    timeout: bool,
    timeout_enabled: bool,
}

impl Default for Emifa {
    fn default() -> Self {
        Self {
            priority: 0xff,
            chip_enable: [CECFG_RESET; CHIP_ENABLES],
            wait_config: AWCC_RESET,
            timeout: false,
            timeout_enabled: false,
        }
    }
}

impl SlavePort for Emifa {
    fn read(&mut self, address: u32, context: PortContext) -> Option<u32> {
        if let Some(index) = chip_enable(address) {
            return Some(self.chip_enable[index]);
        }
        let value = match address {
            MIDR => MIDR_VALUE,
            STAT => STAT_ONES | (u32::from(context.endian == Endian::Big) * STAT_BIG_ENDIAN),
            BPRIO => self.priority,
            AWCC => self.wait_config,
            INTRAW => u32::from(self.timeout) * INT_AT,
            // The masked view only shows the interrupt once it is enabled.
            INTMSK => u32::from(self.timeout && self.timeout_enabled) * INT_AT,
            INTMSKSET | INTMSKCLR => u32::from(self.timeout_enabled) * INT_AT,
            _ => return None,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        if let Some(index) = chip_enable(address) {
            self.chip_enable[index] = value;
            return PortWrite::Done;
        }
        match address {
            MIDR | STAT => {}
            BPRIO => self.priority = value & 0xff,
            AWCC => self.wait_config = value & AWCC_MASK,
            // A write of one to either interrupt view clears the time-out in both of them.
            INTRAW | INTMSK => {
                if value & INT_AT != 0 {
                    self.timeout = false;
                }
            }
            INTMSKSET => {
                if value & INT_AT != 0 {
                    self.timeout_enabled = true;
                }
            }
            INTMSKCLR => {
                if value & INT_AT != 0 {
                    self.timeout_enabled = false;
                }
            }
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

fn chip_enable(address: u32) -> Option<usize> {
    if address % 4 != 0 || address < CECFG_BASE {
        return None;
    }
    let index = ((address - CECFG_BASE) / 4) as usize;
    (index < CHIP_ENABLES).then_some(index)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    #[test]
    fn the_reset_values_are_the_ones_the_manual_gives() {
        // SPRU971E Figures 26 to 31: the module identity, the status bit that always reads one,
        // the burst priority that starts with the feature disabled, the slowest asynchronous
        // strobes and the documented wait configuration.
        let mut emifa = Emifa::default();
        assert_eq!(emifa.read(MIDR, LITTLE_CONTEXT), Some(0x0032_0311));
        assert_eq!(emifa.read(STAT, LITTLE_CONTEXT), Some(1 << 30));
        assert_eq!(emifa.read(STAT, BIG_CONTEXT), Some((1 << 31) | (1 << 30)));
        assert_eq!(emifa.read(BPRIO, LITTLE_CONTEXT), Some(0xff));
        assert_eq!(emifa.read(CECFG_BASE, LITTLE_CONTEXT), Some(0x0fff_fffc));
        assert_eq!(
            emifa.read(CECFG_BASE + 12, LITTLE_CONTEXT),
            Some(0x0fff_fffc)
        );
        assert_eq!(emifa.read(AWCC, LITTLE_CONTEXT), Some(0x4000_0380));
        assert_eq!(emifa.read(INTRAW, LITTLE_CONTEXT), Some(0));
        assert_eq!(emifa.read(INTMSK, LITTLE_CONTEXT), Some(0));
    }

    #[test]
    fn the_interrupt_mask_pair_is_one_enable() {
        // SPRU971E sections 12.9 and 12.10: setting the mask through one register and clearing
        // it through the other move the same enable, and the masked view follows it.
        let mut emifa = Emifa::default();
        emifa.write(INTMSKSET, INT_AT, LITTLE_CONTEXT);
        assert_eq!(emifa.read(INTMSKSET, LITTLE_CONTEXT), Some(INT_AT));
        assert_eq!(emifa.read(INTMSKCLR, LITTLE_CONTEXT), Some(INT_AT));
        // No external device holds ARDY here, so the raw status stays clear.
        assert_eq!(emifa.read(INTMSK, LITTLE_CONTEXT), Some(0));
        emifa.write(INTMSKCLR, INT_AT, LITTLE_CONTEXT);
        assert_eq!(emifa.read(INTMSKSET, LITTLE_CONTEXT), Some(0));
        // The configuration registers keep whatever the driver programs.
        emifa.write(CECFG_BASE + 4, 0x8000_0003, LITTLE_CONTEXT);
        assert_eq!(
            emifa.read(CECFG_BASE + 4, LITTLE_CONTEXT),
            Some(0x8000_0003)
        );
        emifa.write(AWCC, 0xffff_ffff, LITTLE_CONTEXT);
        assert_eq!(emifa.read(AWCC, LITTLE_CONTEXT), Some(AWCC_MASK));
        emifa.write(BPRIO, 0xfe, LITTLE_CONTEXT);
        assert_eq!(emifa.read(BPRIO, LITTLE_CONTEXT), Some(0xfe));
    }
}
