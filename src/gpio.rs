//! The C6455's general-purpose input/output peripheral.
//!
//! SPRU724A: sixteen pins, each an input or an output, with edge-detection logic that raises a
//! CPU interrupt and an EDMA3 synchronisation event per pin. The edge detector runs on the
//! peripheral clock, which SPRU724A Figure 2 gives as the CPU clock divided by six, and it sees
//! output pins too, so software toggling OUT_DATA generates events exactly as an external
//! signal would.
use crate::port::{PortContext, PortWrite, SlavePort};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const BASE: u32 = 0x02b0_0000;
const BINTEN: u32 = BASE + 0x08;
const DIR: u32 = BASE + 0x10;
const OUT_DATA: u32 = BASE + 0x14;
const SET_DATA: u32 = BASE + 0x18;
const CLR_DATA: u32 = BASE + 0x1c;
const IN_DATA: u32 = BASE + 0x20;
const SET_RIS_TRIG: u32 = BASE + 0x24;
const CLR_RIS_TRIG: u32 = BASE + 0x28;
const SET_FAL_TRIG: u32 = BASE + 0x2c;
const CLR_FAL_TRIG: u32 = BASE + 0x30;

const PINS: u32 = 0xffff;
#[cfg(test)]
const CLOCK_DIVIDER: u32 = 6;

pub struct Gpio {
    binten: u32,
    /// A set bit makes the pin an input, which is the reset state (SPRU724A Table 4).
    dir: u32,
    out_data: u32,
    /// RIS_TRIG and FAL_TRIG are internal registers the CPU only reaches through the four
    /// set/clear windows (SPRU724A §3).
    rising: u32,
    falling: u32,
    /// The pin state the edge detector saw on the previous peripheral clock.
    sampled: u32,
    #[cfg(test)]
    divide: u32,
    /// What an external driver holds on each pin. It reaches only the pins configured as
    /// inputs: an output pin is driven by the device, and that is what IN_DATA reports.
    input: u32,
}

impl Default for Gpio {
    fn default() -> Self {
        Self {
            binten: 0,
            dir: PINS,
            out_data: 0,
            rising: 0,
            falling: 0,
            sampled: 0,
            #[cfg(test)]
            divide: 0,
            input: 0,
        }
    }
}

impl SlavePort for Gpio {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let value = match address {
            BINTEN => self.binten,
            DIR => self.dir,
            // Reading either data window returns OUT_DATA, not the pin state.
            OUT_DATA | SET_DATA | CLR_DATA => self.out_data,
            IN_DATA => self.pins(),
            SET_RIS_TRIG | CLR_RIS_TRIG => self.rising,
            SET_FAL_TRIG | CLR_FAL_TRIG => self.falling,
            _ => return None,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        let value = value & PINS;
        match address {
            BINTEN => self.binten = value & 1,
            DIR => self.dir = value,
            OUT_DATA => self.out_data = value,
            SET_DATA => self.out_data |= value,
            CLR_DATA => self.out_data &= !value,
            IN_DATA => {}
            SET_RIS_TRIG => self.rising |= value,
            CLR_RIS_TRIG => self.rising &= !value,
            SET_FAL_TRIG => self.falling |= value,
            CLR_FAL_TRIG => self.falling &= !value,
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

impl Gpio {
    /// The pin state: an output pin carries OUT_DATA, and an input pin has no driver in this
    /// model, so it reads low.
    /// The level on the pins: OUT_DATA where the device drives, and whatever the outside world
    /// holds where the pin is an input (SPRU724A section 3 - IN_DATA reads the pin).
    fn pins(&self) -> u32 {
        (self.out_data & !self.dir) | (self.input & self.dir)
    }

    /// An external driver's levels on the sixteen pins. Only the input pins take them.
    #[cfg(test)]
    pub fn drive_inputs(&mut self, value: u32) {
        self.input = value & PINS;
    }

    #[cfg(test)]
    pub fn tick(&mut self) -> u32 {
        self.divide += 1;
        if self.divide == CLOCK_DIVIDER {
            self.divide = 0;
            self.tick_sysclk3()
        } else {
            0
        }
    }

    pub(crate) fn tick_sysclk3(&mut self) -> u32 {
        let level = self.pins();
        let changed = level ^ self.sampled;
        self.sampled = level;
        if self.binten & 1 == 0 {
            return 0;
        }
        (changed & level & self.rising) | (changed & !level & self.falling)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    fn clock(gpio: &mut Gpio) -> u32 {
        let mut events = 0;
        for _ in 0..CLOCK_DIVIDER {
            events |= gpio.tick();
        }
        events
    }

    #[test]
    fn an_output_pin_drives_its_own_edge_events() {
        // SPRU724A §3: software toggling an output generates the same interrupt and EDMA event
        // an external signal would, and Table 1 selects which edge counts.
        let mut gpio = Gpio::default();
        assert_eq!(gpio.read(DIR, TEST_CONTEXT), Some(0xffff));
        gpio.write(DIR, 0xfffe, TEST_CONTEXT);
        gpio.write(BINTEN, 1, TEST_CONTEXT);
        gpio.write(SET_RIS_TRIG, 1, TEST_CONTEXT);
        gpio.write(SET_DATA, 1, TEST_CONTEXT);
        assert_eq!(gpio.read(OUT_DATA, TEST_CONTEXT), Some(1));
        assert_eq!(gpio.read(IN_DATA, TEST_CONTEXT), Some(1));
        assert_eq!(clock(&mut gpio), 1);
        // A level that does not change raises nothing, and the falling edge is not enabled.
        assert_eq!(clock(&mut gpio), 0);
        gpio.write(CLR_DATA, 1, TEST_CONTEXT);
        assert_eq!(clock(&mut gpio), 0);
        // Enabling the falling edge as well reports both directions.
        gpio.write(SET_FAL_TRIG, 1, TEST_CONTEXT);
        gpio.write(SET_DATA, 1, TEST_CONTEXT);
        assert_eq!(clock(&mut gpio), 1);
        gpio.write(CLR_DATA, 1, TEST_CONTEXT);
        assert_eq!(clock(&mut gpio), 1);
    }

    #[test]
    fn the_bank_enable_gates_every_pin() {
        // BINTEN.EN has to be set for any pin to be an interrupt or event source (SPRU724A 5.1).
        let mut gpio = Gpio::default();
        gpio.write(DIR, 0, TEST_CONTEXT);
        gpio.write(SET_RIS_TRIG, 0xffff, TEST_CONTEXT);
        gpio.write(SET_DATA, 0x8000, TEST_CONTEXT);
        assert_eq!(clock(&mut gpio), 0);
        gpio.write(BINTEN, 1, TEST_CONTEXT);
        // The detector only sees the level it samples, so the pin has to go low for a whole
        // peripheral clock before the next rising edge exists.
        gpio.write(CLR_DATA, 0x8000, TEST_CONTEXT);
        assert_eq!(clock(&mut gpio), 0);
        gpio.write(SET_DATA, 0x8000, TEST_CONTEXT);
        assert_eq!(clock(&mut gpio), 0x8000);
        // An input pin has no driver here, so OUT_DATA no longer reaches the detector.
        gpio.write(DIR, 0xffff, TEST_CONTEXT);
        assert_eq!(clock(&mut gpio), 0);
        assert_eq!(gpio.read(IN_DATA, TEST_CONTEXT), Some(0));
    }
}
