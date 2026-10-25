//! The C6455's inter-integrated circuit (I2C) module.
//!
//! SPRU974A: a master/slave I2C controller with a data transmit register that feeds an internal
//! shift register, a receive register the shift register fills, a status register of write-one-
//! to-clear flags, a mask register, and a vector register that reports and clears the
//! highest-priority pending interrupt. Digital loopback mode is the one data path that is
//! self-contained: the manual states that data written to ICDXR arrives in ICDRR after
//! (prescaler + 1) x 8 DSP cycles by an internal path, so this model uses exactly that.
use crate::port::{PortContext, PortWrite, SlavePort};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

const BASE: u32 = 0x02b0_4000;
const ICOAR: u32 = BASE;
const ICIMR: u32 = BASE + 0x04;
const ICSTR: u32 = BASE + 0x08;
const ICCLKL: u32 = BASE + 0x0c;
const ICCLKH: u32 = BASE + 0x10;
const ICCNT: u32 = BASE + 0x14;
const ICDRR: u32 = BASE + 0x18;
const ICSAR: u32 = BASE + 0x1c;
const ICDXR: u32 = BASE + 0x20;
const ICMDR: u32 = BASE + 0x24;
const ICIVR: u32 = BASE + 0x28;
const ICEMDR: u32 = BASE + 0x2c;
const ICPSC: u32 = BASE + 0x30;

/// ICSTR flags (SPRU974A Table 7).
const AL: u32 = 1 << 0;
const NACK: u32 = 1 << 1;
const ARDY: u32 = 1 << 2;
const ICRRDY: u32 = 1 << 3;
const ICXRDY: u32 = 1 << 4;
const SCD: u32 = 1 << 5;
/// The general-call address bit; nothing addresses this module as a slave in this model, so it
/// only ever reads as zero, alongside the addressed-as-slave and slave-direction bits. The
/// slave modelled here sits on the bus and answers this module acting as the master.
const AD0: u32 = 1 << 8;
const AAS: u32 = 1 << 9;
const XSMT: u32 = 1 << 10;
const RSFULL: u32 = 1 << 11;
const BB: u32 = 1 << 12;
const NACKSNT: u32 = 1 << 13;
const SDIR: u32 = 1 << 14;
/// The flags software clears by writing a one (SPRU974A Figure 15).
const WRITE_ONE_TO_CLEAR: u32 = AL | NACK | ARDY | ICRRDY | ICXRDY | SCD | BB | NACKSNT | SDIR;
/// Reset leaves the transmitter empty and its register ready.
const STATUS_RESET: u32 = XSMT | ICXRDY;
const SLAVE_FLAGS: u32 = AD0 | AAS | SDIR;

/// ICMDR control bits (SPRU974A Table 14).
const MDR_NACKMOD: u32 = 1 << 15;
const MDR_STT: u32 = 1 << 13;
const MDR_STP: u32 = 1 << 11;
const MDR_MST: u32 = 1 << 10;
const MDR_TRX: u32 = 1 << 9;
const MDR_RM: u32 = 1 << 7;
const MDR_DLB: u32 = 1 << 6;
const MDR_IRS: u32 = 1 << 5;

/// The interrupt codes ICIVR reports, in the priority order the register lists them
/// (SPRU974A Table 17): the status flag, and its bit in ICIMR.
const VECTORS: [(u32, u32, u32); 7] = [
    (1, AL, 1 << 0),
    (2, NACK, 1 << 1),
    (3, ARDY, 1 << 2),
    (4, ICRRDY, 1 << 3),
    (5, ICXRDY, 1 << 4),
    (6, SCD, 1 << 5),
    (7, AAS, 1 << 6),
];

pub struct I2c {
    oar: u32,
    imr: u32,
    status: u32,
    clkl: u32,
    clkh: u32,
    cnt: u32,
    drr: u32,
    sar: u32,
    dxr: u32,
    mdr: u32,
    emdr: u32,
    psc: u32,
    /// The internal data counter ICCNT is copied into at a START (SPRU974A §3.5).
    counter: u32,
    /// The byte in the transmit shift register and the DSP cycles left before it lands.
    shifting: Option<u32>,
    delay: u32,
    /// True while a byte is being clocked in from a slave in master-receive mode.
    receiving: bool,
    /// A device on the bus, which is what makes a real transfer possible: it acknowledges its
    /// own address, keeps the bytes the master writes and hands back the bytes it was given to
    /// answer a master read.
    slave: Option<I2cSlave>,
}

/// How many bytes of internal address a serial EEPROM consumes before the data. SPRUEC6G
/// section 2.3.4 makes it a requirement of the board rather than a choice: "The memory device
/// uses two bytes for internal addressing."
const EEPROM_ADDRESS_BYTES: usize = 2;

/// A device attached to the bus at one address.
///
/// Two of them, because the two things this model needs a bus device for want different
/// answers. A case that checks one direction at a time wants a device it can hand a list of
/// bytes to and read back what the master wrote. The ROM's I2C boot wants the device SPRUEC6G
/// section 2.3.4 requires of a board: one that "uses two bytes for internal addressing" and
/// "has the capability to auto-increment its internal address counter such that the contents of
/// the memory device can be read sequentially".
#[derive(Clone)]
pub enum I2cDevice {
    /// Answers reads from a queue, oldest first, and keeps every byte written to it. An
    /// exhausted device answers 0xff, which is what an open bus reads as. Only a case puts one
    /// on the bus, so only a test build has it.
    #[cfg(test)]
    Queue {
        transmit: std::collections::VecDeque<u8>,
        received: Vec<u8>,
    },
    /// A serial EEPROM. The master writes two bytes of internal address, then reads
    /// sequentially from there; the address counter advances on every byte in either direction
    /// and wraps at the end of the device.
    Eeprom {
        contents: Vec<u8>,
        pointer: usize,
        /// How many bytes of a write are still being consumed as the internal address.
        address_left: usize,
    },
}

/// A device attached to the bus at one address.
#[derive(Clone)]
pub struct I2cSlave {
    pub address: u32,
    pub device: I2cDevice,
}

impl I2cSlave {
    /// The byte the device answers a master read with.
    fn read(&mut self) -> u8 {
        match &mut self.device {
            #[cfg(test)]
            I2cDevice::Queue { transmit, .. } => transmit.pop_front().unwrap_or(0xff),
            I2cDevice::Eeprom {
                contents, pointer, ..
            } => {
                if contents.is_empty() {
                    return 0xff;
                }
                let byte = contents[*pointer % contents.len()];
                *pointer = (*pointer + 1) % contents.len();
                byte
            }
        }
    }

    /// Take a byte the master wrote. An EEPROM's first two are its internal address.
    fn write(&mut self, byte: u8) {
        match &mut self.device {
            #[cfg(test)]
            I2cDevice::Queue { received, .. } => received.push(byte),
            I2cDevice::Eeprom {
                contents,
                pointer,
                address_left,
            } => {
                if *address_left > 0 {
                    *address_left -= 1;
                    *pointer = ((*pointer << 8) | usize::from(byte)) & 0xffff;
                    return;
                }
                if !contents.is_empty() {
                    let at = *pointer % contents.len();
                    contents[at] = byte;
                    *pointer = (at + 1) % contents.len();
                }
            }
        }
    }

    /// A START begins a transaction, and what the device expects next depends on its direction.
    /// A master write is the addressing phase: the two bytes that follow set the internal
    /// address. A master read has no addressing phase, so the pointer stays where the previous
    /// transaction left it - which is the whole reason the ROM can write an address and then
    /// read from it (SPRUEC6G section 2.3.4).
    fn start(&mut self, master_transmits: bool) {
        // `Queue` only exists in a test build, so this is the only variant in a release one.
        #[allow(irrefutable_let_patterns)]
        if let I2cDevice::Eeprom { address_left, .. } = &mut self.device {
            *address_left = if master_transmits {
                EEPROM_ADDRESS_BYTES
            } else {
                0
            };
        }
    }
}

impl Default for I2c {
    fn default() -> Self {
        Self {
            oar: 0,
            imr: 0,
            status: STATUS_RESET,
            clkl: 0,
            clkh: 0,
            cnt: 0,
            drr: 0,
            sar: 0,
            dxr: 0,
            mdr: 0,
            emdr: 0,
            psc: 0,
            counter: 0,
            shifting: None,
            delay: 0,
            receiving: false,
            slave: None,
        }
    }
}

/// What one cycle of the module produced.
#[derive(Default, Clone, Copy)]
pub struct I2cEvents {
    pub receive: bool,
    pub transmit: bool,
}

impl I2c {
    /// Put a device on the bus at one address, with the bytes it answers a master read with.
    #[cfg(test)]
    pub fn attach_slave(&mut self, address: u32, transmit: Vec<u8>) {
        self.slave = Some(I2cSlave {
            address: address & 0x3ff,
            device: I2cDevice::Queue {
                transmit: transmit.into(),
                received: Vec::new(),
            },
        });
    }

    /// Put a serial EEPROM on the bus at one address. This is the device the ROM's I2C boot
    /// reads its boot parameters and its boot table out of (SPRUEC6G section 2.3.4).
    pub fn attach_eeprom(&mut self, address: u32, contents: Vec<u8>) {
        self.slave = Some(I2cSlave {
            address: address & 0x3ff,
            device: I2cDevice::Eeprom {
                contents,
                pointer: 0,
                address_left: EEPROM_ADDRESS_BYTES,
            },
        });
    }

    /// What the attached device has been sent.
    #[cfg(test)]
    pub fn slave_received(&self) -> Vec<u8> {
        match self.slave.as_ref().map(|slave| &slave.device) {
            Some(I2cDevice::Queue { received, .. }) => received.clone(),
            _ => Vec::new(),
        }
    }

    /// True while the addressed device is the one on the bus.
    fn slave_addressed(&self) -> bool {
        self.slave
            .as_ref()
            .is_some_and(|slave| slave.address == self.sar)
    }
}

impl SlavePort for I2c {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        let value = match address {
            ICOAR => self.oar,
            ICIMR => self.imr,
            // No master on the bus addresses this module, so the slave-side flags read zero.
            ICSTR => self.status & !SLAVE_FLAGS,
            ICCLKL => self.clkl,
            ICCLKH => self.clkh,
            ICCNT => self.cnt,
            ICDRR => {
                // Reading the receive register clears its ready flag and the overrun it hid.
                self.status &= !(ICRRDY | RSFULL);
                self.drr
            }
            ICSAR => self.sar,
            ICDXR => self.dxr,
            ICMDR => self.mdr,
            // Reading the vector register reports and clears the highest-priority interrupt.
            ICIVR => self.take_vector(),
            ICEMDR => self.emdr,
            ICPSC => self.psc,
            _ => return None,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        match address {
            ICOAR => self.oar = value & 0x3ff,
            ICIMR => self.imr = value & 0x7f,
            ICSTR => self.status &= !(value & WRITE_ONE_TO_CLEAR),
            ICCLKL => self.clkl = value & 0xffff,
            ICCLKH => self.clkh = value & 0xffff,
            ICCNT => self.cnt = value & 0xffff,
            ICDRR | ICIVR => {}
            ICSAR => self.sar = value & 0x3ff,
            ICDXR => {
                self.dxr = value & 0xff;
                // Writing the register clears its ready flag and refills the transmitter.
                self.status = (self.status & !ICXRDY) | XSMT;
                self.load_shift_register();
            }
            ICMDR => self.write_mode(value),
            ICEMDR => self.emdr = value & 3,
            ICPSC => self.psc = value & 0xff,
            _ => return PortWrite::Unhandled,
        }
        PortWrite::Done
    }
}

impl I2c {
    fn write_mode(&mut self, value: u32) {
        let previous = self.mdr;
        self.mdr = value & 0x0000_ffef;
        if self.mdr & MDR_IRS == 0 {
            // A software reset returns every status flag to its default (SPRU974A §2.10.1).
            self.status = STATUS_RESET;
            self.counter = 0;
            self.shifting = None;
            self.receiving = false;
            return;
        }
        if previous & MDR_STT == 0 && self.mdr & MDR_STT != 0 && self.mdr & MDR_MST != 0 {
            // A master START loads the internal data counter, marks the bus busy and clears
            // STT again (SPRU974A Table 14 and §3.5). A device on the bus sees the START and
            // its direction, which is what tells an EEPROM whether the bytes that follow are
            // its internal address or a read from where the last one left the pointer.
            self.counter = if self.cnt == 0 { 0x1_0000 } else { self.cnt };
            self.status |= BB;
            self.mdr &= !MDR_STT;
            let transmitting = self.mdr & MDR_TRX != 0;
            if self.slave_addressed() {
                if let Some(slave) = self.slave.as_mut() {
                    slave.start(transmitting);
                }
            }
            self.load_shift_register();
            return;
        }
        // Repeat mode runs until software asks for the STOP (SPRU974A Table 15, RM = 1 and
        // STP = 1 without STT). A word already on the bus finishes first and the STOP follows
        // it in `finish_word`; with nothing on the bus the STOP goes out at once.
        if self.mdr & (MDR_RM | MDR_STP | MDR_MST) == MDR_RM | MDR_STP | MDR_MST
            && self.status & BB != 0
            && !self.transfer_active()
        {
            self.stop();
        }
    }

    /// A STOP condition: the bus is free, the module drops back to slave and STP clears
    /// (SPRU974A Table 7 and Table 14).
    fn stop(&mut self) {
        self.status |= SCD;
        self.status &= !BB;
        self.mdr &= !(MDR_STP | MDR_MST);
    }

    /// A byte only starts shifting while the module is enabled, is the master transmitter and
    /// has a START behind it.
    fn load_shift_register(&mut self) {
        let ready = self.mdr & (MDR_IRS | MDR_MST | MDR_TRX) == MDR_IRS | MDR_MST | MDR_TRX
            && self.status & BB != 0
            && self.status & ICXRDY == 0;
        if !ready || self.shifting.is_some() {
            return;
        }
        self.shifting = Some(self.dxr);
        // SPRU974A Table 14: in digital loopback mode the byte lands after
        // (prescaler + 1) x 8 DSP cycles; the same figure paces the bus transfer here.
        self.delay = (self.psc + 1) * 8;
    }

    /// A master read clocks a byte in from the addressed device while the receiver has room and
    /// the internal counter still has bytes to fetch (SPRU974A section 3.5).
    fn start_receive(&mut self) {
        let ready = self.mdr & (MDR_IRS | MDR_MST | MDR_TRX) == MDR_IRS | MDR_MST
            && self.status & BB != 0
            && self.status & ICRRDY == 0
            && self.counter != 0
            && self.slave_addressed();
        if !ready {
            return;
        }
        self.receiving = true;
        self.delay = (self.psc + 1) * 8;
    }

    /// Whether a cycle can advance a transfer or must keep driving a pending interrupt level.
    pub fn active(&self) -> bool {
        self.transfer_active() || self.interrupt_pending()
    }

    /// Whether the serialiser can change state without a register write. Pending interrupt
    /// levels are relatched by System's dormant path and do not require ticking the bus.
    pub fn transfer_active(&self) -> bool {
        self.receiving
            || self.shifting.is_some()
            || (self.mdr & (MDR_IRS | MDR_MST | MDR_TRX) == MDR_IRS | MDR_MST
                && self.status & BB != 0
                && self.status & ICRRDY == 0
                && self.counter != 0
                && self.slave_addressed())
    }

    /// One CPU cycle of the serialiser.
    pub fn tick(&mut self) -> I2cEvents {
        let mut events = I2cEvents::default();
        if self.shifting.is_none() && !self.receiving {
            self.start_receive();
        }
        if self.receiving {
            self.delay -= 1;
            if self.delay > 0 {
                return events;
            }
            self.receiving = false;
            let byte = self.slave.as_mut().map_or(0xff, I2cSlave::read);
            self.drr = u32::from(byte);
            self.status |= ICRRDY;
            events.receive = true;
            self.finish_word();
            return events;
        }
        let Some(byte) = self.shifting else {
            return events;
        };
        self.delay -= 1;
        if self.delay > 0 {
            return events;
        }
        self.shifting = None;
        // The byte has left ICDXR, so the register is free again and the transmit event fires.
        self.status |= ICXRDY;
        events.transmit = true;
        if self.mdr & MDR_DLB != 0 {
            // The internal path returns the byte to the receiver, which reports an overrun if
            // the previous one was never read (SPRU974A Figure 24 and Table 7).
            if self.status & ICRRDY != 0 {
                self.status |= RSFULL;
            } else {
                self.drr = byte;
                self.status |= ICRRDY;
                events.receive = true;
            }
        } else if self.slave_addressed() {
            // The device acknowledges, and keeps the byte.
            if let Some(slave) = self.slave.as_mut() {
                slave.write(byte as u8);
            }
        } else {
            // Nothing acknowledges a byte on an otherwise empty bus, or at an address no device
            // on it answers to.
            self.status |= NACK;
        }
        if self.mdr & MDR_NACKMOD != 0 {
            self.status |= NACKSNT;
            self.mdr &= !MDR_NACKMOD;
        }
        self.finish_word();
        self.load_shift_register();
        events
    }

    /// Count the word off the internal counter and act on the end of the transfer: a STOP when
    /// STP is set, otherwise the register-access-ready flag (SPRU974A Table 7 and Table 15).
    /// Repeat mode never counts; the word STP was set during is its last.
    fn finish_word(&mut self) {
        if self.mdr & MDR_RM != 0 {
            if self.mdr & MDR_STP != 0 {
                self.stop();
            } else {
                self.status |= ARDY;
            }
            return;
        }
        self.counter = self.counter.saturating_sub(1);
        if self.counter != 0 {
            return;
        }
        if self.mdr & MDR_STP != 0 {
            self.stop();
        } else {
            self.status |= ARDY;
        }
    }

    /// The masked interrupt the module presents to the CPU.
    pub fn interrupt_pending(&self) -> bool {
        self.vector().is_some()
    }

    fn vector(&self) -> Option<(u32, u32)> {
        VECTORS
            .iter()
            .find(|(_, flag, mask)| self.status & flag != 0 && self.imr & mask != 0)
            .map(|(code, flag, _)| (*code, *flag))
    }

    fn take_vector(&mut self) -> u32 {
        match self.vector() {
            Some((code, flag)) => {
                self.status &= !flag;
                code
            }
            None => 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    #[test]
    fn digital_loopback_returns_the_transmitted_byte() {
        // SPRU974A Table 14: in DLB mode the byte written to ICDXR reaches ICDRR after
        // (prescaler + 1) x 8 DSP cycles, and the status flags follow Table 7.
        let mut i2c = I2c::default();
        assert_eq!(i2c.read(ICSTR, TEST_CONTEXT), Some(XSMT | ICXRDY));
        i2c.write(ICPSC, 1, TEST_CONTEXT);
        i2c.write(ICCNT, 1, TEST_CONTEXT);
        i2c.write(
            ICMDR,
            MDR_IRS | MDR_MST | MDR_TRX | MDR_DLB | MDR_STT | MDR_STP,
            TEST_CONTEXT,
        );
        // The START cleared STT, loaded the counter and marked the bus busy.
        assert_eq!(i2c.read(ICMDR, TEST_CONTEXT).unwrap() & MDR_STT, 0);
        assert_eq!(i2c.read(ICSTR, TEST_CONTEXT).unwrap() & BB, BB);
        i2c.write(ICDXR, 0x5a, TEST_CONTEXT);
        assert_eq!(i2c.read(ICSTR, TEST_CONTEXT).unwrap() & ICXRDY, 0);
        for _ in 0..15 {
            let events = i2c.tick();
            assert!(!events.receive && !events.transmit);
        }
        let events = i2c.tick();
        assert!(events.receive && events.transmit);
        assert_eq!(i2c.read(ICDRR, TEST_CONTEXT), Some(0x5a));
        // Reading ICDRR cleared the receive flag; the transmitter is ready and the counter
        // reached zero with STP set, so the module reported a STOP and left the bus.
        let status = i2c.read(ICSTR, TEST_CONTEXT).unwrap();
        assert_eq!(status & ICRRDY, 0);
        assert_eq!(status & ICXRDY, ICXRDY);
        assert_eq!(status & SCD, SCD);
        assert_eq!(status & BB, 0);
        assert_eq!(i2c.read(ICMDR, TEST_CONTEXT).unwrap() & MDR_MST, 0);
    }

    #[test]
    fn the_vector_register_reports_the_highest_priority_interrupt() {
        // SPRU974A §3.10: reading ICIVR clears the highest-priority masked flag, and a lower
        // one then shows up on the next read.
        let mut i2c = I2c::default();
        i2c.write(ICIMR, (1 << 4) | (1 << 3) | (1 << 1), TEST_CONTEXT);
        i2c.write(
            ICMDR,
            MDR_IRS | MDR_MST | MDR_TRX | MDR_STT | MDR_RM,
            TEST_CONTEXT,
        );
        i2c.write(ICDXR, 0x11, TEST_CONTEXT);
        for _ in 0..8 {
            i2c.tick();
        }
        // A byte on an empty bus draws no acknowledge, so NACK stands beside ICXRDY.
        let status = i2c.read(ICSTR, TEST_CONTEXT).unwrap();
        assert_eq!(status & NACK, NACK);
        assert!(i2c.interrupt_pending());
        assert_eq!(i2c.read(ICIVR, TEST_CONTEXT), Some(2));
        assert_eq!(i2c.read(ICSTR, TEST_CONTEXT).unwrap() & NACK, 0);
        assert_eq!(i2c.read(ICIVR, TEST_CONTEXT), Some(5));
        assert_eq!(i2c.read(ICIVR, TEST_CONTEXT), Some(0));
        assert!(!i2c.interrupt_pending());
    }

    #[test]
    fn a_software_reset_returns_the_status_flags_to_their_defaults() {
        // SPRU974A §2.10.1: clearing IRS forces every status bit to its default value.
        let mut i2c = I2c::default();
        i2c.write(
            ICMDR,
            MDR_IRS | MDR_MST | MDR_TRX | MDR_DLB | MDR_STT | MDR_RM,
            TEST_CONTEXT,
        );
        i2c.write(ICDXR, 0x77, TEST_CONTEXT);
        for _ in 0..8 {
            i2c.tick();
        }
        assert_ne!(i2c.read(ICSTR, TEST_CONTEXT).unwrap() & (ICRRDY | ARDY), 0);
        i2c.write(ICMDR, 0, TEST_CONTEXT);
        assert_eq!(i2c.read(ICSTR, TEST_CONTEXT), Some(STATUS_RESET));
        assert_eq!(i2c.read(ICSTR, TEST_CONTEXT).unwrap() & SLAVE_FLAGS, 0);
    }

    fn tick_until(i2c: &mut I2c, flag: u32) {
        for _ in 0..1000 {
            if i2c.read(ICSTR, TEST_CONTEXT).unwrap() & flag != 0 {
                return;
            }
            i2c.tick();
        }
        panic!("ICSTR flag {flag:#x} never came up");
    }

    /// An EEPROM read the way TI's IBL driver does it (hw/i2c/i2c.c hwI2cMasterRead): repeat
    /// mode throughout, so ICCNT plays no part, and the STOP comes from writing STP alone.
    fn read_in_repeat_mode(i2c: &mut I2c, address: u16, count: usize) -> Vec<u8> {
        let transmit = MDR_IRS | MDR_MST | MDR_TRX | MDR_RM;
        i2c.write(ICSAR, 0x50, TEST_CONTEXT);
        i2c.write(ICMDR, transmit, TEST_CONTEXT);
        i2c.write(ICDXR, u32::from(address >> 8), TEST_CONTEXT);
        i2c.write(ICMDR, transmit | MDR_STT, TEST_CONTEXT);
        tick_until(i2c, ICXRDY);
        i2c.write(ICDXR, u32::from(address & 0xff), TEST_CONTEXT);
        tick_until(i2c, ICXRDY);
        // A repeated START turns the bus round without a STOP in between.
        i2c.write(ICMDR, MDR_IRS | MDR_MST | MDR_RM | MDR_STT, TEST_CONTEXT);
        let mut bytes = Vec::new();
        for _ in 0..count {
            tick_until(i2c, ICRRDY);
            bytes.push(i2c.read(ICDRR, TEST_CONTEXT).unwrap() as u8);
        }
        i2c.write(ICMDR, MDR_IRS | MDR_MST | MDR_RM | MDR_STP, TEST_CONTEXT);
        bytes
    }

    #[test]
    fn stp_ends_a_repeat_mode_read_after_the_byte_on_the_bus() {
        // SPRU974A Table 15: with RM = 1, STT = 0 and STP = 1 the master sends a STOP. The
        // receiver had already started on the next byte when STP was written, so that byte
        // lands first; then the bus is free and the module is a slave again.
        let mut i2c = I2c::default();
        i2c.attach_eeprom(0x50, (0..=255).collect());
        i2c.write(ICMDR, MDR_IRS, TEST_CONTEXT);
        assert_eq!(
            read_in_repeat_mode(&mut i2c, 0x20, 4),
            [0x20, 0x21, 0x22, 0x23]
        );
        assert_ne!(i2c.read(ICSTR, TEST_CONTEXT).unwrap() & BB, 0);
        tick_until(&mut i2c, ICRRDY);
        assert_eq!(i2c.read(ICDRR, TEST_CONTEXT), Some(0x24));
        let status = i2c.read(ICSTR, TEST_CONTEXT).unwrap();
        assert_eq!(status & (BB | SCD), SCD);
        assert_eq!(
            i2c.read(ICMDR, TEST_CONTEXT).unwrap() & (MDR_MST | MDR_STP),
            0
        );
        // Nothing more is clocked in once the STOP is out.
        for _ in 0..1000 {
            i2c.tick();
        }
        assert_eq!(i2c.read(ICSTR, TEST_CONTEXT).unwrap() & ICRRDY, 0);
        // With the bus free the next transaction starts from its own address.
        assert_eq!(read_in_repeat_mode(&mut i2c, 0x80, 2), [0x80, 0x81]);
    }
}
