//! The C6455's two multichannel buffered serial ports.
//!
//! The register file and its bit layouts come from the CSL register appendix (SPRU401J tables
//! B-210 to B-224), the addresses from SPRS276M Tables 7-57 and 7-58: each port answers on its
//! configuration bus, and DRR and DXR answer again on the EDMA data bus.
//!
//! What moves data here is digital loopback, the one path a port can run without a device on its
//! pins: with SPCR.DLB set, the transmit serialiser feeds the receive serialiser, so a word
//! written to DXR arrives in DRR a word's worth of serial clocks later. The clock is the sample
//! rate generator's CLKG, which with SRGR.CLKSM set divides the CPU clock by CLKGDV + 1, and
//! SPCR.GRST holds the generator in reset until software takes it out. The double buffering is
//! the documented pair: DXR hands its word to XSR when XSR is free, which is what clears XRDY
//! and sets it again, and RSR hands its word to DRR, which sets RRDY - or RFULL, when the
//! previous word was never read.
//!
//! Not modelled, and stored without deciding anything: multichannel selection (MCR with the
//! channel enable registers), companding, bit reversal and the frame-sync generator's period and
//! width. Those only become observable with a framing device on the pins, which this model does
//! not have; the loopback path uses a single-phase frame of one word, which is what the reset
//! values of RCR and XCR describe.
use crate::port::{PortContext, PortWrite, SlavePort};
#[cfg(test)]
const TEST_CONTEXT: PortContext = PortContext {
    endian: crate::system::Endian::Little,
};

/// Register offsets within a port's configuration window (SPRS276M Table 7-57).
const DRR: u32 = 0x00;
const DXR: u32 = 0x04;
const SPCR: u32 = 0x08;
const RCR: u32 = 0x0c;
const XCR: u32 = 0x10;
const SRGR: u32 = 0x14;
const MCR: u32 = 0x18;
const RCERE0: u32 = 0x1c;
const XCERE0: u32 = 0x20;
const PCR: u32 = 0x24;
const RCERE1: u32 = 0x28;
const XCERE1: u32 = 0x2c;
const RCERE2: u32 = 0x30;
const XCERE2: u32 = 0x34;
const RCERE3: u32 = 0x38;
const XCERE3: u32 = 0x3c;
const CONFIG_WINDOW: u32 = 0x40;

/// SPCR fields (SPRU401J Table B-213).
const SPCR_RRST: u32 = 1 << 0;
const SPCR_RRDY: u32 = 1 << 1;
const SPCR_RFULL: u32 = 1 << 2;
const SPCR_RSYNCERR: u32 = 1 << 3;
const SPCR_RINTM: u32 = 3 << 4;
const SPCR_DXENA: u32 = 1 << 7;
const SPCR_CLKSTP: u32 = 3 << 11;
const SPCR_RJUST: u32 = 3 << 13;
const SPCR_DLB: u32 = 1 << 15;
const SPCR_XRST: u32 = 1 << 16;
const SPCR_XRDY: u32 = 1 << 17;
const SPCR_XEMPTY: u32 = 1 << 18;
const SPCR_XSYNCERR: u32 = 1 << 19;
const SPCR_XINTM: u32 = 3 << 20;
const SPCR_GRST: u32 = 1 << 22;
const SPCR_FRST: u32 = 1 << 23;
const SPCR_SOFT: u32 = 1 << 24;
const SPCR_FREE: u32 = 1 << 25;
/// The bits software may write; XRDY, XEMPTY, RRDY and RFULL are the port's own status.
const SPCR_WRITABLE: u32 = SPCR_RRST
    | SPCR_RSYNCERR
    | SPCR_RINTM
    | SPCR_DXENA
    | SPCR_CLKSTP
    | SPCR_RJUST
    | SPCR_DLB
    | SPCR_XRST
    | SPCR_XSYNCERR
    | SPCR_XINTM
    | SPCR_GRST
    | SPCR_FRST
    | SPCR_SOFT
    | SPCR_FREE;
/// XEMPTY reads zero while the shift register holds a word and one while it does not, so a port
/// in reset reports an empty transmit shifter and a ready transmit register.
const SPCR_RESET: u32 = SPCR_XEMPTY | SPCR_XRDY;

/// SRGR resets with the internal clock selected and a divider of one (SPRU401J Figure B-212).
const SRGR_CLKSM: u32 = 1 << 29;
const SRGR_RESET: u32 = SRGR_CLKSM | 1;

/// RJUST modes: right-justify zero-filled, right-justify sign-extended, left-justify zero-filled
/// (SPRU401J Table B-213).
const RJUST_RIGHT_ZERO: u32 = 0;
const RJUST_RIGHT_SIGN: u32 = 1;
const RJUST_LEFT_ZERO: u32 = 2;

/// The interrupt modes this model distinguishes: mode 0 follows the ready flag, and the other
/// three need a frame or a synchronisation error, which only a framing device produces.
const INTM_READY: u32 = 0;
const INTM_SYNCERR: u32 = 3;

/// What one CPU cycle of a port produced.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct McbspEvents {
    /// XEVT: the transmit register is ready for another word.
    pub transmit: bool,
    /// REVT: a word has arrived in the receive register.
    pub receive: bool,
    /// XINT and RINT, as XINTM and RINTM select them.
    pub transmit_interrupt: bool,
    pub receive_interrupt: bool,
}

pub struct Mcbsp {
    /// The configuration window's base, and the data-bus addresses of DRR and DXR.
    base: u32,
    data_receive: u32,
    data_transmit: u32,
    drr: u32,
    dxr: u32,
    spcr: u32,
    rcr: u32,
    xcr: u32,
    srgr: u32,
    mcr: u32,
    pcr: u32,
    receive_enable: [u32; 4],
    transmit_enable: [u32; 4],
    /// The word in the transmit shift register, and the serial clocks it still needs.
    shifting: Option<u32>,
    remaining: u32,
    /// CPU cycles until the next CLKG edge.
    divide: u32,
}

impl Mcbsp {
    pub fn new(base: u32, data_receive: u32, data_transmit: u32) -> Self {
        Self {
            base,
            data_receive,
            data_transmit,
            drr: 0,
            dxr: 0,
            spcr: SPCR_RESET,
            rcr: 0,
            xcr: 0,
            srgr: SRGR_RESET,
            mcr: 0,
            pcr: 0,
            receive_enable: [0; 4],
            transmit_enable: [0; 4],
            shifting: None,
            remaining: 0,
            divide: 0,
        }
    }
}

impl SlavePort for Mcbsp {
    fn read(&mut self, address: u32, _context: PortContext) -> Option<u32> {
        if !self.owns(address) {
            return None;
        }
        if address == self.data_receive {
            return Some(self.read_drr());
        }
        if address == self.data_transmit {
            return Some(self.dxr);
        }
        let offset = self.offset(address)?;
        let value = match offset {
            DRR => self.read_drr(),
            DXR => self.dxr,
            SPCR => self.spcr,
            RCR => self.rcr,
            XCR => self.xcr,
            SRGR => self.srgr,
            MCR => self.mcr,
            PCR => self.pcr,
            RCERE0 => self.receive_enable[0],
            RCERE1 => self.receive_enable[1],
            RCERE2 => self.receive_enable[2],
            RCERE3 => self.receive_enable[3],
            XCERE0 => self.transmit_enable[0],
            XCERE1 => self.transmit_enable[1],
            XCERE2 => self.transmit_enable[2],
            XCERE3 => self.transmit_enable[3],
            _ => 0,
        };
        Some(value)
    }

    fn write(&mut self, address: u32, value: u32, _context: PortContext) -> PortWrite {
        if !self.owns(address) {
            return PortWrite::Unhandled;
        }
        if address == self.data_transmit {
            self.write_dxr(value);
            return PortWrite::Done;
        }
        if address == self.data_receive {
            // DRR is read-only to the CPU and the EDMA (SPRS276M Table 7-57).
            return PortWrite::Done;
        }
        let Some(offset) = self.offset(address) else {
            return PortWrite::Unhandled;
        };
        match offset {
            DRR => {}
            DXR => self.write_dxr(value),
            SPCR => self.write_spcr(value),
            RCR => self.rcr = value,
            XCR => self.xcr = value,
            SRGR => self.srgr = value,
            MCR => self.mcr = value,
            PCR => self.pcr = value,
            RCERE0 => self.receive_enable[0] = value,
            RCERE1 => self.receive_enable[1] = value,
            RCERE2 => self.receive_enable[2] = value,
            RCERE3 => self.receive_enable[3] = value,
            XCERE0 => self.transmit_enable[0] = value,
            XCERE1 => self.transmit_enable[1] = value,
            XCERE2 => self.transmit_enable[2] = value,
            XCERE3 => self.transmit_enable[3] = value,
            _ => {}
        }
        PortWrite::Done
    }
}

impl Mcbsp {
    pub fn owns(&self, address: u32) -> bool {
        address == self.data_receive
            || address == self.data_transmit
            || (self.base..self.base + CONFIG_WINDOW).contains(&address)
    }

    /// Whether a CPU cycle can change or reassert any observable port state.
    pub fn active(&self) -> bool {
        if self.spcr & SPCR_GRST != 0 || self.divide != 0 {
            return true;
        }
        let transmit_mode = (self.spcr & SPCR_XINTM) >> 20;
        let receive_mode = (self.spcr & SPCR_RINTM) >> 4;
        (transmit_mode == INTM_SYNCERR && self.spcr & SPCR_XSYNCERR != 0)
            || (receive_mode == INTM_SYNCERR && self.spcr & SPCR_RSYNCERR != 0)
    }

    /// One CPU cycle. The sample rate generator's CLKG paces the serialiser: with CLKSM set it is
    /// the CPU clock divided by CLKGDV + 1, and GRST holds the generator in reset until software
    /// releases it (SPRU401J Table B-217 and Table B-213).
    pub fn tick(&mut self) -> McbspEvents {
        if self.spcr & SPCR_GRST == 0 {
            self.divide = 0;
            return self.events(false, false);
        }
        let divider = (self.srgr & 0xff) + 1;
        self.divide += 1;
        if self.divide < divider {
            return self.events(false, false);
        }
        self.divide = 0;
        self.load_shift_register();
        let Some(word) = self.shifting else {
            return self.events(false, false);
        };
        self.remaining -= 1;
        if self.remaining > 0 {
            return self.events(false, false);
        }
        // The word has left the shift register: XEMPTY says so and a new one can be loaded.
        self.shifting = None;
        self.spcr |= SPCR_XEMPTY;
        let mut received = false;
        if self.spcr & SPCR_DLB != 0 && self.spcr & SPCR_RRST != 0 {
            // Digital loopback: the transmit serialiser drives the receive serialiser, so the
            // word arrives in DRR - or is lost to an overrun, which RFULL reports.
            if self.spcr & SPCR_RRDY != 0 {
                self.spcr |= SPCR_RFULL;
            } else {
                self.drr = self.justify(word);
                self.spcr |= SPCR_RRDY;
                received = true;
            }
        }
        let transmitted = self.load_shift_register();
        self.events(transmitted || self.spcr & SPCR_XRDY != 0, received)
    }

    /// DXR hands its word to XSR when the shifter is free and the transmitter is enabled, which
    /// is what makes DXR ready again (SPRU401J Table B-213, XRDY and XEMPTY).
    fn load_shift_register(&mut self) -> bool {
        if self.shifting.is_some()
            || self.spcr & SPCR_XRST == 0
            || self.spcr & SPCR_XRDY != 0
            || self.spcr & SPCR_GRST == 0
        {
            return false;
        }
        self.shifting = Some(self.dxr);
        self.remaining = self.transmit_bits();
        self.spcr = (self.spcr | SPCR_XRDY) & !SPCR_XEMPTY;
        true
    }

    fn write_dxr(&mut self, value: u32) {
        self.dxr = value;
        // The write consumes the transmitter's readiness until the word reaches the shifter.
        self.spcr &= !SPCR_XRDY;
        self.load_shift_register();
    }

    fn read_drr(&mut self) -> u32 {
        // Reading the receive register clears its ready flag and the overrun that hid behind it.
        self.spcr &= !(SPCR_RRDY | SPCR_RFULL);
        self.drr
    }

    fn write_spcr(&mut self, value: u32) {
        let previous = self.spcr;
        self.spcr = (self.spcr & !SPCR_WRITABLE) | (value & SPCR_WRITABLE);
        if self.spcr & SPCR_XRST == 0 {
            // A transmitter in reset empties its shifter and reports DXR ready (SPRU401J
            // Table B-213).
            self.shifting = None;
            self.spcr = (self.spcr | SPCR_XEMPTY | SPCR_XRDY) & !SPCR_XSYNCERR;
        }
        if self.spcr & SPCR_RRST == 0 {
            self.spcr &= !(SPCR_RRDY | SPCR_RFULL | SPCR_RSYNCERR);
        }
        if self.spcr & SPCR_GRST == 0 {
            self.divide = 0;
        }
        if previous & SPCR_XRST == 0 && self.spcr & SPCR_XRST != 0 {
            self.load_shift_register();
        }
    }

    /// The word length XCR names for phase 1, in bits (SPRU401J Table B-216).
    fn transmit_bits(&self) -> u32 {
        word_bits((self.xcr >> 5) & 7)
    }

    fn receive_bits(&self) -> u32 {
        word_bits((self.rcr >> 5) & 7)
    }

    /// RJUST decides where a received word shorter than 32 bits sits in DRR and what fills the
    /// rest (SPRU401J Table B-213).
    fn justify(&self, word: u32) -> u32 {
        let bits = self.receive_bits();
        if bits >= 32 {
            return word;
        }
        let mask = (1u32 << bits) - 1;
        let value = word & mask;
        match (self.spcr & SPCR_RJUST) >> 13 {
            RJUST_RIGHT_ZERO => value,
            RJUST_RIGHT_SIGN => {
                let sign = 1u32 << (bits - 1);
                if value & sign != 0 {
                    value | !mask
                } else {
                    value
                }
            }
            RJUST_LEFT_ZERO => value << (32 - bits),
            // The fourth encoding is reserved; the port leaves the word alone.
            _ => value,
        }
    }

    /// XINT and RINT follow their mode fields: mode 0 is the ready flag, mode 3 the
    /// synchronisation error, and the two frame modes need a framing device (SPRU401J
    /// Table B-213).
    fn events(&self, transmit: bool, receive: bool) -> McbspEvents {
        let transmit_mode = (self.spcr & SPCR_XINTM) >> 20;
        let receive_mode = (self.spcr & SPCR_RINTM) >> 4;
        McbspEvents {
            transmit,
            receive,
            transmit_interrupt: match transmit_mode {
                INTM_READY => transmit,
                INTM_SYNCERR => self.spcr & SPCR_XSYNCERR != 0,
                _ => false,
            },
            receive_interrupt: match receive_mode {
                INTM_READY => receive,
                INTM_SYNCERR => self.spcr & SPCR_RSYNCERR != 0,
                _ => false,
            },
        }
    }

    fn offset(&self, address: u32) -> Option<u32> {
        if !(self.base..self.base + CONFIG_WINDOW).contains(&address) {
            return None;
        }
        Some(address - self.base)
    }
}

/// The word length encoding both control registers share: 8, 12, 16, 20, 24 or 32 bits, with the
/// two reserved encodings behaving as the longest (SPRU401J Tables B-215 and B-216).
fn word_bits(encoding: u32) -> u32 {
    match encoding {
        0 => 8,
        1 => 12,
        2 => 16,
        3 => 20,
        4 => 24,
        _ => 32,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;
    fn port() -> Mcbsp {
        Mcbsp::new(0x028c_0000, 0x3000_0000, 0x3000_0010)
    }

    #[test]
    fn a_port_comes_up_with_the_documented_resets() {
        let mut port = port();
        // SPCR reports an empty shifter and a ready DXR, SRGR the internal clock and a divider
        // of one, and everything else is clear (SPRU401J Figures B-208 and B-212).
        assert_eq!(
            port.read(0x028c_0008, TEST_CONTEXT),
            Some((1 << 18) | (1 << 17))
        );
        assert_eq!(port.read(0x028c_0014, TEST_CONTEXT), Some((1 << 29) | 1));
        assert_eq!(port.read(0x028c_000c, TEST_CONTEXT), Some(0));
        assert_eq!(port.read(0x028c_0024, TEST_CONTEXT), Some(0));
    }

    #[test]
    fn a_loopback_word_takes_one_serial_clock_per_bit() {
        let mut port = port();
        // 16-bit words, loopback, both halves and the generator out of reset, CLKGDV = 1 so a
        // serial clock is two CPU cycles.
        port.write(0x028c_000c, 2 << 5, TEST_CONTEXT);
        port.write(0x028c_0010, 2 << 5, TEST_CONTEXT);
        port.write(0x028c_0014, SRGR_CLKSM | 1, TEST_CONTEXT);
        port.write(
            0x028c_0008,
            SPCR_DLB | SPCR_XRST | SPCR_RRST | SPCR_GRST | SPCR_FRST,
            TEST_CONTEXT,
        );
        port.write(0x028c_0004, 0xbeef, TEST_CONTEXT);
        let mut cycles = 0;
        let mut arrived = None;
        for _ in 0..200 {
            cycles += 1;
            if port.tick().receive {
                arrived = Some(cycles);
                break;
            }
        }
        // Sixteen bits at two CPU cycles each, and the word that arrives is the word sent.
        assert_eq!(arrived, Some(32));
        assert_eq!(port.read(0x028c_0000, TEST_CONTEXT), Some(0xbeef));
        // Reading DRR cleared RRDY.
        assert_eq!(port.read(0x028c_0008, TEST_CONTEXT).unwrap() & SPCR_RRDY, 0);
    }
}
