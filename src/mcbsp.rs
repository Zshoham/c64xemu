//! The C6455's two multichannel buffered serial ports.
//!
//! The behaviour is SPRU580G, the C6000 McBSP reference guide; the addresses are SPRS276M Tables
//! 7-57 and 7-58: each port answers on its configuration bus, and DRR and DXR answer again on the
//! EDMA data bus.
//!
//! The serialiser runs one step per CLKG cycle. CLKG is the sample rate generator's output: with
//! SRGR.CLKSM set its input is the device's internal source, CPU/6 on the C6455 (SPRU580G section
//! 4.3; SPRS276M Table 2-1), which is the rate of SYSCLK3, divided by CLKGDV + 1. Within a step the
//! transmitter runs first and the receiver samples what it drove, which is digital loopback
//! (SPCR.DLB) and SPI master mode (SPCR.CLKSTP = 1x) alike. A frame is the RCR/XCR layout: one or
//! two phases, each with its own element count and length, the data delay after the frame sync,
//! and the frame-ignore bit deciding whether a sync that arrives mid-frame is an error.
//!
//! Frame syncs come from the frame-sync generator (FSG: FPER and FWID, released by SPCR.FRST),
//! from every DXR-to-XSR copy (PCR.FSXM = 1, SRGR.FSGM = 0), or from a pin. Receive data is
//! triple-buffered (RSR, RBR, DRR) and transmit data double-buffered (DXR, XSR), with RRDY, RFULL,
//! XRDY and XEMPTY following SPRU580G sections 3.2 and 5.5. Companding is G.711 mu-law and A-law
//! on 8-bit elements, compressing on the DXR-to-XSR copy and expanding on the RBR-to-DRR copy;
//! COMPAND = 01b sends 8-bit elements, or 32-bit ones with WDREVRS, LSB first. Multichannel
//! selection enables, disables and masks elements through MCR and the channel-enable registers,
//! in the 32-channel partition mode or the 128-channel enhanced mode.
//!
//! Nothing is attached to the pins. A clock or frame sync the port expects from a pin never
//! arrives, so a section clocked from CLKS, CLKX or CLKR as inputs, or framed from FSX or FSR as
//! inputs, waits forever, and DR and every general-purpose input read low. FREE/SOFT (emulation
//! halt), DXENA (DX turn-on delay), CLKSP and GSYNC (CLKS only) have nothing to act on here.
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

/// SPCR fields (SPRU580G Figure 54, Table 24).
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

/// RCR and XCR share one layout (SPRU580G Figures 55 and 56); bits 15 and 3:0 are reserved.
const CONTROL_PHASE: u32 = 1 << 31;
const CONTROL_FIG: u32 = 1 << 18;
const CONTROL_WDREVRS: u32 = 1 << 4;
const CONTROL_WRITABLE: u32 = !((1 << 15) | 0xf);

/// The COMPAND encodings (SPRU580G Tables 25 and 26).
const COMPAND_LSB_FIRST: u32 = 1;
const COMPAND_MULAW: u32 = 2;
/// Any law other than mu-law is A-law; only the tests name it.
#[cfg(test)]
const COMPAND_ALAW: u32 = 3;

/// SRGR resets with the internal clock selected and a divider of one (SPRU580G Figure 57).
const SRGR_FSGM: u32 = 1 << 28;
const SRGR_CLKSM: u32 = 1 << 29;
const SRGR_RESET: u32 = SRGR_CLKSM | 1;

/// MCR (SPRU580G Figure 58): XCBLK and RCBLK are the port's own, the rest reserved.
const MCR_RMCM: u32 = 1 << 0;
const MCR_RMCME: u32 = 1 << 9;
const MCR_XMCME: u32 = 1 << 25;
const MCR_WRITABLE: u32 = MCR_XMCME | (0xf << 21) | (3 << 16) | MCR_RMCME | (0xf << 5) | MCR_RMCM;

/// PCR (SPRU580G Figure 63): CLKSSTAT and DRSTAT are pins and read low here.
const PCR_CLKRP: u32 = 1 << 0;
const PCR_CLKXP: u32 = 1 << 1;
const PCR_FSRP: u32 = 1 << 2;
const PCR_FSXP: u32 = 1 << 3;
const PCR_CLKRM: u32 = 1 << 8;
const PCR_CLKXM: u32 = 1 << 9;
const PCR_FSRM: u32 = 1 << 10;
const PCR_FSXM: u32 = 1 << 11;
const PCR_RIOEN: u32 = 1 << 12;
const PCR_XIOEN: u32 = 1 << 13;
const PCR_WRITABLE: u32 = 0x3f00 | (1 << 5) | 0xf;

/// RJUST modes: right-justify zero-filled, right-justify sign-extended, left-justify zero-filled
/// (SPRU580G Table 24).
const RJUST_RIGHT_ZERO: u32 = 0;
const RJUST_RIGHT_SIGN: u32 = 1;
const RJUST_LEFT_ZERO: u32 = 2;

/// The interrupt modes (SPRU580G section 3.3): the ready flag, the end of a multichannel
/// subframe, a frame sync, and a synchronisation error.
const INTM_READY: u32 = 0;
const INTM_SUBFRAME: u32 = 1;
const INTM_FRAME: u32 = 2;
const INTM_SYNCERR: u32 = 3;

/// "Data is available in DRR four CPU clocks after being written to DXR" when both sections are
/// in reset and the companding hardware connects them (SPRU580G section 6.1).
const INTERNAL_COMPAND_CYCLES: u8 = 4;

/// What one CPU cycle of a port produced.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
pub struct McbspEvents {
    /// XEVT: XRDY rose, so DXR is ready for another word.
    pub transmit: bool,
    /// REVT: RRDY rose, so DRR holds a word.
    pub receive: bool,
    /// XINT and RINT, as XINTM and RINTM select them.
    pub transmit_interrupt: bool,
    pub receive_interrupt: bool,
}

/// Where a section is within its frames.
#[derive(Clone, Copy, Default)]
struct Framing {
    /// Bit clocks until the first data bit of a frame whose sync has been seen.
    start_in: Option<u32>,
    /// The element and the bit within it the frame in progress is at.
    position: Option<(u32, u32)>,
    /// The length of the element in progress, latched at its first bit: software rewriting
    /// RCR or XCR mid-element changes the elements after it, not the one being shifted.
    width: u32,
    /// Whether the element in progress is enabled and, for the transmitter, driven on DX.
    enabled: bool,
    unmasked: bool,
}

/// One bit clock of a frame: the element, the bit within it, and the element's latched length.
#[derive(Clone, Copy)]
struct Slot {
    element: u32,
    bit: u32,
    bits: u32,
}

impl Slot {
    fn last(&self) -> bool {
        self.bit + 1 >= self.bits
    }
}

/// What a frame sync did to a section (SPRU580G Figures 31 and 36).
#[derive(PartialEq, Eq)]
enum Sync {
    Start,
    Ignored,
    Error,
}

/// A section's frame as RCR or XCR describes it (SPRU580G section 4.5).
#[derive(Clone, Copy)]
struct Layout {
    first: u32,
    second: u32,
    first_bits: u32,
    second_bits: u32,
    delay: u32,
    ignore: bool,
}

impl Layout {
    fn new(control: u32) -> Self {
        // Companded data is always 8 bits wide, whatever the element length says (section 6).
        let companding = compand(control) >= COMPAND_MULAW;
        let width = |encoding| if companding { 8 } else { word_bits(encoding) };
        Self {
            first: ((control >> 8) & 0x7f) + 1,
            second: if control & CONTROL_PHASE != 0 {
                ((control >> 24) & 0x7f) + 1
            } else {
                0
            },
            first_bits: width((control >> 5) & 7),
            second_bits: width((control >> 21) & 7),
            // The fourth data delay encoding is reserved; it behaves as the longest.
            delay: ((control >> 16) & 3).min(2),
            ignore: control & CONTROL_FIG != 0,
        }
    }

    fn elements(&self) -> u32 {
        self.first + self.second
    }

    fn bits(&self, element: u32) -> u32 {
        if element < self.first {
            self.first_bits
        } else {
            self.second_bits
        }
    }

    fn total(&self) -> u32 {
        self.first * self.first_bits + self.second * self.second_bits
    }

    /// The bit clocks the elements after this one need.
    fn bits_after(&self, element: u32) -> u32 {
        let next = element + 1;
        if next < self.first {
            (self.first - next) * self.first_bits + self.second * self.second_bits
        } else {
            self.elements().saturating_sub(next) * self.second_bits
        }
    }

    /// The subframe current once this element has ended: the next element's, or subframe 0
    /// for the frame that follows the last.
    fn next_block(&self, element: u32) -> u32 {
        let next = element + 1;
        if next < self.elements() {
            next / 16 % 8
        } else {
            0
        }
    }

    /// Whether this element ends a 16-element subframe or the frame (section 8.2.2).
    fn ends_subframe(&self, element: u32) -> bool {
        element % 16 == 15 || element + 1 == self.elements()
    }
}

impl Framing {
    /// A frame sync: a frame starts after the data delay, unless one is still in progress more
    /// than the data delay from its end, which is an unexpected sync - ignored with FIG set,
    /// otherwise an error that abandons the frame and starts the next (sections 5.4.1, 5.5.2
    /// and 5.5.5).
    fn sync(&mut self, layout: &Layout) -> Sync {
        let shifting = self.position.map_or(0, |(element, bit)| {
            let current = if bit == 0 {
                layout.bits(element)
            } else {
                self.width
            };
            current.saturating_sub(bit) + layout.bits_after(element)
        });
        let pending = self.start_in.map_or(0, |delay| delay + layout.total());
        if shifting.max(pending) > layout.delay {
            if layout.ignore {
                return Sync::Ignored;
            }
            self.position = None;
            self.start_in = Some(layout.delay);
            return Sync::Error;
        }
        self.start_in = Some(layout.delay);
        Sync::Start
    }

    /// Abandon whatever frame is in progress or about to start.
    fn stop(&mut self) {
        self.start_in = None;
        self.position = None;
    }

    /// One bit clock: the slot this clock carries, if a frame is shifting.
    fn advance(&mut self, layout: &Layout) -> Option<Slot> {
        match self.start_in {
            Some(0) => {
                self.start_in = None;
                self.position = Some((0, 0));
            }
            Some(delay) => self.start_in = Some(delay - 1),
            None => {}
        }
        let (element, bit) = self.position?;
        if bit == 0 {
            self.width = layout.bits(element);
        }
        let bits = self.width;
        self.position = if bit + 1 < bits {
            Some((element, bit + 1))
        } else if element + 1 < layout.elements() {
            Some((element + 1, 0))
        } else {
            None
        };
        Some(Slot { element, bit, bits })
    }
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
    /// SYSCLK3 edges into the current CLKG cycle.
    divide: u32,
    /// The frame-sync generator: CLKG cycles until FSG next rises, whether it is high, and the
    /// further CLKG cycles it stays high.
    fsg_count: u32,
    fsg_high: bool,
    fsg_width: u32,
    /// DXR holds a word written since the last DXR-to-XSR copy.
    dxr_pending: bool,
    /// The transmit shift register, already compressed if companding, and what DXR last handed
    /// it - which an underflowing transmitter sends again (section 5.5.4).
    xsr: Option<u32>,
    last_copied: u32,
    transmit: Framing,
    /// The receive shift register, a complete element waiting to leave it, and the receive
    /// buffer register; each held word carries its element length.
    rsr: u32,
    rsr_full: Option<(u32, u32)>,
    rbr: Option<(u32, u32)>,
    receive: Framing,
    /// The subframes the multichannel logic is in (MCR.XCBLK and MCR.RCBLK).
    transmit_block: u32,
    receive_block: u32,
    /// DXR's word on its way through the companding hardware to DRR, with both sections in reset.
    internal_compand: Option<(u8, u32)>,
    events: McbspEvents,
}

impl Mcbsp {
    pub fn new(base: u32, data_receive: u32, data_transmit: u32) -> Self {
        Self {
            base,
            data_receive,
            data_transmit,
            drr: 0,
            dxr: 0,
            spcr: 0,
            rcr: 0,
            xcr: 0,
            srgr: SRGR_RESET,
            mcr: 0,
            pcr: 0,
            receive_enable: [0; 4],
            transmit_enable: [0; 4],
            divide: 0,
            fsg_count: 0,
            fsg_high: false,
            fsg_width: 0,
            dxr_pending: false,
            xsr: None,
            last_copied: 0,
            transmit: Framing::default(),
            rsr: 0,
            rsr_full: None,
            rbr: None,
            receive: Framing::default(),
            transmit_block: 0,
            receive_block: 0,
            internal_compand: None,
            events: McbspEvents::default(),
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
            MCR => self.mcr | (self.receive_block << 2) | (self.transmit_block << 18),
            PCR => self.read_pcr(),
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
            RCR => self.rcr = value & CONTROL_WRITABLE,
            XCR => self.xcr = value & CONTROL_WRITABLE,
            SRGR => self.srgr = value,
            MCR => self.mcr = value & MCR_WRITABLE,
            PCR => self.pcr = value & PCR_WRITABLE,
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
        self.clock_running()
            || self.events != McbspEvents::default()
            || self.internal_compand.is_some()
            || self.syncerr_interrupts() != (false, false)
    }

    /// One CPU cycle; `sysclk3` says whether it is a SYSCLK3 edge, the CPU/6 clock the sample
    /// rate generator divides by CLKGDV + 1 (SPRU580G section 4.3.2).
    pub fn tick(&mut self, sysclk3: bool) -> McbspEvents {
        if let Some((cycles, value)) = self.internal_compand {
            self.internal_compand = if cycles > 1 {
                Some((cycles - 1, value))
            } else {
                self.drr = value;
                None
            };
        }
        if sysclk3 && self.clock_running() {
            self.divide += 1;
            if self.divide > self.srgr & 0xff {
                self.divide = 0;
                self.clkg_cycle();
            }
        }
        let mut events = std::mem::take(&mut self.events);
        // A synchronisation error drives its interrupt for as long as it stands (section 3.3).
        let (transmit, receive) = self.syncerr_interrupts();
        events.transmit_interrupt |= transmit;
        events.receive_interrupt |= receive;
        events
    }

    /// CLKG runs while the generator is out of reset on the internal clock; CLKS has nothing on
    /// it (SPRU580G section 4.3.1).
    fn clock_running(&self) -> bool {
        self.spcr & SPCR_GRST != 0 && self.srgr & SRGR_CLKSM != 0
    }

    fn syncerr_interrupts(&self) -> (bool, bool) {
        (
            self.transmit_mode() == INTM_SYNCERR && self.spcr & SPCR_XSYNCERR != 0,
            self.receive_mode() == INTM_SYNCERR && self.spcr & SPCR_RSYNCERR != 0,
        )
    }

    fn transmit_mode(&self) -> u32 {
        (self.spcr & SPCR_XINTM) >> 20
    }

    fn receive_mode(&self) -> u32 {
        (self.spcr & SPCR_RINTM) >> 4
    }

    /// One CLKG cycle. CLKX_int is CLKG with CLKXM set; the receiver takes CLKX_int and FSX_int
    /// in digital loopback and in clock-stop mode, and CLKG and FSG with CLKRM and FSRM set
    /// otherwise (SPRU580G Tables 4 to 7 and section 9). A section in reset keeps its frame
    /// sync inactive (section 3.1).
    fn clkg_cycle(&mut self) {
        let fsg = self.frame_generator();
        let transmit_clocked = self.pcr & PCR_CLKXM != 0;
        let (dx, fsx) = if transmit_clocked && self.spcr & SPCR_XRST != 0 {
            self.transmit_clock(fsg)
        } else {
            (None, false)
        };
        let loopback = self.spcr & SPCR_DLB != 0;
        let clock_stop = self.spcr & SPCR_CLKSTP >= 2 << 11;
        let internal = loopback || clock_stop;
        let receive_clocked = if internal {
            transmit_clocked
        } else {
            self.pcr & PCR_CLKRM != 0
        };
        if receive_clocked && self.spcr & SPCR_RRST != 0 {
            let fsr = if internal {
                fsx
            } else {
                fsg && self.pcr & PCR_FSRM != 0
            };
            // In loopback DR is DX; a DX left in high impedance, and a DR pin with nothing on
            // it, read low.
            let dr = loopback && dx == Some(true);
            self.receive_clock(fsr, dr);
        }
    }

    /// The frame-sync generator: once FRST releases it, FSG rises every FPER + 1 CLKG cycles
    /// and stays high for FWID + 1 of them; only a rise is a frame sync (SPRU580G section 4.4.1
    /// and Table 24).
    fn frame_generator(&mut self) -> bool {
        if self.spcr & SPCR_FRST == 0 {
            return false;
        }
        self.fsg_count = self.fsg_count.saturating_sub(1);
        if self.fsg_count == 0 {
            self.fsg_count = self.frame_period();
            self.fsg_width = (self.srgr >> 8) & 0xff;
            let rose = !self.fsg_high;
            self.fsg_high = true;
            return rose;
        }
        if self.fsg_high {
            if self.fsg_width == 0 {
                self.fsg_high = false;
            } else {
                self.fsg_width -= 1;
            }
        }
        false
    }

    fn frame_period(&self) -> u32 {
        ((self.srgr >> 16) & 0xfff) + 1
    }

    /// One transmit bit clock: the DXR-to-XSR copy if XSR is free, the frame sync, and the bit
    /// this clock drives on DX - `None` while DX is in high impedance. Returns that and whether
    /// FSX_int pulsed.
    fn transmit_clock(&mut self, fsg: bool) -> (Option<bool>, bool) {
        let layout = Layout::new(self.xcr);
        // DXR moves to XSR on the first bit clock XSR is free, which is what makes DXR ready
        // again and, with FSGM clear, pulses the frame sync (sections 3.2.2 and 4.4.3). Only a
        // selected element empties XSR, so with XMCM = 01b or 11b the copies follow the
        // selected elements - preloading before the frame and refilling at the end of each
        // selected element, as Figure 44 draws it - and with none selected there is nothing for
        // a word to go out in, so DXR is never taken (section 8.2).
        let mut copied = false;
        if self.xsr.is_none() && self.dxr_pending && self.any_transmit_channel(&layout) {
            self.load_xsr();
            copied = true;
        }
        let fsx = self.pcr & PCR_FSXM != 0
            && if self.srgr & SRGR_FSGM != 0 {
                fsg
            } else {
                copied
            };
        if fsx {
            if self.transmit_mode() == INTM_FRAME {
                self.events.transmit_interrupt = true;
            }
            if self.transmit.sync(&layout) == Sync::Error {
                self.spcr |= SPCR_XSYNCERR;
            }
        }
        let Some(slot) = self.transmit.advance(&layout) else {
            return (None, fsx);
        };
        if slot.bit == 0 {
            let (enabled, unmasked) = self.transmit_channel(slot.element);
            self.transmit.enabled = enabled;
            self.transmit.unmasked = unmasked;
            self.transmit_block = slot.element / 16 % 8;
            if enabled && self.xsr.is_none() {
                // Underflow: the old word goes out again, and XEMPTY stays low until a new one
                // is copied (section 5.5.4).
                self.xsr = Some(self.last_copied);
            }
        }
        let mut dx = None;
        if self.transmit.enabled {
            let word = self.xsr.unwrap_or(0);
            let index = if reversed(self.xcr, slot.bits) {
                slot.bit
            } else {
                slot.bits - 1 - slot.bit
            };
            if self.transmit.unmasked {
                dx = Some((word >> index) & 1 != 0);
            }
            if slot.last() {
                self.xsr = None;
                if !self.dxr_pending {
                    self.spcr &= !SPCR_XEMPTY;
                }
            }
        }
        if slot.last() && layout.ends_subframe(slot.element) {
            // XCBLK moves on with the boundary, so the interrupt that reports it finds the
            // subframe now current (section 8.2.2).
            self.transmit_block = layout.next_block(slot.element);
            if self.mcr & (3 << 16) != 0 && self.transmit_mode() == INTM_SUBFRAME {
                self.events.transmit_interrupt = true;
            }
        }
        (dx, fsx)
    }

    /// Whether any element of the frame takes a word from DXR: all of them unless XMCM is 01b
    /// or 11b, when only the selected ones do (Table 28).
    fn any_transmit_channel(&self, layout: &Layout) -> bool {
        if !matches!((self.mcr >> 16) & 3, 1 | 3) {
            return true;
        }
        (0..layout.elements()).any(|element| self.transmit_channel(element).0)
    }

    /// DXR to XSR, compressing if XCOMPAND says so: DXR is ready again and XSR holds a word.
    fn load_xsr(&mut self) {
        let word = if compand(self.xcr) >= COMPAND_MULAW {
            u32::from(compress(compand(self.xcr), self.dxr as u16 as i16))
        } else {
            self.dxr
        };
        self.xsr = Some(word);
        self.last_copied = word;
        self.dxr_pending = false;
        self.spcr |= SPCR_XRDY | SPCR_XEMPTY;
        self.transmit_ready();
    }

    /// One receive bit clock: the buffer copies, the frame sync, and the bit sampled from DR.
    fn receive_clock(&mut self, fsr: bool, dr: bool) {
        let layout = Layout::new(self.rcr);
        // A complete element moves RSR to RBR and RBR to DRR on the clock after its last bit,
        // each only if the next register is free; a third element finding RBR still full is
        // the overrun RFULL reports, and is lost. The receiver stops there: it takes nothing
        // more until a frame sync after DRR is read (sections 3, 5.1, 5.5.1 and 5.5.2).
        self.copy_to_drr();
        if let Some(word) = self.rsr_full.take() {
            if self.rbr.is_none() {
                self.rbr = Some(word);
            } else {
                self.spcr |= SPCR_RFULL;
                self.receive.stop();
            }
        }
        self.copy_to_drr();
        if fsr {
            if self.receive_mode() == INTM_FRAME {
                self.events.receive_interrupt = true;
            }
            if self.spcr & SPCR_RFULL == 0 && self.receive.sync(&layout) == Sync::Error {
                self.spcr |= SPCR_RSYNCERR;
            }
        }
        let Some(slot) = self.receive.advance(&layout) else {
            return;
        };
        if slot.bit == 0 {
            self.rsr = 0;
            self.receive.enabled = self.receive_channel(slot.element);
            self.receive_block = slot.element / 16 % 8;
        }
        self.rsr = (self.rsr << 1) | u32::from(dr);
        if !slot.last() {
            return;
        }
        if self.receive.enabled {
            let word = if reversed(self.rcr, slot.bits) {
                reverse(self.rsr, slot.bits)
            } else {
                self.rsr
            };
            self.rsr_full = Some((word, slot.bits));
        }
        if layout.ends_subframe(slot.element) {
            self.receive_block = layout.next_block(slot.element);
            if self.mcr & MCR_RMCM != 0 && self.receive_mode() == INTM_SUBFRAME {
                self.events.receive_interrupt = true;
            }
        }
    }

    /// RBR to DRR, through the expander, once DRR has been read (section 3.2.1).
    fn copy_to_drr(&mut self) {
        if self.spcr & SPCR_RRDY != 0 {
            return;
        }
        let Some((word, bits)) = self.rbr.take() else {
            return;
        };
        self.drr = self.receive_word(word, bits);
        self.spcr |= SPCR_RRDY;
        self.events.receive = true;
        if self.receive_mode() == INTM_READY {
            self.events.receive_interrupt = true;
        }
    }

    /// An element on its way into DRR: expanded to a left-justified 16-bit LAW16 if companding,
    /// then justified as RJUST says (SPRU580G section 4.5.6 and Table 13).
    fn receive_word(&self, word: u32, bits: u32) -> u32 {
        let law = compand(self.rcr);
        if law >= COMPAND_MULAW {
            self.justify(u32::from(expand(law, word as u8) as u16), 16)
        } else {
            self.justify(word, bits)
        }
    }

    /// XRDY rose: the EDMA's XEVT, and XINT in mode 0 (section 3.2.2).
    fn transmit_ready(&mut self) {
        self.events.transmit = true;
        if self.transmit_mode() == INTM_READY {
            self.events.transmit_interrupt = true;
        }
    }

    /// Whether a transmit element is enabled - copied from DXR and shifted - and unmasked -
    /// driven on DX - under XMCM (SPRU580G sections 8.2 and 8.3, Table 28).
    fn transmit_channel(&self, element: u32) -> (bool, bool) {
        let enhanced = self.mcr & MCR_XMCME != 0;
        let selected =
            |blocks: u32| channel_enabled(&self.transmit_enable, enhanced, blocks, element);
        match (self.mcr >> 16) & 3 {
            0 => (true, true),
            1 => {
                let on = selected(self.mcr >> 21);
                (on, on)
            }
            2 => (true, selected(self.mcr >> 21)),
            // Symmetric: the receive selection enables, and XCER unmasks within the receive
            // blocks (Table 34).
            _ => {
                let on = self.receive_channel_selected(element);
                (on, on && selected(self.mcr >> 5))
            }
        }
    }

    fn receive_channel(&self, element: u32) -> bool {
        self.mcr & MCR_RMCM == 0 || self.receive_channel_selected(element)
    }

    fn receive_channel_selected(&self, element: u32) -> bool {
        channel_enabled(
            &self.receive_enable,
            self.mcr & MCR_RMCME != 0,
            self.mcr >> 5,
            element,
        )
    }

    fn write_dxr(&mut self, value: u32) {
        self.dxr = value;
        if self.spcr & SPCR_XRST != 0 {
            // The write consumes the transmitter's readiness until the word reaches XSR.
            self.spcr &= !SPCR_XRDY;
            self.dxr_pending = true;
        } else if self.spcr & SPCR_RRST == 0 {
            self.internal_compand = self.internal_compand_word(value);
        }
    }

    /// With both sections in reset the companding hardware connects DXR to DRR: XCOMPAND
    /// compresses, RCOMPAND expands, and DRR has the result four CPU clocks later without
    /// RRDY or XRDY moving (SPRU580G section 6.1). [INFERENCE] The path is taken only when one
    /// side compands, which is what the section describes it for; a side that does not passes
    /// the 8-bit code through.
    fn internal_compand_word(&self, value: u32) -> Option<(u8, u32)> {
        let transmit = compand(self.xcr);
        let receive = compand(self.rcr);
        if transmit < COMPAND_MULAW && receive < COMPAND_MULAW {
            return None;
        }
        let code = if transmit >= COMPAND_MULAW {
            u32::from(compress(transmit, value as u16 as i16))
        } else {
            value & 0xff
        };
        Some((INTERNAL_COMPAND_CYCLES, self.receive_word(code, 8)))
    }

    fn read_drr(&mut self) -> u32 {
        // Reading the receive register clears its ready flag and the overrun behind it.
        self.spcr &= !(SPCR_RRDY | SPCR_RFULL);
        self.drr
    }

    /// PCR as written, except where a section in reset has its pins as general-purpose inputs:
    /// those polarity bits read the pin, and every status bit of an input reads low because
    /// nothing drives it (SPRU580G section 10, Table 17).
    fn read_pcr(&self) -> u32 {
        let mut value = self.pcr;
        if self.spcr & SPCR_XRST == 0 && self.pcr & PCR_XIOEN != 0 {
            if self.pcr & PCR_CLKXM == 0 {
                value &= !PCR_CLKXP;
            }
            if self.pcr & PCR_FSXM == 0 {
                value &= !PCR_FSXP;
            }
        }
        if self.spcr & SPCR_RRST == 0 && self.pcr & PCR_RIOEN != 0 {
            if self.pcr & PCR_CLKRM == 0 {
                value &= !PCR_CLKRP;
            }
            if self.pcr & PCR_FSRM == 0 {
                value &= !PCR_FSRP;
            }
        }
        value
    }

    fn write_spcr(&mut self, value: u32) {
        let previous = self.spcr;
        self.spcr = (previous & !SPCR_WRITABLE) | (value & SPCR_WRITABLE);
        if self.spcr & SPCR_XRST == 0 {
            // A transmitter in reset is neither ready nor holding a word, and its error and
            // framing are cleared (SPRU580G section 3.1).
            self.spcr &= !(SPCR_XRDY | SPCR_XEMPTY | SPCR_XSYNCERR);
            self.dxr_pending = false;
            self.xsr = None;
            self.last_copied = 0;
            self.transmit = Framing::default();
            self.transmit_block = 0;
        } else if previous & SPCR_XRST == 0 {
            // Leaving reset makes DXR ready with XSR empty (section 3.2.2 and 5.5.4).
            self.spcr = (self.spcr | SPCR_XRDY) & !SPCR_XEMPTY;
            self.transmit_ready();
        }
        if self.spcr & SPCR_RRST == 0 {
            self.spcr &= !(SPCR_RRDY | SPCR_RFULL | SPCR_RSYNCERR);
            self.rsr = 0;
            self.rsr_full = None;
            self.rbr = None;
            self.receive = Framing::default();
            self.receive_block = 0;
        }
        let generator = SPCR_GRST | SPCR_FRST;
        if self.spcr & SPCR_GRST == 0 {
            self.divide = 0;
        }
        if self.spcr & generator != generator {
            self.fsg_high = false;
        } else if previous & generator != generator {
            // FSG first rises FPER + 1 CLKG cycles after the frame logic is released (Table 24).
            self.fsg_count = self.frame_period();
        }
    }

    /// RJUST decides where a received word shorter than 32 bits sits in DRR and what fills the
    /// rest (SPRU580G Tables 11 and 12).
    fn justify(&self, word: u32, bits: u32) -> u32 {
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

    fn offset(&self, address: u32) -> Option<u32> {
        if !(self.base..self.base + CONFIG_WINDOW).contains(&address) {
            return None;
        }
        Some(address - self.base)
    }
}

fn compand(control: u32) -> u32 {
    (control >> 19) & 3
}

/// COMPAND = 01b sends 8-bit elements LSB first, and 32-bit ones too with WDREVRS set; other
/// lengths are sent MSB first (SPRU580G sections 4.5.7 and 6.2).
fn reversed(control: u32, bits: u32) -> bool {
    compand(control) == COMPAND_LSB_FIRST
        && (bits == 8 || (bits == 32 && control & CONTROL_WDREVRS != 0))
}

fn reverse(word: u32, bits: u32) -> u32 {
    word.reverse_bits() >> (32 - bits)
}

/// One element's bit in the channel-enable registers. The enhanced mode gives every one of the
/// 128 channels its own bit; the normal mode enables 16 of partition A's even subframes through
/// bits 15:0 and 16 of partition B's odd ones through bits 31:16, the PABLK and PBBLK fields of
/// `blocks` (MCR bits 3:0 of the shifted value) choosing which (SPRU580G Tables 32 and 34).
fn channel_enabled(enable: &[u32; 4], enhanced: bool, blocks: u32, element: u32) -> bool {
    if element >= 128 {
        return false;
    }
    if enhanced {
        return enable[(element / 32) as usize] >> (element % 32) & 1 != 0;
    }
    let subframe = element / 16;
    let (block, shift) = if subframe % 2 == 0 {
        (blocks & 3, 0)
    } else {
        ((blocks >> 2) & 3, 16)
    };
    subframe / 2 == block && enable[0] >> (shift + element % 16) & 1 != 0
}

/// The word length encoding both control registers share: 8, 12, 16, 20, 24 or 32 bits, with the
/// two reserved encodings behaving as the longest (SPRU580G Table 10).
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

/// G.711 compression of a left-justified LAW16 sample: mu-law keeps its 14 most significant
/// bits and A-law its 13, saturating outside that range (SPRU580G section 6, Figure 39).
fn compress(law: u32, sample: i16) -> u8 {
    if law == COMPAND_MULAW {
        let mut magnitude = i32::from(sample) >> 2;
        let mask = if magnitude < 0 {
            magnitude = -magnitude;
            0x7f
        } else {
            0xff
        };
        let biased = magnitude.min(8159) + 33;
        let segment = (0..8).find(|&s| biased <= (0x3f << s) | ((1 << s) - 1));
        match segment {
            Some(s) => (((s << 4) | ((biased >> (s + 1)) & 0xf)) as u8) ^ mask,
            None => 0x7f ^ mask,
        }
    } else {
        let value = i32::from(sample) >> 3;
        let (magnitude, mask) = if value >= 0 {
            (value, 0xd5)
        } else {
            (-value - 1, 0x55)
        };
        let segment = (0..8).find(|&s| magnitude <= (0x1f << s) | ((1 << s) - 1));
        match segment {
            Some(0) => (((magnitude >> 1) & 0xf) as u8) ^ mask,
            Some(s) => (((s << 4) | ((magnitude >> s) & 0xf)) as u8) ^ mask,
            None => 0x7f ^ mask,
        }
    }
}

/// G.711 expansion of an 8-bit code to a left-justified LAW16 sample.
fn expand(law: u32, code: u8) -> i16 {
    if law == COMPAND_MULAW {
        let code = !code;
        let magnitude = ((i32::from(code & 0xf) << 3) + 0x84) << ((code >> 4) & 7);
        (if code & 0x80 != 0 {
            0x84 - magnitude
        } else {
            magnitude - 0x84
        }) as i16
    } else {
        let code = code ^ 0x55;
        let segment = (code >> 4) & 7;
        let mut magnitude = i32::from(code & 0xf) << 4;
        magnitude = match segment {
            0 => magnitude + 8,
            1 => magnitude + 0x108,
            _ => (magnitude + 0x108) << (segment - 1),
        };
        (if code & 0x80 != 0 {
            magnitude
        } else {
            -magnitude
        }) as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::port::SlavePort;

    const BASE: u32 = 0x028c_0000;

    fn port() -> Mcbsp {
        Mcbsp::new(BASE, 0x3000_0000, 0x3000_0010)
    }

    fn write(port: &mut Mcbsp, offset: u32, value: u32) {
        port.write(BASE + offset, value, TEST_CONTEXT);
    }

    fn read(port: &mut Mcbsp, offset: u32) -> u32 {
        port.read(BASE + offset, TEST_CONTEXT).unwrap()
    }

    /// Run whole CLKG cycles of a CLKGDV = 0 port: one SYSCLK3 edge, six CPU cycles each.
    fn clkg(port: &mut Mcbsp, cycles: u32) -> McbspEvents {
        let mut all = McbspEvents::default();
        for _ in 0..cycles {
            for tick in 1..=6 {
                let events = port.tick(tick == 6);
                all.transmit |= events.transmit;
                all.receive |= events.receive;
                all.transmit_interrupt |= events.transmit_interrupt;
                all.receive_interrupt |= events.receive_interrupt;
            }
        }
        all
    }

    /// Service the port the way a polling CPU does for `cycles` CLKG cycles: the next of
    /// `words` into DXR whenever XRDY is up, and DRR read whenever a word arrives.
    fn poll(port: &mut Mcbsp, words: impl IntoIterator<Item = u32>, cycles: u32) -> Vec<u32> {
        let mut words = words.into_iter();
        let mut received = Vec::new();
        for _ in 0..cycles {
            if read(port, SPCR) & SPCR_XRDY != 0 {
                if let Some(word) = words.next() {
                    write(port, DXR, word);
                }
            }
            if clkg(port, 1).receive {
                received.push(read(port, DRR));
            }
        }
        received
    }

    const LOOPBACK: u32 = SPCR_DLB | SPCR_XRST | SPCR_RRST | SPCR_GRST;
    const FRAMED: u32 = LOOPBACK | SPCR_FRST;
    const MASTER: u32 = PCR_CLKXM | PCR_FSXM;

    /// A digital-loopback port at CLKGDV = 0, framed by FSG every `period` CLKG cycles: the
    /// first frame sync is on CLKG cycle `period`.
    fn framed_loopback(xcr: u32, rcr: u32, period: u32) -> Mcbsp {
        let mut port = port();
        write(&mut port, RCR, rcr);
        write(&mut port, XCR, xcr);
        write(
            &mut port,
            SRGR,
            SRGR_CLKSM | SRGR_FSGM | ((period - 1) << 16),
        );
        write(&mut port, PCR, MASTER);
        write(&mut port, SPCR, FRAMED);
        port
    }

    #[test]
    fn a_port_comes_up_with_the_documented_resets() {
        let mut port = port();
        // Every SPCR field resets to zero, SRGR to the internal clock and a divider of one
        // (SPRU580G Figures 54 and 57).
        assert_eq!(read(&mut port, SPCR), 0);
        assert_eq!(read(&mut port, SRGR), SRGR_CLKSM | 1);
        assert_eq!(read(&mut port, RCR), 0);
        assert_eq!(read(&mut port, PCR), 0);
    }

    #[test]
    fn a_loopback_word_takes_one_serial_clock_per_bit_at_cpu_over_six() {
        let mut port = port();
        // 16-bit words, FSX from the DXR-to-XSR copy, no data delay, CLKGDV = 1: a bit clock
        // is two SYSCLK3 edges, twelve CPU cycles.
        write(&mut port, RCR, 2 << 5);
        write(&mut port, XCR, 2 << 5);
        write(&mut port, SRGR, SRGR_CLKSM | 1);
        write(&mut port, PCR, MASTER);
        write(&mut port, SPCR, LOOPBACK);
        // Leaving reset makes DXR ready with XSR empty.
        assert_eq!(read(&mut port, SPCR) & (SPCR_XRDY | SPCR_XEMPTY), SPCR_XRDY);
        write(&mut port, DXR, 0xbeef);
        assert_eq!(read(&mut port, SPCR) & SPCR_XRDY, 0);
        let mut transmit = Vec::new();
        let mut receive = Vec::new();
        for cycle in 1..=300 {
            let events = port.tick(cycle % 6 == 0);
            if events.transmit {
                transmit.push(cycle);
            }
            if events.receive {
                receive.push(cycle);
            }
        }
        // XEVT for leaving reset, then CLKG 1 (CPU cycle 12) copies DXR to XSR and syncs the
        // frame; the bits go out on CLKG 1 to 16, and CLKG 17 (cycle 204) moves the word into
        // DRR.
        assert_eq!(transmit, [1, 12]);
        assert_eq!(receive, [204]);
        assert_eq!(read(&mut port, DRR), 0xbeef);
        // XSR has shifted out and nothing new was written: XEMPTY is low again.
        assert_eq!(read(&mut port, SPCR) & (SPCR_RRDY | SPCR_XEMPTY), 0);
    }

    #[test]
    fn the_receive_buffers_hold_two_words_and_lose_the_third() {
        // 8-bit elements every 8 CLKG cycles: maximum frame frequency with no delay. The words
        // shift on CLKG 8-15, 16-23 and 24-31 and leave RSR the clock after.
        let mut port = framed_loopback(0, 0, 8);
        for word in [0x11, 0x22, 0x33] {
            write(&mut port, DXR, word);
            clkg(&mut port, 8);
        }
        clkg(&mut port, 8);
        // DRR has the first word, RBR the second, and the third found RBR full.
        assert_eq!(
            read(&mut port, SPCR) & (SPCR_RRDY | SPCR_RFULL),
            SPCR_RRDY | SPCR_RFULL
        );
        assert_eq!(read(&mut port, DRR), 0x11);
        assert_eq!(read(&mut port, SPCR) & SPCR_RFULL, 0);
        // Reading DRR lets RBR through on the next bit clock.
        clkg(&mut port, 1);
        assert_eq!(read(&mut port, DRR), 0x22);
        // The third word is gone: nothing more arrives until the frame now shifting ends.
        clkg(&mut port, 6);
        assert_eq!(read(&mut port, SPCR) & SPCR_RRDY, 0);
    }

    #[test]
    fn an_underflowing_transmitter_sends_its_last_word_again() {
        let mut port = framed_loopback(0, 0, 8);
        write(&mut port, DXR, 0x5a);
        clkg(&mut port, 16);
        assert_eq!(read(&mut port, SPCR) & (SPCR_XEMPTY | SPCR_RRDY), SPCR_RRDY);
        assert_eq!(read(&mut port, DRR), 0x5a);
        // No new word: the next frame repeats 0x5A and XEMPTY stays low.
        clkg(&mut port, 8);
        assert_eq!(read(&mut port, SPCR) & (SPCR_XEMPTY | SPCR_RRDY), SPCR_RRDY);
        assert_eq!(read(&mut port, DRR), 0x5a);
    }

    #[test]
    fn a_dual_phase_frame_carries_each_phase_at_its_own_length() {
        // Phase 1: one 16-bit element; phase 2: two 8-bit elements; one-bit data delay. The
        // sync on CLKG 33 puts the three elements on CLKG 34-49, 50-57 and 58-65.
        let control = CONTROL_PHASE | (1 << 24) | (2 << 5) | (1 << 16);
        let mut port = framed_loopback(control, control, 33);
        assert_eq!(
            poll(&mut port, [0xabcd, 0x12, 0x34], 66),
            [0xabcd, 0x12, 0x34]
        );
    }

    #[test]
    fn a_receive_delay_longer_than_the_transmit_delay_drops_the_first_bit() {
        // DX starts with no delay, the receiver samples one bit later: it sees the word shifted
        // left by one, with DX already in high impedance for its last bit.
        let mut port = framed_loopback(0, 1 << 16, 16);
        write(&mut port, DXR, 0xc3);
        clkg(&mut port, 25);
        assert_eq!(read(&mut port, DRR), 0x86);
    }

    #[test]
    fn frame_syncs_follow_fper_and_raise_xint_in_frame_mode() {
        let mut port = port();
        // FSG every 10 CLKG cycles, transmitter out of reset, XINTM = frame sync.
        write(
            &mut port,
            SRGR,
            SRGR_CLKSM | SRGR_FSGM | (9 << 16) | (2 << 8),
        );
        write(&mut port, PCR, MASTER);
        write(
            &mut port,
            SPCR,
            SPCR_XRST | SPCR_GRST | SPCR_FRST | (INTM_FRAME << 20),
        );
        let mut syncs = Vec::new();
        for cycle in 1..=35 {
            if clkg(&mut port, 1).transmit_interrupt {
                syncs.push(cycle);
            }
        }
        assert_eq!(syncs, [10, 20, 30]);
    }

    #[test]
    fn a_frame_sync_held_high_is_one_frame_sync() {
        let mut port = port();
        // FWID longer than FPER: FSG never falls, so only its first rise is a frame sync.
        write(
            &mut port,
            SRGR,
            SRGR_CLKSM | SRGR_FSGM | (3 << 16) | (9 << 8),
        );
        write(&mut port, PCR, MASTER);
        write(
            &mut port,
            SPCR,
            SPCR_XRST | SPCR_GRST | SPCR_FRST | (INTM_FRAME << 20),
        );
        let mut syncs = 0;
        for _ in 0..40 {
            syncs += u32::from(clkg(&mut port, 1).transmit_interrupt);
        }
        assert_eq!(syncs, 1);
    }

    #[test]
    fn frame_sync_interrupts_need_the_section_out_of_reset() {
        let mut port = port();
        // C6455 usage note 2.1.8: the frame-sync interrupt only works out of reset.
        write(&mut port, SRGR, SRGR_CLKSM | SRGR_FSGM | (3 << 16));
        write(&mut port, PCR, MASTER | PCR_FSRM | PCR_CLKRM);
        write(
            &mut port,
            SPCR,
            SPCR_GRST | SPCR_FRST | (INTM_FRAME << 20) | (INTM_FRAME << 4),
        );
        let events = clkg(&mut port, 20);
        assert!(!events.transmit_interrupt && !events.receive_interrupt);
    }

    #[test]
    fn an_early_frame_sync_is_an_error_unless_ignored() {
        // 16-bit elements framed every 8 CLKG cycles: the sync on CLKG 16 lands mid-element.
        let mut port = framed_loopback(2 << 5, 2 << 5, 8);
        write(&mut port, SPCR, FRAMED | (INTM_SYNCERR << 20));
        write(&mut port, DXR, 0x1234);
        clkg(&mut port, 15);
        assert_eq!(read(&mut port, SPCR) & SPCR_XSYNCERR, 0);
        assert!(clkg(&mut port, 1).transmit_interrupt);
        assert_ne!(read(&mut port, SPCR) & SPCR_XSYNCERR, 0);
        assert_ne!(read(&mut port, SPCR) & SPCR_RSYNCERR, 0);
        // Each sync restarts the element, so it never completes.
        clkg(&mut port, 30);
        assert_eq!(read(&mut port, SPCR) & SPCR_RRDY, 0);
        // Clearing the error is software's; the next early sync sets it again.
        write(&mut port, SPCR, FRAMED);
        assert_eq!(read(&mut port, SPCR) & SPCR_XSYNCERR, 0);
        clkg(&mut port, 8);
        assert_ne!(read(&mut port, SPCR) & SPCR_XSYNCERR, 0);

        // With FIG set the sync on CLKG 16 is ignored and the element ends on CLKG 23.
        let control = (2 << 5) | CONTROL_FIG;
        let mut ignoring = framed_loopback(control, control, 8);
        write(&mut ignoring, DXR, 0x1234);
        clkg(&mut ignoring, 24);
        assert_eq!(
            read(&mut ignoring, SPCR) & (SPCR_XSYNCERR | SPCR_RSYNCERR),
            0
        );
        assert_eq!(read(&mut ignoring, DRR), 0x1234);
    }

    #[test]
    fn g711_matches_the_reference_codes() {
        // Silence and both extremes in each law (ITU-T G.711).
        assert_eq!(compress(COMPAND_MULAW, 0), 0xff);
        assert_eq!(compress(COMPAND_MULAW, i16::MAX), 0x80);
        assert_eq!(compress(COMPAND_MULAW, i16::MIN), 0x00);
        assert_eq!(expand(COMPAND_MULAW, 0x80), 32124);
        assert_eq!(expand(COMPAND_MULAW, 0x00), -32124);
        assert_eq!(expand(COMPAND_MULAW, 0xff), 0);
        assert_eq!(compress(COMPAND_ALAW, 0), 0xd5);
        assert_eq!(compress(COMPAND_ALAW, i16::MAX), 0xaa);
        assert_eq!(compress(COMPAND_ALAW, i16::MIN), 0x2a);
        assert_eq!(expand(COMPAND_ALAW, 0xaa), 32256);
        assert_eq!(expand(COMPAND_ALAW, 0xd5), 8);
        // Every code survives expansion and recompression; mu-law's two zeros, 0x7F and
        // 0xFF, both come back as 0xFF.
        for law in [COMPAND_MULAW, COMPAND_ALAW] {
            for code in 0..=255u8 {
                let expected = if law == COMPAND_MULAW && code == 0x7f {
                    0xff
                } else {
                    code
                };
                assert_eq!(
                    compress(law, expand(law, code)),
                    expected,
                    "{law} {code:#x}"
                );
            }
        }
    }

    #[test]
    fn loopback_companding_quantises_and_justifies() {
        // mu-law both ways, sign-extended: -1000 compresses to 0x4E, whose interval centre
        // is -988.
        let control = COMPAND_MULAW << 19;
        let mut port = framed_loopback(control, control, 8);
        write(&mut port, SPCR, FRAMED | (RJUST_RIGHT_SIGN << 13));
        write(&mut port, DXR, (-1000i32) as u32);
        clkg(&mut port, 16);
        assert_eq!(compress(COMPAND_MULAW, -1000), 0x4e);
        assert_eq!(read(&mut port, DRR) as i32, -988);
    }

    #[test]
    fn the_companding_hardware_converts_with_both_sections_in_reset() {
        let mut port = port();
        // Linear to A-law: transmit compands, receive passes the code through, four CPU
        // cycles after the write.
        write(&mut port, XCR, COMPAND_ALAW << 19);
        write(&mut port, DXR, 0x7fff);
        for _ in 0..3 {
            port.tick(false);
        }
        assert_eq!(read(&mut port, DRR), 0);
        port.tick(false);
        assert_eq!(read(&mut port, DRR), 0xaa);
        assert_eq!(read(&mut port, SPCR) & (SPCR_RRDY | SPCR_XRDY), 0);
        // A-law to linear: the code goes straight to the expander, left-justified by RJUST.
        write(&mut port, XCR, 0);
        write(&mut port, RCR, COMPAND_ALAW << 19);
        write(&mut port, SPCR, RJUST_LEFT_ZERO << 13);
        write(&mut port, DXR, 0xaa);
        for _ in 0..4 {
            port.tick(false);
        }
        assert_eq!(read(&mut port, DRR), 32256 << 16);
    }

    #[test]
    fn lsb_first_reverses_the_bit_order_on_the_wire() {
        // Only the transmitter sends LSB first, so the receiver sees 0x01 reversed.
        let mut port = framed_loopback(COMPAND_LSB_FIRST << 19, 0, 8);
        write(&mut port, DXR, 0x01);
        clkg(&mut port, 16);
        assert_eq!(read(&mut port, DRR), 0x80);
        // 32-bit elements reverse only with WDREVRS as well.
        let reversing = (COMPAND_LSB_FIRST << 19) | (5 << 5) | CONTROL_WDREVRS;
        let mut port = framed_loopback(reversing, 5 << 5, 32);
        write(&mut port, DXR, 1);
        clkg(&mut port, 64);
        assert_eq!(read(&mut port, DRR), 0x8000_0000);
        let mut port = framed_loopback((COMPAND_LSB_FIRST << 19) | (5 << 5), 5 << 5, 32);
        write(&mut port, DXR, 1);
        clkg(&mut port, 64);
        assert_eq!(read(&mut port, DRR), 1);
    }

    #[test]
    fn transmit_multichannel_modes_enable_and_mask_elements() {
        // Four 8-bit elements per frame, shifting on CLKG 32-63; channels 1 and 3 selected in
        // XCERE0 with XPABLK = 0 (SPRU580G Figures 43 to 45). A masked or disabled element
        // leaves DX undriven, which the loopback receiver reads as zeros.
        let layout = 3 << 8;
        let frame = |mode: u32| {
            let mut port = framed_loopback(layout, layout, 32);
            write(&mut port, MCR, mode << 16);
            write(&mut port, XCERE0, 0b1010);
            poll(&mut port, 1.., 64)
        };
        // XMCM = 0: every element carries its word.
        assert_eq!(frame(0), [1, 2, 3, 4]);
        // XMCM = 1: only channels 1 and 3 take words from DXR and drive DX.
        assert_eq!(frame(1), [0, 1, 0, 2]);
        // XMCM = 2: all four take words, but only 1 and 3 drive DX.
        assert_eq!(frame(2), [0, 2, 0, 4]);
        // XMCM = 3: the receive selection (channels 1 and 3) enables, XCERE0 (channel 3 only)
        // unmasks (Figure 46).
        let mut port = framed_loopback(layout, layout, 32);
        write(&mut port, MCR, 3 << 16);
        write(&mut port, RCERE0, 0b1010);
        write(&mut port, XCERE0, 0b1000);
        assert_eq!(poll(&mut port, 1.., 64), [0, 0, 0, 2]);
    }

    #[test]
    fn receive_multichannel_selection_follows_the_partition_blocks() {
        // Sixty-four 8-bit elements on CLKG 512-1023, element k carrying word k + 1. Partition
        // A's block 0 is channels 0-15 and partition B's block 0 channels 16-31; RCERE0
        // enables the third of A's and the second of B's.
        let layout = 63 << 8;
        let enables = (1 << 2) | (1 << 17);
        let mut port = framed_loopback(layout, layout, 512);
        write(&mut port, MCR, MCR_RMCM);
        write(&mut port, RCERE0, enables);
        assert_eq!(poll(&mut port, 1.., 1016), [3, 18]);
        // RCBLK follows the subframe in progress: element 63, from CLKG 1016, is in subframe 3.
        assert_eq!((read(&mut port, MCR) >> 2) & 7, 3);
        // RPBBLK = 1 moves partition B to channels 48-63, so the same bit picks channel 49.
        let mut port = framed_loopback(layout, layout, 512);
        write(&mut port, MCR, MCR_RMCM | (1 << 7));
        write(&mut port, RCERE0, enables);
        assert_eq!(poll(&mut port, 1.., 1024), [3, 50]);
    }

    #[test]
    fn enhanced_mode_selects_any_of_128_channels() {
        let layout = 127 << 8;
        let mut port = framed_loopback(layout, layout, 1024);
        write(&mut port, MCR, MCR_RMCM | MCR_RMCME);
        write(&mut port, RCERE3, 1 << 31);
        assert_eq!(poll(&mut port, 1.., 2049), [128]);
    }

    #[test]
    fn end_of_subframe_interrupts_report_the_subframe_just_entered() {
        let layout = 31 << 8;
        let intm = (INTM_SUBFRAME << 4) | (INTM_SUBFRAME << 20);
        let mut port = framed_loopback(layout, layout, 256);
        write(&mut port, SPCR, FRAMED | intm);
        // Drain the XINT for leaving reset, raised while XINTM was still 0.
        port.tick(false);
        let mut interrupts = 0;
        for _ in 0..512 {
            let events = clkg(&mut port, 1);
            interrupts += u32::from(events.receive_interrupt || events.transmit_interrupt);
        }
        // Outside multichannel mode the subframe boundaries interrupt nothing.
        assert_eq!(interrupts, 0);

        // Both sections in multichannel mode with no channel selected, so nothing is received
        // and nothing can overrun.
        let mut port = framed_loopback(layout, layout, 256);
        write(&mut port, MCR, MCR_RMCM | (1 << 16));
        write(&mut port, SPCR, FRAMED | intm);
        port.tick(false);
        let mut blocks = Vec::new();
        for _ in 0..512 {
            let events = clkg(&mut port, 1);
            assert_eq!(events.receive_interrupt, events.transmit_interrupt);
            if events.receive_interrupt {
                let mcr = read(&mut port, MCR);
                blocks.push(((mcr >> 2) & 7, (mcr >> 18) & 7));
            }
        }
        // Element 15 of the frame on CLKG 256-511 ends subframe 0, so RCBLK and XCBLK already
        // name subframe 1; element 31 ends the frame, and the next starts in subframe 0.
        assert_eq!(blocks, [(1, 1), (0, 0)]);
    }

    #[test]
    fn rewriting_the_element_length_mid_element_finishes_the_element_as_started() {
        // A 32-bit element shifts on CLKG 64-95; ten bits in, XCR asks for 8-bit elements.
        // The element in flight keeps its length and arrives whole.
        let mut port = framed_loopback(5 << 5, 5 << 5, 64);
        write(&mut port, DXR, 0x1234_5678);
        clkg(&mut port, 74);
        write(&mut port, XCR, 0);
        clkg(&mut port, 22);
        assert_eq!(read(&mut port, SPCR) & SPCR_RRDY, SPCR_RRDY);
        assert_eq!(read(&mut port, DRR), 0x1234_5678);
    }

    #[test]
    fn a_transmitter_with_no_channel_selected_takes_nothing_from_dxr() {
        // XMCM = 01b with every XCERE bit clear: no element is selected, so DXR is never copied
        // and neither XRDY nor XEVT comes back (section 8.2) - whether FSG frames the
        // transmitter or the copy would have.
        for srgr in [SRGR_CLKSM | SRGR_FSGM | (31 << 16), SRGR_CLKSM] {
            let mut port = framed_loopback(3 << 8, 3 << 8, 32);
            write(&mut port, SRGR, srgr);
            write(&mut port, MCR, 1 << 16);
            assert!(clkg(&mut port, 1).transmit);
            write(&mut port, DXR, 0x5a);
            assert!(!clkg(&mut port, 64).transmit);
            assert_eq!(read(&mut port, SPCR) & SPCR_XRDY, 0);
        }
    }

    #[test]
    fn a_selected_channel_lets_dxr_preload_before_frst() {
        // SPRU580G section 7.1 steps 9 and 10: DXR is serviced, XEMPTY = 1 confirms XSR holds
        // the word, and only then is the frame-sync generator released - in multichannel mode
        // too, with channel 0 selected through XCERE0 (XMCM = 01b) or RCERE0 (XMCM = 11b).
        for mode in [1, 3] {
            let mut port = port();
            write(&mut port, RCR, 3 << 8);
            write(&mut port, XCR, 3 << 8);
            write(&mut port, SRGR, SRGR_CLKSM | SRGR_FSGM | (31 << 16));
            write(&mut port, PCR, MASTER);
            write(&mut port, MCR, mode << 16);
            write(&mut port, XCERE0, 1);
            write(&mut port, RCERE0, 1);
            write(&mut port, SPCR, LOOPBACK);
            port.tick(false);
            write(&mut port, DXR, 0x5a);
            assert!(clkg(&mut port, 1).transmit);
            assert_eq!(
                read(&mut port, SPCR) & (SPCR_XEMPTY | SPCR_XRDY),
                SPCR_XEMPTY | SPCR_XRDY
            );
            // FSG first rises 32 CLKG cycles after FRST; element 0 reaches DRR 8 later.
            write(&mut port, SPCR, FRAMED);
            clkg(&mut port, 40);
            assert_eq!(read(&mut port, DRR), 0x5a, "XMCM = {mode}");
        }
    }

    #[test]
    fn a_selected_element_refills_xsr_when_it_ends_and_the_copy_frames() {
        // Four 8-bit transmit elements with channels 0 and 2 selected, FSX from each copy,
        // XFIG set. Figure 44: the copy for a later selected element happens as soon as the
        // earlier one has emptied XSR, not when the later element starts. Element 0 shifts on
        // CLKG 1-8, so the second word is copied on CLKG 9 - and, as every copy does with FSGM
        // clear, pulses FSX, which the transmitter mid-frame ignores (Table 7).
        let mut port = port();
        write(&mut port, XCR, (3 << 8) | CONTROL_FIG);
        write(&mut port, SRGR, SRGR_CLKSM);
        write(&mut port, PCR, MASTER);
        write(&mut port, MCR, 1 << 16);
        write(&mut port, XCERE0, 0b0101);
        write(&mut port, SPCR, LOOPBACK | (INTM_FRAME << 20));
        port.tick(false);
        write(&mut port, DXR, 0x11);
        let mut copies = Vec::new();
        let mut syncs = Vec::new();
        for cycle in 1..=32 {
            let events = clkg(&mut port, 1);
            if events.transmit {
                copies.push(cycle);
            }
            if events.transmit_interrupt {
                syncs.push(cycle);
            }
            if cycle == 1 {
                write(&mut port, DXR, 0x22);
            }
        }
        assert_eq!(copies, [1, 9]);
        assert_eq!(syncs, [1, 9]);
        assert_eq!(read(&mut port, SPCR) & SPCR_XSYNCERR, 0);
    }

    #[test]
    fn an_overrun_receiver_waits_for_the_next_frame_sync() {
        // Four 8-bit elements per frame on CLKG 64-95, all 0x5A from one write and underflow.
        // The third element finds DRR and RBR full on CLKG 88: RFULL, and the frame is dropped.
        let mut port = framed_loopback(3 << 8, 3 << 8, 64);
        write(&mut port, DXR, 0x5a);
        clkg(&mut port, 88);
        assert_eq!(read(&mut port, SPCR) & SPCR_RFULL, SPCR_RFULL);
        assert_eq!(read(&mut port, DRR), 0x5a);
        clkg(&mut port, 1);
        assert_eq!(read(&mut port, DRR), 0x5a);
        // The fourth element (CLKG 88-95) is not taken: nothing until the sync on CLKG 128
        // restarts reception, whose first element arrives on CLKG 136.
        assert!(!clkg(&mut port, 46).receive);
        assert!(clkg(&mut port, 1).receive);
        assert_eq!(read(&mut port, SPCR) & SPCR_RSYNCERR, 0);
    }

    #[test]
    fn general_purpose_inputs_read_the_undriven_pins() {
        let mut port = port();
        write(&mut port, PCR, PCR_XIOEN | PCR_FSXM | 0xf | (1 << 5));
        // FSX is an output and keeps FSXP; CLKX is an input and reads low. RIOEN is clear,
        // so the receive polarity bits are ordinary.
        assert_eq!(
            read(&mut port, PCR),
            PCR_XIOEN | PCR_FSXM | PCR_FSXP | PCR_FSRP | PCR_CLKRP | (1 << 5)
        );
        // Out of reset the pins are the serial port's again.
        write(&mut port, SPCR, SPCR_XRST);
        assert_eq!(read(&mut port, PCR) & PCR_CLKXP, PCR_CLKXP);
    }

    #[test]
    fn an_spi_master_receives_the_undriven_dr_while_it_transmits() {
        let mut port = port();
        // Clock-stop mode, FSX from each copy, one-bit delays as SPI master requires: the
        // copy on CLKG 1 frames bits on CLKG 2-9, and DRR fills on CLKG 10.
        write(&mut port, RCR, 1 << 16);
        write(&mut port, XCR, 1 << 16);
        write(&mut port, SRGR, SRGR_CLKSM);
        write(&mut port, PCR, MASTER);
        write(
            &mut port,
            SPCR,
            (2 << 11) | SPCR_XRST | SPCR_RRST | SPCR_GRST,
        );
        write(&mut port, DXR, 0xff);
        assert!(!clkg(&mut port, 9).receive);
        assert!(clkg(&mut port, 1).receive);
        assert_eq!(read(&mut port, DRR), 0);
    }
}
